/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <agm_misc_functions.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	Period = 20;
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	timer.start(10);
	return true;
}


#define AGMINNER_UPDATE_EDGE 0

void SpecificWorker::compute()
{
	QMutexLocker l(mutex);
	if (worldModel->size() == 0)
	{
		printf("Waiting for AGM*\n");
		return;
	}

	static bool first = true;
	static std::map<std::string, QTime> backTimes;
	static RoboCompJointMotor::MotorStateMap backMotors;

	if (first)
	{
		try
		{
			jointmotor_proxy->getAllMotorState(backMotors);
			for (auto j : backMotors)
			{
				backTimes[j.first] = QTime::currentTime().addSecs(-1000000);
			}
			first = false;
		}
		catch (const Ice::Exception &ex)
		{
			std::cout << __FILE__ << ":" << __LINE__ << " --> Can't update InnerModel" << std::endl;
		}
	}

	RoboCompJointMotor::MotorStateMap mMap;
	std::vector<AGMModelEdge> edge_sequence;
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for (auto j : mMap)
		{
			printf("Updating: %s (%f)\n", j.first.c_str(), mMap[j.first].pos);
 			if (backTimes[j.first].elapsed()>500 or abs(backMotors[j.first].pos-mMap[j.first].pos) > 0.25*M_PIl/180.) /* send if it changed more than half degree */
			{
				backTimes[j.first] = QTime::currentTime();
				backMotors[j.first] = j.second;
// 				printf("Updating: %s (%d)\n", j.first.c_str(), newModel->size());
				bool found = false;
				for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
				{
					const AGMModelSymbol::SPtr &symbol = *symbol_it;
					std::string imName;
// 					printf("      symbol id: %d\n", symbol->identifier);
					try
					{
// 						printf("<< %d >>\n", symbol->identifier);
						try { imName = symbol->getAttribute("imName"); } catch(...) { }
						if (imName == j.first)
						{
//							printf("found!\n");
							found = true;
							auto parent = newModel->getParentByLink(symbol->identifier, "RT");
// printf("%d (%d -> %d)\n", __LINE__, parent->identifier, symbol->identifier);
							AGMModelEdge &e = newModel->getEdgeByIdentifiers(parent->identifier, symbol->identifier, "RT");
// printf("edge %s\n",  e.toString(newModel).c_str());
// printf("edge rx %s\n", e.getAttribute("rx").c_str());
							e.setAttribute("rx", "0");
							e.setAttribute("ry", "0");
							e.setAttribute("rz", "0");
							std::string axis = symbol->getAttribute("axis");
							e.setAttribute("r"+symbol->getAttribute("axis"), float2str(j.second.pos));
// 							printf("  axis    %s\n", symbol->getAttribute("axis").c_str());
//							printf("  edge rx %s\n", e.getAttribute("rx").c_str());
//							printf("  edge rz %s\n", e.getAttribute("rz").c_str());
//							printf("  edge ry %s\n", e.getAttribute("ry").c_str());
							edge_sequence.push_back(e);
							break;
						}
					}
					catch(...)
					{
						printf(" couldn't retrieve node\n");
					}
				}

				if (not found)
					printf(" couln't find joint: %s\n", j.first.c_str());
			}
		}
		//publish
		if (edge_sequence.size() > 0)
		{
			try
			{
				if( edge_sequence.size() == 1)
					AGMMisc::publishEdgeUpdate(edge_sequence.front(), agmexecutive_proxy);
				else
					AGMMisc::publishEdgesUpdate(edge_sequence, agmexecutive_proxy);
			}
			catch(...)
			{
				printf(" can't update node\n");
			}
		}
		usleep(500);
		//	printf(" done!\n");
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Exception updating InnerModel"<<std::endl;
	}

	manageRestPositionEdge(mMap);

}


bool SpecificWorker::isInRestPosition(const RoboCompJointMotor::MotorStateMap &mMap)
{
	try
	{
		if (fabs(mMap.at("armY").pos-(0.0))>0.08)
		{
			printf("armY noRest %f\n", float(mMap.at("armY").pos));
			return false;
		}
		if (fabs(mMap.at("armX1").pos-(-1.34))>0.08)
		{
			printf("armX1 noRest %f\n", float(mMap.at("armX1").pos));
			return false;
		}
		if (fabs(mMap.at("armX2").pos-(2.5))>0.08)
		{
			printf("armX2 noRest %f\n", float(mMap.at("armX2").pos));
			return false;
		}
		if (fabs(mMap.at("wristX").pos-(0))>0.08)
		{
			printf("wristX noRest %f\n", float(mMap.at("wristX").pos));
			return false;
		}
	}
	catch(...)
	{
		qFatal("Can't query motor state!");
	}
	return true;
}


void SpecificWorker::manageRestPositionEdge(const RoboCompJointMotor::MotorStateMap &mMap)
{
	QMutexLocker locker(mutex);

	if (isInRestPosition(mMap)) // We should make sure the link is there
	{
		printf("IN REST\n");
		try
		{
			AGMModelEdge e = worldModel->getEdgeByIdentifiers(1, 1, "restArm");
		}
		catch (...)
		{
			AGMModel::SPtr newModel(new AGMModel(worldModel));
			newModel->addEdgeByIdentifiers(1, 1, "restArm");
			sendModificationProposal(newModel, worldModel);
		}
	}
	else // We should make sure the edge is removed
	{
		printf("NOT IN REST\n");
		try
		{
			AGMModelEdge e = worldModel->getEdgeByIdentifiers(1, 1, "restArm");
			AGMModel::SPtr newModel(new AGMModel(worldModel));
			newModel->removeEdgeByIdentifiers(1, 1, "restArm");
			sendModificationProposal(newModel, worldModel);
		}
		catch (...)
		{
		}
	}

}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &modification)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(modification, worldModel);
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel, "world", true);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge dst;
		AGMModelConverter::fromIceToInternal(modification,dst);
		AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker l(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
}


void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	mutex->unlock();
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	mutex->lock();
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	mutex->unlock();
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel, string m)
{
	QMutexLocker locker(mutex);

	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "proprioceptionAgent")+m);
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}
