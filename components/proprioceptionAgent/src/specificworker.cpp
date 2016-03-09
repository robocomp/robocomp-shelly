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
	Period = 10;
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
	//timer.start(Period);
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
		}
		catch (const Ice::Exception &ex)
		{
			std::cout << __FILE__ << ":" << __LINE__ << " --> Can't update InnerModel" << std::endl;
		}
	}

	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);
		for (auto j : mMap)
		{
 			if (backTimes[j.first].elapsed()>500 or abs(backMotors[j.first].pos-mMap[j.first].pos) > 0.25*M_PIl/180.) /* send if it changed more than half degree */
			{
				backTimes[j.first] = QTime::currentTime();
				backMotors[j.first] = j.second;
				printf("Updating: %s (%d)\n", j.first.c_str(), newModel->size());
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
							printf("found!\n");
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
							try
							{
// 								printf("  axis    %s\n", symbol->getAttribute("axis").c_str());
// 								printf("  edge rx %s\n", e.getAttribute("rx").c_str());
// 								printf("  edge rz %s\n", e.getAttribute("rz").c_str());
// 								printf("  edge ry %s\n", e.getAttribute("ry").c_str());
								AGMMisc::publishEdgeUpdate(e, agmexecutive_proxy);
								usleep(500);
// 								printf(" done!\n");
							}
							catch(...)
							{
								printf(" can't update node\n");
							}
							break;
						}
					}
					catch(...)
					{
						printf("ddet erte\n");
					}

				}
				if (not found)
					printf("   couln't find joint: %s\n", j.first.c_str());
			}
		}
		
		
		if (first)
		{
			first = false;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
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
	innerModel = agmInner.extractInnerModel(worldModel, "room", true);
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
		agmInner.updateImNodeFromEdge(worldModel, dst, innerModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker l(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	agmInner.updateImNodeFromEdge(worldModel, dst, innerModel);
}


void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "room", true);
	mutex->unlock();
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	mutex->lock();
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "room", true);
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


void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	QMutexLocker l(mutex);
	try
	{
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmexecutive_proxy,"propioception");
	}
	catch(...)
	{
		exit(1);
	}
}




