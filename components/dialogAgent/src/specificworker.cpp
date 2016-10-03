/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
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
	timer.start(Period);
	
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	speech_proxy->say("hola mundo", false);

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	
	actionExecution();
}

void SpecificWorker::actionExecution()
{
	static std::string previousAction = "";
	bool newAction = (previousAction != action);
	
	if (action == "handobject_offer")
	{
		action_handObject_offer(newAction);
	}
	previousAction = action;
}

void SpecificWorker::action_handObject_offer(bool first)
{
	// Lock mutex and get a model's copy
	QMutexLocker locker(mutex);
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	
	// Get action parameters
	try
	{
		symbols = newModel->getSymbolsMap(params, "object", "person", "status");
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}
	// Attempt to get the 'offered' edge, in which case we're done
	try
	{
		newModel->getEdge(symbols["object"], symbols["person"], "offered");
		return;
	}
	catch(...)
	{
	}

	// Attempt to get the person "reach" edge. Proceed if successfull.
	try
	{
		// Get the person "reach" edge.
		newModel->getEdge(symbols["person"], symbols["status"], "reach");
		// Make the robot speak
		if (first)
		{
			speech_proxy->say("toma la puta taza, que cansinos sois con la jodida taza",false);
		}
		sleep(2);
		// Make the action noticeable in the model.
		newModel->addEdge(symbols["object"], symbols["person"], "offered");
		// Publish the modification
		sendModificationProposal(worldModel, newModel);
	}
	catch(...)
	{
		// Edge not present yet or some error raised. Try again in a few milliseconds.
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker l(mutex);

	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
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
	try
	{
		//AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "dialogagentAgent");
	}
	catch(RoboCompAGMExecutive::Locked ex)
	{
		std::cout<<"Exception => locked"<<std::endl;
	}
	catch(RoboCompAGMExecutive::OldModel ex)
	{
		std::cout<<"Exception => OldModel"<<std::endl;
		try
		{
			RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
			structuralChange(w);
		}
		catch(...)
		{
			printf("The executive is probably not running, waiting for first AGM model publication...");
		}

	}
	catch(RoboCompAGMExecutive::InvalidChange ex)
	{
		std::cout<<"Exception => InvalidChange"<<std::endl;
		exit(1);
	}
}




