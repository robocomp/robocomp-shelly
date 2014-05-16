/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	active = false;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	/// FUSCA TEMPORAL
	printf("%s\n", action.c_str());
	if (action == "findobjectvisually")
	{
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	//return 0;
}

bool SpecificWorker::deactivateAgent()
{
	//return 0;
}

StateStruct SpecificWorker::getAgentState()
{
	//return 0;
}

ParameterMap SpecificWorker::getAgentParameters()
{
	//return 0;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
	//return 0;
}

void SpecificWorker::killAgent()
{
}

int SpecificWorker::uptimeAgent()
{
	//return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	//return 0;
}

void SpecificWorker::modelModified(const RoboCompAGMWorldModel::Event& modification)
{
}

void SpecificWorker::modelUpdated(const RoboCompAGMWorldModel::Node& modification)
{
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
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

		if (action == "findObjectVisually")
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
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	return true;
}
