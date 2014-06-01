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
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";

	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus/etc/ursusMM.xml");
	trTag = innerModel->newTransform("trTag", "static", innerModel->getTransform("rgbd_transform"));
	innerModel->getTransform("rgbd_transform")->addChild(trTag);


}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	printf("------------------ %d \n", (int)tagMap.size());
	//
	// Update InnerModel using the joints of the robot
	MotorStateMap mstateMap;
	jointmotor_proxy->getAllMotorState(mstateMap);
	for (MotorStateMap::iterator it= mstateMap.begin(); it!=mstateMap.end(); it++)
	{
		innerModel->updateJointValue(QString::fromStdString(it->first), it->second.pos);
	}
	/// Update wrist position in the robot symbol according to innermodel or the corresponding April tag
	updateWristPose();

	//
	// Remove tags which have not been seen in two seconds
	QTime now = QTime::currentTime(); // Get current time
	for (TagModelMap::iterator i=tagMap.begin();  i!=tagMap.end(); )
	{
		if (i->second.lastTime.msecsTo(now) > 2000)
		{
			printf("Removing %d\n", i->second.id);
			tagMap.erase(i++);
		}
		else
		{
			++i;
		}
	}

	//
	// Create a copy of the model and get the symbol of the robot (just in case)
	bool modelModified = false;
	RoboCompAGMWorldModel::Event e;
	e.sender = "apriltags";
	e.why = RoboCompAGMWorldModel::BehaviorBasedModification;
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	int32_t robotId = newModel->getIdentifierByType("robot");
	if (robotId == -1)
	{
		printf("didn't find the robot!!\n");
		return;
	}
	AGMModelSymbol::SPtr robot = newModel->getSymbol(robotId);

	//
	// Include new tags which are not currently in the model and update those which are
	for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
	{
		printf("* See %d\n", itMap->second.id);
		bool found = false;
		for (AGMModel::iterator itModel=worldModel->begin(); itModel!=worldModel->end(); itModel++)
		{
			if (itModel->symbolType == "object")
			{
				if (itMap->second.id == str2int(itModel->attributes["id"]))
				{
					found = true;
					updateSymbolWithTag(*itModel, itMap->second);
					break;
				}
			}
		}
		if (not found)
		{
			printf("    NOT found in model\n");
			includeObjectInModel(newModel, itMap->second);
			modelModified = true;
		}
		else
			printf("    found in model\n");
	}
	//
	// Remove objects which have not been seen in a while
	std::vector<int32_t> symbolsToRemove;
	for (AGMModel::iterator itModel=worldModel->begin(); itModel!=worldModel->end(); itModel++)
	{
		if (itModel->symbolType == "object")
		{
			printf("* Stored %d\n", str2int(itModel->attributes["id"]));
			bool found = false;
			for (TagModelMap::iterator itMap=tagMap.begin();  itMap!=tagMap.end(); itMap++)
			{
				if (itMap->second.id == str2int(itModel->attributes["id"]))
				{
					found = true;
					printf("    found in current tags\n");
					break;
				}
			}
			if (not found)
			{
				printf("    NOT found\n");
				modelModified = true;
				for (AGMModelSymbol::iterator itEdge=itModel->edgesBegin(newModel); itEdge!=itModel->edgesEnd(newModel); itEdge++)
				{
					if (itEdge->linking == "prop")
					{
						// If the object is known, we should also remove its type label
						AGMModelSymbol::SPtr class_p = newModel->getSymbol(itEdge->symbolPair.second);
						if (class_p->symbolType == "class")
						{
							for (AGMModelSymbol::iterator itEdgeClass=class_p->edgesBegin(newModel); itEdgeClass!=class_p->edgesEnd(newModel); itEdgeClass++)
							{
								symbolsToRemove.push_back(itEdgeClass->symbolPair.second);
							}
						}
						symbolsToRemove.push_back(itEdge->symbolPair.second);
					}
				}
				symbolsToRemove.push_back(itModel->identifier);
			}
		}
	}
	for (uint32_t iii=0; iii<symbolsToRemove.size(); iii++)
	{
		printf("removing  %d (%d)\n", iii, symbolsToRemove[iii]);
		try
		{
			newModel->removeSymbol(symbolsToRemove[iii]);
		}
		catch(...)
		{
			printf("error removing symbol %d\n", symbolsToRemove[iii]);
		}
	}
	
	
	
	
	/// 
	printf("%s\n", action.c_str());
	if (action == "findobjectvisually")
	{
	}
	else if (action == "setobjectreach")
	{
		try
		{
			AGMModelSymbol::SPtr r = newModel->getSymbolByIdentifier(atoi(params["r"].value.c_str()));
			r->symbolType = "reach";
			printf("dentro!\n");
			modelModified = true;
		}
		catch(...)
		{
		}
	}
	else if (action == "graspobject")
	{
	}
	else
	{
		printf("no action\n");
	}

	if (modelModified)
	{
		printf("MODIFIED!\n");
		try
{
// 		AGMMisc::publishModification(newModel, agmagenttopic, worldModel);
		RoboCompAGMWorldModel::Event e;
		e.why = RoboCompAGMWorldModel::BehaviorBasedModification;
		AGMModelConverter::fromInternalToIce(worldModel, e.backModel);
		AGMModelConverter::fromInternalToIce(newModel, e.newModel);
		printf("<<%d\n", newModel->numberOfSymbols());
		// 	AGMModelPrinter::printWorld(newModel);
		agmagenttopic->modificationProposal(e);
		printf(">>\n");
}
catch(...)
{
	exit(1);
}
		
		
	}
}


void SpecificWorker::updateSymbolWithTag(AGMModelSymbol::SPtr symbol, const AprilTagModel &tag)
{
	innerModel->updateTransformValues("trTag", tag.tx, tag.ty, tag.tz, tag.rx+M_PIl, tag.ry, tag.rz);
	QVec T = innerModel->transform("robot", QVec::vec3(0,0,0), "trTag");
	QVec R = innerModel->getRotationMatrixTo("trTag", "robot").extractAnglesR_min();

   //threshold to update set to 20 mm and 0.1 rad
   if ( fabs(str2float((symbol->attributes["tx"])) - T(0)) > 20  ||
	    fabs(str2float((symbol->attributes["ty"])) - T(1)) > 20  ||
	    fabs(str2float((symbol->attributes["tz"])) - T(2)) > 20  ||
	    fabs(str2float((symbol->attributes["rx"])) - R(0)) > 0.1 ||
	    fabs(str2float((symbol->attributes["ry"])) - R(1)) > 0.1 ||
	    fabs(str2float((symbol->attributes["rz"])) - R(2)) > 0.1 )
	{
		symbol->attributes["tx"] = float2str(T(0));
		symbol->attributes["ty"] = float2str(T(1));
		symbol->attributes["tz"] = float2str(T(2));
		symbol->attributes["rx"] = float2str(R(0));
		symbol->attributes["ry"] = float2str(R(1));
		symbol->attributes["rz"] = float2str(R(2));
		RoboCompAGMWorldModel::Node symbolICE;
		AGMModelConverter::fromInternalToIce(symbol, symbolICE);
		agmagenttopic->update(symbolICE);
	}
}

void SpecificWorker::includeObjectInModel(AGMModel::SPtr &newModel, const AprilTagModel &tag)
{
	int32_t robotId = newModel->getIdentifierByType("robot");
	if (robotId == -1)
	{
		return;
	}
	/// object
	AGMModelSymbol::SPtr newTag = newModel->newSymbol("object");
	newModel->addEdgeByIdentifiers(robotId, newTag->identifier, "know");
	newTag->attributes["id"] = int2str(tag.id);
	newTag->attributes["tx"] = float2str(tag.tx);
	newTag->attributes["ty"] = float2str(tag.ty);
	newTag->attributes["tz"] = float2str(tag.tz);
	newTag->attributes["rx"] = float2str(tag.rx);
	newTag->attributes["ry"] = float2str(tag.ry);
	newTag->attributes["rz"] = float2str(tag.rz);
	/// see
	AGMModelSymbol::SPtr   see_p = newModel->newSymbol("see");
	newModel->addEdgeByIdentifiers(newTag->identifier, see_p->identifier, "prop");
	/// reach
	AGMModelSymbol::SPtr reach_p = newModel->newSymbol("noReach");
	newModel->addEdgeByIdentifiers(newTag->identifier, reach_p->identifier, "prop");
	/// pose
	AGMModelSymbol::SPtr  pose_p = newModel->newSymbol("pose");
	newModel->addEdgeByIdentifiers(newTag->identifier, pose_p->identifier, "prop");


	/// TAG-DEPENDENT CODE
	/// class
	AGMModelSymbol::SPtr class_p;
	AGMModelSymbol::SPtr type_p;
	switch(tag.id)
	{
	case 0:
		class_p = newModel->newSymbol("class");
		newModel->addEdgeByIdentifiers(newTag->identifier, class_p->identifier, "prop");
		type_p = newModel->newSymbol("table");
		newModel->addEdgeByIdentifiers(class_p->identifier, type_p->identifier, "prop");
		break;
	case 2:
		class_p = newModel->newSymbol("class");
		newModel->addEdgeByIdentifiers(newTag->identifier, class_p->identifier, "prop");
		type_p = newModel->newSymbol("mug");
		newModel->addEdgeByIdentifiers(class_p->identifier, type_p->identifier, "prop");
		break;
	default:
		AGMModelSymbol::SPtr class_p = newModel->newSymbol("noClass");
		newModel->addEdgeByIdentifiers(newTag->identifier, class_p->identifier, "prop");
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			mutex->lock();
			active = true;
			mutex->unlock();
			return true;
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::deactivateAgent()
{
	mutex->lock();
	active = false;
	mutex->unlock();
	return true;
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (active)
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = action;
	return s;
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

		if (action == "graspobject")
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

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent()
{
	exit(-1);
}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

void SpecificWorker::modelModified(const RoboCompAGMWorldModel::Event& modification)
{
	mutex->lock();
	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	mutex->unlock();	
}

void SpecificWorker::modelUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	mutex->lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	mutex->unlock();
}

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	QTime c = QTime::currentTime();

	for (uint32_t i=0; i<tags.size(); i++)
	{
		if (tagMap.find(tags[i].id) != tagMap.end())
		{
			tagMap[tags[i].id] = AprilTagModel();
		}
		else
		{
			printf("new tag %d\n", tags[i].id);
		}

		tagMap[tags[i].id].id = tags[i].id;
		tagMap[tags[i].id].tx = tags[i].tx;
		tagMap[tags[i].id].ty = tags[i].ty;
		tagMap[tags[i].id].tz = tags[i].tz;
		tagMap[tags[i].id].rx = tags[i].rx;
		tagMap[tags[i].id].ry = tags[i].ry;
		tagMap[tags[i].id].rz = tags[i].rz;
		tagMap[tags[i].id].lastTime = c;
	}
}


bool SpecificWorker::updateWristPose()
{
	// Make sure we have the robot in the model, otherwise there's nothing to do yet...
	int32_t robotId = worldModel->getIdentifierByType("robot");
	if (robotId == -1)
	{
		return false;
	}
	AGMModelSymbol::SPtr robot = worldModel->getSymbol(robotId);

	// Set current T and R
	QVec T = innerModel->transform("robot", QVec::vec3(0,0,0), "arm_right_8");
	QVec R = innerModel->getRotationMatrixTo("arm_right_8", "robot").extractAnglesR_min();

	// Set back T and R
	
	QVec T_back, R_back;
	bool force = false;
	try
	{
		T_back = QVec::vec3(
		  str2float(robot->attributes["rightwrist_tx"]),
		  str2float(robot->attributes["rightwrist_ty"]),
		  str2float(robot->attributes["rightwrist_tz"]));
		R_back = QVec::vec3(
		  str2float(robot->attributes["rightwrist_rx"]),
		  str2float(robot->attributes["rightwrist_ry"]),
		  str2float(robot->attributes["rightwrist_rz"]));
	}
	catch(...)
	{
		force = true;
	}
	#warning These thresholds should be set in the config file!!!
	#warning These thresholds should be set in the config file!!!
	#warning These thresholds should be set in the config file!!!
	#warning These thresholds should be set in the config file!!!
	if ( force or (T-T_back).norm2()>15 or (R-R_back).norm2()>0.05)
	{
		robot->attributes["rightwrist_tx"] = float2str(T(0));
		robot->attributes["rightwrist_ty"] = float2str(T(1));
		robot->attributes["rightwrist_tz"] = float2str(T(2));
		robot->attributes["rightwrist_rx"] = float2str(R(0));
		robot->attributes["rightwrist_ry"] = float2str(R(1));
		robot->attributes["rightwrist_rz"] = float2str(R(2));
		return true;
	}
	
	return false;
}


