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

	humanAdvVel = humanRotVel = humanRot = 0.;
	lastJoystickEvent = QTime::currentTime();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::includeInRCIS()
{
	printf("includeInRCIS begins\n");

	try
	{

		RoboCompInnerModelManager::Pose3D pose;
		pose.x = 3000;
		pose.y = 0;
		pose.z = 1500;
		pose.rx = pose.ry = pose.rz = 0;
		innermodelmanager_proxy->addTransform("fakeperson", "static", "root", pose);

		RoboCompInnerModelManager::meshType mesh;
		mesh.pose.x  = mesh.pose.y  = mesh.pose.z  = 0;
		mesh.pose.rx = 1.570796326794;
		mesh.pose.ry = 0;
		mesh.pose.rz = -1.570796326794;
		mesh.scaleX = mesh.scaleY = mesh.scaleZ = 20;
		mesh.render = 0;
		mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg";
		innermodelmanager_proxy->addMesh("fakeperson_mesh", "fakeperson", mesh);
	}
	catch (...)
	{
		printf("Can't create fake peson\n");
	}

	printf("includeInRCIS ends\n");
}

void SpecificWorker::includeInAGM()
{
	printf("includeInAGM begins\n");

	int idx=0;
	while ((personSymbolId = worldModel->getIdentifierByType("person", idx++)) != -1)
	{
		printf("%d %d\n", idx, personSymbolId);
		if (idx > 4) exit(0);
		if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == "fakeperson")
		{
			printf("found %d!!\n", personSymbolId);
			break;
		}
	}
	if (personSymbolId != -1)
	{
		printf("Fake person already in the AGM model\n");
		return;
	}

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	// Symbolic part
	AGMModelSymbol::SPtr person =   newModel->newSymbol("person");
	personSymbolId = person->identifier;
	printf("Got personSymbolId: %d\n", personSymbolId);
	person->setAttribute("imName", "fakeperson");
	person->setAttribute("imType", "transform");
	AGMModelSymbol::SPtr personSt = newModel->newSymbol("personSt");
	printf("person %d status %d\n", person->identifier, personSt->identifier);

	newModel->addEdge(person, personSt, "hasStatus");
	newModel->addEdge(person, personSt, "noReach");
	newModel->addEdge(person, personSt, "person");
	
	newModel->addEdgeByIdentifiers(person->identifier, 3, "in");


	// Geometric part
	std::map<std::string, std::string> edgeRTAtrs;
	edgeRTAtrs["tx"] = "3000";
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = "1500";
	edgeRTAtrs["rx"] = "0";
	edgeRTAtrs["ry"] = "0";
	edgeRTAtrs["rz"] = "0";
	newModel->addEdgeByIdentifiers(100, person->identifier, "RT", edgeRTAtrs);


	AGMModelSymbol::SPtr personMesh = newModel->newSymbol("mesh");
	printf("personMesh %d\n", personMesh->identifier);
	personMesh->setAttribute("collidable", "false");
	personMesh->setAttribute("imName", "fakepersonMesh");
	personMesh->setAttribute("imType", "mesh");
	personMesh->setAttribute("path", "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg");
	personMesh->setAttribute("render", "NormalRendering");
	personMesh->setAttribute("scalex", "20");
	personMesh->setAttribute("scaley", "20");
	personMesh->setAttribute("scalez", "20");

	edgeRTAtrs["tx"] = "0";
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = "0";
	edgeRTAtrs["rx"] = "1.570796326794";
	edgeRTAtrs["ry"] = "0";
	edgeRTAtrs["rz"] = "-1.570796326794";
	newModel->addEdge(person, personMesh, "RT", edgeRTAtrs);


	while (true)
	{
		try
		{
			sendModificationProposal(worldModel, newModel);
			break;
		}
		catch(const RoboCompAGMExecutive::Locked &e)
		{
			printf("agmexecutive locked...\n");
		}
		catch(const RoboCompAGMExecutive::OldModel &e)
		{
			return;
		}
		catch(const RoboCompAGMExecutive::InvalidChange &e)
		{
			exit(1);
		}
		sleep(1);
	}

	printf("includeInAGM ends\n");
}



void SpecificWorker::receivedJoyStickEvent(int value, int type, int number)
{
	printf("*\n");
	fflush(stdout);
	if (type != 2)
		return;
	
	if (number == 0) // rot
	{
		humanRotVel = float(value)/32767.*1.;
		lastJoystickEvent = QTime::currentTime();
	}
	if (number == 1)
	{
		humanAdvVel = float(-value)/32767.*600.;
		lastJoystickEvent = QTime::currentTime();
	}
	printf("*");
	fflush(stdout);
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

	// Include person in RCIS
	includeInRCIS();

	// Include person in AGM
	includeInAGM();

	// Joystick
	printf("Creating joystick...\n");
	joystick = new QJoyStick("/dev/input/js0");
	if (!joystick->openQJoy())
	{
		cout << "[" << PROGRAM_NAME << "]: Unable to open device: " << joystick->getDeviceName() << endl;
		return EXIT_FAILURE;
	}
	joystick->start();
	printf("Connecting joystick...\n");
	connect(joystick, SIGNAL(inputEvent(int, int, int)), this, SLOT(receivedJoyStickEvent(int, int, int)));
 

	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static QTime lastCompute = QTime::currentTime();

	if (lastJoystickEvent.elapsed()  < 3000)
	{
		printf("vel: %f %f\n", humanAdvVel, humanRotVel);
		RoboCompInnerModelManager::coord3D coordInItem;
		coordInItem.x = 0;
		coordInItem.y = 0;
		coordInItem.z = humanAdvVel*0.001*lastCompute.elapsed();
		RoboCompInnerModelManager::coord3D coordInBase;
		printf("transform (%f, %f, %f) to fake person (%f, %f, %f)\n", coordInItem.x, coordInItem.y, coordInItem.z, coordInBase.x, coordInBase.y, coordInBase.z);
		innermodelmanager_proxy->transform("root", "fakeperson",  coordInItem, coordInBase);

		RoboCompInnerModelManager::Pose3D pose;
		humanRot += humanRotVel*0.001*lastCompute.elapsed(); 
		pose.x = coordInBase.x;
		pose.y = coordInBase.y;
		pose.z = coordInBase.z;
		pose.rx = 0;
		pose.ry = humanRot;
		pose.rz = 0;
		innermodelmanager_proxy->setPoseFromParent("fakeperson", pose);



		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId, "RT");
		AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId, "RT");
		edgeRT.attributes["tx"] = float2str(coordInBase.x);
		edgeRT.attributes["ty"] = float2str(coordInBase.y);
		edgeRT.attributes["tz"] = float2str(coordInBase.z);
		edgeRT.attributes["rx"] = "0";
		edgeRT.attributes["ry"] = float2str(humanRot);
		edgeRT.attributes["rz"] = "0";
		printf("%d----[%f]--->%d\n", personParent->identifier, coordInBase.z, personSymbolId);
		AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
	}
	else
	{
		printf(".");
		fflush(stdout);
	}

	
	lastCompute = QTime::currentTime();
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
	QMutexLocker locker(mutex);
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

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
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
	QMutexLocker l(mutex);
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
		printf("exception in setParametersAndPossibleActivation\n");
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "fakeHumanAgentAgent");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}




