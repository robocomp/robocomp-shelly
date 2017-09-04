/*
 *    Copyright (C) 2006-2016 by RoboLab - University of Extremadura
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
	object_found = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
	worker_params_mutex = new QMutex(QMutex::Recursive);
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	aux.type = "float";
	aux.value = "0";
	worker_params["frameRate"] = aux;
	connect(buscar, SIGNAL(clicked()), this, SLOT(getObject()));
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
		QMutexLocker l(mutex);
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

// 	timer.start(Period);
	timer.start(1000);
	return true;
}


void SpecificWorker::compute()
{
	static QTime reloj = QTime::currentTime();
	static bool first=true;
	if (first)
	{
		qLog::getInstance()->setProxy("both", logger_proxy);
		rDebug2(("objectAgent started"));
		first = false;
	}

	QMutexLocker l(mutex);
	static std::string previousAction = "";

	bool newAction = (previousAction != action);
	if (newAction)
		printf("New action: %s\n", action.c_str());
	printf("action: %s\n", action.c_str());

	if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}

	if (action == "verifyimaginarymug")
	{
		printf("action verifyimaginarymug\n");
		try
		{
		printf("action verifyimaginarymug2 %d\n", __LINE__);
			getObject();
			//	printf("Found it!\n");
			//else
			//	printf("No object found!\n");
		}
		catch(RoboCompAGMExecutive::InvalidChange )
		{
			printf("AGMMisc::InvalidChange \n");
 		}
	}

	if (action == "graspobject" or action == "setobjectreach")
	{
		std::map<std::string, AGMModelSymbol::SPtr> symbols;
		try
		{
			symbols = worldModel->getSymbolsMap(params, "robot", "object");
		}
		catch(...)
		{
			printf("objectAgent: Couldn't retrieve action's parameters\n");
			return;
		}
		bool moving = abs(str2float(symbols["robot"]->getAttribute("movedInLastSecond"))) > 10;
		bool taza_viejuna = false;
		QTime time = QTime::currentTime();
		try
		{
			QTime timeRead = QTime::fromString(QString::fromStdString(symbols["object"]->getAttribute("LastSeenTimeStamp")),"hhmmss");
			qDebug()<<"now: "<<time.toString("hhmmss") << "time readed:" << timeRead.toString("hhmmss")<<"time difference: "<<timeRead.secsTo(time);
			if (timeRead.secsTo(QTime::currentTime()) > 25) //Long time no see
			{
				qDebug()<<"taza viejuna";
				qDebug()<<abs(str2float(symbols["robot"]->getAttribute("movedInLastSecond")));
				taza_viejuna = true;
			}
		}
		catch(...)
		{
			printf("Exception: Could not retrieve LastSeenTimeStamp attribute\n");
		}

		if (not moving and taza_viejuna )
		{
			getObject();
		}
	}




	previousAction = action;

	worker_params_mutex->lock();
		//save framerate in params
		worker_params["frameRate"].value = std::to_string(reloj.restart()/1000.f);
	worker_params_mutex->unlock();
}

bool SpecificWorker::detectAndLocateObject(std::string objectToDetect, bool first)
{
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	QVec poseFromParent;
	static QTime waitTime;
	bool retValue = false;
	if (first)
		waitTime = QTime::currentTime();
	bool object_found = false;
	bool publishModel = false;
	ObjectType poseobj;

	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = newModel->getSymbolsMap(params, "robot", "status", "room", objectToDetect, objectToDetect+"St", "table");
	}
	catch(...)
	{
		printf("objectAgent: Couldn't retrieve action's parameters\n");
		return false;
	}

	if (!cb_simulation->isChecked() )
	{
		printf("checking with real pipeline\n");
		ObjectVector objects;
		StringVector objectsTofind;
		//Pipelining!!
		try
		{
			objectsTofind.push_back(objectToDetect);
			object_found = objectdetection_proxy->findObjects(objectsTofind, objects);
		}
		catch(...)
		{
			printf("Imposible to connect to objectdetection \n");
			return false;
		}
		if (object_found)
		{
			poseobj = objects[0];
			AGMModelSymbol::SPtr symbolTable = newModel->getParentByLink(symbols[objectToDetect]->identifier , "RT");
			QString tableIMName = QString::fromStdString(symbolTable->getAttribute("imName"));
			QVec positionObject = QVec::vec6(poseobj.tx, poseobj.ty, poseobj.tz);
			QMat rotationObject = Rot3D(poseobj.rx, poseobj.ry, poseobj.rz);
			QVec positionFromParent  = innerModel->transform(tableIMName, positionObject, "rgbd");
			QMat rotationRGBD2Parent = innerModel->getRotationMatrixTo(tableIMName, "rgbd"); //matriz rotacion del nodo padre a la rgbd
			QVec rotation;
			rotation = (rotationRGBD2Parent * rotationObject).invert().extractAnglesR_min();
			poseFromParent = QVec::zeros(6);
			poseFromParent.inject(positionFromParent, 0);
			poseFromParent.inject(rotation, 3);
		}

	}
	else
	{
		printf("fake detection\n");
		object_found = cb_mug->isChecked();
		poseFromParent = QVec::vec6(0,97,0,0,0,0);
	}


	try
	{
		newModel->getEdge(symbols["robot"], symbols["status"], "usedOracle");
	}
	catch(...)
	{
		printf("The oracle has already been used\n");
		return false;
	}

	//if object located
	if (object_found )
	{
		printf("Object found\n");
		AGMModelSymbol::SPtr symbolProtoObject = symbols[objectToDetect];
		//convert protoObject to object
		symbolProtoObject->setType("object");
		symbolProtoObject->setAttribute("imName", std::string("object_")+int2str(symbolProtoObject->identifier));

		//finding the table father to the object
		AGMModelSymbol::SPtr symbolTable = newModel->getParentByLink(symbolProtoObject->identifier , "RT");
		try
		{
			AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers(symbolTable->identifier, symbolProtoObject->identifier, "RT");
			poseFromParent.print("poseFromParent");
			edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
			edgeRT->setAttribute("ty", "0");//float2str(poseFromParent.y()));
			edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
			edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
			edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
			edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));
			rDebug2(("objectAgent edgeupdate for table"));
		}
		catch(...)
		{
			qFatal("Impossible to update the RT edge");
		}

		try
		{
			// The oracle has been used, remove flagS
			try
			{
				newModel->removeEdge(symbols["robot"], symbols["status"], "usedOracle");
			}
			catch(...)
			{
				printf("Can't remove edge %d--[usedOracle]-->%d\n", symbols["robot"]->identifier, symbols["status"]->identifier);
			}
			//add mesh
			AGMModelSymbol::SPtr mugMesh = newModel->newSymbol("mugMesh");
			mugMesh->setAttribute("collidable", "true");
			mugMesh->setAttribute("imName", std::string("mesh_")+int2str(mugMesh->identifier));
			mugMesh->setAttribute("imType", "mesh");
			mugMesh->setAttribute("path", "/home/robocomp/robocomp/components/prp/experimentFiles/simulation/mug_blue.3ds");
			mugMesh->setAttribute("render", "NormalRendering");
			mugMesh->setAttribute("scalex", "100");
			mugMesh->setAttribute("scaley", "100");
			mugMesh->setAttribute("scalez", "100");
			//model offset
			std::map<std::string, std::string> edgeRTMeshAtrs;
			edgeRTMeshAtrs["tx"] = "0";
			edgeRTMeshAtrs["ty"] = "-48.5";
			edgeRTMeshAtrs["tz"] = "0";
			edgeRTMeshAtrs["rx"] = "1.57079";
			edgeRTMeshAtrs["ry"] = "0";
			edgeRTMeshAtrs["rz"] = "3.141592";
			newModel->addEdge(symbolProtoObject, mugMesh, "RT", edgeRTMeshAtrs);
			rDebug2(("objectAgent edgeupdate for table"));
		}
		catch(...)
		{
			qFatal("Impossible to create the RT edge to the mug mesh");
		}
		retValue = true;
		publishModel = true;
	}
	else //if (waitTime.elapsed() > 10000) //if not found remove protoObject
	{
		try
		{
			//AGMModelPrinter::printWorld(newModel);
			int id = symbols[objectToDetect]->identifier;
			newModel->removeSymbol(id);
		}
		catch(AGMModelException e)
		{
			printf("Can't remove %d symbol\n", symbols[objectToDetect]->identifier);
			printf("ERROR: %s\n", e.what());
		}
		catch(...)
		{
			printf("Can't remove %d symbol\n", symbols[objectToDetect]->identifier);
		}
		try
		{
			newModel->removeSymbol(symbols[objectToDetect+"St"]->identifier);
		}
		catch(...)
		{
			printf("Can't remove %d symbol\n", symbols[objectToDetect+"St"]->identifier);
		}
		try
		{
			newModel->removeEdge(symbols["robot"], symbols["status"], "usedOracle");
		}
		catch(...)
		{
			printf("Can't remove edge %d--[usedOracle]-->%d\n", symbols["robot"]->identifier, symbols["status"]->identifier);
		}
		try
		{
			newModel->removeDanglingEdges();
		}
		catch(...)
		{
			printf("Oracle was not used, how do we even got here in the first place? \n");
		}
		publishModel = true;
	}

	//publish changes
	if (publishModel)
	{
		printf("modification %d\n", __LINE__);
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectAgent");
		printf("modification %d\n", __LINE__);
	}
	return retValue;
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

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &modification)
{
	QMutexLocker l(mutex);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
	printf("lalalalalalala %d\n", modification.version);
 	AGMModelConverter::fromIceToInternal(modification, worldModel);

	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker l(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
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


void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker l(mutex);
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



void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel, string c)
{
	printf("versoin %d\n", newModel->version);
	while (true)
	{
		try
		{
			printf("modification %d\n", __LINE__);
			AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectAgent"+c);
			printf("modification %d\n", __LINE__);
		}
		catch(const RoboCompAGMExecutive::Locked &e)
		{
			printf("error when sending modification proposal (Locked)\n");
		}
		catch(const RoboCompAGMExecutive::OldModel &e)
		{
			throw e;
		}
		catch(const RoboCompAGMExecutive::InvalidChange &e)
		{
			throw e;
		}
		catch(const Ice::Exception& e)
		{
			printf("error when sending modification proposal\n");
			exit(1);
		}
		catch(...)
		{
			exit(1);
		}
	}
}

void SpecificWorker::updateTag(const tagsList &list)
{
	QMutexLocker l(mutex);

	if (worldModel->numberOfSymbols() == 0)
		return;


	AGMModel::SPtr newModel(new AGMModel(worldModel));

	for (auto ap : list)
	{
		auto ap2 = ap;
		switch(ap2.id)
		{
			case 30:
				if (updateTable(ap2, newModel))
				{
					printf("New table was detected!\n");
					rDebug2(("objectAgent new table detected"));
				}
				qDebug()<<ap2.id<<"POSE: "<<innerModel->transform("robot", QVec::vec3(ap2.tx, ap2.ty, ap2.tz), "rgbd");
				break;
			case 31:
			case 32:
				if (updateMug(ap2, newModel))
				{
					rDebug2(("objectAgent new mug detected"));
					printf("New mug was detected!\n");
				}
				qDebug()<<ap2.id<<"POSE: "<<innerModel->transform("robot", QVec::vec3(ap2.tx, ap2.ty, ap2.tz), "rgbd");
				break;
		}
	}
}

// Get new apriltags!
void SpecificWorker::newAprilTag(const tagsList &list)
{
	//qDebug()<<"**************************\nUsing newAprilTagAndpose instead\n********************************";
}

// Get new apriltags!
void SpecificWorker::newAprilTagAndPose(const tagsList &list,const RoboCompGenericBase::TBaseState &bState,const RoboCompJointMotor::MotorStateMap &hState)
{

	QMutexLocker l(mutex);
	//save previous values
	MotorStateMap backPoses = hState; //save motors name
	for (auto motor: hState)
	{
		backPoses[motor.first].pos = innerModel->getJoint(motor.first)->getAngle();
		innerModel->getJoint(motor.first)->setAngle(motor.second.pos);
	}
	QVec basePose = innerModel->getTransform("robot")->getTr();
	float baseAlpha = innerModel->getTransform("robot")->extractAnglesR_min().y();

	innerModel->updateTransformValues("robot",bState.correctedX,0,bState.correctedZ,0,bState.correctedAlpha,0);

	//update object position
	updateTag(list);

	//restore values
	for (auto motor: hState)
	{
		innerModel->getJoint(motor.first)->setAngle(backPoses[motor.first].pos);
	}
	innerModel->updateTransformValues("robot",basePose(0),0,basePose(2),0,baseAlpha,0);

}



/**
 * \brief Metodo UPDATE TABLE:
 * 1) Tomamos ID de la tabla en AGM y en INNER
 * 2) Calulamospadre.
 * 3) Calculamos enlace anterior E
 * 4) UPDATE AGM
 */
bool SpecificWorker::updateTable(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	printf("===========================\n===   updateTable   =========\n===========================\n");
	bool existing = false;
	AGMModelSymbol::SPtr tableSymbol;

	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		tableSymbol = *symbol_it;
		if (tableSymbol->symbolType == "object" and tableSymbol->attributes.find("tag") != tableSymbol->attributes.end())
		{
			try
			{
				const int32_t tag = str2int(tableSymbol->getAttribute("tag"));
				if (t.id == tag)
				{
					// Si el identificador del nodo coincide con el del tag entonces el simbolo existe.
					// Salimos del bucle porque ya lo hemos encontrado.
					existing = true;
					break;
				}
			}
			catch (...){ printf("   ERROR %s: %d\n", __FILE__, __LINE__);}
		}
	}

	// If the table already exists: update its position
	if (not existing)
	{
		qFatal("Table doesn't exist in AGM: we should create the symbol here but it's not implemented yet");
	}

	QVec positionTag    = QVec::vec6(t.tx, t.ty, t.tz); // tag position from parent
	QMat rotationOffset = Rot3D(-M_PI_2, 0, 0); // apriltags' rotation offset
	QMat rotationTag    = Rot3D(t.rx, t.ry, t.rz); // apriltags' rotation as seen
	//QVec resultingEuler = (rotationTag*rotationOffset.invert()).extractAnglesR_min();
	//QVec poseTag        = QVec::vec6(t.tx, t.ty, t.tz, resultingEuler.rx(), resultingEuler.ry(), resultingEuler.rz()); // tag pose from parent, takin into account mugs' offsets

	QString tableIMName  = QString::fromStdString(tableSymbol->getAttribute("imName"));
	// qDebug() << "Table IM name " << symbolIMName << "    agm identifer: " << symbol->identifier;
	InnerModelNode *tableIM = innerModel->getNode(tableIMName);

	if (not tableIM)
	{
		qDebug() << "Table's node doesnt exist in InnerModel";
	}

	InnerModelNode *parentNodeIM = tableIM->parent;

	if (not parentNodeIM)
	{
		qDebug() << "Parent node doesnt exist in InnerModel";
	}

	QString parentIMName = parentNodeIM->id;
	//qDebug() << "Table's parent: " << parentIMName;


	QVec positionFromParent  = innerModel->transform(parentIMName, positionTag, "rgbd");
	//QVec poseFromParent  = innerModel->transform(parentIMName, poseTag, "rgbd");
	QMat rotationRGBD2Parent = innerModel->getRotationMatrixTo(parentIMName, "rgbd"); //matriz rotacion del nodo padre a la rgbd
	QVec rotation;
	//rotation.print("rotation");
	rotation = (rotationRGBD2Parent * rotationTag * rotationOffset).invert().extractAnglesR_min();
	// COMPONEMOS LA POSE ENTERA:
	QVec poseFromParent = QVec::zeros(6);
	poseFromParent.inject(positionFromParent, 0);
	poseFromParent.inject(rotation, 3);
	//poseFromParent.print("pose from parent");

	//BUSCAR EL ENLACE RT: NECESITAMOS EL SIMBOLO PADRE EN AGM
	bool parentFound = false;
	AGMModelSymbol::SPtr symbolParent;
	for(AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		symbolParent = *symbol_it;
		if (symbolParent->attributes["imName"] == parentIMName.toStdString())
		{
			//qDebug() << "parent in AGM: " << QString::fromStdString(symbolParent->attributes["imName"]);
			parentFound = true;
			break;
		}
	}
	if (parentFound)
	{
		// Si el padre existe en AGM sacamos el enlace RT que va desde el padre hasta el hijo y actualizamos sus valores
		try
		{
			AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers(symbolParent->identifier, tableSymbol->identifier, "RT");
			edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
			edgeRT->setAttribute("ty", "0.0");//float2str(poseFromParent.y()));
			edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
			edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
			edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
			edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));

			AGMInner::updateImNodeFromEdge(newModel, edgeRT, innerModel);
			printf("modification %d\n", __LINE__);
			AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
			printf("modification %d\n", __LINE__);
			rDebug2(("objectAgent edgeupdate for table"));
		}
		catch(...){ qFatal("Impossible to update the RT edge"); }
	}
	else
		qDebug() << "Parent node doesnt exist in AGM";

	return not existing;
}

/**
 * \brief UPDATE MUG
 * 1) Tomamos ID de la taza en AGM y en INNER
 * 2) Calulamos padre.
 * 3) Calculamos enlace anterior E
 * 4) UPDATE AGM
 */
bool SpecificWorker::updateMug(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	printf("===========================\n===   updateMug   =========\n===========================\n");
	bool existing = false;
	float THRESHOLD_mugInTable = 750;
	AGMModelSymbol::SPtr symbolMug,symbolMugSt;

	// Verify that the mug is not already in the model (using the tag attribute)
	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		if (symbol_it->symbolType == "object" and symbol_it->attributes.find("tag") != symbol_it->attributes.end())
		{
			try
			{
				const int32_t tag = str2int(symbol_it->getAttribute("tag"));
				if (t.id == tag)
				{
					symbolMug = *symbol_it;
					existing = true;
					break;
				}
			}
			catch (...){ printf("   ERROR %s: %d\n", __FILE__, __LINE__);}
		}
	}

	// If the mug doesn't already exist: create new symbols for it
	if (not existing)
	{
		try
		{
			std::string mug_name = "mug"+std::to_string(t.id);
			AGMModelSymbol::SPtr robot = newModel->getSymbolByIdentifier(newModel->getIdentifierByType("robot"));
			symbolMug = newModel->newSymbol("object");
			symbolMug->setAttribute("imName",mug_name);
			symbolMug->setAttribute("imType","transform");
			const std::string tagIdStr = int2str(t.id);
			symbolMug->setAttribute("tag", tagIdStr);

			symbolMugSt = newModel->newSymbol("objectSt");

			newModel->addEdge(symbolMug, symbolMugSt, "hasStatus");
			newModel->addEdge(symbolMug, symbolMugSt, "mug");
			newModel->addEdge(symbolMug, symbolMugSt, "see");
			newModel->addEdge(symbolMug, symbolMugSt, "position");
			newModel->addEdge(symbolMug, symbolMugSt, "classified");
			newModel->addEdge(symbolMug, symbolMugSt, "reachable");
			newModel->addEdge(symbolMug, symbolMugSt, "noReach");
			newModel->addEdge(robot, symbolMug, "know");

			// mug reach position
			AGMModelSymbol::SPtr reachMugSt = newModel->newSymbol("objectSt");
			reachMugSt->setAttribute("imName",mug_name+"_reachPos");
			reachMugSt->setAttribute("imType","transform");
			newModel->addEdge(symbolMug, reachMugSt, "reachPosition");
			newModel->addEdge(symbolMug, reachMugSt, "RT");
			AGMModelEdge &edgeRT  = newModel->getEdge(symbolMug, reachMugSt, "RT");
			edgeRT->setAttribute("tx", "0");
			edgeRT->setAttribute("ty", "0");
			edgeRT->setAttribute("tz", "0");
			edgeRT->setAttribute("rx", "0");
			edgeRT->setAttribute("ry", "0");
			edgeRT->setAttribute("rz", "0");

			//mug mesh
			AGMModelSymbol::SPtr symbolMugMesh = newModel->newSymbol("mesh");
			symbolMugMesh->setAttribute("collidable", "true");
			symbolMugMesh->setAttribute("imName", mug_name+"_MugR");
			symbolMugMesh->setAttribute("imType", "mesh");
			symbolMugMesh->setAttribute("path", "/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/autonomyLab/mug_blue.3ds");
			symbolMugMesh->setAttribute("render", "NormalRendering");
			symbolMugMesh->setAttribute("scalex", "120");
			symbolMugMesh->setAttribute("scaley", "120");
			symbolMugMesh->setAttribute("scalez", "120");

			AGMModelSymbol::SPtr symbolMugMeshTransform = newModel->newSymbol("transform");
			symbolMugMeshTransform->setAttribute("engine", "static");
			symbolMugMeshTransform->setAttribute("imName", mug_name+"_RPr");
			symbolMugMeshTransform->setAttribute("imType", "transform");
			symbolMugMeshTransform->setAttribute("mass", "0");
			newModel->addEdge(symbolMugMeshTransform, symbolMugMesh, "RT");
			AGMModelEdge &edgeRT2  = newModel->getEdge(symbolMugMeshTransform, symbolMugMesh, "RT");
			edgeRT2->setAttribute("tx", "0");
			edgeRT2->setAttribute("ty", "0");
			edgeRT2->setAttribute("tz", "0");
			edgeRT2->setAttribute("rx", "1.5707");
			edgeRT2->setAttribute("ry", "0");
			edgeRT2->setAttribute("rz", "-1.5707");

			newModel->addEdge(symbolMug, symbolMugMeshTransform, "RT");
			AGMModelEdge &edgeRT3  = newModel->getEdge(symbolMug, symbolMugMeshTransform, "RT");
			edgeRT3->setAttribute("tx", "-10");
			edgeRT3->setAttribute("ty", "50");
			edgeRT3->setAttribute("tz", "0");
			edgeRT3->setAttribute("rx", "0");
			edgeRT3->setAttribute("ry", "0");
			edgeRT3->setAttribute("rz", "0");

			newModel->addEdge(symbolMug, symbolMugMeshTransform, "RT");
			AGMModelEdge &edgeRT4  = newModel->getEdge(symbolMug, symbolMugMeshTransform, "RT");
			edgeRT4->setAttribute("tx", "-10");
			edgeRT4->setAttribute("ty", "50");
			edgeRT4->setAttribute("tz", "0");
			edgeRT4->setAttribute("rx", "0");
			edgeRT4->setAttribute("ry", "0");
			edgeRT4->setAttribute("rz", "0");

			try
			{
				qDebug()<<"trying to insert a new mug in the model";
				sendModificationProposal(worldModel, newModel, "update mug viejo");
exit(1);
				qDebug()<<"new mug inserted in model ";
			}
			catch(...)
			{
				printf("(sendModificationProposal) objectAgent: Couldn't publish new mug in model\n");
			}
			return existing;
		}
		catch(...)
		{
			printf("ERROR: creating new mug");
		}
	}
	// If the mug already exists: update its position
	else
	{
		//Update last seen tag
		QTime time = QTime::currentTime();
		try
		{
			QTime timeRead = QTime::fromString(QString::fromStdString(symbolMug->getAttribute("LastSeenTimeStamp")),"hhmmss");
			if (timeRead.secsTo(time) > 3 ) //update each 3 seconds
			{
				symbolMug->setAttribute("LastSeenTimeStamp", time.toString("hhmmss").toStdString());
				try
				{
					printf("modification %d\n", __LINE__);
					AGMMisc::publishNodeUpdate(symbolMug, agmexecutive_proxy);
					printf("modification %d\n", __LINE__);
				}
				catch (...)
				{
					printf("LastSeenTimeStamp Exception: Executive not running?\n");
				}
			}
		}
		catch(...)
		{
			//Create atributte first time
			symbolMug->setAttribute("LastSeenTimeStamp", time.toString("hhmmss").toStdString());
			try
			{
				printf("modification %d\n", __LINE__);
				AGMMisc::publishNodeUpdate(symbolMug, agmexecutive_proxy);
				printf("modification %d\n", __LINE__);
			}
			catch (...)
			{
				printf("LastSeenTimeStamp Exception: Executive not running?\n");
			}
			printf("Exception: Could not retrieve LastSeenTimeStamp attribute\n");
		}

		//check mug table in
		//Update innermodel pose
		QVec positionTag    = QVec::vec6(t.tx, t.ty, t.tz); // tag position from parent
		QMat rotationOffset = Rot3D(-M_PI_2, 0, 0); // apriltags' rotation offset
		QMat rotationTag    = Rot3D(t.rx, t.ry, t.rz); // apriltags' rotation as seen
		QVec tagInWorld  = innerModel->transform("world", positionTag, "rgbd");

		//check if mug is in robot
		bool mugInrobot=false;
		try
		{
			int robotID = newModel->getIdentifierByType("robot");
			AGMModelEdge e = newModel->getEdgeByIdentifiers(symbolMug->identifier, robotID, "in");
			mugInrobot = true;
		}
		catch (...)
		{
			printf("Mug not in robot\n");
		}
		//else
		if (!mugInrobot)
		{
			float min_distance = -1;
			AGMModelSymbol::SPtr symbolNewTable, symbolWasInTable;
			for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
			{
				if (symbol_it->symbolType == "object")
				{
					if (isObjectType(newModel, *symbol_it, "table"))
					{
						std::string tableIMName = symbol_it->getAttribute("imName");
						QMat tableInWorld = innerModel->transformS("world", tableIMName);
						float distance = (tagInWorld - tableInWorld).norm2();
						//qDebug()<<"checking distance: "<<tableIMName.c_str()<<"distence"<<distance;
						if (distance < THRESHOLD_mugInTable and (distance < min_distance or min_distance<0) )
						{
							min_distance = distance;
							symbolNewTable = *symbol_it;
						}
					}
					try
					{
 						AGMModelEdge e = newModel->getEdgeByIdentifiers(symbolMug->identifier, symbol_it->identifier, "in");
						symbolWasInTable = *symbol_it;
					}
					catch (...)
					{
					}
				}
			}
			if (symbolNewTable and symbolNewTable != symbolWasInTable)
			{
				AGMModelSymbol::SPtr roomWas, roomIn;
				qDebug()<<"Change tableMug:";
				if(symbolWasInTable)
				{
					qDebug()<<"--> wasIn: "<<symbolWasInTable->getAttribute("imName").c_str();
					//check table room to update mug_>room link
					roomWas = getRoomFromTable(newModel, symbolWasInTable);
					try{
						newModel->removeEdge(symbolWasInTable, symbolMug, "RT");
						newModel->removeEdge(symbolMug, symbolWasInTable, "in");
//						newModel->removeEdge(symbolMug, symbolWasInTable, "wasIn");

					}catch(...)
					{
						qDebug()<<"Error removing wasIn table links";
					}


				}
				roomIn = getRoomFromTable(newModel, symbolNewTable);
				if(roomIn and roomIn != roomWas)
				{
					qDebug()<<"Change roomMug:";
					if(roomWas)
					{
						qDebug()<<"--> wasIn: "<<roomWas->getAttribute("imName").c_str();
						newModel->removeEdge(symbolMug, roomWas, "in");
					}
					qDebug()<<"--> now In: "<<roomIn->getAttribute("imName").c_str();
					newModel->addEdge(symbolMug, roomIn, "in");
				}
				try{
					newModel->addEdge(symbolMug, symbolNewTable, "in");
//					newModel->addEdge(symbolMug, symbolNewTable, "wasIn");
					newModel->addEdge(symbolNewTable, symbolMug, "RT");
					qDebug()<<"--> now in: "<<symbolNewTable->getAttribute("imName").c_str();
				}catch(...)
				{
					qDebug()<<"Error inserting new In links";
				}
				try{
					sendModificationProposal(worldModel, newModel, "update mug viejo");
exit(1);
				}
				catch(...)
				{
					printf("(sendModificationProposal) objectAgent: Couldn't publish new mug in model\n");
				}
				return existing;
			}
		}

		QString symbolIMName       = QString::fromStdString(symbolMug->getAttribute("imName"));
		InnerModelNode *nodeSymbolIM = innerModel->getNode(symbolIMName);

		if (nodeSymbolIM)
		{
			InnerModelNode *parentNodeIM = nodeSymbolIM->parent;
			if (parentNodeIM)
			{
				QString parentIMName    = parentNodeIM->id;
// 				qDebug() << "Mug's parent: " << parentIMName;
				QVec positionFromParent  = innerModel->transform(parentIMName, positionTag, "rgbd");
				QMat rotationTag         = Rot3D(t.rx, t.ry, t.rz); //rotacion propia de la marca
				QMat rotationRGBD2Parent = innerModel->getRotationMatrixTo(parentIMName, "rgbd"); //matriz rotacion del nodo padre a la rgbd
				QVec rotation;
				rotation = (rotationRGBD2Parent * rotationTag * rotationOffset).invert().extractAnglesR_min();
				// COMPONEMOS LA POSE ENTERA:
				QVec poseFromParent = QVec::zeros(6);
				poseFromParent.inject(positionFromParent, 0);
				poseFromParent.inject(rotation, 3);
 				poseFromParent.print("pose from parent");

				//BUSCAR EL ENLACE RT: NECESITAMOS EL SIMBOLO PADRE EN AGM
				bool parentFound = false;
				AGMModelSymbol::SPtr symbolParent;
				for(AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
				{
					symbolParent = *symbol_it;
					if (symbolParent->symbolType == "object" and symbolParent->attributes["imName"]==parentIMName.toStdString())
					{
// 						qDebug() << "parent in AGM: " << QString::fromStdString(symbolParent->attributes["imName"]);
						parentFound = true;
						break;
					}
				}
				if (parentFound)
				{
					// Si el padre existe en AGM sacamos el enlace RT que va desde el padre hasta el hijo y actualizamos sus valores
					try
					{
						AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers(symbolParent->identifier, symbolMug->identifier, "RT");
						edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
						edgeRT->setAttribute("ty", "0.0");//float2str(poseFromParent.y()));
						edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
						// Do not update rotation if id=31 ==> tag
						if(t.id == 31 or t.id == 32)
						{
							edgeRT->setAttribute("rx", "0.0");
							edgeRT->setAttribute("ry", "0.0");
							edgeRT->setAttribute("rz", "0.0");
						}
						else
						{
							edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
							edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
							edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));
						}
// 						qDebug() << "Updating edge!";
						printf("modification %d\n", __LINE__);
						AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
						printf("modification %d\n", __LINE__);
						rDebug2(("objectAgent edgeupdate for mug"));
					}
					catch(...){ qDebug()<<"Impossible to update the RT edge"; }
				}
				else
					qDebug() << "Parent node doesnt exist in AGM";
			}
			else
			{
				qDebug() << "Parent node doesnt exist in InnerModel";
			}
		}
		else
		{
			qDebug() << "Mug's node doesnt exist in InnerModel";
		}
	}
	return not existing;
}

AGMModelSymbol::SPtr SpecificWorker::getRoomFromTable(AGMModel::SPtr model, AGMModelSymbol::SPtr table)
{
	try{
		AGMModelSymbol::SPtr symbolRoom;
		for (auto edgeRoom = table->edgesBegin(model); edgeRoom != table->edgesEnd(model); edgeRoom++)
		{
			if (model->getSymbolByIdentifier(edgeRoom->getSymbolPair().second)->symboltype() == "room" and  edgeRoom->getLabel() == "in" )
			{
				symbolRoom = model->getSymbolByIdentifier(edgeRoom->getSymbolPair().second);
				return symbolRoom;
			}
		}
	}catch(...)
	{}
	return NULL;
}

bool SpecificWorker::isObjectType(AGMModel::SPtr model, AGMModelSymbol::SPtr node, const std::string &t)
{
	QMutexLocker locker(mutex);

	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model); edge_itr++)
	{
		AGMModelEdge edge = *edge_itr;
		if (edge->getLabel() == t)
		{
			return true;
		}
	}
	return false;
}

bool SpecificWorker::updateMilk(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	QMutexLocker l(mutex);

	bool existing = false;

	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object") {
			try {
				const int32_t tag = str2int(symbol->getAttribute("tag"));
				if (t.id == tag) {
// 					QVec v(6); v(0) = t.tx; v(1) = t.ty; v(2) = t.tz; v(3) = t.rx; v(4) = t.ry; v(5) = t.rz;
// 					QVec worldRef = innerModel->transform("world", v, "rgbd");
					existing = true;
				}
			}
			catch (...) { }
		}
	}

/*	if (not existing)
	{
		try
		{
			int32_t objectSymbolID;
			int32_t objectStSymbolID;
			getIDsFor("milk", objectSymbolID, objectStSymbolID);

			auto symbols = newModel->getSymbolsMap(params, "robot", "container");
			AGMModelSymbol::SPtr newMilk = newModel->newSymbol("object", objectSymbolID);
			AGMModelSymbol::SPtr newMilkStatus = newModel->newSymbol("objectSt", objectStSymbolID);
			newModel->addEdge(symbols["robot"], newMilk, "know");
			newModel->addEdge(newMilk, newMilkStatus, "hasStatus");
			newModel->addEdge(newMilk, newMilkStatus, "see");
			newModel->addEdge(newMilk, newMilkStatus, "position");
			newModel->addEdge(newMilk, newMilkStatus, "reachable");
			newModel->addEdge(newMilk, newMilkStatus, "reach");
			newModel->addEdge(newMilk, newMilkStatus, "classifailed");
			newModel->addEdge(newMilk, symbols["container"], "in");

			const std::string tagIdStr = int2str(t.id);
			newMilk->attributes["tag"] = tagIdStr;
			newMilk->attributes["tx"] = "1100";
			newMilk->attributes["ty"] = "820";
			newMilk->attributes["tz"] = "-1350";
			newMilk->attributes["rx"] = "0";
			newMilk->attributes["ry"] = "-3.1415926535";
			newMilk->attributes["rz"] = "0";
		}
		catch(...)
		{
			printf("(updateMilk) objectAgent: Couldn't retrieve action's parameters\n");
		}
	}*/

	const bool forcePublishModel = not existing;
// 	printf("force publish by milk %d (%d)\n", forcePublishModel, t.id);
	return forcePublishModel;
}

bool SpecificWorker::updateCoffee(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	return false;
}

void SpecificWorker::getIDsFor(std::string obj, int32_t &objectSymbolID, int32_t &objectStSymbolID)
{
	objectSymbolID = -1;
	objectStSymbolID = -1;

	QStringList actions = QString::fromStdString(params["plan"].value).toLower().split("\n");

	for (auto a : actions)
	{
		if (a.contains(QString::fromStdString(obj)))
		{
			QStringList parts = a.split("'");
			for (int32_t index=0; index<parts.size()-2; index++)
			{
				if      (parts[index] == "objectr") objectSymbolID   = parts[index+2].toInt();
				else if (parts[index] == "statusr") objectStSymbolID = parts[index+2].toInt();
			}
		}
	}
	printf("------------------------------->%d %d\n", objectSymbolID, objectStSymbolID);
}

void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
{
	printf("me cago en mi vida\n");
	QMutexLocker l(mutex);

	static QTime lastTime;

	if (newAction)
		lastTime = QTime::currentTime();

	if (lastTime.elapsed() > 5000)
	{
 		AGMModel::SPtr newModel(new AGMModel(worldModel));
		auto symbols = newModel->getSymbolsMap(params, "container");
		auto node = symbols["container"];

		for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
		{
			if ((*edge_itr)->getLabel() == "noExplored")
			{
				getObject();
				(*edge_itr)->setLabel("explored");
				rDebug2(("objectAgent action_FindObjectVisuallyInTable"));
exit(1);
				sendModificationProposal(worldModel, newModel);
				return;
			}
		}

	}
}

RoboCompCommonBehavior::ParameterList SpecificWorker::getWorkerParams()
{
	QMutexLocker locker(worker_params_mutex);
	return worker_params;
}




void SpecificWorker::findObject()
{
	printf("%d\n", __LINE__);
	ObjectVector objects;
	StringVector objectsTofind;
	objectsTofind.push_back("yatekomo");
	ObjectType poseobj;
	if(objectdetection_proxy->findObjects(objectsTofind, objects))
	{
		printf("%d\n", __LINE__);
		bool found = false;
		for (auto o : objects)
		{
			if (o.label == "yatekomo")
			{
				found = true;
				poseobj = o;
			}
		}
		if (not found)
		{
			return;
		}
		QVec::vec6(poseobj.tx, poseobj.ty, poseobj.tz, poseobj.rx, poseobj.ry, poseobj.rz).print("Pose recibida: ");
		printf("%d\n", __LINE__);
		QVec posobj = innerModel->transform6D("rgbd",QVec::vec6(poseobj.tx, poseobj.ty, poseobj.tz, poseobj.rx, poseobj.ry, poseobj.rz),"robot");
		printf("%d\n", __LINE__);
		t.id=31;
		t.tx=posobj.x();
		t.ty=posobj.y();
		t.tz=posobj.z();
		t.rx=posobj.rx();
		t.ry=posobj.ry();
		t.rz=posobj.rz();
		printf("%d\n", __LINE__);
		posobj.print("from rgbd");
		printf("%d\n", __LINE__);
		object_found = true;
		printf("%d\n", __LINE__);
	}
	printf("%d\n", __LINE__);
}

void SpecificWorker::getObject()
{
	printf("%d\n", __LINE__);
	printf("SpecificWorker::getObject()\n");
	if(!object_found)
	{
		printf("%d\n", __LINE__);
		findObject();
		printf("%d\n", __LINE__);
	}

	printf("%d\n", __LINE__);
	if (object_found)
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		printf("%d\n", __LINE__);
		updateOracleMug(t,newModel);
		printf("%d\n", __LINE__);
	}
}

/*
void SpecificWorker::getObjects()
{
	ObjectVector objects;
	StringVector objectsTofind;
	if (objectdetection_proxy->findObjects(objectsTofind, objects))
	{
		for (auto object:objects)
		{
			if (object.label=="pringles")
			{
				AGMModel::SPtr newModel(new AGMModel(worldModel));
				RoboCompAprilTags::tag t;
				t.id=32;
				t.tx=object.tx;
				t.ty=object.ty;
				t.tz=object.tz;
				t.rx=object.rx;
				t.ry=object.ry;
				t.rz=object.rz;
				updateMug(t,newModel);
			}
		}
	}
}
*/
void SpecificWorker::updateOracleMug(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	printf("===========================\n===   updateOracleMug   =========\n===========================\n");
	bool foundMug = false;
	float THRESHOLD_mugInTable = 750;
	AGMModelSymbol::SPtr symbolMug;
	std::string mug_obj_name;

	if(action == "verifyimaginarymug")
		mug_obj_name = "mug";
	else
		mug_obj_name = "object";

	std::map<std::string, AGMModelSymbol::SPtr> symbolsss;
	try
	{
		symbolsss = newModel->getSymbolsMap(params, mug_obj_name);
		symbolMug = symbolsss[mug_obj_name];
		foundMug = true;
	}
	catch(...)
	{
		printf("objectAgent: Couldn't retrieve action's parameters\n");
		exit(1);
	}


	if(foundMug)
	{
		std::cout<<"Mug was found, updating biatch"<<std::endl;


		//check mug table in
		//Update innermodel pose
		QVec positionTag    = QVec::vec6(t.tx, t.ty, t.tz); // tag position from parent
		QMat rotationOffset = Rot3D(-M_PI_2, 0, 0); // apriltags' rotation offset
		QMat rotationTag    = Rot3D(t.rx, t.ry, t.rz); // apriltags' rotation as seen
		QVec tagInWorld  = innerModel->transform("world", positionTag, "rgbd");


		QString symbolIMName;
		try
		{

			symbolIMName = QString::fromStdString(symbolMug->getAttribute("imName"));
			qDebug()<<"Found mug's imName";
		}
		catch(...)
		{
			qDebug()<<"No mug's im name found creating new one";
			symbolIMName = QString::fromStdString(symbolMug->symboltype()) + QString::number(symbolMug->identifier);
			qDebug()<<"o no";
		}

		InnerModelNode *nodeSymbolIM = innerModel->getNode(symbolIMName);
		qDebug()<<"SI existe el im en el innermodel";

		if (nodeSymbolIM)
		{
			qDebug()<<"El hiueputa no tiene padre";
			InnerModelNode *parentNodeIM = nodeSymbolIM->parent;
			qDebug()<<"SI existe el im en el innermodel";
			if (parentNodeIM)
			{
				QString parentIMName    = parentNodeIM->id;
// 				qDebug() << "Mug's parent: " << parentIMName;
				QVec positionFromParent  = innerModel->transform(parentIMName, positionTag, "rgbd");
				QMat rotationTag         = Rot3D(t.rx, t.ry, t.rz); //rotacion propia de la marca
				QMat rotationRGBD2Parent = innerModel->getRotationMatrixTo(parentIMName, "rgbd"); //matriz rotacion del nodo padre a la rgbd
				QVec rotation;
				rotation = (rotationRGBD2Parent * rotationTag * rotationOffset).invert().extractAnglesR_min();
				// COMPONEMOS LA POSE ENTERA:
				QVec poseFromParent = QVec::zeros(6);
				poseFromParent.inject(positionFromParent, 0);
				poseFromParent.inject(rotation, 3);
 				poseFromParent.print("pose from parent");

				//BUSCAR EL ENLACE RT: NECESITAMOS EL SIMBOLO PADRE EN AGM
				bool parentFound = false;
				AGMModelSymbol::SPtr symbolParent;
				for(AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
				{
					symbolParent = *symbol_it;
					if (symbolParent->symbolType == "object" and symbolParent->attributes["imName"]==parentIMName.toStdString())
					{
 						qDebug() << "parent in AGM: " << QString::fromStdString(symbolParent->attributes["imName"]);
						parentFound = true;
						break;
					}
				}
				if (parentFound)
				{
					// Si el padre existe en AGM sacamos el enlace RT que va desde el padre hasta el hijo y actualizamos sus valores
					try
					{
						AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers(symbolParent->identifier, symbolMug->identifier, "RT");
						edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
						edgeRT->setAttribute("ty", "0.0");//float2str(poseFromParent.y()));
						edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
						// Do not update rotation if id=31 ==> tag
						if(t.id == 31 or t.id == 32)
						{
							edgeRT->setAttribute("rx", "0.0");
							edgeRT->setAttribute("ry", "0.0");
							edgeRT->setAttribute("rz", "0.0");
						}
						else
						{
							edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
							edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
							edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));
						}
// 						qDebug() << "Updating edge!";
// 						AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
						//rDebug2(("objectAgent edgeupdate for mug");
					}
					catch(...){ qDebug()<<"Impossible to update the RT edge"; }
				}
				else
					qDebug() << "Parent node doesnt exist in AGM";
			}
			else
			{
				qDebug() << "Parent node doesnt exist in InnerModel";
			}
		}

		//If mug was imagined remove image edge and add know
		auto symbols = worldModel->getSymbolsMap(params, mug_obj_name, "robot");
		try
		{
			newModel->removeEdge(symbols["robot"], symbols[mug_obj_name], "imagine");
		}
		catch(...)
		{
			printf("No imagine %d -> %d found, was object imagined by oracle?\n", º, symbols[mug_obj_name]->identifier);
		}

		try
		{
			newModel->addEdge(symbols["robot"], symbols[mug_obj_name], "know");
		}
		catch(...)
		{
			printf("Robot knows about the mug. Know edge can't be added to model.\n");
		}
printf("%d\n", __LINE__);
		if(action == "verifyimaginarymug")
		{
			printf("%d\n", __LINE__);
			try
			{
				printf("%d\n", __LINE__);
				newModel->removeEdgeByIdentifiers(1,2, "usedOracle");
				printf("%d\n", __LINE__);
			}
			catch(...)
			{
				printf("%d\n", __LINE__);
				printf("Can't remove edge %d--[usedOracle]-->%d\n", 1, 2);
				printf("%d\n", __LINE__);
			}
		}

		//add the mesh
		try
		{
			printf("%d\n", __LINE__);

			//add mesh
			AGMModelSymbol::SPtr mugMesh = newModel->newSymbol("mugMesh");
			mugMesh->setAttribute("collidable", "true");
			mugMesh->setAttribute("imName", std::string("mesh_")+int2str(mugMesh->identifier));
			mugMesh->setAttribute("imType", "mesh");
			mugMesh->setAttribute("path", "/home/robocomp/robocomp/components/prp/experimentFiles/simulation/mug_blue.3ds");
			mugMesh->setAttribute("render", "NormalRendering");
			mugMesh->setAttribute("scalex", "100");
			mugMesh->setAttribute("scaley", "100");
			mugMesh->setAttribute("scalez", "100");
			//model offset
			std::map<std::string, std::string> edgeRTMeshAtrs;
			edgeRTMeshAtrs["tx"] = "0";
			edgeRTMeshAtrs["ty"] = "48.5";
			edgeRTMeshAtrs["tz"] = "0";
			edgeRTMeshAtrs["rx"] = "1.57079";
			edgeRTMeshAtrs["ry"] = "0";
			edgeRTMeshAtrs["rz"] = "3.141592";
			newModel->addEdge(symbolMug, mugMesh, "RT", edgeRTMeshAtrs);
			rDebug2(("objectAgent edgeupdate for mesh"));
			printf("%d\n", __LINE__);
		}
		catch(...)
		{
			printf("%d\n", __LINE__);
			qFatal("Impossible to create the RT edge to the mug mesh");
		}
		printf("%d\n", __LINE__);

		static int mnameId=0;
		// string mname = QString::number(mnameId++).toStdString() + ".xml";
		// printf("saving to %s", mname.c_str());
		// newModel->save(mname);

		//Update last seen tag
		printf("%d\n", __LINE__);
		QTime time = QTime::currentTime();
		try
		{
			printf("%d\n", __LINE__);
			QTime timeRead = QTime::fromString(QString::fromStdString(symbolMug->getAttribute("LastSeenTimeStamp")),"hhmmss");
			if (timeRead.secsTo(time) > 3 ) //update each 3 seconds
			{
				symbolMug->setAttribute("LastSeenTimeStamp", time.toString("hhmmss").toStdString());
				try
				{
					printf("modification %d\n", __LINE__);
					AGMMisc::publishNodeUpdate(symbolMug, agmexecutive_proxy);
					printf("modification %d\n", __LINE__);
				}
				catch (...)
				{
					printf("LastSeenTimeStamp Exception: Executive not running?\n");
				}
			}
		}
		catch(...)
		{
			printf("%d\n", __LINE__);
			//Create atributte first time
			symbolMug->setAttribute("LastSeenTimeStamp", time.toString("hhmmss").toStdString());
			try
			{
				printf("modification %d\n", __LINE__);
				AGMMisc::publishNodeUpdate(symbolMug, agmexecutive_proxy);
				printf("modification %d\n", __LINE__);
			}
			catch (...)
			{
				printf("LastSeenTimeStamp Exception: Executive not running?\n");
			}
			printf("Exception: Could not retrieve LastSeenTimeStamp attribute\n");
		}

		//publish changes
		printf("%d\n", __LINE__);
		try
		{
// 			sendModificationProposal(worldModel, newModel, "SpecificWorker::getObject()");
			printf("modification %d\n", __LINE__);
			AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectAgent SpecificWorker::getObject() 1");
			printf("modification %d\n", __LINE__);
		}
		catch(RoboCompAGMExecutive::OldModel)
		{
			printf("AGMMisc::OldModel 1\n");
			structuralChange(agmexecutive_proxy->getModel());
			newModel = AGMModel::SPtr(new AGMModel(worldModel));
			updateOracleMug(t,newModel);
			try
			{
// 				sendModificationProposal(worldModel, newModel, "SpecificWorker::getObject()");
				printf("modification %d\n", __LINE__);
				AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectAgent SpecificWorker::getObject() 2");
				printf("modification %d\n", __LINE__);
			}
			catch(RoboCompAGMExecutive::OldModel)
			{
				printf("AGMMisc::OldModel 2\n");
				qDebug()<<"ModificationProposalSent      FAILS";
			}
			qDebug()<<"ModificationProposalSent2         DONE";
		}
		qDebug()<<"ModificationProposalSent1         DONE";


	}
	//mug was not found
	else
	{
		std::cout<<"ERROR: MUG WAS NOT FOUND, IT WAS PROBABLY NOT IMAGINED PROPERLY"<<std::endl;
	}

}
