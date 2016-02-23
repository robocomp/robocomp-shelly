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
	Period = 20;
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
	haveTarget = false;

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	// ODOMETRY AND LOCATION-RELATED ISSUES
	if (odometryAndLocationIssues() == false)
		return;
// 	innerModel->treePrint();

	
	actionExecution();

// 	printf("ae>\n");
}

/**
 * \brief ESTE ES EL VERDADERO COMPUTE
 */ 
void SpecificWorker::actionExecution()
{
// 	return;
	
	QMutexLocker locker(mutex);
	qDebug()<<"ACTION: "<<QString::fromStdString(action);

	static std::string previousAction = "";
	bool newAction = (previousAction != action);

	if (newAction)
	{
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());
	}

// 	try
// 	{
// 		planningState = trajectoryrobot2d_proxy->getState();
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;
// 	}
// 	catch(...)
// 	{
// 		printf("something else %d\n", __LINE__);
// 	}

	if (action == "changeroom")
	{
		action_ChangeRoom(newAction);
	}
	else if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}
	else if (action == "setobjectreach" or action == "graspobject")
	{
		action_SetObjectReach(newAction);
	}
	else if (action == "detectperson")
	{
		action_DetectPerson(newAction);
	}
	else if (action == "handobject")
	{
		action_HandObject(newAction);
	}
	else
	{
		action_NoAction(newAction);
	}


	if (newAction)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
	printf("actionExecution>>\n");
}

void SpecificWorker::action_DetectPerson(bool newAction)
{

	static bool b=false;
	if (b==false)
        {
		trajectoryrobot2d_proxy->stop();
		b=true;
	}
	try
	{
		omnirobot_proxy->setSpeedBase(0.,0.,0.1);
	}
	catch(...)
	{
		printf("Can't connect to the robot!!\n");
	}

}

void SpecificWorker::action_HandObject(bool newAction)
{
	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) {	exit(-1);  }
	}

	// Get target
	int roomID, personID, robotID;
	try
	{
		if (symbols["room"].get() and symbols["person"].get() and symbols["robot"].get())
		{
			roomID = symbols["room"]->identifier;   //7 ROOM
			personID =symbols["person"]->identifier;//  PERSON
			robotID = symbols["robot"]->identifier; //1 ROBOT
		}
		else
		{
			printf("navigationAgent, action_HandObject: parameters not in the model yet\n");
			return;
		}
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject ERROR: SYMBOL DOESN'T EXIST \n");
		exit(2);
	}
	
	// GET THE INNERMODEL NAMES OF TH SYMBOLS
	QString robotIMID;
	QString roomIMID;
	QString personIMID;
	try
	{
		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
		//we need to obtain the imName of the torso node. TrackingId+"XN_SKEL_TORSO"
		QString trackingId= QString::fromStdString(worldModel->getSymbol(personID)->getAttribute("TrackingId"));
		personIMID = trackingId +"XN_SKEL_TORSO";
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject: ERROR IN GET THE INNERMODEL NAMES\n");
		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
		exit(2);
	}
	
	// GET THE TARGET POSE: 
	try
	{
		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(personIMID)))    return;
		
		QVec poseInRoom = innerModel->transform6D(roomIMID, personIMID); // FROM OBJECT TO ROOM
		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
		qDebug()<<" TARGET POSE: "<< poseInRoom;

// 		currentTarget.first = objectID;
		currentTarget.x = poseInRoom.x();
		currentTarget.y = 0;
		currentTarget.z = poseInRoom.z();
		currentTarget.rx = 0;
		currentTarget.ry = 0;
		currentTarget.rz = 0;
		currentTarget.doRotation = false;
	}
	catch (...) 
	{ 
		qDebug()<<"navigationAgent, action_HandObject: innerModel exception";
	}

	try
	{
// 		if (!haveTarget)
		{
			try
			{
				QVec graspRef = innerModel->transform("robot", "right_shoulder_grasp_pose");
				float th=20;
				trajectoryrobot2d_proxy->goReferenced(currentTarget, graspRef.x(), graspRef.z(), th);
				std::cout << "trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << currentTarget.doRotation << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
				haveTarget = true;
			}
			catch(const Ice::Exception &ex)
			{
				std::cout <<"navigationAgent, action_HandObject: ERROR trajectoryrobot2d->go "<< ex << std::endl;
				throw ex;
			}
		}
		string state;
		try
		{
			state = trajectoryrobot2d_proxy->getState().state;
		}
		catch(const Ice::Exception &ex)
		{
			std::cout <<"navigationAgent, action_HandObject: trajectoryrobot2d->getState().state "<< ex << std::endl;
			throw ex;
		}

		//state="IDLE";
		std::cout<<state<<" haveTarget "<<haveTarget;
		if (state=="IDLE" && haveTarget)
		{
			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
			try
			{
// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				int statusID =symbols["status"]->identifier;
// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				sendModificationProposal(worldModel, newModel);
				haveTarget=false;
			}
			catch (...)
			{
				std::cout<<"\neeeee"<< "\n";
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	

}


/**
 * \brief elmeollo dl asunto
 */ 
void SpecificWorker::action_SetObjectReach(bool newAction)
{
	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
	}
	catch(...)
	{
		printf("navigationAgent: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) {	exit(-1);  }
	}

	// Get target
	int roomID, objectID, robotID;
	try
	{
		if (symbols["room"].get() and symbols["object"].get() and symbols["robot"].get())
		{
			roomID = symbols["room"]->identifier;
			objectID =symbols["object"]->identifier;
			robotID = symbols["robot"]->identifier;
		}
		else
		{
			printf("parameters not in the model yet\n");
			return;
		}
	}
	catch(...)
	{
		printf("ERROR: SYMBOL DOESN'T EXIST \n");
		exit(2);
	}
	

	// GET THE INNERMODEL NAMES OF TH SYMBOLS
	QString robotIMID;
	QString roomIMID;
	QString objectIMID;
	try
	{
		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
		objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));
	}
	catch(...)
	{
		printf("ERROR IN GET THE INNERMODEL NAMES\n");
		exit(2);
	}
	

	// GET THE TARGET POSE: 
	try
	{
		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(objectIMID)))    return;
		QVec poseInRoom = innerModel->transform6D(roomIMID, objectIMID); // FROM OBJECT TO ROOM
		qDebug()<<" TARGET POSE: "<< poseInRoom;

// 		currentTarget.first = objectID;
		currentTarget.x = poseInRoom.x();
		currentTarget.y = 0;
		currentTarget.z = poseInRoom.z();
		currentTarget.rx = 0;
		currentTarget.ry = 0;
		currentTarget.rz = 0;
		currentTarget.doRotation = true;
	}
	catch (...) 
	{ 
		qDebug()<<"innerModel exception";
	}

	// Execute target
	try
	{
// 		if (!haveTarget)
		{
			try
			{
				QVec O = innerModel->transform("right_shoulder_grasp_pose", objectIMID);
				O.print("pose relativa");
				printf("__%f__\n", O.norm2());
				QVec graspRef = innerModel->transform("robot", "right_shoulder_grasp_pose");
				float th=20;
				trajectoryrobot2d_proxy->goReferenced(currentTarget, graspRef.x(), graspRef.z(), th);
				std::cout << "trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
				haveTarget = true;
			}
			catch(const Ice::Exception &ex)
			{
				std::cout <<"ERROR trajectoryrobot2d->go "<< ex << std::endl;
				throw ex;
			}
		}
		string state;
		try
		{
				state = trajectoryrobot2d_proxy->getState().state;
		}
		catch(const Ice::Exception &ex)
		{
				std::cout <<"trajectoryrobot2d->getState().state "<< ex << std::endl;
				throw ex;
		}

		//state="IDLE";
		std::cout<<state<<" haveTarget "<<haveTarget;
		if (state=="IDLE" && haveTarget)
		{
			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
			try
			{
// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				int statusID =symbols["status"]->identifier;
// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				sendModificationProposal(worldModel, newModel);
				haveTarget=false;
			}
			catch (...)
			{
				std::cout<<"\neeeee"<< "\n";
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}


bool SpecificWorker::odometryAndLocationIssues(bool force)
{
	//
	// Get ODOMETRY and update it in the graph. If there's a problem talking to the robot's platform, abort
	try
	{
		omnirobot_proxy->getBaseState(bState);
	}
	catch (...)
	{
		printf("Can't connect to the robot!!\n");
		return false;
	}

	int32_t robotId=-1, roomId=-1;

	robotId = worldModel->getIdentifierByType("robot");
	if (robotId < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return false;
	}

	AGMModelSymbol::SPtr robot = worldModel->getSymbol(robotId);
	for (auto edge = robot->edgesBegin(worldModel); edge != robot->edgesEnd(worldModel); edge++)
	{
		if (edge->getLabel() == "RT")
		{
			const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
			const string secondType = worldModel->getSymbol(symbolPair.first)->symbolType;
			if (symbolPair.second == robotId and secondType == "room")
			{
				roomId = symbolPair.first;
			}
		}
	}

	if (roomId < 0)
	{
		printf("roomId not found, Waiting for Insert innerModel...\n");
		return false;
	}

	
	int32_t robotIsActuallyInRoom;
	if (bState.z<0)
		robotIsActuallyInRoom = 5;
	else
		robotIsActuallyInRoom = 3;

// 	for (const AGMModelEdge &edge : worldModel->edges)
// 	{
// 		if (edge->getSymbolPair().first == robotId and edge->getLabel() == "in")
// 		{
// 		}
// 	}

	if (roomId != robotIsActuallyInRoom)
	{
		try
		{
			AGMModel::SPtr newModel(new AGMModel(worldModel));

			// Modify IN edge
			newModel->removeEdgeByIdentifiers(robotId, roomId, "in");
			newModel->addEdgeByIdentifiers(robotId, robotIsActuallyInRoom, "in");

			// Modify RT edge
			AGMModelEdge edgeRT = newModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			newModel->removeEdgeByIdentifiers(roomId, robotId, "RT");
			printf("(was %d now %d) ---[rt]---> %d\n", roomId, robotIsActuallyInRoom, robotId);
			try
			{
				float bStatex = str2float(edgeRT->getAttribute("tx"));
				float bStatez = str2float(edgeRT->getAttribute("tz"));
				float bStatealpha = str2float(edgeRT->getAttribute("ry"));
				
				// to reduce the publication frequency
				printf("xModel=%f xBase=%f\n", bStatex, bState.correctedX);
				if (fabs(bStatex - bState.correctedX)>5 or fabs(bStatez - bState.correctedZ)>5 or fabs(bStatealpha - bState.correctedAlpha)>0.02 or force)
				{
					//Publish update edge
					printf("\nUpdate odometry...\n");
					qDebug()<<"bState local --> "<<bStatex<<bStatez<<bStatealpha;
					qDebug()<<"bState corrected --> "<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;

					edgeRT->setAttribute("tx", float2str(bState.correctedX));
					edgeRT->setAttribute("tz", float2str(bState.correctedZ));
					edgeRT->setAttribute("ry", float2str(bState.correctedAlpha));
				}
				printf(".");
				fflush(stdout);
				newModel->addEdgeByIdentifiers(robotIsActuallyInRoom, robotId, "RT", edgeRT->attributes);
				AGMMisc::publishModification(newModel, agmexecutive_proxy, "navigationAgent");
			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update room... do edges exist? !!!\n");
			return false;
		}	}
	else
	{
		try
		{
			AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			printf("%d ---[rt]---> %d  (%d\n", roomId, robotId, __LINE__);
			try
			{
				float bStatex = str2float(edge->getAttribute("tx"));
				float bStatez = str2float(edge->getAttribute("tz"));
				float bStatealpha = str2float(edge->getAttribute("ry"));
				
				// to reduce the publication frequency
				printf("xModel=%f xBase=%f\n", bStatex, bState.correctedX);
				if (fabs(bStatex - bState.correctedX)>5 or fabs(bStatez - bState.correctedZ)>5 or fabs(bStatealpha - bState.correctedAlpha)>0.02 or force)
				{
					//Publish update edge
					printf("\nUpdate odometry...\n");
					qDebug()<<"bState local --> "<<bStatex<<bStatez<<bStatealpha;
					qDebug()<<"bState corrected --> "<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;

					edge->setAttribute("tx", float2str(bState.correctedX));
					edge->setAttribute("tz", float2str(bState.correctedZ));
					edge->setAttribute("ry", float2str(bState.correctedAlpha));
					AGMMisc::publishEdgeUpdate(edge, agmexecutive_proxy);
					printf("done\n");
				}
				printf(".");
				fflush(stdout);

			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update odometry in RT, edge does not exist? !!!\n");
			return false;
		}
	}


	return true;
}

void SpecificWorker::updateRobotsCognitiveLocation()
{
	// If the polygons are not set yet, there's nothing to do...
	if (roomsPolygons.size()==0)
		return;

	// Get current location according to the model, if the location is not set yet, there's nothing to do either
	const int32_t currentLocation = getIdentifierOfRobotsLocation(worldModel);
	if (currentLocation == -1) return;

	// Compute the robot's location according to the odometry and the set of polygons
	// If we can't find the room where the robot is, we assume it didn't change, so there's nothing else to do
	int32_t newLocation = -1;
	for (auto &kv : roomsPolygons)
	{
		if (kv.second.containsPoint(QPointF(bState.x,  bState.z), Qt::OddEvenFill))
		{
			newLocation = kv.first;
			break;
		}
	}
	if (newLocation == -1) return;

	// If everyting is ok AND the robot changed its location, update the new location in the model and
	// propose the change to the executive
	if (newLocation != currentLocation and newLocation != -1)
	{
		printf("sending modification!\n");
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		setIdentifierOfRobotsLocation(newModel, newLocation);
		AGMModelPrinter::printWorld(newModel);
		sendModificationProposal(worldModel, newModel);
	}
}


std::map<int32_t, QPolygonF> SpecificWorker::extractPolygonsFromModel(AGMModel::SPtr &worldModel)
{
	std::map<int32_t, QPolygonF> ret;

	for (AGMModel::iterator symbol_itRR=worldModel->begin(); symbol_itRR!=worldModel->end(); symbol_itRR++)
	{
		const AGMModelSymbol::SPtr &symbolRR = *symbol_itRR;
		if (symbolRR->symbolType == "robot")
		{
			for (AGMModelSymbol::iterator edge_itRR=symbolRR->edgesBegin(worldModel); edge_itRR!=symbolRR->edgesEnd(worldModel); edge_itRR++)
			{
				AGMModelEdge edgeRR = *edge_itRR;
				if (edgeRR.linking == "know")
				{
					const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(edgeRR.symbolPair.first);
					if (symbol->symbolType == "object")
					{
						printf("object: %d\n", symbol->identifier);
						for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
						{
							AGMModelEdge edge = *edge_it;
							if (edge.linking == "room")
							{
								const QString polygonString = QString::fromStdString(symbol->getAttribute("polygon"));
								const QStringList coords = polygonString.split(";");
								printf("  it is a room\n");
								qDebug() << " " << coords.size() << " ___ " << polygonString ;
								if (coords.size() < 3)
								{
									qDebug() << coords.size() << " ___ " << polygonString ;
									qDebug() << polygonString;
									for (int32_t i=0; i<coords.size(); i++)
										qDebug() << coords[i];
									qFatal("ABORT %s %d", __FILE__, __LINE__);
								}

								QVector<QPointF> points;
								for (int32_t ci=0; ci<coords.size(); ci++)
								{
									const QString &pointStr = coords[ci];
									if (pointStr.size() < 5) qFatal("%s %d", __FILE__, __LINE__);
									const QStringList coords2 = pointStr.split(",");
									if (coords2.size() < 2) qFatal("%s %d", __FILE__, __LINE__);
									QString a = coords2[0];
									QString b = coords2[1];
									a.remove(0,1);
									b.remove(b.size()-1,1);
									float x = a.toFloat();
									float z = b.toFloat();
									points.push_back(QPointF(x, z));
								}
								if (points.size() < 3) qFatal("%s %d", __FILE__, __LINE__);
								ret[symbol->identifier] = QPolygonF(points);
							}
						}
					}
				}
			}
		}
	}

	return ret;
}

int32_t SpecificWorker::getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel)
{
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				AGMModelEdge edge = *edge_it;
				if (edge.linking == "in")
				{
					return edge.symbolPair.second;
				}
			}
		}
	}

	printf("wheres's the robot?\n");
	return -1;
}

void SpecificWorker::setIdentifierOfRobotsLocation(AGMModel::SPtr &model, int32_t identifier)
{
	bool didSomethin = false;
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{

			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("it was %d\n", edge_it->symbolPair.second);
				}
			}
			for (int32_t edgeIndex=0; edgeIndex<model->numberOfEdges(); edgeIndex++)
			{
				if (model->edges[edgeIndex].linking == "in")
				{
					if (model->edges[edgeIndex].symbolPair.first == symbol->identifier)
					{
						model->edges[edgeIndex].symbolPair.second = identifier;
						didSomethin = true;
					}
				}
			}
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("now is %d\n", edge_it->symbolPair.second);
				}
			}
		}
	}
	if (not didSomethin)
		qFatal("couldn't update robot's room in the cog graph");
}


void SpecificWorker::action_ChangeRoom(bool newAction)
{
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	auto symbols = worldModel->getSymbolsMap(params, "r2");
	
	int32_t roomId = symbols["r2"]->identifier;
	printf("room symbol: %d\n",  roomId);
	std::string imName = symbols["r2"]->getAttribute("imName");
	printf("imName: <%s>\n", imName.c_str());

	const float refX = str2float(symbols["r2"]->getAttribute("x"));
	const float refZ = str2float(symbols["r2"]->getAttribute("z"));
	
	QVec roomPose = innerModel->transformS("world", QVec::vec3(refX, 0, refZ), imName);
	roomPose.print("goal pose");
// 	AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
// 	const float x = str2float(goalRoom->getAttribute("tx"));
// 	const float z = str2float(goalRoom->getAttribute("tz"));
	const float x = roomPose(0);
	const float z = roomPose(2);

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("changeroom to %d\n", symbols["r2"]->identifier);
		go(x, z);
	}
	else
	{
	}
}


void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
{
// 	stop();


	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	AGMModelSymbol::SPtr goalTable;
	AGMModelSymbol::SPtr robot;
	int32_t tableId = -1;
	try
	{
		tableId = str2int(params["container"].value);
		goalTable = worldModel->getSymbol(tableId);
		robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
	}
	catch(...)
	{
		printf("can't access robot or table\n");
		return;
	}

	const float x = str2float(goalTable->getAttribute("tx"));
	const float z = str2float(goalTable->getAttribute("tz"));
	float alpha = tableId==7?-3.141592:0;
// printf("%s: %d\n", __FILE__, __LINE__);

	const float rx = str2float(robot->getAttribute("tx"));
	const float rz = str2float(robot->getAttribute("tz"));
	const float ralpha = str2float(robot->getAttribute("alpha"));
// printf("%s: %d\n", __FILE__, __LINE__);

	// Avoid repeating the same goal and confuse the navigator
	const float errX = abs(rx-x);
	const float errZ = abs(rz-z);
	float errAlpha = abs(ralpha-alpha);
	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
	errAlpha = abs(errAlpha);
	if (errX<20 and errZ<20 and errAlpha<0.02)
		return;
// printf("%s: %d\n", __FILE__, __LINE__);

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("find objects in table %d\n", tableId);
		go(x, tableId==7?z+550:z-550, tableId==7?-3.141592:0, true, 0, 500, 500);
	}
	else
	{
	}
}






/*

void SpecificWorker::action_GraspObject(bool newAction)
{
	std::string state = trajectoryrobot2d_proxy->getState().state;
	printf("action_GraspObject: %s\n", state.c_str());
	if (state != "IDLE")
		trajectoryrobot2d_proxy->stop();
}

*/

void SpecificWorker::action_NoAction(bool newAction)
{
	std::string state = trajectoryrobot2d_proxy->getState().state;
	if (state != "IDLE")
	{
		printf("trajectoryrobot2d state : %s\n", state.c_str());
		stop();
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(20);
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	printf("<<activateAgent\n");
	if (setParametersAndPossibleActivation(prs, activated))
	{
			if (not activated)
			{
				printf("activateAgent 0 >>\n");
				return activate(p);
			}
	}
	else
	{
		printf("activateAgent 1 >>\n");
		return false;
	}
	printf("activateAgent 2 >>\n");
	return true;
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
}

Ice::Int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}


void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
{
	printf("pre <<structuralChange\n");
	QMutexLocker l(mutex);
	printf("<<structuralChange\n");

	AGMModelConverter::fromIceToInternal(modification, worldModel);
	//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
		//roomsPolygons = extractPolygonsFromModel(worldModel);

// 	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "world", true);
	printf("structuralChange>>\n");
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	QMutexLocker l(mutex);

	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "world", true);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker l(mutex);

	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "world", true);
}


void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
// 	agmInner.setWorld(worldModel);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge dst;
		AGMModelConverter::fromIceToInternal(modification,dst);
		agmInner.updateImNodeFromEdge(worldModel, dst, innerModel);
	}
}


/**
 * \brief ACTUALIZACION DEL ENLACE EN INNERMODEL
 */ 
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	QMutexLocker lockIM(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
// 	agmInner.setWorld(worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	agmInner.updateImNodeFromEdge(worldModel, dst, innerModel);
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
		printf("param:%s   value:%s\n", it->first.c_str(), it->second.value.c_str());
		params[it->first] = it->second;
	}
	printf("----\n");

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

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "navigationAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::go(float x, float z, float alpha, bool rot, float xRef, float zRef, float threshold)
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = x;
	tp.z = z;
	tp.y = 0;
	tp.rx = 0;
	tp.ry = 0;
	tp.rz = 0;
	if (rot)
	{
		tp.ry = alpha;
		tp.doRotation = true;
	}
	else
	{
		tp.doRotation = false;
	}
	try
	{
		trajectoryrobot2d_proxy->goReferenced(tp, 80, 350, threshold);
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("something else %d\n", __LINE__);
	}
}


void SpecificWorker::stop()
{
	try
	{
		trajectoryrobot2d_proxy->stop();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("something else %d\n", __LINE__);
	}
}




