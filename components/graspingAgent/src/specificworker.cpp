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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <QMap>



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

	manualMode = false;
	innerViewer = NULL;

#ifdef USE_QTGUI
	osgView = new OsgView(widget);
	show();
// 	printf("%s: %d\n", __FILE__, __LINE__);
	manipulator = new osgGA::TrackballManipulator;
	osgView->setCameraManipulator(manipulator, true);
// 	printf("%s: %d\n", __FILE__, __LINE__);
	show();
	connect(manualButton, SIGNAL(clicked()), this, SLOT(startManualMode()));
// 	connect(buttonLeave,  SIGNAL(clicked()), this, SLOT(leaveObjectSimulation()));
#endif
	

	setRightArmUp_Reflex();
	
	
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
void SpecificWorker::updateViewer()
{
	QTime cc;
	cc = QTime::currentTime();
	QMutexLocker locker(mutex);
#ifdef USE_QTGUI
	if (not innerModel) return;
	if (not innerModel->getNode("root")) return;
// 	printf("root %p\n", innerModel->getNode("root"));

	if (not innerViewer)
	{
		innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
		printf("innerViewer: %p\n", innerViewer);
		innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
	}
// 	printf("(\n");
// 		printf("innerViewer: %p\n", innerViewer);
	innerViewer->update();
// 	printf(")\n");
	osgView->autoResize();
	osgView->frame();
#endif
// 	printf("updateViewer - %d\n", cc.elapsed());
}

void SpecificWorker::compute( )
{
		
	QTime ccc;
	ccc = QTime::currentTime();

	static bool first=true;
	if (first)
	{
		qLog::getInstance()->setProxy("both", logger_proxy);
// 		rDebug2(("graspingAgent started"));
		first = false;
	}

	QMutexLocker locker(mutex);
	{
		if (not manualMode)
		{
// 			printf("Not in manual mode\n");
			if (not innerModel->getNode("shellyArm_grasp_pose"))
			{
				printf("waiting for AGM*\n");
				return;
			}
		}
	}
	

	QTime cc;

	cc = QTime::currentTime();
	manageReachedObjects();
// 	printf("manageReachedObjects - %d\n", cc.elapsed());

	
	cc = QTime::currentTime();
	if (manualMode)
	{
		action_GraspObject();
	}
	else
	{
		// ACTION EXECUTION
		actionExecution();
	}
// 	printf("action - %d\n", cc.elapsed());

#ifdef USE_QTGUI
	updateViewer();
#endif	
// 	printf("compute - %d\n", ccc.elapsed());
}


void SpecificWorker::manageReachedObjects()
{
	float schmittTriggerThreshold = 30;
	float THRESHOLD_mug = 50;
	float THRESHOLD_table = 400;
	std::string m ="  ";

	bool changed = false;
	
	QMutexLocker locker(mutex);
	
	AGMModel::SPtr newModel(new AGMModel(worldModel));


	int robotID = newModel->getIdentifierByType("robot");


	for (AGMModel::iterator symbol_itr=newModel->begin(); symbol_itr!=newModel->end(); symbol_itr++)
	{
		AGMModelSymbol::SPtr node = *symbol_itr;
		if (node->symboltype() == "object")
		{
			// Avoid working with rooms
			if (isObjectType(newModel, node, "room")) continue;
			// Avoid working with tables
			if (isObjectType(newModel, node, "table")) continue;

// 			printf("OBJECT: %d\n", node->identifier);
			//if the object is in robot continue
			try
			{
				AGMModelEdge e = newModel->getEdgeByIdentifiers(node->identifier, robotID, "in");
// 				qDebug()<<"MUG IN ROBOT";
				continue;
			}
			catch (...)
			{
// 				qDebug()<<"MUG NOT IN ROBOT";
			}



			static auto mapt = QMap<int, QTime>();
			bool force_send = false;
			if (not mapt.contains(node->identifier))
			{
				mapt[node->identifier] = QTime::currentTime();
				force_send = true;
			}

			/// Compute distance and new state
			float d2n;
			try
			{
				d2n = distanceToNode("shellyArm_grasp_pose", newModel, node);
			}
			catch(...)
			{
				printf("Ref: shellyArm_grasp_pose: %p\n", (void *)innerModel->getNode("shellyArm_grasp_pose"));
				printf("Obj: %s: %p\n", node->getAttribute("imName").c_str(), (void *)innerModel->getNode(node->getAttribute("imName").c_str()));
				exit(1);
			}
			
			
// 			printf("%d distance %f\n", node->identifier, d2n);
// // // // // // 			innerModel->transformS("robot", "armY").print("armY in r");
// // // // // // 			innerModel->transformS("robot", "armX1").print("armX1 in r");
// // // // // // 			innerModel->transformS("robot", "armX2").print("armX2 in r");
// // // // // // 			innerModel->transformS("robot", "arm_wrist").print("arm_wrist in r");
// // // // // //                         innerModel->transformS("robot", "shellyArm_grasp_pose").print("p in r");
// // // // // //                         innerModel->transformS("robot", "grabPositionHandR").print("G in r");
// 			innerModel->transformS("robot", node->getAttribute("imName")).print("o in r");
// 			innerModel->transformS("shellyArm_grasp_pose", node->getAttribute("imName")).print("o in p");
			QVec oinp = innerModel->transformS("shellyArm_grasp_pose", node->getAttribute("imName"));
// 			fprintf(stderr, "(%f, %f, %f) - %f\n", oinp(0), oinp(1), oinp(2), oinp.norm2());

			if ((force_send or mapt[node->identifier].elapsed() > 500) and (node->identifier == 11))
			{
// 				rDebug2(("%d distance %f") % node->identifier % d2n);
				mapt[node->identifier] = QTime::currentTime();
			}
/*			
			QVec graspPosition = innerModel->transform("room", "shellyArm_grasp_pose");
			graspPosition(1) = 0;
			QVec obj = innerModel->transformS("room", node->getAttribute("imName"));
			obj(1) = 0;
 			graspPosition.print("  no");
 			obj.print("  obj");
// 			printf("%s: %f  (th:%f)\n", node->getAttribute("imName").c_str(), (graspPosition-obj).norm2(), THRESHOLD);
*/
			float THRESHOLD;
			if (isObjectType(newModel, node, "mug"))
			{
				THRESHOLD = THRESHOLD_mug;
			}
			else if (isObjectType(newModel, node, "table"))
			{
				THRESHOLD = THRESHOLD_table;
			}
			else
			{
				qFatal("dededcef or4j ");
			}
			QString name = QString::fromStdString(node->toString());
			if (node->identifier == 11)
			qDebug()<<"Distance To Node (" << node->identifier << ") :"<<name <<" d2n "<<d2n<<"THRESHOLD"<<THRESHOLD;

			for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
			{
				AGMModelEdge &edge = *edge_itr;
				if (edge->getLabel() == "reach" and d2n > THRESHOLD+schmittTriggerThreshold )
				{
					edge->setLabel("noReach");
					printf("object %d STOPS REACH\n", node->identifier);
					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
					changed = true;
					rDebug2(("object %d no-reach") % node->identifier);
				}
				else if (edge->getLabel() == "noReach" and d2n < THRESHOLD/*-schmittTriggerThreshold*/)
				{
					edge->setLabel("reach");
					printf("___ %s ___\n", edge->getLabel().c_str());
					printf("object %d STARTS REACH\n", node->identifier);
					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
					changed = true;
					rDebug2(("object %d reach") % node->identifier);
				}
			}
		}
	}

	/// Publish new model if changed
	if (changed)
	{
		printf("PUBLISH!!!! version%d\n", newModel->version);
		sendModificationProposal(newModel, worldModel, m);
	}
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

// std::vector<std::pair<float, float>> getCoordinates

float SpecificWorker::distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr node)
{
	QMutexLocker locker(mutex);
	
	// check if it's a polygon
// 	bool isPolygon = false;
// 	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model) and isPolygon == false; edge_itr++)
// 	{
// 		if ((*edge_itr)->getLabel() == "table")
// 			isPolygon = true;
// 	}

// 	if (isPolygon)
// 	{
// 		const std::string polygon = node->getAttribute("polygon");
// 		const QVec head_in_floor = innerModel->transform("room", reference_name.c_str());
// 		return distanceToPolygon(head_in_floor, QVec::vec3(x, y, z), polygon);
// 	}
// 	else
// 	{
		QVec arm = innerModel->transformS("world", reference_name);
		arm(1) = 0;
		QVec obj = innerModel->transformS("world", node->getAttribute("imName"));
		obj(1) = 0;
		return (arm-obj).norm2();
// 	}
}

// float SpecificWorker::distanceToPolygon(QVec reference, QVec position, std::string polygon_str)
// {
// 	boost::geometry::model::d2::point_xy<int> point(reference(0), reference(2));
// 	boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float> > poly;
// 
// // 	printf("p %s\n", polygon_str.c_str());
// 	std::vector<std::string> strs;
// 	boost::split(strs, polygon_str, boost::is_any_of(";"));
// // 	printf("d %f  %f\n", position(0);
// 	for (auto coords : strs)
// 	{
// // 		printf("pp %s\n", coords.c_str());
// 		std::vector<std::string> strs_coords;
// 		boost::split(strs_coords, coords, boost::is_any_of("(),"));
// 		if (strs_coords.size()<2)
// 			return std::nan("1");
// // 		for (auto ss : strs_coords) printf("<%d %s\n", ddd++, ss.c_str());
// // 		printf(" s %d\n", strs_coords.size());
// 		const float x = atof(strs_coords[1].c_str());
// 		const float z = atof(strs_coords[2].c_str());
// 		printf("< %f %f\n", x, z);
// 		boost::geometry::model::d2::point_xy<float> vertex(x, z);
// 		boost::geometry::append(poly, vertex);
// 	}
// 
// 
// 	return boost::geometry::distance(poly, point);
// }

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QTime cc;
	cc = QTime::currentTime();
	QMutexLocker locker(mutex);

	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	timer.start(20);
	printf("setParams: %d\n", cc.elapsed());
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	QMutexLocker locker(mutex);
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

bool SpecificWorker::deactivateAgent()
{
	QMutexLocker locker(mutex);
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	QMutexLocker locker(mutex);
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
	QMutexLocker locker(mutex);
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

void SpecificWorker::changeInner ()
{
	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);				
	}

	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
{
	QTime cc;
	cc = QTime::currentTime();
	QMutexLocker locker(mutex);

	AGMModelConverter::fromIceToInternal(modification, worldModel);
	
	if (innerModel) delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel, "world");
	changeInner();
	printf("structuralChange %d\n", cc.elapsed());
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

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
// 		AGMModelEdge dst;
// 		AGMModelConverter::fromIceToInternal(modification,dst);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	QMutexLocker locker(mutex);

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
		backAction = action;
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

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel, string m)
{
	QMutexLocker locker(mutex);

	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "graspingAgent")+m);
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

void SpecificWorker::actionExecution()
{
	QMutexLocker locker(mutex);

	static std::string previousAction = "";
	static QTime actionTime = QTime::currentTime();
	bool newAction = (previousAction != action);

	qDebug()<<"---------------------------------";
	cout<<action<<endl;
	qDebug()<<"---------------------------------";

	if (newAction)
	{
		actionTime = QTime::currentTime();
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());
		ui_actionName->setText(QString::fromStdString(action));
// 		rDebug2(("action %s") % action.c_str() );
	}

	ui_actionTime->setText(QString::number(actionTime.elapsed()));


	if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}
	else if (action == "setobjectreach")
	{
		action_SetObjectReach(newAction);
	}
	else if (action == "graspobject")
	{
		if (not robotIsMoving())
		{
 			rDebug2(("graspingAgent would now start grasping - robot stopped and got 'graspobject'"));
 			action_GraspObject(newAction);
		}
	}
	else if (action == "setrestarmposition")
	{
		action_SetRestArmPosition();
	}

	if (newAction)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
}

// Check if robot is stopped befora calling to graspAction
bool SpecificWorker::robotIsMoving()
{
	int MOVEMENT_THRESHOLD = 5; // in mm
	int robotID = worldModel->getIdentifierByType("robot");
	int robotMovement = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("movedInLastSecond")).toInt();
	printf("Last second robot movement %i\n", robotMovement);
	if (robotMovement > MOVEMENT_THRESHOLD )
                return true;
	return false;
}


void SpecificWorker::directGazeTowards(AGMModelSymbol::SPtr symbol)
{
	QMutexLocker locker(mutex);
	try
	{
		const float x = str2float(symbol->getAttribute("tx"));
		const float y = str2float(symbol->getAttribute("ty"));
		const float z = str2float(symbol->getAttribute("tz"));
		QVec worldRef = QVec::vec3(x, y, z);
		QVec robotRef = innerModel->transform("robot", worldRef, "world");
		printf("saccadic3D\n");
		printf("\n");
		saccadic3D(robotRef, QVec::vec3(0,0,1));
	}
	catch(...)
	{
		printf("directGazeTowards\n");
		throw;
	}
}


void SpecificWorker::action_FindObjectVisuallyInTable(bool first)
{
	qFatal("ddd");
	try
	{
		int32_t tableId = str2int(params["container"].value);
		QMutexLocker locker(mutex);
		directGazeTowards(worldModel->getSymbol(tableId));
	}
	catch(...)
	{
		printf("Can't get the symbol for the container (table)\n[%s: %d]\n", __FILE__, __LINE__);
		throw;
	}
}


void SpecificWorker::startManualMode()
{
	manualMode = true;
	action_GraspObject(true);
}


void SpecificWorker::action_GraspObject(bool first)
{
	static int32_t state = 0;
	static QTime time;

	QMutexLocker locker(mutex);
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	TargetState ikState;
	static int lastTargetId = 0;

	static QTime ttt = QTime::currentTime();
	printf("elapsed: %f\n", (float)ttt.elapsed());

	if (first) state = 0;
	printf("action_GraspObject: first:%d  state=%d\n", (int)first, state);


	static QTime stateTime = QTime::currentTime();
	ui_state->setText(QString::number(state));
	ui_stateTime->setText(QString::number(stateTime.elapsed()));


	auto targetState = inversekinematics_proxy->getTargetState("ARM", lastTargetId);
	if (targetState.finish)
		ui_IKFinished->setChecked(true);
	else
		ui_IKFinished->setChecked(false);
	
	bool someMotorMoving = isSomeMotorMoving();
	if (someMotorMoving)
		ui_motorsMoving->setChecked(true);
	else
		ui_motorsMoving->setChecked(false);
	
	QVec pose = QVec::vec6();
	if (not manualMode)
	{
		try
		{
			symbols = newModel->getSymbolsMap(params, "object", "room", "robot", "table");
		}
		catch(...)
		{
			printf("graspingAgent: Couldn't retrieve action's parameters\n");
		}
	}
	else
	{
#ifdef USE_QTGUI
		pose(0) = xspin->value();
		pose(1) = yspin->value();
		pose(2) = zspin->value();
		pose(3) = 0;
		pose(4) = 0;
		pose(5) = 0;
#else
		qFatal("this shouldn't happen %d\n", __LINE__);
#endif
	}

	
	
// 	const float steps_to_grasp = 1;
	const float yInit = 40;
	const float yGoal = -40;
	const float zInit = -180;
	const float zGoal = -130;


	static QVec offset = QVec::vec3(0,0,0);
	static QVec offsetR = QVec::vec3(0,0,0);
	offset.print("offset");
	switch (state)
	{
		//
		// APPROACH 1 AND OPEN FINGERS
		//
		case 0:
			printf("%d\n", __LINE__);
			try
			{
				inversekinematics_proxy->setJoint("gripperFinger1", -0.3, 0.5);
				inversekinematics_proxy->setJoint("gripperFinger2", 0.3, 0.5);
				inversekinematics_proxy->setJoint("head_pitch_joint", 1., 0.5);
			}
			catch(...) { qFatal("%s: %d\n", __FILE__, __LINE__); }
			offset = QVec::vec3(0, yInit, zInit);
			if (manualMode)
			{
				for (int cc=0; cc<3; cc++) pose(cc) += offset(cc);
				lastTargetId = sendRightArmToPose(pose);
			}
			else
			{
				offsetR = QVec::vec3(-0.15, 0, 0);
				lastTargetId = sendHandToSymbol(symbols["object"],  offset, symbols, offsetR);
			}
			state = 1;
			break;
		//
		// Wait for last movement (it can be the first one or a intermediate one)
		//
		case 1:
			printf("%d\n", __LINE__);
			ikState = inversekinematics_proxy->getTargetState("ARM", lastTargetId);
			if (ikState.finish)
			{
				printf("ik finished! te:%f re:%f\n", ikState.errorT, ikState.errorR);
// 				if (ikState.errorT < 40 and ikState.errorR < 0.5)
				{
					printf("next state!\n");
					if ((offset - QVec::vec3(0, yGoal, zGoal)).norm2() > 10)
					{
						state = 2;
						stateTime = QTime::currentTime();
					}
					else
					{
						state = 3;
						stateTime = QTime::currentTime();
					}
				}
			}
			else
			{
				printf("not finished yet\n");
			}
			break;
		//
		// APPROACH Middle
		//
		case 2:
			offset(0) = 0;
			offset(1) = yGoal;
			offset(2) = zGoal;
			if (manualMode)
			{
				for (int cc=0; cc<3; cc++) pose(cc) += offset(cc);
				lastTargetId = sendRightArmToPose(pose);
			}
			else
			{
					offsetR = QVec::vec3(-0.15, 0, 0);
// 				}
// 				else
// 				{
					offsetR = QVec::vec3(0, 0, 0);
// 				}
				offset.print("offset 2");
				offsetR.print("offsetR 2");
				lastTargetId = sendHandToSymbol(symbols["object"], offset, symbols, offsetR);
			}
			state = 1; // Go back to wait state
			stateTime = QTime::currentTime();
			break;
		case 3:
			try
			{
				inversekinematics_proxy->setJoint("gripperFinger1",  0.89, 1.5);
				inversekinematics_proxy->setJoint("gripperFinger2", -0.89, 1.5);
// 				inversekinematics_proxy->setJoint("wristX", jointmotor_proxy->getMotorState("wristX").pos-0.15, 1.5);
				usleep(600000);
			}
			catch(...)
			{
				qFatal("%s: %d\n", __FILE__, __LINE__);
			}
			state = 4;
			stateTime = QTime::currentTime();
			break;
		case 4:
			try
			{
				usleep(200000);
				if (not manualMode)
				{
					newModel->removeEdge(symbols["object"], symbols["table"], "in");
					newModel->addEdge(   symbols["object"], symbols["robot"], "in");
					{
						QMutexLocker locker(mutex);
// 						rDebug2(("graspingAgent object %d grasped!") % symbols["object"]->identifier);
						sendModificationProposal(newModel, worldModel);
					}
				}
				offset(1)+=80;
				if (manualMode)
				{
					for (int cc=0; cc<3; cc++) pose(cc) += offset(cc);
					lastTargetId = sendRightArmToPose(pose);
				}
				else
				{
					offsetR = QVec::vec3(-0.09, 0, 0);
					lastTargetId = sendHandToSymbol(symbols["object"], offset, symbols, offsetR);
				}
				state++;
			}
			catch(...)
			{
				qFatal("graspingAgent: Couldn't publish new model\n");
			}
			state = 9999;
			stateTime = QTime::currentTime();
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		default:
			break;
	}

	usleep(200000);
}

void SpecificWorker::action_SetRestArmPosition(bool first)
{
	static int32_t state = 0;
	static QTime time;

	QMutexLocker locker(mutex);
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	TargetState ikState;

	static QTime ttt = QTime::currentTime();
	printf("elapsed: %f\n", (float)ttt.elapsed());

	if (first) state = 0;
	printf("action_SetObjectReach: first:%d  state=%d\n", (int)first, state);

	try
	{
		symbols = newModel->getSymbolsMap(params, "robot", "status");
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}

	RoboCompJointMotor::MotorGoalPosition goal;
	switch (state)
	{
	case 0: // SetRestPosition
		goal.maxSpeed = 0.7;
		goal.name = "armY";
		goal.position = 0;
		jointmotor_proxy->setPosition(goal);
		goal.name = "armX1";
		goal.position = -1.0;
		jointmotor_proxy->setPosition(goal);
		goal.name = "armX2";
		goal.position = 2.5;
		jointmotor_proxy->setPosition(goal);
		goal.name = "wristX";
		goal.position = 0;
		jointmotor_proxy->setPosition(goal);
		state = 1;
		break;
	case 1: // Wait for movement to finish
printf("%s: %d\n", __FUNCTION__, __LINE__);
		sleep(2);
printf("%s: %d\n", __FUNCTION__, __LINE__);
		newModel->addEdge(symbols["robot"], symbols["status"], "rest");
printf("%s: %d\n", __FUNCTION__, __LINE__);
		sendModificationProposal(newModel, worldModel);
printf("%s: %d\n", __FUNCTION__, __LINE__);
		state = 0;
		break;
	////////////////////////////////////////////////////////////////////////////////////////////
	default:
		qFatal("internal error: non-controlled case");
		break;
	}

	usleep(200000);
}

void SpecificWorker::leaveObjectSimulation()
{
	QMutexLocker locker(mutex);

	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		newModel->addEdge(   symbols["object"], symbols["table"], "in");
		newModel->removeEdge(symbols["object"], symbols["robot"], "in");
		{
			QMutexLocker locker(mutex);
// 			rDebug2(("graspingAgent object %d left") % symbols["object"]->identifier);
			sendModificationProposal(newModel, worldModel);
		}
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't publish new model\n");
		qFatal("2264w7ertytynvc8");
	}

}


int32_t SpecificWorker::sendHandToSymbol(AGMModelSymbol::SPtr symbol, QVec offset, std::map<std::string, AGMModelSymbol::SPtr> symbols, QVec offsetR)
{
	int32_t lastTargetId_local;
	QVec objectsLocationInRobot;
	// approach the hand
	try
	{
		objectsLocationInRobot = getObjectsLocationInRobot(symbols, symbol); //POSE OBJECT IN ROBOT SYSTEM
// 		objectsLocationInRobot.print("objectsLocationInRobot");
// 		innerModel->transformS("robot", QVec::vec3(0,0,0), symbol->getAttribute("imName")).print("directo");
	}
	catch (...) { printf("%s: %d\n", __FILE__, __LINE__); }

// 	objectsLocationInRobot.print("objectsLocationInRobot");
	// add offset and put rotation
	for (int i=0; i<3; i++) objectsLocationInRobot(i)  += offset(i);
	objectsLocationInRobot(3) = offsetR(0);
	objectsLocationInRobot(4) = offsetR(1);
	objectsLocationInRobot(5) = offsetR(2);
// 	objectsLocationInRobot.print("objectsLocationInRobot + offset");
	try
	{
		lastTargetId_local = sendRightArmToPose(objectsLocationInRobot);
// 		objectsLocationInRobot.print("sent");
// 		qDebug()<<"------> 0 step execution";
	}
	catch (...)
	{
		lastTargetId_local = -1;
		printf("%s: %d\n", __FILE__, __LINE__);
		qFatal("dewer");
	}
	return lastTargetId_local;
}



/**
 * \brief This method calculates the pose of the symbol OBJECT into the robot reference system.
 * @param symbols the symbols of AGM model
 * @param object the goal object
 * @return POSE 6D
 */
QVec SpecificWorker::getObjectsLocationInRobot(std::map<std::string, AGMModelSymbol::SPtr> &symbols, AGMModelSymbol::SPtr &object)
{
	QMutexLocker locker(mutex);

	// Get target
	int robotID, objectID;
	robotID = symbols["robot"]->identifier;
	objectID = symbols["object"]->identifier;

	QString  robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
	QString  objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));

	return innerModel->transform6D(robotIMID, objectIMID);
}

QVec SpecificWorker::fromRobotToRoom(std::map<std::string, AGMModelSymbol::SPtr> &symbols, const QVec vector)
{
	QMutexLocker locker(mutex);

	// Get target
	int roomID, robotID;
	roomID = symbols["room"]->identifier;
	robotID = symbols["robot"]->identifier;

	QString  robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
	QString  roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));

	qDebug() << roomIMID << "  " << robotIMID;

// 	innerModel->getTransformationMatrix(roomIMID, robotIMID).print("robot to room");



// 	innerModel->getTransformationMatrix(roomIMID, robotIMID).extractAnglesR_min().print("angles");

	return innerModel->transform6D(roomIMID, vector, robotIMID);
}

int SpecificWorker::sendRightArmToPose(QVec targetPose)
{
	Pose6D target;
	WeightVector weights;
	try
	{
		target.x = targetPose.x();
		target.y = targetPose.y();
		target.z = targetPose.z();
		weights.x = 1;
		weights.y = 1;
		weights.z = 1;
		target.rx = targetPose.rx();
		target.ry = targetPose.ry();
		target.rz = targetPose.rz();
		weights.rx = 1;
		weights.ry = 1;
		weights.rz = 1;
	}
	catch (...)
	{
		printf("graspingAgent: Error reading data from cognitive model: (%s:%d)\n", __FILE__, __LINE__);
	}

	return inversekinematics_proxy->setTargetPose6D("ARM", target, weights);
}

void SpecificWorker::action_SetObjectReach(bool first)
{
	QMutexLocker locker(mutex);
	printf("void SpecificWorker::action_SetObjectReach()\n");

	///
	///  Lift the hand if it's down, to avoid collisions
	///
	float grasp_height = innerModel->transform("world", "shellyArm_grasp_pose")(1);
	printf("grasp_height %f\n", grasp_height);
	float elbow_height = innerModel->transform("world", "armX2")(1);
	printf("elbow_height %f\n", elbow_height);
	if (first)
	{
		inversekinematics_proxy->setJoint("head_yaw_joint", 0, 0.5);
		backAction = action;
		setRightArmUp_Reflex();
	}





	///
	/// Track the target
	///
	int32_t objectId = -1;
	try
	{
		objectId = str2int(params["object"].value);
	}
	catch (...)
	{
		printf("%s %d\n", __FILE__, __LINE__);
	}
	if (objectId > 0)
	{
		try
		{
			AGMModelSymbol::SPtr goalObject = worldModel->getSymbol(objectId);
			// Get object's relative position from the robot's perspective
			QVec poseRRobot = innerModel->transformS("robot", goalObject->getAttribute("imName"));
			float angleRRobot = atan2(poseRRobot.x(), poseRRobot.z());
			printf("angulo relativa robot %f\n", angleRRobot);
			// Get object's relative position from the yaw's perspective
			QVec poseRYaw = innerModel->transformS("head_yaw_joint", goalObject->getAttribute("imName"));
// 			poseRYaw.print("relativo al yaw");
			float angleRYaw = atan2(poseRYaw.x(), poseRYaw.z());
			printf("angulo relativa a la camara %f\n", angleRYaw);
			// Compute current head's yaw
			float currentYaw = angleRRobot - angleRYaw;
			printf("current yaw: %f\n", currentYaw);
			float angle = 0.5*angleRRobot + 0.5*currentYaw;
			printf("%f -> ", angle);
			if (fabs(angle-currentYaw) > 10.*M_PI/180.)
			{
				printf(" ** ");
				if (angle>currentYaw)
					angle = currentYaw + 10.*M_PI/180.;
				else
					angle = currentYaw - 10.*M_PI/180.;
			}
			printf(" -> %f\n", angle);

			if (angle > +.4) angle = +.4;
			if (angle < -.4) angle = -.4;

			// In the meantime we just move the head downwards:
			inversekinematics_proxy->setJoint("head_pitch_joint", 0.8, 0.5);
			printf("Mandamos angulo %f\n", angle);
			inversekinematics_proxy->setJoint("head_yaw_joint", angle, 0.5);
// // // // // // // 			saccadic3D(QVec::vec3(x,y,z), QVec::vec3(0,0,1));
		}
		catch (...)
		{
			printf("%s %d\n", __FILE__, __LINE__);
			qFatal("d");
		}
	}
	else
	{
		printf("don't have the object to reach in my model %d\n", objectId);
	}

	printf("--------------------\n");

	///
	/// No more work to do. The label is set passively (from this agent's point of view)
	///
}


void SpecificWorker::saccadic3D(QVec point, QVec axis)
{
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
	QVec rel = innerModel->transform("rgbd", QVec::vec3(tx, ty, tz), "world");
// 	rel.print("desde la camara");

	float errYaw   = -atan2(rel(0), rel(2));
	float errPitch = +atan2(rel(1), rel(2));

	RoboCompJointMotor::MotorGoalPosition goal;

	goal.name = "head_yaw_joint";
	goal.maxSpeed = 0.5;
	goal.position = jointmotor_proxy->getMotorState("head_yaw_joint").pos - errYaw;
	jointmotor_proxy->setPosition(goal);

	goal.name = "head_pitch_joint";
	goal.maxSpeed = 0.5;
	goal.position = jointmotor_proxy->getMotorState("head_pitch_joint").pos - errPitch;
	jointmotor_proxy->setPosition(goal);

/*
	RoboCompInverseKinematics::Pose6D targetSight;
	targetSight.x = tx;
	targetSight.y = ty;
	targetSight.z = tz;
	RoboCompInverseKinematics::Axis axSight;
	axSight.x = axx;
	axSight.y = axy;
	axSight.z = axz;
	bool axisConstraint = false;
	float axisAngleConstraint = 0;
	try
	{
		inversekinematics_proxy->stop("HEAD");
		usleep(500000);
		inversekinematics_proxy->pointAxisTowardsTarget("HEAD", targetSight, axSight, axisConstraint, axisAngleConstraint);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
*/

}




void SpecificWorker::setRightArmUp_Reflex()
{
//     printf("not reflex\n");
//     return;
	printf("setRightArmUp_Reflex\n");
        
	MotorGoalPositionList gpList;
	MotorGoalPosition gp;
	gp.maxSpeed = 0.7;
	
        
        float desired_value;
        MotorStateMap mstateMap;
        jointmotor_proxy->getAllMotorState(mstateMap);
        
        
	gp.name = "armY";
        desired_value = 0;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
	gp.name = "armX1";
	desired_value = -1;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
        gp.name ="head_yaw_joint";
        desired_value = 0;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
        gp.name ="head_pitch_joint";
        desired_value = 0.8;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
	gp.name = "armX2";
	desired_value = 2.5;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
	gp.name = "wristX";
	desired_value = 0;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
	gp.name = "gripperFinger1";
	desired_value = 0.2;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }
	
	gp.name = "gripperFinger2";
	desired_value = -0.2;
        if (fabs(mstateMap[gp.name].p-desired_value)>=0.05)
        {
            printf("%s   %f   %f\n, ", gp.name.c_str(), (float)mstateMap[gp.name].p, desired_value);
            gp.position = desired_value;
            gpList.push_back(gp);
        }

        if (gpList.size()>0)
            jointmotor_proxy->setSyncPosition(gpList);
}



void SpecificWorker::on_state1_clicked()
{
	action = "graspobject";
	action_GraspObject(true);

	params["object"].value = "11";
	params["room"  ].value = "7";
	params["robot" ].value = "1";
}


bool SpecificWorker::isSomeMotorMoving()
{
	MotorStateMap allMotorsCurr;
	try
	{
		jointmotor_proxy->getAllMotorState(allMotorsCurr);
	}
	catch(...)
	{
		std::cout<<"Error retrieving all motors state\n";
	}
	for (auto v : allMotorsCurr)
	{
		if (v.second.isMoving)
		{
			return true;
		}
	}

	return false;	
}
