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

SpecificWorker::SpecificWorker(MapPrx &mprx, QWidget *parent) : GenericWorker(mprx)
{
	tState.setState("IDLE");
	hide();
	relojForInputRateControl.start();
}

SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	this->params = params;
	try
	{
		innerModel = new InnerModel(params.at("InnerModel").value);
		#ifdef USE_QTGUI
			viewer = new InnerViewer(innerModel);  //makes a copy of innermodel for internal use
	  #endif
	}
	catch (std::exception e)
	{
		qFatal("Aborting. Error reading config params");		//WE COULD THROW HERE AND HAVA A NICE EXIT FROM MAIN
	}

	//////////////////////////////
	//Initial update InnerModel from robot
	//////////////////////////////
	innerModel->newTransform("virtualRobot", "static", innerModel->getNode("robot"));
	updateInnerModel(innerModel, tState);

	//////////////////////////////////////
	/// Initialize sampler of free space
	/////////////////////////////////////
	sampler.initialize(innerModel, params);

	///////////////////
	//Planner
	///////////////////
	plannerPRM.initialize(&sampler, params);  
	
	///////////////////
	//Initializes the elastic band structure "road"
	///////////////////
	road.initialize(innerModel, params);

	///////////////////
	//This object creates and maintains the road (elastic band) adapting it to the real world using a laser device
	///////////////////
	elasticband.initialize( params);

	//////////////////////////////////////////////////////////////////////////
	//Low level controller that drives the robot on the road
	//by computing VAdv and VRot from the relative position wrt to the local road
	/////////////////////////////////////////////////////////////////////////////////
	controller = new Controller(innerModel, laserData, params, 2 );

	qDebug() << __FUNCTION__ << "All objects initialized";
	
#ifdef USE_QTGUI
	graphdraw.draw(plannerPRM, viewer);
	viewer->start();	
#endif
	
	Period = 100;
	timer.start(Period);
	return true;
};


/**
 * @brief Main execution loop. Checks if there is an active target
 * @return void
 */
void SpecificWorker::compute()
{
	static QTime reloj = QTime::currentTime();

	// Check for connection failure
	if (updateInnerModel(innerModel, tState) == false)
	{
		controller->stopTheRobot(omnirobot_proxy);
		stopCommand(currentTarget, road, tState);
		tState.setState("DISCONNECTED");
		currentTarget.state = CurrentTarget::State::DISCONNECTED;
	}
		
	switch (currentTarget.state)
	{
		case CurrentTarget::State::STOP:
			stopCommand(currentTarget, road, tState);
			break;
		case CurrentTarget::State::CHANGETARGET:
			currentTarget.setState(CurrentTarget::State::GOTO);
			break;
		case CurrentTarget::State::GOTO:
			timer.setInterval(Period);
			//qDebug() << __FUNCTION__ << "GOTO:" << "Robot at:" << innerModel->transform6D("world", "robot") << "Target:" << currentTarget.getFullPose();
			gotoCommand(innerModel, currentTarget, tState, road, laserData);
			break;
		case CurrentTarget::State::SETHEADING:
			setHeadingCommand(innerModel, currentTarget.getRotation().y(), currentTarget, tState, road);
			break;		
		case CurrentTarget::State::BLOCKED:
				road.update();
				elasticband.update(innerModel, road, laserData, currentTarget);
				if( road.isBlocked() == false)
					currentTarget.setState(CurrentTarget::State::GOTO);
				else
					qDebug() << __FUNCTION__ << "Blocked";
			break;
		case CurrentTarget::State::GOBACKWARDS:
			goBackwardsCommand(innerModel, currentTargetBack, currentTarget, tState, road);
			break;
		case CurrentTarget::State::DISCONNECTED:
			timer.setInterval(2000);
			break;
		case CurrentTarget::State::ROBOT_COLLISION:			// Check if robot is inside limits and not in collision.
			
			if (std::get<bool>(sampler.checkRobotValidStateAtTarget(innerModel->transform6D("world", "robot"))))
			{
				tState.setState("IDLE"); tState.setDescription("");
				currentTarget.setState(CurrentTarget::State::IDLE);
			}
			break;
		case CurrentTarget::State::TARGET_COLLISION:			// Check if robot is inside limits and not in collision.
			timer.setInterval(300);
			if (std::get<bool>(sampler.checkRobotValidStateAtTarget(currentTarget.getFullPose())))
			{
				tState.setState("IDLE"); tState.setDescription("");
				currentTarget.setState(CurrentTarget::State::IDLE);
			}
			break;
		case CurrentTarget::State::LEARNING:	
			if(plannerPRM.learnForAWhile(150, false)) //150, Max number of nodes in the graph. 
			{
				#ifdef USE_QTGUI
					//plannerPRM.drawGraph(viewer->innerViewer);  
				#endif
			}
			currentTarget.setState(CurrentTarget::State::IDLE);
			break;
		case CurrentTarget::State::IDLE:
			timer.setInterval(700);
			qDebug() << __FUNCTION__ << "Computed period" << reloj.restart()  << "ms. State. Robot at:" << innerModel->transform6D("world", "robot");
			//currentTarget.setState(CurrentTarget::State::LEARNING);
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Stops the robot
 *
 * @return bool
 */
bool SpecificWorker::stopCommand(CurrentTarget &target, WayPoints &myRoad, TrajectoryState &state)
{
	controller->stopTheRobot(omnirobot_proxy);
	myRoad.setFinished(true);
	myRoad.reset();
	myRoad.endRoad();
#ifdef USE_QTGUI
	//myRoad.clearDraw(viewer->innerViewer);
	//drawGreenBoxOnTarget(target.getTranslation());
#endif
	target.reset();
	state.setElapsedTime(taskReloj.elapsed());
	state.setState("IDLE");
	qDebug() << __FUNCTION__ << "Robot at pose:" << innerModel->transform6D("world", "robot");
	return true;
}

/**
 * @brief Changes current target without interrupting the ongoing process. Introduced to support visual-servoing
 *
 * @param innerModel
 * @return bool
 */
bool SpecificWorker::changeTargetCommand(InnerModel *innerModel, CurrentTarget &target, TrajectoryState &state,
                                         WayPoints &myRoad)
{
	//DEPRECATED
	qDebug() << __FUNCTION__ << "DEPRECATED";
	return false;
}

/**
 * @brief Sends the robot to the target in currentTarget variable.
 * Calls in order
 * 		planner (if currentTarget has not a plan)
 * 			elasticband (to project the road against the laserData)
 * 				road (to  compute all parameters relating the robot to the robot )
 * 					controller (to compute control outputs for the robot to follow the road)
 *
 *  The insert-delete operators should be put here
 *  A new function could be inserted to check model validity: i.e. real laser profile against synthetic laser profile
 *  If a big difference is detected, it will be whether additive or substractive.
 *  If additive a new object can be inserted into innerModel and IMV,
 *  then the difference should be cancelled and currentTarget.isWithoutPlan can be set to True for replanning
 *  If substractive the innerModel object should be removed bacause someone already removed it in IMV (or the world),
 *  no replanning es necessary here.
 *
 * @param innerModel ...
 * @return bool
 */

bool
SpecificWorker::gotoCommand(InnerModel *innerModel, CurrentTarget &target, TrajectoryState &state, WayPoints &myRoad,
                            RoboCompLaser::TLaserData &lData)
{
	QTime reloj = QTime::currentTime();

	/////////////////////////////////////////////////////
	// check for ending conditions
	//////////////////////////////////////////////////////
	if (myRoad.isFinished() == true)
	{
		controller->stopTheRobot(omnirobot_proxy);
		qDebug() << __FUNCTION__ << "Changing to SETHEADING command";
		target.setState(CurrentTarget::State::SETHEADING);
		return true;
	}
 	if (myRoad.isBlocked() == true)		//Road BLOCKED, go to BLOCKED state and wait it the obstacle moves
 	{
		controller->stopTheRobot(omnirobot_proxy);
 		//currentTargetBack.setTranslation(innerModel->transform("world", QVec::vec3(0, 0, -250), "robot"));
 		target.setState(CurrentTarget::State::BLOCKED);
		return false;
 	}

	// Get here when robot is stuck
// 	if(myRoad.requiresReplanning == true)
// 	{
// 	 		//qDebug() << __FUNCTION__ << "STUCK, PLANNING REQUIRED";
// 	 		//computePlan(innerModel);
// 	}
	
	//////////////////////////////////////////
	// Check if there is a plan for the target
	//////////////////////////////////////////
	bool coolPlan = true;
	if (target.isWithoutPlan() == true)
	{
		state.setState("PLANNING");
		QVec localT = target.getTranslation();
		coolPlan = plannerPRM.computePath(localT, innerModel);
		if (coolPlan == false)
		{
			qDebug() << __FUNCTION__ << "Path NOT found. Resetting to IDLE state";
			target.setState(CurrentTarget::State::STOP);
			return false;
		}
		target.setTranslation(localT);
		//qDebug() << __FUNCTION__ << "Plan obtained of " << planner->getPath().size() << " points";
		// take inner to current values
		updateInnerModel(innerModel, state);
		target.setWithoutPlan(false);

		//Init road   REMOVE TRASH FROM HERE
		myRoad.reset();
		myRoad.readRoadFromList(plannerPRM.getPath());
		myRoad.requiresReplanning = false;
		myRoad.computeDistancesToNext();
		myRoad.startRoad();
		state.setPlanningTime(reloj.elapsed());
		state.setState("EXECUTING");
	}

	///////////////////////////////////
	// Update the band
	/////////////////////////////////
	elasticband.update(innerModel, myRoad, laserData, target);

	///////////////////////////////////
	// compute all measures relating the robot to the road
	/////////////////////////////////
	myRoad.update();

	//myRoad.printRobotState(innerModel, target);

	/////////////////////////////////////////////////////
	//move the robot according to the current force field
	//////////////////////////////////////////////////////
	controller->update(innerModel, lData, omnirobot_proxy, myRoad);

	
	#ifdef USE_QTGUI
		waypointsdraw.draw(myRoad, viewer,  target);
	#endif
	
	state.setEstimatedTime(myRoad.getETA());
	return true;
}

/**
 * @brief Turns the robot until it reaches the desired orientation.
 *
 * @param innerModel ...
 * @param alfa angle between robot's z axis and worlds Z axis
 * @return bool Not used
 */
bool
SpecificWorker::setHeadingCommand(InnerModel *innerModel, float alfa, CurrentTarget &target, TrajectoryState &state,
                                  WayPoints &myRoad)
{
	///////////////////////////////////
	/// Check if target has rotation
	///////////////////////////////////
	if (target.hasRotation() == false)
	{
		target.setState(CurrentTarget::State::STOP);
		return true;
	}

	const float MAX_ORIENTATION_ERROR = 0.04;

	float angRobot = innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	alfa = angmMPI(alfa);
	float error = angmMPI(angRobot - alfa);
	state.setState("EXECUTING-TURNING");
	
	/////////////////////////////
	/// Check for minimun admisible orientation error
	//////////////////////////////
	if (fabs(error) < MAX_ORIENTATION_ERROR)
	{
		target.setState(CurrentTarget::State::STOP);
		return true;
	}

	/////////////////////////////
	/// Now let's move the baby
	//////////////////////////////
	float vrot = -0.7 * error;  //Proportional controller
	if (fabs(vrot) > 0.3)    // hard limit
	{
		if (vrot > 0) vrot = 0.3;
		else vrot = -0.3;
	}
	try
	{
		omnirobot_proxy->setSpeedBase(0, 0, vrot);
		return false;
	} catch (const Ice::Exception &ex)
	{
		std::cout << ex.what() << std::endl;
		return false;
	}
}

/** REVISARRRR
 * @brief Sends the robot bakcwards on a straight line until targetT is reached.
 *
 * @param innerModel ...
 * @param target position in World Reference System
 * @return bool
 */
bool SpecificWorker::goBackwardsCommand(InnerModel *innerModel, CurrentTarget &current, CurrentTarget &currentT, TrajectoryState &state, WayPoints &myRoad)
{
	const float MAX_ADV_SPEED = 400.f;
	const float MAX_POSITIONING_ERROR = 40;  //mm
	static float errorAnt = std::numeric_limits<float>::max();
	
	///////////////////
	//CHECK PARAMETERS
	///////////////////
	QVec target = current.getTranslation();
	if (target.size() < 3 or std::isnan(target.x()) or std::isnan(target.y()) or std::isnan(target.z()))
	{
		qDebug() << __FUNCTION__ << "Returning. Invalid target";
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Returning. Invalid target";
		throw ex;
		return false;
	}
	
	state.setState("BACKWARDS");
	QVec rPose = innerModel->transform("world", "robot");
	float error = (rPose - target).norm2();
	bool errorIncreasing = false;
	if (error > errorAnt)
		errorIncreasing = true;
	qDebug() << __FUNCTION__ << "doing backwards" << error;

	if ((error < MAX_POSITIONING_ERROR) or (errorIncreasing == true))        //TASK IS FINISHED
	{
		controller->stopTheRobot(omnirobot_proxy);
		myRoad.requiresReplanning = true;
		currentT.setWithoutPlan(true);
		currentT.setState(CurrentTarget::State::GOTO);
		errorAnt = std::numeric_limits<float>::max();
	}
	else
	{
		float vadv = -0.5 * error;  //Proportional controller
		if (vadv < -MAX_ADV_SPEED) vadv = -MAX_ADV_SPEED;
		try
		{	omnirobot_proxy->setSpeedBase(0, vadv, 0);	} 
		catch (const Ice::Exception &ex)
		{ std::cout << ex << std::endl; }
	}
	errorAnt = error;
	return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
////    SERVANTS
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Changes final point in current trajectory
 *
 * @param target New target pose. Only fist three elements are used as translation
 * @return void
 */
float SpecificWorker::changeTarget(const TargetPose &target)
{
	qDebug() << __FUNCTION__ << "DEPRECATED";
	return 0;
}

float SpecificWorker::go(const TargetPose &target)
{
	return goReferenced(target, 0, 0, 100);
}

float SpecificWorker::goReferenced(const TargetPose &target_, const float xRef, const float zRef, const float threshold)
{
	/////////////////////
	//CHECK PARAMETERS
	/////////////////////
	if (isnan(target_.x) or std::isnan(target_.y) or std::isnan(target_.z) or std::isnan(target_.ry))
	{
		qDebug() << __FUNCTION__ << "Returning. Input parameter -target- is not valid:" << target_.x << target_.y
		         << target_.z << target_.ry;
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Doing nothing. Invalid Target with nan in it";
		throw ex;
	}

	QVec currentT = currentTarget.getTranslation();
	QVec currentRot = currentTarget.getRotation();
	QVec robotT = innerModel->transform("world", "robot");
	QVec robotRot = innerModel->transform6D("world", "robot").subVector(3, 5);
	QVec targetT = QVec::vec3(target_.x, target_.y, target_.z);
	QVec targetRot = QVec::vec3(target_.rx, target_.ry, target_.rz);
	drawTarget(targetT);
	
	qDebug() << __FUNCTION__
	         << "----------------------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__ << ": Target received" << targetT << targetRot << "with ROBOT at" << robotT << robotRot;
	qDebug() << __FUNCTION__ << "Translation error: " << (targetT - robotT).norm2() << "mm. Rotation error:"
	         << targetRot.y() - robotRot.y() << "rads";

	///////////////////////////////////////////////
	//Maximun admitted rate of requests  (one each 250 ms)
	///////////////////////////////////////////////
	if (relojForInputRateControl.elapsed() < 250)
	{
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Fail. Call too close in time. Please wait and call again";
		throw ex;
	}
	else
		relojForInputRateControl.restart();

	///////////////////////////////////////////////
	// Minimun change admitted 60mm and 0.05rads
	///////////////////////////////////////////////
	const int MINCHANGE = 7;
	const float MINROTATION = 0.03;
	if ((targetT - robotT).norm2() < MINCHANGE and fabs(targetRot.y() - robotRot.y()) < MINROTATION)
	{

		QString s = "Fail. Target too close to current pose. TDist=" + QString::number((targetT - robotT).norm2()) +
		            "mm. Min = 60mm. RDist="
		            + QString::number(targetRot.y() - robotRot.y()) + "rads. Min = 0.05rads";
		qDebug() << __FUNCTION__ << s;
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = s.toStdString();
		tState.setDescription("Target too close");
		throw ex;
	}

	///////////////////////////////////////////////
	// Check if robot is inside limits and not in collision.
	///////////////////////////////////////////////
	auto r = sampler.checkRobotValidStateAtTarget(robotT, robotRot);
	if (std::get<bool>(r) == false)
	{
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Execption: Robot is outside limits or colliding. Ignoring request. " + std::get<QString>(r).toStdString();
		qDebug() << __FUNCTION__ << QString::fromStdString(ex.text);
		tState.setDescription(ex.text);
		currentTarget.setState(CurrentTarget::State::ROBOT_COLLISION);
		throw ex;
	}

	///////////////////////////////////////////////
	// Check if target is a a valid point, inside limits and not in collision.
	///////////////////////////////////////////////
	r = sampler.checkRobotValidStateAtTarget(targetT, targetRot);
	if (std::get<bool>(r) == false)
	{
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Exception: Target outside limits or colliding. Ignoring request. " + std::get<QString>(r).toStdString();
		qDebug() << __FUNCTION__ << QString::fromStdString(ex.text);
		tState.setDescription(ex.text);
		currentTarget.setState(CurrentTarget::State::TARGET_COLLISION);
		throw ex;
	}

	///////////////////////////////////////////////
	// Let's see what these guys want
	///////////////////////////////////////////////

	// move virtualrobot to xref, zRef to act as a surrogate
	//innerModel->updateTransformValues("virtualRobot", xRef, 0, zRef, 0, 0, 0, "robot");
	//InnerModelDraw::addPlane_ignoreExisting(innerViewer, "virtualRobot", "robot", QVec::vec3(xRef,0,zRef), QVec::vec3(0,0,0), "#555555", QVec::vec3(50,1000,50));
	
	road.setThreshold(threshold);

	currentTarget.reset(targetT, targetRot, target_.doRotation);
	currentTarget.setState(CurrentTarget::State::GOTO);
	tState.setState("EXECUTING");	
	
	taskReloj.restart();
	return (robotT - targetT).norm2();
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::getState()
{
	return tState.toMiddleware(this->bState, this->road);
}

void SpecificWorker::stop()
{
	currentTarget.setState(CurrentTarget::State::STOP);
}

void SpecificWorker::setHeadingTo(const TargetPose &target)
{
	qDebug() << __FUNCTION__ << "SETHEADING command received";
	currentTarget.setState(CurrentTarget::State::SETHEADING);
}

float SpecificWorker::goBackwards(const TargetPose &target)
{
	qDebug() << __FUNCTION__ << "GOBACKWARDS command received to target:" << target.x << target.y << target.z << target.rz << target.doRotation;

	/////////////////////
	// CHECK PARAMETERS
	/////////////////////
	if (isnan(target.x) or std::isnan(target.y) or std::isnan(target.z) or std::isnan(target.ry))
	{
		qDebug() << __FUNCTION__ << "Returning. Input parameter -target- is not valid." << target.x << target.y << target.z << target.rz << target.doRotation;
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Doing nothing. Invalid Target with NAN in it";
		throw ex;
		return 0.;
	}
	
	currentTarget.setState( CurrentTarget::State::STOP );
	taskReloj.restart();
	
	///////////////////////////////////
	// Wait until the robot has stopped or 5secs
	////////////////////////////////////
	while( (currentTarget.state !=  CurrentTarget::State::IDLE) and taskReloj.elapsed() < 5000 )
	{};
	
	if( currentTarget.state ==  CurrentTarget::State::IDLE)
	{
		currentTarget.setTranslation(QVec::vec3(target.x, target.y, target.z));
		currentTarget.setRotation(QVec::vec3(target.rx, target.ry, target.rz));
		currentTarget.setState(CurrentTarget::State::GOBACKWARDS);
		if (target.doRotation == true)
			currentTarget.setHasRotation(true);
		taskReloj.restart();
	}
	else 
	{
		RoboCompTrajectoryRobot2D::RoboCompException ex;
		ex.text = "Doing nothing. Cannot stop the robot";
		throw ex;
	}
	return 0;
}

/////////////////////
// In development
/////////////////////
void SpecificWorker::mapBasedTarget(const NavigationParameterMap &parameters)
{
	//Examples of task specification
	for( auto p : parameters )
		qDebug() << __FUNCTION__ << QString::fromStdString(p.first) << QString::fromStdString(p.second);

// 	std::string command;
// 	try 
// 	{ 
// 		command = parameters.at("command"); 	
// 		if( command == "go")  //Fill go params
// 		{
// 			QVec target = QVec::zeros(6);
// 			QVec tip = QVec::zeros(6);
// 			bool hasRotation = false;
// 			bool hasTip = false;
// 			float tolerance = 20;
// 			try 
// 			{ 
// 				target[0] = std::stof(parameters.at("tx")); 
// 				target[2] = std::stof(parameters.at("tz")); 
// 			} 
// 			catch(...){};
// 			try 
// 			{ 
// 				target[5] = std::stof(parameters.at("ry")); 
// 				hasRotation = true;
// 			} 
// 			catch(...){};
// 			try
// 			{ 
// 				tip[0] = std::stof(parameters.at("tip_x")); 
// 				tip[2] = std::stof(parameters.at("tip_z")); 
// 				hasTip = true;
// 			} 
// 			catch(...){};
// 			try
// 			{ 
// 				tolerance = std::stof(parameters.at("tolerance")); 
// 			} 
// 			catch(...){};
// 			//goReferenced(target, tip, tolerance);
// 		}
// 		if( command == "stop")  //Fill stop params
// 		{
// 			currentTarget.setState(CurrentTarget::State::STOP);
// 		}
// 	}
// 	catch (const std::out_of_range& oor) 
// 	{ 
// 		std::cerr << "Error, command not found in parameters list" << std::endl;
// 		RoboCompTrajectoryRobot2D::RoboCompException ex;
// 		ex.text = "Error, command not found in parameters list";
// 		throw ex;
// 	};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// CLASS PRIVATE AUXILIARY METHODS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Updates an InnerModel from values read from the robot. Reads laserData too.
 *
 * @param inner InnerModel that is to be updated
 * @return bool
 */
bool SpecificWorker::updateInnerModel(InnerModel *inner, TrajectoryState &state)
{
	try
	{
		omnirobot_proxy->getBaseState(bState);
		inner->updateTransformValues("robot", bState.correctedX, 0, bState.correctedZ, 0, bState.correctedAlpha, 0);
		#ifdef USE_QTGUI
			viewer->innerModel->updateTransformValues("robot", bState.correctedX, 0, bState.correctedZ, 0, bState.correctedAlpha, 0);
		#endif
	}
	catch (const Ice::Exception &ex)
	{
		qDebug() << __FUNCTION__ << "Can't connect to OmniRobot proxy. Retrying...";
		return false;
	}	
	try
	{	laserData = laser_proxy->getLaserData();	}
	catch (const Ice::Exception &ex)
	{
		qDebug() << __FUNCTION__ << "Can't connect to Laser proxy. Retrying...";
		return false;
	}
	//////////////////
	// Connection OK
	//////////////////
	if( currentTarget.state == CurrentTarget::State::DISCONNECTED )
	{
		state.setState("IDLE");
		currentTarget.state = CurrentTarget::State::IDLE;
		timer.setInterval(Period);
	}
	return true;
}

void SpecificWorker::setRobotInitialPose(float x, float z, float alpha)
{
	// 	qDebug()<< __FUNCTION__ << "Sending robot to initial position";
	#ifdef USE_QTGUI
 	  viewer->innerModel->updateTransformValues("initialRobotPose", 0, 0, 0, 0, 0, 0);
		viewer->innerModel->updateTransformValues("robot", x, 0, z, 0, alpha, 0);
	#endif

	try
	{		omnirobot_proxy->setOdometerPose(x, z, alpha);	}
	catch (...)
	{
		qDebug() << __FUNCTION__ << "Error setting robot odometer";
	}
}

float SpecificWorker::angmMPI(float angle)
{
	//float a = angle;
	while (angle > +M_PI) angle -= 2. * M_PI;
	while (angle < -M_PI) angle += 2. * M_PI;
// 	if (fmod(a, M_PI) != angle)
// 	{
// 		qDebug() << __FUNCTION__ << a << angle << fmod(a, M_PI);
// 		//qFatal("FMOD SUCKS");
// 	}
	
	return angle;
	}

/////////////////////////////////////////////////////////////////////////////
//// DRAWING METHODS
////////////////////////////////////////////////////////////////////////////

void SpecificWorker::drawTarget(const QVec &target)
{
#ifdef USE_QTGUI
	//Draw target as red box
	InnerModelDraw::addPlane_ignoreExisting(viewer->innerViewer, "target", "world", QVec::vec3(target(0), 5, target(2)),
	                                        QVec::vec3(1, 0, 0), "#990000", QVec::vec3(80, 80, 80));
#endif
}

void SpecificWorker::drawGreenBoxOnTarget(const QVec &target)
{
#ifdef USE_QTGUI
	InnerModelDraw::addPlane_ignoreExisting(viewer->innerViewer, "target", "world", QVec::vec3(target(0), 1800, target(2)),
	                                        QVec::vec3(1, 0, 0), "#009900", QVec::vec3(100, 100, 150));
#endif

}

void SpecificWorker::printNumberOfElementsInIMV()
{
/*
#ifdef USE_QTGUI
	try
	{	RoboCompInnerModelManager::NodeInformationSequence ni;
		innermodelmanager_proxy->getAllNodeInformation(ni);
// 		qDebug() << __FUNCTION__ << "..............Number of elements in: " << ni.size();
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
#endif
*/

}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
bool SpecificWorker::removeNode(const QString &item)
{
	if (item == "root")
	{
		qDebug() << "Can't remove root elements" << item;
		return false;
	}

	InnerModelNode *node = innerModel->getNode(item);
	if (node == NULL)
	{
		qDebug() << "Can't remove not existing elements" << item;
		return false;
	}

	QStringList l;
	innerModel->getSubTree(node, &l);
	innerModel->removeSubTree(node, &l);

	return true;
}

/**
 * @brief Adds a plane to InnerModel
 * 
 * @param item ...
 * @param parentS ...
 * @param path ...
 * @param scale ...
 * @param t ...
 * @param r ...
 * @return void
 */
void SpecificWorker::addPlane(QString item, QString parentS, QString path, QVec scale, QVec t, QVec r)
{
	InnerModelTransform *parent = dynamic_cast<InnerModelTransform *>(innerModel->getNode(parentS));
	if (innerModel->getNode(item) != NULL)
		removeNode(item);
	InnerModelMesh *mesh = innerModel->newMesh(item, parent, path, scale(0), scale(1), scale(2), 0, t(0), t(1), t(2),
	                                           r(0), r(1), r(2));
	mesh->setScale(scale(0), scale(1), scale(2));
	parent->addChild(mesh);
}


bool SpecificWorker::insertObstacle()
{
// 	QString obstacleParent = "floor";
// 	QString obstacleName = "blockWall";
// 	QVec T = QVec::vec3(800, 400, 800);
// 	QVec R = QVec::vec3(0,0,1);
// 	QVec final = innerModel->transform("world",QVec::vec3(0,0,300),"laser");
// 	QVec sizeObstacle = QVec::vec3(500, 800, 100);
// 	
// 	InnerModelDraw::addPlane_ignoreExisting(innerViewer, obstacleName, obstacleParent, final, R, "#555555", sizeObstacle);
// 	addPlane(obstacleName, obstacleParent, "/home/robocomp/robocomp/files/osgModels/basics/cube.3ds", sizeObstacle, final, R);
// 	innerViewer->update();
// 	try{ controller->stopTheRobot(omnirobot_proxy); } catch(const Ice::Exception &ex){std::cout << ex.what() << std::endl;};
// 	//replaning
// 	plannerPRM.removeGraph(innerViewer);
// 	qDebug() << __FUNCTION__ << "creating graph";
// 	plannerPRM.createGraph();
// 	currentTarget.setWithoutPlan(true);
// 	currentTarget.setState(CurrentTarget::State::GOTO);
	return true;
}

void SpecificWorker::tryingToConnect()
{

}


//CHECK PARAMETERS
// 	QVec target = current.getTranslation();
// 	if( target.size() < 3 or std::isnan(target.x()) or std::isnan(target.y()) or std::isnan(target.z()))
// 	{
// 		qDebug() << __FUNCTION__ << "Returning. Invalid target";
// 		RoboCompTrajectoryRobot2D::RoboCompException ex; ex.text = "Returning. Invalid target";
// 		throw ex;
// 		return false;
// 	}
// 	printf("doing backwards\n");
// 	float MAX_ADV_SPEED = 600.f;
// 	const float MAX_POSITIONING_ERROR  = 50;  //mm
// 	state.setState("EXECUTING");
// 	QVec rPose = innerModel->transform("world","robot");
// 	float error = (rPose-target).norm2();
// 	//float error = target.norm2();
// 	// 	qDebug() << __FUNCTION__ << "Error: " << error;
// 
// 	if( error < MAX_POSITIONING_ERROR )		//TASK IS FINISHED
// 	{
// //		current.setHasRotation(false);
// //		myRoad.setFinished(true);
// 		drawGreenBoxOnTarget( current.getTranslation() );
// // 		current.print();
// // 		current.reset();
// // 		myRoad.reset();
// // 		myRoad.endRoad();
// 		state.setElapsedTime(taskReloj.elapsed());
// 		//state.setState("IDLE");
// 		try
// 		{
// 		  omnirobot_proxy->setSpeedBase(0, 0, 0);
// 		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
// 		//myRoad.requiresReplanning = true;
// 
// 		currentT.setWithoutPlan(false);
// 		///////
// 		//agregar plane		AQUI HAY QUE AGREGAR EL PLANO Y DAR LA ORDEN DE REPLANIFICAR!!
// 		
// 		changeCommand(currentT,CurrentTarget::Command::INSERTOBSTACLE);
// 	}
// 	else
// 	{
// 		float vadv = -0.5 * error;  //Proportional controller
// 		if( vadv < -MAX_ADV_SPEED ) vadv = -MAX_ADV_SPEED;
// 		try
// 		{
// 		  //differentialrobot_proxy->setSpeedBase(vadv, 0);
// 		  omnirobot_proxy->setSpeedBase(0, vadv, 0);
// // 		  omnirobot2_proxy->setSpeedBase(0, vadv, 0);
// 		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
// 	}
// 


//////////////////////////////////////
/////////////////////////////////////

// /**
//  * @brief To go in Controller class or as a separate class with soft real time restrictions.
//  * Keeps the robot safe but, being the last level of control, should try to compute a safe diverging trajectory to keep things moving on the road.
//  *
//  * @param innerModel ...
//  * @param road ...
//  * @param laserData ...
//  * @param vadvance ...
//  * @param vrot ...
//  * @return bool
//  */
// bool SpecificWorker::avoidanceControl(InnerModel& innerModel, const TLaserData& laserData, float& vadvance, float& vrot, uint elapsed)
// {
// 	const float MAX_ADV_ACC = 400;
// 	const float MAX_ROT_ACC = 0.5;
// 	//if( bState.advV == 0 and bState.rotV == 0 )
// 	//	return false;
//
// 	//check collision for a future potential position
// 	//given current speed and maximun acceleration, compute the minimun time to stop
// 	//float time = std::min<float>( fabs((vadvance + bState.advV) / 400.f), 0.1); // mm/sg2
// 	//float time = std::max<float>( (fabs(bState.advV) / 400.f), fabs(bState.rotV / 2.));
// 	float time = fabs(bState.advV + vadvance) / 400.f;
// 	//float time = elapsed / 1000.f;
// 	//we want to decide here if the next movement will cause a collision
// 	QVec repulsionVector;
// 	bool collision;
// 	std::tie(repulsionVector, collision) = checkInminentCollision( innerModel, laserData,
// 																   bState.advV + vadvance,
// 																   bState.rotV + vrot, time);
//
// 	//if( repulsionVector != QVec::zeros(3))
// 	if( collision )
// 	{
// 		qDebug() << "---COLLISION!!!!!!! at direction " << repulsionVector;
//
// 		//compute a <vadv,vrot> command to save the situation
// 		//check the outcome of all possible velocities to find one that saves the obstacle or else stop the robot
// 		//differentialrobot_proxy->setSpeedBase( vadvance, vrot);
// 		differentialrobot_proxy->setSpeedBase( 0, 0);
// 	}
// 	else
// 		try
// 		{
// 			differentialrobot_proxy->setSpeedBase( vadvance, vrot);
// 		}
// 		catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl; }
//
// 	return true;
// }
//
// std::tuple<QVec, bool> SpecificWorker::checkInminentCollision(InnerModel& innerModel, const RoboCompLaser::TLaserData& laserData, float vadv, float vrot, float delta)
// {
// 	//integrate speed to obtain base next position using Borenstein's approximation
// 	float deltaT = delta; // / 1000.f;
// 	float xN = bState.x + (vadv * deltaT) * sin(vrot * deltaT);
// 	float zN = bState.z + (vadv * deltaT) * cos(vrot * deltaT);
// 	float angN = bState.alpha + (vrot * deltaT);
// 	qDebug() << vadv << vrot << deltaT << "now" << bState.x << bState.z << bState.alpha << "new" << xN << zN << angN;
//
// 	//now check if there will be collision in the future position using current laserData
// 	innerModel->updateTransformValues("robot", xN, 0, zN, 0, angN, 0);
// 	// Three points of rectangle approximating the robot base
// 	QVec p1 = innerModel->transform("world", QVec::vec3(220,0,220), "robot" );
// 	QVec p2 = innerModel->transform("world", QVec::vec3(220,0,-220), "robot" );
// 	QVec p3 = innerModel->transform("world", QVec::vec3(-220,0,220), "robot" );
// 	innerModel->updateTransformValues("robot", bState.x, 10, bState.z, 0, bState.alpha, 0);
//
// 	QVec p(3,0.f);
// 	int k=0;
// 	bool collision = false;
// 	QVec repulsionVector = QVec::zeros(3);
// 	for(auto i : laserData)
// 	{
// 		p = innerModel->laserTo("world","laser",i.dist,i.angle);
// 		if( robotLaserCollision( p1, p2, p3, p ))
// 		{
// 			//p = p * (T)(-1);
// 			repulsionVector += ( p * (T)(-1));
// 			collision = true;
// 		}
// 	}
// 	return std::make_tuple(repulsionVector, collision);
// }
//
//
//
// bool SpecificWorker::robotLaserCollision( const QVec &p1, const QVec &p2, const QVec &p3,  const QVec &p)
// {
// 	QVec p21 = p2-p1;
// 	QVec p31 = p3-p1;
//
// 	if ((p.x() - p1.x()) * p21.x() + (p.z() - p1.z()) * p21.z() < 0.0) return false;
// 	if ((p.x() - p2.x()) * p21.x() + (p.z() - p2.z()) * p21.z() > 0.0) return false;
// 	if ((p.x() - p1.x()) * p31.x() + (p.z() - p1.z()) * p31.z() < 0.0) return false;
// 	if ((p.x() - p3.x()) * p31.x() + (p.z() - p3.z()) * p31.z() > 0.0) return false;
//
// 	return true;
// }
//
// /**
//  * @brief Offset computation of each laser beam to account for the geometry of the getBaseState
//  *
//  * @param innerModel ...
//  * @param laserData ...
//  * @return std::vector< float, std::allocator >
//  */
// std::vector<float> SpecificWorker::computeRobotOffsets(InnerModel& innerModel, const RoboCompLaser::TLaserData &laserData)
// {
// 	//Base geometry GET FROM IM!!!
// 	QRectF base( QPointF(-200, 200), QPointF(200, -200));
// 	std::vector<float> baseOffsets;
// 	QVec p(3,0.f);
// 	int k;
//
// 	for(auto i : laserData)
// 	{
// 		for(k=10; k<4000; k++)
// 		{
// 			p = innerModel->laserTo("robot","laser",k,i.angle);
// 			if( base.contains( QPointF( p.x(), p.z() ) ) == false )
// 				break;
// 		}
// 		baseOffsets.push_back(k);
// 	}
// 	return baseOffsets;
// }
//


// bool SpecificWorker::checkRobotValidStateAtTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec &target)
// {
// 	//now check if there will be collision in the future position using current laserData
// 	float height = innerModel->transform("world","robot").y();
// 	//qDebug() << __FUNCTION__ << "height" << height << target.y();
// 	innerModel->updateTransformValues("robot", target.x(), target.y(), target.z(), 0, 0, 0);
//
// 	// Three points of rectangle approximating the robot base
// 	QVec p1 = innerModel->transform("world", QVec::vec3(-220,10,220), "robot" );
// 	QVec p2 = innerModel->transform("world", QVec::vec3(220,10,220), "robot" );
// 	QVec p3 = innerModel->transform("world", QVec::vec3(-220,10,-220), "robot" );
// 	//QVec p4 = innerModel->transform("world", QVec::vec3(220,0,-220), "robot" );
//
// 	QVec p21 = p2-p1;
// 	QVec p31 = p3-p1;
// 	//put back the robot to where it was
// 	innerModel->updateTransformValues("robot", bState.x, height, bState.z, 0, bState.alpha, 0);
//
// 	// 	target.print("target");
// 	// 	p1.print("p1");
// 	// 	p2.print("p2");
// 	// 	p3.print("p3");
//
// 	//Check if any laser point falls inside the rectangle using an angle criterium
// 	QVec p(3,0.f);
// 	for(auto i : laserData)
// 	{
// 		p = innerModel->laserTo("world","laser", i.dist, i.angle);
// 		//Check if inside
// 		if ((p-p1) * p21 > 0
// 			and
// 			(p-p1) * p21 < p21 * p21
// 			and
// 			(p-p1) * p31 > 0
// 			and
// 			(p-p1) * p31 < p31*p31 )
// 		return false;
// 	}
// 	return true;
// }
//
// bool SpecificWorker::searchRobotValidStateCloseToTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec& target)
// {
// 	QVec lastPoint;
//
// 	QVec origin = innerModel->transform("world","robot");
// 	float stepSize = 30.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
// 	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);
// 	float step;
//
// 	//if too close return target
// 	if (nSteps == 0)
// 	{
// 		return false;
// 	}
// 	step = 1./nSteps;
//
// 	//go along visual ray connecting robot pose and target pos in world coordinates. l*robot + (1-r)*roiPos = 0
// 	QVec point(3);
// 	float landa = step;
//
// 	lastPoint = origin;
// 	for(uint i=1 ; i<=nSteps; i++)
// 	{
// 		point = (origin * (1-landa)) + (target * landa);
// 		if (checkRobotValidStateAtTarget(innerModel, laserData, point) )
// 		{
// 			lastPoint  = point;
// 			landa = landa + step;
// 		}
// 		else
// 		{
// 			target = lastPoint;
// 			return true;
// 		}
// 	}
// 	return true;
// }