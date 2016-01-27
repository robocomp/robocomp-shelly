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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	goMotorsGO      = false;
	stateMachine    = State::IDLE;
	abortCorrection = false;
	innerModel      = NULL;
	contador        = 0;
	mutexSolved     = new QMutex(QMutex::Recursive);
	mutexRightHand  = new QMutex(QMutex::Recursive);
	errorInv        = QVec::zeros(6);
	
#ifdef USE_QTGUI
	connect(this->goButton, SIGNAL(clicked()), this, SLOT(goYESButton()));
	innerViewer        = NULL;
	osgView            = new OsgView(this->widget);
 	show();
#endif
	
	QMutexLocker ml(mutex);
	INITIALIZED        = false;
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
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}

#ifdef USE_QTGUI
	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);
		delete innerViewer;
	}
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
#endif
	InnerModelNode *nodeParent = innerModel->getNode("root");
	if( innerModel->getNode("target") == NULL)
	{
		InnerModelTransform *node = innerModel->newTransform("target", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
	}
	rightHand = new VisualHand(innerModel, "grabPositionHandR");	
	timer.start(Period);		
	QMutexLocker ml(mutex);
	INITIALIZED = true;
	
	
	rightTip = rightHand->getTip();

	return true;
}



void SpecificWorker::compute()
{
	const float tagLostThresholdTime=1;

#ifdef USE_QTGUI
		if (innerViewer)
		{
			innerViewer->update();
			osgView->autoResize();
			osgView->frame();
		}
#endif

	updateInnerModel_motors_target_and_visual();
	{
		QMutexLocker ml(mutexRightHand);
		if (rightHand->getSecondsElapsed() > tagLostThresholdTime )
		{
			rightHand->setVisualPosewithInternalError();
		}
		else
		{
			errorInv = rightHand->getInternalErrorInverse();
		}
		rightHandVisualPose = rightHand->getVisualPose();
		rightHandInternalPose = rightHand->getInternalPose();
	}
	
	QMutexLocker ml(mutex);
	switch(stateMachine)
	{
		case State::IDLE:
			if (currentTarget.getState() == Target::State::WAITING)
			{
				stateMachine     = State::INIT_BIK;
				abortCorrection = false;
				qDebug()<<"Ha llegado un TARGET: "<<currentTarget.getPose();
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::INIT_BIK:
			applyFirstApproximation();
			currentTarget.setState(Target::State::IN_PROCESS);
			correctedTarget = currentTarget;
			stateMachine    = State::WAIT_BIK;
			waitForMotorsToStop(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).motors);
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Wait for IK's lower levels to start corrections
			if (inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).finish == false)
				return;
			qDebug()<<"---> El IK ha terminado.";
			stateMachine = State::CORRECT_ROTATION;
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:
			if (inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).finish == false)
				return;
			waitForMotorsToStop(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).motors);
			if (correctPose()==true or abortCorrection==true)
			{
				if(nextTargets.isEmpty()==false)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					currentTarget = nextTargets.head();
					nextTargets.dequeue();
				}
				else
					currentTarget.setState(Target::State::IDLE);
				stateMachine = State::IDLE;
				//goHome("RIGHTARM");
			}
		break;
		//---------------------------------------------------------------------------------------------
		default:
			break;
	}
}


/**
 *\brief This method returns the state of the part.
 * @return bool: If the part hasn't got more pending targets, it returns TRUE, else it returns FALSE
 */
bool SpecificWorker::getPartState(const string &bodyPart)
{
	//return inversekinematics_proxy->getPartState(bodyPart);
	return (nextTargets.isEmpty() and currentTarget.getState()==Target::State::IDLE);;
}


/**
 * \brief this method returns the state of a determinate target.
 * @param part part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	//return inversekinematics_proxy->getTargetState(bodyPart, targetID);
	QMutexLocker ml(mutexSolved);
	RoboCompInverseKinematics::TargetState 	state;
	state.finish = false;
	
	for(int i=0; i<solvedList.size(); i++)
	{
		if(bodyPart=="RIGHTARM" and targetID==solvedList[i].getID_VIK())
		{
			state.finish = true;
			if(solvedList[i].getState()==Target::State::NOT_RESOLVED) state.state = "NOT_RESOLVED";
			if(solvedList[i].getState()==Target::State::RESOLVED)     state.state = "RESOLVED";
		}
	}
	return state;
}


/**
 * \brief This method reimplements the interface method setTargetPose6D of the inversekinematics component
 * in order to aply the visual correction
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{
	float VIK_thresholdT = 25.0;
	float VIK_thresholdR = 0.18;
	
	QMutexLocker ml(mutex);
	cout<<"Recibido target"<<endl;
	if(currentTarget.getState()==Target::State::IDLE)
	{
		currentTarget.setBodyPart    (bodyPart);
		currentTarget.setPose        (target);
		currentTarget.setWeights     (weights);
		currentTarget.setState       (Target::State::WAITING);
		currentTarget.setID_VIK      (contador);
// 		if (thresholdT>0) 
// 			VIK_thresholdT = thresholdT;
// 		if (thresholdR>0) 
// 			VIK_thresholdR = thresholdR;
		currentTarget.setThresholds (VIK_thresholdT, VIK_thresholdR);
	}
	else
	{
		Target auxnextTargets;
		auxnextTargets.setBodyPart (bodyPart);
		auxnextTargets.setPose     (target);
		auxnextTargets.setWeights  (weights);
		auxnextTargets.setState    (Target::State::WAITING);
		auxnextTargets.setID_VIK   (contador);
// 		if (thresholdT>0) 
// 			VIK_thresholdT = thresholdT;
// 		if (thresholdR>0) 
// 			VIK_thresholdR = thresholdR;
		auxnextTargets.setThresholds (VIK_thresholdT, VIK_thresholdR);
		
		nextTargets.enqueue(auxnextTargets);
	}
	contador++;
	
	return contador-1;
}


/**
 * @brief Make the body part advance along a given direction. It is meant to work as a simple translational joystick to facilitate grasping operations
 * @param bodyPart  name of the body part.
 * @param ax the direction
 * @param dist step to advance un milimeters
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	return inversekinematics_proxy->setTargetAdvanceAxis(bodyPart, ax, dist);
}


/**
 * \brief This method  of the interface stores a new ALIGNAXIS target into the correspondig part of the robot.
 * @param bodypart part of the robot body
 * @param target pose of the goal position
 * @param ax axis to be aligned
 */
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	//NOTE: When the VIK is connected to IK directly
// 	int id = inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
// 	while (inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
// 	waitForMotorsToStop(inversekinematics_proxy->getTargetState(bodyPart, id).motors);
// 	return id;
	//NOTE: When the VIK is connected to GIK and GIK is connected to IK.
	return inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
}


/**
 * \brief this method moves the motors to their home value
 * @param bodyPart the part of the robot that we want to move to the home
 */
void SpecificWorker::goHome(const string &bodyPart)
{
	inversekinematics_proxy->goHome(bodyPart);
}

void SpecificWorker::stop(const string &bodyPart)
{
	QMutexLocker ml(mutex);
	nextTargets.clear();
	currentTarget.setState(Target::State::IDLE);
	stateMachine = State::IDLE;

	inversekinematics_proxy->stop(bodyPart);
}

/**
 * \brief this method changes the position of a determina joint.
 * @param joint the joint to change
 * @param angle the new angle of the joint.
 * @param maxSpeed the speed of the joint
 */
void SpecificWorker::setJoint(const string &joint, const float angle, const float maxSpeed)
{
	inversekinematics_proxy->setJoint(joint, angle, maxSpeed);
}


/**
 * @brief Set the fingers of the right hand position so there is d mm between them
 * @param d milimeters between fingers
 */
void SpecificWorker::setFingers(const float d)
{
	inversekinematics_proxy->setFingers(d);
}


void SpecificWorker::newAprilTag(const tagsList &tags)
{
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
			{
				QMutexLocker ml(mutexRightHand);
				rightHand->setVisualPose(tag);
			}
		}
	}
}



void SpecificWorker::applyFirstApproximation()
{
	printf("\n\n\ncall applyFirstApproximation\n");

	const Pose6D tt = currentTarget.getPose6D();
	innerModel->updateTransformValues("target",    tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	innerModel->updateTransformValues("corrected", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	
	correctPoseWithErrInv();
	usleep(500000);
}

bool SpecificWorker::correctPose()
{
	printf("\n\n\nCall correctPose\n");
	const float umbralMaxTime=15;
	//const float umbralErrorT=25.0, umbralErrorR=0.18;
	printf("Seconds without tag: %f\n", rightHand->getSecondsElapsed());
	
	QVec errorInvP           = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", "target") * errorInvP;
	errorInvP_from_root.print("errorInvP_from_root");

	QVec target = innerModel->transform("root", "target");
	printf("target             [ %f %f %f ]\n", target(0), target(1), target(2));
	printf("visualPose         [ %f %f %f ]\n", rightHandVisualPose(0), rightHandVisualPose(1), rightHandVisualPose(2));
	printf("internalPose       [ %f %f %f ]\n", rightHandInternalPose(0), rightHandInternalPose(1), rightHandInternalPose(2));

	if (currentTarget.getRunTime()>umbralMaxTime)
	{
		abortCorrection = true;
		currentTarget.setState(Target::State::NOT_RESOLVED);
		innerModel->transform6D("target", "visual_hand").print("abort with visual error");
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return false;
	}

	QVec visualError = innerModel->transform6D("target", "visual_hand");
	printf("visualError         [ %f %f %f ]\n", visualError(0), visualError(1), visualError(2));
	float Tnorm = QVec::vec3(visualError.x(),  visualError.y(),  visualError.z()).norm2();
	float Rnorm = QVec::vec3(visualError.rx(), visualError.ry(), visualError.rz()).norm2();
	if (Tnorm<currentTarget.getThresholds()[0] and Rnorm<currentTarget.getThresholds()[1])
	{
		currentTarget.setState(Target::State::RESOLVED);
		innerModel->transform6D("target", "visual_hand").print("done with visual error");
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return true;
	}



	bool r = correctPoseWithErrInv();
	return r;
}


bool SpecificWorker::correctPoseWithErrInv()
{
	qDebug()<<"-------------------------";

	// CORRECT TRANSLATION
	QVec errorInvP           = QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).operator*(1.0);
	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", "target") * errorInvP;
// 	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", "target") * errorInvP;
 

	printf("suma a la pose     [ %f %f %f ]\n", errorInvP_from_root(0), errorInvP_from_root(1), errorInvP_from_root(2));


	QVec poseCorregida   = innerModel->transform("root", "target") + errorInvP_from_root;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal(0) = poseCorregida(0);
	correccionFinal(1) = poseCorregida(1);
	correccionFinal(2) = poseCorregida(2);
	

	printf("correccion nueva   [ %f %f %f ]\n", poseCorregida(0), poseCorregida(1), poseCorregida(2));
	
	
	// CORRECT ROTATION
	QMat rotErrorInv = Rot3D(errorInv.rx(), errorInv.ry(), errorInv.rz());
 	QVec angulosFinales = (innerModel->getRotationMatrixTo("root", "target")*rotErrorInv.invert()).extractAnglesR_min();
	correccionFinal(3) = angulosFinales(0);
	correccionFinal(4) = angulosFinales(1);
	correccionFinal(5) = angulosFinales(2);
	correctedTarget.setPose(correccionFinal);
	printf("correction pose    [ %f %f %f ]\n", correccionFinal(0), correccionFinal(1), correccionFinal(2));
	innerModel->updateTransformValues("corrected", correccionFinal(0), correccionFinal(1), correccionFinal(2), correccionFinal(3), correccionFinal(4), correccionFinal(5));


	// Call BIK and wait for it to finish
	printf("Mandamos %f %f %f\n", correctedTarget.getPose6D().x, correctedTarget.getPose6D().y, correctedTarget.getPose6D().z);
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID_IK(identifier);

	return false;
}



void SpecificWorker::updateInnerModel_motors_target_and_visual()
{
	//qDebug() << "--------";
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		//	qDebug() << QString::fromStdString(j.first) << j.second.pos;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
	}
	const Pose6D tt = currentTarget.getPose6D();
	innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	const QVec pR = rightHand->getVisualPose();
	innerModel->updateTransformValues("visual_hand", pR.x(), pR.y(), pR.z(), pR.rx(), pR.ry(), pR.rz());
}


/**
 * \brief UPDATE MOTORS
 */
void SpecificWorker::waitForMotorsToStop(RoboCompInverseKinematics::MotorList motors)
{
	MotorStateMap allMotorsAct, allMotorsBack;
	jointmotor_proxy->getAllMotorState(allMotorsBack);
	usleep(300000);
	for (bool allStill=false; allStill==false; allMotorsBack=allMotorsAct)
	{
		jointmotor_proxy->getAllMotorState(allMotorsAct);
		allStill = true;
		for (auto v : allMotorsAct)
		{
			if (abs(v.second.pos - allMotorsBack[v.first].pos) > 0.01)
			{
				allStill = false;
				break;
			}
		}
		if (allStill)
			break;
		usleep(100000);
	} 

}



int SpecificWorker::mapBasedTarget(const string &bodyPart, const StringMap &strings, const ScalarMap &scalars)
{
	Pose6D target;
	WeightVector weights;
	float thresholdT = 25.0;
	float thresholdR = 0.18;
	target.x = 250;


	if (scalars.find("wtx") != scalars.end())
	{
		weights.x = scalars["wtx"];
	}

	if (scalars.find("wty") != scalars.end())
	{
		weights.y = scalars["wty"];
	}
	if (scalars.find("wtz") != scalars.end())
	{
		weights.z = scalars["wtz"];
	}

	if (scalars.find("wrx") != scalars.end())
	{
		weights.rx = scalars["wrx"];
	}
	if (scalars.find("wry") != scalars.end())
	{
		weights.ry = scalars["wry"];
	}
	if (scalars.find("wrz") != scalars.end())
	{
		weights.rz = scalars["wrz"];
	}

	
	// ALERT: if tt is 0 then, weights 0 for no correction
	if (scalars.find("ttx") != scalars.end())
	{
		target.x = scalars["ttx"];
	}
	else
	{
		weights.x = 0.0;
	}

	if (scalars.find("tty") != scalars.end())
	{
		target.y = scalars["tty"];
	}
	else
	{
		weights.y = 0.0;
	}
	if (scalars.find("ttz") != scalars.end())
	{
		target.z = scalars["ttz"];
	}
	else
	{
		weights.z = 0.0;
	}
	
	if (scalars.find("trx") != scalars.end())
	{
		target.rx = scalars["trx"];
	}
	else
	{
		weights.rx = 0.0;
	}
	if (scalars.find("try") != scalars.end())
	{
		target.ry = scalars["try"];
	}
	else
	{
		weights.ry = 0.0;
	}
	if (scalars.find("trz") != scalars.end())
	{
		target.rz = scalars["trz"];
	}
	else
	{
		weights.rz = 0.0;
	}



	
	if (scalars.find("thresholdT") != scalars.end())
	{
		thresholdT = scalars["thresholdT"];
	}
	if (scalars.find("thresholdR") != scalars.end())
	{
		thresholdR = scalars["thresholdR"];
	}

	
	float VIK_thresholdT = thresholdT;
	float VIK_thresholdR = thresholdR;
	
	QMutexLocker ml(mutex);
	cout<<"Recibido target"<<endl;
	if(currentTarget.getState()==Target::State::IDLE)
	{
		currentTarget.setBodyPart    (bodyPart);
		currentTarget.setPose        (target);
		currentTarget.setWeights     (weights);
		currentTarget.setState       (Target::State::WAITING);
		currentTarget.setID_VIK      (contador);
		currentTarget.setThresholds (VIK_thresholdT, VIK_thresholdR);
	}
	else
	{
		Target auxnextTargets;
		auxnextTargets.setBodyPart (bodyPart);
		auxnextTargets.setPose     (target);
		auxnextTargets.setWeights  (weights);
		auxnextTargets.setState    (Target::State::WAITING);
		auxnextTargets.setID_VIK   (contador);
		auxnextTargets.setThresholds (VIK_thresholdT, VIK_thresholdR);
		
		nextTargets.enqueue(auxnextTargets);
	}
	contador++;
	
	return contador-1;
}













