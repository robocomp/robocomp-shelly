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
#include <agm.h>
#include <boost/concept_check.hpp>


/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	setlocale (LC_ALL, "C");

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	active = false;

	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus/etc/ursusMM.xml");
	currenState =STOP;

	exec=false;

}

void SpecificWorker::stop()
{
	qDebug()<<"stop()"<<"exec"<<exec;
}

void SpecificWorker::closeHand()
{
	qDebug()<<"closeHand()"<<elapsedTime.elapsed()<<currenState<<"exec"<<exec;

	if (elapsedTime.elapsed()>1000)
	{
		qDebug()<<"Hand should be closed"<<currenState<<"exec"<<exec;
		if (currenState==CLOSEHAND)
		{
			exec=false;
			elapsedTime.restart();
			currenState=APPROACHFINGER;
			return;
		}
		else if (currenState==APPROACHHAND)
		{
			exec=false;
			elapsedTime.restart();
			currenState=TAKE;
			return;
		}
		else
		{
			qDebug() << "comentario inapropiado, closeHand";
		}
	}
	if (exec==true)
		return;
	try
	{
		bodyinversekinematics_proxy->setFingers(1.0);
		exec=true;
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::closeHand(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::approachFinger()
{
	static bool firstTime = true;
// 	qDebug()<<"approachFinger()"<<"exec"<<exec<<"currenState"<<currenState<<"elapsedTime.elapsed()"<<elapsedTime.elapsed();
	int32_t object = atoi(params["o"].value.c_str());
	int32_t robot = worldModel->getIdentifierByType("robot");
// 	printf("go get %d\n", object);

// 	if (elapsedTime.elapsed()>2000)
// 	{
// 		qDebug()<<"Fingers should be close to target";
// 		currenState=TOUCHFINGER;
// // 		currenState=STOP;
// // 		exec=false;
// 		exec=true;
// 		elapsedTime.restart();
// 		return;
// 	}
// 	//este exec no me convence, mientras estoy haciendo la accion no llamo
// 	if (exec == true)
// 		return;

	try
	{
		const float tx = str2float(worldModel->getSymbol(object)->getAttribute("tx"));
		const float ty = str2float(worldModel->getSymbol(object)->getAttribute("ty"));
		const float tz = str2float(worldModel->getSymbol(object)->getAttribute("tz"));
		// float rx = str2float(worldModel->getSymbol(object)->getAttribute("rx"));
		// float ry = str2float(worldModel->getSymbol(object)->getAttribute("ry"));
		// float rz = str2float(worldModel->getSymbol(object)->getAttribute("rz"));
		const float rwtx = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_tx"));
		const float rwty = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_ty"));
		const float rwtz = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_tz"));

// 		qDebug()<<"marca según aprilAgent"<<tx<<ty<<tz<<"munieca robot Tr"<<rwtx<<rwty<<rwtz;
// 		innerModel->transform("world", QVec::vec3(tx,ty,tz), "robot").print("Marca AR desde el mundo sin offset ");
		QVec offset = QVec::vec3(0., -60., -80.);
		QVec poseTr = innerModel->transform("world", QVec::vec3(tx,ty,tz)+offset, "robot");
		QVec poseWrist = QVec::vec3(rwtx, rwty, rwtz)+offset;

// 		printf("\n\n-------------------------------------------------------------------------\n");
// 		poseTr.print("goal");
// 		poseWrist.print("current");
// 		QVec error = poseTr-poseWrist;
// 		error.print("error");
// 		printf("error_norm: %f\n", error.norm2());

// 		if (error.norm2()>30)
		{
			QVec rot = QVec::vec3(M_PIl, -1.5, 0.);


			if (firstTime )
			{
				printf("gooooooo T=(%.2f, %.2f, %.2f)  R=(%.2f, %.2f, %.2f)\n", poseTr(0), poseTr(1), poseTr(2), rot(0), rot(1), rot(2));
				sendRightHandPose(poseTr, rot, QVec::vec3(1,1,1), QVec::vec3(1,0,1));
				sleep(5);
				exec =true;
				firstTime = false;
			}


		}
		QVec sightPoint = (poseTr+poseWrist).operator*(0.5);
		//saccadic3D(sightPoint, QVec::vec3(0, -1, 0));
//
		//timerAutomataApproachFinger.start(5000);
	}
	catch(AGMModelException &e)
	{
		printf("I don't know object %d\n", object);
	}
}


void SpecificWorker::touchFinger()
{
	qDebug()<<"touchFinger(), waiting for touchSensor"<<"exec"<<exec;

	if (elapsedTime.elapsed()>2000)
	{
		qDebug()<<"Fingers should be close to target";
		currenState=OPENHAND;
		exec=false;
		elapsedTime.restart();
		return;
	}
	//este exec no me convence, mientras estoy haciendo la accion no llamo
	if (exec == true)
		return;

	try
	{
		bodyinversekinematics_proxy->setFingers(1.0);
		exec=true;

	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::closeHand(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
	//ok
// 	timerAutomataTouch.start(500);
// 	if (timerAutomataTouch.isActive())
// 	   return;
// 	currenState=OPENHAND;

}
void SpecificWorker::openHand()
{
	qDebug()<<"openHand(), waiting for OPEN"<<"exec"<<exec;

	if (elapsedTime.elapsed()>2000)
	{
		qDebug()<<"HAND should be open";
		currenState=APPROACHHAND;
		exec=false;
		elapsedTime.restart();
		return;
	}
	//este exec no me convence, mientras estoy haciendo la accion no llamo
	if (exec == true)
		return;
	try
	{
		bodyinversekinematics_proxy->setFingers(1.0);
		exec=true;
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::openHand(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::approachHand()
{
	qDebug()<<"approachHand(), avancing through the axis"<<"currenState"<<currenState;
	if (elapsedTime.elapsed()>1000)
	{
		qDebug()<<"HAND should be close to target";
		currenState=GRASP;
		exec=false;
		elapsedTime.restart();
		sleep(2);
		return;
	}
	//este exec no me convence, mientras estoy haciendo la accion no llamo
	if (exec == true)
		return;

	try
	{
// 		RoboCompBodyInverseKinematics::Axis axis;
// 		axis.x=0.0;axis.y=0.;axis.z=1.0;
		//bodyinversekinematics_proxy->advanceAlongAxis("RIGHTARM",axis, -100.0);
		exec=true;

	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::approachHand(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::grasp()
{
	 qDebug()<<"take, I hang on the mug";

	if (elapsedTime.elapsed()>2000)
	{
		qDebug()<<"HAND should be grasping the MUG";
		currenState=TAKE;
		exec=false;
		elapsedTime.restart();
		return;
	}
	//este exec no me convence, mientras estoy haciendo la accion no llamo
	if (exec == true)
		return;

	try
	{
		bodyinversekinematics_proxy->setFingers(0.0);
		exec=true;

	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::grasp(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::take()
{
	 qDebug()<<"take, I hang on the mug";

	if (elapsedTime.elapsed()>2000)
	{
		qDebug()<<"HAND should be grasping the MUG";
		currenState=CLOSEHAND;
		exec=false;
		elapsedTime.restart();
		return;
	}
	//este exec no me convence, mientras estoy haciendo la accion no llamo
	if (exec == true)
		return;

	RoboCompBodyInverseKinematics::Pose6D target;
	target.x = 0.;
	target.y = 1000.;
	target.z = 200.;
	target.rx = 0.0;
	target.ry = -1.50;
	target.rz = 0;
	RoboCompBodyInverseKinematics::WeightVector weights;
	weights.x  = weights.y  = weights.z  = 1;
	weights.rx = weights.ry = weights.rz = 1;
	try
	{
		///pn

		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 100);
		exec =true;

	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::approachFinger(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::stateMachine()
{
	switch (currenState)
	{
	case STOP :
		stop();
		break;
	case CLOSEHAND :
		closeHand();
		break;
	case APPROACHFINGER :
		approachFinger();
		break;
	case TOUCHFINGER:
		touchFinger();
		break;
	case OPENHAND:
		openHand();
		break;
	case APPROACHHAND:
		approachHand();
		break;
	case GRASP:
		grasp();
		break;
	case TAKE:
		take();
		break;
	default:
		qDebug()<<"state not valid"<<currenState;
		break;
	}
}

///slot
void SpecificWorker::compute( )
{
	usleep(500000);

	printf("action: %s\n", action.c_str());
	if (action == "findobjectvisually")
	{
		//saccadic3D(0, 820, 280, 0, -1, 0);
		return;
	}
	else if (action == "graspobject" )
	{
// 		if (currenState==STOP )
// 		{
// 			currenState=CLOSEHAND;
// 			elapsedTime.start();
// 		}
		approachFinger();
		if (exec)
			ajusteFino();
		return;
	}
	else
	{
		printf("ignoring this action (%s)...\n", action.c_str());
	}
// 	stateMachine();
}

void SpecificWorker::slotStop()
{
	currenState=STOP;
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


		if (action == "findobjectvisually")
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

void SpecificWorker::ajusteFino()
{
	static int firstTime = 0;
	int32_t object = atoi(params["o"].value.c_str());

	// Make sure we have the robot in the model, otherwise there's nothing to do yet...
	int32_t robot = worldModel->getIdentifierByType("robot");


	float tx, ty, tz;
	float rwtx, rwty, rwtz;
	try
	{
		//habría que sacar las posiciones de las marcas del modelo, target y aprilWrist
		tx = str2float(worldModel->getSymbol(object)->getAttribute("tx"));
		ty = str2float(worldModel->getSymbol(object)->getAttribute("ty"));
		tz = str2float(worldModel->getSymbol(object)->getAttribute("tz"));
		//float rx = str2float(worldModel->getSymbol(object)->getAttribute("rx"));
		//float ry = str2float(worldModel->getSymbol(object)->getAttribute("ry"));
		//float rz = str2float(worldModel->getSymbol(object)->getAttribute("rz"));
		rwtx = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_tx"));
		rwty = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_ty"));
		rwtz = str2float(worldModel->getSymbol(robot)->getAttribute("rightwrist_tz"));
		//float rxw = str2float(robot->getAttribute("wrist_rx"));
		//float ryw = str2float(robot->getAttribute("wrist_ry"));
		//mfloat rzw = str2float(robot->getAttribute("wrist_rz"));
	}
	catch(...)
	{
		printf("right wrist position is not set yet... aborting\n");
		return;
	}
	//both are in the world coordinate system
	if (firstTime > 5 )
	{
		qDebug()<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeee";
		return;
	}
	firstTime++;
	QVec targetT = QVec::vec3(tx,  ty , tz);
	QVec wristT  = QVec::vec3(rwtx, rwty, rwtz);
	QVec poseTr =  targetT - wristT;
	targetT.print("targetT");
	wristT.print("wrist");
	poseTr.print("poseTr");
	float d = poseTr.norm2();
	QVec vNormal = poseTr.normalize();
	vNormal.print("vNormal");
	qDebug()<<"d"<<d;
	qDebug()<<"-----------";
	try
	{
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x=vNormal.x() ;axis.y=vNormal.y();axis.z=vNormal.z();
		bodyinversekinematics_proxy->advanceAlongAxis("RIGHTARM",axis, d/10.0);
		sleep(2);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::approachFinger(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}


void SpecificWorker::sendRightHandPose(QVec t, QVec r, QVec wt, QVec wr)
{
	sendRightHandPose(t, r, wt(0), wt(1), wt(2), wr(0), wr(1), wr(2));
}

void SpecificWorker::sendRightHandPose(QVec t, QVec r, float wtx, float wty, float wtz, float wrx, float wry, float wrz)
{
	sendRightHandPose(t(0), t(1), t(2), r(0), r(1), r(2), wtx, wty, wtz, wrx, wry, wrz);
}

void SpecificWorker::sendRightHandPose(float tx, float ty, float tz, float rx, float ry, float rz, float wtx, float wty, float wtz, float wrx, float wry, float wrz)
{
	RoboCompBodyInverseKinematics::Pose6D target;
	target.x = tx;
	target.y = ty;
	target.z = tz;
	target.rx = rx;
	target.ry = ry;
	target.rz = rz;
	RoboCompBodyInverseKinematics::WeightVector weights;
	weights.x = wtx;
	weights.y = wty;
	weights.z = wtz;
	weights.rx = wrx;
	weights.ry = wry;
	weights.rz = wrz;
	try
	{
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 100);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
}


void SpecificWorker::saccadic3D(QVec point, QVec axis)
{
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
	RoboCompBodyInverseKinematics::Pose6D targetSight;
	targetSight.x = tx;
	targetSight.y = ty;
	targetSight.z = tz;
	RoboCompBodyInverseKinematics::Axis axSight;
	axSight.x = axx;
	axSight.y = axy;
	axSight.z = axz;
	bool axisConstraint = false;
	float axisAngleConstraint = 0;
	try
	{
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", targetSight, axSight, axisConstraint, axisAngleConstraint);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
}




