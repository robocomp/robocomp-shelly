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
	
	connect(&timerAutomataApproachFinger,SIGNAL(timeout()),this,SLOT(slotStop())),
	
	timerAutomataApproachFinger.setSingleShot(true);
	timerAutomataTouch.setSingleShot(true);
	timerAutomataApproachHand.setSingleShot(true);

	exec=false;

}

void SpecificWorker::stop()
{
    qDebug()<<"stop()"<<"exec"<<exec;
    timerAutomataApproachFinger.stop();
    timerAutomataTouch.stop();
    timerAutomataApproachHand.stop();
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
		bodyinversekinematics_proxy->setFingers(0.0);
		exec=true;
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::closeHand(): Error talking to bodyinversekinematics_proxy"<<e.what();
	}
}

void SpecificWorker::approachFinger()
{
	qDebug()<<"approachFinger()"<<"exec"<<exec<<"currenState"<<currenState<<"elapsedTime.elapsed()"<<elapsedTime.elapsed();
	int32_t object = atoi(params["o"].value.c_str());
	printf("go get %d\n", object);
	
// 	if (elapsedTime.elapsed()>2000)
// 	{
// 		qDebug()<<"Fingers should be close to target";
// // 		currenState=TOUCHFINGER;
// 		currenState=STOP;
// // 		exec=false;
// 		exec=true;
// 		elapsedTime.restart();
// 		return;
// 	}
// 	//este exec no me convence, mientras estoy haciendo la accion no llamo
// 	if (exec == true)
// 		return;
// 	
	try
	{
		float tx = str2float(worldModel->getSymbol(object)->getAttribute("tx"));
		float ty = str2float(worldModel->getSymbol(object)->getAttribute("ty"));
		float tz = str2float(worldModel->getSymbol(object)->getAttribute("tz"));
		float rx = str2float(worldModel->getSymbol(object)->getAttribute("rx"));
		float ry = str2float(worldModel->getSymbol(object)->getAttribute("ry"));
		float rz = str2float(worldModel->getSymbol(object)->getAttribute("rz"));

		QVec poseTr = innerModel->transform("world", QVec::vec3(tx, ty, tz), "robot");
		tx = poseTr(0);
		ty = poseTr(1);
		tz = poseTr(2);
		RoboCompBodyInverseKinematics::Pose6D target;
		target.x = tx;
		target.y = ty-70;
		target.z = tz;
		target.rx = rx-M_PI_2;
		target.ry = ry;
		target.rz = rz;
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x  = weights.y  = weights.z  = 1;
		weights.rx = 1;
		weights.ry = 0;
		weights.rz = 1;
		try
		{
			printf("gooooooo T=(%.2f, %.2f, %.2f)  R=(%.2f, %.2f, %.2f)\n", target.x, target.y, target.z, target.rx, target.ry, target.rz);
			bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);			
			exec =true;
			//timerAutomataApproachFinger.start(5000);
		}
		catch (Ice::Exception e)
		{
			qDebug()<<"SpecificWorker::approachFinger(): Error talking to bodyinversekinematics_proxy"<<e.what();
		}
	}
	catch(AGMModelException &e)
	{
		printf("I don't know about object %d\n", object);
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
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x=0.0;axis.y=0.;axis.z=1.0;
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
					
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);			
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
	printf("action: %s\n", action.c_str());
	if (action == "graspobject" )
	{		
// 		if (currenState==STOP )
// 		{
// 			currenState=CLOSEHAND;		
// 			elapsedTime.start();
// 		}
		approachFinger();
		return;
	}
	else
	{
		printf("ignoring this action...\n");
	}
	stateMachine();
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


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}