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
#include <boost/concept_check.hpp>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	QMutexLocker l (mutex);
	RoboCompJointMotor::MotorGoalPositionList list;
	list.clear();
	RoboCompJointMotor::MotorGoalPosition p_goal1, p_goal2, p_goal3, p_goal4, p_goal5, p_goal6, p_goal7;

	p_goal1.name="head1";
	p_goal1.maxSpeed=0.5;
	p_goal1.position=0.0;

	p_goal2.name="head2";
	p_goal2.maxSpeed=0.5;
	p_goal2.position=0.0;
	
	p_goal3.name="head3";
	p_goal3.maxSpeed=0.5;
	p_goal3.position=0.0;
	
	p_goal4.name="rightWrist1";
	p_goal4.maxSpeed=0.5;
	p_goal4.position=0.0;
	
	p_goal5.name="rightWrist2";
	p_goal5.maxSpeed=0.5;
	p_goal5.position=0.0;
	
	p_goal6.name="leftWrist1";
	p_goal6.maxSpeed=0.5;
	p_goal6.position=0.0;
	
	p_goal7.name="leftWrist2";
	p_goal7.maxSpeed=0.5;
	p_goal7.position=0.0;
	
	
	list.push_back(p_goal1);
	list.push_back(p_goal2);
	list.push_back(p_goal3);
	list.push_back(p_goal4);
	list.push_back(p_goal5);
	list.push_back(p_goal6);
	list.push_back(p_goal7);
	
	
	try
	{
		jointmotor_proxy->setSyncPosition(list);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"error talking to jointmotor_proxy. We can't put to any motor"<<e.what();
	}
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};


void SpecificWorker::sendData(const TData& data)
{
	qDebug()<<"---";
	QMutexLocker l (mutex);
	RoboCompJointMotor::MotorGoalPositionList list;

// 	//rightFinger1 minPosL cerrao (sin cerrar del to), maxPosL abierto
	static float minPosL=-0.8;
	static float maxPosL=0;
	
	///rightFinger2 maxPosR abierto, minPosR cerrao (sin cerrar del to)
	static float minPosR=0.8;
	static float maxPosR=0;
	
	if (data.axes[0].name=="pan")
	{
		qDebug()<<"pan"<<data.axes[0].value;
		RoboCompJointMotor::MotorGoalPosition p_goal;
		p_goal.name="head3";
		p_goal.maxSpeed=0.5;
		p_goal.position=normalize(data.axes[0].value, -1., 1., -0.6,0.6);		
		
		list.push_back(p_goal);
	}
		
	if (data.axes[1].name=="tilt")
	{
		qDebug()<<"tilt"<<data.axes[1].value;
		RoboCompJointMotor::MotorGoalPosition p_goal;
		p_goal.name="head1";
		p_goal.maxSpeed=0.5;
		p_goal.position=normalize(data.axes[1].value, -1., 1., 0.6,-0.6);		
		
		list.push_back(p_goal);
	}
	//gatillo joystick
	if (data.buttons[0].clicked )
	{
		static bool open;
		qDebug()<<"activar pinza"<<"open?"<<open;
		RoboCompJointMotor::MotorGoalPosition p_goalLeft, p_goalRight, p_goalWrist1,p_goalWrist2;
		p_goalLeft.name="rightFinger1";
		p_goalLeft.maxSpeed=0.5;
		p_goalRight.name="rightFinger2";
		p_goalRight.maxSpeed=0.5;
			
		if (open)
		{
			p_goalLeft.position=minPosL;
			p_goalRight.position=minPosR;
			open = false;
		}
		else
		{
			p_goalLeft.position=maxPosL;
			p_goalRight.position=maxPosR;
			open = true;
		}
		qDebug()<<"p_goalLeft.position, p_goalRight.position"<<p_goalLeft.position<<p_goalRight.position;
		list.push_back(p_goalLeft);
		list.push_back(p_goalRight);
				
		
		///put to zero right wrist
				
		p_goalWrist1.name="rightWrist1";
		p_goalWrist1.maxSpeed=0.5;
		p_goalWrist1.position=0;
		
		p_goalWrist2.name="rightWrist2";
		p_goalWrist2.maxSpeed=0.5;
		p_goalWrist2.position=0;
		
		list.push_back(p_goalWrist1);
		list.push_back(p_goalWrist2);
		
		
	}
	
	
	///left button on the mini button
	if (data.buttons[7].clicked )		
	{
		qDebug()<<"mini"<<data.axes[1].value;
		qDebug()<<"data.axes[2]"<<data.axes[2].value;
		
		///limites fisicos cuando las pinzas están cerrados
		float minPosLeft=-1.6;
		float minPosRight=1.6;
		
		
		RoboCompJointMotor::MotorGoalPosition p_goalLeft, p_goalRight;
		p_goalLeft.name="rightFinger1";
		p_goalLeft.maxSpeed=0.5;
		p_goalRight.name="rightFinger2";
		p_goalRight.maxSpeed=0.5;
		
		p_goalLeft.position=normalize(data.axes[2].value, -1., 1., minPosLeft,maxPosL);
		p_goalRight.position=normalize(data.axes[2].value, -1., 1., minPosRight,maxPosR);
		qDebug()<<"p_goalLeft.position"<<p_goalLeft.position;
		qDebug()<<"p_goalRight.position"<<p_goalRight.position;
		
		
		minPosL=p_goalLeft.position;
		minPosR=p_goalRight.position;
		list.push_back(p_goalLeft);
		list.push_back(p_goalRight);
		
		
	}
	try
	{
		jointmotor_proxy->setSyncPosition(list);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"error talking to jointmotor_proxy"<<e.what();
	}
	
      
      
      if (data.buttons[10].clicked )		
      {
	qDebug()<<"boton 10";
	RoboCompJointMotor::MotorGoalPositionList list;
	list.clear();
	RoboCompJointMotor::MotorGoalPosition p_goal1, p_goal2, p_goal3, p_goal4, p_goal5, p_goal6, p_goal7;

	p_goal1.name="head1";
	p_goal1.maxSpeed=0.5;
	p_goal1.position=0.0;

	p_goal2.name="head2";
	p_goal2.maxSpeed=0.5;
	p_goal2.position=0.0;
	
	p_goal3.name="head3";
	p_goal3.maxSpeed=0.5;
	p_goal3.position=0.0;
	
	p_goal4.name="rightWrist1";
	p_goal4.maxSpeed=0.5;

	p_goal4.position=0.0;
	
	p_goal5.name="rightWrist2";
	p_goal5.maxSpeed=0.5;
	p_goal5.position=0.0;
	
	p_goal6.name="leftWrist1";
	p_goal6.maxSpeed=0.5;
	p_goal6.position=0.0;
	
	p_goal7.name="leftWrist2";
	p_goal7.maxSpeed=0.5;
	p_goal7.position=0.0;
	
	
	list.push_back(p_goal1);
	list.push_back(p_goal2);
	list.push_back(p_goal3);
	list.push_back(p_goal4);
	list.push_back(p_goal5);
	list.push_back(p_goal6);
	list.push_back(p_goal7);
	
	
	try
	{
		jointmotor_proxy->setSyncPosition(list);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"error talking to jointmotor_proxy. We can't put to zero any motor"<<e.what();
		
	}
    }
	
	///for debug
// 	qDebug()<<QString::fromStdString(data.id);
// 	qDebug()<<data.velAxisIndex<<data.dirAxisIndex;
// 	//buttons
// 	qDebug()<<"data.buttons";
// 	for (int i=0; i<data.buttons.size();i++)
// 		qDebug()<<"button"<<i<<data.buttons[i].clicked;
// 	//axis
// 	
// 	for (int i=0; i<data.axes.size();i++)
// 	{
// 		qDebug()<<"data.axes"<<i;
// 		qDebug()<<QString::fromStdString(data.axes[i].name);
// 		qDebug()<<data.axes[i].value;
// 	}
	
}

float SpecificWorker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}
