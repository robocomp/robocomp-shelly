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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	
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
	QMutexLocker l (mutex);
	RoboCompJointMotor::MotorGoalPositionList list;
	static float minPosLeft=-0.1;
	static float minPosRight=0.1;
	
	static float minPosL=-0.3;
	static float minPosR=0.3;
	
	if (data.axes[0].name=="pan")
	{
		qDebug()<<"pan"<<data.axes[0].value;
		RoboCompJointMotor::MotorGoalPosition p_goal;
		p_goal.name="pan";
		p_goal.maxSpeed=0.5;
		p_goal.position=normalize(data.axes[0].value, -1., 1., -0.6,0.5);		
		list.push_back(p_goal);
	}
		
	if (data.axes[1].name=="tilt")
	{
		qDebug()<<"tilt"<<data.axes[1].value;
		RoboCompJointMotor::MotorGoalPosition p_goal;
		p_goal.name="tilt";
		p_goal.maxSpeed=0.5;
		p_goal.position=normalize(data.axes[1].value, -1., 1., -0.2,0.2);		
		list.push_back(p_goal);
	}
	//gatillo joystick
	if (data.buttons[0].clicked )
	{
		static bool open;
		qDebug()<<"activar pinza"<<"open?";
		RoboCompJointMotor::MotorGoalPosition p_goalLeft, p_goalRight;
		p_goalLeft.name="pinzaLeft";
		p_goalLeft.maxSpeed=0.5;
		p_goalRight.name="pinzaRight";
		p_goalRight.maxSpeed=0.5;
		
		float maxPosL=-0.9;
		//float minPosL=-0.3;
		
		float maxPosR=0.7;
		
		
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
		
	}
	if (data.buttons[7].clicked )		
	{
		qDebug()<<"mini"<<data.axes[1].value;
		float maxPosL=-0.9;
		float maxPosR=0.7;
		
		
		RoboCompJointMotor::MotorGoalPosition p_goalLeft, p_goalRight;
		p_goalLeft.name="pinzaLeft";
		p_goalLeft.maxSpeed=0.5;
		p_goalRight.name="pinzaRight";
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
