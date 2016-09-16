/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "controller.h"

Controller::Controller(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, const RoboCompCommonBehavior::ParameterList &params, int delay)  //in secs
{
	this->time = QTime::currentTime();
	this->delay = delay*1000;	//msecs

	//compute offsets from laser center to the border of the robot base
	baseOffsets = computeRobotOffsets(innerModel, laserData);
	
	try
	{
		MAX_ADV_SPEED = QString::fromStdString(params.at("MaxZSpeed").value).toFloat();
		MAX_ROT_SPEED = QString::fromStdString(params.at("MaxRotationSpeed").value).toFloat();
		MAX_SIDE_SPEED = QString::fromStdString(params.at("MaxXSpeed").value).toFloat();
		MAX_LAG = std::stof(params.at("MinControllerPeriod").value);
		
		qDebug() << "SHIRTTTTTTTTT" << MAX_ADV_SPEED << MAX_ROT_SPEED << MAX_SIDE_SPEED;
		
	}
	catch (const std::out_of_range& oor) 
	{   std::cerr << "Controller. Out of Range error reading parameters: " << oor.what() << '\n'; }
}

Controller::~Controller()
{
}

bool Controller::update(InnerModel *innerModel, RoboCompLaser::TLaserData &laserData, RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy, WayPoints &road, bool print)
{
	static QTime reloj = QTime::currentTime();   //TO be used for a more accurate control (predictive).
	long epoch = 100;

	//Estimate the space that will be blindly covered and reduce Adv speed to remain within some boundaries
	//qDebug() << __FILE__ << __FUNCTION__ << "entering update with" << road.at(road.getIndexOfClosestPointToRobot()).pos;

	//////////////////////////////////////////////	
	// Check if robot goal is achieved already
	//////////////////////////////////////////////	
	if(road.isFinished() == true ) 
	{		
		qDebug() << __FUNCTION__ << "Controller: road finished. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
	if(road.requiresReplanning == true ) 
	{		
		qDebug() << __FUNCTION__ << "Controller: requiresReplanning. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
	if(road.isLost == true ) 
	{		
		qDebug() << __FUNCTION__ << "robot is lost. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
		

	/////////////////////////////////////////////////
	//////  CHECK CPU AVAILABILITY. If lagging reduce speed
	/////////////////////////////////////////////////
	if ( this->time.elapsed() > this->delay )   //Initial wait in secs so the robot waits for everything is setup. Maybe it could be moved upwards
	{
		if( epoch  > MAX_LAG )				//Damp max speed if elapsed time is too long  TAKE CONSTANT OUT!
		{
			MAX_ADV_SPEED = 200 * exponentialFunction(epoch-100, 200, 0.2);
			MAX_ROT_SPEED = 0.3 * exponentialFunction(epoch-100, 200, 0.2);
		}
		//float vadvance = 0;
		//float vrot = 0;
	}
	
	
	/////////////////////////////////////////////////
	///  SPEED DIRECTION AND MODULUS
	////////////////////////////////////////////////
	
	
	
	//Rotational speed 
	//Now we incorporate the rotational component without changing the advance speed
	//	- We want the most of the turn required to align with the road be made ASAP, so laser field becomes effective
	//	- We also want the final orientation is solved before arriving to the target, so a subsequent turniong action is avoided
	//	- Also, is the target is very close, <500mm, avoid turning to allow for small correcting displacements
	float vrot = 0;
	if( road.getRobotDistanceToTarget() > 500 )
	{
		vrot = 0.5 * road.getAngleWithTangentAtClosestPoint();
 		if(vrot > MAX_ROT_SPEED) vrot = MAX_ROT_SPEED;
 		if(vrot < -MAX_ROT_SPEED) vrot = -MAX_ROT_SPEED;
	}
	
	//Translational speed
	//We want the speed vector to align with the tangent to road at the current point.
	//	First get the radial line leaving the robot along the road:
	QLine2D radialLine = road.getTangentToCurrentPointInRobot(innerModel);
	
	//Normalize it into a unitary 2D vector
	QVec radialDir = radialLine.getNormalizedDirectionVector();
	
	//Now change sense and scale according to properties of the road and target
	float modulus = MAX_ADV_SPEED 
									* exp(-fabs(1.6 * road.getRoadCurvatureAtClosestPoint()))
									* exponentialFunction(1./road.getRobotDistanceToTarget(),1./500,0.5, 0.1)
									* exponentialFunction(vrot, 0.6, 0.01);
									
	radialDir = radialDir * (T)-modulus;
	
	//Next, decompose it into vadvance and vside componentes by projecting on robot's Z and X axis
	float vside = radialDir * QVec::vec2(1.,0.);
	float vadvance = radialDir * QVec::vec2(0.,1.);

	if(vadvance > MAX_ADV_SPEED) vadvance = MAX_ADV_SPEED;
	if(vadvance < -MAX_ADV_SPEED) vadvance = -MAX_ADV_SPEED;
	
	if(vside > MAX_SIDE_SPEED) vside = MAX_SIDE_SPEED;
	if(vside < -MAX_SIDE_SPEED) vside = -MAX_SIDE_SPEED;
	
	////////////////////////////////////////////////
	//////   Print control values
	////////////////////////////////////////////////
	
	if( print )
	{
		qDebug() << "------------------Controller Report ---------------;";
		qDebug() <<"	VAdv: " << vadvance << "|\nVRot: " << vrot << "\nVSide: " 
						<< vside << "\n Dist2Target: " << road.getRobotDistanceToTarget()
						<< "\n PerpDist: " << road.getRobotPerpendicularDistanceToRoad();
		qDebug() << "---------------------------------------------------;";
	}
	
	////////////////////////////////////////////////
	//////   EXECUTION
	////////////////////////////////////////////////
	
	try { omnirobot_proxy->setSpeedBase(vside, vadvance, vrot);}
	catch (const Ice::Exception &e) { std::cout << e << "Omni robot not responding" << std::endl; }

	epoch = reloj.restart();  //epoch time in ms
	return false;
}


////////////////////////////////////////
//// Auxiliary functions
///////////////////////////////////////

std::vector<float> Controller::computeRobotOffsets(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData)
{
	//Base geometry GET FROM IM!!!
	//QRectF base( QPointF(-200, 200), QPointF(200, -200));
	std::vector<float> baseOffsets;
	QVec p(3,0.f);
	int k;

	if(	innerModel->getNode("robot") == false or innerModel->getNode("laser") == false)	
	{
		qDebug() << __FUNCTION__ << "No laser or robot nodes in InnerModel. Aborting";
		throw;
	}

	for(auto i : laserData)
	{
		for(k = 10; k < 4000; k++)
		{
				p = innerModel->getNode<InnerModelLaser>("laser")->laserTo("robot", k, i.angle);
				if( sqrt(p.x()*p.x() + p.z()*p.z()) - 250 >= 0) 
				//if( base.contains( QPointF( p.x(), p.z() ) ) == false )
					break;
		}
		baseOffsets.push_back(k);
	}
	return baseOffsets;
}

void Controller::stopTheRobot(RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy)
{
	qDebug() << __FUNCTION__ << "Stopping the robot";
	try {	omnirobot_proxy->setSpeedBase( 0.f, 0.f, 0.f);	}
	catch (const Ice::Exception &e) { std::cout << e << std::endl;}
}

float Controller::exponentialFunction(float value, float xValue, float yValue, float min)
{
	Q_ASSERT( yValue>0 );

	float landa = -fabs(xValue) / log(yValue);
	//qDebug() << landa << value << value/landa << exp(-fabs(value)/landa);
	float res = exp(-fabs(value)/landa);
	if( res < min )
		return min;
	else
		return res;
}
