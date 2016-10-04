/*
 * Copyright 2013 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "elasticband.h"

ElasticBand::ElasticBand()
{}

void ElasticBand::initialize( const RoboCompCommonBehavior::ParameterList& params )
{
	ROBOT_WIDTH =  QString::fromStdString(params.at("RobotXWidth").value).toFloat();
	ROBOT_LENGTH =  QString::fromStdString(params.at("RobotZLong").value).toInt();
	ROBOT_RADIUS =  QString::fromStdString(params.at("RobotRadius").value).toFloat();
	ATRACTION_FORCE_COEFFICIENT = 0.3;  //PARAMS
 	REPULSION_FORCE_COEFFICIENT = 1.;
	
	qDebug() << __FUNCTION__ << "Robot dimensions in ElasticBand: " << ROBOT_WIDTH  << ROBOT_LENGTH;
	
	
	////////////////////////////
	/// points if the frontal edge of the robot used to test robot's hypothetical positions in the laser field
	////////////////////////////

	pointsMat = QMat::zeros(3,4);

	//front edge
	pointsMat(0,0) = -ROBOT_WIDTH/2;
	pointsMat(0,2) =  ROBOT_LENGTH/2;
	pointsMat(0,3) =  1.f;
	pointsMat(1,0) =  ROBOT_WIDTH/2;
	pointsMat(1,2) =  ROBOT_LENGTH/2;
	pointsMat(1,3) =  1.f;
	pointsMat(2,0) =  0;
	pointsMat(2,2) =  ROBOT_LENGTH/2;
	pointsMat(2,3) =  1.f;

	//lower
	// 	pointsMat(3,0) = -xs/2;
	// 	pointsMat(3,2) = -zs/2;
	// 	pointsMat(3,3) =  1.f;
	// 	pointsMat(4,0) =  xs/2;
	// 	pointsMat(4,2) = -zs/2;
	// 	pointsMat(4,3) =  1.f; 
	// 	pointsMat(5,0) =  0;
	// 	pointsMat(5,2) = -zs/2;
	// 	pointsMat(5,3) =  1.f;
	//middle
	// 	pointsMat(6,0) = -xs/2;
	// 	pointsMat(6,2) =  0;
	// 	pointsMat(6,3) =  1.f;
	// 	pointsMat(7,0) =  xs/2;
	// 	pointsMat(7,2) =  0;
	// 	pointsMat(7,3) =  1.f;

	pointsMat = pointsMat.transpose();
}

ElasticBand::~ElasticBand()
{
}

bool ElasticBand::update(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData,
                         const CurrentTarget &currentTarget, uint iter)
{
	//qDebug() << __FILE__ << __FUNCTION__ << "road size"<<  road.size();
	if (road.isFinished() == true)
		return false;

	/////////////////////////////////////////////
	//Tags all points in the road ar visible or blocked, depending on laser visibility. Only visible points are processed in this iteration
	/////////////////////////////////////////////
	checkVisiblePoints(innermodel, road, laserData);

	/////////////////////////////////////////////
	//Check if there is a sudden shortcut to take
	/////////////////////////////////////////////
	//shortCut(innermodel, road, laserData);

	/////////////////////////////////////////////
	//Add points to achieve an homogenoeus chain
	/////////////////////////////////////////////
	addPoints(road, currentTarget);

	/////////////////////////////////////////////
	//Remove point too close to each other
	/////////////////////////////////////////////
	cleanPoints(road);

	/////////////////////////////////////////////
	//Compute the scalar magnitudes
	/////////////////////////////////////////////
	computeForces(innermodel, road, laserData);

	/////////////////////////////////////////////
	//Delete half the tail behind, if greater than 6, to release resources
	/////////////////////////////////////////////
	if (road.getIndexOfClosestPointToRobot() > 2)
	{
		for (auto it = road.begin(); it != road.begin() + (road.getIndexOfCurrentPoint() / 2); ++it)
			road.backList.append(it->pos);
		road.erase(road.begin(), road.begin() + (road.getIndexOfCurrentPoint() / 2));
	}
	return true;
}

/**
 * @brief Check if some point ahead on the road is closer (L2) than along the road, to take a shortcut
 * 
 * @param innermodel ...
 * @param road ...
 * @param laserData ...
 * @return bool
 */
bool ElasticBand::shortCut(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	//Compute distances from robot to all points ahead. If any of them is laser-visible and  significantly shorter than de distance along de road, try it!
	WayPoints::iterator robot = road.getIterToClosestPointToRobot();
	WayPoints::iterator best = road.begin();
	for (WayPoints::iterator it = robot + 1; it != road.end(); ++it)
	{
		//qDebug() << __FUNCTION__ << it->isVisible << (it->pos - robot->pos).norm2() << road.computeDistanceBetweenPointsAlongRoad(robot, it);
		if ( it->isVisible )
		{
			if (road.computeDistanceBetweenPointsAlongRoad(robot, it) - (it->pos - robot->pos).norm2() >  300)  //Half robot SACARRRR
			{
				qDebug() << __FUNCTION__ << "Candidato";
				//Check if the robot passes through the straight line
				if (checkCollisionAlongRoad(innermodel, laserData, road, robot, it, ROBOT_RADIUS))
				{
					//Is so remove all intermadiate points between robot and new subtarget
					qDebug() << __FUNCTION__ << "Confirmado";
					best = it;
				}
			}
		}
		else
			break;
	}
	if (best != road.begin() and (robot + 1) != road.end())
		road.erase(robot + 1, best);
	return false;
}

/**
 * @brief Adds points to the band if two existing ones are too far apart (ROBOT_RADIUS)
 * 
 * @param road ...
 * @return void
 */
bool ElasticBand::addPoints(WayPoints &road, const CurrentTarget &currentTarget)
{
	if( road.size() < 2) 
		return false;

	int offset = 1;
	for (int i = 0; i < road.size() - offset; i++)
	{
		if (i > 0 and road[i].isVisible == false)			//This might not work with visible detection
			break;

		WayPoint &w = road[i];
		WayPoint &wNext = road[i + 1];
		float dist = (w.pos - wNext.pos).norm2();
		if (dist > ROAD_STEP_SEPARATION)  
		{
			float l = 0.9 * ROAD_STEP_SEPARATION / dist;   //Crucial que el punto se ponga mas cerca que la condición de entrada
			WayPoint wNew((w.pos * (1 - l)) + (wNext.pos * l));
			road.insert(i + 1, wNew);
		}
	}
	return true;
}


bool ElasticBand::cleanPoints(WayPoints &road)
{
	int offset = 2;	// to avoid deleting the last (target) point in the road.
	if( road.size() - offset < 2) 
		return false;

	int i;

	for (i = 0; i < road.size() - offset; i++)
	{
		if (road[i].isVisible == false)
			break;
		WayPoint &w = road[i];
		WayPoint &wNext = road[i + 1];

		float dist = (w.pos - wNext.pos).norm2();
		if (dist < ROAD_STEP_SEPARATION / 2.)									//TAKE TO PARAMS
		{
			road.removeAt(i + 1);
		}
	}
	return true;
}


bool ElasticBand::checkVisiblePoints(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	//Simplify laser polyline using Ramer-Douglas-Peucker algorithm
	std::vector<Point> points, res;
	QVec wd;
	for (auto &ld : laserData)
	{
		//wd = innermodel->laserTo("world", "laser", ld.dist, ld.angle); //OPTIMIZE THIS FOR ALL CLASS METHODS
		wd = innermodel->getNode<InnerModelLaser>("laser")->laserTo("world", ld.dist, ld.angle);
		points.push_back(Point(wd.x(), wd.z()));
	}
	res = simPath.simplifyWithRDP(points, 70);  ///PARAMS
	//qDebug() << __FUNCTION__ << "laser polygon after simp" << res.size();

	// Create a QPolygon so we can check if robot outline falls inside
	QPolygonF polygon;
	for (auto &p: res)
		polygon << QPointF(p.x, p.y);

	// Move the robot along the road
	int robot = road.getIndexOfNextPoint();
	QVec memo = innermodel->transform6D("world", "robot");
	for(int it = robot; it<road.size(); ++it)
	{
		road[it].isVisible = true;
		innermodel->updateTransformValues("robot", road[it].pos.x(), road[it].pos.y(), road[it].pos.z(), 0, road[it].rot.y(), 0);
		//get Robot transformation matrix
		QMat m = innermodel->getTransformationMatrix("world", "robot");
		// Transform all points at one to world RS
		//m.print("m");
		//pointsMat.print("pointsMat");
		QMat newPoints = m * pointsMat;

		//Check if they are inside the laser polygon
		for (int i = 0; i < newPoints.nCols(); i++)
		{
			if (polygon.containsPoint(QPointF(newPoints(0, i), newPoints(2, i)),Qt::OddEvenFill) == false)
			{
				road[it].isVisible = false;
				//qFatal("fary");
				break;
			}
		}
		//		if( road[it].isVisible == false)
		//		{
		//			for (int k = it; k < road.size(); ++k)
		//				road[k].isVisible = false;
		//			break;
		//		}
	}

	// Set the robot back to its original state
	innermodel->updateTransformValues("robot", memo.x(), memo.y(), memo.z(), 0, memo.ry(), 0);

	//road.print();
	return true;
}

float ElasticBand::computeForces(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	if (road.size() < 3)
		return 0;

	// Go through all points in the road
	float totalChange = 0.f;
	for (int i = 1; i < road.size()-1; i++)
	{
	
		WayPoint &w0 = road[i - 1];
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i + 1];

		// Atraction force caused by the trajectory stiffnes, trying to straighten itself. It is computed as a measure of local curvature
		//QVec atractionForce(3);
		//float n = (w0.pos - w1.pos).norm2() / ((w0.pos - w1.pos).norm2() + w1.initialDistanceToNext);
		//atractionForce = (w2.pos - w0.pos) * n - (w1.pos - w0.pos);
		
		QVec atractionForce = QVec::zeros(3);
		atractionForce = ((w2.pos - w0.pos) - (w1.pos - w0.pos));
		//float atractionMod = ((w2.pos - w0.pos) - (w1.pos - w0.pos)).norm2();
		//atractionForce = ((((w2.pos - w0.pos)/(T)2.) + w0.pos) - w1.pos) * (T)atractionMod;
// 		w0.pos.print("w0");
// 		w1.pos.print("w1");
// 		w2.pos.print("w2");
// 		atractionForce.print("att");
		//qDebug() << __FUNCTION__ << "ATT --------------" << atractionMod;
		
		QVec repulsionForce = QVec::zeros(3);
		if (road[i].isVisible)
		{	
			//Compute derivative of force field and store values in w1.bMinuxX .... and w1.minDist. Also variations wrt former epochs
			computeDistanceField(innermodel, w1, laserData, FORCE_DISTANCE_LIMIT);

			
			QVec jacobian(3);

			// space interval to compute the derivative. Related to to robot's size
			float h = DELTA_H;
			if ((w1.minDistHasChanged == true) /*and (w1.minDist < 250)*/ )
			{
				jacobian = QVec::vec3(w1.bMinusX - w1.bPlusX,
															0,
															w1.bMinusY - w1.bPlusY) * (T) (1.f / (2.f * h));

				// repulsion force is computed in the direction of maximun laser-point distance variation and scaled so it is 0 is beyond FORCE_DISTANCE_LIMIT and FORCE_DISTANCE_LIMIT if w1.minDist.
				repulsionForce = jacobian * (T)(FORCE_DISTANCE_LIMIT - w1.minDist);

			}
		}
		
		// ATRACTION_FORCE_COEFFICIENT negative values between 0 and -1. The bigger in magnitude, the stiffer the road becomes PARAM
		// REPULSION_FORCE_COEFFICIENT Positive values between 0 and 1	 The bigger in magnitude, more separation from obstacles

		ATRACTION_FORCE_COEFFICIENT = -0.3;  //PARAMS
		REPULSION_FORCE_COEFFICIENT = 1.;
		
		QVec change = (atractionForce * ATRACTION_FORCE_COEFFICIENT) + (repulsionForce * REPULSION_FORCE_COEFFICIENT);
		
		if (std::isnan(change.x()) or std::isnan(change.y()) or std::isnan(change.z()))
		{
			road.print();
			qDebug() << atractionForce << repulsionForce;
			return(0);
		}
		
		//Now we remove the tangencial component of the force to avoid recirculation of band points
		//QVec pp = road.getTangentToCurrentPoint().getPerpendicularVector();
		//QVec nChange = pp * (pp * change);

		w1.pos = w1.pos - change;
		totalChange = totalChange + change.norm2();
	}
	return totalChange;
}

void ElasticBand::computeDistanceField(InnerModel *innermodel, WayPoint &ball, const RoboCompLaser::TLaserData &laserData,
                                  float forceDistanceLimit)
{

	ball.minDist = ball.bMinusX = ball.bPlusX = ball.bMinusY = ball.bPlusY = std::numeric_limits<float>::max();
	ball.minDistHasChanged = false;

	QVec c = ball.pos;
	c[1] = innermodel->getNode("laser")->getTr()[1]; //Put the y coordinate to laser height so norm() works allright
	int index = -1;

	for (uint i = 0; i < laserData.size(); i++)
	{
		QVec l = innermodel->laserTo("world", "laser", laserData[i].dist, laserData[i].angle);

		float dist = (l - c).norm2();

		if (dist < ball.minDist)
		{
			ball.minDist = dist;
			index = i;
		}

		float h = DELTA_H; //Delta
		dist = (l - (c - QVec::vec3(h, 0, 0))).norm2();
		if (dist < ball.bMinusX) ball.bMinusX = dist;

		dist = (l - (c + QVec::vec3(h, 0, 0))).norm2();
		if (dist < ball.bPlusX) ball.bPlusX = dist;

		dist = (l - (c - QVec::vec3(0, 0, h))).norm2();
		if (dist < ball.bMinusY) ball.bMinusY = dist;

		dist = (l - (c + QVec::vec3(0, 0, h))).norm2();
		if (dist < ball.bPlusY) ball.bPlusY = dist;
	}

	QVec lw = innermodel->laserTo("world", "laser", laserData[index].dist, laserData[index].angle);
	ball.minDistPoint = (lw - c).normalize();

	//correct minDist to take into account the size of the robot in the ball.minDistPoint direction. For now let's suposse it is a ball
	ball.minDistPoint -= ROBOT_RADIUS;

	if (ball.minDist < forceDistanceLimit)
	{
		if (fabs(ball.minDist - ball.minDistAnt) > 2) //mm
		{
			ball.minDistAnt = ball.minDist;
			ball.minDistHasChanged = true;
		}
	}
	else
		ball.minDist = forceDistanceLimit;
}

 bool ElasticBand::checkCollisionAlongRoad(InnerModel *innermodel, const RoboCompLaser::TLaserData& laserData, WayPoints &road,  WayPoints::const_iterator robot,
                                            WayPoints::const_iterator target, float robotRadius)
 {
	//Simplify laser polyline using Ramer-Douglas-Peucker algorithm
	std::vector<Point> points, res;
	QVec wd;
	for( auto &ld : laserData)
	{
		wd = innermodel->laserTo("world", "laser", ld.dist, ld.angle);      //OPTIMIZE THIS FOR ALL CLASS METHODS
		points.push_back(Point(wd.x(), wd.z()));
	}
	res = simPath.simplifyWithRDP(points, 70);
	qDebug() << __FUNCTION__ << "laser polygon after simp" << res.size();

	// Create a QPolygon so we can check if robot outline falls inside
	QPolygonF polygon;
	for (auto &p: res)
		polygon << QPointF(p.x, p.y);

	// Move the robot along the road
	QVec memo = innermodel->transform6D("world","robot");
	bool free = false;
	for( WayPoints::const_iterator it = robot; it != target; ++it)
	{
		if( it->isVisible == false)
			break;
		// compute orientation of the robot at the point

		innermodel->updateTransformValues("robot", it->pos.x(), it->pos.y(), it->pos.z(), 0, it->rot.y(), 0);
		//get Robot transformation matrix
		QMat m = innermodel->getTransformationMatrix("world", "robot");
		// Transform all points at one
		qDebug() << __FUNCTION__ << "hello2";
		m.print("m");
		pointsMat.print("pointsMat");
		QMat newPoints = m * pointsMat;
		qDebug() << __FUNCTION__ << "hello3";

		//Check if they are inside the laser polygon
		for( int i=0; i<newPoints.nRows(); i++)
			if( polygon.containsPoint(QPointF(pointsMat(i,0)/pointsMat(i,3), pointsMat(i,2)/pointsMat(i,3)), Qt::OddEvenFill ) == false)
			{
				free = false;
				break;
			}
		free = true;
	}
	 qDebug() << __FUNCTION__ << "hello";

	 // Set the robot back to its original state
	innermodel->updateTransformValues("robot", memo.x(), memo.y(), memo.z(), 0, memo.ry(), 0);
	return free ? true : false;
 }

bool ElasticBand::checkIfNAN(const WayPoints &road)
{
	for (auto it = road.begin(); it != road.end(); ++it)
		if (std::isnan(it->pos.x()) or std::isnan(it->pos.y()) or std::isnan(it->pos.z()))
		{
			road.print();
			return true;
		}
	return false;
}








