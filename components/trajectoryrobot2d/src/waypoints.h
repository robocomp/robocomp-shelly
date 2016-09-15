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

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <CommonBehavior.h>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <iostream>
#include <fstream>
//#include <innermodeldraw.h>
#include <float.h>
#include "qline2d.h"
#include "currenttarget.h"

#define ROBOT_RADIUS 250   //GET FROM XML

class WayPoint
{
	public:
		WayPoint()			{ pos = QVec::zeros(3); minDist = ROBOT_RADIUS; minDistAnt = 0.f; isVisible = true; minDistPoint = QVec::zeros(3); hasRotation = false;};
		WayPoint(QVec p) 	{ pos = p; minDist = ROBOT_RADIUS; minDistAnt = 0.f; isVisible = true; minDistPoint = QVec::zeros(3); hasRotation = false;};
		~WayPoint()			{};
	
		//For ElasticBand
		QVec pos;								// 3D point (x,y,z)
		QVec rot;								// Euler angles (0,ry,0)
		float minDist, minDistAnt;
		QVec minDistPoint; //In world ref system
		float bMinusY, bPlusY, bMinusX, bPlusX;
		bool minDistHasChanged;
		QString centerTransformName, centerMeshName, centerLineName, centerPointName, ballTransformName, ballMeshName;
		bool isVisible;
		float initialDistanceToNext;
		float visibleLaserAngle;
		float visibleLaserDist;
		QVec posInRobotFrame;
		bool hasRotation;
		
}; 

class WayPoints : public QList< WayPoint >
{
	public:
		WayPoints();
		~WayPoints();
		void reset();
		void startRoad();
		void endRoad();
		void setThreshold(const float _threshold) 																												{ QMutexLocker ml(&mutex); threshold = _threshold;};
		void initialize( InnerModel *inner, const RoboCompCommonBehavior::ParameterList &params);
		void readRoadFromFile(InnerModel *innerModel, std::string name);
		void readRoadFromList(QList<QVec> list);
		void printRobotState(InnerModel* innerModel, const CurrentTarget& currentTarget);
		void print() const;
		QList<QVec> backList;
		
		/**
		* @brief Computes all scalar values defining the interaction of the Robot and the Road. After executing update, all variables are ready to be used 
		* through thier get methods
		* @return void
		*/
		void update();
		
		/**
		 * @brief computes robot distance to current active point in the road
		 * 
		 * @param innerModel ...
		 * @return float
		*/
		float robotDistanceToCurrentPoint(InnerModel *innerModel);
		
		/**
		 * @brief computes robot distance to the point after the current active one in the road
		 * @param innerModel ...
		 * @return float
		*/
		float robotDistanceToNextPoint(InnerModel *innerModel);
		
		/**
		 * @brief computes distance between two points along the road
		 * @param innerModel ...
		 * @return float
		*/
		float computeDistanceBetweenPointsAlongRoad(WayPoints::iterator firstPoint, WayPoints::iterator secondPoint);
		
		/**
		* @brief provides the robot front pointin axis as a QLine2D in WRS
		* @param innerModel ...
		* @return QLine2D
		*/
		QLine2D getRobotZAxis(InnerModel* innerModel);
		
		/**
		* @brief Computes, for each point in the road, its distance in mm to the next point and stores it in the WayPoint variable initialDistanceToNext;
		* @param innerModel ...
		* @return QLine2D
		*/
		void computeDistancesToNext();
		
		/**
		* @brief Computes de line joinning road's currentpoint and nextcurrentpoint, aproximating the tangent at currentpoint
		* 	
		* @return QLine2D in WORLD system of reference
		*/
		QLine2D getTangentToCurrentPoint();
		
		/**
		* @brief Computes de line joinning road's currentpoint and nextcurrentpoint, aproximating the tangent at currentpoint
		* 	
		* @return QLine2D in ROBOT system of reference
		*/
		QLine2D getTangentToCurrentPointInRobot(InnerModel *innerModel);
		/**
		 * @brief returns a WayPoints iterator to the closest point in the road to the robot
		 * 
		 * @return WayPoints::iterator
		*/
		WayPoints::iterator getIterToClosestPointToRobot() const 	      					{ return iterToClosestPointToRobot;};
		
		/**
		* @brief Computes the tangent to the robot at the road's closest point to it.
		* 	
		* @return QLine2D in world system of reference
		*/
		QLine2D getTangentAtClosestPoint() const																	{ return roadTangentAtClosestPoint;};
		
		/**
		* @brief Computes the distance from the robot to the closest point of the road. 
		* 	
		* @return float distance in mm
		*/
		float getRobotDistanceToClosestPoint() 	const															{ return robotDistanceToClosestPoint;};
		
		/**
		* @brief Computes the perpendicular distance from the robot to road. 
		* 	
		* @return float distance in mm
		*/
		float getRobotPerpendicularDistanceToRoad()	const													{ return robotPerpendicularDistanceToRoad;};
		QLine2D roadTangentAtClosestPoint;
		float getAngleWithTangentAtClosestPoint() const														{ return angleWithTangentAtClosestPoint;};
		uint getIndexOfCurrentPoint() const																				{ return indexOfCurrentPoint;};
		uint getIndexOfNextPoint() const																					{ return indexOfNextPoint;};
		WayPoint getCurrentPoint() const   	    							                		{ return (*this)[indexOfCurrentPoint];};
		WayPoint getNextPoint() const	    							                 					{ return (*this)[indexOfNextPoint];};
		float getRoadCurvatureAtClosestPoint() const															{ return roadCurvatureAtClosestPoint;};
		float getRobotDistanceToTarget() const																		{ return robotDistanceToTarget;};
		float getRobotDistanceToLastVisible() const																{ return robotDistanceToLastVisible;};
		float getRobotDistanceVariationToTarget() const 													{ return robotDistanceVariationToTarget;};
		ulong getETA() const 																											{ return estimatedTimeOfArrival;};
		WayPoints::iterator getIterToLastVisiblePoint() const											{ return iterToLastVisiblePoint;};
		uint32_t getIndexOfClosestPointToRobot() const														{ return indexOfClosestPointToRobot;};
		bool isBlocked() const																										{ return blockedRoad;};
		bool isFinished() const                                                   { return finish;};
		bool isVisible(int i) const  																					    { if( i>=0 and i< this->size()) return (*this)[i].isVisible; else return false;};
		void setFinished( bool b)																									{ QMutexLocker ml(&mutex); finish = b; }
		void setBlocked(bool b)																										{ blockedRoad = b;};
		
		int indexOfNextPoint = 1;
		bool blockedRoad = false;
		bool isLost = false;
		int currentCollisionIndex = 0;
		float currentDistanceToFrontier = 0;
		bool requiresReplanning = false;
	
	private:
		float robotDistanceToClosestPoint = 0.f;
		float robotPerpendicularDistanceToRoad = 0.f;
		WayPoints::iterator iterToClosestPointToRobot, iterToLastVisiblePoint;
		uint32_t indexOfClosestPointToRobot = 0;
		uint indexOfCurrentPoint = 0;
		float angleWithTangentAtClosestPoint = 0.f;
		float roadCurvatureAtClosestPoint = 0.f;
		float robotDistanceToTarget = 0.f;
		float robotDistanceVariationToTarget = 0.f;
		float robotDistanceToLastVisible = 0.f;
		float threshold = 20.f;  //Default tolerance on arrival
		bool finish = false;
		ulong estimatedTimeOfArrival = 0;
		InnerModel *innerModel;
		QTime reloj;
		float meanSpeed = 200;  
		long elapsedTime = 0;
		int initialDurationEstimation = 0;
		float antDist = std::numeric_limits<float>::max(); 

		void setclosestPointToRobot(WayPoints::iterator it) 				    							{ iterToClosestPointToRobot = it;};
		void setTangentAtClosestPoint(const QLine2D &tangent) 												{ roadTangentAtClosestPoint = tangent;};
		void setRobotDistanceToClosestPoint(float dist) 															{ robotDistanceToClosestPoint = dist;};
		void setRobotPerpendicularDistanceToRoad(float dist) 													{ robotPerpendicularDistanceToRoad = dist;};
		void setAngleWithTangentAtClosestPoint( float ang)														{ angleWithTangentAtClosestPoint = ang;};
		void setRoadCurvatureAtClosestPoint( float c)																	{ roadCurvatureAtClosestPoint = c;};
		void setRobotDistanceToTarget( float dist)																		{ robotDistanceToTarget = dist;};
		void setRobotDistanceToLastVisible( float dist)																{ robotDistanceToLastVisible = dist;};
		void setRobotDistanceVariationToTarget(float dist)														{ robotDistanceVariationToTarget = dist;};
		void changeTarget(const QVec &target)																					{ QMutexLocker ml(&mutex); replace(length()-1, target); antDist = std::numeric_limits< float >::max();};
		void setETA();

		QMutex mutex;
		
		float computeRoadCurvature(WayPoints::iterator closestPoint, uint pointsAhead);
		float computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos);
		float computeDistanceToLastVisible(WayPoints::iterator closestPoint, const QVec &robotPos);

		/**
		* @brief Computes the road tangent at point pointed by iterator w.
		* @param w WayPoints::iterator pointing to the point of interest
		* @return QLine2D
		*/
		QLine2D computeTangentAt(WayPoints::iterator w) const;
		WayPoints::iterator computeClosestPointToRobot(const QVec& robot);
};

#endif // WAYPOINTS_H
