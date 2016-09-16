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

#ifndef ELASTICBAND_H
#define ELASTICBAND_H

#include <CommonBehavior.h>
#include <QtCore>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <innermodeldraw.h>
#include <Laser.h>
#include <limits>       
#include "waypoints.h"
#include <assert.h>
#include "currenttarget.h"
#include "linesimplifier/simplifyPath.h"

#define FORCE_DISTANCE_LIMIT (ROBOT_WIDTH * 2)  //mm
#define ROBOT_STEP (ROBOT_WIDTH * 0.1)
#define DELTA_H (ROBOT_WIDTH * 0.1)
#define ROAD_STEP_SEPARATION (ROBOT_LENGTH * 0.7)

/**
 * @brief This class computes laser-road force interaction, effectively projecting the "mental" road onto the physical world of distances
 * 
 */
class ElasticBand
{
	public:
		ElasticBand();
		~ElasticBand();
		void initialize(const RoboCompCommonBehavior::ParameterList& params);
		bool update(InnerModel* innermodel, WayPoints& road, const RoboCompLaser::TLaserData& laserData, const CurrentTarget& currentTarget, uint iter = 1);
		void addPoints(WayPoints &road, const CurrentTarget &currentTarget);

	private:		
		//float computeIntersectionChord( const WayPoint b1, const WayPoint b2);
		void computeDistanceField(InnerModel *innermodel, WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit);
		//bool computeFreePath(InnerModel* innermodel, const WayPoint& w1, const WayPoint& w2, const RoboCompLaser::TLaserData& laserData );
		//void checkBlocked(InnerModel* innermodel, WayPoints& road, const RoboCompLaser::TLaserData& laserData);
		//bool checkCollision(InnerModel* innermodel, WayPoints& road, const RoboCompLaser::TLaserData& laserData, float robotRadius);
		float computeForces(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData& laserData);
		void cleanPoints(WayPoints &road);
		bool checkVisiblePoints(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData);
		bool shortCut(InnerModel *innermodel, WayPoints& road, const RoboCompLaser::TLaserData &laserData);
		bool checkIfNAN(const WayPoints &road);
		bool checkCollisionAlongRoad(InnerModel *innermodel, const RoboCompLaser::TLaserData &laserData, WayPoints &road, WayPoints::const_iterator robot,
		                             WayPoints::const_iterator target, float robotRadius);

		simplifyPath simPath;

		// Points along robot's contour in robot's coordinate system
		QMat pointsMat;
		
		//Robot dimensions
		float ROBOT_LENGTH;
		float ROBOT_WIDTH;

};

#endif // ELASTICBAND_H
