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

#define FORCE_DISTANCE_LIMIT (ROBOT_WIDTH*1.5)  //mm
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
		bool addPoints(WayPoints &road, const CurrentTarget &currentTarget);

	private:		
		
		/**
		* @brief Computes de numerical derivative of the laser force field wrt to the point, by perturbing it locally.
		* @param innermodel
		* @param ball, point of the trajectory to be analyzed
		* @param laserData
		* @param forceDistanceLimit, max effective action of the force field.
		*/
		void computeDistanceField(InnerModel *innermodel, WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit);
		
		/**
		* @brief Computes the forces exerted on the elements of the road and updates it following a simplified version on Newton physics
		*	 
		* @param innerModel ...
		* @param road ...
		* @param laserData ...
		* @return float total force exerted on the trajectory as the sum of individual changes
		*/
		float computeForces(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData& laserData);
		
		/**
		* @brief Removes points from the band if two of them are too close, ROBOT_RADIUS/3.
		* 
		* @param road ...
		* @return void
		*/
		bool cleanPoints(WayPoints &road);
		
		/**
		* @brief A point of the road is visible if it is between the robot and the laser beam running through it, and if the previous point was visible
		* All points in the road are updated
		* @param road ...
		* @param laserData ...
		* @return bool
		*/
		bool checkVisiblePoints(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData);
		
		bool shortCut(InnerModel *innermodel, WayPoints& road, const RoboCompLaser::TLaserData &laserData);
		
		/**
		* @brief Check if any of the waypoints has nan coordinates
		* 
		* @param road ...
		* @return bool
		*/
		bool checkIfNAN(const WayPoints &road);
		
		/**
		* @brief Moves a virtual copy of the robot along the road checking for enough free space around it
		* 
		* @param innermodel ...
		* @param road ...
		* @param laserData ...
		* @param robotRadius ...
		* @return bool
		*/
		bool checkCollisionAlongRoad(InnerModel *innermodel, const RoboCompLaser::TLaserData &laserData, WayPoints &road, WayPoints::const_iterator robot,
		                             WayPoints::const_iterator target, float robotRadius);

		simplifyPath simPath;

		// Points along robot's contour in robot's coordinate system
		QMat pointsMat;
		
		//Robot dimensions
		float ROBOT_LENGTH;
		float ROBOT_WIDTH;
		float ROBOT_RADIUS;
		float ATRACTION_FORCE_COEFFICIENT;
		float REPULSION_FORCE_COEFFICIENT;

};

#endif // ELASTICBAND_H
