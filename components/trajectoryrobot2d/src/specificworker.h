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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <iostream>
#include <fstream>
#include "controller.h"
#include "elasticband.h"
#include "waypoints.h"
#include "plannerompl.h"
#include "plannerprm.h"
#include "currenttarget.h"
#include "sampler.h"
#include "graphdraw.h"
#include "waypointsdraw.h"

#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif

/**
       \brief Component that plans and executes a trajectory for an omnidirectional robot. The trajectory es planned using a combination of PRM and RRT and 
       projected onto the outside world through the laser field. The projection allows for real-time dynamic corrections. A controller drives the robot
       along the trajectory.
       @author Pablo Bustos
*/

/**
 * @brief ...Auxiliary class to keep the state of the algorithm and make it accesible to the middleware
 * 
 */
class TrajectoryState
{
	public:
		TrajectoryState() { state = "IDLE";}
		~TrajectoryState() {};
		void setElapsedTime(long t){ QMutexLocker l(&m); elapsedTime = t;};
		void setEstimatedTime(long t){ QMutexLocker l(&m); estimatedTime = t;};
		void setPlanningTime(long t){ QMutexLocker l(&m); planningTime = t;};
		void setState(const std::string &state_){ state = state_;}
		void setDescription(const std::string &desc_){ description = desc_;}
		long getElapsedTime(){ QMutexLocker l(&m); return elapsedTime;};
		long getEstimatedtime(){ QMutexLocker l(&m); return estimatedTime;};
		long getPlanningTime(){ QMutexLocker l(&m); return planningTime;};
		std::string getState(){ QMutexLocker l(&m); return state;};
		RoboCompTrajectoryRobot2D::NavState toMiddleware( const RoboCompOmniRobot::TBaseState &bState, const WayPoints &road)
		{
			QMutexLocker l(&m);
			RoboCompTrajectoryRobot2D::NavState n;
			n.state = state;
			n.description = description;
			n.elapsedTime = elapsedTime;
			n.planningTime = planningTime;
			n.estimatedTime = estimatedTime;
			//////////// MORE DATA CAN GO HERE LIKE robot position, dist to targt, current road, etc
			n.x = bState.x;
			n.z = bState.z;
			n.ang = bState.alpha;
			n.advV = bState.advVz;
			n.rotV = bState.rotV;
			n.distanceToTarget = road.getRobotDistanceToTarget();
			return n;
		};
	private:
		QMutex m;
		long elapsedTime = 0;
		long estimatedTime = 0;
		long planningTime = 0;
		std::string state, description;
};


class SpecificWorker : public GenericWorker
/**
 * @brief Main class of the component
 * 
 */
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx, QWidget *parent = 0);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		////////////////////////////////////////
		//SERVANTS ATTENDING EXTERNAL INTERFACE
		////////////////////////////////////////
		RoboCompTrajectoryRobot2D::NavState	getState();
		float goBackwards(const TargetPose &target);
		void stop();
		void setHeadingTo(const TargetPose& target);
		float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold);
		float changeTarget(const TargetPose &target);
		/**
	   * @brief Sends the robot to a new target position. 
		 * There can be only one active target
		 * A new target overrides the existing one without stopping the robot
		 * The state of the process is recorded in the TrajectoryState structure that cnan be accessed through the getState() method
		 * @param target RoboCompTrajectoryRobot2D struct with the desired (x,z) coordinates of the robot. y is ignored
		 * @return float Distance in mm to target along a straight line
		*/
		float go(const TargetPose &target);
			
	public slots:
		void	compute(); 	
		
	private:
		RoboCompOmniRobot::TBaseState bState;
		TrajectoryState tState;	// object coding changing state for external interface
		RoboCompCommonBehavior::ParameterList params;
		RoboCompLaser::TLaserData laserData;
		CurrentTarget currentTarget;
		CurrentTarget currentTargetAnt, currentTargetBack;
		InnerModel *innerModel;
		
		/**
		 * @brief Instance of the Sampler of free space. Needs to be initialized. Robot's workspace is defined as a rectangle in outerRegion.
		 * Internal regions not to be visited by the robot are defined as a list of rectangles in innerRegions
		 * 
		 */
		Sampler sampler;
		QList<QRectF> innerRegions;
		QRectF outerRegion;
	
		/**
		 * @brief Object holding the trajectory as a list (WayPoints) of points (WayPoint). It has an update method to compute a set of variables that 
		 * hold the relation between the robot and the road. Those variables can be accessed bi corresponding getter methods.
		 * The road checks if the robot has achived the current target.
		 * 
		 */
		WayPoints road;
		
		/**
		 * @brief Local robot controller that implements a set of equations relating robot's pose relative to the road, state of the robot and target.
		 * It outputs three speeds: advance, lateral and rotational and uses the robot's proxy to send them to the physical/simulated robot
		 * 
		 */
		Controller *controller;
		
		/**
		 * @brief Projects the road onto the physical world through a provided laser field. It transforms distance relations between the road and the surrounding 
		 * obstacles into a repulsion force acting on the road. It also computes an internal string-type force that straightens the road, effectively smoothing the path.
		 * It has an update method that has to be called.
		 */
		ElasticBand elasticband;
		
		/**
		 * @brief Adpapter for the OMPL library. Not too generic so far but works. It is instantiated here to connecto to the RRT algorithm. It is used when 
		 * the PRM planner does not find a viable way between the robot and the target.
		 */
		PlannerOMPL *plannerOMPL;
		
		/**
		 * @brief Hand made, boost graph libray based PRM planner based on the original Kavrani et al paper. 
		 * If there is not a "grafo.dot" file in the local directory the class will create a new sampled graph of the free space using InnerModel's FCL 
		 * collision detector and the provided free space sampler.
		 * 
		 */
		PlannerPRM plannerPRM;
		
			/**
		 * @brief Drawing classes to draw the planner's graph and the road. These classes abstract away the drwaing code from PlannerPRM and WayPoints
		 * 
		 */
		#ifdef USE_QTGUI
			GraphDraw graphdraw;
			WaypointsDraw waypointsdraw;
		#endif
	
		/**
		 * @brief InnerModelViewer wrapper that runs on its own thread
		 */
		//Threaded InnerModelViewer
		#ifdef USE_QTGUI
			InnerViewer *viewer;
		#endif
			
		
		/**
		 * @brief QTimer used to implement a filter limiting the maximun input frequency of target requests through the go/goReferenced methods.
		 * 
		 */
		QTime relojForInputRateControl;  //used to limit input frequency
		QTime taskReloj;  //Measures duration of commands


		////////////////////////////////////////////////////////////////////////
		//Commands corresponding to servant methods, but running on local thread
		/////////////////////////////////////////////////////////////////////////
		bool gotoCommand(InnerModel* innerModel, CurrentTarget& target, TrajectoryState &state, WayPoints& myRoad, RoboCompLaser::TLaserData &lData);
		bool setHeadingCommand(InnerModel* innerModel, float alfa, CurrentTarget& target, TrajectoryState& state, WayPoints& myRoad);
		bool stopCommand( CurrentTarget& target, WayPoints& myRoad, TrajectoryState &state);
		bool changeTargetCommand(InnerModel* innerModel, CurrentTarget& target,  TrajectoryState &stat, WayPoints& myRoad);
		bool goBackwardsCommand(InnerModel* innerModel, CurrentTarget& current,CurrentTarget &currentT, TrajectoryState& state, WayPoints& myRoad);
		bool learnCommand(CurrentTarget &target,const WayPoints& myRoad);
		
		////////////////////////////////////////////////////////////////////////
		//Auxiliary methods
		/////////////////////////////////////////////////////////////////////////
		void tryingToConnect();
		bool updateInnerModel(InnerModel* inner, TrajectoryState &state);
		bool insertObstacle();
		bool removeNode(const QString &item);
		void addPlane(QString item, QString parent, QString path, QVec scale, QVec t, QVec r);
		void readRoadFromFile(string name, WayPoints *road);
		void setRobotInitialPose(float x, float z, float alpha);
		void drawTarget(const QVec &target);
		void drawGreenBoxOnTarget(const QVec &target);
		void printNumberOfElementsInIMV();
		float angmMPI(float angle);

		void mapBasedTarget(const NavigationParameterMap  &parameters);

};


#endif
