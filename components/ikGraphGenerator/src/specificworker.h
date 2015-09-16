/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

#include <innermodeldraw.h>
//#include <nabo/nabo.h>
#include <djk.h>
#include <graph.h>

using namespace boost;

#define MAX_ERROR_IK 5.
#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

enum GIKTargetState { GIK_NoTarget, GIK_GoToInit, GIK_GoToEnd, GIK_GoToActualTargetSend, GIK_GoToActualTargetSent };

/**
 * \class WorkerThread inherites of QThread
 */ 
class WorkerThread : public QThread
{
Q_OBJECT
public:
	WorkerThread(void *data_)
	{
		data = data_;
	}
	void *data;
	void run();
};
/**
 * \brief INVERSE KINEMATICS GRAPH GENERATOR COMPONENT
 * \class SpecificWorker main class
 */ 
class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:
	SpecificWorker (MapPrx& mprx);
	~SpecificWorker();
	bool setParams (RoboCompCommonBehavior::ParameterList params);
	
	
	int         setTargetAdvanceAxis (const string &bodyPart, const Axis &ax, const float dist);
	int         setTargetAlignaxis   (const string &bodyPart, const Pose6D &target, const Axis &ax);
	int         setTargetPose6D      (const string &bodyPart, const Pose6D &target, const WeightVector &weights);
	
	void        setFingers           (const float d);
	void        setJoint             (const string &joint, const float angle, const float maxSpeed);

	void        stop                 (const string &bodyPart);
	void        goHome               (const string &bodyPart);

	bool        getPartState         (const string &bodyPart);
	TargetState getTargetState       (const string &bodyPart, const int targetID);


public slots:
	void initFile    ();
	void initGenerate();
	void computeHard ();
	void compute     ();

	void goIK        ();
	void goHome      ();

private:
	struct Target
	{
		QString       part;
		int           id_IKG;
		int           id_IK;
		Pose6D        pose;
		WeightVector  weights;
		TargetState   state;
	};
	
	
	void updateFrame      (uint wait_usecs=0);
	bool goAndWait        (int nodeId, MotorGoalPositionList &mpl, int &recursive);
	bool goAndWait        (float x, float y, float z, int node, MotorGoalPositionList &mpl, int &recursive);
	void goAndWaitDirect  (const MotorGoalPositionList &mpl);
	void updateInnerModel ();
	void waitForMotorsToStop       ();

	////////////////////////////////////////
	bool                    READY;
	InnerModel              *innerModel;
	GIKTargetState          state;
	//QQueue<Target>          nextTargets;     // lista de targets ejecutandose o en espera de ser ejecutados
	Target                  currentTarget;
	QQueue<Target>          solvedList;      // lista de targets resueltos.
	QMutex                  *mutexSolved;
	int                     targetCounter;   // contador de targets
	int                     closestToInit;
	int                     closestToEnd;
	int                     targetId;
	std::vector<int>        path;
	std::pair<float, float> xrange;
	std::pair<float, float> yrange;
	std::pair<float, float> zrange;
	std::string             lastFinish;
	
	MotorGoalPositionList   centerConfiguration;
	MotorGoalPositionList   lastMotorGoalPositionList;
	
	ConnectivityGraph       *graph;
	WorkerThread            *workerThread;

#ifdef USE_QTGUI
	OsgView                 *osgView;
	InnerModelViewer        *innerViewer;
	InnerModel              *innerVisual;
#endif
	
};

#endif

