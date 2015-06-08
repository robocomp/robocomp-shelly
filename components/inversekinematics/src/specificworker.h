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

#include <iostream>
#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <QtCore>
#include <iterator>

#include "target.h"
#include "bodypart.h"
#include "metric.h"
#include "inversedkinematic.h"
#include "plannerompl.h"
#include "reflexx.h"
#include "sampler.h"

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

using namespace std;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
				SpecificWorker			(MapPrx& mprx);	
				~SpecificWorker			();
	bool 		setParams				(RoboCompCommonBehavior::ParameterList params);

	void 		setFingers				(const float d);
	void 		setRobot				(const int type);
	TargetState getState				(const string &part);
	void 		setNewTip				(const string &part, const string &transform, const Pose6D &pose);
	void 		stop					(const string &part);
	void 		goHome					(const string &part);
	void 		setTargetPose6D			(const string &bodyPart, const Pose6D &target, const WeightVector &weights, const float radius);
	void 		advanceAlongAxis		(const string &bodyPart, const Axis &ax, const float dist);
	void 		pointAxisTowardsTarget	(const string &bodyPart, const Pose6D &target, const Axis &ax, const bool &axisConstraint, const float axisAngleConstraint);
	void 		setJoint				(const string &joint, const float position, const float maxSpeed);
	void 		sendData				(const TData &data);

public slots:
	void 		compute					(); 

private:
	void 	updateInnerModel			();
	void	updateRCIS					(QVec newAngles, BodyPart part);
	void	init 						();
	void	createInnerModelTarget		(Target &target);
	void	removeInnerModelTarget		(Target &target);
	void	showInformation				(BodyPart part, Target target);

private:
	int 						correlativeID;
	
	QMap<QString, BodyPart> 	bodyParts;
	QStringList					availableParts;
	InnerModel					*innermodel;
	PlannerOMPL 				*planner;
	Sampler 					sampler;
	InversedKinematic			*inversedkinematic;
	
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
#endif

};

#endif

