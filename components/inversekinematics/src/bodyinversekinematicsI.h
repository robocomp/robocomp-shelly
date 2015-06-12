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
#ifndef BODYINVERSEKINEMATICS_H
#define BODYINVERSEKINEMATICS_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <BodyInverseKinematics.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompBodyInverseKinematics;

class BodyInverseKinematicsI : public QObject , public virtual RoboCompBodyInverseKinematics::BodyInverseKinematics
{
Q_OBJECT
public:
	BodyInverseKinematicsI( GenericWorker *_worker, QObject *parent = 0 );
	~BodyInverseKinematicsI();
	
	void setFingers(const float  d, const Ice::Current&);
	void setRobot(const int  type, const Ice::Current&);
	TargetState getState(const string  &part, const Ice::Current&);
	void setNewTip(const string  &part, const string  &transform, const Pose6D  &pose, const Ice::Current&);
	void stop(const string  &part, const Ice::Current&);
	void goHome(const string  &part, const Ice::Current&);
	void setTargetPose6D(const string  &bodyPart, const Pose6D  &target, const WeightVector  &weights, const float  radius, const Ice::Current&);
	void advanceAlongAxis(const string  &bodyPart, const Axis  &ax, const float  dist, const Ice::Current&);
	void pointAxisTowardsTarget(const string  &bodyPart, const Pose6D  &target, const Axis  &ax,  bool  axisConstraint, const float  axisAngleConstraint, const Ice::Current&);
	void setJoint(const string  &joint, const float  position, const float  maxSpeed, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
