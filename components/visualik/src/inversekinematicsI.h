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
#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <InverseKinematics.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompInverseKinematics;

class InverseKinematicsI : public QObject , public virtual RoboCompInverseKinematics::InverseKinematics
{
Q_OBJECT
public:
	InverseKinematicsI( GenericWorker *_worker, QObject *parent = 0 );
	~InverseKinematicsI();
	
	void setFingers(const float  d, const Ice::Current&);
	TargetState getTargetState(const string  &bodyPart, const int  targetID, const Ice::Current&);
	int setTargetAdvanceAxis(const string  &bodyPart, const Axis  &ax, const float  dist, const Ice::Current&);
	void goHome(const string  &bodyPart, const Ice::Current&);
	void stop(const string  &bodyPart, const Ice::Current&);
	int setTargetPose6D(const string  &bodyPart, const Pose6D  &target, const WeightVector  &weights, const Ice::Current&);
	bool getPartState(const string  &bodyPart, const Ice::Current&);
	void setJoint(const string  &joint, const float  angle, const float  maxSpeed, const Ice::Current&);
	int setTargetAlignaxis(const string  &bodyPart, const Pose6D  &target, const Axis  &ax, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
