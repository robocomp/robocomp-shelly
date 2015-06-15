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
#include "bodyinversekinematicsI.h"

BodyInverseKinematicsI::BodyInverseKinematicsI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


BodyInverseKinematicsI::~BodyInverseKinematicsI()
{
}

void BodyInverseKinematicsI::setFingers(const float  d, const Ice::Current&)
{
	worker->setFingers(d);
}

void BodyInverseKinematicsI::setRobot(const int  type, const Ice::Current&)
{
	worker->setRobot(type);
}

TargetState BodyInverseKinematicsI::getState(const string  &part, const Ice::Current&)
{
	return worker->getState(part);
}

void BodyInverseKinematicsI::setNewTip(const string  &part, const string  &transform, const Pose6D  &pose, const Ice::Current&)
{
	worker->setNewTip(part, transform, pose);
}

void BodyInverseKinematicsI::stop(const string  &part, const Ice::Current&)
{
	worker->stop(part);
}

void BodyInverseKinematicsI::goHome(const string  &part, const Ice::Current&)
{
	worker->goHome(part);
}

void BodyInverseKinematicsI::setTargetPose6D(const string  &bodyPart, const Pose6D  &target, const WeightVector  &weights, const float  radius, const Ice::Current&)
{
	worker->setTargetPose6D(bodyPart, target, weights, radius);
}

void BodyInverseKinematicsI::advanceAlongAxis(const string  &bodyPart, const Axis  &ax, const float  dist, const Ice::Current&)
{
	worker->advanceAlongAxis(bodyPart, ax, dist);
}

void BodyInverseKinematicsI::pointAxisTowardsTarget(const string  &bodyPart, const Pose6D  &target, const Axis  &ax,  bool  axisConstraint, const float  axisAngleConstraint, const Ice::Current&)
{
	worker->pointAxisTowardsTarget(bodyPart, target, ax, axisConstraint, axisAngleConstraint);
}

void BodyInverseKinematicsI::setJoint(const string  &joint, const float  position, const float  maxSpeed, const Ice::Current&)
{
	worker->setJoint(joint, position, maxSpeed);
}






