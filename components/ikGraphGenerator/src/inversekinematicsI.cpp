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
#include "inversekinematicsI.h"

InverseKinematicsI::InverseKinematicsI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


InverseKinematicsI::~InverseKinematicsI()
{
}

void InverseKinematicsI::setFingers(const float  d, const Ice::Current&)
{
	worker->setFingers(d);
}

TargetState InverseKinematicsI::getTargetState(const string  &bodyPart, const int  targetID, const Ice::Current&)
{
	return worker->getTargetState(bodyPart, targetID);
}

int InverseKinematicsI::setTargetAdvanceAxis(const string  &bodyPart, const Axis  &ax, const float  dist, const Ice::Current&)
{
	return worker->setTargetAdvanceAxis(bodyPart, ax, dist);
}

void InverseKinematicsI::goHome(const string  &bodyPart, const Ice::Current&)
{
	worker->goHome(bodyPart);
}

void InverseKinematicsI::stop(const string  &bodyPart, const Ice::Current&)
{
	worker->stop(bodyPart);
}

int InverseKinematicsI::setTargetPose6D(const string  &bodyPart, const Pose6D  &target, const WeightVector  &weights, const Ice::Current&)
{
	return worker->setTargetPose6D(bodyPart, target, weights);
}

bool InverseKinematicsI::getPartState(const string  &bodyPart, const Ice::Current&)
{
	return worker->getPartState(bodyPart);
}

void InverseKinematicsI::setJoint(const string  &joint, const float  angle, const float  maxSpeed, const Ice::Current&)
{
	worker->setJoint(joint, angle, maxSpeed);
}

int InverseKinematicsI::setTargetAlignaxis(const string  &bodyPart, const Pose6D  &target, const Axis  &ax, const Ice::Current&)
{
	return worker->setTargetAlignaxis(bodyPart, target, ax);
}






