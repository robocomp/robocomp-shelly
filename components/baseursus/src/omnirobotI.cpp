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
#include "omnirobotI.h"

OmniRobotI::OmniRobotI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


OmniRobotI::~OmniRobotI()
{
	// Free component resources here
}

// Component functions, implementation
void OmniRobotI::getBaseState(RoboCompOmniRobot::TBaseState& state, const Ice::Current&)
{
	worker->getBaseState(state);
}

void OmniRobotI::getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current&)
{
	worker->getBasePose(x,z,alpha);
}

void OmniRobotI::setSpeedBase(Ice::Float advx, Ice::Float advz, Ice::Float rot, const Ice::Current&)
{
	worker->setSpeedBase(advx,advz,rot);
}

void OmniRobotI::stopBase(const Ice::Current&)
{
	worker->stopBase();
}

void OmniRobotI::resetOdometer(const Ice::Current&)
{
	worker->resetOdometer();
}

void OmniRobotI::setOdometer(const RoboCompOmniRobot::TBaseState& state, const Ice::Current&)
{
	worker->setOdometer(state);
}

void OmniRobotI::setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	worker->setOdometerPose(x,z,alpha);
}

void OmniRobotI::correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	worker->correctOdometer(x,z,alpha);
}


