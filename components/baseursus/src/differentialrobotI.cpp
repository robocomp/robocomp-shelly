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
#include "differentialrobotI.h"
#include <OmniRobot.h>

DifferentialRobotI::DifferentialRobotI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


DifferentialRobotI::~DifferentialRobotI()
{
	// Free component resources here
}

// Component functions, implementation
void DifferentialRobotI::getBaseState(RoboCompDifferentialRobot::TBaseState& state, const Ice::Current&)
{
	RoboCompOmniRobot::TBaseState ostate;
	worker->getBaseState(ostate);
	state.x     = ostate.x;
	state.z     = ostate.z;
	state.alpha = ostate.alpha;
	state.correctedX = ostate.correctedX;
	state.correctedZ = ostate.correctedZ;
	state.correctedAlpha = ostate.correctedAlpha;
}

void DifferentialRobotI::getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current&)
{
	worker->getBasePose(x,z,alpha);
}

void DifferentialRobotI::setSpeedBase(Ice::Float adv, Ice::Float rot, const Ice::Current&)
{
	worker->setSpeedBase(0,adv,rot);
}

void DifferentialRobotI::stopBase(const Ice::Current&)
{
	worker->stopBase();
}

void DifferentialRobotI::resetOdometer(const Ice::Current&)
{
	worker->resetOdometer();
}

void DifferentialRobotI::setOdometer(const RoboCompDifferentialRobot::TBaseState& state, const Ice::Current&)
{
	RoboCompOmniRobot::TBaseState ostate;
	ostate.x     = state.x;
	ostate.z     = state.z;
	ostate.alpha = state.alpha;
	worker->setOdometer(ostate);
}

void DifferentialRobotI::setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	worker->setOdometerPose(x,z,alpha);
}

void DifferentialRobotI::correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	worker->correctOdometer(x,z,alpha);
}


