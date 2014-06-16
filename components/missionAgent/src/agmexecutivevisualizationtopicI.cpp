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
#include "agmexecutivevisualizationtopicI.h"

AGMExecutiveVisualizationTopicI::AGMExecutiveVisualizationTopicI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


AGMExecutiveVisualizationTopicI::~AGMExecutiveVisualizationTopicI()
{
	// Free component resources here
}

// Component functions, implementation
void AGMExecutiveVisualizationTopicI::update(const RoboCompAGMWorldModel::World& world, const RoboCompAGMWorldModel::World& target, const RoboCompPlanning::Plan& plan, const Ice::Current&)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	worker->update(world,target,plan);
	printf("%s: %d\n", __FILE__, __LINE__);
}

void AGMExecutiveVisualizationTopicI::successFulChange(const RoboCompPlanning::ActionSequence &s, const Ice::Current&)
{
	printf("%s: %d\n", __FILE__, __LINE__);
}

void AGMExecutiveVisualizationTopicI::aimedChange(const RoboCompPlanning::Action &a, const Ice::Current&)
{
	printf("%s: %d\n", __FILE__, __LINE__);
}

void AGMExecutiveVisualizationTopicI::invalidChange(const std::string &c, const Ice::Current&)
{
	printf("%s: %d\n", __FILE__, __LINE__);
}



