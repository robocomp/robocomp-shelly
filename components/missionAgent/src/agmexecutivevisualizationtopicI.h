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
#ifndef AGMEXECUTIVEVISUALIZATIONTOPICI_H
#define AGMEXECUTIVEVISUALIZATIONTOPICI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <AGMExecutive.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAGMExecutive;

class AGMExecutiveVisualizationTopicI : public QObject , public virtual RoboCompAGMExecutive::AGMExecutiveVisualizationTopic
{
Q_OBJECT
public:
	AGMExecutiveVisualizationTopicI( GenericWorker *_worker, QObject *parent = 0 );
	~AGMExecutiveVisualizationTopicI();
	void  update(const RoboCompAGMWorldModel::World& world, const RoboCompAGMWorldModel::World& target, const RoboCompPlanning::Plan& plan, const Ice::Current& = Ice::Current());

	void  successFulChange(const RoboCompPlanning::ActionSequence &s, const Ice::Current&);
	void  aimedChange(const RoboCompPlanning::Action &a, const Ice::Current&);
	void  invalidChange(const std::string &c, const Ice::Current&);


	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif