/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#include <CommonBehavior.h>

#include <Planning.h>
#include <GenericBase.h>
#include <OmniRobot.h>
#include <Logger.h>
#include <AprilBasedLocalization.h>
#include <CGR.h>
#include <AGMCommonBehavior.h>
#include <AGMExecutive.h>
#include <AGMExecutiveTopic.h>
#include <AGMWorldModel.h>
#include <agm.h>

#include <agm.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompPlanning;
using namespace RoboCompGenericBase;
using namespace RoboCompOmniRobot;
using namespace RoboCompLogger;
using namespace RoboCompAprilBasedLocalization;
using namespace RoboCompCGR;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAGMExecutiveTopic;
using namespace RoboCompAGMWorldModel;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};

class GenericWorker :
public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	bool activate(const BehaviorParameters& parameters);
	bool deactivate();
	bool isActive() { return active; }


	AGMExecutivePrx agmexecutive_proxy;
	OmniRobotPrx omnirobot_proxy;
	LoggerPrx logger_pubproxy;

	virtual bool AGMCommonBehavior_activateAgent(const ParameterMap &prs) = 0;
	virtual bool AGMCommonBehavior_deactivateAgent() = 0;
	virtual ParameterMap AGMCommonBehavior_getAgentParameters() = 0;
	virtual StateStruct AGMCommonBehavior_getAgentState() = 0;
	virtual void AGMCommonBehavior_killAgent() = 0;
	virtual bool AGMCommonBehavior_reloadConfigAgent() = 0;
	virtual bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs) = 0;
	virtual int AGMCommonBehavior_uptimeAgent() = 0;
	virtual void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification) = 0;
	virtual void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications) = 0;
	virtual void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes) = 0;
	virtual void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType) = 0;
	virtual void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w) = 0;
	virtual void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification) = 0;
	virtual void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications) = 0;
	virtual void AprilBasedLocalization_newAprilBasedPose(const float x, const float z, const float alpha) = 0;
	virtual void CGR_resetPose(const float x, const float z, const float alpha) = 0;

protected:

	QTimer timer;
	int Period;
	bool active;
	AGMModel::SPtr worldModel;
	BehaviorParameters p;
	ParameterMap params;
	int iter;
	bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
	RoboCompPlanning::Action createAction(std::string s);

private:


public slots:
	virtual void compute() = 0;
    virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
