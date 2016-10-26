/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	RoboCompCommonBehavior::ParameterList getWorkerParams();
	
	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void structuralChange(const RoboCompAGMWorldModel::World &w);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification);
	void newAprilBasedPose(const float x, const float z, const float alpha);
	void newCGRPose(const float poseUncertainty, const float x, const float z, const float alpha);
	void newCGRCorrection(float poseUncertainty, float x1, float z1, float alpha1, float x2, float z2, float alpha2);

public slots:
	void compute(); 

private:
	RoboCompGenericBase::TBaseState lastState, cgrState, aprilState, omniState;
	bool newCGR, newApril;
	bool useCGR, useApril;
	float CGRWeight, aprilWeight;
	
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	
	
	bool odometryAndLocationIssues(bool force=false);
	void includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot);
	void setCorrectedPosition(const RoboCompGenericBase::TBaseState &bState);
	bool enoughDifference(const RoboCompGenericBase::TBaseState &lastState, const RoboCompGenericBase::TBaseState &newState);
	RoboCompCommonBehavior::ParameterList worker_params;
	QMutex *worker_params_mutex;	
};

#endif

