/*
 *    Copyright (C) 2015-2016 by RoboLab - University of Extremadura
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

	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool deactivateAgent();
	void killAgent();
	int uptimeAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap &prs);
	RoboCompCommonBehavior::ParameterList getWorkerParams();
	
	void structuralChange(const RoboCompAGMWorldModel::World &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
	bool detectAndLocateObject(std::string objectToDetect, bool first);

public slots:
	void compute();

private:
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	
	void action_FindObjectVisuallyInTable  (bool newAction);

	void getIDsFor(std::string obj, int32_t &objectSymbolID, int32_t &objectStSymbolID);

	void updateTag(const tagsList &list);
	void newAprilTag(const tagsList &list);
	void newAprilTagAndPose(const tagsList &list,const RoboCompGenericBase::TBaseState &bState,const RoboCompJointMotor::MotorStateMap &hState);
	
	bool updateTable (const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel);
	bool updateMug   (const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel);
	bool updateMilk  (const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel);
	bool updateCoffee(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel);
	bool isObjectType(AGMModel::SPtr model, AGMModelSymbol::SPtr node, const std::string &t);
	AGMModelSymbol::SPtr getRoomFromTable(AGMModel::SPtr model, AGMModelSymbol::SPtr table);
	void getObject();
private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;

	RoboCompCommonBehavior::ParameterList worker_params;
	QMutex *worker_params_mutex;
};

#endif

