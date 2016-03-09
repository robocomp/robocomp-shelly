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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <agm.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();
	void structuralChange(const RoboCompAGMWorldModel::World& modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);

public slots:
	void compute();
	
	void startManualMode();

private:
	bool manualMode;

	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel, std::string m="");


	QVec getObjectsLocationInRobot(std::map<std::string, AGMModelSymbol::SPtr> &symbols, AGMModelSymbol::SPtr &object);
	QVec fromRobotToRoom(std::map<std::string, AGMModelSymbol::SPtr> &symbols, const QVec vector);
	int sendRightArmToPose(QVec p);

	void manageReachedObjects();
	void leaveObjectSimulation();

	std::map<std::string, AGMModelSymbol::SPtr> symbols;

	void actionExecution();
	void action_FindObjectVisuallyInTable(bool first=false);
	void action_SetObjectReach(bool first=false);
	void action_GraspObject(bool first=false);

	void directGazeTowards(AGMModelSymbol::SPtr symbol);
	void saccadic3D(QVec point, QVec axis);
	void saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz);


	bool isObjectType(AGMModel::SPtr model, AGMModelSymbol::SPtr node, const std::string &t);
	float distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr symbol);
// 	float distanceToPolygon(QVec reference, QVec position, std::string polygon_str);

	void setRightArmUp_Reflex();

	void updateViewer();
	void changeInner ();



private:
	
	std::string action, backAction;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	osgGA::TrackballManipulator *manipulator;
	OsgView *osgView;	
	InnerModelViewer *innerViewer; 
	
	bool active;

	int32_t sendHandToSymbol(AGMModelSymbol::SPtr symbol, QVec offset, std::map<std::string, AGMModelSymbol::SPtr> symbols);
	
	
public slots:
	void on_state1_clicked();
};

#endif

