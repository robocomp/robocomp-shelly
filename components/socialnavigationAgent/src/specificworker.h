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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <innermodel/innermodel.h>

#include <boost/format.hpp>

#define THRESHOLD 40

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
  
    
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
//bool para indicar si se ha movido la persona, lo utilizare para imprimir la coordenada de la persona cada vez que se mueva
	bool cambiopos=false;
	float x;
	float z;
	float rot;
	
	
	//////////////
	/// SERVANTS
	//////////////
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void  killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();
	
	void  structuralChange(const RoboCompAGMWorldModel::World & modification);
	void  symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void  symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence & modification);
	void  edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);


public slots:
 	void compute();
	void readTrajState();

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel,std::string m);
	void includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot);

	

	
	void go(float x, float z, float alpha=0, bool rot=false, float xRef=0, float zRef=0, float threshold=200);
	void stop();

	void actionExecution();
	int32_t getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel);
	void setIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel, int32_t identifier);

private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool haveTarget;
	QTimer trajReader;
	
	int32_t personSymbolId;	

	RoboCompTrajectoryRobot2D::NavState planningState;


	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;
	
	void manageReachedPose();
	float distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr object);
private:
	void action_WaitingToAchieve();
	void action_Stop(bool newAction = true);
	void action_ReachPose(bool newAction = true);
	void action_ChangeRoom(bool newAction = true);
	void action_FindObjectVisuallyInTable(bool newAction = true);
	void action_SetObjectReach(bool newAction = true);
//	void action_GraspObject(bool newAction = true);
	void action_DetectPerson (bool newAction = true);
	void action_HandObject(bool newAction = true);
	void action_NoAction(bool newAction = true);
	void action_HandObject_Offer(bool newAction = true);
	void action_HandObject_leave(bool newAction = true);
//CHECK
	//void updateRobotsCognitiveLocation();
//	std::map<int32_t, QPolygonF> roomsPolygons;
//	std::map<int32_t, QPolygonF> extractPolygonsFromModel(AGMModel::SPtr &worldModel);	
//	RoboCompOmniRobot::TBaseState bState;
	
};

#endif


