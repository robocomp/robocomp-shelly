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
#include <qjoystick/qjoystick.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT

 
struct TButton {
bool up =false;
bool down =false;
bool right =false;
bool left =false;
bool rotacion=false;
};

TButton tbutton;
int valorgiro;



public:
  
	void move();
	
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

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

public slots:
	void compute();
	//void receivedJoyStickEvent(int value, int type, int number);
	
	void upP ();
	void upR ();
	void downP ();
	void downR ();
	void rightP ();
	void rightR ();
	void leftP ();
	void leftR ();
	void rotar(int valor);
	void giroP();
	void giroR();

private:
  
	RoboCompInnerModelManager::Pose3D pose;
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	
	
	void includeInRCIS();
	void includeInAGM();
	
	
	int32_t personSymbolId;
	
	QTime lastJoystickEvent;
	QJoyStick *joystick;
	float humanAdvVel, humanRotVel;
	float humanRot;
};

#endif

