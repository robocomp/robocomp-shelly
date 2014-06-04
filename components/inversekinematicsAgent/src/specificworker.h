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
#include <agm.h>

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
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void killAgent();
	int uptimeAgent();
	bool reloadConfigAgent();
	void modelModified(const RoboCompAGMWorldModel::Event& modification);
	void modelUpdated(const RoboCompAGMWorldModel::Node& modification);

public slots:
 	void compute();
	void slotStop();

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	
	void stateMachine();
	///states functions.
	void stop();
	void closeHand();
	void approachFinger();
	void touchFinger();
	void openHand();
	void approachHand();	
	void grasp();
	void take();
	
	void ajusteFino();
	
	
private:
	inline void sendRightHandPose(QVec t, QVec r, QVec wt, QVec wr);
	inline void sendRightHandPose(QVec t, QVec r, float wtx, float wty, float wtz, float wrx, float wry, float wrz);
	inline void sendRightHandPose(float tx, float ty, float tz, float rx, float ry, float rz, float wtx, float wty, float wtz, float wrx, float wry, float wrz);
	inline void saccadic3D(QVec point, QVec axis);
	inline void saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz);

private:
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	typedef enum nameStates {STOP,CLOSEHAND, APPROACHFINGER, TOUCHFINGER,OPENHAND,APPROACHHAND,GRASP,TAKE } nameStates;
	nameStates currenState;	
	QTime elapsedTime;	
	bool exec;
	

};

#endif
