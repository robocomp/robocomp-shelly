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

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif
/**
       \brief
       @author authorname
*/
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx &mprx);

	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPosition(const MotorGoalPosition& goal);
	void setVelocity(const MotorGoalVelocity& goal);
	void setZeroPos(const string& name);
	void setSyncPosition(const MotorGoalPositionList& listGoals);
	void setSyncVelocity(const MotorGoalVelocityList& listGoals);
	void setSyncZeroPos();
	MotorParams getMotorParams(const string& motor);
	MotorState getMotorState(const string& motor);
	MotorStateMap getMotorStateMap(const MotorList& mList);
	void getAllMotorState(MotorStateMap& mstateMap);
	MotorParamsList getAllMotorParams();
	BusParams getBusParams();

public slots:
	void compute();
	
	
private:
	InnerModel *innerModel;
	void recursiveIncludeMeshes(InnerModelNode *node, std::vector<QString> &in);
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *imv;
#endif

	RoboCompJointMotor::MotorParamsList motorList1, motorList0;
	std::map<string,RoboCompJointMotor::JointMotorPrx> prxMap; 
	void init();
	
	
	void actualizarInnerModel();
	
	
	std::vector< std::pair<QString, QString> > pairs;
	bool checkFuturePosition(const MotorGoalPositionList &goals, std::pair<QString, QString> &ret);

};

#endif
