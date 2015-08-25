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

#include <agm.h>
#include <agm_model.h>
#include <agm_modelPrinter.h>
#include <agm_modelDrawer.h>
#include <graphModelViewer.h>

#include <innermodel/innermodel.h>
#include <innermodel/innermodeldraw.h>
#include <innermodel/innermodelreader.h>
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
#include <agmInner/agmInner.h>

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
	void structuralChange(const RoboCompAGMWorldModel::Event& modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);
	void update(const RoboCompAGMWorldModel::World &a, const RoboCompAGMWorldModel::World &b, const RoboCompPlanning::Plan &p);

	bool setParams(RoboCompCommonBehavior::ParameterList params) { return true; }
	bool activateAgent(const ParameterMap& params) {return true; }
	bool deactivateAgent() {return true; }
	StateStruct getAgentState() { StateStruct a; return a; }
	ParameterMap getAgentParameters() { ParameterMap a; return a; }
	bool setAgentParameters(const ParameterMap& params);
	void killAgent() {}
	int uptimeAgent() { return 1; }
	bool reloadConfigAgent() { return true; }
	void set3DViewer();

public slots:
	
	
	
	void compute();
	void setGeometry();
	void quitButtonClicked();

	void activateClicked();
	void deactivateClicked();
	void resetClicked();

// 	void broadcastButtonClicked();
	void broadcastPlanButtonClicked();
	void broadcastModelButtonClicked();

	void setMission();

	void addMission(std::string name, std::string path)
	{
		missions->addItem(QString::fromStdString(name));
		missionPaths[missions->count()-1] = path;
	}
	void imShow();
	void showRobot();
	void showMesh();
	void showPlane();
	void itemSelected(QString nameItem);

private:
	bool refresh;
	std::map<int, std::string> missionPaths;
	QMutex modelMutex, planMutex;
	AGMModel::SPtr worldModel, targetModel;
	RoboCompAGMWorldModel::World worldModelICE, targetModelICE;
	RoboCompPlanning::Plan plan;
	AGMModelDrawer *modelDrawer, *targetDrawer;
	RCDraw *rcdraw1, *rcdraw2;
	GraphModelViewer *graphViewer;

	
	osgGA::TrackballManipulator *manipulator;
	OsgView *osgView;
	InnerModel *innerModelVacio;
	InnerModelViewer *innerViewer; 
	AgmInner agmInner;


	
	 void updateInner3D();
	 void insertNodeInnerModel(InnerModelNode* node);
	 void changeInner (InnerModel *inner);
	void fillItemList();


};

#endif
