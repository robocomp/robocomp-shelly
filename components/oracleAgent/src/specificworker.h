/*
 *    Copyright (C)2017 by YOUR NAME HERE
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
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

#define RGBD_IMAGE_WIDTH 640
#define RGBD_IMAGE_HEIGHT 480

#define CAMERA_IMAGE_WIDTH 1280
#define CAMERA_IMAGE_HEIGTH 720

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
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
	void semanticDistance(const string &word1, const string &word2, float &result);
	void getLabelsFromImage(const RoboCompRGBD::ColorSeq &image, ResultList &result);
	void structuralChange(const RoboCompAGMWorldModel::World &w);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
	
	//Image, sensorName, InnerModel name of the object, width and depth: object parameters, height of ROI, offset=extra length in width and deepth
	void extractRectangleROI(Mat img, QString sensorName, QString imName, int width, int depth, int height = 80, int offset = 0, bool draw=true);

public slots:
	void compute();

private:
	InnerModel *innerModel;
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	
	
	RoboCompRGBD::ColorSeq rgbdImage;		
	RoboCompRGBDBus::Image cameraImage;
	cv::Mat rgbdImageColor, cameraImageColor;
	
	RoboCompGenericBase::TBaseState bState;
	RoboCompJointMotor::MotorStateMap hState;
	//RoboCompCommonHead::THeadState cState;
	RoboCompRGBDBus::CameraParams camParams;
	
	void regenerateInnerModelViewer();
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);

};

#endif
