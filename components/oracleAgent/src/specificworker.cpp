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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

#ifdef USE_QTGUI
	innerModelViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);
#endif
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
	rgbdImageColor.create(RGBD_IMAGE_HEIGHT,RGBD_IMAGE_WIDTH,CV_8UC3);
	cameraImageColor.create(CAMERA_IMAGE_HEIGTH,CAMERA_IMAGE_WIDTH,CV_8UC3);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }


#ifdef USE_QTGUI
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
#endif


	timer.start(Period);

	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	if (worldModel->numberOfSymbols()<1)
	{
		try
		{
			RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
			structuralChange(w);			
			return;
		}
		catch(...)
		{
			printf("The executive is probably not running, waiting for first AGM model publication...");
		}
	}
	try
	{
		rgbd_proxy->getRGB(rgbdImage, hState, bState);
		memcpy(rgbdImageColor.data , &rgbdImage[0], RGBD_IMAGE_WIDTH*RGBD_IMAGE_HEIGHT*3);
		//READ FROM MODEL int witdh=850, depth=850, height=80, offset=0;
		extractRectangleROI(rgbdImageColor,"rgbd","tableD",850,850);		
		imshow("RGB from ASUS", rgbdImageColor);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading form RGBD " << e << std::endl;
	}
	
	//For cameras
	try
	{
		RoboCompRGBDBus::ImageMap images;
		CameraList cameraList;
		cameraList.push_back(std::string("default"));
		rgbdbus_proxy->getImages(cameraList, images);
		for (auto i : images)
		{		
			memcpy(cameraImageColor.data, &i.second.colorImage[0], CAMERA_IMAGE_WIDTH*CAMERA_IMAGE_HEIGTH*3);
			extractRectangleROI(cameraImageColor,"camera","tableD", 850,850);		
			imshow("RGB from CAMERA", cameraImageColor);
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from CAMERA" << e << std::endl;
	}
	
	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}

#ifdef USE_QTGUI
	if (innerModelViewer) innerModelViewer->update();
	osgView->frame();
#endif
}

bool SpecificWorker::reloadConfigAgent()
{
//implementCODE
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
//implementCODE
	return params;
}

void SpecificWorker::killAgent()
{
//implementCODE

}

int SpecificWorker::uptimeAgent()
{
//implementCODE
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
//implementCODE
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
//implementCODE
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::semanticDistance(const string &word1, const string &word2, float &result)
{
//implementCODE

}

void SpecificWorker::getLabelsFromImage(const RoboCompRGBD::ColorSeq &image, ResultList &result)
{
//implementCODE

}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	qDebug()<<"structuralChange";	
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	regenerateInnerModelViewer();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		printf("%d--[%s]-->%d\n", modification.a, modification.edgeType.c_str(), modification.b);
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}



void SpecificWorker::regenerateInnerModelViewer()
{
	if (innerModelViewer)
	{
		osgView->getRootGroup()->removeChild(innerModelViewer);
	}

	innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "oracleAgentAgent");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}

void SpecificWorker::extractRectangleROI(Mat img, QString sensorName, QString imName, int width, int depth, int height, int offset, bool draw)
{
		auto tableCenterInWorld= innerModel->transform("world",imName);
		//project in screen coordinates		
		auto tableCenterInScreenCoords = innerModel->getCamera(sensorName)->project("world",tableCenterInWorld);	
		auto worldCoordinates = tableCenterInWorld;
		vector<Point> coordinatesVector;		
		coordinatesVector.resize(8);
	
		tableCenterInWorld.print("tableCenterInWorld");
		for (uint i=0; i<coordinatesVector.size(); i++) 
		{			
			//p1 
			if (i==0)
			{
				worldCoordinates[0] = tableCenterInWorld[0] - width/2 - offset;
				worldCoordinates[2] = tableCenterInWorld[2] + depth/2 + offset;
			}
			
			//p2 
			if (i==1)
			{
				worldCoordinates[0] = tableCenterInWorld[0] + width/2 + offset;
				worldCoordinates[2] = tableCenterInWorld[2] + depth/2 + offset;
			}
			//p3 
			if (i==2)
			{
				worldCoordinates[0] = tableCenterInWorld[0] + width/2 + offset;
				worldCoordinates[2] = tableCenterInWorld[2] - depth/2 - offset;
			}
			//p4 
			if (i==3)
			{
				worldCoordinates[0] = tableCenterInWorld[0] - width/2 - offset;
				worldCoordinates[2] = tableCenterInWorld[2] - depth/2 - offset;
			}
			//p5
			if (i==4)
			{
				worldCoordinates[0] = tableCenterInWorld[0] - width/2 - offset;
				worldCoordinates[1] = tableCenterInWorld[1] + height;
				worldCoordinates[2] = tableCenterInWorld[2] + depth/2 + offset;
			}
			
			//p6 
			if (i==5)
			{
				worldCoordinates[0] = tableCenterInWorld[0] + width/2 + offset;
				worldCoordinates[1] = tableCenterInWorld[1] + height;
				worldCoordinates[2] = tableCenterInWorld[2] + depth/2 + offset;
			}
			//p7 
			if (i==6)
			{
				worldCoordinates[0] = tableCenterInWorld[0] + width/2 + offset;
				worldCoordinates[1] = tableCenterInWorld[1] + height;
				worldCoordinates[2] = tableCenterInWorld[2] - depth/2 - offset;
			}
			//p8 
			if (i==7)
			{
				worldCoordinates[0] = tableCenterInWorld[0] - width/2 - offset;
				worldCoordinates[1] = tableCenterInWorld[1] + height;
				worldCoordinates[2] = tableCenterInWorld[2] - depth/2 - offset;
			}
// 			worldCoordinates.print("worldCoordinates");
			
			coordinatesVector[i].x=innerModel->getCamera(sensorName)->project("world",worldCoordinates)[0];		
			coordinatesVector[i].y=innerModel->getCamera(sensorName)->project("world",worldCoordinates)[1];				
			
			
		}
		
		
		//Convex HULL like example:https://github.com/opencv/opencv/blob/master/samples/cpp/convexhull.cpp				
		vector<int> hull;
		cv::convexHull(Mat(coordinatesVector), hull, true);
		int hullcount = (int)hull.size();
		Point pt0 = coordinatesVector[hull[hullcount-1]];
// 		std::cout<<"hullcount"<<hullcount<<std::endl;
		
		//black will be the final image with only the content of the ConvexHull Region
		Mat black(img.rows, img.cols, img.type(), cv::Scalar::all(0));
		Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
		
		//hullpoints have the points themselves
		vector< vector<Point> >  hullPoints;
		hullPoints.push_back(vector<Point>());
		cv::convexHull(Mat(coordinatesVector), hullPoints[0], true);
		//std::cout<<hullPoints;
		drawContours( mask,hullPoints,0, Scalar(255),CV_FILLED, 8 );
		img.copyTo(black,mask);  
	
		auto boundRect = boundingRect( Mat(hullPoints[0]) );
		//minEnclosingCircle( (Mat)co_ordinates[0], center[i], radius[i] );
		cv::Mat croppedImage = black(boundRect);
		
		//draw points, crow image and convexHull 
		if(draw) 
		{	
			imshow("crop image", croppedImage);
			cv::Point center;
			center.x=tableCenterInScreenCoords[0];center.y=tableCenterInScreenCoords[1];
			cv::circle(img,center,6,Scalar(0,0,255),4);
		
				for (uint i=0; i<coordinatesVector.size(); i++) 		
				{
					if (i<4)
						cv::circle(img,coordinatesVector[i],1,Scalar(255,0,0),8);
					else 
						cv::circle(img,coordinatesVector[i],1,Scalar(0,255,0),8);
				}
				
				//ConvexHull draw
				for( int i = 0; i < hullcount; i++ )
				{
					Point pt = coordinatesVector[hull[i]];
					line(img, pt0, pt, Scalar(255, 0, 0), 2);
					pt0 = pt;
					
				}
		}
}

