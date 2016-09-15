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

#include "innerviewer.h"

InnerViewer::InnerViewer( InnerModel *innerModel_, QObject *parent )
{
	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000., 4000., 1000.));
	osg::Vec3d center(osg::Vec3(0., 0., -0.));
	osg::Vec3d up(osg::Vec3(0., -1., 0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye, center, up));
	viewer.setCameraManipulator(tb);
	osgViewer::Viewer::ThreadingModel threadingModel = osgViewer::Viewer::AutomaticSelection;
	viewer.setThreadingModel(threadingModel);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	createWindow(viewer);
	osg::Group *root = new osg::Group();
	
	osg::Vec4 p = viewer.getLight()->getPosition();
	p.set(p.x(),1,p.z(),p.w());
	viewer.getLight()->setPosition(p);
	
	innerModel = innerModel_->copy();
	innerViewer = new InnerModelViewer(innerModel, "root", root, true);
	viewer.setSceneData(root);
	
	//////////////////////////
	//RESTORE FORMER VIEW					QUEDA CAPTURAR EL EVENTO DE CIERRE DE LA VENTANA PARA GUARDAR LA MATRIZ ACTUAL
	/////////////////////////
	QSettings *settings = new QSettings("RoboComp", "InnerViewer");
	QString path(".");
	if (path == settings->value("path").toString() )
	{
		QStringList l = settings->value("matrix").toStringList();
		if (l.size() > 0)
		{
			osg::Matrixd m;
			for (int i=0; i<4; i++ )
				for (int j=0; j<4; j++ )
					m(i,j)=l.takeFirst().toDouble();
			tb->setByMatrix(m);
		}
		else
			innerViewer->setMainCamera(tb, InnerModelViewer::TOP_POV);
	}
	else
		settings->setValue("path",path);
	
	
	// 	osg::Matrixd m = tb->getMatrix();
	// 	QString s="";
	// 	QStringList l;
	// 	for (int i=0; i<4; i++ )
	// 		for (int j=0; j<4; j++ )
	// 			l.append(s.number(m(i,j)));
	// 	settings->setValue("matrix", l);
	// 	settings->sync();
 	
	viewer.realize();
}

InnerViewer::~InnerViewer()
{
}

void InnerViewer::createWindow(osgViewer::Viewer& viewer)
{
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
		return;
	}

	unsigned int width, height;
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = 400;
	traits->height = 400;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid())
	{
		osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;

		// need to ensure that the window is cleared make sure that the complete window is set the correct colour
		// rather than just the parts of the window that are under the camera's viewports
		gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else
	{
		osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
	}
	unsigned int numCameras = 1;
	double aspectRatioScale = 1.0;///(double)numCameras;
	for(unsigned int i=0; i<numCameras;++i)
	{
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport((i*width)/numCameras,(i*height)/numCameras, width/numCameras, height/numCameras));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);

		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::scale(aspectRatioScale,1.0,1.0));
	}
}


void InnerViewer::run()
{
	while( true)
	{
		innerViewer->mutex->lock();
			innerViewer->update();
			viewer.frame();
		innerViewer->mutex->unlock();
		usleep(10000);
	}
}
