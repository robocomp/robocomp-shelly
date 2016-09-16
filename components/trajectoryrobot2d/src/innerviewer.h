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

#ifndef INNERVIEWER_H
#define INNERVIEWER_H

#include <genericworker.h>
#include <iostream>
#include <fstream>
#include "waypoints.h"
#include <QtCore>

//#include <osgviewer/osgview.h>
#include <osgViewer/Viewer>
#include <innermodel/innermodelviewer.h>
#include <innermodeldraw.h>

class InnerViewer: public QThread
{
	public:
		InnerViewer(InnerModel *innerModel_, QObject *parent = 0);
		virtual ~InnerViewer();
		void run();
		InnerModelViewer *innerViewer;
		InnerModel *innerModel;

	private:
		osgViewer::Viewer viewer;
		void createWindow(osgViewer::Viewer& viewer);
};

#endif