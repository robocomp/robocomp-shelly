# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import Ice, sys, math, traceback

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *


class C(QWidget):
	
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.Lista=list();
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompInnerModelManager'].InnerModelManagerPrx.checkedCast(self.prx)
		
		self.a = 0.
		self.check = QCheckBox("move", self)
		self.check.show()
		
		
		print "creation"
		first = 0
		pose = self.mods['RoboCompInnerModelManager'].Pose3D()
		pose.x = 0
		pose.y = 0
		pose.z = 1000
		self.proxy.addTransform("test_T", "static", "sensor_transform", pose);
		pose.x = 0
		pose.y = 0
		pose.z = 0
		pose.rx = 1.57
		mesh = self.mods['RoboCompInnerModelManager'].meshType()
		mesh.pose = pose
		
		mesh.scaleX = 100
		mesh.scaleY = 100
		mesh.scaleZ = 100;
		mesh.render = 0 
		mesh.meshPath = "/home/robocomp/robocomp/Files/osgModels/mobiliario/taza.osg";
		self.proxy.addMesh("test", "test_T", mesh);
	
		self.show()

	def job(self):
		if self.check.isChecked():

			pose = self.mods['RoboCompInnerModelManager'].Pose3D()
			#pose.x = 500.*math.cos(self.a)
			pose.x = 0
			pose.y = 0
			pose.z = 1000
			pose.rx = 1.57
			pose.ry = 0
			pose.rz = 0
			self.proxy.setPoseFromParent("test_T", pose)
			self.a += 0.015
		

