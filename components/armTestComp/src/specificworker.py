#
# Copyright (C) 2016 by YOUR NAME HERE
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

import sys, os, Ice, traceback, time
from random import randint

from PySide import *
from genericworker import *



#Position dictionary: could contain:  
# 6D postion ==> 'name':(,x,y,z,rx,ry,rz)
# or joint positions ==> 'name':[('motor1',angle1),('motor2',angle2)...]
posDict = {'zero':(0,0,800,0,0,0),'elbowdown':[('armY',0.0),('armX1',1.0),('armX2',-2.5),('wristX',0.0)],'elbowup':[('armY',0.0),('armX1',-1.0),('armX2',2.5),('wristX',0.0)],'anygrabposition':(0,0,800,0,0,0)}


#Body part to send commands:
bodyPart = "ARM"


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		print proxy_map
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		self.test_timer = QtCore.QTimer()
		self.test_timer.timeout.connect(self.test)
		self.test_active = False

		print 'Load positions: ',
		for name in posDict.keys():
			print name
			self.ui.position_cb.addItem(name)

		self.ui.go_pb.clicked.connect( self.set_position_list );
		self.ui.setPosition_pb.clicked.connect( self.set_position );
		self.ui.stop_pb.clicked.connect( self.stop );
		self.ui.test_pb.clicked.connect( self.enableTest );


	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
#		print 'SpecificWorker.compute...'
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True

	@QtCore.Slot()
	def set_position(self):
		target = RoboCompInverseKinematics.Pose6D()
		target.x = self.ui.x_sb.value()
		target.y = self.ui.y_sb.value()
		target.z = self.ui.z_sb.value()
		target.rx = self.ui.rx_sb.value()
		target.ry = self.ui.ry_sb.value()
		target.rz = self.ui.rz_sb.value()
		self.sendMove(target)

	@QtCore.Slot()
	def set_position_list(self):
		target = RoboCompInverseKinematics.Pose6D()
		values = posDict[str(self.ui.position_cb.currentText())]
		print "\nMove: " + str(self.ui.position_cb.currentText())
		if isinstance(values,tuple):
			print '6D move'
			print values
			target.x = values[0]
			target.y = values[1]
			target.z = values[2]
			target.rx = values[3]
			target.ry = values[4]
			target.rz = values[5]
			self.sendMove(target)
		elif isinstance(values,list):
			print 'Joint move'
			posList = []
			for item in values:
				print item
				pos = RoboCompJointMotor.MotorGoalPosition()
				pos.name = item[0]
				pos.position = item[1]
				pos.maxSpeed = 0.4
				posList.append(pos)
			self.setSyncPosition(posList)
		else: 
			print "Unknow movement type, check movements list"

	@QtCore.Slot()
	def stop(self):
		try:
			self.inversekinematics_proxy.stop(bodyPart)
		except:
			print sys.exc_info()[0]

	def setSyncPosition(self, positionList):
		try:
			self.jointmotor_proxy.setSyncPosition(positionList)
		except:
			print sys.exc_info()[0]

	def sendMove(self,target):
		weights = RoboCompInverseKinematics.WeightVector()
		weights.x = 1
		weights.y = 1
		weights.z = 1
		weights.rx = 1
		weights.ry = 1
		weights.rz = 1
		try:
			self.inversekinematics_proxy.setTargetPose6D(bodyPart, target, weights)
		except:
			print sys.exc_info()[0]

	def test(self):
		aux ='\n\nMove\n'
		self.ui.position_cb.setCurrentIndex(randint(0,self.ui.position_cb.count()-1))
		self.set_position_list()
		time.sleep(3)

	@QtCore.Slot()
	def enableTest(self):
		if self.test_active:
			self.ui.test_pb.setText('Start test')
			self.test_timer.stop()
		else:
			self.ui.test_pb.setText('Stop test')
			self.test_timer.start(1)
		self.test_active = not self.test_active
