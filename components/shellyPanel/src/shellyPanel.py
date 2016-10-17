#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# \mainpage RoboComp::shellyPanel
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/shellyPanel --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, IceStorm, subprocess, threading, time, Queue, os, copy

# Ctrl+c handling
import signal

from PySide import *

from specificworker import *


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler, _communicator):
		self.handler = _handler
		self.communicator = _communicator
	def getFreq(self, current = None):
		self.handler.getFreq()
	def setFreq(self, freq, current = None):
		self.handler.setFreq()
	def timeAwake(self, current = None):
		try:
			return self.handler.timeAwake()
		except:
			print 'Problem getting timeAwake'
	def killYourSelf(self, current = None):
		self.handler.killYourSelf()
	def getAttrList(self, current = None):
		try:
			return self.handler.getAttrList(self.communicator)
		except:
			print 'Problem getting getAttrList'
			traceback.print_exc()
			status = 1
			return



if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	elif len(params) == 1:
		params.append('--Ice.Config=config')
	ic = Ice.initialize(params)
	status = 0
	mprx = {}
	parameters = {}
	for i in ic.getProperties():
		parameters[str(i)] = str(ic.getProperties().getProperty(i))

	# Topic Manager
	proxy = ic.getProperties().getProperty("TopicManager.Proxy")
	obj = ic.stringToProxy(proxy)
	try:
		topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
	except Ice.ConnectionRefusedException, e:
		print 'Cannot connect to IceStorm! ('+proxy+')'
		sys.exit(-1)

	# Remote object connection for JointMotor
	try:
		proxyString = ic.getProperties().getProperty('JointMotorProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			jointmotor_proxy = JointMotorPrx.checkedCast(basePrx)
			mprx["JointMotorProxy"] = jointmotor_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (JointMotor)', proxyString
			mprx["JointMotorProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get JointMotorProxy property.'
		status = 1


	# Remote object connection for Laser
	try:
		proxyString = ic.getProperties().getProperty('LaserProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			laser_proxy = LaserPrx.checkedCast(basePrx)
			mprx["LaserProxy"] = laser_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (Laser)', proxyString
			mprx["LaserProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get LaserProxy property.'
		status = 1


	# Remote object connection for RGBD
	try:
		proxyString = ic.getProperties().getProperty('RGBDProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			rgbd_proxy = RGBDPrx.checkedCast(basePrx)
			mprx["RGBDProxy"] = rgbd_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (RGBD)', proxyString
			mprx["RGBDProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get RGBDProxy property.'
		status = 1


	# Remote object connection for OmniRobot
	try:
		proxyString = ic.getProperties().getProperty('OmniRobotProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			omnirobot_proxy = OmniRobotPrx.checkedCast(basePrx)
			mprx["OmniRobotProxy"] = omnirobot_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (OmniRobot)', proxyString
			mprx["OmniRobotProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get OmniRobotProxy property.'
		status = 1


	# Remote object connection for Speech
	try:
		proxyString = ic.getProperties().getProperty('SpeechProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			speech_proxy = SpeechPrx.checkedCast(basePrx)
			mprx["SpeechProxy"] = speech_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (Speech)', proxyString
			mprx["SpeechProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get SpeechProxy property.'
		status = 1


	# Remote object connection for TrajectoryRobot2D
	try:
		proxyString = ic.getProperties().getProperty('TrajectoryRobot2DProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			trajectoryrobot2d_proxy = TrajectoryRobot2DPrx.checkedCast(basePrx)
			mprx["TrajectoryRobot2DProxy"] = trajectoryrobot2d_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (TrajectoryRobot2D)', proxyString
			mprx["TrajectoryRobot2DProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get TrajectoryRobot2DProxy property.'
		status = 1

	## CommonBehavior interfaces 
	# Remote object connection for April
	try:
		proxyString = ic.getProperties().getProperty('AprilCommonProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			april_common_proxy = CommonBehaviorPrx.checkedCast(basePrx)
			mprx["AprilCommonProxy"] = april_common_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (AprilCommonProxy)', proxyString
			mprx["AprilCommonProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get AprilCommonProxy property.'
		status = 1

	# Remote object connection for ObjectAgent
	try:
		proxyString = ic.getProperties().getProperty('ObjectAgentCommonProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			object_agent_common_proxy = CommonBehaviorPrx.checkedCast(basePrx)
			mprx["ObjectAgentCommonProxy"] = object_agent_common_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (ObjectAgentCommonProxy)', proxyString
			mprx["ObjectAgentCommonProxy"] = None
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get ObjectAgentCommonProxy property.'
		status = 1

	worker = SpecificWorker(mprx)
	worker.setParams(parameters)
	
	if mprx["OmniRobotProxy"] == None:
		worker.ui.tabWidget.setTabText(0, worker.ui.tabWidget.tabText(0)+" (OFF)")
	if mprx["TrajectoryRobot2DProxy"] == None:
		worker.ui.tabWidget.setTabText(1, worker.ui.tabWidget.tabText(1)+" (OFF)")
	if mprx["SpeechProxy"] == None:
		worker.ui.tabWidget.setTabText(2, worker.ui.tabWidget.tabText(2)+" (OFF)")
	if mprx["JointMotorProxy"] == None:
		worker.ui.tabWidget.setTabText(3, worker.ui.tabWidget.tabText(3)+" (OFF)")
	if mprx["LaserProxy"] == None:
		worker.ui.tabWidget.setTabText(4, worker.ui.tabWidget.tabText(4)+" (OFF)")
	if mprx["RGBDProxy"] == None:
		worker.ui.tabWidget.setTabText(5, worker.ui.tabWidget.tabText(5)+" (OFF)")

	adapter = ic.createObjectAdapter('CommonBehavior')
	adapter.add(CommonBehaviorI(worker, ic), ic.stringToIdentity('commonbehavior'))
	adapter.activate()


	ASRPublish_adapter = ic.createObjectAdapter("ASRPublishTopic")
	asrpublishI_ = ASRPublishI(worker)
	asrpublish_proxy = ASRPublish_adapter.addWithUUID(asrpublishI_).ice_oneway()

	subscribeDone = False
	try:
		asrpublish_topic = topicManager.retrieve("ASRPublish")
		subscribeDone = True
	except Ice.Exception, e:
		asrpublish_topic = None
		print "Error. ASRPublish Topic does not exist (yet)"
		status = 0
	qos = {}
	try:
		asrpublish_topic.subscribeAndGetPublisher(qos, asrpublish_proxy)
		ASRPublish_adapter.activate()
	except:
		pass

	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
