#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2015 by YOUR NAME HERE
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

import sys, traceback, Ice, IceStorm, subprocess, threading, time, Queue, os, copy

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *

from specificworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior
Ice.loadSlice(preStr+"JointMotor.ice")
import RoboCompJointMotor
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
import RoboCompTrajectoryRobot2D
Ice.loadSlice(preStr+"OmniRobot.ice")
import RoboCompOmniRobot
Ice.loadSlice(preStr+"Speech.ice")
import RoboCompSpeech
Ice.loadSlice(preStr+"ASRPublish.ice")
import RoboCompASRPublish
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior


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
	elif len(params) == 0:
		params.append('--Ice.Config=config')
	ic = Ice.initialize(params)
	status = 0
	mprx = {}
	try:

		# Remote object connection for JointMotor
		try:
			proxyString = ic.getProperties().getProperty('JointMotorProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				jointmotor_proxy = RoboCompJointMotor.JointMotorPrx.checkedCast(basePrx)
				mprx["JointMotorProxy"] = jointmotor_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (JointMotor)', proxyString
				#traceback.print_exc()
				status = 1
		except Ice.Exception, e:
			print e
			print 'Cannot get JointMotorProxy property.'
			status = 1


		# Remote object connection for TrajectoryRobot2D
		try:
			proxyString = ic.getProperties().getProperty('TrajectoryRobot2DProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				trajectoryrobot2d_proxy = RoboCompTrajectoryRobot2D.TrajectoryRobot2DPrx.checkedCast(basePrx)
				mprx["TrajectoryRobot2DProxy"] = trajectoryrobot2d_proxy
			except Ice.Exception:
				mprx["TrajectoryRobot2DProxy"] = None
				print 'Cannot connect to the remote object (TrajectoryRobot2D)', proxyString
				#traceback.print_exc()
				status = 1
		except Ice.Exception, e:
			print e
			print 'Cannot get TrajectoryRobot2DProxy property.'
			status = 1


		# Remote object connection for OmniRobot
		try:
			proxyString = ic.getProperties().getProperty('OmniRobotProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				omnirobot_proxy = RoboCompOmniRobot.OmniRobotPrx.checkedCast(basePrx)
				mprx["OmniRobotProxy"] = omnirobot_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (OmniRobot)', proxyString
				#traceback.print_exc()
				status = 1
				mprx["OmniRobotProxy"] = None
		except Ice.Exception, e:
			print e
			print 'Cannot get OmniRobotProxy property.'
			status = 1


		# Remote object connection for Speech
		try:
			proxyString = ic.getProperties().getProperty('SpeechProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				speech_proxy = RoboCompSpeech.SpeechPrx.checkedCast(basePrx)
				mprx["SpeechProxy"] = speech_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (Speech)', proxyString
				#traceback.print_exc()
				status = 1
				mprx["SpeechProxy"] = None
		except Ice.Exception, e:
			print e
			print 'Cannot get SpeechProxy property.'
			status = 1


		# Topic Manager
		proxy = ic.getProperties().getProperty("TopicManager.Proxy")
		obj = ic.stringToProxy(proxy)
		topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
	except:
			traceback.print_exc()
			status = 1


	if status == 0 or True:
		worker = SpecificWorker(mprx)


		adapter = ic.createObjectAdapter('CommonBehavior')
		adapter.add(CommonBehaviorI(worker, ic), ic.stringToIdentity('commonbehavior'))
		adapter.activate()


		ASRPublish_adapter = ic.createObjectAdapter("ASRPublishTopic")
		asrpublishI_ = ASRPublishI(worker)
		asrpublish_proxy = ASRPublish_adapter.addWithUUID(asrpublishI_).ice_oneway()

		subscribeDone = False
		asrpublish_topic = None
		try:
			asrpublish_topic = topicManager.retrieve("ASRPublish")
			subscribeDone = True
			qos = {}
			asrpublish_topic.subscribeAndGetPublisher(qos, asrpublish_proxy)
			ASRPublish_adapter.activate()
		except Ice.Exception, e:
			print "Error. Topic does not exist (yet)"
			status = 0
			time.sleep(1)
		except:
			pass


#		adapter.add(CommonBehaviorI(<LOWER>I, ic), ic.stringToIdentity('commonbehavior'))

		app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
