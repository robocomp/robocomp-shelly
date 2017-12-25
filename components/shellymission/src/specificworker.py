#
# Copyright (C) 2017 by YOUR NAME HERE
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

import sys, os, traceback, time

from PySide import QtGui, QtCore
from genericworker import *
from collections import namedtuple
import datetime

Mission = namedtuple('Mission', ['name', 'time', 'path'])

class MissionQueue(object):
	def __init__(self, defaultMissionPath=None):
		self.defaultMissionPath = defaultMissionPath
		self.data = []
		self.generators = []
	def addMission(self, mission):
		self.data.append(mission)
		self.data = sorted(self.data, key=lambda x: x.time)
	def addGenerator(self, generator):
		self.generators.append(generator)
	def runGenerators(self):
		for generator in self.generators:
			generator.handleMissionQueue(self)
	def get(self):
		got = None
		now = datetime.datetime.now()
		for mission in self.data:
			if mission.time <= now:
				got = mission
				break
		print 'returning misssion', got
 		print self.data, '\n'
		if got != None:
			print 'removing mission', got
			self.data.remove(got)
		print 'queue'
 		print self.data, '\n'
		if got == None:
			got = Mission('default mission', datetime.datetime.now(), self.defaultMissionPath)
			print 'No mission found in the queue, using the default mission provided:', got
		return got

	def __str__(self):
		return self.getText()

	def getText(self):
		ret = ''
		for m in self.data:
			ret += str(m.time) + ': ' + str(m.name) + '\n'
		return ret

class PeriodicMissionGenerator(object):
	def __init__(self, name, period, path):
		self.name   = name
		self.period = period
		self.path   = path
	def handleMissionQueue(self, queue):
		n = 0
		stored = None
		for m in queue.data:
			if m.name == self.name:
				n += 1
				stored = m
		if n == 0:
			queue.addMission(Mission(self.name, datetime.datetime.now(),               self.path))
			queue.addMission(Mission(self.name, datetime.datetime.now() + self.period, self.path))
		if n == 1:
			queue.addMission(Mission(self.name,             stored.time + self.period, self.path))

class TimedMissionGenerator(object):
	def __init__(self, name, timeOfDay, path):
		self.name   = name
		self.path   = path
		self.time  = timeOfDay
	def handleMissionQueue(self, queue):
		n = 0
		stored = None
		for m in queue.data:
			if m.name == self.name:
				n += 1
				stored = m
		if n == 0:
			day = datetime.date.today()
			nextTime = datetime.datetime.combine(day, self.time)
			if nextTime < datetime.datetime.now():
				day += datetime.timedelta(days=1)
				nextTime = datetime.datetime.combine(day, self.time)
			queue.addMission(Mission(self.name, nextTime, self.path))



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

		self.worldVersion = int(self.agmexecutive_proxy.getModel().version)
		self.planVersion = -1

		self.currentMission = None
		self.plan = ''
		self.inhibition = False
		print 'init inhibition as false', self.inhibition

		self.queue = MissionQueue(defaultMissionPath='/home/robocomp/robocomp/components/robocomp-shelly/etc/targetGoPoseWhereHuman.aggt')
		self.queue.addGenerator(PeriodicMissionGenerator('goToTableA', datetime.timedelta(seconds=300), '/home/robocomp/robocomp/components/robocomp-shelly/etc/targetReachTableA.aggt'))
		self.queue.addGenerator(PeriodicMissionGenerator('goToTableB', datetime.timedelta(seconds=300), '/home/robocomp/robocomp/components/robocomp-shelly/etc/targetReachTableB.aggt'))
		self.queue.addGenerator(TimedMissionGenerator(   'goToTableC', datetime.time(12, 30),           '/home/robocomp/robocomp/components/robocomp-shelly/etc/targetReachTableC.aggt'))

		self.ui.currentLabel.setText('<b>none</b>')

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'compute'
		#
		#  Make sure we've got the plan
		#
		try:
			print self.planVersion, self.worldVersion, len(plan)
		except NameError:
			self.agmexecutive_proxy.broadcastPlan()
			# return
		#
		#  Set label's content using the missions in the queue
		#
		self.ui.label.setText(self.queue.getText())
		#
		#  Update mission queue with the new missions that the mission generators might trigger
		#
		print 'run generators'
		self.queue.runGenerators()
		if self.inhibition:
			print 'return by inhibition'
			return
		#
		#  Send a new mission in case
		#
		print '-------------------------------------------------'
		try:
			print self.worldVersion, self.planVersion
			if self.planVersion != self.worldVersion and self.planVersion != -1:
				print 'same versions'
				return
			length = len(self.plan)
			if length != 0:
				print 'plan underway'
				return
		except:
			pass



		# If we get here, it's time to move to the next task

		print 'NEXT MISSION!'
		self.currentMission = self.queue.get()

		print self.currentMission
		if self.currentMission != None:
			self.setMission(self.currentMission.path)

		if self.currentMission == None:
			self.ui.currentLabel.setText('<b>none</b>')
		else:
			self.ui.currentLabel.setText(str(self.currentMission.time) + ': ' + str(self.currentMission.name) + '\n')

		return True

	def setMission(self, path, enqueueCurrentIfAny=False):
		print 'Sending new mission', path
		self.inhibition = True
		if enqueueCurrentIfAny and self.currentMission != None:
			self.queue.addMission(self.currentMission)
		print 'set inhibition as true __setMission__', self.inhibition
		self.agmexecutive_proxy.setMission(path)

	def activateAgent(self, prs):
		plan = prs['plan'].value.strip()
		print 'PRS', prs['modelversion'].value
		try:
			nextPlanVersion = int(prs['modelversion'].value)
			# if nextPlanVersion != self.planVersion:
				# self.inhibition = False
				# print 'set inhibition as false __activateAgent__', self.inhibition
			if nextPlanVersion == self.worldVersion and len(plan) == 0:
				self.currentMission = None
				self.inhibition = False
				print 'set inhibition as false __activateAgent__', self.inhibition
			self.planVersion = nextPlanVersion
		except KeyError:
			return False
		return True

	def structuralChange(self, w):
		self.worldVersion = int(w.version)







	#
	# edgesUpdated
	#
	def edgesUpdated(self, modifications):
		pass


	#
	# edgeUpdated
	#
	def edgeUpdated(self, modification):
		pass


	#
	# symbolUpdated
	#
	def symbolUpdated(self, modification):
		pass


	#
	# symbolsUpdated
	#
	def symbolsUpdated(self, modifications):
		pass


	#
	# reloadConfigAgent
	#
	def reloadConfigAgent(self):
		ret = bool()
		return ret

	#
	# setAgentParameters
	#
	def setAgentParameters(self, prs):
		print 'GOT', prs
		return True

	#
	# getAgentParameters
	#
	def getAgentParameters(self):
		ret = ParameterMap()
		return ret

	#
	# killAgent
	#
	def killAgent(self):
		pass

	#
	# uptimeAgent
	#
	def uptimeAgent(self):
		ret = int()
		return ret

	#
	# deactivateAgent
	#
	def deactivateAgent(self):
		ret = bool()
		return ret

	#
	# getAgentState
	#
	def getAgentState(self):
		ret = StateStruct()
		return ret
