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

import sys, os, traceback, time

from PySide import *
from genericworker import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

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
		print 'SpecificWorker.compute...'
		#computeCODE
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True


	#
	# structuralChange
	#
	def structuralChange(self, w):
		#
		#subscribesToCODE
		#
		pass


	#
	# edgesUpdated
	#
	def edgesUpdated(self, modifications):
		#
		#subscribesToCODE
		#
		pass


	#
	# edgeUpdated
	#
	def edgeUpdated(self, modification):
		#
		#subscribesToCODE
		#
		pass


	#
	# symbolUpdated
	#
	def symbolUpdated(self, modification):
		#
		#subscribesToCODE
		#
		pass


	#
	# symbolsUpdated
	#
	def symbolsUpdated(self, modifications):
		#
		#subscribesToCODE
		#
		pass


	#
	# reloadConfigAgent
	#
	def reloadConfigAgent(self):
		ret = bool()
		#
		#implementCODE
		#
		return ret


	#
	# activateAgent
	#
	def activateAgent(self, prs):
		ret = bool()
		#
		#implementCODE
		#
		return ret


	#
	# setAgentParameters
	#
	def setAgentParameters(self, prs):
		ret = bool()
		#
		#implementCODE
		#
		return ret


	#
	# getAgentParameters
	#
	def getAgentParameters(self):
		ret = ParameterMap()
		#
		#implementCODE
		#
		return ret


	#
	# killAgent
	#
	def killAgent(self):
		#
		#implementCODE
		#
		pass


	#
	# uptimeAgent
	#
	def uptimeAgent(self):
		ret = int()
		#
		#implementCODE
		#
		return ret


	#
	# deactivateAgent
	#
	def deactivateAgent(self):
		ret = bool()
		#
		#implementCODE
		#
		return ret


	#
	# getAgentState
	#
	def getAgentState(self):
		ret = StateStruct()
		#
		#implementCODE
		#
		return ret





