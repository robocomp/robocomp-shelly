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

import sys, os, Ice

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
from RoboCompCommonBehavior import *

class CommonBehaviorI(CommonBehavior):
	def __init__(self, worker):
		self.worker = worker

	def reloadConfig(self, c):
		return self.worker.reloadConfig()
	def setPeriod(self, period, c):
		return self.worker.setPeriod(period)
	def getState(self, c):
		return self.worker.getState()
	def setParameterList(self, l, c):
		return self.worker.setParameterList(l)
	def timeAwake(self, c):
		return self.worker.timeAwake()
	def getParameterList(self, c):
		return self.worker.getParameterList()
	def killYourSelf(self, c):
		return self.worker.killYourSelf()
	def getPeriod(self, c):
		return self.worker.getPeriod()





