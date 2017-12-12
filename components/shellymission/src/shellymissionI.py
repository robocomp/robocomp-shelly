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

additionalPathStr = ''
icePaths = []
try:
	icePaths.append('/opt/robocomp/interfaces')
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

ice_AGMExecutive = False
for p in icePaths:
	if os.path.isfile(p+'/AGMExecutive.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"AGMExecutive.ice"
		Ice.loadSlice(wholeStr)
		ice_AGMExecutive = True
		break
if not ice_AGMExecutive:
	print 'Couldn\'t load AGMExecutive'
	sys.exit(-1)
from RoboCompAGMExecutive import *
ice_ShellyMission = False
for p in icePaths:
	if os.path.isfile(p+'/ShellyMission.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"ShellyMission.ice"
		Ice.loadSlice(wholeStr)
		ice_ShellyMission = True
		break
if not ice_ShellyMission:
	print 'Couldn\'t load ShellyMission'
	sys.exit(-1)
from RoboCompShellyMission import *

class ShellyMissionI(ShellyMission):
	def __init__(self, worker):
		self.worker = worker

	def setMission(self, path, c):
		return self.worker.setMission(path)
