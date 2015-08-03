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

import sys, os, Ice, traceback
import socket, paramiko, time

from PySide import *
from genericworker import *


from collections import OrderedDict



ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"JointMotor.ice")
from RoboCompJointMotor import *
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
from RoboCompTrajectoryRobot2D import *
Ice.loadSlice(preStr+"OmniRobot.ice")
from RoboCompOmniRobot import *
Ice.loadSlice(preStr+"Speech.ice")
from RoboCompSpeech import *
Ice.loadSlice(preStr+"ASRPublish.ice")
from RoboCompASRPublish import *
Ice.loadSlice(preStr+"CommonBehavior.ice")
from RoboCompCommonBehavior import *


from commonbehaviorI import *
from asrpublishI import *


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
	
		# timer for the get informations of nucs
		timer2 = QtCore.QTimer(self)		
		timer2.timeout.connect(self.computeTemp)
		timer2.start(10000)
		timer3 = QtCore.QTimer(self)
		timer3.timeout.connect(self.computeLoad)
		timer3.start(15000)
		timer4 = QtCore.QTimer(self)
		timer4.timeout.connect(self.computeWifi)
		timer4.start(5000)
		
		self.ui.navigationButton.clicked.connect(self.goNavigation)
		self.ui.ikButton.clicked.connect(self.goIK)
		self.ui.vikButton.clicked.connect(self.goVIK)
		self.ui.lowButton.clicked.connect(self.lowPosition)
		self.ui.topButton.clicked.connect(self.topPosition)
		self.ui.shutdown1Button.clicked.connect(self.shutdownNUC1)
		self.ui.shutdown2Button.clicked.connect(self.shutdownNUC2)
		self.ui.reset1Button.clicked.connect(self.resetNUC1)
		self.ui.reset2Button.clicked.connect(self.resetNUC2)
		
		self.on_1 = True
		self.on_2 = True
		
		# variables for work with ssh
		self.ssh = None
		self.transport = None
		self.hosts = dict()
		try:
			for line in open(os.getenv("HOME")+'/.rcremote', 'r').readlines():
				try:
					s = line.strip().split('#', 1)
					if len(s) < 2:
						continue
					self.hosts[s[0]] = s[1]
				except:
					print ("can't find password for "+str(s[0])+" in ~/.rcremote  (format 'host#password')")
		except:
			print ("can't open file ~/.rcmemote")


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
		#try:
		#	differentialrobot_proxy.setSpeed(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e		
		#return True
		

	def computeTemp(self):
		if self.on_1 is True and self.on_2 is True:
			self.loadTemp()
		else:
			pass

	def computeLoad(self):
		if self.on_1 is True and self.on_2 is True:
			self.loadAverage()
		else:
			pass

	def computeWifi(self):
		if self.on_1 is True and self.on_2 is True:
			self.loadWiFiSignal()
		else:
			pass


	#
	# newText
	#
	def newText(self, text):
		#
		# YOUR CODE HERE
		#
		pass


	def loadTemp(self):
		hostnames = ['robonuc1.local','robonuc2.local']
		for hostname in hostnames:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('sensors')
			if output:
				output = output.split('\n')
				temC1 = (output[8].split('         ')[1].split('  ')[0])[1:-3]
				temC2 = (output[9].split('         ')[1].split('  ')[0])[1:-3]
				if hostname == 'robonuc1.local':					
					self.ui.temperatura1.setText('Temp: '+ str((float(temC1) + float(temC2))/float(2.0)))
				else:
					self.ui.temperatura2.setText('Temp: '+ str((float(temC1) + float(temC2))/float(2.0)))
				self.disconnect()
			else:
				print 'ERROR with: '+str(hostname)



	def loadAverage(self):
		maxAttemp = 50
		hostnames = ['robonuc1.local','robonuc2.local']
		for hostname in hostnames:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('top -bn '+str(maxAttemp)+' -d 0.01 | grep \'%Cpu(s)\' | gawk \'{print $2+$4+$6}\'')
			if output:
				output = output.split('\n')
				aux = 0
				cont = 0
				for load in output:
					if load != '':
						aux += float(load[:-1])
						cont += 1
				aux = aux / maxAttemp
				if hostname == 'robonuc1.local':
					self.ui.loadavg1.setText('Load: '+str(aux))
				else:
					self.ui.loadavg2.setText('Load: '+str(aux))
			else:
				print 'ERROR with: '+str(hostname)



	def loadWiFiSignal(self):
		ifaceN1 = 'wlan2'
		ifaceN2 = 'wlan3'
		hostnames = ['robonuc1.local','robonuc2.local']
		for hostname in hostnames:
			self.connect(hostname,"robolab",self.hosts[hostname])
			if hostname == 'robonuc1.local':
				output=self.runcmd('iwconfig '+ifaceN1+' | grep -e "Signal level"')
			else:
				output=self.runcmd('iwconfig '+ifaceN2+' | grep -e "Signal level"')
			if output:
				signal = 'Wifi: '+((output.split('='))[2])

				if hostname == 'robonuc1.local':
					#self.ui.loadwifi1.setText(signal)
					print 'Wifi NUC1',signal
				else:
					#self.ui.loadwifi2.setText(signal)
					print 'Wifi NUC2',signal
			else:
				print 'ERROR with: '+str(hostname)			
######################################################################	
#### NAVEGACION
	@QtCore.Slot()
	def goNavigation(self):
		# TODO: positionCOMBO list:elements of room with position
		print "NAVIGATION"
		tp = TargetPose()
		tp.x = self.ui.xBox.value()
		tp.z = self.ui.zBox.value()
		tp.y = 0
		tp.rx = tp.rz = tp.ry = 0
		if self.ui.ignoreAngle.isChecked() is False:
			tp.ry = self.ui.aBox.value()				
		threshold = self.ui.thBox.value()

		try:
			#self.trajectoryrobot2d_proxy.go(tp)
			self.trajectoryrobot2d_proxy.goReferenced(tp, 0, 0, threshold)
		except:
			print "Error: Sleeping 10 :-)"
			time.sleep(10)
######################################################################	
#### MANIPULACION
	@QtCore.Slot()
	def lowPosition(self):
		print "LOW POSITION"
		mapa = [['rightWrist2',0.0], ['rightWrist1',0.0], ['rightForeArm',0.1], ['rightElbow',0.15], ['rightShoulder3',0.10], ['rightShoulder2',-0.10], ['rightShoulder1',-0.10]]
		self.gotoPosition(mapa)

	@QtCore.Slot()
	def topPosition(self):
		print "TOP POSITION"
		mapa = [['rightWrist2',0.0], ['rightWrist1',0.0], ['rightForeArm',0.1], ['rightElbow',0.10], ['rightShoulder3',0.50], ['rightShoulder2',-0.20], ['rightShoulder1',-3.0]]
		self.gotoPosition(mapa)

	def gotoPosition(self, mapa):
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.name = str(motor[0])
			goal.position = float(motor[1])
			goal.maxSpeed = float(0.9)
			time.sleep(0.5)
			try:
				self.jointmotor_proxy.setPosition(goal)
			except CollisionException:
				print "Error en gotoPosition: ", CollisionException
				
	
	@QtCore.Slot()
	def goIK(self):
		print "AUN NO ESTA PREPARADO: METER IK PROXY"
		#import RoboCompInverseKinematics
		#pose6D = RoboCompInverseKinematics.Pose6D() #target al que se movera
		#pose6D.x  = float(self.ui.txbox.value())
		#pose6D.y  = float(self.ui.tybox.value())
		#pose6D.z  = float(self.ui.tzbox.value())
		#pose6D.rx = float(self.ui.rxbox.value())
		#pose6D.ry = float(self.ui.rybox.value())
		#pose6D.rz = float(self.ui.rzbox.value())
		#print '---> ',pose6D
		#weights = RoboCompInverseKinematics.WeightVector() #vector de pesos
		#weights.x = 1
		#weights.y = 1
		#weights.z = 1
		#weights.rx = 0.1
		#weights.ry = 0.1
		#weights.rz = 0.1

		#part = "RIGHTARM"
		#try:
			#identificador = self.inversekinematics_proxy.setTargetPose6D(part,pose6D, weights)
		#except RoboCompInverseKinematics.IKException, e:
			#print "Expection in tester (sendPose): ", e
	
	@QtCore.Slot()
	def goVIK(self):
		print "AUN NO ESTA PREPARADO: METER VIK PROXY"
		#import RoboCompInverseKinematics
		#pose6D = RoboCompInverseKinematics.Pose6D() #target al que se movera
		#pose6D.x  = float(self.ui.txbox.value())
		#pose6D.y  = float(self.ui.tybox.value())
		#pose6D.z  = float(self.ui.tzbox.value())
		#pose6D.rx = float(self.ui.rxbox.value())
		#pose6D.ry = float(self.ui.rybox.value())
		#pose6D.rz = float(self.ui.rzbox.value())
		#print '---> ',pose6D
		#weights = RoboCompInverseKinematics.WeightVector() #vector de pesos
		#weights.x = 1
		#weights.y = 1
		#weights.z = 1
		#weights.rx = 0.1
		#weights.ry = 0.1
		#weights.rz = 0.1
		
		#try:
			##PRIMERO MOVEMOS CABEZA:
			#axis = RoboCompInverseKinematics.Axis() #vector de pesos
			#axis.x = 0
			#axis.y = 0
			#axis.z = 1
			#part = "HEAD"
			#self.inversekinematics_proxy.setTargetAlignaxis(part, pose6D, axis)
			## DESPUES MOVEMOS BRAZO YEAH!
			#part = "RIGHTARM"
			#identificador = self.inversekinematics_proxy.setTargetPose6D(part,pose6D, weights)
		#except RoboCompInverseKinematics.IKException, e:
			#print "Expection in tester (sendPose): ", e

######################################################################	
#### OTRAS COSAS			

###############################
###			management nucs
###############################
	def disconnect (self):
		if self.transport is not None:
			self.transport.close()
		if self.ssh is not None:
			self.ssh.close()

	def connect(self,hostname,username,password,port=22):
		self.hostname = hostname
		self.username = username
		self.password = password

		self.ssh = paramiko.SSHClient()
		#Don't use host key auto add policy for production servers
		self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		self.ssh.load_system_host_keys()
		try:
			self.ssh.connect(hostname,port,username,password)
			self.transport=self.ssh.get_transport()
		except (socket.error,paramiko.AuthenticationException) as message:
			print "ERROR: SSH connection to "+self.hostname+" failed: " +str(message)
			#sys.exit(1)
		return  self.transport is not None

	def runcmd(self,cmd,sudoenabled=True):
		if sudoenabled:
			fullcmd="echo " + self.password + " |   sudo -S -p '' " + cmd
		else:
			fullcmd=cmd
		if self.transport is None:
			return "ERROR: connection was not established"
		session=self.transport.open_session()
		session.set_combine_stderr(True)
		if sudoenabled:
			session.get_pty()
		session.exec_command(fullcmd)
		stdout = session.makefile('rb', -1)
		output=stdout.read()
		session.close()
		return output

	@QtCore.Slot()
	def shutdownNUC1(self):
		print "POWEROFF NUC1"
		hostname = 'robonuc1.local'
		i = 3
		while i > 0:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('sudo poweroff')
			if output:
				print output
				self.disconnect()
				self.on_1 = False
				break
			else:
				i -= 1
				print 'ERROR shutdown NUC1. Trying number '+str(i)

	@QtCore.Slot()
	def resetNUC1(self):
		print "REBOOT NUC1"
		hostname = 'robonuc1.local'
		i = 3
		while i > 0:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('sudo reboot')
			if output:
				print output
				self.disconnect()
				break
			else:
				i -= 1
				print ' Error reset NUC1.  Trying number '+str(i)

	@QtCore.Slot()
	def shutdownNUC2(self):
		print "POWEROFF NUC2"
		hostname = 'robonuc2.local'
		i = 3
		while i > 0:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('sudo poweroff')
			if output:
				print output
				self.disconnect()
				self.on_2 = False
				break;
			else:
				i -= 1
				print 'ERROR shutdown NUC2.  Trying number '+str(i)

	@QtCore.Slot()
	def resetNUC2(self):
		print "REBOOT NUC2"
		hostname = 'robonuc2.local'
		i = 3
		while i > 0:
			self.connect(hostname,"robolab",self.hosts[hostname])
			output=self.runcmd('sudo reboot')
			if output:
				print output
	                        self.disconnect()
				break
			else:
				i -= 1
				print 'ERROR reset NUC1. Trying number '+str(i)


###################################################################################################















	#
	# reloadConfig
	#
	def reloadConfig(self):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# setPeriod
	#
	def setPeriod(self, period):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# getState
	#
	def getState(self):
		ret = State()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# setParameterList
	#
	def setParameterList(self, l):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# timeAwake
	#
	def timeAwake(self):
		ret = int()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# getParameterList
	#
	def getParameterList(self):
		ret = ParameterList()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# killYourSelf
	#
	def killYourSelf(self):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# getPeriod
	#
	def getPeriod(self):
		ret = int()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# newText
	#
	def newText(self, text):
		#
		# YOUR CODE HERE
		#
		pass










