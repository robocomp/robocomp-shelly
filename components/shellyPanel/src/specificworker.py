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

import math

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


##############################################################################
############		MANAGEMENT NUCS IN BACKGROUND		##############
##############################################################################

hosts = dict()

class MyShellyThread(QtCore.QThread):
	def __init__(self, parent = super):
		QtCore.QThread.__init__(self, None)
		self.exiting = False
		# variables for work with ssh
		self.ssh = None
		self.transport = None

	def run(self):
		reloj = QtCore.QTime.currentTime()
		while True:
			time.sleep(1)
			try:
				if reloj.elapsed() > 5000:
					self.loadAverage()
					reloj.restart();
			except:
				traceback.print_exc()
				print "can't load NUCs data"
				time.sleep(10)


	def loadAverage(self):
		global hosts
		i = 1
		maxAttemp = 50
		while i <= 3:
			hostname = 'robonuc'+str(i)+'.local'
			self.ssh_connect(hostname, "robolab", hosts[hostname])
			output=self.runcmd('top -bn '+str(maxAttemp)+' -d 0.01 | grep \'%Cpu(s)\' | gawk \'{print $2+$4+$6}\'')
			#if output:
			if not "ERROR" in output:
				output = output.split('\n')
				aux = 0
				cont = 0
				for load in output:
					if load != '':
						aux += float(load[:-1])
						cont += 1
				aux = aux / maxAttemp
				global loadN1, loadN2
				if i == 1:
					self.ui.load1.setText(aux)
				elif i == 2:
					self.ui.load2.setText(aux)
				elif i == 3:
					self.ui.load3.setText(aux)
			else:
				print 'ERROR with: '+str(hostname)				
				loadN1 = loadN2 = "ERROR"


	def ssh_disconnect (self):
		if self.transport is not None:
			self.transport.close()
		if self.ssh is not None:
			self.ssh.close()

	def ssh_connect(self,hostname,username,password,port=22):
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
	def resetNUC(self, n):
		global hosts
		print "REBOOT NUC" + str(n)
		hostname = 'robonuc'+n+'.local'
		i = 3
		while i > 0:
			self.ssh_connect(hostname,"robolab",hosts[hostname])
			output=self.runcmd('sudo reboot')
			if output:
				print output
				self.ssh_disconnect()
				break
			else:
				i -= 1
				print ' Error reset NUC'+n+'.  Attempt number '+str(i)

	@QtCore.Slot()
	def shutdownNUC(self, n):
		print "POWEROFF NUC" + str(n)
		hostname = 'robonuc'+n+'.local'
		i = 3
		global hosts
		while i > 0:
			self.ssh_connect(hostname,"robolab",hosts[hostname])
			output=self.runcmd('sudo poweroff')
			if output:
				print output
				self.ssh_disconnect()
				#global on_2
				#on_2 = False
				break;
			else:
				i -= 1
				print 'ERROR shutdown NUC'+n+'.  Trying number '+str(i)


	@QtCore.Slot()
	def shutdownNUC1(self):
		self.shutdownNUC(1)
	@QtCore.Slot()
	def resetNUC1(self):
		self.resetNUC(1)
	@QtCore.Slot()
	def shutdownNUC2(self):
		self.shutdownNUC(1)
	@QtCore.Slot()
	def resetNUC2(self):
		self.resetNUC(1)
	@QtCore.Slot()
	def shutdownNUC3(self):
		self.shutdownNUC(1)
	@QtCore.Slot()
	def resetNUC3(self):
		self.resetNUC(1)
	@QtCore.Slot()
	def shutdownNUC4(self):
		print 'TODO shutdownNUC4'
	@QtCore.Slot()
	def resetNUC4(self):
		print 'TODO resetNUC4'


class LaserDraw(QtGui.QGraphicsScene):
	def __init__(self, parent, main):
		super(LaserDraw, self).__init__(parent)
		self.main = main

	def work(self):
		if self.main.ui.tabWidget.currentIndex()==5:
			print 'LaserDraw::work'
			self.clear()
			try:
				if len(self.main.laser_data) <= 0:
					print 'no laser data a'
					return
			except:
				print 'no laser data b'
				return
			print 'eeeeoo'
			ww = self.main.ui.laser_graphicsView.width()
			wh = self.main.ui.laser_graphicsView.height()
			xOff = ww/2.
			yOff = wh/2.

			#painter = QtGui.QPainter()
			#painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

			for point in self.main.laser_data:
				newCoor = self.main.laser_measure2coord(point)
				self.addRect(newCoor[0]-1, newCoor[1]-1, 2, 2)

			#for wm in range(10):
				#w = 1000. * (1.+wm) * self.main.ui.laserSpinBox.value()
				#painter.drawEllipse(QtCore.QRectF(0.5*ww-w/2., 0.5*wh-w/2., w, w))
			#for wm in range(5):
				#w = 200. * (1.+wm) * self.main.ui.laserSpinBox.value()
				#painter.drawEllipse(QtCore.QRectF(0.5*ww-w/2., 0.5*wh-w/2., w, w))
			#painter.end()
			#painter = None

####################################################################################################
class myGraphicsSceneJoyStick(QtGui.QGraphicsScene):
	def __init__(self, parent):
		super(myGraphicsSceneJoyStick,self).__init__() 
		self.move = False
		self.press = False
		self.vaca = None
		self.vacaW = None
		self.vacaH = None
		self.parent = parent
		self.timerJoystick = QtCore.QTime.currentTime()

		self.setSceneRect(-100,-100,200,200)
		# Regla de 3. Para extrapolar el valor de la escena que va de 0 a 100 a los de 0 a 1024 del pwm
		# El negativo es porque en el scene el punto el 0,1 es en el centro hacia abajo.
		self.vAdvanceRobot = 300.
		# No le pongo el negativo porque izquierda y derecha es correcto.
		self.vRotationRobot = 0.2

		posX = 0
		posY = 0
		self.vaca = self.addEllipse(posX-5,posY-5,10,10)      
		self.crosslineW = QtCore.QLineF(-self.width(),posY,self.width(),posY)
		self.crosslineH = QtCore.QLineF(posX,-self.height(),posX,self.height())
		self.vacaW = self.addLine(self.crosslineW)
		self.vacaH = self.addLine(self.crosslineH)
		self.update()
		self.parent.setRobotSpeed(0,0)
 
 
	def mousePressAndMoveEvent(self,event):
		if self.press and self.move:
			self.removeItem(self.vaca)
			self.removeItem(self.vacaW)
			self.removeItem(self.vacaH)
			posX = event.scenePos().x()
			posY = event.scenePos().y()
			self.vaca = self.addEllipse(posX-5,posY-5,10,10)      
			self.crosslineW = QtCore.QLineF(-self.width(),posY,self.width(),posY)
			self.crosslineH = QtCore.QLineF(posX,-self.height(),posX,self.height())
			self.vacaW = self.addLine(self.crosslineW)
			self.vacaH = self.addLine(self.crosslineH)
			self.update()
			if self.timerJoystick.elapsed() > 500:
				cmd_z = -posY*self.vAdvanceRobot/200.
				cmd_r = posX*self.vRotationRobot/200
				print "Moving--> Advance: ", cmd_z, " <--> Rotation: ", cmd_r
				self.parent.setRobotSpeed(cmd_z, cmd_r)
				self.timerJoystick.restart()
  
	def mousePressEvent(self,event):
		self.press = True
		self.mousePressAndMoveEvent(event)

	def mouseMoveEvent(self,event):
		self.move = True
		self.mousePressAndMoveEvent(event)

	def mouseReleaseEvent(self,event):
		self.press = False
		self.move = False

		self.removeItem(self.vaca)
		self.removeItem(self.vacaW)
		self.removeItem(self.vacaH)
		posX = 0
		posY = 0
		self.vaca = self.addEllipse(posX-5,posY-5,10,10)      
		self.crosslineW = QtCore.QLineF(-self.width(), posY,self.width(), posY)
		self.crosslineH = QtCore.QLineF(posX, -self.height(), posX, self.height())
		self.vacaW = self.addLine(self.crosslineW)
		self.vacaH = self.addLine(self.crosslineH)
		self.update()
		self.parent.setRobotSpeed(0,0)
####################################################################################################
####################################################################################################
class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		self.ui.navigationButton.clicked.connect(self.goNavigation)


		self.ui.sayButton.clicked.connect(self.sayText)
		self.ui.jointButton.clicked.connect(self.jointCommand)

		self.sceneJoyStick = myGraphicsSceneJoyStick(self)

		self.laserDrawer = LaserDraw(self.ui.laser_graphicsView, self)
		self.ui.laser_graphicsView.setScene(self.laserDrawer)

		self.vaca = self.ui.graphicsViewJoyStick.setScene(self.sceneJoyStick)
		self.sceneJoyStick.update()
		
		# variables for work with ssh
		#self.ssh = None
		#self.transport = None
		#self.hosts = dict()
		
		self.thread = MyShellyThread()
		#workerThread = new WorkerThread(this);
		self.thread.start();
		try:
			for line in open(os.getenv("HOME")+'/.rcremote', 'r').readlines():
				try:
					s = line.strip().split('#', 1)
					if len(s) < 2:
						continue
					global hosts
					hosts[s[0]] = s[1]
				except:
					print ("can't find password for "+str(s[0])+" in ~/.rcremote  (format 'host#password')")
		except:
			print ("can't open file ~/.rcmemote")
			
		self.ui.shutdown1Button.clicked.connect(self.thread.shutdownNUC1)
		self.ui.shutdown2Button.clicked.connect(self.thread.shutdownNUC2)
		self.ui.shutdown3Button.clicked.connect(self.thread.shutdownNUC3)
		self.ui.shutdown4Button.clicked.connect(self.thread.shutdownNUC4)
		self.ui.reset1Button.clicked.connect(self.thread.resetNUC1)
		self.ui.reset2Button.clicked.connect(self.thread.resetNUC2)
		self.ui.reset3Button.clicked.connect(self.thread.resetNUC3)
		self.ui.reset4Button.clicked.connect(self.thread.resetNUC4)

		self.initializeMotors()


	def initializeMotors(self):
		self.joint_motors = self.jointmotor_proxy.getAllMotorParams()
		print 'Motors: ',
		for item in self.joint_motors:
			print item.name
			self.ui.jointCombo.addItem(item.name)
		if len(self.joint_motors) == 0:
			print 'JointMotor: Error: No motors.'
		self.states = self.jointmotor_proxy.getAllMotorState()



	def setRobotSpeed(self,vAdvance,vRotation):
		self.omnirobot_proxy.setSpeedBase(0, vAdvance, vRotation)
		return True
	
	def sayText(self):
		self.speech_proxy.say(self.ui.ttsEdit.text(), False)
	
	def jointCommand(self):
		goal = MotorGoalPosition()
		goal.position = self.ui.jointTarget.value()
		goal.name = str(self.ui.jointCombo.currentText())
		goal.maxSpeed = self.ui.jointSpeed.value()
		print "target pos" , goal.position, "name", goal.name, "maxSpeed" , goal.maxSpeed
		self.jointmotor_proxy.setPosition(goal)


	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		self.computeJointMotor()
		self.computeLaser()
		self.computeRGBD()

	def computeJointMotor(self):
		if self.ui.tabWidget.currentIndex() != 4:
			return;
		self.states = self.jointmotor_proxy.getAllMotorState()
		state = self.states[str(self.ui.jointCombo.currentText())]
		self.ui.jointPosition.setValue(state.pos)

	def computeLaser(self):
		if self.ui.tabWidget.currentIndex() != 5:
			return;

		#try:
		self.laser_data, basura = self.laser_proxy.getLaserAndBStateData()
		print '-----'
		m = -1
		M = -1
		for d in self.laser_data:
			if m == -1 or d.dist < m:
				m = d.dist
			if M == -1 or d.dist > M:
				M = d.dist
		print len(self.laser_data), ' from', m, 'to', M
		
		self.laserDrawer.update()
		#except:
			#print 'No laser connection.'
		self.laserDrawer.work()

	def laser_measure2coord(self, measure):
		const_mul = self.ui.laserSpinBox.value()
		x = math.cos(measure.angle-0.5*math.pi)*measure.dist*const_mul+(0.5*self.width())
		y = math.sin(measure.angle-0.5*math.pi)*measure.dist*const_mul+(0.5*self.height())
		return x, y

	
	
	def computeRGBD(self):
		if self.ui.tabWidget.currentIndex() != 6:
			return;


	def paintEvent(self, event=None):
		self.laserDrawer.update()


	#
	# newText
	#
	def newText(self, text):
		#
		# YOUR CODE HERE
		#
		pass

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








