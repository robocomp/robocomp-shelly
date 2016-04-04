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
import sys, time, os
import speech_recognition as sr

from PySide import *
from genericworker import *


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
Ice.loadSlice(preStr+"AGMExecutive.ice")
from RoboCompAGMExecutive import *



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		self.setState('waiting_command')
		self.setLastText('')
		self.setLastCommand('')

	def setParams(self, params):
		return True

	def countPositiveNegative(self, text):
		yesList = ['yes', 'see', 'yep', 'yup', 'affirmative', 'correct', 'positive']
		yesCount = len(set(yesList)&set(text.split(' ')))
		noList = ['no', 'nop', 'nope', 'negative', 'error', 'incorrect']
		noCount = len(set(noList)&set(text.split(' ')))
		return yesCount, noCount

	def wordsInText(self, words, text):
		if type(words) != type([]):
			w = words.split()
		else:
			w = words
		if type(text) != type([]):
			t = text.split()
		else:
			t = text
		return len(set(w)&set(t))
		

	def guessCommand(self, text):
		if len(text) == 0:
			return ''

		mugCount = self.wordsInText('mug map coffee', text)
		plateCount = self.wordsInText('plate', text)
		ginCount = self.wordsInText('gin', text)

		if 'bring me' in text:
			if mugCount: return 'bing_mug'
			if plateCount: return 'bing_plate'
			if ginCount: return 'bing_gin'
		return ''
	
	def send_mission(self):
		if self.lastCommand == 'bring_mug':
			self.agmexecutive_proxy.setMission('/home/robocomp/robocomp/components/robocomp-shelly/etc/targetGraspMug.aggt')
		else:
			print 'don\'t have such mission yet'
	
	@QtCore.Slot()
	def compute(self):
		if self.state == 'waiting_command':
			print ('compute', self.state)
			text = self.getAudio('command_')
			command = self.guessCommand(text)
			if len(command) > 0:
				self.setLastCommand(command)
				self.setState('waiting_confirmation')
				self.log('Do yo want me to perform the following command?: ' + self.lastCommand)
			else:
				self.log('I didn\'t understood the mission')
		elif self.state == 'waiting_confirmation':
			print ('compute', self.state)
			text = self.getAudio('confirmation_')
			yesCount, noCount = self.countPositiveNegative(text)
			if yesCount > 0 and noCount == 0:
				self.send_mission()
				self.setState('waiting_command')
			elif yesCount == 0 and noCount > 0:
				self.log('Ok, let\'s start over')
				self.setState('waiting_command')
			else:
				self.log('I didn\'t get that')
		else:
			print ('unknown state', self.state)
			sys.exit(-1)
		return True
	
	def log(self, text):
		self.ui.textEdit.insertPlainText(text + '\n')

	def getAudio(self, pattern):
		GOOGLE_SPEECH_KEY = 'AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw'
		r = sr.Recognizer()
		QtCore.QCoreApplication.processEvents()
		os.system('rec -q -V0 -r 44.1k /tmp/'+pattern+'.wav  silence -l 1 0.1 3% 0 2.0 3%')
		QtCore.QCoreApplication.processEvents()
		filename = pattern + time.strftime('%Y%m%d%H%M%S', time.localtime(time.time())) + '.wav'
		QtCore.QCoreApplication.processEvents()
		os.system('cp /tmp/'+pattern+'.wav ' + filename)
		QtCore.QCoreApplication.processEvents()
		with sr.WavFile('/tmp/'+pattern+'.wav') as source:
			audio = r.record(source)
		try:
			#QtCore.QCoreApplication.processEvents()
			#os.system('mplayer ' + filename + ' &')
			#QtCore.QCoreApplication.processEvents()
			transcription = str(r.recognize_google(audio, language='en-US', key=GOOGLE_SPEECH_KEY)).lower()
			self.setLastText(transcription)
			return transcription
		except LookupError:
			print('Could not understand audio')
			return ''
		except sr.UnknownValueError:
			print('Could not understand audio')
			return ''
		except:
			print 'other reason'
			return ''
		print ('whaaat fre')
		sys.exit(-1)
	def setState(self, state):
		self.state = state
		self.ui.stateLabel.setText(self.state)

	def setLastText(self, text):
		self.lastText = text
		print('text: ', self.lastText)
		self.ui.lastTextLabel.setText(str(self.lastText))

	def setLastCommand(self, text):
		self.lastCommand = text
		self.ui.lastCommandLabel.setText(self.lastCommand)



