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
import telepot
import speech_recognition as sr
import unicodedata

from PySide import QtCore, QtGui
from genericworker import *
from compConfig import *

def dateStringFromTimestamp(t):
	return str(time.strftime("%Y%m%d%H%M%S", time.localtime(float(t))))



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		self.timer.timeout.connect(self.compute)
		self.Period = 10000
		self.timer.start(self.Period)


	def setParams(self, params):
		try:
			self.TELEGRAM_API = params["TELEGRAM_API"]
			print 'TELEGRAM_API:', self.TELEGRAM_API
		except:
			traceback.print_exc()
			print "Error reading config param: TELEGRAM_API"
			return False
		try:
			self.TELEGRAM_ID = params["TELEGRAM_ID"]
			print 'TELEGRAM_ID:', self.TELEGRAM_ID
		except:
			traceback.print_exc()
			print "Error reading config param: TELEGRAM_ID"
			return False
		try:
			self.GOOGLE_SPEECH_KEY = params["GOOGLE_SPEECH_KEY"]
			print 'GOOGLE_SPEECH_KEY:', self.GOOGLE_SPEECH_KEY
		except:
			traceback.print_exc()
			print "Error reading config param: GOOGLE_SPEECH_KEY"
			return False
		try:
			self.LANGUAGE = params["LANGUAGE"]
			print 'LANGUAGE:', self.LANGUAGE
		except:
			traceback.print_exc()
			print "Error reading config param: LANGUAGE"
			return False



		self.goSet, self.graspSet, self.bringSet, self.objects = getLanguageParams(self.LANGUAGE)
		print 'goSet'
		print self.goSet
		print 'graspSet'
		print self.graspSet
		print 'bringSet'
		print self.bringSet
		print 'objects'
		print self.objects



		# Telegram stuff
		self.bot = telepot.Bot(self.TELEGRAM_API)
		self.bot.message_loop(self.handle)

		return True


	@QtCore.Slot()
	def compute(self):
		return True

	def handle(self, message):
		print 'handle'
		if 'text' in message.keys():
			self.handle_text(message)
		elif 'voice' in message.keys():
			self.handle_audio(message)
		else:
			print message

	def handle_audio(self, message):
		d = dateStringFromTimestamp(message['date'])
		r = sr.Recognizer()
		file_path = d + '_voice.ogg'
		self.bot.download_file(message['voice']['file_id'], file_path)
		os.system('avconv -y -i ' + file_path + ' audio.wav')
		with sr.WavFile("audio.wav") as source:
			audio = r.record(source)
		try:
			transcription = r.recognize_google(audio, language=self.LANGUAGE, key=self.GOOGLE_SPEECH_KEY)
			self.do(transcription)
		except LookupError:
			print("Could not understand audio")

	def handle_text(self, message):
		self.do(message['text'])

	def remove_weird_chars_and_lower(self, s):
		return ''.join((c for c in unicodedata.normalize('NFD', s.lower()) if unicodedata.category(c) != 'Mn'))

	def do(self, text):
		text = self.remove_weird_chars_and_lower(text)
		print 'do:', text
		output_message = 'RECIBIDO: "'+text+'"\n'

		mission, output_message = self.missionFromKnownSentences(text, output_message)
		if mission == None:
			mission, output_message = self.missionFromAnalysis(text, output_message)

		self.bot.sendMessage(self.TELEGRAM_ID, output_message)

		if mission != None:
			target = '/home/robocomp/robocomp/components/robocomp-shelly/etc/target' + mission + '.aggt'
			self.shellymission_proxy.setMission(target)
			self.bot.sendMessage(self.TELEGRAM_ID, target)



	def missionFromKnownSentences(self, text, output_message):
		if text in [ 'stop', 'para', 'parate', 'no', 'quieta', 'quieto', 'zara' ]:
			output_message += ' (knownSentence) '
			return 'Stop', output_message
		if text in [ 'descansa', 'rest' ]:
			output_message += ' (knownSentence) '
			return 'GoWaitPosition', output_message
		return None, output_message


	def missionFromAnalysis(self, text, output_message):
		mission = None
		wordSet = set(text.split(' '))
		ir = len(wordSet.intersection(self.goSet)) > 0
		print 'IR?:', ir
		coger = len(wordSet.intersection(self.graspSet)) > 0
		print 'COGER?:', coger
		dar = len(wordSet.intersection(self.bringSet)) > 0
		print 'DAR?:', dar
		objects = []
		for o in self.objects:
			if o in text:
				objects.append(self.objects[o])
		print 'OBJECTS:', objects
		output_message += 'VARIABLES: ir("'+str(ir)+') coger('+str(coger)+') dar('+str(dar)+') objetos('+str(objects)+')\n'
		if (ir and not coger and not dar):# and len(objects) == 1:
			if   objects[0] == 'tableA':
				mission = 'ReachTableA'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'tableB':
				mission = 'ReachTableB'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'tableC':
				mission = 'ReachTableC'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'mug':
				mission = 'ReachAMug'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'waitPosition':
				mission = 'GoWaitPosition'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'room3':
				mission = 'GoRoom3'
				output_message += 'MISION: '+mission+'\n'
			elif objects[0] == 'room5':
				mission = 'GoRoom5'
				output_message += 'MISION: '+mission+'\n'
			else:
				output_message += 'No entiendo bien a donde quieres que vaya\n'
		elif (coger and not ir and not dar):# and len(objects) == 1:
			if objects[0] == 'mug':
				mission = 'GraspMug'
				output_message += 'MISION: '+mission+'\n'
			else:
				output_message += 'No entiendo bien que quieres que coja\n'
		elif (dar and not ir and not coger):# and len(objects) == 1:
			if objects[0] == 'mug':
				mission = 'DeliverMug'
				output_message += 'MISION: '+mission+'\n'
			else:
				output_message += 'No entiendo bien que quieres que coja\n'
		else:
			output_message += 'No te entiendo, solo se moverme y coger tazas\n'
		if output_message[-1] == "\n":
			output_message = output_message[:-1]

		return mission, output_message
