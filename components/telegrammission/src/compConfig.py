def getLanguageParams(desiredLanguage):
		if desiredLanguage == 'es-ES':
			goSet    = set(['ve', 'ir', 'vete', 'vente', 'acercate', 'corre', 'muevete'])
			graspSet = set(['coge', 'toma', 'agarra', 'pilla'])
			bringSet = set(['trae', 'traeme', 'dame', 'acercame'])
			objects = {
				   u'mesa a'          : 'tableA',
				   u'mesa uno'        : 'tableA',
				   u'mesa 1'          : 'tableA',
				   u'mesa numero 1'   : 'tableA',
				   u'mesa numero uno' : 'tableA',

				   u'mesa b'          : 'tableB',
				   u'mesa dos'        : 'tableB',
				   u'mesa 2'          : 'tableB',
				   u'mesa numero 2'   : 'tableB',
				   u'mesa numero dos' : 'tableB',

				   u'mesa c'           : 'tableC',
				   u'mesa tres'        : 'tableC',
				   u'mesa 3'           : 'tableC',
				   u'mesa numero 3'    : 'tableC',
				   u'mesa numero tres' : 'tableC',

				   u'habitacion verde'          : 'room3',
				   u'habitacion uno'            : 'room3',
				   u'habitacion 1'              : 'room3',
				   u'habitacion numero 1'       : 'room3',
				   u'habitacion numero uno'     : 'room3',
				   u'habitacion de color verde' : 'room3',

				   u'habitacion naranja'          : 'room5',
				   u'habitacion dos'              : 'room5',
				   u'habitacion 2'                : 'room5',
				   u'habitacion numero 2'         : 'room5',
				   u'habitacion numero dos'       : 'room5',
				   u'habitacion de color naranja' : 'room5',

				   u'taza': 'mug',
				   u'vaso': 'mug',
				   u'copa': 'mug',
				   u'noodles': 'mug',
				   u'fideos': 'mug',

				   u'espera': 'waitPosition',
				   u'descanso': 'waitPosition',
				   u'descansar': 'waitPosition',
				   u'parada': 'waitPosition'
			}
			return (goSet, graspSet, bringSet, objects)
		elif desiredLanguage == 'en-UK' or desiredLanguage == 'en-US':
			goSet = set(['go', 'approach', 'move'])
			graspSet = set(['grasp', 'take', 'get'])
			graspSet = set(['bring'])
			objects = {
				   u'mesa a'          : 'tableA',
				   u'mesa one'        : 'tableA',
				   u'mesa 1'          : 'tableA',
				   u'mesa number 1'   : 'tableA',
				   u'mesa number one' : 'tableA',

				   u'mesa b'          : 'tableB',
				   u'mesa two'        : 'tableB',
				   u'mesa 2'          : 'tableB',
				   u'mesa number 2'   : 'tableB',
				   u'mesa number two' : 'tableB',

				   u'mesa c'            : 'tableC',
				   u'mesa three'        : 'tableC',
				   u'mesa 3'            : 'tableC',
				   u'mesa number 2'     : 'tableC',
				   u'mesa number three' : 'tableC',

				   u'mug'   : 'mug',
				   u'glass' : 'mug',
				   u'cup'   : 'mug',

				   u'wait': 'waitPosition',
				   u'rest': 'waitPosition',
				   u'stop': 'waitPosition'
			}
			return (goSet, graspSet, bringSet, objects)
		else:
			print 'Language "'+desiredLanguage+'" not supported'
			retun (None, None, None)
