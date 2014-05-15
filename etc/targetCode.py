import copy, sys
sys.path.append('/usr/local/share/agm/')
from AGGL import *
from agglplanner import *
def CheckTarget(graph):
	n2id = dict()

	score = 0
	maxScore = 0
	scoreA = 0

	# Easy score
	typesDict = dict()
	typesDict['grasp'] = 1
	typesDict['mug'] = 1
	typesDict['object'] = 1
	typesDict['reach'] = 1
	typesDict['robot'] = 1
	for n in graph.nodes:
		if graph.nodes[n].sType in typesDict:
			scoreA += 1
			typesDict[graph.nodes[n].sType] -= 1
			if typesDict[graph.nodes[n].sType] == 0:
				del typesDict[graph.nodes[n].sType]

	# Hard score
	# 1
	symbol_1_name = '1'
	symbol_1 = graph.nodes[symbol_1_name]
	n2id['1'] = symbol_1_name
	if symbol_1.sType == 'robot':
		score = 100
		if score > maxScore: maxScore = score
		# mug
		for symbol_mug_name in graph.nodes:
			symbol_mug = graph.nodes[symbol_mug_name]
			n2id['mug'] = symbol_mug_name
			if symbol_mug.sType == 'object' and symbol_mug.name!=symbol_1.name and [n2id["1"],n2id["mug"],"know"] in graph.links:
				score = 200
				if score > maxScore: maxScore = score
				# mugGrasp
				for symbol_mugGrasp_name in graph.nodes:
					symbol_mugGrasp = graph.nodes[symbol_mugGrasp_name]
					n2id['mugGrasp'] = symbol_mugGrasp_name
					if symbol_mugGrasp.sType == 'grasp' and symbol_mugGrasp.name!=symbol_1.name and symbol_mugGrasp.name!=symbol_mug.name and [n2id["mug"],n2id["mugGrasp"],"prop"] in graph.links:
						score = 300
						if score > maxScore: maxScore = score
						# mugReach
						for symbol_mugReach_name in graph.nodes:
							symbol_mugReach = graph.nodes[symbol_mugReach_name]
							n2id['mugReach'] = symbol_mugReach_name
							if symbol_mugReach.sType == 'reach' and symbol_mugReach.name!=symbol_1.name and symbol_mugReach.name!=symbol_mug.name and symbol_mugReach.name!=symbol_mugGrasp.name and [n2id["mug"],n2id["mugReach"],"prop"] in graph.links:
								score = 400
								if score > maxScore: maxScore = score
								# mugType
								for symbol_mugType_name in graph.nodes:
									symbol_mugType = graph.nodes[symbol_mugType_name]
									n2id['mugType'] = symbol_mugType_name
									if symbol_mugType.sType == 'mug' and symbol_mugType.name!=symbol_1.name and symbol_mugType.name!=symbol_mug.name and symbol_mugType.name!=symbol_mugGrasp.name and symbol_mugType.name!=symbol_mugReach.name and [n2id["mug"],n2id["mugType"],"prop"] in graph.links:
										score = 500
										if score > maxScore: maxScore = score
										if [n2id["1"],n2id["mug"],"know"] in graph.links : score += 100
										if [n2id["mug"],n2id["mugGrasp"],"prop"] in graph.links : score += 100
										if [n2id["mug"],n2id["mugReach"],"prop"] in graph.links : score += 100
										if [n2id["mug"],n2id["mugType"],"prop"] in graph.links: score += 100
										if score > maxScore: maxScore = score
										if score == 900:
											return score+scoreA, True
	return score+scoreA, False
	
	
