import copy, sys
sys.path.append('/usr/local/share/agm/')
from AGGL import *
from agglplanner import *

def getNewIdForSymbol(node):
	m = 1
	for k in node.graph.nodes:
			if int(node.graph.nodes[k].name) >= m:
				m = int(node.graph.nodes[k].name)+1
	return m

lastNodeId = 0

class RuleSet(object):
	def __init__(self):
		object.__init__(self)

	def getRules(self):
		mapping = dict()
		mapping['findObjectVisually'] = self.findObjectVisually
		mapping['recognizeObjMug'] = self.recognizeObjMug
		mapping['recognizeObjTable'] = self.recognizeObjTable
		mapping['recognizeObjFails'] = self.recognizeObjFails
		mapping['setObjectReach'] = self.setObjectReach
		mapping['graspObject'] = self.graspObject
		mapping['setObjectSee'] = self.setObjectSee
		mapping['setObjectPosition'] = self.setObjectPosition
		mapping['tellHumanAboutMug'] = self.tellHumanAboutMug
		mapping['tellHumanAboutTable'] = self.tellHumanAboutTable
		mapping['tellHumanAboutUnknownObject'] = self.tellHumanAboutUnknownObject
		mapping['makeHumanLook'] = self.makeHumanLook
		mapping['humanClassifiesMug'] = self.humanClassifiesMug
		mapping['humanClassifiesTable'] = self.humanClassifiesTable
		mapping['humanTellsUsAboutMug'] = self.humanTellsUsAboutMug
		mapping['humanTellsUsAboutTable'] = self.humanTellsUsAboutTable
		return mapping

	def getTriggers(self):
		mapping = dict()
		mapping['findObjectVisually'] = self.findObjectVisually_trigger
		mapping['recognizeObjMug'] = self.recognizeObjMug_trigger
		mapping['recognizeObjTable'] = self.recognizeObjTable_trigger
		mapping['recognizeObjFails'] = self.recognizeObjFails_trigger
		mapping['setObjectReach'] = self.setObjectReach_trigger
		mapping['graspObject'] = self.graspObject_trigger
		mapping['setObjectSee'] = self.setObjectSee_trigger
		mapping['setObjectPosition'] = self.setObjectPosition_trigger
		mapping['tellHumanAboutMug'] = self.tellHumanAboutMug_trigger
		mapping['tellHumanAboutTable'] = self.tellHumanAboutTable_trigger
		mapping['tellHumanAboutUnknownObject'] = self.tellHumanAboutUnknownObject_trigger
		mapping['makeHumanLook'] = self.makeHumanLook_trigger
		mapping['humanClassifiesMug'] = self.humanClassifiesMug_trigger
		mapping['humanClassifiesTable'] = self.humanClassifiesTable_trigger
		mapping['humanTellsUsAboutMug'] = self.humanTellsUsAboutMug_trigger
		mapping['humanTellsUsAboutTable'] = self.humanTellsUsAboutTable_trigger
		return mapping


	# Rule findObjectVisually
	def findObjectVisually(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
		else:
			symbol_r_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_r_name in symbol_r_nodes:
			symbol_r = nodes[symbol_r_name]
			n2id['r'] = symbol_r_name
			if symbol_r.sType == 'robot':
				# At this point we meet all the conditions.
				# Insert additional conditions manually here if you want.
				# (beware that the code could be regenerated and you might lose your changes).
				stack2        = copy.deepcopy(stack)
				equivalences2 = copy.deepcopy(equivalences)
				r1 = self.findObjectVisually_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
				c = copy.deepcopy(r1)
				if 'fina' in locals():
					c.history.append(finishesCombo)
				if len(stack2) > 0: c.stop = True
				ret.append(c)
				if len(stack2) > 0:
					# Set symbol for r...
					for equiv in equivalences2:
						if [me, 'r'] in equiv[0]:
							equiv[1] = symbol_r_name
					newNode = WorldStateHistory(r1)
					global lastNodeId
					lastNodeId += 1
					newNode.nodeId = lastNodeId
					newNode.parentId = snode.nodeId
					derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
					if 'fina' in locals():
						for n in derivsx: n.history.append(finishesCombo)
						for n in derivsx: n.history.append(fina)
					ret.extend(derivsx)
		return ret
		
		

	# Rule findObjectVisually
	def findObjectVisually_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot'):
				raise WrongRuleExecution('findObjectVisually_trigger1')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['re'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'noReach')
		newName = str(getNewIdForSymbol(newNode))
		smap['se'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'see')
		newName = str(getNewIdForSymbol(newNode))
		smap['cl'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'noClass')
		newName = str(getNewIdForSymbol(newNode))
		smap['po'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'pose')
		newName = str(getNewIdForSymbol(newNode))
		smap['o'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'object')
		# Retype nodes
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['po'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['re'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['se'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['cl'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['r'], smap['o'], 'know')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('findObjectVisually@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule recognizeObjMug
	def recognizeObjMug(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for other
			symbol_other_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'other'] in equiv[0] and equiv[1] != None:
					symbol_other_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for po
			symbol_po_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'po'] in equiv[0] and equiv[1] != None:
					symbol_po_nodes = [equiv[1]]
			# Find equivalence for se
			symbol_se_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'se'] in equiv[0] and equiv[1] != None:
					symbol_se_nodes = [equiv[1]]
		else:
			symbol_c_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_other_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_po_nodes = symbol_nodes_copy
			symbol_se_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_se_name in symbol_se_nodes:
							symbol_se = nodes[symbol_se_name]
							n2id['se'] = symbol_se_name
							if symbol_se.sType == 'see' and symbol_se.name!=symbol_o.name and symbol_se.name!=symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links:
								for symbol_po_name in symbol_po_nodes:
									symbol_po = nodes[symbol_po_name]
									n2id['po'] = symbol_po_name
									if symbol_po.sType == 'pose' and symbol_po.name!=symbol_o.name and symbol_po.name!=symbol_r.name and symbol_po.name!=symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links:
										for symbol_c_name in symbol_c_nodes:
											symbol_c = nodes[symbol_c_name]
											n2id['c'] = symbol_c_name
											if symbol_c.sType == 'noClass' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_se.name and symbol_c.name!=symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
												for symbol_other_name in symbol_other_nodes:
													symbol_other = nodes[symbol_other_name]
													n2id['other'] = symbol_other_name
													if symbol_other.sType == 'object' and symbol_other.name!=symbol_o.name and symbol_other.name!=symbol_r.name and symbol_other.name!=symbol_se.name and symbol_other.name!=symbol_po.name and symbol_other.name!=symbol_c.name:
														# At this point we meet all the conditions.
														# Insert additional conditions manually here if you want.
														# (beware that the code could be regenerated and you might lose your changes).
														stack2        = copy.deepcopy(stack)
														equivalences2 = copy.deepcopy(equivalences)
														r1 = self.recognizeObjMug_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
														c = copy.deepcopy(r1)
														if 'fina' in locals():
															c.history.append(finishesCombo)
														if len(stack2) > 0: c.stop = True
														ret.append(c)
														if len(stack2) > 0:
															# Set symbol for o...
															for equiv in equivalences2:
																if [me, 'o'] in equiv[0]:
																	equiv[1] = symbol_o_name
															# Set symbol for r...
															for equiv in equivalences2:
																if [me, 'r'] in equiv[0]:
																	equiv[1] = symbol_r_name
															# Set symbol for se...
															for equiv in equivalences2:
																if [me, 'se'] in equiv[0]:
																	equiv[1] = symbol_se_name
															# Set symbol for po...
															for equiv in equivalences2:
																if [me, 'po'] in equiv[0]:
																	equiv[1] = symbol_po_name
															# Set symbol for c...
															for equiv in equivalences2:
																if [me, 'c'] in equiv[0]:
																	equiv[1] = symbol_c_name
															# Set symbol for other...
															for equiv in equivalences2:
																if [me, 'other'] in equiv[0]:
																	equiv[1] = symbol_other_name
															newNode = WorldStateHistory(r1)
															global lastNodeId
															lastNodeId += 1
															newNode.nodeId = lastNodeId
															newNode.parentId = snode.nodeId
															derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
															if 'fina' in locals():
																for n in derivsx: n.history.append(finishesCombo)
																for n in derivsx: n.history.append(fina)
															ret.extend(derivsx)
		return ret
		
		

	# Rule recognizeObjMug
	def recognizeObjMug_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('recognizeObjMug_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjMug_trigger2')
			test_symbol_se = snode.graph.nodes[n2id['se']]
			if not (test_symbol_se.sType == 'see' and test_symbol_se.name!=test_symbol_o.name and test_symbol_se.name!=test_symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjMug_trigger3')
			test_symbol_po = snode.graph.nodes[n2id['po']]
			if not (test_symbol_po.sType == 'pose' and test_symbol_po.name!=test_symbol_o.name and test_symbol_po.name!=test_symbol_r.name and test_symbol_po.name!=test_symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjMug_trigger4')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'noClass' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_se.name and test_symbol_c.name!=test_symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjMug_trigger5')
			test_symbol_other = snode.graph.nodes[n2id['other']]
			if not (test_symbol_other.sType == 'object' and test_symbol_other.name!=test_symbol_o.name and test_symbol_other.name!=test_symbol_r.name and test_symbol_other.name!=test_symbol_se.name and test_symbol_other.name!=test_symbol_po.name and test_symbol_other.name!=test_symbol_c.name):
				raise WrongRuleExecution('recognizeObjMug_trigger6')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['m'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'mug')
		# Retype nodes
		newNode.graph.nodes[n2id['c']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['m'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['other'], 'in')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('recognizeObjMug@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule recognizeObjTable
	def recognizeObjTable(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for se
			symbol_se_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'se'] in equiv[0] and equiv[1] != None:
					symbol_se_nodes = [equiv[1]]
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for po
			symbol_po_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'po'] in equiv[0] and equiv[1] != None:
					symbol_po_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_se_nodes = symbol_nodes_copy
			symbol_c_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_po_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_se_name in symbol_se_nodes:
							symbol_se = nodes[symbol_se_name]
							n2id['se'] = symbol_se_name
							if symbol_se.sType == 'see' and symbol_se.name!=symbol_o.name and symbol_se.name!=symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links:
								for symbol_po_name in symbol_po_nodes:
									symbol_po = nodes[symbol_po_name]
									n2id['po'] = symbol_po_name
									if symbol_po.sType == 'pose' and symbol_po.name!=symbol_o.name and symbol_po.name!=symbol_r.name and symbol_po.name!=symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links:
										for symbol_c_name in symbol_c_nodes:
											symbol_c = nodes[symbol_c_name]
											n2id['c'] = symbol_c_name
											if symbol_c.sType == 'noClass' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_se.name and symbol_c.name!=symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
												# At this point we meet all the conditions.
												# Insert additional conditions manually here if you want.
												# (beware that the code could be regenerated and you might lose your changes).
												stack2        = copy.deepcopy(stack)
												equivalences2 = copy.deepcopy(equivalences)
												r1 = self.recognizeObjTable_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
												c = copy.deepcopy(r1)
												if 'fina' in locals():
													c.history.append(finishesCombo)
												if len(stack2) > 0: c.stop = True
												ret.append(c)
												if len(stack2) > 0:
													# Set symbol for o...
													for equiv in equivalences2:
														if [me, 'o'] in equiv[0]:
															equiv[1] = symbol_o_name
													# Set symbol for r...
													for equiv in equivalences2:
														if [me, 'r'] in equiv[0]:
															equiv[1] = symbol_r_name
													# Set symbol for se...
													for equiv in equivalences2:
														if [me, 'se'] in equiv[0]:
															equiv[1] = symbol_se_name
													# Set symbol for po...
													for equiv in equivalences2:
														if [me, 'po'] in equiv[0]:
															equiv[1] = symbol_po_name
													# Set symbol for c...
													for equiv in equivalences2:
														if [me, 'c'] in equiv[0]:
															equiv[1] = symbol_c_name
													newNode = WorldStateHistory(r1)
													global lastNodeId
													lastNodeId += 1
													newNode.nodeId = lastNodeId
													newNode.parentId = snode.nodeId
													derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
													if 'fina' in locals():
														for n in derivsx: n.history.append(finishesCombo)
														for n in derivsx: n.history.append(fina)
													ret.extend(derivsx)
		return ret
		
		

	# Rule recognizeObjTable
	def recognizeObjTable_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('recognizeObjTable_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjTable_trigger2')
			test_symbol_se = snode.graph.nodes[n2id['se']]
			if not (test_symbol_se.sType == 'see' and test_symbol_se.name!=test_symbol_o.name and test_symbol_se.name!=test_symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjTable_trigger3')
			test_symbol_po = snode.graph.nodes[n2id['po']]
			if not (test_symbol_po.sType == 'pose' and test_symbol_po.name!=test_symbol_o.name and test_symbol_po.name!=test_symbol_r.name and test_symbol_po.name!=test_symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjTable_trigger4')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'noClass' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_se.name and test_symbol_c.name!=test_symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjTable_trigger5')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['t'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'table')
		# Retype nodes
		newNode.graph.nodes[n2id['c']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['t'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('recognizeObjTable@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule recognizeObjFails
	def recognizeObjFails(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for se
			symbol_se_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'se'] in equiv[0] and equiv[1] != None:
					symbol_se_nodes = [equiv[1]]
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for po
			symbol_po_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'po'] in equiv[0] and equiv[1] != None:
					symbol_po_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_se_nodes = symbol_nodes_copy
			symbol_c_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_po_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_se_name in symbol_se_nodes:
							symbol_se = nodes[symbol_se_name]
							n2id['se'] = symbol_se_name
							if symbol_se.sType == 'see' and symbol_se.name!=symbol_o.name and symbol_se.name!=symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links:
								for symbol_po_name in symbol_po_nodes:
									symbol_po = nodes[symbol_po_name]
									n2id['po'] = symbol_po_name
									if symbol_po.sType == 'pose' and symbol_po.name!=symbol_o.name and symbol_po.name!=symbol_r.name and symbol_po.name!=symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links:
										for symbol_c_name in symbol_c_nodes:
											symbol_c = nodes[symbol_c_name]
											n2id['c'] = symbol_c_name
											if symbol_c.sType == 'noClass' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_se.name and symbol_c.name!=symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
												# At this point we meet all the conditions.
												# Insert additional conditions manually here if you want.
												# (beware that the code could be regenerated and you might lose your changes).
												stack2        = copy.deepcopy(stack)
												equivalences2 = copy.deepcopy(equivalences)
												r1 = self.recognizeObjFails_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
												c = copy.deepcopy(r1)
												if 'fina' in locals():
													c.history.append(finishesCombo)
												if len(stack2) > 0: c.stop = True
												ret.append(c)
												if len(stack2) > 0:
													# Set symbol for o...
													for equiv in equivalences2:
														if [me, 'o'] in equiv[0]:
															equiv[1] = symbol_o_name
													# Set symbol for r...
													for equiv in equivalences2:
														if [me, 'r'] in equiv[0]:
															equiv[1] = symbol_r_name
													# Set symbol for se...
													for equiv in equivalences2:
														if [me, 'se'] in equiv[0]:
															equiv[1] = symbol_se_name
													# Set symbol for po...
													for equiv in equivalences2:
														if [me, 'po'] in equiv[0]:
															equiv[1] = symbol_po_name
													# Set symbol for c...
													for equiv in equivalences2:
														if [me, 'c'] in equiv[0]:
															equiv[1] = symbol_c_name
													newNode = WorldStateHistory(r1)
													global lastNodeId
													lastNodeId += 1
													newNode.nodeId = lastNodeId
													newNode.parentId = snode.nodeId
													derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
													if 'fina' in locals():
														for n in derivsx: n.history.append(finishesCombo)
														for n in derivsx: n.history.append(fina)
													ret.extend(derivsx)
		return ret
		
		

	# Rule recognizeObjFails
	def recognizeObjFails_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('recognizeObjFails_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjFails_trigger2')
			test_symbol_se = snode.graph.nodes[n2id['se']]
			if not (test_symbol_se.sType == 'see' and test_symbol_se.name!=test_symbol_o.name and test_symbol_se.name!=test_symbol_r.name and [n2id["o"],n2id["se"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjFails_trigger3')
			test_symbol_po = snode.graph.nodes[n2id['po']]
			if not (test_symbol_po.sType == 'pose' and test_symbol_po.name!=test_symbol_o.name and test_symbol_po.name!=test_symbol_r.name and test_symbol_po.name!=test_symbol_se.name and [n2id["o"],n2id["po"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjFails_trigger4')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'noClass' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_se.name and test_symbol_c.name!=test_symbol_po.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('recognizeObjFails_trigger5')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		# Retype nodes
		newNode.graph.nodes[n2id['c']].sType = 'classFail'
		# Remove nodes
		# Remove links
		# Create links
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('recognizeObjFails@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule setObjectReach
	def setObjectReach(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'noReach' and symbol_r.name!=symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links:
						# At this point we meet all the conditions.
						# Insert additional conditions manually here if you want.
						# (beware that the code could be regenerated and you might lose your changes).
						stack2        = copy.deepcopy(stack)
						equivalences2 = copy.deepcopy(equivalences)
						r1 = self.setObjectReach_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
						c = copy.deepcopy(r1)
						if 'fina' in locals():
							c.history.append(finishesCombo)
						if len(stack2) > 0: c.stop = True
						ret.append(c)
						if len(stack2) > 0:
							# Set symbol for o...
							for equiv in equivalences2:
								if [me, 'o'] in equiv[0]:
									equiv[1] = symbol_o_name
							# Set symbol for r...
							for equiv in equivalences2:
								if [me, 'r'] in equiv[0]:
									equiv[1] = symbol_r_name
							newNode = WorldStateHistory(r1)
							global lastNodeId
							lastNodeId += 1
							newNode.nodeId = lastNodeId
							newNode.parentId = snode.nodeId
							derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
							if 'fina' in locals():
								for n in derivsx: n.history.append(finishesCombo)
								for n in derivsx: n.history.append(fina)
							ret.extend(derivsx)
		return ret
		
		

	# Rule setObjectReach
	def setObjectReach_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('setObjectReach_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'noReach' and test_symbol_r.name!=test_symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('setObjectReach_trigger2')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		# Retype nodes
		newNode.graph.nodes[n2id['r']].sType = 'reach'
		# Remove nodes
		# Remove links
		# Create links
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('setObjectReach@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule graspObject
	def graspObject(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'reach' and symbol_r.name!=symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links:
						# At this point we meet all the conditions.
						# Insert additional conditions manually here if you want.
						# (beware that the code could be regenerated and you might lose your changes).
						stack2        = copy.deepcopy(stack)
						equivalences2 = copy.deepcopy(equivalences)
						r1 = self.graspObject_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
						c = copy.deepcopy(r1)
						if 'fina' in locals():
							c.history.append(finishesCombo)
						if len(stack2) > 0: c.stop = True
						ret.append(c)
						if len(stack2) > 0:
							# Set symbol for o...
							for equiv in equivalences2:
								if [me, 'o'] in equiv[0]:
									equiv[1] = symbol_o_name
							# Set symbol for r...
							for equiv in equivalences2:
								if [me, 'r'] in equiv[0]:
									equiv[1] = symbol_r_name
							newNode = WorldStateHistory(r1)
							global lastNodeId
							lastNodeId += 1
							newNode.nodeId = lastNodeId
							newNode.parentId = snode.nodeId
							derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
							if 'fina' in locals():
								for n in derivsx: n.history.append(finishesCombo)
								for n in derivsx: n.history.append(fina)
							ret.extend(derivsx)
		return ret
		
		

	# Rule graspObject
	def graspObject_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('graspObject_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'reach' and test_symbol_r.name!=test_symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('graspObject_trigger2')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['g'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'grasp')
		# Retype nodes
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['g'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('graspObject@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule setObjectSee
	def setObjectSee(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'noSee' and symbol_r.name!=symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links:
						# At this point we meet all the conditions.
						# Insert additional conditions manually here if you want.
						# (beware that the code could be regenerated and you might lose your changes).
						stack2        = copy.deepcopy(stack)
						equivalences2 = copy.deepcopy(equivalences)
						r1 = self.setObjectSee_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
						c = copy.deepcopy(r1)
						if 'fina' in locals():
							c.history.append(finishesCombo)
						if len(stack2) > 0: c.stop = True
						ret.append(c)
						if len(stack2) > 0:
							# Set symbol for o...
							for equiv in equivalences2:
								if [me, 'o'] in equiv[0]:
									equiv[1] = symbol_o_name
							# Set symbol for r...
							for equiv in equivalences2:
								if [me, 'r'] in equiv[0]:
									equiv[1] = symbol_r_name
							newNode = WorldStateHistory(r1)
							global lastNodeId
							lastNodeId += 1
							newNode.nodeId = lastNodeId
							newNode.parentId = snode.nodeId
							derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
							if 'fina' in locals():
								for n in derivsx: n.history.append(finishesCombo)
								for n in derivsx: n.history.append(fina)
							ret.extend(derivsx)
		return ret
		
		

	# Rule setObjectSee
	def setObjectSee_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('setObjectSee_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'noSee' and test_symbol_r.name!=test_symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('setObjectSee_trigger2')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		# Retype nodes
		newNode.graph.nodes[n2id['r']].sType = 'see'
		# Remove nodes
		# Remove links
		# Create links
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('setObjectSee@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule setObjectPosition
	def setObjectPosition(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
		else:
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'noPose' and symbol_r.name!=symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links:
						# At this point we meet all the conditions.
						# Insert additional conditions manually here if you want.
						# (beware that the code could be regenerated and you might lose your changes).
						stack2        = copy.deepcopy(stack)
						equivalences2 = copy.deepcopy(equivalences)
						r1 = self.setObjectPosition_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
						c = copy.deepcopy(r1)
						if 'fina' in locals():
							c.history.append(finishesCombo)
						if len(stack2) > 0: c.stop = True
						ret.append(c)
						if len(stack2) > 0:
							# Set symbol for o...
							for equiv in equivalences2:
								if [me, 'o'] in equiv[0]:
									equiv[1] = symbol_o_name
							# Set symbol for r...
							for equiv in equivalences2:
								if [me, 'r'] in equiv[0]:
									equiv[1] = symbol_r_name
							newNode = WorldStateHistory(r1)
							global lastNodeId
							lastNodeId += 1
							newNode.nodeId = lastNodeId
							newNode.parentId = snode.nodeId
							derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
							if 'fina' in locals():
								for n in derivsx: n.history.append(finishesCombo)
								for n in derivsx: n.history.append(fina)
							ret.extend(derivsx)
		return ret
		
		

	# Rule setObjectPosition
	def setObjectPosition_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('setObjectPosition_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'noPose' and test_symbol_r.name!=test_symbol_o.name and [n2id["o"],n2id["r"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('setObjectPosition_trigger2')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		# Retype nodes
		newNode.graph.nodes[n2id['r']].sType = 'pose'
		# Remove nodes
		# Remove links
		# Create links
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('setObjectPosition@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule tellHumanAboutMug
	def tellHumanAboutMug(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for m
			symbol_m_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'm'] in equiv[0] and equiv[1] != None:
					symbol_m_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for cont1
			symbol_cont1_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cont1'] in equiv[0] and equiv[1] != None:
					symbol_cont1_nodes = [equiv[1]]
			# Find equivalence for cont2
			symbol_cont2_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cont2'] in equiv[0] and equiv[1] != None:
					symbol_cont2_nodes = [equiv[1]]
		else:
			symbol_c_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_m_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_p_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_cont1_nodes = symbol_nodes_copy
			symbol_cont2_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_cont1_name in symbol_cont1_nodes:
							symbol_cont1 = nodes[symbol_cont1_name]
							n2id['cont1'] = symbol_cont1_name
							if symbol_cont1.sType == 'object' and symbol_cont1.name!=symbol_o.name and symbol_cont1.name!=symbol_r.name and [n2id["o"],n2id["cont1"],"in"] in snode.graph.links:
								for symbol_p_name in symbol_p_nodes:
									symbol_p = nodes[symbol_p_name]
									n2id['p'] = symbol_p_name
									if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_r.name and symbol_p.name!=symbol_cont1.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links:
										for symbol_m_name in symbol_m_nodes:
											symbol_m = nodes[symbol_m_name]
											n2id['m'] = symbol_m_name
											if symbol_m.sType == 'mug' and symbol_m.name!=symbol_o.name and symbol_m.name!=symbol_r.name and symbol_m.name!=symbol_cont1.name and symbol_m.name!=symbol_p.name and [n2id["o"],n2id["m"],"prop"] in snode.graph.links:
												for symbol_h_name in symbol_h_nodes:
													symbol_h = nodes[symbol_h_name]
													n2id['h'] = symbol_h_name
													if symbol_h.sType == 'human' and symbol_h.name!=symbol_o.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_cont1.name and symbol_h.name!=symbol_p.name and symbol_h.name!=symbol_m.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links:
														for symbol_cont2_name in symbol_cont2_nodes:
															symbol_cont2 = nodes[symbol_cont2_name]
															n2id['cont2'] = symbol_cont2_name
															if symbol_cont2.sType == 'object' and symbol_cont2.name!=symbol_o.name and symbol_cont2.name!=symbol_r.name and symbol_cont2.name!=symbol_cont1.name and symbol_cont2.name!=symbol_p.name and symbol_cont2.name!=symbol_m.name and symbol_cont2.name!=symbol_h.name and [n2id["cont1"],n2id["cont2"],"eq"] in snode.graph.links:
																for symbol_c_name in symbol_c_nodes:
																	symbol_c = nodes[symbol_c_name]
																	n2id['c'] = symbol_c_name
																	if symbol_c.sType == 'class' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_cont1.name and symbol_c.name!=symbol_p.name and symbol_c.name!=symbol_m.name and symbol_c.name!=symbol_h.name and symbol_c.name!=symbol_cont2.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
																		# At this point we meet all the conditions.
																		# Insert additional conditions manually here if you want.
																		# (beware that the code could be regenerated and you might lose your changes).
																		stack2        = copy.deepcopy(stack)
																		equivalences2 = copy.deepcopy(equivalences)
																		r1 = self.tellHumanAboutMug_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
																		c = copy.deepcopy(r1)
																		if 'fina' in locals():
																			c.history.append(finishesCombo)
																		if len(stack2) > 0: c.stop = True
																		ret.append(c)
																		if len(stack2) > 0:
																			# Set symbol for o...
																			for equiv in equivalences2:
																				if [me, 'o'] in equiv[0]:
																					equiv[1] = symbol_o_name
																			# Set symbol for r...
																			for equiv in equivalences2:
																				if [me, 'r'] in equiv[0]:
																					equiv[1] = symbol_r_name
																			# Set symbol for cont1...
																			for equiv in equivalences2:
																				if [me, 'cont1'] in equiv[0]:
																					equiv[1] = symbol_cont1_name
																			# Set symbol for p...
																			for equiv in equivalences2:
																				if [me, 'p'] in equiv[0]:
																					equiv[1] = symbol_p_name
																			# Set symbol for m...
																			for equiv in equivalences2:
																				if [me, 'm'] in equiv[0]:
																					equiv[1] = symbol_m_name
																			# Set symbol for h...
																			for equiv in equivalences2:
																				if [me, 'h'] in equiv[0]:
																					equiv[1] = symbol_h_name
																			# Set symbol for cont2...
																			for equiv in equivalences2:
																				if [me, 'cont2'] in equiv[0]:
																					equiv[1] = symbol_cont2_name
																			# Set symbol for c...
																			for equiv in equivalences2:
																				if [me, 'c'] in equiv[0]:
																					equiv[1] = symbol_c_name
																			newNode = WorldStateHistory(r1)
																			global lastNodeId
																			lastNodeId += 1
																			newNode.nodeId = lastNodeId
																			newNode.parentId = snode.nodeId
																			derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
																			if 'fina' in locals():
																				for n in derivsx: n.history.append(finishesCombo)
																				for n in derivsx: n.history.append(fina)
																			ret.extend(derivsx)
		return ret
		
		

	# Rule tellHumanAboutMug
	def tellHumanAboutMug_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('tellHumanAboutMug_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger2')
			test_symbol_cont1 = snode.graph.nodes[n2id['cont1']]
			if not (test_symbol_cont1.sType == 'object' and test_symbol_cont1.name!=test_symbol_o.name and test_symbol_cont1.name!=test_symbol_r.name and [n2id["o"],n2id["cont1"],"in"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger3')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_r.name and test_symbol_p.name!=test_symbol_cont1.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger4')
			test_symbol_m = snode.graph.nodes[n2id['m']]
			if not (test_symbol_m.sType == 'mug' and test_symbol_m.name!=test_symbol_o.name and test_symbol_m.name!=test_symbol_r.name and test_symbol_m.name!=test_symbol_cont1.name and test_symbol_m.name!=test_symbol_p.name and [n2id["o"],n2id["m"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger5')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_cont1.name and test_symbol_h.name!=test_symbol_p.name and test_symbol_h.name!=test_symbol_m.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger6')
			test_symbol_cont2 = snode.graph.nodes[n2id['cont2']]
			if not (test_symbol_cont2.sType == 'object' and test_symbol_cont2.name!=test_symbol_o.name and test_symbol_cont2.name!=test_symbol_r.name and test_symbol_cont2.name!=test_symbol_cont1.name and test_symbol_cont2.name!=test_symbol_p.name and test_symbol_cont2.name!=test_symbol_m.name and test_symbol_cont2.name!=test_symbol_h.name and [n2id["cont1"],n2id["cont2"],"eq"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger7')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'class' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_cont1.name and test_symbol_c.name!=test_symbol_p.name and test_symbol_c.name!=test_symbol_m.name and test_symbol_c.name!=test_symbol_h.name and test_symbol_c.name!=test_symbol_cont2.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutMug_trigger8')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['ch'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'class')
		newName = str(getNewIdForSymbol(newNode))
		smap['oh'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'object')
		newName = str(getNewIdForSymbol(newNode))
		smap['hr'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'reach')
		newName = str(getNewIdForSymbol(newNode))
		smap['mh'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'mug')
		newName = str(getNewIdForSymbol(newNode))
		smap['hs'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'see')
		newName = str(getNewIdForSymbol(newNode))
		smap['ph'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'pose')
		# Retype nodes
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['h'], smap['oh'], 'know')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ch'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ph'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['mh'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['oh'], 'eq')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['cont2'], 'in')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hr'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hs'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('tellHumanAboutMug@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule tellHumanAboutTable
	def tellHumanAboutTable(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for m
			symbol_m_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'm'] in equiv[0] and equiv[1] != None:
					symbol_m_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
		else:
			symbol_c_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_m_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_p_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_p_name in symbol_p_nodes:
							symbol_p = nodes[symbol_p_name]
							n2id['p'] = symbol_p_name
							if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_r.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links:
								for symbol_m_name in symbol_m_nodes:
									symbol_m = nodes[symbol_m_name]
									n2id['m'] = symbol_m_name
									if symbol_m.sType == 'table' and symbol_m.name!=symbol_o.name and symbol_m.name!=symbol_r.name and symbol_m.name!=symbol_p.name and [n2id["o"],n2id["m"],"prop"] in snode.graph.links:
										for symbol_h_name in symbol_h_nodes:
											symbol_h = nodes[symbol_h_name]
											n2id['h'] = symbol_h_name
											if symbol_h.sType == 'human' and symbol_h.name!=symbol_o.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_p.name and symbol_h.name!=symbol_m.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links:
												for symbol_c_name in symbol_c_nodes:
													symbol_c = nodes[symbol_c_name]
													n2id['c'] = symbol_c_name
													if symbol_c.sType == 'class' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_p.name and symbol_c.name!=symbol_m.name and symbol_c.name!=symbol_h.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
														# At this point we meet all the conditions.
														# Insert additional conditions manually here if you want.
														# (beware that the code could be regenerated and you might lose your changes).
														stack2        = copy.deepcopy(stack)
														equivalences2 = copy.deepcopy(equivalences)
														r1 = self.tellHumanAboutTable_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
														c = copy.deepcopy(r1)
														if 'fina' in locals():
															c.history.append(finishesCombo)
														if len(stack2) > 0: c.stop = True
														ret.append(c)
														if len(stack2) > 0:
															# Set symbol for o...
															for equiv in equivalences2:
																if [me, 'o'] in equiv[0]:
																	equiv[1] = symbol_o_name
															# Set symbol for r...
															for equiv in equivalences2:
																if [me, 'r'] in equiv[0]:
																	equiv[1] = symbol_r_name
															# Set symbol for p...
															for equiv in equivalences2:
																if [me, 'p'] in equiv[0]:
																	equiv[1] = symbol_p_name
															# Set symbol for m...
															for equiv in equivalences2:
																if [me, 'm'] in equiv[0]:
																	equiv[1] = symbol_m_name
															# Set symbol for h...
															for equiv in equivalences2:
																if [me, 'h'] in equiv[0]:
																	equiv[1] = symbol_h_name
															# Set symbol for c...
															for equiv in equivalences2:
																if [me, 'c'] in equiv[0]:
																	equiv[1] = symbol_c_name
															newNode = WorldStateHistory(r1)
															global lastNodeId
															lastNodeId += 1
															newNode.nodeId = lastNodeId
															newNode.parentId = snode.nodeId
															derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
															if 'fina' in locals():
																for n in derivsx: n.history.append(finishesCombo)
																for n in derivsx: n.history.append(fina)
															ret.extend(derivsx)
		return ret
		
		

	# Rule tellHumanAboutTable
	def tellHumanAboutTable_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('tellHumanAboutTable_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutTable_trigger2')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_r.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutTable_trigger3')
			test_symbol_m = snode.graph.nodes[n2id['m']]
			if not (test_symbol_m.sType == 'table' and test_symbol_m.name!=test_symbol_o.name and test_symbol_m.name!=test_symbol_r.name and test_symbol_m.name!=test_symbol_p.name and [n2id["o"],n2id["m"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutTable_trigger4')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_p.name and test_symbol_h.name!=test_symbol_m.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutTable_trigger5')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'class' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_p.name and test_symbol_c.name!=test_symbol_m.name and test_symbol_c.name!=test_symbol_h.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutTable_trigger6')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['ch'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'class')
		newName = str(getNewIdForSymbol(newNode))
		smap['oh'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'object')
		newName = str(getNewIdForSymbol(newNode))
		smap['hr'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'reach')
		newName = str(getNewIdForSymbol(newNode))
		smap['mh'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'mug')
		newName = str(getNewIdForSymbol(newNode))
		smap['hs'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'see')
		newName = str(getNewIdForSymbol(newNode))
		smap['ph'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'pose')
		# Retype nodes
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['h'], smap['oh'], 'know')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ch'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ph'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['mh'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['oh'], 'eq')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hr'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hs'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 1
			newNode.depth += 1
		newNode.history.append('tellHumanAboutTable@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule tellHumanAboutUnknownObject
	def tellHumanAboutUnknownObject(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for c
			symbol_c_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'c'] in equiv[0] and equiv[1] != None:
					symbol_c_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
		else:
			symbol_p_nodes = symbol_nodes_copy
			symbol_c_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_name in symbol_o_nodes:
			symbol_o = nodes[symbol_o_name]
			n2id['o'] = symbol_o_name
			if symbol_o.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
						for symbol_p_name in symbol_p_nodes:
							symbol_p = nodes[symbol_p_name]
							n2id['p'] = symbol_p_name
							if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_r.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_p.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links:
										for symbol_c_name in symbol_c_nodes:
											symbol_c = nodes[symbol_c_name]
											n2id['c'] = symbol_c_name
											if symbol_c.sType == 'classFail' and symbol_c.name!=symbol_o.name and symbol_c.name!=symbol_r.name and symbol_c.name!=symbol_p.name and symbol_c.name!=symbol_h.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links:
												# At this point we meet all the conditions.
												# Insert additional conditions manually here if you want.
												# (beware that the code could be regenerated and you might lose your changes).
												stack2        = copy.deepcopy(stack)
												equivalences2 = copy.deepcopy(equivalences)
												r1 = self.tellHumanAboutUnknownObject_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
												c = copy.deepcopy(r1)
												if 'fina' in locals():
													c.history.append(finishesCombo)
												if len(stack2) > 0: c.stop = True
												ret.append(c)
												if len(stack2) > 0:
													# Set symbol for o...
													for equiv in equivalences2:
														if [me, 'o'] in equiv[0]:
															equiv[1] = symbol_o_name
													# Set symbol for r...
													for equiv in equivalences2:
														if [me, 'r'] in equiv[0]:
															equiv[1] = symbol_r_name
													# Set symbol for p...
													for equiv in equivalences2:
														if [me, 'p'] in equiv[0]:
															equiv[1] = symbol_p_name
													# Set symbol for h...
													for equiv in equivalences2:
														if [me, 'h'] in equiv[0]:
															equiv[1] = symbol_h_name
													# Set symbol for c...
													for equiv in equivalences2:
														if [me, 'c'] in equiv[0]:
															equiv[1] = symbol_c_name
													newNode = WorldStateHistory(r1)
													global lastNodeId
													lastNodeId += 1
													newNode.nodeId = lastNodeId
													newNode.parentId = snode.nodeId
													derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
													if 'fina' in locals():
														for n in derivsx: n.history.append(finishesCombo)
														for n in derivsx: n.history.append(fina)
													ret.extend(derivsx)
		return ret
		
		

	# Rule tellHumanAboutUnknownObject
	def tellHumanAboutUnknownObject_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object'):
				raise WrongRuleExecution('tellHumanAboutUnknownObject_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutUnknownObject_trigger2')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_r.name and [n2id["o"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutUnknownObject_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_p.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutUnknownObject_trigger4')
			test_symbol_c = snode.graph.nodes[n2id['c']]
			if not (test_symbol_c.sType == 'classFail' and test_symbol_c.name!=test_symbol_o.name and test_symbol_c.name!=test_symbol_r.name and test_symbol_c.name!=test_symbol_p.name and test_symbol_c.name!=test_symbol_h.name and [n2id["o"],n2id["c"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('tellHumanAboutUnknownObject_trigger5')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['ch'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'noClass')
		newName = str(getNewIdForSymbol(newNode))
		smap['oh'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'object')
		newName = str(getNewIdForSymbol(newNode))
		smap['hr'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'reach')
		newName = str(getNewIdForSymbol(newNode))
		smap['hs'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'see')
		newName = str(getNewIdForSymbol(newNode))
		smap['ph'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'pose')
		# Retype nodes
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['h'], smap['oh'], 'know')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ch'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['ph'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['oh'], 'eq')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hr'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['oh'], smap['hs'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 3
			newNode.depth += 1
		newNode.history.append('tellHumanAboutUnknownObject@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule makeHumanLook
	def makeHumanLook(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for nS
			symbol_nS_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'nS'] in equiv[0] and equiv[1] != None:
					symbol_nS_nodes = [equiv[1]]
			# Find equivalence for o_h
			symbol_o_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o_h'] in equiv[0] and equiv[1] != None:
					symbol_o_h_nodes = [equiv[1]]
		else:
			symbol_h_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_p_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_nS_nodes = symbol_nodes_copy
			symbol_o_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_h_name in symbol_o_h_nodes:
			symbol_o_h = nodes[symbol_o_h_name]
			n2id['o_h'] = symbol_o_h_name
			if symbol_o_h.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o_h.name:
						for symbol_o_name in symbol_o_nodes:
							symbol_o = nodes[symbol_o_name]
							n2id['o'] = symbol_o_name
							if symbol_o.sType == 'object' and symbol_o.name!=symbol_o_h.name and symbol_o.name!=symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o_h.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links:
										for symbol_p_name in symbol_p_nodes:
											symbol_p = nodes[symbol_p_name]
											n2id['p'] = symbol_p_name
											if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o_h.name and symbol_p.name!=symbol_r.name and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_h.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links:
												for symbol_nS_name in symbol_nS_nodes:
													symbol_nS = nodes[symbol_nS_name]
													n2id['nS'] = symbol_nS_name
													if symbol_nS.sType == 'noSee' and symbol_nS.name!=symbol_o_h.name and symbol_nS.name!=symbol_r.name and symbol_nS.name!=symbol_o.name and symbol_nS.name!=symbol_h.name and symbol_nS.name!=symbol_p.name and [n2id["o_h"],n2id["nS"],"prop"] in snode.graph.links:
														# At this point we meet all the conditions.
														# Insert additional conditions manually here if you want.
														# (beware that the code could be regenerated and you might lose your changes).
														stack2        = copy.deepcopy(stack)
														equivalences2 = copy.deepcopy(equivalences)
														r1 = self.makeHumanLook_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
														c = copy.deepcopy(r1)
														if 'fina' in locals():
															c.history.append(finishesCombo)
														if len(stack2) > 0: c.stop = True
														ret.append(c)
														if len(stack2) > 0:
															# Set symbol for o_h...
															for equiv in equivalences2:
																if [me, 'o_h'] in equiv[0]:
																	equiv[1] = symbol_o_h_name
															# Set symbol for r...
															for equiv in equivalences2:
																if [me, 'r'] in equiv[0]:
																	equiv[1] = symbol_r_name
															# Set symbol for o...
															for equiv in equivalences2:
																if [me, 'o'] in equiv[0]:
																	equiv[1] = symbol_o_name
															# Set symbol for h...
															for equiv in equivalences2:
																if [me, 'h'] in equiv[0]:
																	equiv[1] = symbol_h_name
															# Set symbol for p...
															for equiv in equivalences2:
																if [me, 'p'] in equiv[0]:
																	equiv[1] = symbol_p_name
															# Set symbol for nS...
															for equiv in equivalences2:
																if [me, 'nS'] in equiv[0]:
																	equiv[1] = symbol_nS_name
															newNode = WorldStateHistory(r1)
															global lastNodeId
															lastNodeId += 1
															newNode.nodeId = lastNodeId
															newNode.parentId = snode.nodeId
															derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
															if 'fina' in locals():
																for n in derivsx: n.history.append(finishesCombo)
																for n in derivsx: n.history.append(fina)
															ret.extend(derivsx)
		return ret
		
		

	# Rule makeHumanLook
	def makeHumanLook_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o_h = snode.graph.nodes[n2id['o_h']]
			if not (test_symbol_o_h.sType == 'object'):
				raise WrongRuleExecution('makeHumanLook_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o_h.name):
				raise WrongRuleExecution('makeHumanLook_trigger2')
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object' and test_symbol_o.name!=test_symbol_o_h.name and test_symbol_o.name!=test_symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('makeHumanLook_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o_h.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links):
				raise WrongRuleExecution('makeHumanLook_trigger4')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o_h.name and test_symbol_p.name!=test_symbol_r.name and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_h.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('makeHumanLook_trigger5')
			test_symbol_nS = snode.graph.nodes[n2id['nS']]
			if not (test_symbol_nS.sType == 'noSee' and test_symbol_nS.name!=test_symbol_o_h.name and test_symbol_nS.name!=test_symbol_r.name and test_symbol_nS.name!=test_symbol_o.name and test_symbol_nS.name!=test_symbol_h.name and test_symbol_nS.name!=test_symbol_p.name and [n2id["o_h"],n2id["nS"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('makeHumanLook_trigger6')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		# Retype nodes
		newNode.graph.nodes[n2id['nS']].sType = 'see'
		# Remove nodes
		# Remove links
		# Create links
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 2
			newNode.depth += 1
		newNode.history.append('makeHumanLook@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule humanClassifiesMug
	def humanClassifiesMug(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for cl
			symbol_cl_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cl'] in equiv[0] and equiv[1] != None:
					symbol_cl_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for see
			symbol_see_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'see'] in equiv[0] and equiv[1] != None:
					symbol_see_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for cont2
			symbol_cont2_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cont2'] in equiv[0] and equiv[1] != None:
					symbol_cont2_nodes = [equiv[1]]
			# Find equivalence for o_h
			symbol_o_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o_h'] in equiv[0] and equiv[1] != None:
					symbol_o_h_nodes = [equiv[1]]
		else:
			symbol_cl_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_p_nodes = symbol_nodes_copy
			symbol_see_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_cont2_nodes = symbol_nodes_copy
			symbol_o_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_h_name in symbol_o_h_nodes:
			symbol_o_h = nodes[symbol_o_h_name]
			n2id['o_h'] = symbol_o_h_name
			if symbol_o_h.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o_h.name:
						for symbol_o_name in symbol_o_nodes:
							symbol_o = nodes[symbol_o_name]
							n2id['o'] = symbol_o_name
							if symbol_o.sType == 'object' and symbol_o.name!=symbol_o_h.name and symbol_o.name!=symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o_h.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links:
										for symbol_see_name in symbol_see_nodes:
											symbol_see = nodes[symbol_see_name]
											n2id['see'] = symbol_see_name
											if symbol_see.sType == 'see' and symbol_see.name!=symbol_o_h.name and symbol_see.name!=symbol_r.name and symbol_see.name!=symbol_o.name and symbol_see.name!=symbol_h.name and [n2id["o_h"],n2id["see"],"prop"] in snode.graph.links:
												for symbol_p_name in symbol_p_nodes:
													symbol_p = nodes[symbol_p_name]
													n2id['p'] = symbol_p_name
													if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o_h.name and symbol_p.name!=symbol_r.name and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_h.name and symbol_p.name!=symbol_see.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links:
														for symbol_cl_name in symbol_cl_nodes:
															symbol_cl = nodes[symbol_cl_name]
															n2id['cl'] = symbol_cl_name
															if symbol_cl.sType == 'noClass' and symbol_cl.name!=symbol_o_h.name and symbol_cl.name!=symbol_r.name and symbol_cl.name!=symbol_o.name and symbol_cl.name!=symbol_h.name and symbol_cl.name!=symbol_see.name and symbol_cl.name!=symbol_p.name and [n2id["o_h"],n2id["cl"],"prop"] in snode.graph.links:
																for symbol_cont2_name in symbol_cont2_nodes:
																	symbol_cont2 = nodes[symbol_cont2_name]
																	n2id['cont2'] = symbol_cont2_name
																	if symbol_cont2.sType == 'object' and symbol_cont2.name!=symbol_o_h.name and symbol_cont2.name!=symbol_r.name and symbol_cont2.name!=symbol_o.name and symbol_cont2.name!=symbol_h.name and symbol_cont2.name!=symbol_see.name and symbol_cont2.name!=symbol_p.name and symbol_cont2.name!=symbol_cl.name:
																		# At this point we meet all the conditions.
																		# Insert additional conditions manually here if you want.
																		# (beware that the code could be regenerated and you might lose your changes).
																		stack2        = copy.deepcopy(stack)
																		equivalences2 = copy.deepcopy(equivalences)
																		r1 = self.humanClassifiesMug_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
																		c = copy.deepcopy(r1)
																		if 'fina' in locals():
																			c.history.append(finishesCombo)
																		if len(stack2) > 0: c.stop = True
																		ret.append(c)
																		if len(stack2) > 0:
																			# Set symbol for o_h...
																			for equiv in equivalences2:
																				if [me, 'o_h'] in equiv[0]:
																					equiv[1] = symbol_o_h_name
																			# Set symbol for r...
																			for equiv in equivalences2:
																				if [me, 'r'] in equiv[0]:
																					equiv[1] = symbol_r_name
																			# Set symbol for o...
																			for equiv in equivalences2:
																				if [me, 'o'] in equiv[0]:
																					equiv[1] = symbol_o_name
																			# Set symbol for h...
																			for equiv in equivalences2:
																				if [me, 'h'] in equiv[0]:
																					equiv[1] = symbol_h_name
																			# Set symbol for see...
																			for equiv in equivalences2:
																				if [me, 'see'] in equiv[0]:
																					equiv[1] = symbol_see_name
																			# Set symbol for p...
																			for equiv in equivalences2:
																				if [me, 'p'] in equiv[0]:
																					equiv[1] = symbol_p_name
																			# Set symbol for cl...
																			for equiv in equivalences2:
																				if [me, 'cl'] in equiv[0]:
																					equiv[1] = symbol_cl_name
																			# Set symbol for cont2...
																			for equiv in equivalences2:
																				if [me, 'cont2'] in equiv[0]:
																					equiv[1] = symbol_cont2_name
																			newNode = WorldStateHistory(r1)
																			global lastNodeId
																			lastNodeId += 1
																			newNode.nodeId = lastNodeId
																			newNode.parentId = snode.nodeId
																			derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
																			if 'fina' in locals():
																				for n in derivsx: n.history.append(finishesCombo)
																				for n in derivsx: n.history.append(fina)
																			ret.extend(derivsx)
		return ret
		
		

	# Rule humanClassifiesMug
	def humanClassifiesMug_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o_h = snode.graph.nodes[n2id['o_h']]
			if not (test_symbol_o_h.sType == 'object'):
				raise WrongRuleExecution('humanClassifiesMug_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o_h.name):
				raise WrongRuleExecution('humanClassifiesMug_trigger2')
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object' and test_symbol_o.name!=test_symbol_o_h.name and test_symbol_o.name!=test_symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesMug_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o_h.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesMug_trigger4')
			test_symbol_see = snode.graph.nodes[n2id['see']]
			if not (test_symbol_see.sType == 'see' and test_symbol_see.name!=test_symbol_o_h.name and test_symbol_see.name!=test_symbol_r.name and test_symbol_see.name!=test_symbol_o.name and test_symbol_see.name!=test_symbol_h.name and [n2id["o_h"],n2id["see"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesMug_trigger5')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o_h.name and test_symbol_p.name!=test_symbol_r.name and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_h.name and test_symbol_p.name!=test_symbol_see.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesMug_trigger6')
			test_symbol_cl = snode.graph.nodes[n2id['cl']]
			if not (test_symbol_cl.sType == 'noClass' and test_symbol_cl.name!=test_symbol_o_h.name and test_symbol_cl.name!=test_symbol_r.name and test_symbol_cl.name!=test_symbol_o.name and test_symbol_cl.name!=test_symbol_h.name and test_symbol_cl.name!=test_symbol_see.name and test_symbol_cl.name!=test_symbol_p.name and [n2id["o_h"],n2id["cl"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesMug_trigger7')
			test_symbol_cont2 = snode.graph.nodes[n2id['cont2']]
			if not (test_symbol_cont2.sType == 'object' and test_symbol_cont2.name!=test_symbol_o_h.name and test_symbol_cont2.name!=test_symbol_r.name and test_symbol_cont2.name!=test_symbol_o.name and test_symbol_cont2.name!=test_symbol_h.name and test_symbol_cont2.name!=test_symbol_see.name and test_symbol_cont2.name!=test_symbol_p.name and test_symbol_cont2.name!=test_symbol_cl.name):
				raise WrongRuleExecution('humanClassifiesMug_trigger8')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['newobj'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'mug')
		# Retype nodes
		newNode.graph.nodes[n2id['cl']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['newobj'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o_h'], smap['cont2'], 'in')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 2
			newNode.depth += 1
		newNode.history.append('humanClassifiesMug@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule humanClassifiesTable
	def humanClassifiesTable(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for cl
			symbol_cl_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cl'] in equiv[0] and equiv[1] != None:
					symbol_cl_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for p
			symbol_p_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'p'] in equiv[0] and equiv[1] != None:
					symbol_p_nodes = [equiv[1]]
			# Find equivalence for see
			symbol_see_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'see'] in equiv[0] and equiv[1] != None:
					symbol_see_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o_h
			symbol_o_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o_h'] in equiv[0] and equiv[1] != None:
					symbol_o_h_nodes = [equiv[1]]
		else:
			symbol_cl_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_p_nodes = symbol_nodes_copy
			symbol_see_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_h_name in symbol_o_h_nodes:
			symbol_o_h = nodes[symbol_o_h_name]
			n2id['o_h'] = symbol_o_h_name
			if symbol_o_h.sType == 'object':
				for symbol_r_name in symbol_r_nodes:
					symbol_r = nodes[symbol_r_name]
					n2id['r'] = symbol_r_name
					if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o_h.name:
						for symbol_o_name in symbol_o_nodes:
							symbol_o = nodes[symbol_o_name]
							n2id['o'] = symbol_o_name
							if symbol_o.sType == 'object' and symbol_o.name!=symbol_o_h.name and symbol_o.name!=symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o_h.name and symbol_h.name!=symbol_r.name and symbol_h.name!=symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links:
										for symbol_see_name in symbol_see_nodes:
											symbol_see = nodes[symbol_see_name]
											n2id['see'] = symbol_see_name
											if symbol_see.sType == 'see' and symbol_see.name!=symbol_o_h.name and symbol_see.name!=symbol_r.name and symbol_see.name!=symbol_o.name and symbol_see.name!=symbol_h.name and [n2id["o_h"],n2id["see"],"prop"] in snode.graph.links:
												for symbol_p_name in symbol_p_nodes:
													symbol_p = nodes[symbol_p_name]
													n2id['p'] = symbol_p_name
													if symbol_p.sType == 'pose' and symbol_p.name!=symbol_o_h.name and symbol_p.name!=symbol_r.name and symbol_p.name!=symbol_o.name and symbol_p.name!=symbol_h.name and symbol_p.name!=symbol_see.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links:
														for symbol_cl_name in symbol_cl_nodes:
															symbol_cl = nodes[symbol_cl_name]
															n2id['cl'] = symbol_cl_name
															if symbol_cl.sType == 'noClass' and symbol_cl.name!=symbol_o_h.name and symbol_cl.name!=symbol_r.name and symbol_cl.name!=symbol_o.name and symbol_cl.name!=symbol_h.name and symbol_cl.name!=symbol_see.name and symbol_cl.name!=symbol_p.name and [n2id["o_h"],n2id["cl"],"prop"] in snode.graph.links:
																# At this point we meet all the conditions.
																# Insert additional conditions manually here if you want.
																# (beware that the code could be regenerated and you might lose your changes).
																stack2        = copy.deepcopy(stack)
																equivalences2 = copy.deepcopy(equivalences)
																r1 = self.humanClassifiesTable_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
																c = copy.deepcopy(r1)
																if 'fina' in locals():
																	c.history.append(finishesCombo)
																if len(stack2) > 0: c.stop = True
																ret.append(c)
																if len(stack2) > 0:
																	# Set symbol for o_h...
																	for equiv in equivalences2:
																		if [me, 'o_h'] in equiv[0]:
																			equiv[1] = symbol_o_h_name
																	# Set symbol for r...
																	for equiv in equivalences2:
																		if [me, 'r'] in equiv[0]:
																			equiv[1] = symbol_r_name
																	# Set symbol for o...
																	for equiv in equivalences2:
																		if [me, 'o'] in equiv[0]:
																			equiv[1] = symbol_o_name
																	# Set symbol for h...
																	for equiv in equivalences2:
																		if [me, 'h'] in equiv[0]:
																			equiv[1] = symbol_h_name
																	# Set symbol for see...
																	for equiv in equivalences2:
																		if [me, 'see'] in equiv[0]:
																			equiv[1] = symbol_see_name
																	# Set symbol for p...
																	for equiv in equivalences2:
																		if [me, 'p'] in equiv[0]:
																			equiv[1] = symbol_p_name
																	# Set symbol for cl...
																	for equiv in equivalences2:
																		if [me, 'cl'] in equiv[0]:
																			equiv[1] = symbol_cl_name
																	newNode = WorldStateHistory(r1)
																	global lastNodeId
																	lastNodeId += 1
																	newNode.nodeId = lastNodeId
																	newNode.parentId = snode.nodeId
																	derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
																	if 'fina' in locals():
																		for n in derivsx: n.history.append(finishesCombo)
																		for n in derivsx: n.history.append(fina)
																	ret.extend(derivsx)
		return ret
		
		

	# Rule humanClassifiesTable
	def humanClassifiesTable_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o_h = snode.graph.nodes[n2id['o_h']]
			if not (test_symbol_o_h.sType == 'object'):
				raise WrongRuleExecution('humanClassifiesTable_trigger1')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o_h.name):
				raise WrongRuleExecution('humanClassifiesTable_trigger2')
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object' and test_symbol_o.name!=test_symbol_o_h.name and test_symbol_o.name!=test_symbol_r.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesTable_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o_h.name and test_symbol_h.name!=test_symbol_r.name and test_symbol_h.name!=test_symbol_o.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesTable_trigger4')
			test_symbol_see = snode.graph.nodes[n2id['see']]
			if not (test_symbol_see.sType == 'see' and test_symbol_see.name!=test_symbol_o_h.name and test_symbol_see.name!=test_symbol_r.name and test_symbol_see.name!=test_symbol_o.name and test_symbol_see.name!=test_symbol_h.name and [n2id["o_h"],n2id["see"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesTable_trigger5')
			test_symbol_p = snode.graph.nodes[n2id['p']]
			if not (test_symbol_p.sType == 'pose' and test_symbol_p.name!=test_symbol_o_h.name and test_symbol_p.name!=test_symbol_r.name and test_symbol_p.name!=test_symbol_o.name and test_symbol_p.name!=test_symbol_h.name and test_symbol_p.name!=test_symbol_see.name and [n2id["o_h"],n2id["p"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesTable_trigger6')
			test_symbol_cl = snode.graph.nodes[n2id['cl']]
			if not (test_symbol_cl.sType == 'noClass' and test_symbol_cl.name!=test_symbol_o_h.name and test_symbol_cl.name!=test_symbol_r.name and test_symbol_cl.name!=test_symbol_o.name and test_symbol_cl.name!=test_symbol_h.name and test_symbol_cl.name!=test_symbol_see.name and test_symbol_cl.name!=test_symbol_p.name and [n2id["o_h"],n2id["cl"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanClassifiesTable_trigger7')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['newobj'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'table')
		# Retype nodes
		newNode.graph.nodes[n2id['cl']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o_h'], smap['newobj'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 2
			newNode.depth += 1
		newNode.history.append('humanClassifiesTable@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule humanTellsUsAboutMug
	def humanTellsUsAboutMug(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for ch
			symbol_ch_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'ch'] in equiv[0] and equiv[1] != None:
					symbol_ch_nodes = [equiv[1]]
			# Find equivalence for f
			symbol_f_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'f'] in equiv[0] and equiv[1] != None:
					symbol_f_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for mh
			symbol_mh_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'mh'] in equiv[0] and equiv[1] != None:
					symbol_mh_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for cont1
			symbol_cont1_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cont1'] in equiv[0] and equiv[1] != None:
					symbol_cont1_nodes = [equiv[1]]
			# Find equivalence for cont2
			symbol_cont2_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'cont2'] in equiv[0] and equiv[1] != None:
					symbol_cont2_nodes = [equiv[1]]
			# Find equivalence for o_h
			symbol_o_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o_h'] in equiv[0] and equiv[1] != None:
					symbol_o_h_nodes = [equiv[1]]
		else:
			symbol_ch_nodes = symbol_nodes_copy
			symbol_f_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_mh_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_cont1_nodes = symbol_nodes_copy
			symbol_cont2_nodes = symbol_nodes_copy
			symbol_o_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_h_name in symbol_o_h_nodes:
			symbol_o_h = nodes[symbol_o_h_name]
			n2id['o_h'] = symbol_o_h_name
			if symbol_o_h.sType == 'object':
				for symbol_o_name in symbol_o_nodes:
					symbol_o = nodes[symbol_o_name]
					n2id['o'] = symbol_o_name
					if symbol_o.sType == 'object' and symbol_o.name!=symbol_o_h.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links:
						for symbol_r_name in symbol_r_nodes:
							symbol_r = nodes[symbol_r_name]
							n2id['r'] = symbol_r_name
							if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o_h.name and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o_h.name and symbol_h.name!=symbol_o.name and symbol_h.name!=symbol_r.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links:
										for symbol_cont2_name in symbol_cont2_nodes:
											symbol_cont2 = nodes[symbol_cont2_name]
											n2id['cont2'] = symbol_cont2_name
											if symbol_cont2.sType == 'object' and symbol_cont2.name!=symbol_o_h.name and symbol_cont2.name!=symbol_o.name and symbol_cont2.name!=symbol_r.name and symbol_cont2.name!=symbol_h.name and [n2id["o_h"],n2id["cont2"],"in"] in snode.graph.links:
												for symbol_cont1_name in symbol_cont1_nodes:
													symbol_cont1 = nodes[symbol_cont1_name]
													n2id['cont1'] = symbol_cont1_name
													if symbol_cont1.sType == 'object' and symbol_cont1.name!=symbol_o_h.name and symbol_cont1.name!=symbol_o.name and symbol_cont1.name!=symbol_r.name and symbol_cont1.name!=symbol_h.name and symbol_cont1.name!=symbol_cont2.name and [n2id["cont1"],n2id["cont2"],"eq"] in snode.graph.links:
														for symbol_mh_name in symbol_mh_nodes:
															symbol_mh = nodes[symbol_mh_name]
															n2id['mh'] = symbol_mh_name
															if symbol_mh.sType == 'mug' and symbol_mh.name!=symbol_o_h.name and symbol_mh.name!=symbol_o.name and symbol_mh.name!=symbol_r.name and symbol_mh.name!=symbol_h.name and symbol_mh.name!=symbol_cont2.name and symbol_mh.name!=symbol_cont1.name and [n2id["o_h"],n2id["mh"],"prop"] in snode.graph.links:
																for symbol_f_name in symbol_f_nodes:
																	symbol_f = nodes[symbol_f_name]
																	n2id['f'] = symbol_f_name
																	if symbol_f.sType == 'classFail' and symbol_f.name!=symbol_o_h.name and symbol_f.name!=symbol_o.name and symbol_f.name!=symbol_r.name and symbol_f.name!=symbol_h.name and symbol_f.name!=symbol_cont2.name and symbol_f.name!=symbol_cont1.name and symbol_f.name!=symbol_mh.name and [n2id["o"],n2id["f"],"prop"] in snode.graph.links:
																		for symbol_ch_name in symbol_ch_nodes:
																			symbol_ch = nodes[symbol_ch_name]
																			n2id['ch'] = symbol_ch_name
																			if symbol_ch.sType == 'class' and symbol_ch.name!=symbol_o_h.name and symbol_ch.name!=symbol_o.name and symbol_ch.name!=symbol_r.name and symbol_ch.name!=symbol_h.name and symbol_ch.name!=symbol_cont2.name and symbol_ch.name!=symbol_cont1.name and symbol_ch.name!=symbol_mh.name and symbol_ch.name!=symbol_f.name and [n2id["o_h"],n2id["ch"],"prop"] in snode.graph.links:
																				# At this point we meet all the conditions.
																				# Insert additional conditions manually here if you want.
																				# (beware that the code could be regenerated and you might lose your changes).
																				stack2        = copy.deepcopy(stack)
																				equivalences2 = copy.deepcopy(equivalences)
																				r1 = self.humanTellsUsAboutMug_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
																				c = copy.deepcopy(r1)
																				if 'fina' in locals():
																					c.history.append(finishesCombo)
																				if len(stack2) > 0: c.stop = True
																				ret.append(c)
																				if len(stack2) > 0:
																					# Set symbol for o_h...
																					for equiv in equivalences2:
																						if [me, 'o_h'] in equiv[0]:
																							equiv[1] = symbol_o_h_name
																					# Set symbol for o...
																					for equiv in equivalences2:
																						if [me, 'o'] in equiv[0]:
																							equiv[1] = symbol_o_name
																					# Set symbol for r...
																					for equiv in equivalences2:
																						if [me, 'r'] in equiv[0]:
																							equiv[1] = symbol_r_name
																					# Set symbol for h...
																					for equiv in equivalences2:
																						if [me, 'h'] in equiv[0]:
																							equiv[1] = symbol_h_name
																					# Set symbol for cont2...
																					for equiv in equivalences2:
																						if [me, 'cont2'] in equiv[0]:
																							equiv[1] = symbol_cont2_name
																					# Set symbol for cont1...
																					for equiv in equivalences2:
																						if [me, 'cont1'] in equiv[0]:
																							equiv[1] = symbol_cont1_name
																					# Set symbol for mh...
																					for equiv in equivalences2:
																						if [me, 'mh'] in equiv[0]:
																							equiv[1] = symbol_mh_name
																					# Set symbol for f...
																					for equiv in equivalences2:
																						if [me, 'f'] in equiv[0]:
																							equiv[1] = symbol_f_name
																					# Set symbol for ch...
																					for equiv in equivalences2:
																						if [me, 'ch'] in equiv[0]:
																							equiv[1] = symbol_ch_name
																					newNode = WorldStateHistory(r1)
																					global lastNodeId
																					lastNodeId += 1
																					newNode.nodeId = lastNodeId
																					newNode.parentId = snode.nodeId
																					derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
																					if 'fina' in locals():
																						for n in derivsx: n.history.append(finishesCombo)
																						for n in derivsx: n.history.append(fina)
																					ret.extend(derivsx)
		return ret
		
		

	# Rule humanTellsUsAboutMug
	def humanTellsUsAboutMug_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o_h = snode.graph.nodes[n2id['o_h']]
			if not (test_symbol_o_h.sType == 'object'):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger1')
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object' and test_symbol_o.name!=test_symbol_o_h.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger2')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o_h.name and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o_h.name and test_symbol_h.name!=test_symbol_o.name and test_symbol_h.name!=test_symbol_r.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger4')
			test_symbol_cont2 = snode.graph.nodes[n2id['cont2']]
			if not (test_symbol_cont2.sType == 'object' and test_symbol_cont2.name!=test_symbol_o_h.name and test_symbol_cont2.name!=test_symbol_o.name and test_symbol_cont2.name!=test_symbol_r.name and test_symbol_cont2.name!=test_symbol_h.name and [n2id["o_h"],n2id["cont2"],"in"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger5')
			test_symbol_cont1 = snode.graph.nodes[n2id['cont1']]
			if not (test_symbol_cont1.sType == 'object' and test_symbol_cont1.name!=test_symbol_o_h.name and test_symbol_cont1.name!=test_symbol_o.name and test_symbol_cont1.name!=test_symbol_r.name and test_symbol_cont1.name!=test_symbol_h.name and test_symbol_cont1.name!=test_symbol_cont2.name and [n2id["cont1"],n2id["cont2"],"eq"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger6')
			test_symbol_mh = snode.graph.nodes[n2id['mh']]
			if not (test_symbol_mh.sType == 'mug' and test_symbol_mh.name!=test_symbol_o_h.name and test_symbol_mh.name!=test_symbol_o.name and test_symbol_mh.name!=test_symbol_r.name and test_symbol_mh.name!=test_symbol_h.name and test_symbol_mh.name!=test_symbol_cont2.name and test_symbol_mh.name!=test_symbol_cont1.name and [n2id["o_h"],n2id["mh"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger7')
			test_symbol_f = snode.graph.nodes[n2id['f']]
			if not (test_symbol_f.sType == 'classFail' and test_symbol_f.name!=test_symbol_o_h.name and test_symbol_f.name!=test_symbol_o.name and test_symbol_f.name!=test_symbol_r.name and test_symbol_f.name!=test_symbol_h.name and test_symbol_f.name!=test_symbol_cont2.name and test_symbol_f.name!=test_symbol_cont1.name and test_symbol_f.name!=test_symbol_mh.name and [n2id["o"],n2id["f"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger8')
			test_symbol_ch = snode.graph.nodes[n2id['ch']]
			if not (test_symbol_ch.sType == 'class' and test_symbol_ch.name!=test_symbol_o_h.name and test_symbol_ch.name!=test_symbol_o.name and test_symbol_ch.name!=test_symbol_r.name and test_symbol_ch.name!=test_symbol_h.name and test_symbol_ch.name!=test_symbol_cont2.name and test_symbol_ch.name!=test_symbol_cont1.name and test_symbol_ch.name!=test_symbol_mh.name and test_symbol_ch.name!=test_symbol_f.name and [n2id["o_h"],n2id["ch"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutMug_trigger9')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['mr'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'mug')
		# Retype nodes
		newNode.graph.nodes[n2id['f']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['mr'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		l = AGMLink(smap['o'], smap['cont1'], 'in')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 3
			newNode.depth += 1
		newNode.history.append('humanTellsUsAboutMug@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		

	# Rule humanTellsUsAboutTable
	def humanTellsUsAboutTable(self, snode, stackP=[], equivalencesP=[]):
		stack        = copy.deepcopy(stackP)
		equivalences = copy.deepcopy(equivalencesP)
		symbol_nodes_copy = copy.deepcopy(snode.graph.nodes)
		finishesCombo = ''
		if len(stack) > 0:
			pop = stack.pop()
			me = pop[0]
			if len(pop)>2:
				finishesCombo = copy.deepcopy(pop[2])
				fina = copy.deepcopy(pop[2])
			# Find equivalence for ch
			symbol_ch_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'ch'] in equiv[0] and equiv[1] != None:
					symbol_ch_nodes = [equiv[1]]
			# Find equivalence for f
			symbol_f_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'f'] in equiv[0] and equiv[1] != None:
					symbol_f_nodes = [equiv[1]]
			# Find equivalence for h
			symbol_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'h'] in equiv[0] and equiv[1] != None:
					symbol_h_nodes = [equiv[1]]
			# Find equivalence for mh
			symbol_mh_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'mh'] in equiv[0] and equiv[1] != None:
					symbol_mh_nodes = [equiv[1]]
			# Find equivalence for o
			symbol_o_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o'] in equiv[0] and equiv[1] != None:
					symbol_o_nodes = [equiv[1]]
			# Find equivalence for r
			symbol_r_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'r'] in equiv[0] and equiv[1] != None:
					symbol_r_nodes = [equiv[1]]
			# Find equivalence for o_h
			symbol_o_h_nodes = symbol_nodes_copy
			for equiv in equivalences:
				if [me, 'o_h'] in equiv[0] and equiv[1] != None:
					symbol_o_h_nodes = [equiv[1]]
		else:
			symbol_ch_nodes = symbol_nodes_copy
			symbol_f_nodes = symbol_nodes_copy
			symbol_h_nodes = symbol_nodes_copy
			symbol_mh_nodes = symbol_nodes_copy
			symbol_o_nodes = symbol_nodes_copy
			symbol_r_nodes = symbol_nodes_copy
			symbol_o_h_nodes = symbol_nodes_copy
		ret = []
		nodes = copy.deepcopy(snode.graph.nodes)
		n2id = dict()
		for symbol_o_h_name in symbol_o_h_nodes:
			symbol_o_h = nodes[symbol_o_h_name]
			n2id['o_h'] = symbol_o_h_name
			if symbol_o_h.sType == 'object':
				for symbol_o_name in symbol_o_nodes:
					symbol_o = nodes[symbol_o_name]
					n2id['o'] = symbol_o_name
					if symbol_o.sType == 'object' and symbol_o.name!=symbol_o_h.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links:
						for symbol_r_name in symbol_r_nodes:
							symbol_r = nodes[symbol_r_name]
							n2id['r'] = symbol_r_name
							if symbol_r.sType == 'robot' and symbol_r.name!=symbol_o_h.name and symbol_r.name!=symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links:
								for symbol_h_name in symbol_h_nodes:
									symbol_h = nodes[symbol_h_name]
									n2id['h'] = symbol_h_name
									if symbol_h.sType == 'human' and symbol_h.name!=symbol_o_h.name and symbol_h.name!=symbol_o.name and symbol_h.name!=symbol_r.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links:
										for symbol_mh_name in symbol_mh_nodes:
											symbol_mh = nodes[symbol_mh_name]
											n2id['mh'] = symbol_mh_name
											if symbol_mh.sType == 'table' and symbol_mh.name!=symbol_o_h.name and symbol_mh.name!=symbol_o.name and symbol_mh.name!=symbol_r.name and symbol_mh.name!=symbol_h.name and [n2id["o_h"],n2id["mh"],"prop"] in snode.graph.links:
												for symbol_f_name in symbol_f_nodes:
													symbol_f = nodes[symbol_f_name]
													n2id['f'] = symbol_f_name
													if symbol_f.sType == 'classFail' and symbol_f.name!=symbol_o_h.name and symbol_f.name!=symbol_o.name and symbol_f.name!=symbol_r.name and symbol_f.name!=symbol_h.name and symbol_f.name!=symbol_mh.name and [n2id["o"],n2id["f"],"prop"] in snode.graph.links:
														for symbol_ch_name in symbol_ch_nodes:
															symbol_ch = nodes[symbol_ch_name]
															n2id['ch'] = symbol_ch_name
															if symbol_ch.sType == 'class' and symbol_ch.name!=symbol_o_h.name and symbol_ch.name!=symbol_o.name and symbol_ch.name!=symbol_r.name and symbol_ch.name!=symbol_h.name and symbol_ch.name!=symbol_mh.name and symbol_ch.name!=symbol_f.name and [n2id["o_h"],n2id["ch"],"prop"] in snode.graph.links:
																# At this point we meet all the conditions.
																# Insert additional conditions manually here if you want.
																# (beware that the code could be regenerated and you might lose your changes).
																stack2        = copy.deepcopy(stack)
																equivalences2 = copy.deepcopy(equivalences)
																r1 = self.humanTellsUsAboutTable_trigger(snode, n2id, stack2, equivalences2, copy.deepcopy(finishesCombo))
																c = copy.deepcopy(r1)
																if 'fina' in locals():
																	c.history.append(finishesCombo)
																if len(stack2) > 0: c.stop = True
																ret.append(c)
																if len(stack2) > 0:
																	# Set symbol for o_h...
																	for equiv in equivalences2:
																		if [me, 'o_h'] in equiv[0]:
																			equiv[1] = symbol_o_h_name
																	# Set symbol for o...
																	for equiv in equivalences2:
																		if [me, 'o'] in equiv[0]:
																			equiv[1] = symbol_o_name
																	# Set symbol for r...
																	for equiv in equivalences2:
																		if [me, 'r'] in equiv[0]:
																			equiv[1] = symbol_r_name
																	# Set symbol for h...
																	for equiv in equivalences2:
																		if [me, 'h'] in equiv[0]:
																			equiv[1] = symbol_h_name
																	# Set symbol for mh...
																	for equiv in equivalences2:
																		if [me, 'mh'] in equiv[0]:
																			equiv[1] = symbol_mh_name
																	# Set symbol for f...
																	for equiv in equivalences2:
																		if [me, 'f'] in equiv[0]:
																			equiv[1] = symbol_f_name
																	# Set symbol for ch...
																	for equiv in equivalences2:
																		if [me, 'ch'] in equiv[0]:
																			equiv[1] = symbol_ch_name
																	newNode = WorldStateHistory(r1)
																	global lastNodeId
																	lastNodeId += 1
																	newNode.nodeId = lastNodeId
																	newNode.parentId = snode.nodeId
																	derivsx = self.getRules()[stack2[-1][1]](newNode, stack2, equivalences2)
																	if 'fina' in locals():
																		for n in derivsx: n.history.append(finishesCombo)
																		for n in derivsx: n.history.append(fina)
																	ret.extend(derivsx)
		return ret
		
		

	# Rule humanTellsUsAboutTable
	def humanTellsUsAboutTable_trigger(self, snode, n2id, stack=[], equivalences=[], checked=True, finish=''):
		if not checked:
			test_symbol_o_h = snode.graph.nodes[n2id['o_h']]
			if not (test_symbol_o_h.sType == 'object'):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger1')
			test_symbol_o = snode.graph.nodes[n2id['o']]
			if not (test_symbol_o.sType == 'object' and test_symbol_o.name!=test_symbol_o_h.name and [n2id["o"],n2id["o_h"],"eq"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger2')
			test_symbol_r = snode.graph.nodes[n2id['r']]
			if not (test_symbol_r.sType == 'robot' and test_symbol_r.name!=test_symbol_o_h.name and test_symbol_r.name!=test_symbol_o.name and [n2id["r"],n2id["o"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger3')
			test_symbol_h = snode.graph.nodes[n2id['h']]
			if not (test_symbol_h.sType == 'human' and test_symbol_h.name!=test_symbol_o_h.name and test_symbol_h.name!=test_symbol_o.name and test_symbol_h.name!=test_symbol_r.name and [n2id["r"],n2id["h"],"interact"] in snode.graph.links and [n2id["h"],n2id["o_h"],"know"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger4')
			test_symbol_mh = snode.graph.nodes[n2id['mh']]
			if not (test_symbol_mh.sType == 'table' and test_symbol_mh.name!=test_symbol_o_h.name and test_symbol_mh.name!=test_symbol_o.name and test_symbol_mh.name!=test_symbol_r.name and test_symbol_mh.name!=test_symbol_h.name and [n2id["o_h"],n2id["mh"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger5')
			test_symbol_f = snode.graph.nodes[n2id['f']]
			if not (test_symbol_f.sType == 'classFail' and test_symbol_f.name!=test_symbol_o_h.name and test_symbol_f.name!=test_symbol_o.name and test_symbol_f.name!=test_symbol_r.name and test_symbol_f.name!=test_symbol_h.name and test_symbol_f.name!=test_symbol_mh.name and [n2id["o"],n2id["f"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger6')
			test_symbol_ch = snode.graph.nodes[n2id['ch']]
			if not (test_symbol_ch.sType == 'class' and test_symbol_ch.name!=test_symbol_o_h.name and test_symbol_ch.name!=test_symbol_o.name and test_symbol_ch.name!=test_symbol_r.name and test_symbol_ch.name!=test_symbol_h.name and test_symbol_ch.name!=test_symbol_mh.name and test_symbol_ch.name!=test_symbol_f.name and [n2id["o_h"],n2id["ch"],"prop"] in snode.graph.links):
				raise WrongRuleExecution('humanTellsUsAboutTable_trigger7')
		smap = copy.deepcopy(n2id)
		newNode = WorldStateHistory(snode)
		global lastNodeId
		lastNodeId += 1
		newNode.nodeId = lastNodeId
		newNode.parentId = snode.nodeId
		# Create nodes
		newName = str(getNewIdForSymbol(newNode))
		smap['mr'] = newName
		newNode.graph.nodes[newName] = AGMSymbol(newName, 'table')
		# Retype nodes
		newNode.graph.nodes[n2id['f']].sType = 'class'
		# Remove nodes
		# Remove links
		# Create links
		l = AGMLink(smap['o'], smap['mr'], 'prop')
		if not l in newNode.graph.links:
			newNode.graph.links.append(l)
		# Misc stuff
		newNode.probability *= 1.
		if len(stack) == 0:
			newNode.cost += 3
			newNode.depth += 1
		newNode.history.append('humanTellsUsAboutTable@' + str(n2id) )
		if finish!='': newNode.history.append(finish)
		return newNode
		
		
