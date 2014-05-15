/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

/**
       \brief
       @author authorname
*/

struct ModificationListsContainer
{
	// Nodes                        GualzruWorldNode
	RoboCompAGMWorldModel::NodeSequence newNodes;
	RoboCompAGMWorldModel::NodeSequence constantNodes;
	RoboCompAGMWorldModel::NodeSequence removedNodes;
	// Edges
	RoboCompAGMWorldModel::EdgeSequence newEdges;
	RoboCompAGMWorldModel::EdgeSequence constantEdges;
	RoboCompAGMWorldModel::EdgeSequence removedEdges;
};

class SpecificWorker : public GenericWorker
{
 
  	bool initialized;
  
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void  killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();
	void  modelModified(const RoboCompAGMWorldModel::Event& modification);
	void  modelUpdated(const RoboCompAGMWorldModel::Node& modification);
	void RCIS_removeNode_nonexistingok(std::string nodeName);
	

public slots:
 	void compute(); 	
	
	
private:
  
  bool initial_broadcast;
  ModificationListsContainer modificationList;
  
  //Node management functions
  std::string node2String(const RoboCompAGMWorldModel::Node &node);
  void buildModificationLists(const RoboCompAGMWorldModel::Event &event);
  void buildNodeModificationLists(const RoboCompAGMWorldModel::Event &event);
	void buildEdgeModificationLists(const RoboCompAGMWorldModel::Event &event);
  
  //Update Nodes from RCIS
  void updateRCISNode(const RoboCompAGMWorldModel::Node &modified);
  void RCIS_addObjectNode(RoboCompAGMWorldModel::Node node);
  
  //Specific RCIS Update
  void RCIS_update_object(RoboCompAGMWorldModel::Node &node);
  void updateRCISModel(const RoboCompAGMWorldModel::Event& modification);
  
};

#endif