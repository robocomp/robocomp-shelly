
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
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	worldModel = AGMModel::SPtr(new AGMModel());
	initialized = false;
	initial_broadcast=false;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
  if (!initial_broadcast)
	 agmexecutive_proxy -> broadcastModel();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
bool SpecificWorker::activateAgent(const ParameterMap& prs){
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated)){
		if (not activated){
			return activate(p);
		}
	}else{
		return false;
	}
	return true;
}

bool SpecificWorker::deactivateAgent(){
		return deactivate();
}

StateStruct SpecificWorker::getAgentState(){
	StateStruct s;
	if (isActive()){
		s.state = Running;
	}else{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

ParameterMap SpecificWorker::getAgentParameters(){
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs){
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent(){
}
Ice::Int SpecificWorker::uptimeAgent(){
	return 0;
}

bool SpecificWorker::reloadConfigAgent(){
	return true;
}


void SpecificWorker::modelModified(const RoboCompAGMWorldModel::Event& modification){
	 printf(" - - - - m - - - -\n");
	 initial_broadcast=true;
	 mutex->lock();
	 AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	 //updateRCISModel(modification);
	 buildModificationLists(modification);
	 if (initialized)
	 {
		  updateRCISModel(modification);
		  mutex->unlock();
	 }
	 else
	 {
		  initialized = true;
		  RoboCompAGMWorldModel::Event modification2 = modification;
		  modification2.backModel.nodes.clear();
		  modification2.backModel.edges.clear();
		  mutex->unlock();
		  modelModified(modification2);
	 }
	mutex->unlock();
}

void SpecificWorker::modelUpdated(const RoboCompAGMWorldModel::Node& modification){
 //  printf(" - - - - u - - - -\n");
	if(initial_broadcast)
	{
		mutex->lock();
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		updateRCISNode(modification);
		mutex->unlock();
	}
}

std::string SpecificWorker::node2String(const RoboCompAGMWorldModel::Node &node)
{
	std::ostringstream ss;
	ss << node.nodeType << "_" << node.nodeIdentifier;
	return ss.str();
}

void SpecificWorker::updateRCISNode(const RoboCompAGMWorldModel::Node &modified)
{
	 RoboCompAGMWorldModel::Node node = modified;
    printf(" - - - - updateRCISNode %s\n", node2String(modified).c_str());
	 if(node.nodeType == "object")
		RCIS_update_object(node);
}

void SpecificWorker::updateRCISModel(const RoboCompAGMWorldModel::Event& modification)
{
    printf(" - - - - updateRCISModel - - - -\n");
	 
	/// Remove nodes that have been removed in the model
	for (uint32_t node=0; node<modificationList.removedNodes.size(); node++)
	{
	//	printf("NODE: %s\n", modificationList.removedNodes[node].nodeType);
		if (modificationList.removedNodes[node].nodeType=="object") 
		{
		  printf("DELETE\n");
		  RCIS_removeNode_nonexistingok(node2String(modificationList.removedNodes[node]));
		}
	}

	/// Add nodes that have been added in the model
  for (uint32_t nodeIdx=0; nodeIdx<modificationList.newNodes.size(); nodeIdx++)
  {
	 RoboCompAGMWorldModel::Node &node = modificationList.newNodes[nodeIdx];
// 	 bool print = false;
	 if (node.nodeType == "object")              { RCIS_addObjectNode(node); }
  }
	
	 
}

void SpecificWorker::buildModificationLists(const RoboCompAGMWorldModel::Event &event)
{
  	buildNodeModificationLists(event);
	buildEdgeModificationLists(event);
}

void SpecificWorker::buildNodeModificationLists(const RoboCompAGMWorldModel::Event &event)
{
	modificationList.newNodes.clear();
	modificationList.constantNodes.clear();
	modificationList.removedNodes.clear();
// 	printf("buildNodeModificationLists() [%d->%d]\n", (int)event.backModel.nodes.size(), (int)event.newModel.nodes.size());
	/// Append to 'removed' vector those nodes which are in the previous vector but not in the current one.
	/// Append to 'remains' those which are both in the previous and the current node vector.
// 	printf("REMOVES, CONSTANT\n");
	printf("SIZES: BACK-MODEL: %lu    NEW-MODEL: %lu\n", event.backModel.nodes.size(), event.newModel.nodes.size());
	for (uint32_t bi=0; bi<event.backModel.nodes.size(); bi++)
	{
// 		printf("Check for %s:\n", node2String(event.backModel.nodes[bi]).c_str());
		bool found = false;
		for (uint32_t ci=0; ci<event.newModel.nodes.size(); ci++)
		{
// 			printf(" - %s\n", node2String(event.newModel.nodes[ci]).c_str());
			if (event.backModel.nodes[bi] == event.newModel.nodes[ci])
			{
				found = true;
				break;
			}
		}
		if (not found)
		{
 			printf("not found! --> including it in removedNodes\n");
			modificationList.removedNodes.push_back(event.backModel.nodes[bi]);
		}
		else
		{
 			printf("  found!   --> including it in constantNodes\n");
			modificationList.constantNodes.push_back(event.backModel.nodes[bi]);
		}
	}
	/// Append to 'added' vector those nodes which are in the current vector but not in the previous one.
 	printf("ADDS\n");
	for (uint32_t ci=0; ci<event.newModel.nodes.size(); ci++)
	{
 		printf("Check for %s:\n", node2String(event.newModel.nodes[ci]).c_str());
		bool found = false;
		for (uint32_t bi=0; bi<event.backModel.nodes.size(); bi++)
		{
// 			printf(" - %s\n", node2String(event.backModel.nodes[bi]).c_str());
			if (event.backModel.nodes[bi] == event.newModel.nodes[ci])
			{
				found = true;
				break;
			}
		}
		if (not found)
		{
 			printf("not found! --> including it in newNodes\n");
			modificationList.newNodes.push_back(event.newModel.nodes[ci]);
		}
// 		else
// 			printf("found\n");
	}
}

void SpecificWorker::buildEdgeModificationLists(const RoboCompAGMWorldModel::Event &event)
{
	modificationList.newEdges.clear();
	modificationList.constantEdges.clear();
	modificationList.removedEdges.clear();

	/// Append to 'removed' vector those edges which are in the previous vector but not in the current one.
	/// Append to 'remains' those which are both in the previous and the current edge vector.
	for (uint32_t bi=0; bi<event.backModel.edges.size(); bi++)
	{
		bool found = false;
		for (uint32_t ci=0; ci<event.newModel.edges.size(); ci++)
		{
			if (event.backModel.edges[bi] == event.newModel.edges[ci])
			{
				found = true;
				break;
			}
		}
		if (not found)
			modificationList.removedEdges.push_back(event.backModel.edges[bi]);
		else
			modificationList.constantEdges.push_back(event.backModel.edges[bi]);
	}
	/// Append to 'added' vector those edges which are in the current vector but not in the previous one.
	for (uint32_t ci=0; ci<event.newModel.edges.size(); ci++)
	{
		bool found = false;
		for (uint32_t bi=0; bi<event.backModel.edges.size(); bi++)
		{
			if (event.backModel.edges[bi] == event.newModel.edges[ci])
			{
				found = true;
				break;
			}
		}
		if (not found)
			modificationList.newEdges.push_back(event.newModel.edges[ci]);
	}
}

void SpecificWorker::RCIS_addObjectNode(RoboCompAGMWorldModel::Node node)
{
  try
  { 
	 RoboCompInnerModelManager::Pose3D pose;
	 
	 pose.x = str2float(node.attributes["tx"]);
	 pose.y = str2float(node.attributes["ty"]);
	 pose.z = str2float(node.attributes["tz"]);
	 pose.rx = str2float(node.attributes["rx"]);
	 pose.ry = str2float(node.attributes["ry"]);
	 pose.rz = str2float(node.attributes["rz"]);
	 
	 
	printf("Rx: %s\n",std::string(node.attributes[std::string("rx")]).c_str() );
	printf("Ry: %s\n",std::string(node.attributes[std::string("ry")]).c_str());
	printf("Rz: %s\n",std::string(node.attributes[std::string("rz")]).c_str());
	printf("Tx: %s\n",std::string(node.attributes[std::string("tx")]).c_str());
	printf("Ty: %s\n",std::string(node.attributes[std::string("ty")] ).c_str());
	printf("Tz: %s\n",std::string(node.attributes[std::string("tz")]).c_str());
// 	 pose.rx = 0;
//  	 pose.ry = 0;
//   	 pose.rz = 0;
	 
	 QString ident = QString::fromStdString(node.nodeType);
	 ident += "_";
	 ident += QString::number(node.nodeIdentifier);
	 //add the mesh
	 
	 RoboCompInnerModelManager::meshType mesh;
	 
	 //mesh.pose = pose;
	 mesh.pose.x = 0;
	 mesh.pose.y = 0;
	 mesh.pose.z = 0;
	 mesh.pose.rx = 0;
	 mesh.pose.ry = 0;
	 mesh.pose.rz = 0;
	 
	 mesh.scaleX = mesh.scaleY = 100; // <--- A 674mm radius table has a scale of "100"
	 mesh.scaleZ =              100; // <--- A 800mm height table has a scale of "100"
	 mesh.render = 0;
	 
	 if ( node.nodeIdentifier == 8)
	 {
		printf("taza!\n");
		mesh.meshPath = "/home/robocomp/robocomp/Files/osgModels/mobiliario/taza.osg";
		mesh.pose.rx += 1.57;
		mesh.pose.rz += 1.57;
	 }
	 else
	 {
		printf("mesa!\n");
		mesh.meshPath = "/home/robocomp/robocomp/Files/osgModels/mobiliario/mesa_redonda.osg";
		//mesh.pose.rx += -1.57; //mesh from table is rotated
	 }
	 
	 //add the transofrm
	 innermodelmanager_proxy->addTransform(ident.toStdString() + "_T", "static", "sensor_transform", pose);
	 //innermodelmanager_proxy->addTransform("test_T", "static", "sensor_transform", pose);
	 
	 innermodelmanager_proxy->addMesh(ident.toStdString(), ident.toStdString() + "_T", mesh);
	 //innermodelmanager_proxy->addMesh("test", "test_T", mesh);
	 printf("ADDED: %s", ident.toStdString().c_str());
  }	
  catch (InnerModelManagerError e)
  {
	 if (e.err != NodeAlreadyExists)
		  qFatal("%s", e.text.c_str());
  }
  catch (...)
  {
	 qFatal("Can't connect to RCIS: %s:%d\n", __FILE__, __LINE__);
  }
}

void SpecificWorker::RCIS_removeNode_nonexistingok(std::string nodeName)
{
	try
	{
// 		innermodelmanager_proxy->removeNode(nodeName);
		innermodelmanager_proxy->removeNode(nodeName + "_T");
		printf("DELETED: %s", nodeName.c_str());
	}
	catch (RoboCompInnerModelManager::InnerModelManagerError e)
	{
			if (e.err != RoboCompInnerModelManager::NonExistingNode)
				qFatal("%s", e.text.c_str());
	}
	catch (...)
	{
		qFatal("Can't connect to RCIS: %s:%d\n", __FILE__, __LINE__);
	}
}

//Update object position
void SpecificWorker::RCIS_update_object(RoboCompAGMWorldModel::Node &node)
{
	try
	{
	   RoboCompInnerModelManager::Pose3D pose;

 		pose.rx = str2float(node.attributes["rx"]);
 		pose.ry = str2float(node.attributes["ry"]);
		pose.rz = str2float(node.attributes["rz"]);
 		pose.x = str2float(node.attributes[std::string("tx") ]);
 		pose.y = str2float(node.attributes[std::string("ty") ]);
 		pose.z = str2float(node.attributes[std::string("tz") ]);
		
		
		printf("Rx: %s\n",std::string(node.attributes[std::string("rx")]).c_str() );
		printf("Ry: %s\n",std::string(node.attributes[std::string("ry")]).c_str());
		printf("Rz: %s\n",std::string(node.attributes[std::string("rz")]).c_str());
		printf("Tx: %s\n",std::string(node.attributes[std::string("tx")]).c_str());
		printf("Ty: %s\n",std::string(node.attributes[std::string("ty")] ).c_str());
		printf("Tz: %s\n",std::string(node.attributes[std::string("tz")]).c_str());
		
		QString ident = QString::fromStdString(node.nodeType);
		ident += "_";
		ident += QString::number(node.nodeIdentifier);
		
		printf("UPDATED: %s", ident.toStdString().c_str());
		
		innermodelmanager_proxy->setPoseFromParent(ident.toStdString()+"_T", pose);
		//innermodelmanager_proxy->setPoseFromParent("test_T", pose);
		printf("UPDATED: %s", ident.toStdString().c_str());
	}
	catch (RoboCompInnerModelManager::InnerModelManagerError e)
	{
			if (e.err != RoboCompInnerModelManager::NonExistingNode)
				qFatal("%s", e.text.c_str());
	}
	catch (...)
	{
		printf("SHIT %s:%d\n", __FILE__, __LINE__);
	}
}