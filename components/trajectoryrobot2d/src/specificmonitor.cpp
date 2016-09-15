/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{
		ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{

}

void SpecificMonitor::run()
{
	initialize();
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
		this->sleep(period);
	}
}

/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 *   (1) Ice parameters
 *   (2) Local component parameters read at start
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	readPConfParams(params);
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::Running;
}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList& params )
{
	RoboCompCommonBehavior::Parameter aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","InnerModel", aux.value,"/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");
	params["InnerModel"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","ArrivalTolerance", aux.value,"20");
	params["ArrivalTolerance"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","MaxZSpeed", aux.value,"400");
	params["MaxZSpeed"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","MaxXSpeed", aux.value,"200");
	params["MaxXSpeed"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","MaxRotationSpeed", aux.value,"0.3");
	params["MaxRotationSpeed"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","RobotXWidth", aux.value,"500");
	params["RobotXWidth"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","RobotZLong", aux.value,"500");
	params["RobotZLong"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","RobotRadius", aux.value,"300");
	params["RobotRadius"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","MinControllerPeriod", aux.value,"100");
	params["MinControllerPeriod"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","PlannerGraphPoints", aux.value,"100");
	params["PlannerGraphPoints"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","PlannerGraphNeighbours", aux.value,"20");
	params["PlannerGraphNeighbours"] = aux;

	aux.editable = false;
	configGetString( "TrajectoryRobot2D","PlannerGraphMaxDistanceToSearch", aux.value,"2500");
	params["PlannerGraphMaxDistanceToSearch"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","OuterRegionLeft", aux.value,"0");
	params["OuterRegionLeft"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","OuterRegionRight", aux.value,"6000");
	params["OuterRegionRight"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","OuterRegionBottom", aux.value,"-4250");
	params["OuterRegionBottom"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","OuterRegionTop", aux.value,"4250");
	params["OuterRegionTop"] = aux;
	
	aux.editable = false;
	configGetString( "TrajectoryRobot2D","ExcludedObjectsInCollisionCheck", aux.value,"floor_plane");
	params["ExcludedObjectsInCollisionCheck"] = aux;
}


//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	
	//Check InnerModel exists
	std::ifstream f(l["InnerModel"].value);
	if(f.good() == false)
		correct = false;
	
	//copy parameters
	if(correct)
 		config_params = l;
	return correct;
}

bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		if(worker->setParams(params)) 
			return true;
	}
	else
	{
		rError("Incorrect parameters");
	}
	return false;

}

