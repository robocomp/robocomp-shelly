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
#include <boost/graph/graph_concepts.hpp>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	//qDebug() << "hola";
}

void SpecificWorker::init()
{
	//Dynamixel bus
	try
	{
		motorList0 = jointmotor0_proxy->getAllMotorParams();
		std::pair<std::map< std::string, RoboCompJointMotor::JointMotorPrx >:: iterator, bool> ret;
		for(uint i=0; i<motorList0.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(motorList0[i].name, jointmotor0_proxy));
			if( ret.second == false )
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << ret.first->second << " already exists" << cout<<endl;
				qFatal("Fary");
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
			cout << __FUNCTION__ << __FUNCTION__ << __LINE__ << "Error communicating with jointmotor0_proxy " << ex << endl;
	};
	
	//Faulhaber bus
	try
	{
		motorList1 = jointmotor1_proxy->getAllMotorParams();
		std::pair<std::map< std::string, RoboCompJointMotor::JointMotorPrx >:: iterator, bool> ret;
		for(uint i=0; i<motorList1.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(motorList1[i].name, jointmotor1_proxy));
			if( ret.second == false )
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << ret.first->second << " already exists" << cout<<endl;
				qFatal("Fary");
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
			cout << __FUNCTION__ << "Error communicating with jointmotor1_proxy " << ex << endl;
	};
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	
	innerModel = new InnerModel(params["InnerModel"].value);
	init();
	return true;
};

/// SERVANT METHODS /////////////////////////////////////////////////////////////////////7


void SpecificWorker::setPosition(const MotorGoalPosition& goal)
{

	try
	{		prxMap.at(goal.name)->setPosition(goal);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << goal.name << " not found in initial proxy list" << std::endl;	};
	
}
void SpecificWorker::setVelocity(const MotorGoalVelocity& goal)
{
	try
	{		prxMap.at(goal.name)->setVelocity(goal);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << goal.name << " not found in initial proxy list" << std::endl;	};
	
}
void SpecificWorker::setZeroPos(const string& name)
{
	try
	{		prxMap.at(name)->setZeroPos(name);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << name << " not found in initial proxy list" << std::endl;	};
	
}
void SpecificWorker::setSyncPosition(const MotorGoalPositionList& listGoals)
{
	RoboCompJointMotor::MotorGoalPositionList l0,l1;
	for(uint i=0;i<listGoals.size();i++)
	{
			if (prxMap.at( listGoals[i].name) == jointmotor0_proxy )
				l0.push_back(listGoals[i]);
			else if (prxMap.at( listGoals[i].name) == jointmotor1_proxy )
				l1.push_back(listGoals[i]);
			else 
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << listGoals[i].name << " not found in initial proxy list";
	}
	try
	{		jointmotor0_proxy->setSyncPosition(l0);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Dynamixel" << std::endl;	};
	try
	{		jointmotor1_proxy->setSyncPosition(l1);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Faulhaber" << std::endl;	};
	
}
void SpecificWorker::setSyncVelocity(const MotorGoalVelocityList& listGoals)
{
	RoboCompJointMotor::MotorGoalVelocityList l0,l1;
	for(uint i=0;i<listGoals.size();i++)
	{
			if (prxMap.at( listGoals[i].name) == jointmotor0_proxy )
				l0.push_back(listGoals[i]);
			else if (prxMap.at( listGoals[i].name) == jointmotor1_proxy )
				l1.push_back(listGoals[i]);
			else 
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << listGoals[i].name << " not found in initial proxy list";
	}
	try
	{		jointmotor0_proxy->setSyncVelocity(l0);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncVelocity to Dynamixel" << std::endl;	};
	try
	{		jointmotor1_proxy->setSyncVelocity(l1);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Faulhaber" << std::endl;	};

}
void SpecificWorker::setSyncZeroPos()
{
}
MotorParams SpecificWorker::getMotorParams(const string& motor)
{
	MotorParams mp;
	try
	{		mp 	= prxMap.at(motor)->getMotorParams(motor);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << motor << " not found in initial proxy list" << std::endl;	};
	return mp;
}

MotorState SpecificWorker::getMotorState(const string& motor)
{
	MotorState ms;
	try
	{		ms = prxMap.at(motor)->getMotorState(motor);	
	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << motor << " not found in initial proxy list" << std::endl;	};
	return ms;	
}

MotorStateMap SpecificWorker::getMotorStateMap(const MotorList& mList)
{
	MotorList l0,l1;
	MotorStateMap m0, m1;
	
	for(uint i=0;i<mList.size();i++)
	{
		if (prxMap.at( mList[i]) == jointmotor0_proxy )
			l0.push_back(mList[i]);
		else if (prxMap.at( mList[i]) == jointmotor1_proxy )
			l1.push_back(mList[i]);
		else 
			std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << mList[i] << " not found in initial proxy list";
	}
	
	try
	{		m0 = jointmotor0_proxy->getMotorStateMap(l0);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in getMotorStateMap to Dynamixel" << std::endl;	};
	try
	{		m1 = jointmotor1_proxy->getMotorStateMap(l1);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Faulhaber" << std::endl;	};

	m0.insert(m1.begin(), m1.end());
	return m0;
}

void SpecificWorker::getAllMotorState(MotorStateMap& mstateMap)
{
	MotorStateMap map1;
	try
	{		jointmotor0_proxy->getAllMotorState(mstateMap);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading MotorStateMap from Dynamixel bus" << std::endl;	};
	try
	{		jointmotor1_proxy->getAllMotorState(map1);	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading MotorStateMap from Faulhaber bus" << std::endl;	};

	mstateMap.insert(map1.begin(), map1.end());

}
MotorParamsList SpecificWorker::getAllMotorParams()
{
	MotorParamsList par1, par;
	try
	{		par = jointmotor0_proxy->getAllMotorParams();	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading getAllMotorParams from Dynamixel bus" << std::endl;	};
	try
	{		par1 = jointmotor1_proxy->getAllMotorParams();	}
	catch(std::exception &ex)
	{		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading getAllMotorParams from Faulhaber bus" << std::endl;	};

	par.insert(par.end(), par1.begin(), par1.end());
	
	return par;
	
}
BusParams SpecificWorker::getBusParams()
{
		RoboCompJointMotor::BusParams bus;
		std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Not implemented" << std::endl;
		return bus;
}
