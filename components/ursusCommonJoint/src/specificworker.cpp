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

SpecificWorker::SpecificWorker(MapPrx & mprx) : GenericWorker(mprx)
{
#ifdef USE_QTGUI
	imv = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
#endif
	mutex = new QMutex(QMutex::Recursive);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	usleep(50000);
	// Actualizamos el innerModel y la ventada del viewer
	QMutexLocker locker(mutex);
	if (imv) imv->update();
	osgView->frame();
}


void SpecificWorker::init()
{
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);

	QMutexLocker locker(mutex);

	//Dynamixel bus
	try
	{
		motorList0 = jointmotor0_proxy->getAllMotorParams();
		std::pair<std::map< std::string, RoboCompJointMotor::JointMotorPrx >:: iterator, bool> ret;
		for (uint i=0; i<motorList0.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(motorList0[i].name, jointmotor0_proxy));
			if (ret.second == false)
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << ret.first->second << " already exists" << endl;
				qFatal("Name %s already exists\n", motorList0[i].name.c_str());
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
		for (uint i=0; i<motorList1.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(motorList1[i].name, jointmotor1_proxy));
			if( ret.second == false )
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << motorList1[i].name.c_str() << " already exists" << endl;
				qFatal("Name %s already exists\n", motorList1[i].name.c_str());
			}
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout << __FUNCTION__ << "Error communicating with jointmotor1_proxy " << ex << endl;
	};
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(mutex);

	timer.start(Period);

	innerModel = new InnerModel(params["InnerModel"].value);
	init();
#ifdef USE_QTGUI
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
	show();
#endif

	std::vector<std::pair<QString, QString> > exclusionList;
	QString exclusion = QString::fromStdString(params["ExclusionList"].value);
	if (exclusion.size()<2)
	{
		qFatal("Couldn't read ExclusionList\n");
		return false;
	}	
	for (auto parejaTexto : exclusion.split(";", QString::SkipEmptyParts))
	{
		QStringList parejaLista = parejaTexto.split(",");
		exclusionList.push_back(std::pair<QString, QString>(parejaLista[0], parejaLista[1]));
		exclusionList.push_back(std::pair<QString, QString>(parejaLista[1], parejaLista[0]));
	}
	for (auto e: exclusionList)
	{
		printf("%s %s\n", e.first.toStdString().c_str(), e.second.toStdString().c_str());
	}


	std::vector<QString> meshes;
	recursiveIncludeMeshes(innerModel->getNode("robot"), meshes);
	std::sort(meshes.begin(), meshes.end());

	for (auto a: meshes)
	{
		if (innerModel->collidable(a))
		{
			for (auto b: meshes)
			{
				if (innerModel->collidable(b))
				{
					if (a>b and (std::find(exclusionList.begin(), exclusionList.end(), std::pair<QString, QString>(a, b)) == exclusionList.end()))
					{
						pairs.push_back(std::pair<QString,QString>(a, b));
						printf("pair: %s - %s\n", a.toStdString().c_str(), b.toStdString().c_str());
					}
				}
			}
		}
	}

	
	return true;
}

/// SERVANT METHODS /////////////////////////////////////////////////////////////////////7
bool SpecificWorker::checkFuturePosition(const MotorGoalPositionList &goals, std::pair<QString, QString> &ret)
{
	QMutexLocker locker(mutex);

	MotorGoalPositionList backPoses = goals;
	for (uint i=0; i<goals.size(); i++)
	{
		backPoses[i].position = innerModel->getJoint(backPoses[i].name)->getAngle();
	}

	for (uint i=0; i<goals.size(); i++)
	{
		innerModel->getJoint(backPoses[i].name)->setAngle(goals[i].position);
	}

	innerModel->cleanupTables();
	for (auto p: pairs)
	{
		if (innerModel->collide(p.first, p.second))
		{
			for (uint i=0; i<goals.size(); i++)
			{
				innerModel->getJoint(backPoses[i].name)->setAngle(backPoses[i].position);
			}
			innerModel->cleanupTables();
			ret = p;
			printf("%s %s\n", p.first.toStdString().c_str(), p.second.toStdString().c_str());
			return true;
		}
	}

	return false;
}

void SpecificWorker::setPosition(const MotorGoalPosition &goal)
{
	QMutexLocker locker(mutex);

	MotorGoalPositionList listGoals;
	listGoals.push_back(goal);
	std::pair<QString, QString> ret;
	if (checkFuturePosition(listGoals, ret))
	{
		printf("%s,%s\n", ret.first.toStdString().c_str(), ret.second.toStdString().c_str());
		throw RoboCompJointMotor::OutOfRangeException("collision");
	}
	try { prxMap.at(goal.name)->setPosition(goal); }
	catch (std::exception &ex) { std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << goal.name << std::endl; };
}

void SpecificWorker::setVelocity(const MotorGoalVelocity& goal)
{
	QMutexLocker locker(mutex);

	try { prxMap.at(goal.name)->setVelocity(goal); }
	catch (std::exception &ex) { std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << goal.name << std::endl; };
}

void SpecificWorker::setZeroPos(const string& name)
{
	QMutexLocker locker(mutex);

	try { prxMap.at(name)->setZeroPos(name); }
	catch (std::exception &ex) { std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << name << " not found in proxy list" << std::endl; };
}

void SpecificWorker::setSyncPosition(const MotorGoalPositionList& listGoals)
{
	QMutexLocker locker(mutex);

	std::pair<QString, QString> ret;
	if (checkFuturePosition(listGoals, ret))
	{
		printf("%s,%s\n", ret.first.toStdString().c_str(), ret.second.toStdString().c_str());
		throw RoboCompJointMotor::OutOfRangeException("collision");
	}
	RoboCompJointMotor::MotorGoalPositionList l0,l1;
	for (uint i=0; i<listGoals.size(); i++)
	{
		if (prxMap.at(listGoals[i].name)==jointmotor0_proxy)
			l0.push_back(listGoals[i]);
		else if (prxMap.at( listGoals[i].name)==jointmotor1_proxy)
			l1.push_back(listGoals[i]);
		else
			std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << listGoals[i].name << " not found in proxy list\n";
	}
	try { jointmotor0_proxy->setSyncPosition(l0); }
	catch(std::exception &ex) { std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition prx 0" << std::endl; };
	try { jointmotor1_proxy->setSyncPosition(l1); }
	catch(std::exception &ex) {std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition prx 1" << std::endl; };
}

void SpecificWorker::setSyncVelocity(const MotorGoalVelocityList& listGoals)
{
	QMutexLocker locker(mutex);

	RoboCompJointMotor::MotorGoalVelocityList l0,l1;
	for (uint i=0;i<listGoals.size();i++)
	{
		if (prxMap.at( listGoals[i].name) == jointmotor0_proxy )
			l0.push_back(listGoals[i]);
		else if (prxMap.at( listGoals[i].name) == jointmotor1_proxy )
			l1.push_back(listGoals[i]);
		else 
			std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << listGoals[i].name << " not found in initial proxy list\n";
	}
	try { jointmotor0_proxy->setSyncVelocity(l0); }
	catch(std::exception &ex) { std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncVelocity to Dynamixel" << std::endl; };
	try { jointmotor1_proxy->setSyncVelocity(l1); }
	catch(std::exception &ex) { std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Faulhaber" << std::endl; };
}

void SpecificWorker::setSyncZeroPos()
{
}

MotorParams SpecificWorker::getMotorParams(const string& motor)
{
	QMutexLocker locker(mutex);

	MotorParams mp;
	try { mp = prxMap.at(motor)->getMotorParams(motor); }
	catch(std::exception &ex) { std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << motor << " not found in initial proxy list" << std::endl; };
	return mp;
}

MotorState SpecificWorker::getMotorState(const string& motor)
{
	QMutexLocker locker(mutex);

	MotorState ms;
	try { ms = prxMap.at(motor)->getMotorState(motor); }
	catch(std::exception &ex) { std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << motor << " not found in initial proxy list" << std::endl; };
	return ms;
}

MotorStateMap SpecificWorker::getMotorStateMap(const MotorList& mList)
{
	QMutexLocker locker(mutex);

	MotorList l0,l1;
	MotorStateMap m0, m1;
	
	for (uint i=0; i<mList.size(); i++)
	{
		if (prxMap.at( mList[i]) == jointmotor0_proxy )
		{
			l0.push_back(mList[i]);
		}
		else if (prxMap.at( mList[i]) == jointmotor1_proxy )
		{
			l1.push_back(mList[i]);
		}
		else 
		{
			std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Motor " << mList[i] << " not found in initial proxy list\n";
		}
	}
	

	try
	{
		m0 = jointmotor0_proxy->getMotorStateMap(l0);
	}
	catch(std::exception &ex)
	{
		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in getMotorStateMap to Dynamixel" << std::endl;
	}
	try
	{
		m1 = jointmotor1_proxy->getMotorStateMap(l1);
	}
	catch(std::exception &ex)
	{
		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error in setSyncPosition to Faulhaber" << std::endl;
	}
	m0.insert(m1.begin(), m1.end());

	return m0;
}

void SpecificWorker::getAllMotorState(MotorStateMap& mstateMap)
{
	QMutexLocker locker(mutex);

	MotorStateMap map1;
	try
	{
		jointmotor0_proxy->getAllMotorState(mstateMap);
		try
		{
			jointmotor1_proxy->getAllMotorState(map1);
			mstateMap.insert(map1.begin(), map1.end());
		}
		catch(std::exception &ex)
		{
			std::cout << "Error reading motor bus 1\n";
			std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading MotorStateMap from Faulhaber bus" << std::endl;
		}
	}
	catch(std::exception &ex)
	{
		std::cout << "Error reading motor bus 0\n";
		std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading MotorStateMap from Dynamixel bus" << std::endl;
	}
}

MotorParamsList SpecificWorker::getAllMotorParams()
{
	QMutexLocker locker(mutex);

	MotorParamsList par1, par;

	try
	{ par = jointmotor0_proxy->getAllMotorParams(); }
	catch(std::exception &ex)
	{ std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading getAllMotorParams from Dynamixel bus" << std::endl; };

	try
	{ par1 = jointmotor1_proxy->getAllMotorParams(); }
	catch(std::exception &ex)
	{ std::cout << ex.what() << __FILE__ << __FUNCTION__ << __LINE__ << "Error reading getAllMotorParams from Faulhaber bus" << std::endl; };

	par.insert(par.end(), par1.begin(), par1.end());
	
	return par;
}

BusParams SpecificWorker::getBusParams()
{
	QMutexLocker locker(mutex);

	RoboCompJointMotor::BusParams bus;
	std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Not implemented" << std::endl;
	return bus;
}


void SpecificWorker::recursiveIncludeMeshes(InnerModelNode *node, std::vector<QString> &in)
{
	QMutexLocker locker(mutex);

	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;

	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		for (int i=0; i<node->children.size(); i++)
		{
			recursiveIncludeMeshes(node->children[i], in);
		}
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		in.push_back(node->id);
	}
}


