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
 * \brief Default constructor. It initializes the attributes of the ursusCommonJoint component
 * @param mprx
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
	state = Idle;
	transitionSteps.clear();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
/**
 * \brief This method initializes the variables with the config params.
 * @param params list of config params
 * @return boolean
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(mutex);
	timer.start(Period);
	try
	{
		innerModel = new InnerModel(params["InnerModel"].value);
	}catch(std::exception e) { qFatal("Error reading Innermodel");}
	
	init(); //initializes the Faulhaber and Dynamixel motors
#ifdef USE_QTGUI
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
// 	show();
#endif

	std::vector<std::pair<QString, QString> > exclusionList; //lista de exclusion de meshes
	QString exclusion = QString::fromStdString(params["ExclusionList"].value);
 	if (exclusion.size()<2)
 	{
 		qFatal("Couldn't read ExclusionList\n");
 		return false;
 	}
 	for (auto parejaTexto : exclusion.split(";", QString::SkipEmptyParts))
 	{
 		QStringList parejaLista = parejaTexto.split(",");
		qDebug()<<parejaLista;
 		exclusionList.push_back(std::pair<QString, QString>(parejaLista[0], parejaLista[1]));
 		exclusionList.push_back(std::pair<QString, QString>(parejaLista[1], parejaLista[0]));
 	}
/* 	printf("ExclusionList content:\n");
 	for (auto e: exclusionList)
 	{
 		printf("%s %s\n", e.first.toStdString().c_str(), e.second.toStdString().c_str());
 	}
*/ 	

	std::vector<QString> meshes;
	recursiveIncludeMeshes(innerModel->getNode("robot"), meshes); //recoge los meshes del xml
	std::sort(meshes.begin(), meshes.end()); //ordenamos ascendentemente
	for (auto a: meshes)
	{
		if (innerModel->collidable(a)) //si el mesh A no puede chocarse con nada
		{
			for (auto b: meshes)
			{
				if (innerModel->collidable(b)) //si el mesh B no puede chocarse con nada
				{
					if (a>b and (std::find(exclusionList.begin(), exclusionList.end(), std::pair<QString, QString>(a, b)) == exclusionList.end()))
					{
						pairs.push_back(std::pair<QString,QString>(a, b)); //guardamos pareja que no puede chocarse
						//printf("pair: %s - %s\n", a.toStdString().c_str(), b.toStdString().c_str());
					}
				}
			}
		}
	}
	// read collision bounding boxes restrictions
	QString boxes = QString::fromStdString(params["RestrictedBoundingBoxes"].value);
	printf("Collision bounding boxes\n");
	for (auto boxValues : boxes.split(";", QString::SkipEmptyParts))
 	{
 		QStringList voxPoints = boxValues.split(",");
		if (voxPoints.size() != 7){
			qFatal("Bounding boxes collision errors");
			return false;
		}
		qDebug()<<voxPoints;
		QVec a1 = QVec::vec3(voxPoints[1].toFloat(),voxPoints[2].toFloat(),voxPoints[3].toFloat());
		QVec a2 = QVec::vec3(voxPoints[4].toFloat(),voxPoints[5].toFloat(),voxPoints[6].toFloat());
		collisionBoxMap[voxPoints[0]] = std::pair<QVec,QVec>(a1,a2);
	}
	//NOTE cuidado al meter motores nuevos a pelo: los xml no coinciden
	//NOTE Movemos a pelo la cabeza hacia abajo... se puede quitar para despues
	std::vector<std::pair<std::string, float> > initializations = { {"head_pitch_joint",0.8}, {"head_yaw_joint",0} };
	MotorGoalPosition goal;
	for (auto init : initializations)
	{
		QMutexLocker locker(mutex);
		std::pair<QString, QString> ret;
		goal.position = init.second;
		goal.name = init.first;
		goal.maxSpeed = 0.5;
		try { prxMap.at(init.first)->setPosition(goal); }
		catch (std::exception &ex) { std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << goal.name << std::endl; };
	}

	// read known positions and transitions
	printf("TransitionList content:\n");
	QString transitions = QString::fromStdString(params["knownTransitions"].value);
 	for (auto parejaTexto : transitions.split(";", QString::SkipEmptyParts))
 	{
 		QStringList parejaLista = parejaTexto.split(":" , QString::SkipEmptyParts);
		QStringList init_final = parejaLista[0].split("," , QString::SkipEmptyParts);
		QStringList intermediates = parejaLista[1].split("," , QString::SkipEmptyParts);
		std::pair<QString, QString> key = std::pair<QString,QString>(init_final[0], init_final[1]);
		std::vector<QString> intermediate_list;
		for (auto intermdiate_name: intermediates)
			 intermediate_list.push_back(intermdiate_name);
		knownTransitions.insert(std::pair<std::pair<QString,QString>,std::vector<QString>>(key,intermediate_list));
		qDebug()<<"\t transition: "<<init_final[0] << ":"<<init_final[1] <<" => "<<intermediates;
 	}
 	QString positions = QString::fromStdString(params["knownPositions"].value);
 	for (auto parejaTexto : positions.split(";", QString::SkipEmptyParts))
 	{
		qDebug()<<"position name =>"<<parejaTexto;
		std::map<QString, float> motorList;
 		QStringList parejaLista = parejaTexto.split("," , QString::SkipEmptyParts);
		QStringList motors = QString(parejaLista[1]).remove(QChar('[')).remove(QChar(']')).split("_" , QString::SkipEmptyParts);
		for (auto motor : motors)
		{
			QStringList motor_values = motor.split(":", QString::SkipEmptyParts);
			motorList.insert(std::pair<QString, float>(motor_values[0],motor_values[1].toFloat()));
//			qDebug()<<motor_values[0] << "=>"<<motor_values[1];
		}
		knownPositions.insert(std::pair<QString,std::map<QString,float> >(parejaLista[0], motorList));
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief principal SLOT
 * This updates the innerModel and the innerViewer
 */ 
void SpecificWorker::compute( )
{
	usleep(50000);
	// Actualizamos el innerModel y la ventana del viewer
	QMutexLocker locker(mutex);
	RoboCompJointMotor::MotorStateMap mMap;
	motorStateMap.clear();
	try
	{
		jointmotor0_proxy->getAllMotorState(mMap); 
		motorStateMap.insert(mMap.begin(), mMap.end());
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción en actualizar InnerModel: 0 (DYNAMIXEL)\n" << ex.what();
	}
	for (auto j : mMap)
	{
		try
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		}
		catch (...)
		{
			cout << "--> exception uptading " << j.first << endl;
		}
	}
	try
	{
		jointmotor1_proxy->getAllMotorState(mMap);
		motorStateMap.insert(mMap.begin(), mMap.end());
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción en actualizar InnerModel: 1 (FAULHABER)\n" << ex.what();
	}
	for (auto j : mMap)
	{
		try
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		}
		catch (...)
		{
			cout << "--> exception uptading " << j.first << endl;
		}
	}

#ifdef USE_QTGUI
	if (imv) imv->update();
	osgView->frame();
#endif
	
	switch(state)
	{
		case Idle:
			break;
		case GoPos:
			qDebug()<<"goPos ";
			// check motors are moving
			for (auto motor: motorStateMap)
			{
				if (motor.second.isMoving)
				{
					qDebug()<<"waiting motor " <<QString::fromStdString(motor.first) << " is moving";
					return;
				}
			}
			if (not transitionSteps.isEmpty())
			{
				QString next = transitionSteps.takeFirst();
				qDebug() << "next step : " <<next;
				RoboCompJointMotor::MotorGoalPositionList goalList = convertKnownPos2Goal(next);
				sendPos2Motors(goalList);
			}
			else
			{
				state = Idle;
			}
			break;
	}
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
/// SERVANT METHODS ////////////////////////////////////////////////////////////////
/**
 * \brief this method moves the motors to the goal angles.
 * NOTE NOW IT CHECKS THE COLLISION BETWEEN THE MESHES. IF ANYONE COLLIDING, THEN IT THROWS AN EXCEPTION
 * @param goal the target angle for one motor
 */ 
void SpecificWorker::setPosition(const MotorGoalPosition &goal)
{
	QMutexLocker locker(mutex);
	MotorGoalPositionList listGoals;
	listGoals.push_back(goal); //guardamos el angulo objetivo para un motor
	std::pair<QString, QString> ret;
	std::string result;
	if (checkMovementNeeded(listGoals))
	{
		if (checkFuturePosition(listGoals, ret))
		{
			//Si la comprobacion de choque devuelve TRUE, entonces hay colision.
			//Lanzamos la excepcion y NO movemos motores. NOTE para desactivar comente el throw exception
			printf("|| setPosition: %s with %s\n", ret.first.toStdString().c_str(), ret.second.toStdString().c_str());
			throw RoboCompJointMotor::CollisionException("collision between "+ret.first.toStdString()+" and "+ret.second.toStdString());
		}
		if (checkMotorLimits(listGoals, result))
		{
			printf("|| setPosition: %s\n", result.c_str());
			throw RoboCompJointMotor::OutOfRangeException(result);
		}
		//movemos el motor:
		try
		{ 
			prxMap.at(goal.name)->setPosition(goal);
		}catch (std::exception &ex) 
		{ 
			std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << goal.name << std::endl; 
		}
	}
}
/**
 * \brief this method changes the speed of the motor.
 * @param goal the new speed of the motor
 */ 
void SpecificWorker::setVelocity(const MotorGoalVelocity& goal)
{
	QMutexLocker locker(mutex);
	try { prxMap.at(goal.name)->setVelocity(goal); }
	catch (std::exception &ex) { std::cout << ex.what() << __FILE__ << __LINE__ << "Motor " << goal.name << std::endl; };
}
/**
 * \brief this method moves the motor to its Zero position
 * @param name the name of the motor
 */ 
void SpecificWorker::setZeroPos(const string& name)
{
	QMutexLocker locker(mutex);
	try { prxMap.at(name)->setZeroPos(name); }
	catch (std::exception &ex) { std::cout<<ex.what()<<__FILE__<<__LINE__<<"Motor "<<name<<" not found in proxy list"<<std::endl;};
}
/**
 * \brief this method moves more than one motor at the same time.
 * @param listGoals motor (with target angles) list
 */ 
void SpecificWorker::setSyncPosition(const MotorGoalPositionList& listGoals)
{
	QMutexLocker locker(mutex);
	if (state != Idle)
	{
		printf("Going to known position, ignoring actual listGoals\n");
		return;
	}
	std::pair<QString, QString> ret;
	std::string result;
	if (checkMovementNeeded(listGoals))
	{
		if (checkMotorLimits(listGoals, result))
		{
			printf("|| setPosition: %s\n", result.c_str());
			throw RoboCompJointMotor::OutOfRangeException(result);
		}
		else
		{ // inside motor limits
			if (checkFuturePosition(listGoals, ret)) 
			{ //collision
				//Try to avoid using known positions
				QString goal_name = isKnownPosition(listGoals);
				QString actual_name = isKnownPosition(motorStateMap);
				qDebug()<<"result goal "<<goal_name << " result actual "<<actual_name;
				
				// search position in known list
				std::map< std::pair<QString, QString>, std::vector<QString> >::iterator it;
				it = knownTransitions.find(std::pair<QString, QString>(actual_name,goal_name));
				if (it != knownTransitions.end())
				{
					for (auto pos: it->second)
					{
						transitionSteps.append(pos);
					}
					transitionSteps.append(goal_name);
					qDebug()<<"Transition list needed: "<<transitionSteps;
					state = GoPos;
				}
				else
				{
					printf("|| setSyncPosition: %s,%s\n Unkown transition chain\n", ret.first.toStdString().c_str(), ret.second.toStdString().c_str());
					throw RoboCompJointMotor::CollisionException("collision between "+ret.first.toStdString()+" and "+ret.second.toStdString());
				}
			}
			else
			{ //no collision
				sendPos2Motors(listGoals);
			}
		}
	}
}

void SpecificWorker::sendPos2Motors(const RoboCompJointMotor::MotorGoalPositionList &listGoals)
{
	RoboCompJointMotor::MotorGoalPositionList l0,l1;
	for (uint i=0; i<listGoals.size(); i++)
	{
		if (prxMap.at(listGoals[i].name)==jointmotor0_proxy)
			l0.push_back(listGoals[i]);
		else if (prxMap.at( listGoals[i].name)==jointmotor1_proxy)
			l1.push_back(listGoals[i]);
		else
			std::cout<<__FILE__<<__FUNCTION__<<__LINE__<<"Motor "<<listGoals[i].name<<" not found in proxy list\n";
	}
	try{ 
		jointmotor0_proxy->setSyncPosition(l0); 
	}
	catch(std::exception &ex)
	{
		std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in setSyncPosition prx 0"<<std::endl;
	}
	try{
		jointmotor1_proxy->setSyncPosition(l1); 
	}
	catch(std::exception &ex){
		std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in setSyncPosition prx 1"<<std::endl;
	}
}


/**
 * \brief this method changes more than one motor speed at the same time.
 * @param listGoals list of target speeds
 */ 
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
			std::cout<<__FILE__<<__FUNCTION__<<__LINE__<<"Motor "<<listGoals[i].name<<" not found in initial proxy list\n";
	}
	try { jointmotor0_proxy->setSyncVelocity(l0); }
	catch(std::exception &ex) 
	{std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in setSyncVelocity to Dynamixel"<<std::endl;};
	try { jointmotor1_proxy->setSyncVelocity(l1); }
	catch(std::exception &ex) 
	{std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in setSyncPosition to Faulhaber"<<std::endl; };
}
/**
 * \brief TODO
 */ 
void SpecificWorker::setSyncZeroPos()
{
}
/**
 * \brief returns the params os one motor
 * @param motor the name of the motor.
 * @return MotorParams.
 */ 
MotorParams SpecificWorker::getMotorParams(const string& motor)
{
	QMutexLocker locker(mutex);
	MotorParams mp;
	try { mp = prxMap.at(motor)->getMotorParams(motor); }
	catch(std::exception &ex)
	{std::cout<< ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Motor "<<motor<<" not found in initial proxy list"<<std::endl; };
	return mp;
}
/**
 * \brief this method returns the state of one motor.
 * @param motor the name of the motos.
 * @return MotorState
 */ 
MotorState SpecificWorker::getMotorState(const string& motor)
{
	QMutexLocker locker(mutex);
	MotorState ms;
	try { ms = prxMap.at(motor)->getMotorState(motor); }
	catch(std::exception &ex) 
	{std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Motor "<<motor<<" not found in initial proxy list"<<std::endl; };
	return ms;
}
/**
 * \brief this method returns the state of several motors.
 * @param mList list of motor names.
 * @return MotorStateMap
 */ 
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
			std::cout<<__FILE__<<__FUNCTION__<<__LINE__<<"Motor "<<mList[i]<<" not found in initial proxy list\n";
		}
	}
	try{ m0 = jointmotor0_proxy->getMotorStateMap(l0);}
	catch(std::exception &ex) 
	{std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in getMotorStateMap to Dynamixel"<<std::endl; }
	try{ m1 = jointmotor1_proxy->getMotorStateMap(l1);}
	catch(std::exception &ex) 
	{std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error in setSyncPosition to Faulhaber"<<std::endl; }

	m0.insert(m1.begin(), m1.end());
	return m0;
}
/**
 * \brief this method returns the state of all motors of the robot.
 * @param mstateMap the map where it stores the states of all motors.
 */ 
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
			std::cout<<"Error reading motor bus 1\n";
			std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error reading MotorStateMap from Faulhaber bus"<<std::endl;
		}
	}
	catch(std::exception &ex)
	{
		std::cout<<"Error reading motor bus 0\n";
		std::cout<<ex.what()<<__FILE__<<__FUNCTION__<<__LINE__<<"Error reading MotorStateMap from Dynamixel bus"<<std::endl;
	}
}
/**
 * \brief this method returns all the params of all the motors.
 * @return MotorParamsList
 */ 
MotorParamsList SpecificWorker::getAllMotorParams()
{
	QMutexLocker locker(mutex);
	return motorParamList;
}

/**
 * \brief this method returns the params of the motors bus.
 * @return BusParams
 */ 
BusParams SpecificWorker::getBusParams()
{
	QMutexLocker locker(mutex);

	RoboCompJointMotor::BusParams bus;
	std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Not implemented" << std::endl;
	return bus;
}


/**
 * \brief This initializes the Dynamixel bus and the Faulhaber bus.
 */ 
void SpecificWorker::init()
{
	QMutexLocker locker(mutex);
	MotorParamsList par1, par0;
	motorParamMap.clear();
	motorParamList.clear();
	//DYNAMIXEL bus
	try
	{
		par0 = jointmotor0_proxy->getAllMotorParams();
		std::pair<std::map< std::string, RoboCompJointMotor::JointMotorPrx >:: iterator, bool> ret;
		for (uint i=0; i<par0.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(par0[i].name, jointmotor0_proxy));
			motorParamMap.insert(std::pair<std::string,RoboCompJointMotor::MotorParams>(par0[i].name, par0[i]));
			motorParamList.push_back(par0[i]);
			if (ret.second == false)
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << ret.first->second << " already exists" << endl;
				qFatal("Name %s already exists\n", par0[i].name.c_str());
			}
		}
	}catch(const Ice::Exception &ex)
	{std::cout << __FUNCTION__ << __FUNCTION__ << __LINE__ << "Error communicating with jointmotor0_proxy " << ex << endl;};
	//FAULHABER bus
	try
	{
		par1 = jointmotor1_proxy->getAllMotorParams();
		std::pair<std::map< std::string, RoboCompJointMotor::JointMotorPrx >:: iterator, bool> ret;
		for (uint i=0; i<par1.size(); i++)
		{
			ret = prxMap.insert(std::pair<std::string, RoboCompJointMotor::JointMotorPrx>(par1[i].name, jointmotor1_proxy));
			motorParamMap.insert(std::pair<std::string,RoboCompJointMotor::MotorParams>(par1[i].name, par1[i]));
			motorParamList.push_back(par1[i]);
			if( ret.second == false )
			{
				std::cout << __FILE__ << __FUNCTION__ << __LINE__ << "Name " << par1[i].name.c_str() << " already exists" << endl;
				qFatal("Name %s already exists\n", par1[i].name.c_str());
			}
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << __FUNCTION__ << "Error communicating with jointmotor1_proxy " << ex <<std::endl;
		
	}
}

bool SpecificWorker::checkMovementNeeded(const MotorGoalPositionList &goals)
{
	QMutexLocker locker(mutex);
	bool needed = false;
	for (uint i=0; i<goals.size(); i++)
	{
		if(fabs(goals[i].position - innerModel->getJoint(goals[i].name)->getAngle()) > 0.01)
		{
			needed = true;
			break;
		}
	}
	return needed;
}

bool SpecificWorker::checkMotorLimits(const MotorGoalPositionList &goals, std::string &ret)
{
	QMutexLocker locker(mutex);
	for (uint i=0; i<goals.size(); i++)
	{
		if ( goals[i].position < motorParamMap.at(goals[i].name).minPos || goals[i].position > motorParamMap.at(goals[i].name).maxPos ){
			ret = "Goal position outside motor limit, " +  goals[i].name + " => pos: " + std::to_string(goals[i].position) + " range[" + std::to_string(motorParamMap.at(goals[i].name).minPos) + "," + std::to_string(motorParamMap.at(goals[i].name).maxPos) + "]"; 
			return true;
		}
	}
	return false;
}
/**
 * \brief This method checks the final state in which the robot would stay if we move its motors.
 * @param goals list of taget poses of the motors
 * @param ret pair of meshes that colliding between each other
 * @return boolean TRUE if there is a collision and FALSE if there is not a collision.
 */ 
bool SpecificWorker::checkFuturePosition(MotorGoalPositionList goals, std::pair<QString, QString> &ret)
{
	QMutexLocker locker(mutex);
	MotorGoalPositionList backPoses = goals; //guardamos nombres de los motores
	//Guardamos para cada joint su angulo actual 
	float position = 0.f,actual_position=0.f, iter_move = 0.f;
	float range = 0.f, time = 0.f,iter_time = 0.f;
	for (uint i=0; i<goals.size(); i++)
	{
//		qDebug()<< "max speed motor "<< i << goals[i].maxSpeed << "inverse" <<motorParamMap[goals[i].name].invertedSign;
		// If maxSpeed not provided, is taken from motor param
		if ( isnan(goals[i].maxSpeed) || goals[i].maxSpeed > motorParamMap[goals[i].name].maxVelocity || goals[i].maxSpeed <= 0)
		{
			goals[i].maxSpeed = motorParamMap[goals[i].name].maxVelocity;
//			qDebug() << "truncate vel"<< goals[i].maxSpeed;
		}
		backPoses[i].position = innerModel->getJoint(backPoses[i].name)->getAngle();
		if( (fabs(goals[i].position - backPoses[i].position) / goals[i].maxSpeed ) > time )
		{
			range = fabs(goals[i].position - backPoses[i].position);
			time = (range / goals[i].maxSpeed);
		}
//		qDebug()<< "motor: "<< i <<"range: "<<backPoses[i].position << goals[i].position << fabs(goals[i].position - backPoses[i].position) << "time" << (fabs(goals[i].position - backPoses[i].position) / goals[i].maxSpeed);
	}
	bool collision = false;
	uint num_iter = round(range / MAX_MOVE);
	if (num_iter == 0)	//at least one iteration is needed
		num_iter = 1;
	iter_time = time / num_iter;
//	qDebug()<<"time" <<time << num_iter << iter_time;

	for (uint j=0;j<num_iter;j++)
	{
		// Insert objective angle into innermodel
		for (uint i=0; i<goals.size(); i++)
		{
			actual_position = innerModel->getJoint(backPoses[i].name)->getAngle();
			int direction = 1;
			if (goals[i].position < backPoses[i].position)
				direction = -1;
			iter_move = (goals[i].maxSpeed * iter_time);
			if (iter_move < fabs(goals[i].position - actual_position))
				position = actual_position + iter_move*direction;
			else
				position = goals[i].position;
			
//			qDebug()<<"iter " << j << "motor: "<< i <<" max position "<<goals[i].position << position;
			innerModel->getJoint(backPoses[i].name)->setAngle(position, true);
			// check collision
			innerModel->cleanupTables(); 
			for (auto p: pairs)
			{
				//if (innerModel->collide(p.first, p.second))
				if (innerModel->distance(p.first, p.second)< 100) //Using minimum distance instead of collision flag
				{
					ret = p;
					collision = true;
					break;
				}
			}
		}
	}
	if(not collision) 
	{
		//check not inside bounding box
		for (auto box: collisionBoxMap)
		{
	//		printf("node %s\n",box.first.toStdString().c_str());
			QVec point = innerModel->transform("robot",box.first.toStdString().c_str());
			QVec a = box.second.first;
			QVec b = box.second.second;
			if((a(0) < point(0) && point(0) < b(0)) && (a(1) < point(1) && point(1) < b(1))  && (a(2) < point(2) && point(2) < b(2))){
				collision = true;
				ret = std::pair<QString, QString>("Collision box",box.first);
				break;
			}
		}
	}
	
	// If there is a collision, all joints return to previous position
	if (collision)
	{
		for (uint i=0; i<goals.size(); i++)
			innerModel->getJoint(backPoses[i].name)->setAngle(backPoses[i].position, true);
		
		innerModel->cleanupTables();
		printf("|| checkFuturePosition: %s with %s\n", ret.first.toStdString().c_str(), ret.second.toStdString().c_str());
	}
	return collision;
}
/**
 * \brief this method checks the meshes from one node of the innerModel
 * @param node
 * @param in
 */ 
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


//check if position is known
// position is similar if all motor involved are 0.1 or closer 
//return position name if found "none "otherwise
QString SpecificWorker::isKnownPosition(RoboCompJointMotor::MotorStateMap mstate)
{
	// convert MotorStateMap to MotorGoalPositionList
	RoboCompJointMotor::MotorGoalPositionList goalList;
	for (auto state: mstate)
	{
		RoboCompJointMotor::MotorGoalPosition goal;
		goal.name = state.first;
		goal.position = state.second.pos;
		goalList.push_back(goal);
	}
	return isKnownPosition(goalList);
	
}
QString SpecificWorker::isKnownPosition(RoboCompJointMotor::MotorGoalPositionList goals)
{
	bool nextPosition = false;
	for (auto position: knownPositions)
	{
//		qDebug()<<"check position "<< position.first;
		uint cont = 0;
		for (auto motor: position.second)
		{
			for (uint i=0; i<goals.size(); i++)
			{
				if ( motor.first == QString::fromStdString(goals[i].name))
				{
					if (fabs(motor.second-goals[i].position) <= POS_OFFSET)
					{
						cont++;
						continue;
					}
					else
					{
						nextPosition = true;
						break;
					}
				}
			}
			if(nextPosition)
			{
				nextPosition = false;
				break;
			}
		}
		if (cont == position.second.size())
		{
			return position.first;
		}
	}
	return "ik";
}

RoboCompJointMotor::MotorGoalPositionList SpecificWorker::convertKnownPos2Goal(QString pos_name)
{
	RoboCompJointMotor::MotorGoalPositionList goalList;
	std::map<QString, std::map<QString, float> >::iterator it;
	it = knownPositions.find(pos_name);
//	qDebug()<<"pos to motor values ";
	if (it != knownPositions.end())
	{
		for (auto motor: it->second)
		{
			RoboCompJointMotor::MotorGoalPosition goal;
			goal.name = motor.first.toStdString();
			goal.position = motor.second;
			goalList.push_back(goal);	
//			qDebug()<<"motor found"<<motor.first <<goal.position;
		}
	}
	return goalList;
}
