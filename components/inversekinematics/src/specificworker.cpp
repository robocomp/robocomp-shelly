/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	inversedkinematic = new InversedKinematic();
	correlativeID = 0;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
/**
 * \brief This method reads the configuration file's params. If the config file doesn't 
 * have all the robot's parts (arm right, arm left and head) we must indicate what parts
 * are available, in order to execute correctly the inverse kinematic.
 * @param params list with all the file's params
 * NOTE: We must modified the readConfig method of the specificmonitor in order to be able
 * of read the params in the config file.
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innermodel = new InnerModel(par.value);
			Metric::moveInnerModelFromMillimetersToMeters(innermodel->getRoot()); /// CONVERT THE METRIC
		}
		else 
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}
		
	QString 	tipR, 				tipL, 		tipH;
	QString 	motorsR, 			motorsL,	motorsH;
	QStringList	auxiliar_motor_list;

	tipR = QString::fromStdString(params["RIGHTTIP"].value);	/// READ THE RIGHT ARM'S TIP
	tipL = QString::fromStdString(params["LEFTTIP"].value);		/// READ THE LEFT ARM'S TIP
	tipH = QString::fromStdString(params["HEADTIP"].value);		/// READ THE HEAD'S TIP
	
	motorsR = QString::fromStdString(params["RIGHTARM"].value);	/// READ THE RIGHT ARM'S MOTORS
	motorsL = QString::fromStdString(params["LEFTARM"].value);	/// READ THE LEFT ARM'S MOTORS
	motorsH = QString::fromStdString(params["HEAD"].value);		/// READ THE HEAD'S MOTORS
					
	if(motorsR.size()>2 and motorsR!="EMPTY" and tipR.size()>2 and tipR!="EMPTY")
	{
		for (auto motor : motorsR.split(";", QString::SkipEmptyParts)) 					/// WE DIVIDE THE STRING BY THE ";"
			auxiliar_motor_list.push_back(motor);
		
		availableParts.push_back("RIGHTARM");
		bodyParts.insert("RIGHTARM", BodyPart("RIGHTARM",tipR, auxiliar_motor_list));	/// PUT THE LIST INTO THE BODY'S PART
		auxiliar_motor_list.clear();
	}
	if(motorsL.size()>2 and motorsL!="EMPTY" and tipL.size()>2 and tipL!="EMPTY")
	{
		for (auto motor : motorsL.split(";", QString::SkipEmptyParts))
			auxiliar_motor_list.push_back(motor);
		
		availableParts.push_back("LEFTARM");
		bodyParts.insert("LEFTARM", BodyPart("LEFTARM",tipL, auxiliar_motor_list));		/// PUT THE LIST INTO THE BODY'S PART
		auxiliar_motor_list.clear();
	}
	if(motorsH.size()>2 and motorsH!="EMPTY" and tipH.size()>2 and tipH!="EMPTY")
	{
		for (auto motor : motorsH.split(";", QString::SkipEmptyParts))
			auxiliar_motor_list.push_back(motor);
		
		availableParts.push_back("HEAD");		
		bodyParts.insert("HEAD", BodyPart("HEAD",tipH, auxiliar_motor_list));			/// PUT THE LIST INTO THE BODY'S PART
	}	
	timer.start(Period);
	return true;
}
/** ÑAPA PARA INICIALIZAR EL OSGVIEWER **/
void SpecificWorker::init()
{
#ifdef USE_QTGUI
	this->osgView = new OsgView(this);
	this->innerViewer = new InnerModelViewer(this->innermodel, "root", this->osgView->getRootGroup(), true);
	show();
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this is the principal SLOT of the component. There is a thread that always 
 * compute this SLOT every (timer) seconds.
 */ 
void SpecificWorker::compute()
{
	static bool primera_vez = true;
	if(primera_vez)
	{
		init();
		primera_vez = false;
	}
	/*---------------- FIN DE ÑAPA --------------*/
	

	updateInnerModel();
		
	QMap<QString, BodyPart>::iterator partsIterator;
	for(partsIterator = bodyParts.begin(); partsIterator != bodyParts.end(); ++partsIterator)
	{
		if(!partsIterator.value().getTargetList().isEmpty())
		{
			cout<<"Hay targets"<<endl;
			
			partsIterator.value().getTargetList()[0].setTargetState(Target::IN_PROCESS);
			
			qDebug()<<partsIterator.value().getTargetList()[0].getTargetState();
						
			createInnerModelTarget(partsIterator.value().getTargetList()[0]);

			inversedkinematic->solveTarget(&partsIterator.value(), innermodel);
		}
	}

	
#ifdef USE_QTGUI
	if (this->innerViewer)
	{
		this->innerViewer->update();
		this->osgView->autoResize();
		this->osgView->frame();
		return;
	}
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method update the angles of each robot's joint and the robot's base.
 */ 
void SpecificWorker::updateInnerModel()
{
	MotorList mList;
	QMap<QString, BodyPart>::iterator partsIterator;
	for(partsIterator = bodyParts.begin(); partsIterator != bodyParts.end(); ++partsIterator)
	{
		int i=0, j=0;
		try
		{
			for (i=0; i<partsIterator.value().getMotorList().size(); i++)
				mList.push_back(partsIterator.value().getMotorList()[i].toStdString());
			
			RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap(mList);

			for (/*int*/ j=0; j<partsIterator.value().getMotorList().size(); j++)
				innermodel->updateJointValue(partsIterator.value().getMotorList()[j], mMap.at(partsIterator.value().getMotorList()[j].toStdString()).pos);
		}
		catch (const Ice::Exception &ex) { cout<<"--> Excepción en actualizar InnerModel: (i,j)"<<": "<<i<<"," <<j<<" "<<ex<<endl;}
	}

	/*try
	{
		RoboCompOmniRobot::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		try
		{
			innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		}
		catch (const Ice::Exception &ex){ cout<<"--> Exception updating transform values: "<<ex<<endl;}
	}
	catch (const Ice::Exception &ex){ cout<<"--> Excepción reading OmniRobot: "<<ex<<endl;}*/
}
/**
 * \brief This method creates dynamically a new node in innermodel that represents the target.
 * @param target the target whose pose we need to introduce in the innermodel
 */ 
void SpecificWorker::createInnerModelTarget(Target &target)
{
	InnerModelNode *nodeParent = innermodel->getNode("root");
	target.setTargetNameInInnerModel(QString::number(correlativeID++));
	InnerModelTransform *node = innermodel->newTransform(target.getTargetNameInInnerModel(), "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	
	QVec p = target.getTargetPose();
	innermodel->updateTransformValues(target.getTargetNameInInnerModel(),p.x(), p.y(), p.z(), p.rx(), p.ry(), p.rz(), "root");
	innermodel->updateTransformValues("target", p.x(), p.y(), p.z(), p.rx(), p.ry(), p.rz(), "root");
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::setFingers(const float d)
{
}

void SpecificWorker::setRobot(const int type)
{
}

TargetState SpecificWorker::getState(const string &part)
{
}

void SpecificWorker::setNewTip(const string &part, const string &transform, const Pose6D &pose)
{
}

void SpecificWorker::stop(const string &part)
{
}
/**
 * \brief This method send the joints to the original position (home position or start position)
 * @param part name of the body part
 */ 
void SpecificWorker::goHome(const string &part)
{
	QString partName = QString::fromStdString(part);

	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	qDebug() << "----------------------------------------";
	qDebug() << "Go gome" << QString::fromStdString(part);
	qDebug() << bodyParts[partName].getMotorList();

	QStringList lmotors = bodyParts[partName].getMotorList();
	for(int i=0; i<lmotors.size(); i++)
	{
		try 
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = lmotors.at(i).toStdString();
			nodo.position = innermodel->getJoint(lmotors.at(i))->home;
			nodo.maxSpeed = 1; //radianes por segundo
			mutex->lock();
				jointmotor_proxy->setPosition(nodo);
			mutex->unlock();

		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;
			throw ex;
		}
	}
}
/**
 * \brief Method of the interface. Store a new POSE6D target into the correspondig part of the robot.
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @param radius   radius of the target. IS NOT USED????
 */ 
void SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights, const float radius)
{
	if(!radius) return;
	
	QVec pose_	 	= QVec::vec6( target.x /(T)1000,  target.y/(T)1000,  target.z/(T)1000,  target.rx,  target.ry,  target.rz);	
	QVec weights_ 	= QVec::vec6(weights.x,          weights.y,         weights.z,         weights.rx, weights.ry, weights.rz);
	Target newTarget = Target(pose_, weights_, radius/1000.f, Target::TargetType::POSE6D);
	
	QString partName = QString::fromStdString(bodyPart);
	
	if (this->bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part: "<<partName;
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << pose_ << ". With weights: " << weights_;
	qDebug() << "----------------------------------------------------------------------------------";
		
	QMutexLocker locker(mutex);
	bodyParts[partName].addTargetToList(newTarget); // <---- falla aqui
	
	qDebug()<<"HOLA";
}

void SpecificWorker::advanceAlongAxis(const string &bodyPart, const Axis &ax, const float dist)
{
}

void SpecificWorker::pointAxisTowardsTarget(const string &bodyPart, const Pose6D &target, const Axis &ax, const bool &axisConstraint, const float axisAngleConstraint)
{
}

void SpecificWorker::setJoint(const string &joint, const float position, const float maxSpeed)
{
}

void SpecificWorker::sendData(const TData &data)
{
}



