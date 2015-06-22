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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	file.open("/home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt", ios::out | ios::app);
	if (file.is_open()==false)
		qFatal("ARCHIVO NO ABIERTO");
	
	innermodel = NULL;
#ifdef USE_QTGUI
	innerViewer = NULL;
	this->osgView = new OsgView(this);
// 	show();
#endif
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
#ifdef USE_QTGUI
	if (innerViewer)
	{
		this->osgView->getRootGroup()->removeChild(innerViewer);
		delete innerViewer;
	}
	this->innerViewer = new InnerModelViewer(this->innermodel, "root", this->osgView->getRootGroup(), true);
#endif

	QString tipR, tipL, tipH;
	QString motorsR, motorsL, motorsH;
	QStringList	auxiliar_motor_list;

	tipR = QString::fromStdString(params["RIGHTTIP"].value); /// READ THE RIGHT ARM'S TIP
	tipL = QString::fromStdString(params["LEFTTIP"].value);  /// READ THE LEFT ARM'S TIP
	tipH = QString::fromStdString(params["HEADTIP"].value);  /// READ THE HEAD'S TIP
	
	motorsR = QString::fromStdString(params["RIGHTARM"].value); /// READ THE RIGHT ARM'S MOTORS
	motorsL = QString::fromStdString(params["LEFTARM"].value);  /// READ THE LEFT ARM'S MOTORS
	motorsH = QString::fromStdString(params["HEAD"].value);     /// READ THE HEAD'S MOTORS

	if(motorsR.size()>2 and motorsR!="EMPTY" and tipR.size()>2 and tipR!="EMPTY")
	{
		for (auto motor : motorsR.split(";", QString::SkipEmptyParts)) /// WE DIVIDE THE STRING BY THE ";"
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

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this is the principal SLOT of the component. There is a thread that always 
 * compute this SLOT every (timer) seconds.
 */ 
void SpecificWorker::compute()
{
	updateInnerModel();
		
	QMap<QString, BodyPart>::iterator partsIterator;
	for(partsIterator = bodyParts.begin(); partsIterator != bodyParts.end(); ++partsIterator)
	{
		// COMPROBAR SI ES LA PRIMERA VEZ QUE ENTRA EL TARGET--> 
		// CREAR PLAN PARA ALCANZAR EL TARGET USANDO GRAFO. ESE PLANNER TENDRA GRAFO (LO CREA O LO LEE), EL INNER Y EL TARGET
		// 1) BUSCA DE DONDE ESTAMOS AL GRAFO
		// 2) BUSCA DEL TARGET AL GRAFO
		// 3) BUSCAR CAMINO POR EL GRAFO DEVUELVE EL CAMINO COMO LISTA DE TARGET INYECTADOS AL PRINCIPIO.
			
		QMutexLocker locker(mutex);
		if(!partsIterator.value().getTargetList().isEmpty())
		{			
			partsIterator.value().getTargetList()[0].setTargetState(Target::IN_PROCESS);	
			createInnerModelTarget(partsIterator.value().getTargetList()[0]);
			inversedkinematic->solveTarget(&partsIterator.value(), innermodel);
			
			if(partsIterator.value().getTargetList()[0].getTargetState() == Target::FINISH) /// The inversedkinematic has finished
			{
				float errorT, errorR;
				partsIterator.value().getTargetList()[0].getTargetError(errorT, errorR);
				if(partsIterator.value().getTargetList()[0].getTargetType()!=Target::TargetType::ALIGNAXIS and abs(errorT) < 0.001)
					qDebug()<<"--------------> FINISH TARGET    OK\n";
				else if(partsIterator.value().getTargetList()[0].getTargetType()==Target::TargetType::ALIGNAXIS and abs(errorR) < 0.001)
					qDebug()<<"--------------> FINISH TARGET    OK\n";
				
				else
					qDebug()<<"--------------> FINISH TARGET    NOT OK\n";
				updateAngles(partsIterator.value().getTargetList()[0].getTargetFinalAngles(), partsIterator.value());
				
				if(inversedkinematic->deleteTarget() == true)
				{
					showInformation(partsIterator.value(), partsIterator.value().getTargetList()[0]);
					removeInnerModelTarget(partsIterator.value().getTargetList()[0]);
					partsIterator.value().addSolvedToList();
				}
			}
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
void SpecificWorker::stop(const string &part)
{

}
/**
 * \brief Method of the interface that stores a new POSE6D target into the correspondig part of the robot.
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 */ 
int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{	
	QString partName = QString::fromStdString(bodyPart);
	if (!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part: "<<partName;
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	
	QVec pose_	 	= QVec::vec6( target.x /(T)1000,  target.y/(T)1000,  target.z/(T)1000,  target.rx,  target.ry,  target.rz);	
	QVec weights_ 	= QVec::vec6(weights.x,          weights.y,         weights.z,         weights.rx, weights.ry, weights.rz);
	Target newTarget = Target(0, pose_, weights_, Target::TargetType::POSE6D);
	
	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << pose_ << ". With weights: " << weights_;
	qDebug() << "----------------------------------------------------------------------------------";
		
	QMutexLocker locker(mutex);
	bodyParts[partName].addTargetToList(newTarget);
	//qDebug()<<"ID: "<<newTarget.getTargetIdentifier();
	return newTarget.getTargetIdentifier();
}

int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	return 0;
}
/**
 * \brief This method  of the interface stores a new ALIGNAXIS target into the correspondig part of the robot.
 * @param bodypart part of the robot body
 * @param target pose of the goal position
 * @param ax axis to be aligned
 */ 
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	/*QString partName = QString::fromStdString(bodyPart);
	if (!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	QVec pose_	 	 = QVec::vec6(target.x /(T)1000, target.y/(T)1000, target.z/(T)1000, target.rx, target.ry, target.rz);	
	QVec weights_ 	 = QVec::vec6(                0,                0,                0,         1,         1,         1); //Weights vector ONLY ROTATION
	QVec axis_ 		 = QVec::vec3(ax.x , ax.y, ax.z);
	Target newTarget = Target(pose_, weights_, axis_,Target::TargetType::ALIGNAXIS);
	
	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << pose_ << ". With weights: " << weights_;
	qDebug() << "----------------------------------------------------------------------------------";

	QMutexLocker locker(mutex);
	bodyParts[partName].addTargetToList(newTarget);
	return newTarget.getTargetIdentifier();*/
}

bool SpecificWorker::getPartState(const string &bodyPart)
{
	return false;
}
/**
 * \brief this method returns the state of a determinate target.
 * @param part part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */ 
TargetState SpecificWorker::getTargetState(const string &part, const int targetID)
{
	QString partName =QString::fromStdString(part);
	if(!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part: "<<partName;
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+part;
		throw ex;
	}	
	RoboCompInverseKinematics::MotorList 	ml;
	RoboCompInverseKinematics::Motor 		m;
	RoboCompInverseKinematics::TargetState 	state;
	state.finish = false;
	state.elapsedTime = 0;
	
	if(bodyParts[partName].getSolvedList().isEmpty()==false)
	{
		for(int i=0; i<bodyParts[partName].getSolvedList().size();i++)
		{
			if(bodyParts[partName].getSolvedList()[i].getTargetIdentifier() == targetID)
			{
				state.finish = true;
				state.elapsedTime = bodyParts[partName].getSolvedList()[i].getTargetTimeExecution();
				
				for(int j=0; j<bodyParts[partName].getSolvedList()[i].getTargetFinalAngles().size(); j++)
				{
					m.name = bodyParts[partName].getMotorList()[j].toStdString();
					m.angle = bodyParts[partName].getSolvedList()[i].getTargetFinalAngles()[j];			
					ml.push_back(m);
				}
				state.motors=ml;
			}
		}
	}
	return state;
}

void SpecificWorker::goHome(const string &part)
{
	try
	{
		RoboCompJointMotor::MotorGoalPosition nodo;
		
		nodo.name = "rightShoulder1";
		nodo.position = -2.05; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightShoulder2";
		nodo.position = -0.2; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightShoulder3";
		nodo.position = 0.5; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightElbow";
		nodo.position = 0.8; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightForeArm";
		nodo.position = 0.1; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightWrist1";
		nodo.position = 0.1; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightWrist2";
		nodo.position = 0.1; // posición en radianes
		nodo.maxSpeed = 0.95; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
	} 
	catch (const Ice::Exception &ex) {	cout<<"Exception in goHome: "<<ex<<endl;}
}

void SpecificWorker::sendData(const TData &data)
{

}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method update the angles of each robot's joint and the robot's base.
 */ 
void SpecificWorker::updateInnerModel()
{
	RoboCompJointMotor::MotorList mList;
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
}
/**
 * \brief This method updates the RCIS with the values of all the robot joints
 * @param newAngles
 * @param part
 */ 
void SpecificWorker::updateAngles(QVec newAngles, BodyPart part)
{
	for(int i=0; i<part.getMotorList().size(); i++)
		innermodel->updateJointValue(part.getMotorList()[i], newAngles[i]);
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
}
/**
 * \brief this method removes dynamically the target node created before in createInnerModelTarget
 *@param target target to delete. 
 */ 
void SpecificWorker::removeInnerModelTarget(Target& target)
{
	try
	{
		innermodel->removeNode(target.getTargetNameInInnerModel());
	}
	catch(const Ice::Exception &ex) {cout<<"removeInnerModelTarget: the node doesn't exist";}
}
/**
 * \brief this method show all the information about one target of one body part
 * @param part
 */ 
void SpecificWorker::showInformation(BodyPart part, Target target)
{
	qDebug()<<"-------------------> TARGET INFORMATION:";
	qDebug()<<"Part name:		"    <<part.getPartName();
	qDebug()<<"Pose Target:		"    <<target.getTargetPose();
	qDebug()<<"Weights Target:		"<<target.getTargetWeight();

	QString state, finalState;
	if(target.getTargetState() == Target::TargetState::IDLE)		state = "IDLE";
	if(target.getTargetState() == Target::TargetState::IN_PROCESS)	state = "IN_PROCESS";
	if(target.getTargetState() == Target::TargetState::FINISH)		state = "FINISH";

	if(target.getTargetFinalState() == Target::TargetFinalState::LOW_ERROR) finalState = "LOW_ERROR";
	if(target.getTargetFinalState() == Target::TargetFinalState::LOW_INCS)	finalState = "LOW_INCS";
	if(target.getTargetFinalState() == Target::TargetFinalState::NAN_INCS)	finalState = "NAN_INCS";
	if(target.getTargetFinalState() == Target::TargetFinalState::KMAX)		finalState = "KMAX";

	qDebug()<<"State Target:		"<<state<<" (final: "<<finalState<<")";
	qDebug()<<"Final angles:		"<<target.getTargetFinalAngles();
	
	float errorT, errorR;
	qDebug()<<"Vector error:		"<<target.getTargetError(errorT, errorR)<<"\\";
	qDebug()<<"(T: "<<abs(errorT)<<" , R: "<<abs(errorR)<<")";
	
	file<<"P: ("      <<target.getTargetPose();
	file<<")    ERROR_T:"<<abs(errorT);
	file<<"     ERROR_R:" <<abs(errorR);
	file<<"     END: "<<finalState.toStdString()<<endl;
	flush(file);
}


