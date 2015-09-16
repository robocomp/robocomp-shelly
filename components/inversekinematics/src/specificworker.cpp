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
	INITIALIZE_READY    = false;
	UPDATE_READY        = true; 
	mutexSolved         = new QMutex(QMutex::Recursive);
// 	file.open("/home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt", ios::out | ios::app);
// 	if (file.is_open()==false)
// 		qFatal("ARCHIVO NO ABIERTO");

	innermodel          = NULL;
#ifdef USE_QTGUI
	innerViewer         = NULL;
	this->osgView       = new OsgView(this);
// 	show();
#endif
	inversedkinematic   = new InversedKinematic();
	correlativeID       = 0;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	//file.close();
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
	}catch(std::exception e) {
		qFatal("Error reading Innermodel param");
	}
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
	qDebug() << "RIGHT TIP" << tipR;
	tipH = QString::fromStdString(params["HEADTIP"].value);  /// READ THE HEAD'S TIP

	motorsR = QString::fromStdString(params["RIGHTARM"].value); /// READ THE RIGHT ARM'S MOTORS
	motorsL = QString::fromStdString(params["LEFTARM"].value);  /// READ THE LEFT ARM'S MOTORS
	motorsH = QString::fromStdString(params["HEAD"].value);     /// READ THE HEAD'S MOTORS

	qDebug() << "RIGHT MOTORS" << motorsR;

	if(motorsR.size()>2 and motorsR!="EMPTY" and tipR.size()>2 and tipR!="EMPTY")
	{
		for (auto motor : motorsR.split(";", QString::SkipEmptyParts)) /// WE DIVIDE THE STRING BY THE ";"
		{
			auxiliar_motor_list.push_back(motor);
			qDebug() << motor;
		}
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

	QMutexLocker lock(mutex);
	INITIALIZE_READY = true;

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
	if (UPDATE_READY == true)	updateInnerModel();

	QMap<QString, BodyPart>::iterator partsIterator;
	for(partsIterator = bodyParts.begin(); partsIterator != bodyParts.end(); ++partsIterator)
	{
		QMutexLocker locker(mutex);
		if(!partsIterator.value().getTargetList().isEmpty())
		{
			partsIterator.value().getTargetList()[0].setTargetState(Target::IN_PROCESS);
			
			createInnerModelTarget(partsIterator.value().getTargetList()[0]);
			inversedkinematic->solveTarget(&partsIterator.value(), innermodel);
			UPDATE_READY = false;

			if(partsIterator.value().getTargetList()[0].getTargetState() == Target::FINISH) /// The inversedkinematic has finished
			{
				//TODO QUITAR DESPUES
				//updateMotors(partsIterator.value(), partsIterator.value().getTargetList()[0].getTargetFinalAngles());
				updateAngles(partsIterator.value().getTargetList()[0].getTargetFinalAngles(), partsIterator.value());

				if(inversedkinematic->deleteTarget() == true)
				{
					showInformation(partsIterator.value(), partsIterator.value().getTargetList()[0]);
					if(partsIterator.value().getTargetList()[0].getTargetDivided()==false)
						UPDATE_READY = true;
					
					addTargetSolved(partsIterator.value().getPartName(), partsIterator.value().getTargetList()[0]);
					removeInnerModelTarget(partsIterator.value().getTargetList()[0]);
					partsIterator.value().removeTarget();
				}
				else
					partsIterator.value().getTargetList()[0].setTargetState(Target::IN_PROCESS);
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
/**
 * \brief Method of the interface that stores a new POSE6D target into the correspondig part of the robot.
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @return the identifier of the target (an int)
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

	QVec pose_    = QVec::vec6( target.x /(T)1000,  target.y/(T)1000,  target.z/(T)1000,  target.rx,  target.ry,  target.rz);
	qDebug()<<"PESOS: "<<weights.x<<", "<<weights.y<<", "<<weights.z<<", "<<weights.rx<<", "<<weights.ry<<", "<<weights.rz;
	QVec weights_ = QVec::vec6(weights.x,           weights.y,         weights.z,         weights.rx, weights.ry, weights.rz);
	
	if(weights_.x()<=0 and weights_.y()<=0 and weights_.z()<=0 and weights_.rx()<=0 and weights_.ry()<=0 and weights_.rz()<=0)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "ZERO WEIGHTS: "<<weights_;
		RoboCompInverseKinematics::IKException ex;
		ex.text = "ZERO WEIGHTS";
		throw ex;
	}
	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << pose_ << ". With weights: " << weights_;
	qDebug() << "----------------------------------------------------------------------------------";
	
	if(weights_.rx()!=0 and weights_.ry()!=0 and weights_.rz()!=0)
	{
		QVec weights_aux 		= QVec::vec6(1, 1, 1,       0, 0, 0); //Sin rotacion
		Target newTarget_aux	= Target(0, pose_, weights_aux, true, Target::TargetType::POSE6D); //Sin rotacion
		
		QMutexLocker locker(mutex);
		bodyParts[partName].addTargetToList(newTarget_aux);
	}
	Target newTarget_ = Target(0, pose_, weights_, false, Target::TargetType::POSE6D); //Con rotacion o sin ella
	
	QMutexLocker locker(mutex);
	bodyParts[partName].addTargetToList(newTarget_); //cambia el identificador del target
	return newTarget_.getTargetIdentifier(); //devolvemos con rotacion
}

/**
 * \brief This method  of the interface stores a new ALIGNAXIS target into the correspondig part of the robot.
 * @param bodypart part of the robot body
 * @param target pose of the goal position
 * @param ax axis to be aligned
 */
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	QString partName = QString::fromStdString(bodyPart);
	if (!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	QVec pose_    = QVec::vec6(target.x /(T)1000, target.y/(T)1000, target.z/(T)1000, target.rx, target.ry, target.rz);
	QVec weights_ = QVec::vec6(                0,                0,                0,         1,         1,         1); //Weights vector ONLY ROTATION
	QVec axis_    = QVec::vec3(ax.x , ax.y, ax.z);
	Target newTarget = Target(0, pose_, weights_, axis_,Target::TargetType::ALIGNAXIS);

	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << pose_ << ". With weights: " << weights_;
	qDebug() << "----------------------------------------------------------------------------------";

	QMutexLocker locker(mutex);
	bodyParts[partName].addTargetToList(newTarget);
	return newTarget.getTargetIdentifier();
}

/**
 * @brief Make the body part advance along a given direction. It is meant to work as a simple translational joystick to facilitate grasping operations
 * @param bodyPart  name of the body part.
 * @param ax the direction
 * @param dist step to advance un milimeters
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	QString partName = QString::fromStdString(bodyPart);
	if (!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}

	QVec axis_ = QVec::vec3(ax.x, ax.y,	ax.z).normalize(); 	//Code axis as a dist norm vector in tip reference frame
	//Bigger jump admisible
	float step_;
	if(dist > 300)  step_ = 300;
	if(dist < -300) step_ = -300;
	step_ = dist / 1000.;   //PASANDO A METROS
	Target newTarget  = Target(0, axis_, step_, Target::TargetType::ADVANCEAXIS);

	qDebug() << "----------------------------------------------------------------------------------";
	qDebug() <<  __FILE__ << __FUNCTION__ << "New target arrived: " << partName << "dist" << step_ << "axis" << axis_;
	qDebug() << "----------------------------------------------------------------------------------";

	QMutexLocker locker(mutex);
	if( bodyParts[partName].getTargetList().size() < 2 )
	{
		bodyParts[partName].addTargetToList(newTarget);
		return newTarget.getTargetIdentifier();
	}
	return -1;
}


/**
 *\brief This method returns the state of the part.
 * @param bodypart the part of the robot.
 * @return bool: If the part hasn't got more pending targets, it returns TRUE, else it returns FALSE
 */
bool SpecificWorker::getPartState(const string &bodyPart)
{
	QString partName =QString::fromStdString(bodyPart);
	if(!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part: "<<partName;
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	QMutexLocker locker(mutex);
	return bodyParts[partName].getTargetList().isEmpty();
}


/**
 * \brief this method returns the state of a determinate target.
 * @param bodypart part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	QString partName =QString::fromStdString(bodyPart);
	if(!bodyParts.contains(partName))
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part: "<<partName;
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	RoboCompInverseKinematics::TargetState 	state;
	state.finish = false;
	

	QMutexLocker mm(mutexSolved);
	for(int i=0; i<targetsSolved.size(); i++)
	{
		//qDebug()<<"QUIERO: "<<partName<<" con "<<targetID<<"    PERO TENGO: "<<targetsSolved[i].part<<" con "<<targetsSolved[i].target.getTargetIdentifier();
		if(targetsSolved[i].part==partName and targetsSolved[i].target.getTargetIdentifier()==targetID)
			return targetsSolved[i].state;
	}
	return state;
}
/**
 * \brief This method of the interface stops the part of the robot and reset the queue of Targets
 * @param bodyPart the name of the part.
 */
void SpecificWorker::stop(const string &bodyPart)
{
	QString partName = QString::fromStdString(bodyPart);
	if(bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompInverseKinematics::IKException ex;
		ex.text = "Not recognized body part: "+bodyPart;
		throw ex;
	}
	qDebug() << "-------------------------------------------------------------------";
	qDebug() << "New COMMAND arrived " << __FUNCTION__ << partName;
	qDebug() << "-------------------------------------------------------------------";

	QMutexLocker locker(mutex);
	bodyParts[partName].reset();
}
/**
 * \brief this method moves the motors to their home value
 * @param bodyPart the part of the robot that we want to move to the home
 */
void SpecificWorker::goHome(const string &bodyPart)
{
}

/**
 * \brief this method changes the position of a determina joint.
 * @param joint the joint to change
 * @param angle the new angle of the joint.
 * @param maxSpeed the speed of the joint
 */
void SpecificWorker::setJoint (const string &joint, const float angle, const float maxSpeed)
{
	try
	{
		RoboCompJointMotor::MotorGoalPosition nodo;
		nodo.name = joint;
		nodo.position = angle;      // posición en radianes
		nodo.maxSpeed = maxSpeed;   // radianes por segundo
		jointmotor_proxy->setPosition(nodo);
	}
	catch (const Ice::Exception &ex){
		cout<< ex << "Exception moving " << joint << endl;
		RoboCompInverseKinematics::IKException exep;
		exep.text = "Not recognized joint: "+joint;
		throw exep;
	}
}
/**
 * @brief Set the fingers of the right hand position so there is d mm between them
 *
 * @param d millimeters between fingers
 * @return void
 */
void SpecificWorker::setFingers(const float d)
{
// 	qDebug() << __FUNCTION__;
// 
// 	float len = innermodel->transform("rightFinger1", QVec::zeros(3), "finger_right_1_1_tip").norm2();
// 	float D = (d/1000)/2.;  // half distnace in meters
// 	float s = D/len;
// 	if( s > 1) s = 1;
// 	if( s < -1) s = -1;
// 	float ang = asin(s);    // 1D inverse kinematics
// 	QVec angles = QVec::vec2( ang - 1, -ang + 1);
// 
// 	/// NOTE!!! DO THAT WITH innerModel->getNode("rightFinger1")->min
// 	QStringList joints;
// 	joints << "rightFinger1" << "rightFinger2";
// 	for(int i=0; i<joints.size(); i++)
// 	{
// 		try
// 			{
// 				RoboCompJointMotor::MotorGoalPosition nodo;
// 				nodo.name = joints.at(i).toStdString();
// 				nodo.position = angles[i]; // posición en radianes
// 				nodo.maxSpeed = 0.5; //radianes por segundo TODO Bajar velocidad.
// 				jointmotor_proxy->setPosition(nodo);
// 			} catch (const Ice::Exception &ex) {
// 				cout<<"EXECEPTION IN setFingers"<<ex<<endl;
// 				RoboCompInverseKinematics::IKException exep;
// 				exep.text = "Not recognized joint: "+joints.at(i).toStdString();
// 				throw exep;
// 			}
// 	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief SED DATA
 * 		  The BodyInverseKinematics component is subscribed to
 * 		  joystickAdapter in order to take the position that the
 * 		  publisher joystick (FALCON) send to him.
 * @param data struct with the axis coordinates, the speed and the buttons clicked.
 * @return void
 */
void SpecificWorker::sendData(const TData &data)
{
	if( INITIALIZE_READY == true)
	{
		//Preparamos los datos para enviarlo al IK:
		RoboCompInverseKinematics::Axis axis;
		for(auto a : data.axes)
		{
			if( a.name == "x") axis.x = a.value;
			if( a.name == "y") axis.y = a.value;
			if( a.name == "z") axis.z = a.value;
		}
		setTargetAdvanceAxis("RIGHTARM", axis, 150);
	}
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

			for (j=0; j<partsIterator.value().getMotorList().size(); j++)
				innermodel->updateJointValue(partsIterator.value().getMotorList()[j], mMap.at(partsIterator.value().getMotorList()[j].toStdString()).pos);
		}catch (const Ice::Exception &ex) {
			cout<<"--> Excepción en actualizar InnerModel: (i,j)"<<": "<<i<<"," <<j<<" "<<ex<<endl;
		}
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
	}catch(const Ice::Exception &ex) {
		cout<<"removeInnerModelTarget: the node doesn't exist";
	}
}
/**
 * \brief this method show all the information about one target of one body part
 * @param part
 */
void SpecificWorker::showInformation(BodyPart part, Target target)
{
	qDebug()<<"-------------------> TARGET INFORMATION:";
	qDebug()<<"Part name:      " <<part.getPartName();
	qDebug()<<"Target ID:      " <<target.getTargetIdentifier();
	qDebug()<<"Pose Target:    " <<target.getTargetPose();
	qDebug()<<"Weights Target: " <<target.getTargetWeight();

	QString state, finalState;
	if(target.getTargetState() == Target::TargetState::IDLE)		state = "IDLE";
	if(target.getTargetState() == Target::TargetState::IN_PROCESS)	state = "IN_PROCESS";
	if(target.getTargetState() == Target::TargetState::FINISH)		state = "FINISH";

	if(target.getTargetFinalState() == Target::TargetFinalState::LOW_ERROR) finalState = "LOW_ERROR";
	if(target.getTargetFinalState() == Target::TargetFinalState::LOW_INCS)	finalState = "LOW_INCS";
	if(target.getTargetFinalState() == Target::TargetFinalState::NAN_INCS)	finalState = "NAN_INCS";
	if(target.getTargetFinalState() == Target::TargetFinalState::KMAX)		finalState = "KMAX";

	qDebug()<<"State Target:   " <<state<<" (final: "<<finalState<<")";
	qDebug()<<"Final angles:   " <<target.getTargetFinalAngles();

	float errorT, errorR;
	qDebug()<<"Vector error:   "<<target.getTargetError(errorT, errorR)<<"\\";
	qDebug()<<"(T: "<<abs(errorT)*1000<<"mm , R: "<<abs(errorR)<<"rad)";

// 	file<<"P: ("      <<target.getTargetPose();
// 	file<<")    ERROR_T:"<<abs(errorT)*1000;
// 	file<<"     ERROR_R:" <<abs(errorR);
// 	file<<"     END: "<<finalState.toStdString()<<endl;
// 	flush(file);
}
/**
 * \brief This method stores the target solved into a structure
 * @param part  name of the part.
 * @param t target
 */ 
void SpecificWorker::addTargetSolved(QString part, Target t)
{
	RoboCompInverseKinematics::MotorList ml;
	RoboCompInverseKinematics::Motor m;
	RoboCompInverseKinematics::TargetState state;
	
	state.finish = true;
	state.elapsedTime = t.getTargetTimeExecution();
	t.getTargetError(state.errorT, state.errorR);
	state.errorT = state.errorT*1000; //a milimetros
	if(t.getTargetFinalState() == Target::TargetFinalState::LOW_ERROR) state.state = "LOW_ERROR";
	if(t.getTargetFinalState() == Target::TargetFinalState::LOW_INCS)  state.state = "LOW_INCS";
	if(t.getTargetFinalState() == Target::TargetFinalState::NAN_INCS)  state.state = "NAN_INCS";
	if(t.getTargetFinalState() == Target::TargetFinalState::KMAX)      state.state = "KMAX";
	
	for(int j=0; j<t.getTargetFinalAngles().size(); j++)
	{
		m.name  = bodyParts[part].getMotorList()[j].toStdString();
		m.angle = t.getTargetFinalAngles()[j];
		ml.push_back(m);
	}
	state.motors = ml;
		
	stTargetsSolved ts;
	ts.part   = part;
	ts.target = t;
	ts.state  = state;
	
	QMutexLocker mm(mutexSolved);
	targetsSolved.enqueue(ts);
}

// TODO QUITAR DESPUES
// void SpecificWorker::updateMotors (BodyPart bp, QVec angles)
// {
// 	for(int i=0; i<bp.getMotorList().size(); i++)
// 	{
// 		try
// 		{
// 			RoboCompJointMotor::MotorGoalPosition nodo;
// 			nodo.name = bp.getMotorList()[i].toStdString();
// 			nodo.position = angles[i]; // posición en radianes
// 			nodo.maxSpeed = 3; //radianes por segundo TODO Bajar velocidad.
// 			jointmotor_proxy->setPosition(nodo);
// 		} catch (const Ice::Exception &ex) {
// 			cout<<"EXCEPTION IN UPDATE MOTORS: "<<ex<<endl;
// 		}
// 	}
// 	sleep(1);
// }

