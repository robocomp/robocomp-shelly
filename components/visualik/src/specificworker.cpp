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
// 	file.open("/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt", ios::out | ios::app);
// 	if (file.is_open() == false)
// 		qFatal("ARCHIVO NO ABIERTO");

	goMotorsGO         = false;
	stateMachine       = State::IDLE;
	abortCorrection    = false;
	innerModel         = NULL;
	contador           = 0;
	timeSinMarca       = 0.0;
	mutexSolved        = new QMutex(QMutex::Recursive);
	mutexRightHand     = new QMutex(QMutex::Recursive);
	firstCorrection    = QVec::zeros(3);
	
	kinematicDeviationTarget.x = kinematicDeviationTarget.y = kinematicDeviationTarget.z = 0;
	kinematicDeviation       = QVec::vec6();
	
#ifdef USE_QTGUI
	connect(this->goButton, SIGNAL(clicked()), this, SLOT(goYESButton()));
	innerViewer        = NULL;
	//osgView          = new OsgView(this);
	osgView            = new OsgView(this->widget);
 	show();
#endif
	
	QMutexLocker ml(mutex);
	INITIALIZED        = false;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
// 	file.close();
}


/**
 * \brief Metodo SET PARAM
 * Metodo desde el cual se cargaran los elementos que especifiquemos dentro del fichero config
 * del componente. Carga el innermodel, el osgview y actualiza el nodo target.
 * @param params mapa de parametros
 */
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	qDebug()<<"YEAAAAAAAAH: 11111";

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug()<<"YEAAAAAAAAH: 22222";

			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug()<<"YEAAAAAAAAH: 333333"<<QString::fromStdString(par.value);
		}
		else
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}

#ifdef USE_QTGUI
	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);
		delete innerViewer;
	}
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
#endif
	InnerModelNode *nodeParent = innerModel->getNode("root");
	if( innerModel->getNode("target") == NULL)
	{
		InnerModelTransform *node = innerModel->newTransform("target", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
	}
	rightHand = new VisualHand(innerModel, "grabPositionHandR");	
	timer.start(Period);		
	QMutexLocker ml(mutex);
	INITIALIZED = true;
	
	return true;
}


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Slot COMPUTE
 * Bucle ejecutado por el hilo del programa. Se encarga de actualizar el innermodel de la clase y de pintar la
 * posicion del target que ha llegado y la posición de la marca que esta viendo la camara.
 * MAQUINA DE ESTADOS:
 * 	-IDLE: estado en espera. Inicialmente el currentTarget se crea con estado IDLE, indicando que no tiene nada
 * 		   Pero cuando cambia a WAITING indica que ha llegado un target que espera ser ejecutado. En ese momento
 * 		   pasamos a INIT_BIK.
 * 	-INIT_BIK: independientemente de los pesos del currentTarget, enviamos el target al BIK para que se ejecute.
 * 			   Luego pasamos al estado WAIT_BIK.
 * 	-WAIT_BIK: esperamos a que el brazo deje de moverse para empezar con la correccion. Cuando se pare pasamos
 * 			   al estado CORRECT_TRASLATION.
 * 	-CORRECT_ROTATION: corrige los errores de la pose del tip del robot, arreglando tanto traslacion como rotacion.
 * 	-CORRECT_TRASLATION: corrige solo los errores de traslacion del tip.
 */
void SpecificWorker::compute()
{
#ifdef USE_QTGUI
		if (innerViewer)
		{
			innerViewer->update();
			osgView->autoResize();
			osgView->frame();
		}
#endif
	updateInnerModel_motors_target_and_visual();
	QMutexLocker ml(mutex);
	switch(stateMachine)
	{
		case State::IDLE:
			if (currentTarget.getState() == Target::State::WAITING)
			{
				stateMachine     = State::INIT_BIK;
				abortCorrection = false;
				qDebug()<<"Ha llegado un TARGET: "<<currentTarget.getPose();
				timeSinMarca = 0.0;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::INIT_BIK:
			applyFirstCorrection();
			currentTarget.setState(Target::State::IN_PROCESS); // El currentTarget pasa a estar siendo ejecutado:
			correctedTarget = currentTarget;
			stateMachine    = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Esperamos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).finish == true)
			{
				qDebug()<<"---> El IK ha terminado.";
				//updateMotors(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).motors);
				stateMachine = State::CORRECT_ROTATION;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:
			//la primera vez el ID de corrected es igaula al anterior así que entra seguro
			if(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).finish == false) return;
			//updateMotors(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).motors);
			if (correctRotation()==true or abortCorrection==true)
			{
				if(nextTargets.isEmpty()==false)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					currentTarget = nextTargets.head();
					nextTargets.dequeue();
				}
				else
					currentTarget.setState(Target::State::IDLE);
				stateMachine = State::IDLE;
				//goHome("RIGHTARM");
			}
		break;
		//---------------------------------------------------------------------------------------------
		default:
			break;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


/**
 *\brief This method returns the state of the part.
 * @return bool: If the part hasn't got more pending targets, it returns TRUE, else it returns FALSE
 */
bool SpecificWorker::getPartState(const string &bodyPart)
{
	//return inversekinematics_proxy->getPartState(bodyPart);
	return (nextTargets.isEmpty() and currentTarget.getState()==Target::State::IDLE);;
}


/**
 * \brief this method returns the state of a determinate target.
 * @param part part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	//return inversekinematics_proxy->getTargetState(bodyPart, targetID);
	QMutexLocker ml(mutexSolved);
	RoboCompInverseKinematics::TargetState 	state;
	state.finish = false;
	
	for(int i=0; i<solvedList.size(); i++)
	{
		if(bodyPart=="RIGHTARM" and targetID==solvedList[i].getID_VIK())
		{
			state.finish = true;
			if(solvedList[i].getState()==Target::State::NOT_RESOLVED) state.state = "NOT_RESOLVED";
			if(solvedList[i].getState()==Target::State::RESOLVED)     state.state = "RESOLVED";
		}
	}
	return state;
}


/**
 * \brief This method reimplements the interface method setTargetPose6D of the inversekinematics component
 * in order to aply the visual correction
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{
	QMutexLocker ml(mutex);
	cout<<"Recibido target"<<endl;
	if(currentTarget.getState()==Target::State::IDLE)
	{
		currentTarget.setBodyPart (bodyPart);
		currentTarget.setPose     (target);
		currentTarget.setWeights  (weights);
		currentTarget.setState    (Target::State::WAITING);
		currentTarget.setID_VIK   (contador);
	}
	else
	{
		Target auxnextTargets;
		auxnextTargets.setBodyPart (bodyPart);
		auxnextTargets.setPose     (target);
		auxnextTargets.setWeights  (weights);
		auxnextTargets.setState    (Target::State::WAITING);
		auxnextTargets.setID_VIK   (contador);
		//nextTargets.clear();
		nextTargets.enqueue(auxnextTargets);
	}
	contador++;
	
	return contador-1;
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
	return inversekinematics_proxy->setTargetAdvanceAxis(bodyPart, ax, dist);
}


/**
 * \brief This method  of the interface stores a new ALIGNAXIS target into the correspondig part of the robot.
 * @param bodypart part of the robot body
 * @param target pose of the goal position
 * @param ax axis to be aligned
 */
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	//NOTE: When the VIK is connected to IK directly
// 	int id = inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
// 	while (inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
// 	updateMotors(inversekinematics_proxy->getTargetState(bodyPart, id).motors);
// 	return id;
	//NOTE: When the VIK is connected to GIK and GIK is connected to IK.
	return inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
}


/**
 * \brief this method moves the motors to their home value
 * @param bodyPart the part of the robot that we want to move to the home
 */
void SpecificWorker::goHome(const string &bodyPart)
{
	inversekinematics_proxy->goHome(bodyPart);
}

void SpecificWorker::stop(const string &bodyPart)
{
	QMutexLocker ml(mutex);
	nextTargets.clear();
	currentTarget.setState(Target::State::IDLE);
	stateMachine = State::IDLE;
	
	inversekinematics_proxy->stop(bodyPart);
}

/**
 * \brief this method changes the position of a determina joint.
 * @param joint the joint to change
 * @param angle the new angle of the joint.
 * @param maxSpeed the speed of the joint
 */
void SpecificWorker::setJoint(const string &joint, const float angle, const float maxSpeed)
{
	inversekinematics_proxy->setJoint(joint, angle, maxSpeed);
}


/**
 * @brief Set the fingers of the right hand position so there is d mm between them
 * @param d millimeters between fingers
 */
void SpecificWorker::setFingers(const float d)
{
	inversekinematics_proxy->setFingers(d);
}


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	// Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
			{
// 				tag.tx *= 0.65;
// 				tag.ty *= 0.65;
// 				tag.tz *= 0.65;
				QMutexLocker ml(mutexRightHand);
				rightHand->setVisualPose(tag);
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Metodo auxiliar que guarda en el fichero el resultado de la correccion de un target.
 * @param errorInv 
 */ 
void SpecificWorker::printXXX(const QVec errorInv/*, bool camaraNoVista*/)
{	
// 	file<<"P: ("      <<currentTarget.getPose();
// 	file<<")   ErrorVisual_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
// 	file<<"   ErrorVisual_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
// 	file<<"   ErrorDirecto_T:" <<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).errorT;
// 	file<<"   ErrorDirecto_R: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).errorR;
// 	file<<"   END: "    <<currentTarget.getRunTime();
// 	file<<"   WHY?: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).state;
// 	if(timeSinMarca > (60/4))
// 		file<<"   CAMARA PERDIDA: "<<1<<" - "<<timeSinMarca<<endl;
// 	else
// 		file<<"   CAMARA PERDIDA: "<<0<<" - "<<timeSinMarca<<endl;
// 	flush(file);	
}

/**
 * \brief Metodo Aniadido APPLY CORRECTION
 * Metodo aniadido para aplicar a la cinematica inversa una correccion inicial del target.
 * La primera vez que se ejecuta la correccion vale 0, pero a medida que se van ejecutando 
 * los targets, el error calculado entre el target y el target corregido se almacena y se
 * aplica en esta correcion inicial.
 * TODO HAY QUE PROBARLO MUUUUUUUUCHO
 */ 
void SpecificWorker::applyFirstCorrection()
{
	// Use stored deviation only if the current target is close to the one the last deviation was associated with
	if (0)
	{
		// Here we just send the IK the target we get
		// That's all
		return;
	}
	
	// If it is close, then and only then, we try to guess the error and fix it...

	// Compute a matrix containing the corrected target
	RTMat targetM(
	  currentTarget.getPose6D().rx, currentTarget.getPose6D().ry, currentTarget.getPose6D().rz,
	  currentTarget.getPose6D().x,  currentTarget.getPose6D().y,  currentTarget.getPose6D().z
	);
	// Create a matrix for the stored kinematic deviation 
	RTMat initCorrM(
	  kinematicDeviation.rx(), kinematicDeviation.ry(), kinematicDeviation.rz(), 
	  kinematicDeviation.x(),  kinematicDeviation.y(), kinematicDeviation.z()
	);
	RTMat result = initCorrM * targetM;
	// Create a target (translation & rotation vector) based on the resulting matrix
	QVec correccion = QVec::vec6(0,0,0,  0,0,0);
	for (int i=0; i<3; i++) correccion(i) = result(i, 3);	
	for (int i=0; i<3; i++) correccion(i+3) = result.extractAnglesR_min()(i);
	// Set and send the corrected target
	correctedTarget.setPose(correccion);
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID_IK(identifier);	
}

/**
 * \brief Metodo aniadido STORE TARGET CORRECTION
 * Almacena en la variable de clase firstCorrection, la correccion entre el target original y la última posicion corregida
 * asociada a ese target para que pueda ser utilizada por el siguiente target como correccion inicial.
 * TODO HAY QUE PROBARLO
 */ 
void SpecificWorker::storeKinematicDeviation()
{
	kinematicDeviation = rightHand->getInternalError();
	kinematicDeviationTarget = currentTarget.getPose6D();
}

/**
 * \brief Metodo CORRECT ROTATION
 * Corrige la posicion de la mano en traslacion y en rotacion.
 * @return bool TRUE si el target esta perfecto o FALSE si no lo esta.
 */
bool SpecificWorker::correctRotation()
{
	qDebug()<<"\n\n\n-------------------------";
	const float umbralMaxTime=250, umbralMinTime=3;
	const float tagLostThresholdTime=1;
	const float umbralErrorT=10.0, umbralErrorR=0.1;


	QString rightTip = rightHand->getTip();
	QVec rightHandVisualPose, rightHandInternalPose;
	QVec errorInv;

	{
		QMutexLocker ml(mutexRightHand);
		updateInnerModel_motors_target_and_visual();
		if (rightHand->getSecondsElapsed() > tagLostThresholdTime) // If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
		{
			//qFatal("tag lost!");
			timeSinMarca = timeSinMarca+rightHand->getSecondsElapsed();
			rightHand->setVisualPosewithInternalError();
		}
		// COMPROBAMOS EL ERROR:
		errorInv = rightHand->getTargetErrorInverse(); // error: visual hand from the target's perspective
		rightHandVisualPose = rightHand->getVisualPose();
		rightHandInternalPose = rightHand->getInternalPose();
	}

	if (currentTarget.getRunTime()>umbralMaxTime)
	{
		abortCorrection = true;
		currentTarget.setState(Target::State::NOT_RESOLVED);
		errorInv.print("abort with error INV");
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return false;
	}

	storeKinematicDeviation();
	
	// If the error is below a threshold accept the current kinematic configuration
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR and currentTarget.getRunTime()>umbralMinTime)
	{
		currentTarget.setState(Target::State::RESOLVED);
		errorInv.print("done with error INV");
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return true;
	}

	// CORREGIR TRASLACION
	QVec errorInvP           = QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z());
	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", "target") * errorInvP;
	errorInvP_from_root.print("err1");
	errorInvP_from_root = errorInvP_from_root.operator*(0.5);
	innerModel->transformS("root", "target").print("target");
	innerModel->transformS("root", "visual_hand").print("visual_hand");
	errorInvP_from_root.print("err2");
	QVec poseCorregida   = innerModel->transform("root", "target") + errorInvP_from_root;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	for (int i=0; i<3; i++) correccionFinal(i) = poseCorregida(i);
	
	// CORREGIR ROTACION
	//Pasamos los angulos a matriz de rotacion
	QMat rotErrorInv = Rot3D(errorInv.rx(), errorInv.ry(), errorInv.rz());
	// multiplicamos la matriz del tip en el root por la matriz generada antes
	//extraemos los angulos de la nueva matriz y esos son los angulas ya corregidos
// 	QVec angulosFinales = (innerModel->getRotationMatrixTo("root", "target")*rotErrorInv.invert()).extractAnglesR_min();
	QVec angulosFinales = (innerModel->getRotationMatrixTo("root", "target")).extractAnglesR_min();
	for (int i=0; i<3; i++) correccionFinal(i+3) = angulosFinales(i);

	
	correctedTarget.setPose(correccionFinal);
	
	errorInv.print("ERROR INVERSE: ");
	correccionFinal.print("CORRECCION: ");

	// Llamamos al BIK con el nuevo target corregido y esperamos
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID_IK(identifier);

// 	stateMachine = State::IDLE;

// 	sleep(1);
	return false;
}


/**
 * \brief Metodo UPDATE ALL
 * Se encarga de actualizar la posicion de los motores del robot (el innerModel),
 * la pose del target que le enviamos y la pose de la marca visual que ve.
 */
void SpecificWorker::updateInnerModel_motors_target_and_visual()
{
	//qDebug() << "--------";
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		//	qDebug() << QString::fromStdString(j.first) << j.second.pos;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
	}
	const Pose6D tt = currentTarget.getPose6D();
	innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	const QVec pR = rightHand->getVisualPose();
	innerModel->updateTransformValues("visual_hand", pR.x(), pR.y(), pR.z(), pR.rx(), pR.ry(), pR.rz());
}


/**
 * \brief Metodo UPDATE MOTORS
 */
void SpecificWorker::updateMotors (RoboCompInverseKinematics::MotorList motors)
{
	for (auto motor : motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle; // posición en radianes
			nodo.maxSpeed = 3; //radianes por segundo TODO Bajar velocidad.
			jointmotor_proxy->setPosition(nodo);
		}
		catch (const Ice::Exception &ex)
		{
			std::cout<<"EXCEPTION IN UPDATE MOTORS: "<<ex<<std::endl;
			//ABANDONAMOS TARGET SI LA IK NO PUEDE MOVERSE NI SIQUIERA CERCA DEL TARGET
		}
	}
	
	MotorStateMap allMotorsAct, allMotorsBack;
	jointmotor_proxy->getAllMotorState(allMotorsBack);
	for (bool allStill=false;   allStill==false;   allMotorsBack=allMotorsAct)
	{
		usleep(500000);
		jointmotor_proxy->getAllMotorState(allMotorsAct);
		allStill = true;
		for (auto v : allMotorsAct)
		{
			if (abs(v.second.pos - allMotorsBack[v.first].pos) > 0.01)
			{
				allStill = false;
				break;
			}
		}
	} 
	usleep(500000);

}
















