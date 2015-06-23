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
	file.open("/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt", ios::out | ios::app);
	if (file.is_open()==false)
		qFatal("ARCHIVO NO ABIERTO");

	QMutexLocker ml(&mutex);
	INITIALIZED			= false;
	stateMachine		= State::IDLE;
	abortatraslacion 	= false;
	abortarotacion 		= false;
	innerModel 			= NULL;
#ifdef USE_QTGUI	
	innerViewer 		= NULL;
	osgView 			= new OsgView(this);
 	show();
#endif
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
/**
 * \brief Metodo SET PARAM
 * Metodo desde el cual se cargaran los elementos que especifiquemos dentro del fichero config
 * del componente. Carga el innermodel, el osgview y actualiza el nodo target.
 * @param params mapa de parametros
 */
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
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
	QMutexLocker ml(&mutex);
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
	static int i = 0;
	if (i%20 != 0)
	{
#ifdef USE_QTGUI
		if (innerViewer)
		{
			innerViewer->update();
			osgView->autoResize();
			osgView->frame();
			return;
		}
#endif
	}
	updateAll();
	QMutexLocker ml(&mutex);
	switch(stateMachine)
	{
		case State::IDLE:
			if (currentTarget.getState() == Target::State::WAITING)
			{
				stateMachine 		= State::INIT_BIK;
				abortatraslacion 	= false;
				abortarotacion 		= false;
				qDebug()<<"Ha llegado un TARGET: "<<currentTarget.getPose();
				RoboCompInverseKinematics::Axis axis;
				axis.x = 0; axis.y = 0; axis.z = 1;
				setTargetAlignaxis(currentTarget.getBodyPart(), currentTarget.getPose6D(), axis);
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::INIT_BIK:
			try
			{
				int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), currentTarget.getPose6D(), currentTarget.getWeights6D());
				currentTarget.setID(identifier);
			}catch (const Ice::Exception &ex){
				std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT IK: "<<ex<<std::endl;
			}
			currentTarget.setState(Target::State::IN_PROCESS); // El currentTarget pasa a estar siendo ejecutado:
			correctedTarget = currentTarget;
			stateMachine    = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Esperamos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID()).finish == true)
			{
				qDebug()<<"---> El IK ha terminado.";
				updateMotors(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID()).motors);
				sleep(2);
				stateMachine = State::CORRECT_TRASLATION;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_TRASLATION:
			//la primera vez el ID de corrected es igual al de current así que enetra seguro.
			if(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID()).finish == false) return;
			updateMotors(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID()).motors);
			if(correctTraslation()==true or abortatraslacion==true)
			{
				const WeightVector weights = currentTarget.getWeights6D();
				if (fabs(weights.rx)!=0 and fabs(weights.ry)!=0 and fabs(weights.rz)!=0)				
				{
					std::cout<<"--> Correccion completada.\nPasamos a corregir la rotacion.\n";
					stateMachine = State::CORRECT_ROTATION;
				}
				else
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
					goHome("RIGHTARM");
				}
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:
			//la primera vez el ID de corrected es igaula al anterior así que entra seguro
			if(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID()).finish == false) return;
			updateMotors(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID()).motors);
			if (correctRotation()==true or abortarotacion==true)
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
				goHome("RIGHTARM");
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
 * \brief this method returns the state of a determinate target.
 * @param part part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	return inversekinematics_proxy->getTargetState(bodyPart, targetID);
}

int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{
	QMutexLocker ml(&mutex);
	cout<<"Recibido target"<<endl;
	if(currentTarget.getState()==Target::State::IDLE)
	{
		currentTarget.setBodyPart	(bodyPart);
		currentTarget.setPose		(target);
		currentTarget.setWeights	(weights);
		currentTarget.setState		(Target::State::WAITING);
	}
	else
	{
		Target auxnextTargets;
		auxnextTargets.setBodyPart	(bodyPart);
		auxnextTargets.setPose		(target);
		auxnextTargets.setWeights	(weights);
		auxnextTargets.setState		(Target::State::WAITING);
		
		nextTargets.enqueue(auxnextTargets);
	}
	return -1;
}

int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	return inversekinematics_proxy->setTargetAdvanceAxis(bodyPart, ax, dist);
}

int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	int id = inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
	while (inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
	updateMotors(inversekinematics_proxy->getTargetState(bodyPart, id).motors);
	return id;
}

void SpecificWorker::goHome(const string &bodyPart)
{
	//inversekinematics_proxy->goHome(bodyPart);
	try
	{
		RoboCompJointMotor::MotorGoalPosition nodo;
		
		nodo.name = "rightShoulder1";
		nodo.position = -2.3; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightShoulder2";
		nodo.position = -0.11; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightShoulder3";
		nodo.position = 0.11; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightElbow";
		nodo.position = 0.8; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightForeArm";
		nodo.position = 0.11; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightWrist1";
		nodo.position = 0.11; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
		
		nodo.name = "rightWrist2";
		nodo.position = 0.11; // posición en radianes
		nodo.maxSpeed = 2; //radianes por segundo TODO Bajar velocidad.
		jointmotor_proxy->setPosition(nodo);
	} 
	catch (const Ice::Exception &ex) {	cout<<"Exception in goHome: "<<ex<<endl;}
	
	sleep(3);
}

void SpecificWorker::stop(const string &bodyPart)
{
	inversekinematics_proxy->stop(bodyPart);
}

bool SpecificWorker::getPartState(const string &bodyPart)
{
	//return inversekinematics_proxy->getPartState(bodyPart);
	return (nextTargets.isEmpty() and currentTarget.getState()==Target::State::IDLE);;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	//Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	QMutexLocker ml(&mutex);
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
			{
				rightHand->setVisualPose(tag);
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Metodo CORRECT TRASLATION.
 * Se encarga de correctedTarget los errores de traslacion del tip del robot (donde esta realmente la mano). Calcula el
 * error entre la posicion en la que deberia estar (donde el cree que esta, internalPose) y la posicion en la que
 * verdaderamente esta (la pose que esta viendo la camara RGBD).
 * 		- Si el error es miserable no hace nada e indica que el target ha sido resueltoy devolvemos TRUE.
 * 		- Si el error es superior al umbral, calcula la pose a la que debe mover la mano y la manda al BIK.
 *		  Quedamos en espera de que el BIK termine de mover el brazo y devolvemos FALSE.
 * TODO MIRAR UMBRAL.
 * TODO ¿Que pasa cuando no ve la marca?
 *@return bool TRUE si el target esta perfecto o FALSE si no lo esta.
 */
bool SpecificWorker::correctTraslation	()
{
	qDebug()<<"\nCORRIGIENDO TRASLACION...";
	static float umbralMaxTime = 65, umbralMinTime = 6;
	static float umbralElapsedTime = 2, umbralError = 5;

	if(currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime)
	{
		abortatraslacion = true;   
		qDebug()<<"Abort traslation";
		return false;
	}
	if(rightHand->getSecondsElapsed() >umbralElapsedTime)
	{
		// If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
		std::cout<<"La camara no ve la marca..."<<std::endl;
		rightHand->setVisualPosewithInternal();
	}
	// COMPROBAMOS EL ERROR: Si es miserable no hacemos nada y acabamos la corrección.
	QVec errorInv = rightHand->getErrorInverse();
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2() < umbralError)
	{
		currentTarget.setState(Target::State::RESOLVED);
		qDebug()<<"done!";
		return true;
	}

	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
	qDebug()<<"Error T: "<<QVec::vec3(errorInv.x(), errorInv.y(),errorInv.z()).norm2();

	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
	correctedTarget.setPose(correccionFinal);
	qDebug()<<"Posicion  corregida: "<<correctedTarget.getPose();

	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),   correctedTarget.getPose().y(), correctedTarget.getPose().z(), 
									               correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose().rz());
	//Llamamos al BIK con el nuevo target corregido y esperamos
	WeightVector weights; //pesos a cero
	weights.x = 1;     weights.y = 1;    weights.z = 1;
	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
	
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), weights);
	correctedTarget.setID(identifier);
	return false;
}
/**
 * \brief Metodo CORRECT ROTATION
 * Corrige la posicion de la mano en traslacion y en rotacion.
 * @return bool TRUE si el target esta perfecto o FALSE si no lo esta.
 */
bool SpecificWorker::correctRotation()
{
	qDebug()<<"\nCORRIGIENDO ROTACION...";
	static float umbralMaxTime = 65, umbralMinTime = 6;
	static float umbralElapsedTime = 2, umbralErrorT = 5, umbralErrorR=0.01;

	// If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
	if (rightHand->getSecondsElapsed() > umbralElapsedTime)
	{
		std::cout<<"La camara no ve la marca..."<<std::endl;
		rightHand->setVisualPosewithInternal();
	}
	// COMPROBAMOS EL ERROR:
	QVec errorInv = rightHand->getErrorInverse();
	if(currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime)
	{
		abortarotacion = true;
		qDebug()<<"Abort rotation";
		file<<"P: ("      <<currentTarget.getPose();
		file<<") ERROR_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
		file<<" ERROR_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		file<<" END: "    <<currentTarget.getRunTime()<<"-->"<<abortatraslacion<<","<<abortarotacion<<endl;;
		flush(file);
		return false;
	}
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR)
	{
		currentTarget.setState(Target::State::RESOLVED);
		qDebug()<<"done!";
		file<<"P: ("      <<currentTarget.getPose();
		file<<") ERROR_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
		file<<" ERROR_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		file<<" END: "    <<currentTarget.getRunTime()<<"-->"<<abortatraslacion<<","<<abortarotacion<<endl;
		flush(file);
		return true;
	}
	
	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
	qDebug()<<"Error T: "<<QVec::vec3(errorInv.x(), errorInv.y(),errorInv.z()).norm2();

	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
	correctedTarget.setPose(correccionFinal);
	qDebug()<<"Correccion final: "<<correctedTarget.getPose();

	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),  correctedTarget.getPose().y(),   correctedTarget.getPose().z(), 
									              correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose6D().rz);
	//Llamamos al BIK con el nuevo target corregido y esperamos
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID(identifier);
	return false;
}
/**
 * \brief Metodo UPDATE ALL
 * Se encarga de actualizar la posicion de los motores del robot (el innerModel),
 * la pose del target que le enviamos y la pose de la marca visual que ve.
 */
void SpecificWorker::updateAll()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
	}catch (const Ice::Exception &ex){ 
		cout<<"--> Excepción en actualizar InnerModel";
	}
	/*try
	{
		RoboCompOmniRobot::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		try
		{
			innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		}catch (const Ice::Exception &ex){ 
			cout<<"--> Exception updating transform values: "<<ex<<endl;
		}
	}catch (const Ice::Exception &ex){
		cout<<"--> Excepción reading OmniRobot: "<<ex<<endl;
	}*/
	
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
	for(auto motor : motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle; // posición en radianes
			nodo.maxSpeed = 0.5; //radianes por segundo TODO Bajar velocidad.
			jointmotor_proxy->setPosition(nodo);
		} catch (const Ice::Exception &ex) {
			cout<<"EXCEPTION IN UPDATE MOTORS: "<<ex<<endl;
		}
	}
	sleep(1);
}

