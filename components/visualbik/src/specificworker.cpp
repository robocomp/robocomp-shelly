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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	file.open("datosObtenidos.txt", ios::out);

	QMutexLocker im(&mutex);
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
		file.close();
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
	rightHand 			= new VisualHand(innerModel, "grabPositionHandR");
	leftHand 			= new VisualHand(innerModel, "grabPositionHandL");
	
	timer.start(Period);		
	QMutexLocker im(&mutex);
	INITIALIZED = true;
	return true;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										SLOTS DEL PROGRAMA													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * \brief Slot COMPUTE
 * Bucle ejecutado por el hilo del programa. Se encarga de actualizar el innermodel de la clase y de pintar la
 * posicion del target que ha llegado y la posición de la marca que esta viendo la camara.
 * MAQUINA DE ESTADOS:
 * 	-IDLE: estado en espera. Inicialmente el trueTarget se crea con estado IDLE, indicando que no tiene nada
 * 		   Pero cuando cambia a WAITING indica que ha llegado un target que espera ser ejecutado. En ese momento
 * 		   pasamos a INIT_BIK.
 * 	-INIT_BIK: independientemente de los pesos del trueTarget, enviamos el target al BIK para que se ejecute.
 * 			   Luego pasamos al estado WAIT_BIK.
 * 	-WAIT_BIK: esperamos a que el brazo deje de moverse para empezar con la correccion. Cuando se pare pasamos
 * 			   al estado CORRECT_ROTATION o CORRECT_TRASLATION dependiendo de los pesos del target.
 * 	-CORRECT_ROTATION: corrige los errores de la pose del tip del robot, arreglando tanto traslacion como rotacion.
 * 	-CORRECT_TRASLATION: corrige solo los errores de traslacion del tip.
 */
void SpecificWorker::compute()
{
	static int i = 0;
	if (i++%20 != 0)
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
	actualizarTodo();
	QMutexLocker ml(&mutex);
	switch(stateMachine)
	{
		case State::IDLE:
			// Estado inicial en espera hasta que trueTarget tenga un valor aceptable y este esperando a
			// ser ejecutado. Cambiamos a INIT_BIK
			if (trueTarget.getState() == Target::State::WAITING)
			{
				stateMachine 		= State::INIT_BIK;
				abortatraslacion 	= false;
				abortarotacion 		= false;
				qDebug()<<"Ha llegado un TARGET: "<<trueTarget.getPose();
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::INIT_BIK:
			// Nos da igual si el target tiene en cuenta las rotaciones o las traslaciones, lo enviamos al BIK
			// que ya se encargará de manejar los dos casos.
			try
			{
				bodyinversekinematics_proxy->setTargetPose6D(trueTarget.getBodyPart(), trueTarget.getPose6D(), trueTarget.getWeights6D(), 250);
			}
			catch (const Ice::Exception &ex)
			{
				std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT BIK: "<<ex<<std::endl;
			}

			trueTarget.setState(Target::State::IN_PROCESS); // El trueTarget pasa a estar siendo ejecutado:
			correctedTarget = trueTarget;
			stateMachine = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Espermos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(bodyinversekinematics_proxy->getState(trueTarget.getBodyPart()).finish == true)
			{
				qDebug()<<"---> El BIK ha terminado.";
				stateMachine = State::CORRECT_TRASLATION;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_TRASLATION:
			if (bodyinversekinematics_proxy->getState(trueTarget.getBodyPart()).finish != true) return;
			if(correctTraslation()==true or abortatraslacion==true)
			{
				// Si la correccion ha terminado y hay que coregir la rotacion...
				const WeightVector weights = trueTarget.getWeights6D();
				if (fabs(weights.rx)!=0 and fabs(weights.ry)!=0 and fabs(weights.rz)!=0)				
				{
					std::cout<<"--> Correccion completada.\nPasamos a corregir la rotacion.\n";
					stateMachine = State::CORRECT_ROTATION;
				}
				else
				{
					//Si no hay que orregir la rotacion pasamos al siguiente target...
					if(nextTargets.isEmpty()==false)
					{
						std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
						trueTarget = nextTargets.head();
						nextTargets.dequeue();
					}
					else
						trueTarget.setState(Target::State::IDLE);
				}
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:
			if (bodyinversekinematics_proxy->getState(trueTarget.getBodyPart()).finish != true) return;
			if (correctRotation()==true or abortarotacion==true)
			{
				// Si la correccion ha terminado y hay un target esperando
				if(nextTargets.isEmpty()==false)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					trueTarget = nextTargets.head();
					nextTargets.dequeue();
				}
				else
					trueTarget.setState(Target::State::IDLE);
				stateMachine = State::IDLE;
				bodyinversekinematics_proxy->goHome("RIGHTARM");
			}
		break;
		//---------------------------------------------------------------------------------------------
		default:
			break;
	}

#ifdef USE_QTGUI
	if (innerViewer)
	{
		innerViewer->update();
		osgView->autoResize();
		osgView->frame();
	}
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 											METODOS PRIVADOS												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
bool SpecificWorker::correctTraslation()
{
	qDebug()<<"\nCORRIGIENDO TRASLACION...";
	static int iteraciones = 0, 	maxIteraciones = 10;    // Para evitar que se quede atascado.
	float umbralElapsedTime = 0.5, 	umbralError = 5;

	if(iteraciones > maxIteraciones)
	{
		abortatraslacion = true;
		iteraciones = 0;
		return false;
	}
	if(rightHand->getSecondsElapsed() >umbralElapsedTime)
	{
		// If the hand's tag is lost we assume that the internal possition 
		// (according to the direct kinematics) is correct TODO
		std::cout<<"La camara no ve la marca..."<<std::endl;
		rightHand->setVisualPosewithInternal();
	}
	// COMPROBAMOS EL ERROR:
	QVec errorInv = rightHand->getErrorInverse();
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2() < umbralError)
	{
		trueTarget.setState(Target::State::RESOLVED);
		printf("done!\n");
		iteraciones = 0;
		return true;
	}

	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
	qDebug()<<"errorInv en root: "<<errorInvPEnAbsoluto;

	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(QVec::vec3(trueTarget.getPose().rx(), trueTarget.getPose().ry(), trueTarget.getPose().rz()),3);	
	correctedTarget.setPose(correccionFinal);
	qDebug()<<"Posicion  corregida: "<<correctedTarget.getPose();

	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),   correctedTarget.getPose().y(), correctedTarget.getPose().z(), 
									               correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose().rz());
	//Llamamos al BIK con el nuevo target corregido y esperamos
	float radius = 1;
	WeightVector weights; //pesos a cero
	weights.x = 1;     weights.y = 1;    weights.z = 1;
	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
	bodyinversekinematics_proxy->setTargetPose6D(trueTarget.getBodyPart(), correctedTarget.getPose6D(), weights, radius);
	iteraciones++;
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
	static int iteraciones = 0, maxIteraciones = 10;    // Para evitar que se quede atascado.
	float umbralElapsedTime = 0.5, umbralErrorT = 5, umbralErrorR=0.17;

	// If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
	if (rightHand->getSecondsElapsed() > umbralElapsedTime)
	{
		std::cout<<"La camara no ve la marca..."<<std::endl;
		rightHand->setVisualPosewithInternal();
	}
	// COMPROBAMOS EL ERROR:
	QVec errorInv = rightHand->getErrorInverse();
// 	errorInv.print("errorInv");
	if(iteraciones > maxIteraciones)
	{
		abortarotacion = true;
		QVec pose = trueTarget.getPose();
		file<<"P: ("      <<pose.x()<<","<<pose.y()<<","<<pose.z()<<","<<pose.rx()<<","<<pose.ry()<<","<<pose.rz();
		file<<") ERROR_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
		file<<" ERROR_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		file<<" END: "    <<iteraciones<<"-->"<<abortatraslacion<<","<<abortarotacion<<endl;;
		flush(file);
		iteraciones = 0;
		return false;
	}
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR)
	{
		trueTarget.setState(Target::State::RESOLVED);
		printf("done!\n");
		QVec pose = trueTarget.getPose();
		file<<"P: ("      <<pose.x()<<","<<pose.y()<<","<<pose.z()<<","<<pose.rx()<<","<<pose.ry()<<","<<pose.rz();
		file<<") ERROR_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
		file<<" ERROR_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		file<<" END: "    <<iteraciones<<"-->"<<abortatraslacion<<","<<abortarotacion<<endl;
		flush(file);
		iteraciones = 0;
		return true;
	}
	
	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
	qDebug()<<"errorInv en absoluto: "<<errorInvPEnAbsoluto<<errorInv;

	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(QVec::vec3(trueTarget.getPose().rx(), trueTarget.getPose().ry(), trueTarget.getPose().rz()),3);	
	correctedTarget.setPose(correccionFinal);
	qDebug()<<"Correccion final: "<<correctedTarget.getPose();

	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),  correctedTarget.getPose().y(),   correctedTarget.getPose().z(), 
									              correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose6D().rz);
	//Llamamos al BIK con el nuevo target corregido y esperamos
	float radius = 1;
	bodyinversekinematics_proxy->setTargetPose6D(trueTarget.getBodyPart(), correctedTarget.getPose6D(), trueTarget.getWeights6D(), radius);

	iteraciones++;
	return false;
}
	


/**
 * \brief Metodo ACTUALIZAR TODO
 * Se encarga de actualizar la posicion de los motores del robot (el innerModel),
 * la pose del target que le enviamos y la pose de la marca visual que ve.
 */
void SpecificWorker::actualizarTodo()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
	}
	catch (const Ice::Exception &ex){ cout<<"--> Excepción en actualizar InnerModel: (i)";}

	try
	{
		RoboCompOmniRobot::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		try
		{
			innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		}
		catch (const Ice::Exception &ex){ cout<<"--> Exception updating transform values: "<<ex<<endl;}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción reading OmniRobot: "<<ex<<endl;
	}
	
	const Pose6D tt = trueTarget.getPose6D();
	innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	
 	const QVec pR = rightHand->getVisualPose();
 	innerModel->updateTransformValues("visual_hand", pR.x(), pR.y(), pR.z(), pR.rx(), pR.ry(), pR.rz());
	
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								METODOS DE LA INTERFAZ DEL COMPONENTE										   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void SpecificWorker::setFingers(const float d)
{
	bodyinversekinematics_proxy->setFingers(d);
}

void SpecificWorker::setRobot(const int type)
{
	bodyinversekinematics_proxy->setRobot(type);
}

TargetState SpecificWorker::getState(const string &part)
{
	return bodyinversekinematics_proxy->getState(part);
}

void SpecificWorker::setNewTip(const string &part, const string &transform, const Pose6D &pose)
{
	bodyinversekinematics_proxy->setNewTip(part, transform, pose);
}

void SpecificWorker::stop(const string &part)
{
	bodyinversekinematics_proxy->stop(part);
}

void SpecificWorker::goHome(const string &part)
{
	bodyinversekinematics_proxy->goHome(part);
}

void SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights, const float radius)
{
	QMutexLocker ml(&mutex);
	cout<<"Recibido target"<<endl;
	if(trueTarget.getState()==Target::State::IDLE)
	{
		trueTarget.setBodyPart	(bodyPart);
		trueTarget.setPose		(target);
		trueTarget.setWeights	(weights);
		trueTarget.setState		(Target::State::WAITING);
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
}

void SpecificWorker::advanceAlongAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	bodyinversekinematics_proxy->advanceAlongAxis(bodyPart, ax, dist);
}

void SpecificWorker::pointAxisTowardsTarget(const string &bodyPart, const Pose6D &target, const Axis &ax, const bool &axisConstraint, const float axisAngleConstraint)
{
	bodyinversekinematics_proxy->pointAxisTowardsTarget(bodyPart, target, ax, axisConstraint, axisAngleConstraint);
}

void SpecificWorker::setJoint(const string &joint, const float position, const float maxSpeed)
{
	bodyinversekinematics_proxy->setJoint(joint, position, maxSpeed);
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								METODOS A LOS QUE SE SUBSCRIBE												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	//Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	QMutexLocker ml(&mutex);
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
				rightHand->setVisualPose(tag);
			else if (tag.id == 24)
				leftHand->setVisualPose(tag);
		}
	}
}





