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
	//Iniciamos con estado igual a IDLE:
	this->stateMachine = State::IDLE;
	
 	this->innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ursus_bik.xml");

	this->rightHand = new VisualHand(this->innerModel, "grabPositionHandR");
	this->leftHand = new VisualHand(this->innerModel, "grabPositionHandL");
	
#ifdef USE_QTGUI
	this->osgView = new OsgView(this);
	this->innerViewer = new InnerModelViewer(this->innerModel, "root", this->osgView->getRootGroup(), true);
	show();
#endif

	InnerModelNode *nodeParent = this->innerModel->getNode("root");
	if( this->innerModel->getNode("target") == NULL)
	{
		InnerModelTransform *node = this->innerModel->newTransform("target", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
	}

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
 * del componente.
 * @param params mapa de parametros
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			this->innerModel = new InnerModel(par.value);
// 		}
// 		else
// 			qFatal("IMpossible read the innerModel file. Exit now.");
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	timer.start(Period);

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
		if (this->innerViewer)
		{
			this->innerViewer->update();
			this->osgView->autoResize();
			this->osgView->frame();
			return;
		}
#endif
	}
	this->actualizarInnermodel();
	const Pose6D tt = this->trueTarget.getPose6D();
	this->innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
 	const Pose6D p = *(this->rightHand);
 	this->innerModel->updateTransformValues("visual_hand", p.x, p.y, p.z, p.rx, p.ry, p.rz);

	QMutexLocker ml(&this->mutex);
	switch(this->stateMachine)
	{
		case State::IDLE:
			// Estado inicial en espera hasta que trueTarget tenga un valor aceptable y este esperando a
			// ser ejecutado. Cambiamos a INIT_BIK
			if (this->trueTarget.getState() == Target::State::WAITING)
			{
				this->stateMachine = State::INIT_BIK;
				printf("Ha llegado un TARGET: (%f, %f, %f,   %f, %f, %f)\n",tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
			}
		break;
		//---------------------------------------------------------------------------------------------	
		case State::INIT_BIK:
			// Nos da igual si el target tiene en cuenta las rotaciones o las traslaciones, lo enviamos al BIK
			// que ya se encargará de manejar los dos casos.
			try
			{
				this->bodyinversekinematics_proxy->setTargetPose6D(this->trueTarget.getBodyPart(), this->trueTarget.getPose6D(), this->trueTarget.getWeights(), 250); 
			}
			catch (const Ice::Exception &ex) { std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT BIK: "<<ex<<std::endl;}
			
			this->trueTarget.changeState(Target::State::IN_PROCESS); // El trueTarget pasa a estar siendo ejecutado:
			this->corregir = this->trueTarget;
			this->stateMachine = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------				
		case State::WAIT_BIK:
			// Espermos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(this->bodyinversekinematics_proxy->getState(this->trueTarget.getBodyPart()).finish == true)
			{
				qDebug()<<"---> El BIK ha terminado.";

				const WeightVector weights = this->trueTarget.getWeights();
				if(weights.rx==0 and weights.ry==0 and weights.rz==0)
					this->stateMachine = State::CORRECT_TRASLATION;
				else
					this->stateMachine = State::CORRECT_ROTATION;
			}
		break;
		//---------------------------------------------------------------------------------------------						
		case State::CORRECT_TRASLATION:
 			if(this->correctTraslation()==true)
			{
				// Si la correccion ha terminado y hay un target esperando
				if (this->nextTarget.getState() == Target::State::WAITING)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					this->trueTarget = this->nextTarget;
					this->nextTarget.changeState(Target::State::IDLE);
				}
				else
					this->trueTarget.changeState(Target::State::IDLE);
				this->stateMachine = State::IDLE;
			}	
		break;
		//---------------------------------------------------------------------------------------------	
		case State::CORRECT_ROTATION:
			
			if(this->correctRotation()==true)
			{
				// Si la correccion ha terminado y hay un target esperando
				if (this->nextTarget.getState() == Target::State::WAITING)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					this->trueTarget = this->nextTarget;
					this->nextTarget.changeState(Target::State::IDLE);
				}
				else
					this->trueTarget.changeState(Target::State::IDLE);
				this->stateMachine = State::IDLE;
			}	
			
		break;
		//---------------------------------------------------------------------------------------------			
		default:
			break;
	}

#ifdef USE_QTGUI
	if (this->innerViewer)
	{
		this->innerViewer->update();
		this->osgView->autoResize();
		this->osgView->frame();
	}
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 											METODOS PRIVADOS												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo CORRECT TRASLATION.
 * Se encarga de corregir los errores de traslacion del tip del robot (donde esta realmente la mano). Calcula el
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
	float umbralElapsedTime = 1.5; //un segundo y medio.
	float umbralError = 5;
	
	//Si hemos perdido la marca decimos que marca = internalPose.
	if(this->rightHand->secondsElapsed() > umbralElapsedTime)
	{
		std::cout<<"La camara no ve la marca..."<<std::endl;
		this->rightHand->x = this->rightHand->getInternalPose().x;
		this->rightHand->y = this->rightHand->getInternalPose().y;
		this->rightHand->z = this->rightHand->getInternalPose().z;
		this->rightHand->rx = this->rightHand->getInternalPose().rx;
		this->rightHand->ry = this->rightHand->getInternalPose().ry;
		this->rightHand->rz = this->rightHand->getInternalPose().rz;
	}
	
	// COMPROBAMOS EL ERROR:
	Pose6D error = this->rightHand->getError(this->trueTarget.getPose6D());
	error.rx = error.ry = error.rz = 0; //anumalos los errores de rotacion
		
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	QVec eP = QVec::vec3(error.x, error.y, error.z);
	QVec eR = QVec::vec3(error.rx, error.ry, error.rz);
	printf("NORMA: %f\n",eP.norm2());
	if(eP.norm2()< umbralError)
	{
		this->trueTarget.changeState(Target::State::RESOLVED);
		printf("done!\n");
		return true;
	}

	if (eP.norm2() > 80)
		eP.operator*(0.3); //para que vaya despacito
	else
		eP.operator*(1.5); //para que vaya despacito
	
	eP.print("correccion");
	// Corregimos error de traslacion entre la pose interna (la que cree) y el target:
	Pose6D aux; 
	aux.x = corregir.getPose6D().x + eP(0);
	aux.y=  corregir.getPose6D().y + eP(1);
	aux.z = corregir.getPose6D().z + eP(2);
	
	corregir.changePose6D(aux); 

	//Llamamos al BIK con el nuevo target corregido y esperamos
	this->bodyinversekinematics_proxy->setTargetPose6D(this->trueTarget.getBodyPart(), corregir.getPose6D(), this->trueTarget.getWeights(), 250);
	while(this->bodyinversekinematics_proxy->getState(this->trueTarget.getBodyPart()).finish != true);
	
	//Tomo la pose internal del robot
	QVec pose = this->innerModel->transform("root", "grabPositionHandR");
	Pose6D internalPose;
	internalPose.x = pose.x();
	internalPose.y = pose.y();
	internalPose.z = pose.z();
	internalPose.rx = pose.rx();
	internalPose.ry = pose.ry();
	internalPose.rz = pose.rz();
	this->rightHand->setInternalPose(internalPose);
	
	return false;
}

/**
 * Metodo CORRECT ROTATION
 * Corrige la posicion de la mano en traslacion y en rotacion.
 */ 
bool SpecificWorker::correctRotation()
{
	float umbralElapsedTime = 1.; //un segundo y medio.
	float umbralError = 5;
	
	//Si hemos perdido la marca decimos que marca = internalPose.
	if(this->rightHand->secondsElapsed() > umbralElapsedTime)
	{
		std::cout<<"La camara no ve la marca..."<<std::endl;
		this->rightHand->x = this->rightHand->getInternalPose().x;
		this->rightHand->y = this->rightHand->getInternalPose().y;
		this->rightHand->z = this->rightHand->getInternalPose().z;
		this->rightHand->rx = this->rightHand->getInternalPose().rx;
		this->rightHand->ry = this->rightHand->getInternalPose().ry;
		this->rightHand->rz = this->rightHand->getInternalPose().rz;
	}
	
	// COMPROBAMOS EL ERROR:
	Pose6D error = this->rightHand->getError(this->trueTarget.getPose6D());
		
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	QVec eP = QVec::vec3(error.x, error.y, error.z);
	QVec eR = QVec::vec3(error.rx, error.ry, error.rz);
	printf("NORMA: %f\n",eP.norm2());
	if(eP.norm2()< umbralError and eR.norm2()<0.3)
	{
		this->trueTarget.changeState(Target::State::RESOLVED);
		printf("done!\n");
		return true;
	}

// 	if (eP.norm2() > 80)
// 		eP.operator*(0.9); //para que vaya despacito
// 	else
		eP.operator*(0.5); //para que vaya despacito
// 	

eP.print("correccion T");
	eR.print("correccion R");
	// Corregimos error de traslacion entre la pose interna (la que cree) y el target:
	Pose6D aux; 
	aux.x = corregir.getPose6D().x + eP(0);
	aux.y = corregir.getPose6D().y + eP(1);
	aux.z = corregir.getPose6D().z + eP(2);

	
	Rot3D c(corregir.getPose6D().rx, corregir.getPose6D().ry, corregir.getPose6D().rz);
	Rot3D me(0, 0, 0);
	QVec anglesC = (c * me.invert()).extractAnglesR_min();

	aux.rx = anglesC(0);
	aux.ry = anglesC(1);
	aux.rz = anglesC(2);

	corregir.changePose6D(aux); 

	printf("Mandamos al BIK: %f %f %f          %f %f %f\n", corregir.getPose6D().x, corregir.getPose6D().y, corregir.getPose6D().z, corregir.getPose6D().rx, corregir.getPose6D().ry, corregir.getPose6D().rz);
	//Llamamos al BIK con el nuevo target corregido y esperamos
	this->bodyinversekinematics_proxy->setTargetPose6D(this->trueTarget.getBodyPart(), corregir.getPose6D(), this->trueTarget.getWeights(), 250);
	while(this->bodyinversekinematics_proxy->getState(this->trueTarget.getBodyPart()).finish != true);
	
	//Tomo la pose internal del robot
	QVec pose = this->innerModel->transform("root", "grabPositionHandR");
	Pose6D internalPose;
	internalPose.x = pose.x();
	internalPose.y = pose.y();
	internalPose.z = pose.z();
	internalPose.rx = pose.rx();
	internalPose.ry = pose.ry();
	internalPose.rz = pose.rz();
	this->rightHand->setInternalPose(internalPose);
	
	return false;



}

/**
 * \brief Metodo ACTUALIZAR INNERMODEL 
 * Se encarga de actualizar la posicion de los motores del robot.
 */ 
void SpecificWorker::actualizarInnermodel()
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
	
	//AÑADIDO: ACTUALIZAMOS LA POSE INTERNA DE LA MANO DEL ROBOT
	QVec pose = this->innerModel->transform("root", "grabPositionHandR");
	Pose6D internalPose;
	internalPose.x = pose.x();		internalPose.y = pose.y(); 		internalPose.z = pose.z();
	internalPose.rx = pose.rx();	internalPose.ry = pose.ry();	internalPose.rz = pose.rz();
	this->rightHand->setInternalPose(internalPose);
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
	//if (this->stateMachine == State::IDLE)
	//{
		cout<<"Recibido target"<<endl;
		if(this->trueTarget.getState()==Target::State::IDLE)	
		{
			this->trueTarget.changeBodyPart(bodyPart);
			this->trueTarget.changePose6D(target);
			this->trueTarget.changeWeights(weights);
			this->trueTarget.changeState(Target::State::WAITING);
		}
		else
		{
			this->nextTarget.changeBodyPart(bodyPart);
			this->nextTarget.changePose6D(target);
			this->nextTarget.changeWeights(weights);
			this->nextTarget.changeState(Target::State::WAITING);
		}
	//}
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
	for (auto tag : tags)
	{
		if (tag.id == 25)
		{
			//printf("RIGHT HAND SEEN: (tag id %d)\n", tag.id);
			QMutexLocker l(&this->mutex);
			
			//POSE VISUAL:
			this->rightHand->setVisualPose_APRIL(tag);
			
			//POSE INTERNA:
			//Calculamos la pose interna de la mano en este momento
			QVec pose = this->innerModel->transform("root", "grabPositionHandR");
			Pose6D internalPose;
			internalPose.x = pose.x();
			internalPose.y = pose.y();
			internalPose.z = pose.z();
			internalPose.rx = pose.rx();
			internalPose.ry = pose.ry();
			internalPose.rz = pose.rz();
			this->rightHand->setInternalPose(internalPose);
		}
		else if (tag.id == 24)
		{
			//printf("LEFT HAND SEEN: (tag id %d)\n", tag.id);
			QMutexLocker l(&this->mutex);
			
			//POSE VISUAL:
			this->leftHand->setVisualPose_APRIL(tag);
			
			//POSE INTERNA:
			//Calculamos la pose interna de la mano en este momento
			QVec pose = this->innerModel->transform("root", "grabPositionHandL");
			Pose6D internalPose;
			internalPose.x = pose.x();
			internalPose.y = pose.y();
			internalPose.z = pose.z();
			internalPose.rx = pose.rx();
			internalPose.ry = pose.ry();
			internalPose.rz = pose.rz();
			this->leftHand->setInternalPose(internalPose);
		}
	}
}





