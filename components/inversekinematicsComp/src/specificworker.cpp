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
#include <qt4/QtCore/QMutexLocker>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 							CONSTRUCTORES Y DESTRUCTORES DE LA CLASE 								*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief Default Constructor
 */
SpecificWorker::SpecificWorker(MapPrx& mprx, QWidget *parent) : GenericWorker(mprx)	
{	
	correlativeID = 0;		//Unique ID to name provisional targets
	hide();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	fichero.close();
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 							MÉTODOS DE INICIALIZACIÓN DE LA CLASE 									*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief SET PARAMS
 * Method called by the thread Monitor to pass the configuration parmaeters read from the config file
 * TODO Config con la lista de joints de las distintas partes del robot y crear el mapa de las partes
 * del cuerpo del robot AQUÍ.
 * 
 * @param params ...
 * @return bool
 */
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("LEFTARM") ;
// 		
// 		qFatal("FARY");
// 		
// 	}catch(std::exception e) {qFatal("Error al leer la cadena de motores");	}
// 	
	
	try
	{
		// Leemos donde está el InnerModel. Si existe cargamos el InnerModel en nuestra variable de clase
		// y convertimos los milímitros en metros. Si no existe lanzamos mensage de error y salimos.
		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			convertInnerModelFromMilimetersToMeters(innerModel->getRoot());
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
		}
		else
		{	
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}catch(std::exception e) {qFatal("Error reading config params");	}
	

	//timer.start(Period);
	init();
	timer.start(50);
	return true;
}

/**
 * @brief Initializing procedures to be done once params are read
 * 
 * @return void
 */
void SpecificWorker::init()
{
	// RECONFIGURABLE PARA CADA ROBOT: Listas de motores de las distintas partes del robot
	listaBrazoIzquierdo	<< "leftShoulder1" << "leftShoulder2" << "leftShoulder3" << "leftElbow" << "leftForeArm" << "leftWrist1" << "leftWrist2";
	listaBrazoDerecho 	<< "rightShoulder1" << "rightShoulder2" << "rightShoulder3" << "rightElbow"<< "rightForeArm" << "rightWrist1" << "rightWrist2";
	listaCabeza 		<< "head1" << "head2" << "head3";
	listaMotores 		<< "leftShoulder1" << "leftShoulder2" << "leftShoulder3" << "leftElbow" << "leftForeArm" << "leftWrist1" << "leftWrist2"
						<< "rightShoulder1" << "rightShoulder2" << "rightShoulder3" << "rightElbow"<< "rightForeArm" << "rightWrist1" << "rightWrist2"
						<< "base" << "head1" << "head2" << "head3";
	
	// PREPARA LA CINEMATICA INVERSA: necesita el innerModel, los motores y el tip:
	QString tipRight = "grabPositionHandR";
	QString tipLeft = "grabPositionHandL";
	QString nose = "head3";  //OJO PROV
	
	IK_BrazoDerecho = new Cinematica_Inversa(innerModel, listaBrazoDerecho, tipRight);
	IK_BrazoIzquierdo = new Cinematica_Inversa(innerModel, listaBrazoIzquierdo, tipLeft);
	IK_Cabeza = new Cinematica_Inversa(innerModel, listaCabeza, nose);
							 
	// CREA EL MAPA DE PARTES DEL CUERPO: por ahora los brazos.
	bodyParts.insert("LEFTARM", BodyPart(innerModel, IK_BrazoIzquierdo, "LEFTARM", tipLeft, listaBrazoIzquierdo));
	bodyParts.insert("RIGHTARM", BodyPart(innerModel, IK_BrazoDerecho, "RIGHTARM", tipRight, listaBrazoDerecho)); 
	bodyParts.insert("HEAD", BodyPart(innerModel, IK_Cabeza, "HEAD", nose, listaCabeza)); 

	//Initialize proxy to RCIS
	proxy = jointmotor0_proxy;

	actualizarInnermodel(listaMotores);  // actualizamos los ángulos de los motores del brazo derecho
	
	//goHomePosition(listaMotores); 
 	foreach(BodyPart p, bodyParts)
 		goHome(p.getPartName().toStdString());
	sleep(1);
	actualizarInnermodel(listaMotores);
	innerModel->transform("world", QVec::zeros(3),tipRight).print("RightTip in World");
		
	//Open file to write errors
	fichero.open("errores.txt", ios::out);
	
	
	//RRT path-Planning stuff
// 	planner = new Planner(innerModel);
// 	qDebug("Planning ...");
// 	QVec targetToGo;
// 	planner->computePath(target);	
// 	if(planner->getPath().size() == 0)
// 		qFatal("Path NOT found");
// 	QList<QVec> path = planner->getPath();
	
	//Online trajectory generaration with Reflexxes
	
	
	qDebug();
	qDebug() << "---------------------------------";
	qDebug() << "BodyInverseKinematics --> Waiting for requests!";
}


/**
 * @brief Transforms all InnerModel in mm to meters until BIK can operate in meters directly
 * 
 * @param node starting node of InnerModel
 * @return void
 */
void SpecificWorker::convertInnerModelFromMilimetersToMeters(InnerModelNode* node)
{	
	const  float FACTOR = 1000.f;

	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;
	InnerModelJoint *joint;
	
	// Find out which kind of node are we dealing with
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))   //Aquí se incluyen Transform, Joint, PrismaticJoint, DifferentialRobot
	{	
		if( (joint = dynamic_cast<InnerModelJoint *>(node)) == false)
		{
			transformation->setTr(transformation->getTr() / FACTOR);
		}
		//qDebug() << node->id << node->getTr();
		for(int i=0; i<node->children.size(); i++)
		{
			convertInnerModelFromMilimetersToMeters(node->children[i]);
		}
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		plane->point = plane->normal / FACTOR;
		plane->width /= FACTOR; plane->height /= FACTOR; plane->depth /= FACTOR;
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		mesh->tx /= FACTOR; mesh->ty /= FACTOR; mesh->tz /= FACTOR;
		mesh->scalex /= FACTOR; mesh->scaley /= FACTOR; mesh->scalez /= FACTOR;
	}
}		




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										SLOTS DE LA CLASE											*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void SpecificWorker::compute( )				
{
		QMap<QString, BodyPart>::iterator iterador;
		for( iterador = bodyParts.begin(); iterador != bodyParts.end(); ++iterador)
		{	
			if(iterador.value().noTargets() == false)
			{
				Target &target = iterador.value().getHeadFromTargets(); 	
				if (target.isMarkedforRemoval() == false)
				{
					target.annotateInitialTipPose();
					target.setInitialAngles(iterador.value().getMotorList());
					createInnerModelTarget(target);  	//Crear "target" online y borrarlo al final para no tener que meterlo en el xml
					target.print("BEFORE PROCESSING");
					
					iterador.value().getInverseKinematics()->resolverTarget(target);

					if(target.getError() <= 0.9 and target.isAtTarget() == false) //local goal achieved: execute the solution
					{
						moveRobotPart(target.getFinalAngles(), iterador.value().getMotorList());
						//Acumulamos los angulos en una lista en bodyPart para lanzarlos con Reflexx
						iterador.value().addJointStep(target.getFinalAngles());
						usleep(100000);
						target.setExecuted(true);
					}
					else  
					{
						target.markForRemoval(true);
					}
					actualizarInnermodel(listaMotores); 			//actualizamos TODOS los motores.	OJO si la trayectoria no se ejecuta aquí, el Innermodel debe ser restaurado si la acción no se realiza
					target.annotateFinalTipPose();
					removeInnerModelTarget(target);
					target.print("AFTER PROCESSING");
				}
// 				if( target.isChopped() == false)
// 				{
// 					doReflexxes( iterador.value().getJointStepList(), iterador.value().getMotorList());
// 				}
				if(target.isChopped() == false or target.isMarkedforRemoval() == true or target.isAtTarget() )
				{
						mutex->lock();	
 							iterador.value().removeHeadFromTargets(); //eliminamos el target resuelt
							iterador.value().cleanJointStep();
						mutex->unlock();
				}	
				
	
			}
		}
	actualizarInnermodel(listaMotores); //actualizamos TODOS los motores.
}


void SpecificWorker::doReflexxes( const QList<QVec> &jointValues, const QStringList &motors )
{
	Reflexx *reflexx = new Reflexx(proxy, jointValues, motors);
	//reflexx->updateMotorState(motors);
	//reflexx->setSyncPosition( listGoals );
	reflexx->start();
	qDebug() << __FUNCTION__ << "Waiting for Reflexx...";
	reflexx->wait(5000);
}

/**
 * @brief Creates a target element inside InnerModel to be used by IK. Avoids having a "target" in the XML file.
 * 		  Each "target" node in InnerModel is created for each target that arrives here, and deleted when finished.
 * 		  Each bodypart may have a different target and BIK eliminates its dependence of InnerModelManager
 * 
 * @param target ...
 * @return void
 */
void SpecificWorker::createInnerModelTarget(Target &target)
{
	InnerModelNode *nodeParent = innerModel->getNode("world");
	target.setNameInInnerModel(QString::number(correlativeID++));
	InnerModelTransform *node = innerModel->newTransform(target.getNameInInnerModel(), "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	QVec p = target.getPose();
	innerModel->updateTransformValues(target.getNameInInnerModel(),p.x(), p.y(), p.z(), p.rx(), p.ry(), p.rz(), "world");
	
}
void SpecificWorker::removeInnerModelTarget(const Target& target)
{
	innerModel->removeNode(target.getNameInInnerModel());
}


///////////////////////////////////////////////
/// SERVANTS  OJO se ejecuta en el  hilo de ICE
//////////////////////////////////////////////


/**
 * @brief ...
 * 
 * @param bodyPart ...
 * @param target ...
 * @param weights ...
 * @return void
 */
void SpecificWorker::setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights, float radius)
{
	QString partName = QString::fromStdString(bodyPart);
	if ( this->bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	
	//Se debe comprobar condiciones del target cuando las tengamos.
	QVec tar(6);
	tar[0] = target.x;	tar[1] = target.y;	tar[2] = target.z;
	tar[3] = target.rx;	tar[4] = target.ry;	tar[5] = target.rz;

	//PASAMOS A METROS
	tar[0] = tar[0] / (T)1000;
	tar[1] = tar[1] / (T)1000;
	tar[2] = tar[2] / (T)1000;
	
	
	//Weights vector
	QVec w(6);
	w[0]  = weights.x; 	w[1]  = weights.y; w[2]  = weights.z; w[3]  = weights.rx; w[4] = weights.ry; w[5] = weights.rz;

   Target t(Target::POSE6D, innerModel, bodyParts[partName].getTip(), tar, w, false);
   t.setRadius(radius/1000.f);
  
    mutex->lock();
        bodyParts[partName].addTargetToList(t);
    mutex->unlock();
	
	qDebug() << "--------------------------------------------------------------------------";
	qDebug() << __LINE__<< "New target arrived: " << partName;
}

/**
 * @brief ...
 * 
 * @param bodyPart ...
 * @param target ...
 * @param axis ...
 * @param axisConstraint ...
 * @param axisAngleConstraint ...
 * @return void
 */
void SpecificWorker::pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const Axis& axis, bool axisConstraint, float axisAngleConstraint)
{
	QString partName = QString::fromStdString(bodyPart);
	
	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part or incorrect axis";
		throw ex;
	}
	
	//Se deben comprobar condiciones del target cuando las tengamos.
	QVec tar(6);
	tar[0] = target.x;	tar[1] = target.y;	tar[2] = target.z;	tar[3] = target.rx;	tar[4] = target.ry;	tar[5] = target.rz;

	//PASAMOS A METROS
	tar[0] = tar[0] / (T)1000;
	tar[1] = tar[1] / (T)1000;
	tar[2] = tar[2] / (T)1000;
	
	//Weights vector ONLY ROTATION
	QVec w(6);
	w[0] = 0; w[1] = 0; w[2] = 0; w[3] = 1; w[4] = 1; w[5] = 1;
	
	QVec ax = QVec::vec3( axis.x , axis.y, axis.z);
	
	Target t(Target::ALIGNAXIS, innerModel, bodyParts[partName].getTip(), tar, ax, w);
	
	mutex->lock();
		bodyParts[partName].addTargetToList(t);
	mutex->unlock();
	
	qDebug() << "-----------------------------------------------------------------------";
	qDebug() << __FUNCTION__ << __LINE__<< "New target arrived: " << partName;
}


/**
 * @brief Make the body part advance along a given direction. It is meant to work as a simple translational joystick to facilitate grasping operations
 * 
 * @param bodyPart ...
 * @param ax ...
 * @param dist step to advance un milimeters
 * @return void
 */
void SpecificWorker::advanceAlongAxis(const string& bodyPart, const Axis& ax, float dist)
{
	QString partName = QString::fromStdString(bodyPart);
	
	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	
	//Code axis as a dist norm vector in tip reference frame
	QVec axis = QVec::vec3(	ax.x, ax.y,	ax.z).normalize();
	
	//Bigger jump admisible
	if(dist > 300) dist = 300;
	if(dist < -300) dist = -300;
	dist = dist / 1000.;   //PASANDO A METROS
	
	Target t(Target::ADVANCEAXIS, innerModel, bodyParts[partName].getTip(), axis, dist);
	
	mutex->lock();
		bodyParts[partName].addTargetToList(t);
	mutex->unlock();
	
	qDebug() << "-----------------------------------------------------------------------";
	qDebug() <<  __FILE__ << __FUNCTION__ << __LINE__<< "New target arrived: " << partName;
}

/**
 * @brief Set the fingers position
 * 
 * @param d ...
 * @return void
 */
void SpecificWorker::setFingers(float d)
{
	qDebug() << __FUNCTION__;
	
	//float len = 0.07; // check
	//float len = innerModel->getTranslationVectorTo("rightFinger1","finger_right_1_1_tip").norm2();   //CONSULTAR CON LUIS POR QUÉ NO VA Y EL DE ABAJO SI
	float len = innerModel->transform("rightFinger1", QVec::zeros(3), "finger_right_1_1_tip").norm2();
	//qDebug() << "len" << len;
	float D = (d/1000)/2.; 			// half distnace in meters
	float s = D/len;
	if( s > 1) s = 1;
	if( s < -1) s = -1;
	float ang = asin(s); 	// 1D inverse kinematics
	QVec angles = QVec::vec2( ang - 1, -ang + 1);
		
	/// OJO!!! DO THAT WITH innerModel->getNode("rightFinger1")->min
	QStringList joints;
	joints << "rightFinger1" << "rightFinger2";
	moveRobotPart(angles, joints);
	
	// fingerRight1, fingerRight2 are the joints going from -1 to 0 (left) and from 1 to 0 (right)
	// se anclan en "arm_right_8"
	// necesitamos la longitud total desde fingerRightX hasta la punta (l)
	// entonces D sería la distancia desde el eje central hasta la punta y se calcularía como D = l*sin(ang) where and debe ser 0 para D = 0 y pi/2 para D=l
	// ang = ang1 + 1 y ang = -ang2+1
	// dedo izquierdo: D=l*sin(ang)
	// dedo derecho: D=l*sin(ang)
	// Ahora cinemática inversa: ang = asin(D/l)
	// ang1  = ang - 1;
	// ang2 = -ang + 1;
	
}

/**
 * @brief ...
 * 
 * @param part ...
 * @return void
 */
void SpecificWorker::goHome(const string& part)
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
	for(int i=0; i<lmotors.size(); i++){
		try {
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = lmotors.at(i).toStdString();
			nodo.position = innerModel->getJoint(lmotors.at(i))->home;
			nodo.maxSpeed = 5; //radianes por segundo
			mutex->lock();
				proxy->setPosition(nodo);
			mutex->unlock();
			
		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;	
		}
	}
	//goHomePosition( bodyParts[partName].getMotorList());
	//sleep(1);
	
}

/**
 * @brief Sets the variable Proxy to the robot or RCIS Ice proxy
 *
 * @param type ...
 * @return void
 */
void SpecificWorker::setRobot(const int t)
{
	mutex->lock();
	this->typeR = t;
	if (this->typeR == 0)
		proxy = jointmotor0_proxy;
	else if (this->typeR == 1)
		proxy = jointmotor1_proxy;
	mutex->unlock();
}


/**
 * @brief 
 * 
 * @param part ...
 * @return RoboCompBodyInverseKinematics::Statedesigner
 * 
 */
TargetState SpecificWorker::getState(const std::string &part)
{
	TargetState state;
	state.finish = false;
	state.elapsedTime = 0;
	state.estimatedEndTime = -1;
	
	QString partName = QString::fromStdString(part);
	
	if ( bodyParts.contains(partName) == false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
 		qDebug() << "----------------------------------------";
 		qDebug() << "New COMMAND arrived "<< __FUNCTION__ << partName;

		mutex->lock();
		if( bodyParts[partName].getTargets().isEmpty() )
			state.finish = true;
		else
			state.elapsedTime = bodyParts[partName].getHeadFromTargets().getElapsedTime();
		mutex->unlock();
	}
	return state;	
}


/**
 * @brief Stops robot part and cleans it queue
 * 
 * @param part ...
 * @return void
 */
void SpecificWorker::stop(const std::string& part)
{
	QString partName = QString::fromStdString(part);
	
	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
		qDebug() << "-------------------------------------------------------------------";
		qDebug() << "New COMMAND arrived " << __FUNCTION__ << QString::fromStdString(part);
		mutex->lock();
			bodyParts[partName].markForRemoval();
		mutex->unlock();
	}	
}


/**
 * @brief Changes the current tip of the corresponding part
 * 
 * @param part ...
 * @return void
 */
void SpecificWorker::setNewTip(const std::string &part, const std::string &transform, const Pose6D &pose)
{
	QString partName = QString::fromStdString(part);
	QString transformName = QString::fromStdString(transform);

	//Check if tip is a valid new tip
	if (bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
		qDebug() << "-------------------------------------------------------------------";
		qDebug() << "New COMMAND arrived " << __FUNCTION__ << QString::fromStdString(part);

		mutex->lock();
		//bodyParts[partName].setNewVisualTip(pose);
		//innerModel->transform("world", QVec::zeros(3),part).print("antes setNewTip");
		innerModel->updateTransformValues(transformName, pose.x/1000., pose.y/1000., pose.z/1000., pose.rx, pose.ry, pose.rz);
		//innerModel->updateTransformValues( "grabPositionHandR", pose.x/1000., pose.y/1000., pose.z/1000., 0,0,0);	
		
		//innerModel->transform("world", QVec::zeros(3), part).print("despues setNewTipo");
		mutex->unlock();
	}
}



/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    PRIVADOS                                *
 *-----------------------------------------------------------------------------*/ 


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    ACTUALIZADORES                          *
 *-----------------------------------------------------------------------------*/ 
/*
 * Método actualizarInnermodel
 * Actualiza el InnerModel con las nuevas posiciones de los motores del robot.  
 * FUNCIONA.
 */ 
void SpecificWorker::actualizarInnermodel(const QStringList &listaJoints)
{
	try 
	{
		MotorList mList;
		for(int i=0; i<listaJoints.size(); i++)
			mList.push_back(listaJoints[i].toStdString());
		
		RoboCompJointMotor::MotorStateMap mMap = proxy->getMotorStateMap(mList);
		
		for(int j=0; j<listaJoints.size(); j++)
			innerModel->updateJointValue(listaJoints[j], mMap.at(listaJoints[j].toStdString()).pos);

	} catch (const Ice::Exception &ex) {
		cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;
	}
}


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS PARA MOVER                                 *
 *-----------------------------------------------------------------------------*/ 


/*
 * Método moverBrazo.
 * Mueve el brazo cambiando los ángulos que forman con el eje X 
 * (el primer segmento) y con el primer segmento (el segundo segmento).
 * FUNCIONA.
 */ 
void SpecificWorker::moveRobotPart(QVec angles, const QStringList &listaJoints)
{
	for(int i=0; i<angles.size(); i++)
	{
		try 
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = listaJoints.at(i).toStdString();			
			nodo.position = angles[i]; // posición en radianes
			nodo.maxSpeed = 3; //radianes por segundo TODO Bajar velocidad.
			proxy->setPosition(nodo);	
		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;	
		}
	}
}


/*
 * Método moverTarget versión 2.
 * Mueve el target a una posición que se le pasa como parámetro de entrada. 
 * Crea una pose3D a cero y actualiza sus traslaciones tx, ty y tz y sus 
 * rotaciones rx, ry y rz con los datos del parámetro de entrada.
 * Sirve para colocar el target en el innerModel. Para nada más.
 */ 
// void SpecificWorker::moverTarget(const QVec &pose)
// {
// 	try
// 	{
// 		RoboCompInnerModelManager::Pose3D p;
// 		p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.
// 
// 		p.x = pose[0]; p.y = pose[1]; p.z = pose[2];
// 		p.rx = pose[3]; p.ry = pose[4]; p.rz = pose[5];
// 			
// 		innermodelmanager_proxy->setPoseFromParent("target",p);
// 		innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);
// 		}
// 	catch (const Ice::Exception &ex) 
// 	{
// 		cout<<"Excepción en moverTarget: "<<ex<<endl;
// 	}
// }



/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    AUXILIARES                              *
 *-----------------------------------------------------------------------------*/ 
/**
 * @brief ...
 * 
 * @param t ...
 * @return float
 */
float SpecificWorker::standardRad(float t)
{
	if (t >= 0.) {
		t = fmod(t+M_PI, M_PI/2.) - M_PI;
	} else {
		t = fmod(t-M_PI, -M_PI/2.) + M_PI;
	}
	return t;
}


void SpecificWorker::calcularModuloFloat(QVec &angles, float mod)
{
    for(int i=0; i<angles.size(); i++)
    {
        int cociente = (int)(angles[i] / mod);
        angles[i] = angles[i] -(cociente*mod);

        if(angles[i] > M_PI)
            angles[i] = angles[i]- M_PI;
        else
            if(angles[i] < -M_PI)
                angles[i] = angles[i] + M_PI;
    }
}

/*
 * Método getRotacionMano
 * Devuelve la rotación de la mano del robot
 */ 
// QVec SpecificWorker::getRotacionMano (QString puntaMano)
// {
// 	QMat matriz = innerModel->getRotationMatrixTo("world", puntaMano);
// 	QVec ManoEnMundo = innerModel->getTransformationMatrix("world", puntaMano).extractAnglesR3(matriz);
// 	QVec angulos1 = QVec::vec3(ManoEnMundo[0], ManoEnMundo[1], ManoEnMundo[2]);
// 	QVec angulos2 = QVec::vec3(ManoEnMundo[3], ManoEnMundo[4], ManoEnMundo[5]);
// 	QVec rot;
// 	if(angulos1.norm2() < angulos2.norm2())
// 		rot = angulos1;
// 	else
// 		rot = angulos2;
// 	
// 	return rot;
// }



// /*
//  * Método moverTarget.
//  * Mueve el target (la esfera objetivo del innerModel) a una posición guardada 
//  * en la lista de posiciones de Targets listaTargets. Crea una pose a cero y le
//  * actualiza las traslaciones tx, ty y tz y las rotaciones rx, ry y rz con los 
//  * datos almacenados en la lista.
//  */ 
// void SpecificWorker::moverTarget(int contador)
// {
// 	if(contador < listaPosicionTarget.size())
// 	{
// 		try{
// 			RoboCompInnerModelManager::Pose3D p;
// 			p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.
// 			ptarget = listaPosicionTarget.at(contador); //sacamos posicion del target de la lista de posiciones.
// 		
// // 			p.x = ptarget[0]; p.y = ptarget[1]; p.z = ptarget[2];
// // 			p.rx = ptarget[3]; p.ry = ptarget[4]; p.rz = ptarget[5];
// 			innerModel->transform("world", QVec::zeros(3), "tip");
// 			p.x = 0.35; p.y = 0.8; p.z = 0.2;
// 			p.rx = 0; p.ry = 0; p.rz = 0; //rot a 0
// 				
// 			innermodelmanager_proxy->setPoseFromParent("target",p);
// 			innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);
// 			
// 			}catch (const Ice::Exception &ex) {
// 				cout<<"Excepción en moverTarget: "<<ex<<endl;
// 			}
// 	}
// 	else
// 		qDebug()<<"ERROR al mover target: Fuera de la lista de posiciones";
// }

// /*
//  * Metodo goHomePosition.
//  * Lleva al brazo a una posicion determinada para comenzar a moverlo.
//  */ 
// void SpecificWorker::goHomePosition(const QStringList &listaJoints )
// {
// 	
// 	for(int i=0; i<listaJoints.size(); i++){
// 		try {
// 			RoboCompJointMotor::MotorGoalPosition nodo;
// 			nodo.name = listaJoints.at(i).toStdString();
// 			nodo.position = innerModel->getJoint(listaJoints.at(i))->home;
// 			nodo.maxSpeed = 5; //radianes por segundo
// 			proxy->setPosition(nodo);
// 			
// 		} catch (const Ice::Exception &ex) {
// 			cout<<"Excepción en mover Brazo: "<<ex<<endl;	
// 		}
// 	}
// }
// 
