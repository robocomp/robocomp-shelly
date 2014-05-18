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
#include "generador.h"

/**
* \brief Default constructor
* COSAS QUE HACE:
* 			- 0) Abre el fichero.
* 			- 1) Inicializa el innerModel y lo actualiza
* 			- 2) Crea la lista de motores del brazo.
* 			- 3) Crea el generador de puntos para target y de ángulos.
* 			- 4) Crea la lista de ángulos del brazo y mueve el brazo a la primera posición.
* 			- 5) Crea la lista de posiciones del target alredor de la primera posición del brazo.
* 			- 6) Coloca el target en el primer destino.
* 			- 7) Inicializa la cinematica inversa IK.
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{	
	qDebug()<<"\n--------------- CONSTRUCTOR ------------------\n";
	
	// RECONFIGURABLE PARA CADA ROBOT: Listas de motores de las distintas partes del robot
	listaBrazoIzquierdo <<"leftShoulder1"<<"leftShoulder2"<<"leftShoulder3"<<"leftElbow"<<"leftForeArm"<<"leftWrist2"<<"leftWrist3";
	listaBrazoDerecho <<"rightShoulder1"<<"rightShoulder2"<<"rightShoulder3"<<"rightElbow"<<"rightForeArm"<<"rightWrist2"<<"rightWrist3";
	listaCabeza << " ";
	listaMotores <<"rightShoulder1"<<"rightShoulder2"<<"rightShoulder3"<<"rightElbow"<<"rightForeArm"<<"rightWrist1"<<"rightWrist2"
							 <<"rightWrist3"<<"leftShoulder1"<<"leftShoulder2"<<"leftShoulder3"<<"leftElbow"<<"leftForeArm"<<"leftWrist1"
							 <<"leftWrist2"<<"leftWrist3";
							 
 	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus2Metros.xml");

	// PREPARA LA CINEMATICA INVERSA: necesita el innerModel, los motores y el tip:
	QString tipRight = "grabPositionHandR";
	QString tipLeft = "grabPositionHandL";
	IK_BrazoDerecho = new Cinematica_Inversa(innerModel, listaBrazoDerecho, tipRight);
	IK_BrazoIzquierdo = new Cinematica_Inversa(innerModel, listaBrazoIzquierdo, tipLeft);
			
	// CREA EL MAPA DE PARTES DEL CUERPO: por ahora los brazos.
	bodyParts.insert("LEFTARM", BodyPart(innerModel, IK_BrazoIzquierdo, "LEFTARM", tipLeft, listaBrazoIzquierdo));
	bodyParts.insert("RIGHTARM", BodyPart(innerModel, IK_BrazoDerecho, "RIGHTARM", tipRight, listaBrazoDerecho)); 
										 
printf("%s: %d\n", __FILE__, __LINE__);
	actualizarInnermodel(listaMotores);  // actualizamos los ángulos de los motores del brazo derecho
printf("%s: %d\n", __FILE__, __LINE__);
	
	goHomePosition(listaMotores); sleep(2);
	actualizarInnermodel(listaMotores);
		
	//Open file to write errors
	fichero.open("");
	
	//CREA LA LISTA DE TARGETS PARA CADA PARTE DEL CUERPO: necesita innerModel y el bodypart
	listaTargetsBrazoDerecho = generador.generarListaTargets(innerModel, bodyParts.value("RIGHTARM"));
	bodyParts["RIGHTARM"].addListaTarget(listaTargetsBrazoDerecho);
	
	listaTargetsBrazoIzquierdo = generador.generarListaTargets(innerModel, bodyParts.value("LEFTARM")); //añadido
	bodyParts["LEFTARM"].addListaTarget(listaTargetsBrazoIzquierdo);
		
	qDebug()<<"\n---------------FIN CONSTRUCTOR ------------------\n";
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	fichero.close();
}

void SpecificWorker::compute( )
{
	mutex->lock();
	
	// SIN ITERADOR :
// 		for(int i=0; i<2; i++) //desde i=0 hasta 1
// 		{
// 			// Preparamos para sacar la parte del cuerpo que corresponda: Si 0--> la izquierda, si no 0--> la derecha.
// 			QString parte;	
// 			if(i==0) parte = "LEFTARM";  
// 			else parte = "RIGHTARM";
// 			
// 			// Hacemos la cinemática inversa para esa parte:
// 			if(bodyParts[parte].getListaTargets().isEmpty() == false)
// 			{
// 				Target target = bodyParts[parte].getHeadFromListaTargets(); //miramos el primer target.
// 				moverTarget(target.getPose()); // colocamos el target en su posición 8en el innermodel).
// 
// 				QVec angulos = bodyParts[parte].getInverseKinematics()->resolverTarget(target.getPose());
// 				moverBrazo(angulos, bodyParts[parte].getMotorList());
// 				usleep(100000);
// 				actualizarInnermodel(listaMotores); //actualizamos TODOS los motores.
// 				
// 				bodyParts[parte].removeHeadFromListaTargets(); //eliminamos el target resuelto.
// 				
// 				float error = bodyParts[parte].getInverseKinematics()->devolverError();
// 				fichero <<error;
// 			}
// 		}
	
	// CON ITERADOR:
	 QMapIterator<QString, BodyPart> iterador (bodyParts);
	 while (iterador.hasNext())
	 {
		 iterador.next();
		 
		  if(bodyParts[iterador.value().getPartName()].getListaTargets().isEmpty() == false)
      {
				Target target = bodyParts[iterador.value().getPartName()].getHeadFromListaTargets(); //miramos el primer target.
        moverTarget(target.getPose()); // colocamos el target en su posición 8en el innermodel).
        
        QVec angulos = bodyParts[iterador.value().getPartName()].getInverseKinematics()->resolverTarget(target.getPose());
        moverBrazo(angulos, bodyParts[iterador.value().getPartName()].getMotorList());
        usleep(100000);
        actualizarInnermodel(listaMotores); //actualizamos TODOS los motores.
               
        bodyParts[iterador.value().getPartName()].removeHeadFromListaTargets(); //eliminamos el target resuelto.
               
        float error = bodyParts[iterador.value().getPartName()].getInverseKinematics()->devolverError();
        fichero <<error;
			}
	 }
	 
	mutex->unlock();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//timer.start(Period);
	timer.start(0);
	
	return true;
};

///////////////////////////////////////////////
/// SERVANTS  OJO se ejecuta en el  hilo de ICE
//////////////////////////////////////////////
bool SpecificWorker::setTarget(const std::string& bodyPart, const Pose6D& target)
{
	QString partName;
	BodyPart bodypart;
	if ( bodyParts.contains(QString::fromStdString(bodyPart)))
	{
		partName = QString::fromStdString(bodyPart);
		bodypart = bodyParts.value(partName);
	}
	else
	{
		qDebug() << "Not recognized body part";
		return false;
	}
	
	//Se debe comprobar condiciones del target cuando las tengamos.
	QVec tar(6);
	tar[0] = target.x;
	tar[1] = target.y;
	tar[2] = target.z;
	tar[3] = target.rx;
	tar[4] = target.ry;
	tar[5] = target.rz;
	
	Target t(innerModel, tar, bodypart.getTip());
	t.setActivo(true);
	
	mutex->lock();
		if(bodyPart=="RIGHTARM")
			listaTargetsBrazoDerecho.enqueue( t );
	mutex->unlock();
	
	qDebug() << "New target arrived: " << partName;
	tar.print("target in world coordinates");
	
	return true;
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
		{
			printf("AAA%s\n", listaJoints[i].toStdString().c_str());
			mList.push_back(listaJoints[i].toStdString());
		}

		RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap(mList);
		
		for(int j=0; j<listaJoints.size(); j++)
		{
			printf("BBB %s\n", listaJoints[j].toStdString().c_str());
			innerModel->updateJointValue(listaJoints[j], mMap.at(listaJoints[j].toStdString()).pos);
		}
	}
	catch (const Ice::Exception &ex)
	{
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
void SpecificWorker::moverBrazo(QVec angles, const QStringList &listaJoints)
{
	for(int i=0; i<angles.size(); i++)
	{
		try {
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = listaJoints.at(i).toStdString();			
			nodo.position = angles[i]; // posición en radianes
			nodo.maxSpeed = 5; //radianes por segundo TODO Bajar velocidad.
			jointmotor_proxy->setPosition(nodo);
			
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
void SpecificWorker::moverTarget(const QVec &pose)
{
	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.

		p.x = pose[0]; p.y = pose[1]; p.z = pose[2];
		p.rx = pose[3]; p.ry = pose[4]; p.rz = pose[5];
			
		innermodelmanager_proxy->setPoseFromParent("target",p);
		innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);
		
		}
	catch (const Ice::Exception &ex) 
	{
		cout<<"Excepción en moverTarget: "<<ex<<endl;
	}
}


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
// 

/*
 * Metodo goHomePosition.
 * Lleva al brazo a una posicion determinada para comenzar a moverlo.
 */ 
void SpecificWorker::goHomePosition(const QStringList &listaJoints )
{
	
	for(int i=0; i<listaJoints.size(); i++){
		try {
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = listaJoints.at(i).toStdString();
			nodo.position = innerModel->getJoint(listaJoints.at(i))->home;
			nodo.maxSpeed = 5; //radianes por segundo
			jointmotor_proxy->setPosition(nodo);
			
		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;	
		}
	}
}


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    AUXILIARES                              *
 *-----------------------------------------------------------------------------*/ 
/*
 * Método getRotacionMano
 * Devuelve la rotación de la mano del robot
 */ 
QVec SpecificWorker::getRotacionMano (QString puntaMano)
{
	QMat matriz = innerModel->getRotationMatrixTo("world", puntaMano);
	QVec ManoEnMundo = innerModel->getTransformationMatrix("world", puntaMano).extractAnglesR3(matriz);
	QVec angulos1 = QVec::vec3(ManoEnMundo[0], ManoEnMundo[1], ManoEnMundo[2]);
	QVec angulos2 = QVec::vec3(ManoEnMundo[3], ManoEnMundo[4], ManoEnMundo[5]);
	QVec rot;
	if(angulos1.norm2() < angulos2.norm2())
		rot = angulos1;
	else
		rot = angulos2;
	
	return rot;
}


