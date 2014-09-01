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
#include <boost/concept_check.hpp>

//**********************************************************************************************************//
//				CONSTRUCTORES Y DESTRUCTORES																//
//**********************************************************************************************************//
/**
* \brief Default constructor
* 1) Inicializa el proxy a utilizar por defecto con el RCIS.
* 2) Conecta los botones de la interfaz
* 3) Guarda los nombres de los motores del robot y sus parámetros
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
	this->changeProxy(0);	// Inicialmente las misiones se llevaran a cabo por el RCIS:
	this->connectButtons();
	
	this->motorparamList = jointmotor_proxy->getAllMotorParams();
	foreach(RoboCompJointMotor::MotorParams motorParam, this->motorparamList)
		this->motorList.push_back(motorParam.name);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

//**********************************************************************************************************//
//				MÉTODOS DE INICIALIZACIÓN Y CONEXIÓN?														//
//**********************************************************************************************************//
/**
 * @brief Method SET PARAMS. 
 * It's called for the MONITOR thread, which initialize the component with the 
 * parameters of the correspondig config file.
 * 
 * @param params
 * 
 * @return bool
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Guardamos el innerModel que se nos pasa como parámetro de inicialización.
	// ¡CUIDADO CON EL INNERMODEL! Debe ser el mismo que el que utiliza LOKIARM!!!
	try
	{
 		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel");

 		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
		}
		else
		{	qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}
	catch(std::exception e)	{ qFatal("Error reading config params"); }

	timer.start(Period);
	return true;
}

/**
 * @brief  SUBSCRIBED METHOD FROM APRILTAGS
 * Método para subscribire al apriltag. El aprilTags publica al tester.
 * 
 * @param tags lista de objetivos o marcas detectadas por el aprilTag.
 * @return void 
 */
void SpecificWorker::newAprilTag(const tagsList& tags)
{
	// Definimos una variable de tiempo para hacer este método periódico...
	static QTime reloj = QTime::currentTime();
	
	if( reloj.elapsed() > 1000)
	{	
		for(uint i=0; i<tags.size(); i++)
			this->processTag(tags[i]);	
	}
}

//**********************************************************************************************************//
//			SLOTS PÚBLICOS																					//
//**********************************************************************************************************//
/**
 * @brief SLOT COMPUTE
 * Bucle de control del programa. Se encarga de actualizar el RCIS
 * 
 * @return void
 */ 
void SpecificWorker::compute( )
{
	mutex->lock();
		this->updateRCIS();
	mutex->unlock();
}

/**
 * @brief SLOT MISSION
 * Se encarga de ver qué proxy está seleccionado y lo cambia en
 * el inverseKinematicsComp.
 * 
 * @param index índice del combo box.
 * 
 * @return void
 */ 
void SpecificWorker::mission(int index)
{
	
	qDebug()<<"Change proxy to: "<<missionButton->currentText();
	
	switch (index)
	{
		case 0:
			this->changeProxy(0);
			this->changeText(0);
			break;
		
		case 1:
			this->changeProxy(1);
			this->changeText(1);
			break;
			
		case 2:
			qDebug()<<"POR HACER";
			break;
			
		default:
			qDebug()<<"\n--->ERROR!!\n";
			break;
	}
}

/**
 * @brief SLOT público STOP. 
 * Este método se encarga de llamar al método de la interfaz stop de inverseKinematicsComp
 * con las (por ahora) tres partes del cuerpo del robot, el brazo derecho, el izquierdo y
 * la cabeza. Este método de la interfaz debería encargarse de parar la ejecución del target
 * asignado a las partes del robot.
 * Además el SLOT stop baja la bandera de trayectorias y existen targets para anular los 
 * targets que se envían.
 * 
 * @return void
 */ 
void SpecificWorker::stop()
{
	qDebug()<<"\n----> ABORTIN MISSION!!!\n";
	
	try 
	{
		bodyinversekinematics_proxy->stop("LEFTARM");
		bodyinversekinematics_proxy->stop("RIGHTARM");
		bodyinversekinematics_proxy->stop("HEAD");	
		
// 		flagListTargets=false;
// 		existTarget = false;
	} 
	catch (const Ice::Exception &ex) {	cout<<"Excepcion in STOP: "<<ex<<endl;	}
}

/**
 * @brief SLOT HOME. 
 * Envía a la posición de home TODAS las partes del robot.
 * 
 * @return void
 */
void SpecificWorker::home()
{
	this->goHome("All");
}

//**********************************************************************************************************//
//			MÉTODOS PÚBLICOS																				//
//**********************************************************************************************************//


//**********************************************************************************************************//
//			MÉTODOS PRIVADOS																				//
//**********************************************************************************************************//
/**
 * @brief Método CONNECT BUTTONS.
 * Se encarga de conectar TODOS los botones de la interfaz de usuario (de TODAS las pestañas). Es
 * llamado en el constructor de la clase para hacer todas las conexiones de los botones a sus 
 * correspondientes slots.
 * 
 * @return void
 */ 
void SpecificWorker::connectButtons()
{
	qDebug()<<"Conectando botones...";
	
	// SUBMIT BUTTONS:
	connect(	stopButton, 	SIGNAL(clicked()), 					this, 	SLOT(stop	()		));
	connect(	homeButton,		SIGNAL(clicked()), 					this,	SLOT(home	()		));
	connect(	missionButton,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(mission(int)	));

}

/**
 * @brief Método UPDATE RCIS. 
 * Actualiza el InnerModel con las nuevas posiciones de los motores del robot.  
 * 
 * @return void
 */ 
void SpecificWorker::updateRCIS()
{
	try 
	{
		RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap( this->motorList);

		for(uint j=0; j<this->motorList.size(); j++)
			this->innerModel->updateJointValue(QString::fromStdString(this->motorList[j]), mMap.at(this->motorList[j]).pos);
		
	} catch (const Ice::Exception &ex) {cout<<"--> Exception in UPDATE RCIS: "<<ex<<endl;	}
}

/**
 * @brief Metodo GO HOME. 
 * Envia a la posicion de HOME la parte del cuerpo del robot que este seleccionada en la pestaña 
 * "Home". Existe la opción "All" dentro de esta pestaña, en la que hay que enviar TODAS las partes
 * del cuerpo al HOME.
 * 
 * @param partName nombre de la parte a enviar al home.
 * 
 * @return void
 */
void SpecificWorker::goHome(QString partName)
{
	std::string part = partName.toStdString();
	qDebug() << "\n---> Go gome \n" << partName;
	
	try 
	{	
		if(partName=="All")
		{
				bodyinversekinematics_proxy->goHome("HEAD");
				bodyinversekinematics_proxy->goHome("LEFTARM");
				bodyinversekinematics_proxy->goHome("RIGHTARM");
		}
		else
			bodyinversekinematics_proxy->goHome(part);
	} 
	catch (Ice::Exception ex) {cout <<"Exception in GO HOME: "<< ex << endl;}
	
	//existTarget=false;
}

/**
 * @brief Método CHANGE PROXY
 * Cambia el proxy al que se le enviará la misión.
 * 
 * @param p número del proxy: 0 para el RCIS y 1 para el robot real.
 * 
 * @return void
 */ 
void SpecificWorker::changeProxy(int p)
{
	try 
	{ 
		bodyinversekinematics_proxy->setRobot(p);	
	}catch(const Ice::Exception &ex) {std::cout <<"CONST"<< ex << std::endl;};
}

/**
 * @brief Método CHANGE TEXT 
 * Este método se encarga de cambiar el texto delo cuadrado de texto, indicando
 * a qué proxy se va a enviar el target dependiendo de la acción que esté pulsada,
 * RCIS o REAL ROBOT.
 * 
 * @param text entero que indica que texto hay que poner en las ventanitas.
 * 
 * @return void
 */ 
void SpecificWorker::changeText(int text)
{
	switch (text)
	{
		case 0:
			textLabel->clear(); 
			textLabel->insertPlainText("\nEl target se enviara al RCIS .\n Seleccione destino en el menu de al lado.");
			break;
		case 1:
			textLabel->clear(); 
			textLabel->insertPlainText("\nEl target se enviara al ROBOT REAL .\n Seleccione destino en el menu de al lado.");
			break;
		default:
			break;
	}
}

/**
 * @brief Método PROCESS TAG
 * Procesa la marca que recibe como parámetro de netrada y mira si
 * se trata de la marca del objetivo, la marca de la mano izquierda
 * u otro tipo de marca.
 * 
 * @param tag marca que está viendo la cámara del robot.
 * 
 * @return void
 * 
 * TODO: PUEDE DAR PROBLEMAS. SI ES ASÍ TOMAMOS EL CÓDIGO ANTIGUO DE AGUSTÍN Y LISTO.
 */ 
void SpecificWorker::processTag(tag tag)
{
	if(this->correctTag(tag))
	{
		switch (tag.id)
		{
			case 11:
				qDebug()<<"Marca 11: manos";
				this->tag11(tag);
				break;
				
			case 12:
				//qDebug()<<"Marca 12: objetivo 1";
				this->tag12(tag);
				break;
				
			case 13:
				qDebug()<<"Marca 13: objetivo 2";
				this->tag13(tag);
				break;
				
			default:
				qDebug()<<"\n---> Marca no identificada\n";
				break;
		}
	}
	else
		qDebug()<<"\n---> ERROR TAG!";
}

/**
 * @brief Método CORRECT TAG
 * Comprueba si la marca está bien o si se ha leído mal.
 * 
 * @return bool;
 */ 
bool SpecificWorker::correctTag(tag tag)
{
	if ((tag.tx >= -0.0001 && tag.tx <= 0.0001) && 
		(tag.ty >= -0.0001 && tag.ty <= 0.0001) && 
		(tag.tz >= -0.0001 && tag.tz <= 0.0001))
		return false;
	else
		return true;
}

/**
 * @brief Método TAG 11
 * Se encarga de implementar las funciones necesarias cuando la cámara del
 * robot detecta la marca de la MANO.
 * Objetivo principal: recalibrarse, calcular el error entre lo que la cámara ve y lo que
 * el robot cree.
 * 
 * @return void
 */ 
void SpecificWorker::tag11(tag tag)
{
	// Si estamos aquí es que la marca se ha leído bien y procedemos a calcular...
	// Primero guardamos las coordenadas (X, Y, Z) de la marca, así como sus rotaciones (RX, RY, RZ)
	this->manoApril = QVec::zeros(6);
	this->manoApril[0] = tag.tx; 	this->manoApril[1] = tag.ty; 	this->manoApril[2] = tag.tz; 
	this->manoApril[3] = tag.rx; 	this->manoApril[4] = tag.ry; 	this->manoApril[5] = tag.rz;    
	
	//Mostramos la posición de la mano izquierda en el mundo (lo que el robot cree)
	QVec hand_L_World (6);
	hand_L_World.inject(this->innerModel->transform("world", QVec::zeros(3), "grabPositionHandL"),0);
	hand_L_World.inject(this->innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min(), 3);
	qDebug()<<"\n\n MANO IZQUIERDA EN EL MUNDO -->    "<<hand_L_World;
	
	// Al lío... Si no lo hemos creado ya, creamos el nodo de la marca vista desde la cámara:
	if(!innerModel->getNode("tag_Hand_L_Camera"))
	{
		qDebug()<<"\n Creamos nuevo nodo con las coordenadas de la marca de la mano izquierda!\n";
		InnerModelNode *nodeParent = this->innerModel->getNode("rgbd");
		InnerModelTransform *node = this->innerModel->newTransform("tag_Hand_L_Camera", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
		nodeParent->addChild(node);
		this->innerModel->updateTransformValues("tag_Hand_L_Camera",
												this->manoApril.x(),	this->manoApril.y(),	this->manoApril.z(), 
												this->manoApril.rx(),	this->manoApril.ry(),	this->manoApril.rz());
	}
	QVec tag_L_World (6);
	tag_L_World.inject(this->innerModel->transform("world", QVec::zeros(3), "tag_Hand_L_Camera"), 0);
	tag_L_World.inject(this->innerModel->getRotationMatrixTo("world","tag_Hand_L_Camera").extractAnglesR_min(), 3);
	qDebug()<<"\n\n MARCA DE MANO IZQUIERDA EN EL MUNDO -->   "<<tag_L_World;
	
	/*


				// Esto es sólo para mostrar la posición de la marca vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
				QVec marca2TInWorld = innerModel->transform("world", QVec::zeros(3), "marcaHandInCamera3");
				QVec marca2RInWorld = innerModel->getRotationMatrixTo("world","marcaHandInCamera3").extractAnglesR_min();
				QVec marca2InWorld(6);
				marca2InWorld.inject(marca2TInWorld,0);
				marca2InWorld.inject(marca2RInWorld,3);
				//qDebug() << "Marca de la mano en el mundo vista desde la camara" << marca2InWorld;
				
				QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "ThandMesh1");
				QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","ThandMesh1").extractAnglesR_min();
				QVec marcaInWorld(6);
				marcaInWorld.inject(marcaTInWorld,0);
				marcaInWorld.inject(marcaRInWorld,3);
				//qDebug() << "ThandMesh1 en el mundo vista desde RCIS" << marcaInWorld;
				//qDebug() << "Diferencia" <<  marca2InWorld - marcaInWorld;
				
				
				// Calculamos el error de la marca
				// Ponemos la marca vista desde la cámara en el sistma de coordenadas de la marca de la mano, si no hay error debería ser todo cero
				QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh1", QVec::zeros(3), "marcaHandInCamera3");
				QVec visualMarcaRInHandMarca = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera3").extractAnglesR_min();
				QVec visualMarcaInHandMarca(6);
				visualMarcaInHandMarca.inject(visualMarcaTInHandMarca,0);
				visualMarcaInHandMarca.inject(visualMarcaRInHandMarca,3);
				//qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << visualMarcaInHandMarca;
				
				// Cogemos la matriz de rotación dek tHandMesh1 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh1
				QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera3");
				QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh1_pre","ThandMesh1");
					
				// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
				QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
				QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
					
				// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
				QVec handMarcaTInParent = innerModel->transform("ThandMesh1_pre", QVec::zeros(3), "ThandMesh1");
				QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
				
				// Esto es sólo para mostar como está el ThandMesh1 respecto al padre antes de las modificaciones
				QVec inicialHandMarca(6);
				inicialHandMarca.inject(handMarcaTInParent,0);
				inicialHandMarca.inject(handMarcaRInParentMat.extractAnglesR_min(),3);	
				//qDebug() << "Posicion inicial del ThandMesh1 respecto al padre" << inicialHandMarca;
				
				// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
				QVec finalHandMarca(6);
				finalHandMarca.inject(finalHandMarcaT,0);
				finalHandMarca.inject(finalHandMarcaR,3);
				
				//qDebug() << "Posicion final si se corrigiese del ThandMesh1 respecto al padre" << finalHandMarca;
				
				//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
				grabTInWorld = innerModel->transform("world", QVec::zeros(3), "grabPositionHandL");
				grabRInWorld = innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min();
				grabInWorld.inject(grabTInWorld,0);
				grabInWorld.inject(grabRInWorld,3);
				//qDebug() << "Grab en el mundo despues de modificar" << grabInWorld;
					
				//Eliminamos el nodo creado
				innerModel->removeNode("marcaHandInCamera3");
		
				//qDebug() << "\n";
				
				qDebug() << "";
				qDebug() << "Grab en el mundo antes de modificar" << grabInWorld;
				qDebug() << "Marca de la mano en el mundo vista desde la camara" << marca2InWorld;
				qDebug() << "ThandMesh1 en el mundo vista desde RCIS" << marcaInWorld;
				qDebug() << "Diferencia" <<  marca2InWorld - marcaInWorld;
				qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << visualMarcaInHandMarca;
				qDebug() << "Posicion inicial del ThandMesh1 respecto al padre" << inicialHandMarca;
				qDebug() << "Posicion final corregida del ThandMesh1 respecto al padre" << finalHandMarca;
				qDebug() << "Grab en el mundo despues de modificar" << grabInWorld;
				qDebug() << "";
*/
				
}

void SpecificWorker::tag12(tag tag)
{

}


void SpecificWorker::tag13(tag tag)
{

}



