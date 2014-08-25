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
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
	// Inicialmente las misiones se llevaran a cabo por el RCIS:
	changeProxy(0);
	// Conectamos los botones:
	this->connectButtons();
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
 * TODO ARREGLAR
 * 
 * @return bool
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Guardamos el innerModel que se nos pasa como parámetro de inicialización.
	// ¡CUIDADO CON EL INNERMODEL! Debe ser el mismo que el que utiliza LOKIARM!!!
	try
	{
 		RoboCompCommonBehavior::Parameter par;// = params.at("BIK.InnerModel") ; // No lee bien esto
 		qDebug()<<params.size();
 		//par = params.at("BIK");

//  		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel = new InnerModel(par.value);
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
// 		}
// 		else
// 		{	qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
// 			qFatal("Exiting now.");
// 		}
	}
	catch(std::exception e)	{ qFatal("Error reading config params"); }

	timer.start(Period);
	return true;
}

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	// Definimos una variable de tiempo para hacer este método periódico...
	static QTime reloj = QTime::currentTime();
	
	if( reloj.elapsed() > 1000)
	{	
	}
}

//**********************************************************************************************************//
//			SLOTS PÚBLICOS																					//
//**********************************************************************************************************//
void SpecificWorker::compute( )
{
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
	goHome("All");
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
	}catch(const Ice::Exception &ex){std::cout <<"CONST"<< ex << std::endl;};
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

