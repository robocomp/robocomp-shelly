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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "trayectoria.h"

#include <innermodel/innermodel.h>

#include "../../inversekinematicsComp/src/target.h"
#include <qt4/Qt/qcheckbox.h>
#include <qt4/Qt/qwidget.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qspinbox.h>
#include <qt4/QtGui/QFrame>
#include <time.h>
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
// #include <innermodel/innermodelreader.h>


/**
       \brief INVERSE KINEMATICS TESTER COMP
       @author authorname
*/


class SpecificWorker : public GenericWorker
{
	
Q_OBJECT
public:
	// Constructores y destructores:
	SpecificWorker				(MapPrx& mprx);	
	~SpecificWorker				();
	
	// Métodos públicos:
	bool 	setParams			(RoboCompCommonBehavior::ParameterList params);	//Parámetros de configuración
	void	newAprilTag			(const tagsList& tags); 						//Publicación del AprilTags

public slots:

	void 	compute				(); 
	
	//// SLOTS DE LOS BOTONES DE EJECUCIÓN DE LA INTERFAZ. ////
	void 	stop				(); 		// Botón de parada segura. Para abortar la ejecución del movimiento tanto en RCIS como en Robot real
	void 	enviarHome			();			// Envía los brazos y la cabeza a la posición de home.
	void 	enviarRCIS			();			// Selecciona como proxy el del RCIS.
	void 	enviarROBOT			();			// Selecciona como proxy el del robot real.
	
	//// SLOTS DE LA PESTAÑA POSE6D ////
	void 	camareroZurdo		();			// Trayectoria cuadrada para el brazo izquierdo
	void 	camareroDiestro		();			// Trayectoria cuadrada para el brazo derecho
	void 	camareroCentro		();			// Trayectoria cuadrada para ambos brazos 
	void 	puntosEsfera		();			// Trayectoria cuadrada para ambos brazos 
	void 	puntosCubo			();			// Trayectoria cuadrada para ambos brazos
	
	//// SLOTS ÚTILES PARA FACILITAR EL USO DE LA GUI ////
	void 	updateBodyPartsBox	(); 		//Activa/desactiva la opción de escoger una o más partes del cuerpo.
	void 	send				();			// Botón que indica que existe un target para ser resuelto.
	
	//// SLOTS DIABÓLICOS 1: UNIT TESTS ////
	void 	abrirPinza			();			// Abre la mano del robot dejando una distancia de separación entre las dos pinzas 
	void 	posicionInicial		();			// Mueve la mano a una posición inicial desde la que aproximarse para coger la taza
	void 	posicionCoger		();			// Mueve la mano a la posición donde debería estar la taza para cogerla.
	void 	cerrarPinza			();			// Cierra la mano del robot dejando una distancia de separación entre las dos pinzas 
	void 	posicionSoltar		();			// Mueve el brazo hasta la bandeja de la mano izquierda para soltar la taza.
	void 	izquierdoRecoger	();			// Mueve el brazo izquierdo para dejar la bandeja debajo de la taza.
	void 	retroceder			();			// Después de dejar la taza, retira el brazo derecho.
	void 	goHomeR				();			// Lleva el brazo derecho a la posición de home.
	void 	izquierdoOfrecer	();			// Mueve el brazo izquierdo para que ofrezca la taza al usuario.



	

	
	// Métods AÑADIDOS +++
	void boton_1();
	void boton_2();
	void boton_3();
	void boton_4();
	void boton_5();
	void boton_6();
	void boton_7();	
	void boton_8();
	void boton_9();
	void boton_10();
	void boton_11();
	void boton_12();
	void boton_13();
	void boton_14();
	void boton_15();
	void boton_16();
// 	void boton_17();	
// 	void boton_18();
// 	void boton_19();
	void boton_21();
	void boton_22();
	void boton_23();
	void boton_24();
	

	
	
	
	
	void moveToFrom(const QVec &poseTarget, const QVec &poseFrom);
	void moveToApril(const QVec &poseTarget);
	void moveToApril2(const QVec &poseTarget);
	void moveToTarget(const QVec &targetInWorld);

	void ballisticPartToAprilTarget(int xoffset = 100);
	void finePartToAprilTarget();
private:

	//// ATRIBUTOS PRIVADOS DE LA CLASE ////
	Trayectoria 		tra;
	
	RoboCompJointMotor::MotorParamsList 	motorparamList;			// Lista de parámetros de los motores del robot. Para sacar valores angulares.
	RoboCompJointMotor::MotorList 			motorList;				// Lista con los nombres de los motores del robot.

	InnerModel 			*innerModel;								// Puntero para trabajar con el innerModel (pintar el target y obtener valores angulares)
	InnerModelViewer 	*imv;										// Puntero para pintar el innerModel en la pestaña DirectKinematics
	OsgView 			*osgView;									// Puntero para pintar innerModel en una de las pestañas.
	QFrame				*frameOsg;									// Puntero para pintar innerModel en una de las pestañas.
	QQueue<QVec>		trayectoria;								// Cola de poses donde se guardan las trayectorias de los Camareros
	QVec				partesActivadas;							// Vector de partes (se ponen a 0 si NO se les envía target y a 1 si SÍ se les envía target)
	QVec 				marcaBote;									
	QVec 				marcaBote2;									// Vector de 6 elementos-->coordenadas de traslación y rotación, de la marca nº 13, la del bote.
	QVec 				savedAprilTarget;
	QVec 				manoApril;
	
	bool				flagListTargets;							// Se pone a TRUE si hay una trayectoria para enviar. FALSE si no hay trayectoria.
	bool				existTarget;								// Se pone a TRU cuando hay un target o una lista de targets a enviar al RCIS o al ROBOT
	QString 			tabName;									//Name of current tab
	int 				tabIndex;									//Index of current tabIndex	
	
	////////////////  MÉTODOS PRIVADOS  ////////////////
	/// MÉTODOS PRIVADOS DE ENVÍO ///
	void 	sendTarget					();					// Envia los targets, HOME o FINGERS al RCIS o al ROBOT.
	void 	sendPose6D					(QVec p);			// Envía targets de tipo POSE6D
	void 	sendAxisAlign				();					// Envía targets de tipo AXISALIGN
	void 	sendAlongAxis				();					// Envía targets de tipo ALONGAXIS
	void 	goHome						(QString partName);	// Envía la parte del cuerpo seleccionada al home.
	void 	closeFingers				();					// Cierra o abre las pinzas de la mano del robot

	/// MÉTODOS MUY SIMPLES Y AUXILIARES TOTALES QUE NO TIENEN MAYOR IMPORTANCIA PERO QUE LIMPIAN CÓDIGO ///
	void 	connectButtons				();							// Conecta botones de la interfaz de usuario con sus SLOTS correspondientes.
	void 	initDirectKinematicsFlange	();							// Inicializa datos de la pestaña DirectKinematics. 
	void 	showKinematicData			();							// Mantiene actualizados los valores de la pestaña DirectKinematics.
	void 	changeText					(int type);					// Cambia el texto de las ventanitas aDonde
	void 	actualizarInnerModel		();							// ACtualiza el innerModel
	void 	moveTargetRCIS				(const QVec &pose);			// Mueve la marca en RCIS
	void 	calcularModuloFloat			(QVec &angles, float mod);	// Normaliza ángulos entre -pi y pi.
};

#endif
