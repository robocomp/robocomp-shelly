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
#include <innermodel/innermodel.h>

#include "../../inversekinematicsComp/src/target.h"
#include <qt4/Qt/qcheckbox.h>
#include <qt4/Qt/qwidget.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qspinbox.h>
#include <qt4/QtGui/QFrame>

#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
// #include <innermodel/innermodelreader.h>


/**
       \brief
       @author authorname
*/

// extern std::pair< graph_traits< G >::out_edge_iterator, graph_traits< G >::out_edge_iterator > p;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	
	void compute(); 
	
	// SLOTS DE LOS BOTONES DE EJECUCIÓN DE LA INTERFAZ.
	void enviarRCIS(); 		// select RCIS.
	void enviarROBOT(); 	// select robot
	void enviar();				// send data to robot or rcis
	void enviarHome();
	void actualizarInnerModel();
	
	
	// SLOTS DE LA PESTAÑA POSE6D:
	void activarDesactivar(); //ACtiva/desactiva la opción de escoger una o más partes del cuerpo.
	void camareroZurdo();
	void camareroDiestro();
	void camareroCentro();
	void camareroCentro2();
	void stop();
	void closeFingers();
	void goHome(QString partName);
	
	// MÉTODOS AÑADIDOS: REVISAR
	void abrirPinza();
	void cerrarPinza();
	void posicionInicial();
	void posicionCoger();
	void posicionSoltar();
	void retroceder();
	void goHomeR();
	void izquierdoRecoger();
	void izquierdoOfrecer();

	
private:
	
	// ATRIBUTOS
	InnerModel *innerModel;								// Para trabajar con el innerModel
	OsgView *osgView;
	QFrame *frameOsg;
	InnerModelViewer *imv;
	QQueue<QVec> trayectoria;							// Cola de poses donde se guardan las trayectorias de los Camareros
	QVec partesActivadas;								//vector de partes (se ponen a 0 si NO se les envía target y a 1 si SÍ se les envía target)
	RoboCompJointMotor::MotorParamsList motorparamList;	//lista de motores.
	RoboCompJointMotor::MotorList motorList;

	bool banderaTrayectoria;							// Se pone a TRUE si hay una trayectoria para enviar. FALSE si no hay trayectoria.
	bool banderaRCIS;										//indica que se ha pulsado el boton de rcis ÑAPA
	QString tabName;									//Name of current tab
	int tabIndex;										//Index of current tabIndex	
		
	// MÉTODOS
	void moverTargetEnRCIS(const QVec &pose);
	void enviarPose6D(QVec p);
	void enviarAxisAlign();
	void moveAlongAxis();
	void mostrarDatos();
	void calcularModuloFloat(QVec &angles, float mod);
	
};

#endif
