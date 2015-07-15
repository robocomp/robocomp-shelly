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

/**
  \brief  VISUAL INVERSE KINEMATICS COMP
  @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <iostream>
#include <fstream>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <target.h>
#include <visualhand.h>


#ifdef USE_QTGUI
    #include <osgviewer/osgview.h>
    #include <innermodel/innermodelviewer.h>

#endif

using namespace std;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
	public:
	SpecificWorker(MapPrx& mprx);    
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	TargetState getTargetState       (const string &bodyPart, const int targetID);
	int         setTargetAdvanceAxis (const string &bodyPart, const Axis &ax, const float dist);
	void        goHome               (const string &bodyPart);
	void        stop                 (const string &bodyPart);
	int         setTargetPose6D      (const string &bodyPart, const Pose6D &target, const WeightVector &weights);
	bool        getPartState         (const string &bodyPart);
	int         setTargetAlignaxis   (const string &bodyPart, const Pose6D &target, const Axis &ax);
	void        setJoint             (const string &joint, const float angle, const float maxSpeed);
	void        setFingers           (const float d);
	void        newAprilTag          (const tagsList &tags);

public slots:
	void    compute ();

private:
	enum class State {IDLE, INIT_BIK, WAIT_BIK, CORRECT_TRASLATION, CORRECT_ROTATION}; // ESTADOS POR LOS QUE PASA LA MAQUINA DE ESTADOS DEL VISUAL BIK:
	State                      stateMachine; // LA VARIABLE QUE GUARDA EL ESTADO DEL VISUAL BIK
	VisualHand                 *rightHand;   // VARIABLE QUE GUARDA LA POSE VISUAL DE LA MARCA DE LA MANO DERECHA DEL ROBOT
	QMap<QString, QStringList> motorchains;  // MAPA CON LAS CADENAS DE MOTORES

	// VARIABLES QUE GUARDAN EL TARGET QUE SE ESTA EJECUTANDO Y LOS SIGUIENTES A EJECUTAR.
	Target           currentTarget;
	Target           correctedTarget;
	QQueue<Target>   nextTargets;
	QQueue<Target>   solvedList;          // TARGETS RESUELTOS
	QMutex           *mutexSolved;        // MUTEX PARA ZONAS CRITICAS
	InnerModel       *innerModel;         // EL MODELO INTERNO DEL ROBOT
	ofstream         file;                // EL FICHERO DONDE GUARDAR DATOS
	bool             abortatraslacion;    // PARA QUE NO SE QUEDE COLGADO CUANDO CORRIGE TRASLACION
	bool             abortarotacion;      // PARA QUE NO SE QUEDE COLGADO CUANDO CORRIGE ROTACION
	bool             INITIALIZED;         // PARA QUE NO SE ADELANTE AL SETPARAMS
	int              contador;
        float            timeSinMarca;

	#ifdef USE_QTGUI
	OsgView           *osgView;
	InnerModelViewer  *innerViewer;
	#endif

	// METODOS PRIVADOS
	bool correctTraslation ();
	bool correctRotation   ();
	void updateAll         ();
	void updateMotors      (RoboCompInverseKinematics::MotorList motors);
	void printXXX          (QVec errorInv/*, bool camaraNoVista*/);

};

#endif

