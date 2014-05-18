/*
    Copyright (c) 2014 <copyright holder> <email>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef TARGET_H
#define TARGET_H

/*
 * CLASE AÑADIDA: TARGET.
 * Sirve para marcar puntos objetivos donde llevar diferentes partes del robot, como el brazo
 * derecho, el brazo izquierdo o la cabeza... Necesita varias cosas:
 * 		- Un vector POSE que le indique cuál es su traslación y su rotación.
 * 		- El tip o endEffector al que está asignado.
 * 		- El innerModel parapoder trocear la trayectoria desde el endEffector hasta el target.
 */ 
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qqueue.h>

using namespace std;

class Target
{
	
public:
	
	enum TargetType {POSE6D, ALIGNAXIS, ADVANCEALONGAXIS};
	
	Target();
	Target(InnerModel *inner, QVec pose, QString tip, const QVec &weights, TargetType tt = POSE6D, QString axisName = "", bool axisConstraint = false, float axisAngleConstraint=0);
	~Target();
	
	// MÉTODOS GET:
	QString getTipName() const { return this->tip; }; 	//Devuelve el nombre del TIP.
	QVec getPose() const { return this->pose; }; 				// Devuelve el vector pose del target
	QTime getStartTime() const { return this->start; }; // Devuelve el tiempo del target.
	bool getActivo() const { return this->activo; };		// Devuelve el estado del target
	QVec getWeights() const { return this->weights; };
	TargetType getType() const { return targetType;};
	QString getAxisName() const { return axisName;};
	bool getAxisConstraint() const {return axisConstraint;};
	float getAxisAngleConstraint() const {return axisAngleConstraint;};
	
	
	// MÉTODOS SET:
	void setInnerModel (InnerModel *newInner);
	void setPose(QVec newPose);
	void setStartTime (QTime newStart);
	void setActivo (bool newActivo);	
	void setWeights(const QVec &weights);
	void setAxis(const QVec &axis, QString axisName);
	
	// OTROS MÉTODOS
	void trocearTarget();
	void print();
	
private:
	
	QString tip; 							// Nombre del efector final al que está asociado el target
	QTime start;							// Tiempo en que comenzó a trabajar el robot con el target original.
	bool activo;							// Bandera para indicar si el target es válido y el robot debe trabajar con él o no.
	QVec pose; 								// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
	QVec axis;								// Target axis to be aligned with
	QString axisName;						// Name of tip axis to aligned with "axis"
	QQueue<QVec> subtargets;				// Cola de subtargets (trayectorias troceadas) para cada target.
	InnerModel *inner;						// Innermodel para calcular cosas.
	QVec weights;							// Pesos para restringir translaciones, rotaciones o ponderar el resultado
	TargetType targetType;					// 
	bool axisConstraint;					// True if constraint is to be applien on axis for ALINGAXIS mode
	float axisAngleConstraint;				// constraint oto be applied to axis in case axisConstrint is TRUE
	
};


#endif // TARGET_H