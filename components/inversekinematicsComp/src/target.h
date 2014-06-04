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
#include <boost/graph/graph_concepts.hpp>

using namespace std;

class Target
{
	
public:
	
	enum TargetType {POSE6D, ALIGNAXIS, ADVANCEAXIS};
	enum FinishStatus { LOW_ERROR, KMAX, LOW_INCS, NAN_INCS, START };
	
	Target();
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &weights, bool chop=true);											// For Pose6D
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &axis, const QVec &weights);		// For AlingAxis
	Target(TargetType tt, InnerModel* inner, const QString &tip, const QVec& axis, float step);																	// For ADVANCEALONGAXIS

	~Target();
	
	// MÉTODOS GET:
	QString getTipName() const					{ return this->tip; }; 				//Devuelve el nombre del TIP.
	QVec getPose() const 						    { return this->pose6D; }; 			// Devuelve el vector pose del target
	QTime getStartTime() const 					{ return this->start; }; 			// Devuelve el tiempo del target.
	bool getActivo() const 						  { return this->activo; };			// Devuelve el estado del target
	QVec getWeights() const 					  { return this->weights; };
	TargetType getType() const 					{ return targetType;};
	bool getAxisConstraint() const 			{ return axisConstraint;};
	float getAxisAngleConstraint() const{ return axisAngleConstraint;};
	QString getNameInInnerModel() const { return nameInInnerModel;};
	float getError() const 						  { return error;};
	QVec getAxis() const 						    { return axis;};
	float getStep() const 						  { return step;};
	QTime getRunTime() const 					  { return runTime;};
	int getElapsedTime() const 					{ return runTime.elapsed();};
	FinishStatus getStatus() const 			{ return finish;};
	QVec getErrorVector() const 				{ return errorVector;};
	QVec getFinalAngles() const 				{ return finalAngles;};
	QQueue<Target> getSubtargets() const{ return subtargets;};
	Target& getHeadFromSubtargets() 			{ return subtargets.head();};
	void removeHeadFromSubtargets()			{ subtargets.dequeue();};
	
	// MÉTODOS SET:
	void setInnerModel (InnerModel *newInner);
	void setPose(QVec newPose);
	void setStartTime (QTime newStart);
	void setActivo (bool newActivo);	
	void setWeights(const QVec &weights);
	void setNameInInnerModel(const QString &name);
	void setError(float error)							{ this->error = error; };
	void setStatus(FinishStatus status)			{ finish = status;};
	void setElapsedTime(ulong e)						{ elapsedTime = e;};
	void setRunTime(const QTime &t)					{ runTime = t;};
	void setIter(uint it)										{ iter = it;};
	void setErrorVector(const QVec &e)			{ errorVector = e;};
	void setFinalAngles(const QVec &f)			{ finalAngles = f;};
	
	
	// OTROS MÉTODOS
	//void chopPath();
	void print(const QString &msg = QString());
	
private:
	
	QString tip; 										// Nombre del efector final al que está asociado el target
	QTime start;										// Tiempo en que comenzó a trabajar el robot con el target original.
	bool activo;										// Bandera para indicar si el target es válido y el robot debe trabajar con él o no.
	QVec pose6D; 										// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
	QVec axis;											// Target axis to be aligned with
	QQueue<Target> subtargets;			// Cola de subtargets (trayectorias troceadas) para cada target.
	InnerModel *inner;							// Innermodel para calcular cosas.
	QVec weights;										// Pesos para restringir translaciones, rotaciones o ponderar el resultado
	TargetType targetType;					// 
	bool axisConstraint;						// True if constraint is to be applien on axis for ALINGAXIS mode
	float axisAngleConstraint;			// constraint oto be applied to axis in case axisConstrint is TRUE
	QString nameInInnerModel;				// generated name to create provisional targets
	float error;										// Error after IK
	QVec errorVector;								// Error vector
	float step;											// step to advance along axis
	QTime startTime;								// timestamp indicating when the target is created
	QTime runTime;									// timestamp indicating when the target is executed
	ulong elapsedTime;          		// timestamp indicating when the duration of the target execution in milliseconds.
	FinishStatus finish;        		// Enumerated to show finish reason
	uint iter;											// Number of iterations before completing
	QVec finalAngles;								// Mercedes lo documenta luego
	
	float standardRad(float t);
};


#endif // TARGET_H