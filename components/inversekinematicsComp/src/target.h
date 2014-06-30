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
 * CONTEMPLAD LA GLORIA DE LA DOCUMENTACIÓN
 * Sirve para marcar puntos objetivos donde llevar diferentes partes del robot, como el brazo
 * derecho, el brazo izquierdo o la cabeza... Necesita varias cosas:
 * 		- Un vector POSE que le indique cuál es su traslación y su rotación.
 * 		- El tip o endEffector al que está asignado.
 * 		- El innerModel parapoder trocear la trayectoria desde el endEffector hasta el target.
 * 
 * MODIFICACIONES: HE QUITADO EL FLOAT ERROR PARA QUEDAR SÓLO EL ERRORVECTOR. SI SE QUIERE PREGUNTAR POR EL FLOAT ERROR
 * SE HACE LA NORMA DEL ERRORVECTOR. ASÍ NOS EVITAMOS QUE UNO ESTE ACTUALIZADO Y EL OTRO NO Y ESAS COSAS TAN RARAS QUE
 * SUCEDEN A MENUDO.
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
	
	// TIPOS ENUMERADOS: EL TIPO DEL TARGET Y EL ESTADO CON EL QUE TERMINA DE PROCESARSE EN EL Levenberg-Marquardt
	enum TargetType {POSE6D, ALIGNAXIS, ADVANCEAXIS};
	enum FinishStatus { LOW_ERROR, KMAX, LOW_INCS, NAN_INCS, START };
	
	// CONSTRUCTORES Y DESTRUCTORES:
	Target();
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &weights, bool chop=true);			// For Pose6D
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &axis, const QVec &weights);		// For AlingAxis
	Target(TargetType tt, InnerModel* inner, const QString &tip, const QVec& axis, float step);										// For ADVANCEALONGAXIS

	~Target();
	
	
	////////////////////////////////               MÉTODOS DE CONSULTA                   ////////////////////////////////////////////
	QString getTipName() const				{ return this->tip; }				// Devuelve el nombre del efector final de la cadena cinemática del robot..
	QString getNameInInnerModel() const		{ return this->nameInInnerModel; }	// Devuelve el nombre del target en el innerModel.
	
	bool isActive() const					{ return this->activo; }          	// Devuelve el estado del target
	bool getAxisConstraint() const			{ return this->axisConstraint; }	// Devuelve si hay restricciones en los ejes.
	bool getExecuted() const				{ return this->executed; }			// Devuelve si se ejecuta en el robot o no.
	bool isChopped() const					{ return this->chopped; }			// Devuelve si está troceado o no.
	bool isMarkedforRemoval() const			{ return this->removal; }			// Devuelve si está marcado para remover o no.
	bool isAtTarget() const 				{ return this->atTarget; };			// Devuelve si ya se ha alcanzado el punto objetivo o no
	
	float getAxisAngleConstraint() const 	{ return this->axisAngleConstraint;}// Devuelve las restricciones sobre el eje.
	float getError() const					{ return this->errorVector.norm2();}// Devuelve la norma del vector de errror con que termina de procesar el target el Levenberg-Marquardt
	float getStep() const					{ return this->step; }				// Devuelve la distancia que avanza sobre alongAxis
	float getRadius() const 				{ return this->radius; }			// Devuelve el radio donde debe quedarse parado el efector antes de llegar al target.
	
	int getElapsedTime() const				{ return this->runTime.elapsed(); }	// Devuelve el tiempo que ha estado ejecutadose el target en el Levenberg-Marquardt.

	QTime getStartTime() const				{ return this->startTime; }			// Devuelve el tiempo de creación del target.
	QTime getRunTime() const				{ return this->runTime; }			// Devuelve el tiempo de ejecución del target en el Levenberg-Marquardt

	QVec getPose() const					{ return this->pose6D; }			// Devuelve el vector pose del target
	QVec getWeights() const					{ return this->weights; } 			// Devuelve el vector de pesos asociado al target.
	QVec getAxis() const					{ return this->axis; }				// Devuelve los ejes del target (con los que hay que alinearse o por donde hay que avanzar??)
	QVec getErrorVector() const				{ return this->errorVector; }		// Devuelve el vector de error con el que ha terminado de procesarse el target en el Levenberg-Marquardt
	QVec getFinalAngles() const				{ return this->finalAngles; }		// Devuelve el vector de ángulos finales con los que se alcanza el target.

	TargetType getType() const				{ return this->targetType; }		// Devuelve el tipo del target.
	FinishStatus getStatus() const			{ return this->finish; }			// Devuelve el estado con el que se termina de procesar el target.	
	
	
	////////////////////////////////               MÉTODOS PARA MODIFICAR                   ////////////////////////////////////////////	
	void setNameInInnerModel(const QString &name)		{ this->nameInInnerModel = name;}	// Guarda el nombre del target en el InnerModel.
	
	void setActivo 			(const bool newActivo)		{ this->activo = newActivo; }		// Guarda el estado del target
	void setAtTarget		(const bool a)  			{ this->atTarget = a; }				// Guarda si se ha alcanzado el target o no.
	void setExecuted		(const bool e)				{ this->executed = e; }				// Guarda si está siendo ejecutado por el robot o no.
	void setChopped			(const bool c)				{ this->chopped = c; }				// Guarda si está siendo troceado o no.
	void markForRemoval		(const bool m) 				{ this->removal = m; }				// Guarda si el target puede ser eliminado o no.
	
	void setRadius			(const float r)       		{ this->radius = r; }				// Guarda el radio entre el target y el efector final.
	
	void setStartTime 		(const QTime newStart)		{ this->startTime = newStart;}		// Guarda el tiempo de creación del target.
	void setRunTime			(const QTime &t)			{ this->runTime = t; }				// Guarda el tiempo de ejecución del target.

	void setPose			(const QVec &newPose)		{ this->pose6D = newPose;};			// Guarda la pose del target.
	void setChoppedPose		(const QVec &newPose)		{ this->pose6DChopped = newPose;}	// Guarda la pose del subtarget.
	void setWeights			(const QVec &weights)		{ this->weights = weights; }		// Guarda el vector de pesos asociado al target.
	void setErrorVector		(const QVec &e)   			{ this->errorVector = e; }			// Guarda el vector de error con el que ha terminado de procesar el target el Levenberg-Marquardt
	void setFinalAngles		(const QVec &f)  			{ this->finalAngles = f; }			// Guarda los ángulos finales con los que se alcanza el target.

	void setStatus			(const FinishStatus status)	{ this->finish = status; }			// Guarda el estado final del target.
	
	void setElapsedTime		(const ulong e)				{ this->elapsedTime = e; }			// Guarda el tiempo trasncurrido entre la creación y la ejecución del target.
	void setIter			(const uint it)				{ this->iter = it; }				// Guarda la iteración en la que el Levenberg-Marquardt termina de procesar el target.
	
	void annotateInitialTipPose	();								// Guarda la pose inicial del efector final de la cadena cinemática a la que es asignada el target
	void annotateFinalTipPose	();								// Guarda la pose final con la que el efector alcanza el target.
	void setInitialAngles		(const QStringList &motors);	// Guarda los ángulos iniciales de los joints de la cadena cinemática.
	
	// OTROS MÉTODOS
	void print(const QString &msg = QString());
	
	
	
private:
	
	// VARIABLE DE INNERMODEL:
    InnerModel *inner;						// Innermodel para calcular cosas.
    
    // VARIABLES DE QString:
    QString tip;                     		// Nombre del efector final al que está asociado el target
    QString nameInInnerModel;				// generated name to create provisional targets
	
	// BANDERAS (BOOLEANOS):
    bool activo;							// Bandera para indicar si el target es válido y el robot debe trabajar con él o no.
    bool axisConstraint;					// True if constraint is to be applien on axis for ALINGAXIS mode
    bool executed;                    		// Indica si está siendo ejecutado en el robot real (TRUE) o en el simulador (FALSE)
	bool removal;           		        // Indica si el target debe eliminarse.
	bool atTarget;							// Indica si ya se está en el target.
	bool chopped;							// Indica si el target está siendo troceado.
	
	// VARIABLES FLOAT:
	float axisAngleConstraint;				// constraint to be applied to axis in case axisConstrint is TRUE
    float step;								// step to advance along axis
   	float radius; 							// if > 0 the robot stops when reaching a ball of radius "radius" with center in Pose6D

	// VARIABLES DE TIEMPO:
    QTime startTime;						// timestamp indicating when the target is created
    QTime runTime;							// timestamp indicating when the target is executed

    // VECTORES:
    QVec pose6D; 							// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz del target original
    QVec pose6DChopped;						// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz del subtarget
    QVec initialAngles;               		// Ángulos originales antes de ejecutar el Levenberg-Marquardt y resolver el target
    QVec finalAngles;						// ángulos finales con los que se alcanza el target.
    QVec initialTipPose;            		// Tip position in world reference frame BEFORE processing the target
    QVec finalTipPose;              		// Tip position in world reference frame after processing the target
    QVec weights;							// Pesos para restringir translaciones, rotaciones o ponderar el resultado
    QVec axis;								// Ejes objetivos con los que se deben alinear
    QVec errorVector;						// Error vector

	// VARIABLES DE TIPO ENUMERADO
    TargetType targetType;					// Tipo del target.
	FinishStatus finish;        			// Enumerated to show finish reason

	// OTRAS VARIABLES
	ulong elapsedTime;          			// timestamp indicating when the duration of the target execution in milliseconds.
    uint iter;								// Number of iterations before completing
};


#endif // TARGET_H
