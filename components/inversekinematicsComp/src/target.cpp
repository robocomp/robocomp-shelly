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

#include "target.h"
#include <boost/graph/buffer_concepts.hpp>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto
 */
Target::Target()
{
	this->activo = false;
}

/**
 * \brief Constructor parametrizado
 * Inicializa las estructuras que componen sus atributos de clase. 
 * Le pasamos: 
 * 			- El innerModel, para poder hacer sus cálculos cuando tenga que trocear la trayectoria.
 * 			- El vector POSE, vector de 3 traslaciones y 3 rotaciones.
 * 			- El nombre del EFECTOR FINAL.
 */ 


/**
 * \brief Destructor por defecto
 */ 
Target::~Target()
{
}

/**
 * @brief Constructor for ADVANCE ALONG AXIS target
 * 
 * @param tt ...
 * @param inner ...
 * @param tip ...
 * @param axis ...
 * @param step ...
 */
Target::Target(Target::TargetType tt, InnerModel* inner, const QString &tip, const QVec &axis, float step)
{
	this->activo = true;
	this->pose6D = QVec::zeros(6);  //Needed to resize the Jacobian
	this->tip = tip;
	this->inner = inner;
	this->targetType = tt;
	this->axis = axis;
	this->error = 0.f;
	QVec w(6);w.set((T)1);
	this->weights = w;
	this->step = step;
	this->startTime = QTime::currentTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
}

/**
 * @brief Constructor for ALIGN AXIS target
 * 
 * @param tt ...
 * @param inner ...
 * @param tip ...
 * @param pose6D ...
 * @param axis ...
 * @param weights ...
 */
Target::Target(Target::TargetType tt, InnerModel* inner, const QString &tip, const QVec &pose6D, const QVec &axis, const QVec &weights)
{
	this->activo = true;
	this->pose6D = pose6D;
	this->tip = tip;
	this->inner = inner;
	this->targetType = tt;
	this->axis = axis;
	this->axisConstraint = axisConstraint;
	this->error = 0.f;
	this->weights = weights;
	this->startTime = QTime::currentTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
}

/**
 * @brief Constructor for POSE6D target
 * 
 * @param tt ...
 * @param inner ...
 * @param tip ...
 * @param pose6D ...
 * @param weights ...
 */
Target::Target(Target::TargetType tt, InnerModel* inner, const QString &tip, const QVec &pose6D, const QVec& weights)
{
	this->activo = true;
	this->pose6D = pose6D;
	this->tip = tip;
	this->inner = inner;
	this->targetType = tt;
	this->axis = QVec();
	this->error = 0.f;
	this->weights = weights;
	this->startTime = QTime::currentTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
}

/*--------------------------------------------------------------------------*
 *		      									MÉTODOS PÚBLICOS															*
 *--------------------------------------------------------------------------*/
/*
 * Método SET POSE
 * Asigna el valor del vector de 6 elementos de entrada (3 traslaciones y 3 rotaciones)
 * al atributo pose de la clase TARGET.
 */ 
void Target::setPose(QVec newPose)
{
	this->pose6D = newPose;
}

/*
 * Método SET ACTIVO.
 * Asigna el valor del booleano de entrada newActivo, al atributo activo de la clase
 */ 
void Target::setActivo(bool newActivo)
{
	this->activo = newActivo;
}

/*
 * Método SET START TIME
 * Asigna el valor del QTime de entrada al atributo de tiempo start de la clase.
 */ 
void Target::setStartTime(QTime newStart)
{
	this->start = newStart;
}

/*
 * Método SET WEIGHTS
 * Asigna el valor del QTime de entrada al atributo de tiempo start de la clase.
 */ 
void Target::setWeights(const QVec &weights)
{
	this->weights = weights;
}


/**
 * @brief ...
 * 
 * @param name ...
 * @return void
 */
void Target::setNameInInnerModel(const QString& name)
{
	nameInInnerModel = name;
}



/*
 * Método TROCEAR TARGET
 * Crea una recta entre el tip y el target colocando subtargets cada distanciaMax
 * permitida. Troceamos con la ecuación  R= (1-Landa)*P + Landa*Q
 */ 
void Target::trocearTarget()
{
	
	//TRASLACIÓN: 
	QVec traslacionPose = QVec::vec3(pose6D[0], pose6D[1], pose6D[2]); //sacamos la traslación de la pose.
	QVec traslaciones = (traslacionPose - inner->transform("world", QVec::zeros(3), this->tip));
	
	//ROTACIÓN:
	// Hay que calcular las rotaciones de otra forma. No tenemos actualizado el innerModel para saber cúanto
	// debe girar el this->tip para alcanzar la rotación del target.
	// Sacamos las rotaciones del this->tip y restamos rotaciones. Si son iguales la resta da 0.
	QVec tipEnMundo =  inner->getRotationMatrixTo("world", this->tip).extractAnglesR();
	QVec angulos1 = QVec::vec3(tipEnMundo[0], tipEnMundo[1], tipEnMundo[2]);
	QVec angulos2 = QVec::vec3(tipEnMundo[3], tipEnMundo[4], tipEnMundo[5]);
	QVec rot;
	if(angulos1.norm2() < angulos2.norm2())
		rot = angulos1;
	else
		rot = angulos2;
	
	QVec rotacionPose = QVec::vec3(pose6D[3], pose6D[4], pose6D[5]); 
	QVec rotaciones = rotacionPose - rot;
		
	QVec ambos(6);
	for(int i=0; i<3; i++)
	{
		ambos[i] = traslaciones[i];
		ambos[i+3] = rotaciones[i];
	}
	
	float dist = ambos.norm2();
	// Si la distancia es mayor que 1cm troceamos la trayectoria:
	if(dist >0.01)
	{
		float Npuntos = dist / 0.01;
		
		for(float i=0; i<Npuntos; i++)
		{
			float landa = 1/(Npuntos) * i;
			
			QVec R = inner->transform("world", QVec::zeros(3), this->tip)*(1-landa) + pose6D*landa;
			subtargets.append(R); //añadimos subtarget a la lista.
		}
	}
}

void Target::print(const QString &msg)
{
	qDebug() << "-----TARGET BEGIN----------" << msg;
	if(targetType == POSE6D)
		qDebug() << "	TargetType: POSE6D";
	if(targetType == ALIGNAXIS)
		qDebug() << "	TargetType: ALIGNAXIS";
	if(targetType == ADVANCEAXIS)
		qDebug() << "	TargetType: ADVANCEAXIS";
	qDebug() << "	Tip " << tip;
	qDebug() << "	Activo " << activo;
	qDebug() << "	Pose6D" << pose6D;
	if(targetType == ALIGNAXIS)
		qDebug() << "Axis of the tip to be aligned" << axis;
	if(targetType == ADVANCEAXIS)
	{
		qDebug() << "Axis of the tip to move" << axis;
		qDebug() << "Distance to advance in m." << step;
	}
	qDebug() << "	Weights" << weights;
	qDebug() << "	Error vector" << errorVector;
	qDebug() << "	Error vector norm" << error;
	qDebug() << "	Name in Inner" << nameInInnerModel;
	if(finish == START)
		qDebug() << "	Status = START";
	if(finish == LOW_ERROR)
		qDebug() << "	Status = LOW_ERROR";
	if(finish == KMAX)
		qDebug() << "	Status = KMAX";
	if(finish == LOW_INCS)
		qDebug() << "	Status = LOW_INCS";
	if(finish == NAN_INCS)
		qDebug() << "	Status = NAN_INCS";
	qDebug() << " 	Start time" << startTime;
	qDebug() << " 	Running time" << runTime;
	qDebug() << " 	Elapsed time" << elapsedTime << "ms";
	qDebug() << "		Final angles after IK" << finalAngles;
	qDebug() << "-----TARGET END-----------------";
}
