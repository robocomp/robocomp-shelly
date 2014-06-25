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
 * @brief Constructor for ADVANCE ALONG AXIS target
 * 
 * @param tt type of the damn target
 * @param inner variable of innerModel
 * @param tip end effector of the cinematics concatenation.
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
	QVec w(6);w.set((T)1);
	this->weights = w;
	this->step = step;
	this->startTime = QTime::currentTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
	this->executed = false;
	this->initialTipPose = QVec(6);
	this->finalTipPose = QVec(6);
	this->initialAngles = QVec();
	this->finalAngles = QVec();
	this->chopped = false;
	removal = false;
	atTarget = false;
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
	this->weights = weights;
	this->startTime = QTime::currentTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
	this->executed = false;
	this->initialTipPose = QVec(6);
	this->finalTipPose = QVec(6);
	this->initialAngles = QVec();
	this->finalAngles = QVec();
	this->chopped = false;
	removal = false;
	atTarget = false;
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
Target::Target(Target::TargetType tt, InnerModel* inner, const QString &tip, const QVec &pose6D, const QVec& weights, bool chop)
{
	this->activo = true;
	this->pose6D = pose6D;
	this->tip = tip;
	this->inner = inner;
	this->targetType = tt;
	this->axis = QVec();
	this->weights = weights;
	this->startTime = QTime::currentTime();
	this->runTime = QTime();
	this->iter = 0;
	this->elapsedTime = 0;
	this->finish = START;
	this->executed = false;
	this->finalTipPose = QVec(6);
	this->initialTipPose = QVec(6);
	this->initialAngles = QVec();
	this->finalAngles = QVec();
	this->chopped = false;
	removal = false;
	atTarget = false;

}

/**
 * \brief Destructor por defecto
 */ 
Target::~Target()
{
}

/*------------------------------------------------------------------------------------------------------*
 *		      									MÉTODOS PÚBLICOS										*
 *------------------------------------------------------------------------------------------------------*/
/**
 * @brief SET INITIAL ANGLES Method. 
 * Save the initial angles of the joints in Variable initialAngles
 * 
 * @param motors names's joints stored in a list.
 * 
 * @return void
 */
void Target::setInitialAngles(const QStringList &motors)
{
	initialAngles.clear();

	foreach( QString motor, motors)
		initialAngles.append(inner->getJoint(motor)->getAngle());
}

/**
 * @brief ANNOTATE INITIAL TIP POSE Method.
 * Save the initial pose (translations and rotations) of the end effector. 
 * 
 * @return void
 */
void Target::annotateInitialTipPose()
{
	// Guardamos las traslaciones y las rotaciones del efector final vistas desde el MUNDO.
	initialTipPose.inject(inner->transform("world", QVec::zeros(3), getTipName()),0);
	initialTipPose.inject(inner->getRotationMatrixTo("world",getTipName()).extractAnglesR_min(),3);
}
	
/**
 * @brief ANNOTATE FINAL TIP POSE Method.
 * Save the final pose (translations and rotations) of the end effector. 
 * 
 * @return void
 */
void Target::annotateFinalTipPose()
{
	// Guardamos las traslaciones y las rotaciones del efector final vistas desde el MUNDO.
	finalTipPose.inject(inner->transform("world", QVec::zeros(3), getTipName()),0);
	finalTipPose.inject(inner->getRotationMatrixTo("world",getTipName()).extractAnglesR_min(),3);
}
	
	
/**
 * @brief PRINT Method. 
 * Print the internal information of the target
 * 
 * @return void
 */
void Target::print(const QString &msg)
{
	qDebug() << "-----TARGET BEGIN----------" << msg;
	
	// Información propia del tip o efector final al que pertenece.
	qDebug() << "-----> Information of the END EFFECTOR: ";
	qDebug() << "	Tip " << tip;
	qDebug() << "	Initial Tip pos." << initialTipPose;
	qDebug() << "	Final Tip pos." << finalTipPose;

	// Información propia del target
	qDebug() << "-----> Information of the TARGET: ";
	qDebug() << "	Name in Inner" << nameInInnerModel;
	// Imprimimos tipo del target:
	if(targetType == POSE6D) 		qDebug() << "	TargetType: POSE6D";
	if(targetType == ALIGNAXIS)		qDebug() << "	TargetType: ALIGNAXIS";
	if(targetType == ADVANCEAXIS)	qDebug() << "	TargetType: ADVANCEAXIS";
	
	qDebug() << "	Activo " << activo;
	qDebug() << "	Pose6D" << pose6D;
	qDebug() << "	Chopped" << chopped;
	qDebug() << "	pose6DChopped" << pose6DChopped;
	
	if(executed == true)			qDebug() << "	Target Executed on Robot!";
	if(targetType == ALIGNAXIS)		qDebug() << "	Axis of the tip to be aligned" << axis;
	if(targetType == ADVANCEAXIS){	qDebug() << "	Axis of the tip to move" << axis;		qDebug() << "	Distance to advance in m." << step;	}
	
	qDebug() << "	Weights" << weights;
	qDebug() << "	Error vector" << errorVector;
	qDebug() << "	Error vector norm" << errorVector.norm2();
	
	if(finish == START)				qDebug() << "	Status = START";
	if(finish == LOW_ERROR)			qDebug() << "	Status = LOW_ERROR";
	if(finish == KMAX)				qDebug() << "	Status = KMAX";
	if(finish == LOW_INCS)			qDebug() << "	Status = LOW_INCS";
	if(finish == NAN_INCS)			qDebug() << "	Status = NAN_INCS";
	
	qDebug() << " 	Start time" << startTime;
	qDebug() << " 	Running time" << runTime;
	qDebug() << " 	Elapsed time" << elapsedTime << "ms";
	qDebug() << "	Angles increment after IK" << finalAngles - initialAngles;
	qDebug() << "	Initial angles after IK" << initialAngles;
	qDebug() << "	Final angles after IK" << finalAngles;
	
	if( executed == true) 						qDebug() << "	Target Executed!";
	if( removal == true and atTarget==false)	qDebug() << " 	Attention: Marked for removal without success";
	if( atTarget == true)						qDebug() << " 	Attention: Marked for removal with success";
	
	qDebug() << "-----TARGET END-----------------";
}

