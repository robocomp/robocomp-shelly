/* ------------------------------------------------------------------
 *  CLASS INVERSED KINEMATIC
 * IT DOES THE INVERSED KINEMATIC CALCULATIONS
 * ------------------------------------------------------------------*/ 
#ifndef INVERSEDKINEMATIC_H
#define INVERSEDKINEMATIC_H

#include <innermodel/innermodel.h>
#include <iostream>
#include <QtCore>


#include "bodypart.h"
#include "target.h"

using namespace std;

class InversedKinematic 
{
public:
	InversedKinematic();
	~InversedKinematic();
	
	void solveTarget(BodyPart *bodypart_, InnerModel *innermodel_);
	bool deleteTarget();
	
private:
	QMat jacobian(QVec motors);				
	QVec computeErrorVector(Target &target);	
	void levenbergMarquardt(Target &target);
	
	QVec computeAngles();
	void computeFloatModule(QVec &angles, float mod);
	void updateAngles(QVec new_angles);
	
	bool outLimits (QVec &angles, QVec &motors);
	QStringList	checkMotors();

	
private:
	InnerModel *innermodel;
	BodyPart *bodypart;
	
	int repetitions;	
};
#endif