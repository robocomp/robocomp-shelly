/* ------------------------------------------------------------------
 *  CLASS INVERSED KINEMATIC
 * IT DOES THE INVERSED KINEMATIC CALCULATIONS
 * ------------------------------------------------------------------*/ 
#ifndef INVERSEDKINEMATIC_H
#define INVERSEDKINEMATIC_H

#include <innermodel/innermodel.h>
#include <iostream>

#include "bodypart.h"
#include "target.h"

using namespace std;

class InversedKinematic 
{
public:
			InversedKinematic	();
			~InversedKinematic	();
	
	void 	solveTarget			(BodyPart *bodypart_, 	InnerModel *innermodel_);
	
private:
	QMat 	jacobian			(QVec motors);				
	QVec 	computeErrorVector	(Target &target);	
	void 	levenbergMarquardt	(Target &target);
	
	QVec	computeAngles		();
	void	computeFloatModule	(QVec &angles, float mod);
	
	void 	updateAngles		(QVec new_angles);
	bool	outLimits			(QVec &angles, QVec &motors);

	
private:
	InnerModel	*innermodel;
	BodyPart	*bodypart;
	
};
#endif