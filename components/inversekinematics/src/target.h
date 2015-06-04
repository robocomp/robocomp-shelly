/* ------------------------------------------------------------------
 *  CLASS TARGET
 * IT REPRESETS A TARGET POSITION OR GOAL POSE, WITH ONE TRASLATION IN
 * X, Y, AND Z, AND ONE ROTATION IN AXIS X, Y, AND Z. A TARGET HAS SOME
 * ATTRIBUTES:
 * 		--> THE TYPE OF THE TARGET.
 * 		--> THE STATE OF THE TARGET.
 * 		--> THE POSE 6D (TX, TY, TZ,          RX, RY, RZ)
 * 		--> THE WEIGHTS VECTOR
 * ------------------------------------------------------------------*/ 
#ifndef TARGET_H
#define TARGET_H

#include <innermodel/innermodel.h>
#include <iostream>

using namespace std;

class Target
{
public:
	enum	TargetType 		{POSE6D, ALIGNAXIS, ADVANCEAXIS};
	enum	TargetState		{IDLE, IN_PROCESS, FINISH, LOW_ERROR, KMAX, LOW_INCS, NAN_INCS};
	
	Target	();
	Target(const QVec &pose_, const QVec &weights_, const float radius_,TargetType type_=TargetType::POSE6D, 	bool chop=false);	/// CONSTRUCTOR FOR POSE6D TARGET
	Target(const QVec &pose_, const QVec &weights_, const QVec &axis_, 	TargetType type_=TargetType::ALIGNAXIS);					/// CONSTRUCTOR FOR ALING AXIS
	Target(const QVec& axis_, float step_,								TargetType type_=TargetType::ADVANCEAXIS);					/// CONSTRUCTOR FOR ADVANCE AXIS					
	~Target	();
	
	Target 			operator=(Target target);
	
	QString			getTargetNameInInnerModel	();
	TargetType		getTargetType				();
	TargetState		getTargetState				();
	QVec			getTargetPose				();
	QVec			getTargetWeight				();
	QVec			getTargetAxis				();
	float			getTargetStep				();
	
	void			setTargetNameInInnerModel	(QString nameInInnerModel_);
	void			setTargetState				(Target::TargetState state_);
	void			setTargetError				(QVec errors_);
	void			setTargetFinalAngles		(QVec finalangles_);
	
private:
	/// GENERAL ATTRIBUTES
	QString			nameInInnerModel;
	QVec			finalangles;
	QVec			errorvector;
	float			error;
	TargetType		type;
	TargetState		state;
	/// ATTRIBUTES OF POSE6D TARGET
	QVec			pose;
	QVec			weight;
	float 			radius;
	/// ATTRIBUTES ADVANCEAXIS TARGET
    QVec 			axis;
	float 			step;

};
#endif // TARGET_H