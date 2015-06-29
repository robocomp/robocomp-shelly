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
	enum	TargetType 			{POSE6D, ALIGNAXIS, ADVANCEAXIS};
	enum	TargetState			{IDLE, IN_PROCESS, FINISH};
	enum 	TargetFinalState	{LOW_ERROR, KMAX, LOW_INCS, NAN_INCS};

private:
	/// GENERAL ATTRIBUTES
	int					identifier;
	bool				divided;
	QString				nameInInnerModel;
	QTime 				runTime;
	QVec				finalangles;
	QVec				errorvector;
	TargetType			type;
	TargetState			state;
	TargetFinalState 	finalstate;
	/// ATTRIBUTES OF POSE6D TARGET
	QVec				pose;
	QVec				weight;
	/// ATTRIBUTES ADVANCEAXIS TARGET
    QVec 				axis;
	float 				step;

public:
	Target	();
	Target(int id_,const QVec &pose_, const QVec &weights_, bool divided_,		TargetType type_=TargetType::POSE6D);		/// CONSTRUCTOR FOR POSE6D TARGET
	Target(int id_,const QVec &pose_, const QVec &weights_, const QVec &axis_, 	TargetType type_=TargetType::ALIGNAXIS);	/// CONSTRUCTOR FOR ALING AXIS
	Target(int id_,const QVec& axis_, float step_,								TargetType type_=TargetType::ADVANCEAXIS);	/// CONSTRUCTOR FOR ADVANCE AXIS					
	~Target	();
	
	Target 				operator=(Target target);
	int					getTargetIdentifier			();
	QString				getTargetNameInInnerModel	();
	TargetType			getTargetType				();
	TargetState			getTargetState				();
	TargetFinalState	getTargetFinalState			();
	QVec				getTargetPose				();
	QVec				getTargetWeight				();
	QVec				getTargetAxis				();
	float				getTargetStep				();
	float 			 	getTargetTimeExecution		();
	QVec				getTargetError				(float &errorT, float &errorR);
	QVec				getTargetFinalAngles		();
	bool				getTargetDivided			();
	
	void				setTargetIdentifier			(int id_);
	void				setTargetNameInInnerModel	(QString nameInInnerModel_);
	void				setTargetState				(Target::TargetState state_);
	void				setTargetFinalState			(Target::TargetFinalState finalstate_);
	void				setTargetError				(QVec errors_);
	void				setTargetFinalAngles		(QVec finalangles_);
	

	

};
#endif // TARGET_H