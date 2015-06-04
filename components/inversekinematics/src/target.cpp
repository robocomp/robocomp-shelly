#include "target.h"
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Default constructor.
 */ 
Target::Target()
{
	state = TargetState::IDLE;
}
/**
 * \brief Cosntructor for POSE6D targets.
 * @param type_ target type --> POSE6D
 * @param pose_ qvec 6 with the target pose
 * @param weights_ qvec 6 with the weights of each traslation and rotation component.
 * @param chop
 */ 
Target::Target(const QVec &pose_, const QVec &weights_, const float radius_, TargetType type_, bool chop)
{
	nameInInnerModel = "target";
	state 	= TargetState::IDLE;
	type 	= type_;
	pose 	= pose_;
	weight	= weights_;
	radius 	= radius_;
}
/**
 * \brief Constructor for ADVANCEAXIS Targets.
 * @param type_ target type --> POSE6D
 * @param axis_ Target axis to be aligned with
 * @param step_ step to advance along axis
 */ 
Target::Target(const QVec& axis_, float step_, TargetType type_)
{
	state 	= TargetState::IDLE;
	type 	= type_;
	axis 	= axis_;
	step	= step_;
}
/**
 * \brief Constructor for ALIGNAXIS targets.
 * @param type_ target type --> POSE6D
 * @param pose_ qvec 6 with the target pose
 * @param weights_ qvec 6 with the weights of each traslation and rotation component.
 * @param axis_ Target axis to be aligned with
 */ 
Target::Target(const QVec &pose_, const QVec &weights_, const QVec &axis_, 	TargetType type_)
{
	state 	= TargetState::IDLE;
	type 	= type_;
	pose 	= pose_;
	weight	= weights_;
	axis 	= axis_;
}

/**
 * \brief Default destructor.
 */ 
Target::~Target()
{
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method equals two targets.
 * @return Target the current target with new values
 */ 
Target Target::operator=( Target target ) 
{
	type 	= target.getTargetType();
	state	= target.getTargetState();
	
	pose	=	target.getTargetPose();   //std::swap(pose, target.getTargetPose());
	weight	=	target.getTargetWeight(); //std::swap(weight, target.getTargetWeight());
	
	return *this;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method returns the name of the target in innermodel.
 * @return QString nameInInnerModel
 */ 
QString Target::getTargetNameInInnerModel()
{
	return nameInInnerModel;
}
/**
 * \brief This method returns the type of the target.
 * @return Target::TargetType type
 */ 
Target::TargetType Target::getTargetType()
{
	return type;
}
/**
 * \brief This method returns the state of the target.
 * @return Target::TargetState state
 */ 
Target::TargetState Target::getTargetState()
{
	return state;
}
/**
 * \brief This method returns the pose 6D of the target.
 * @return QVec pose
 */ 
QVec Target::getTargetPose()
{
	return pose;
}
/**
 * \brief This method returns the weights vector of each component of the pose.
 * @return QVec weight
 */ 
QVec Target::getTargetWeight()
{
	return weight;
}
/**
 * \brief This method returns the axis of the target.
 * @return QVec axis
 */ 
QVec Target::getTargetAxis()
{
	return axis;
}
/**
 * \brief This method returns the step of the target
 * @return float step
 */ 
float Target::getTargetStep()
{
	return step;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method changes the value of the target name in innermodel
 * @param nameInInnerModel_ the new name of the target in innermodel
 */ 
void Target::setTargetNameInInnerModel(QString nameInInnerModel_)
{
	nameInInnerModel = nameInInnerModel_;
}
/**
 * \brief this method changes the vaule of the target state
 * @param state_ the new value of the target's state.
 */ 
void Target::setTargetState(Target::TargetState state_)
{
	state = state_;
	qDebug()<<"HEy!: "<<state;
}
/**
 * \brief this method stores the error with that the target end his execution.
 * @param errors_
 */ 
void Target::setTargetError(QVec errors_)
{
	errorvector = errors_;
	error 		= errors_.norm2();
}
/**
 * \brief this method stores the final angles of the robot.
 * @param finalangles_
 */ 
void Target::setTargetFinalAngles(QVec finalangles_)
{
	finalangles = finalangles_;
}






