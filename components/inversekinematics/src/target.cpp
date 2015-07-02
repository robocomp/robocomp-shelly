#include "target.h"
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Default constructor.
 */ 
Target::Target()
{
	state 		= TargetState::IDLE;
	identifier 	= 0;
	runTime 	= QTime::currentTime();
	divided	= false;
}
/**
 * \brief Cosntructor for POSE6D targets.
 * @param id_
 * @param type_ target type --> POSE6D
 * @param pose_ qvec 6 with the target pose
 * @param weights_ qvec 6 with the weights of each traslation and rotation component.
 * @param divided_
 */ 
Target::Target(int id_,const QVec &pose_, const QVec &weights_, bool divided_, TargetType type_)
{
	identifier	= id_;
	state 		= TargetState::IDLE;
	type 		= type_;
	pose 		= pose_;
	weight		= weights_;
	runTime  	= QTime::currentTime();
	divided		= divided_;
}
/**
 * \brief Constructor for ADVANCEAXIS Targets.
 * @param type_ target type --> ADVANCEAXIS
 * @param axis_ Target axis to be aligned with
 * @param step_ step to advance along axis
 */ 
Target::Target(int id_, const QVec& axis_, float step_, TargetType type_)
{
	identifier	= id_;
	state 		= TargetState::IDLE;
	type 		= type_;
	axis 		= axis_;
	step		= step_;
	runTime		= QTime::currentTime();
	divided		= false;
}
/**
 * \brief Constructor for ALIGNAXIS targets.
 * @param type_ target type --> ALIGNAXIS
 * @param pose_ qvec 6 with the target pose
 * @param weights_ qvec 6 with the weights of each traslation and rotation component.
 * @param axis_ Target axis to be aligned with
 */ 
Target::Target(int id_, const QVec &pose_, const QVec &weights_, const QVec &axis_, TargetType type_)
{
	identifier	= id_;
	state 		= TargetState::IDLE;
	type 		= type_;
	pose 		= pose_;
	weight		= weights_;
	axis 		= axis_;
	runTime 	= QTime::currentTime();
	divided		= false;
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
	identifier       = target.getTargetIdentifier();
	divided          = target.getTargetDivided();
	type 	         = target.getTargetType();
	state	         = target.getTargetState();
	if(state==TargetState::FINISH)
		finalstate   = target.getTargetFinalState();
	nameInInnerModel = target.getTargetNameInInnerModel();
	finalangles      = target.getTargetFinalAngles();
	pose	         = target.getTargetPose();   //std::swap(pose, target.getTargetPose());
	weight	         = target.getTargetWeight(); //std::swap(weight, target.getTargetWeight());
	
	return *this;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method returns the identifier of the target.
 * @return int identifier
 */ 
int Target::getTargetIdentifier()
{
	return identifier;
}
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
 * \brief This method returns the finalstate of the target if it is finised.
 * @return Target::TargetFinalState finalstate
 */ 
Target::TargetFinalState Target::getTargetFinalState()
{
	if(state == Target::TargetState::FINISH)
		return finalstate;
	else
		throw ("El target aun no ha terminado");
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
/**
 * \brief This method returns the vector error, the traslation error norm and the rotation 
 * error norm.
 * @param errorT traslation error norm
 * @param errorR rotation error norm.
 * @return QVec error
 */ 
QVec Target::getTargetError(float& errorT, float& errorR)
{
	errorT = QVec::vec3(errorvector.x(),  errorvector.y(),  errorvector.z()).norm2();
	errorR = QVec::vec3(errorvector.rx(), errorvector.ry(), errorvector.rz()).norm2();
	
	return errorvector;
}
/**
 * \brief This method returns the final angles with the target ends his execution.
 * @return QVec finalangles
 */ 
QVec Target::getTargetFinalAngles()
{
	return finalangles;
}
/**
 * \brief This method calculates the seconds that the target is being executed.
 * @return float seconds
 */ 
float Target::getTargetTimeExecution()
{
	if(state == TargetState::IDLE)
		return 0;
	else
		return runTime.elapsed()/1000;
}
/**
 * \brief This method returns the flag divided.
 * @return bool
 */ 
bool Target::getTargetDivided()
{
	return divided;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this method changes the value of the target identifier
 * @param id_ the new value of the target's identifier.
 */ 
void Target::setTargetIdentifier(int id_)
{
	identifier = id_;
}
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
	if(state==TargetState::IDLE and state_==TargetState::IN_PROCESS)
		runTime.start();
	
	state = state_;
}
/**
 * \brief This method changes the finalstate of the target, when it is finish
 * @param finalstate_
 */ 
void Target::setTargetFinalState(Target::TargetFinalState finalstate_)
{
	if(state == TargetState::FINISH)
		finalstate = finalstate_;
}
/**
 * \brief this method stores the error with that the target end his execution.
 * @param errors_
 */ 
void Target::setTargetError(QVec errors_)
{
	errorvector = errors_;
}
/**
 * \brief this method stores the final angles of the robot.
 * @param finalangles_
 */ 
void Target::setTargetFinalAngles(QVec finalangles_)
{
	finalangles = finalangles_;
}






