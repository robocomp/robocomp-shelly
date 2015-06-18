#include "target.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto
 */
Target::Target()
{
	state 		= State::IDLE;
	bodyPart 	= "";
	pose 		= QVec::vec6(0,0,0,0,0,0);
	weights		= QVec::vec6(0,0,0,0,0,0);
}
 /**
 * \brief Constructor parametrizado
 * Inicializa las estructuras que componen sus atributos de clase. 
 * @param bodyPart_ nombre de la parte del robot a la que pertenece el target.
 * @param pose_ pose de tres traslaciones y tres rotaciones que define las coordenadas y orientacion del target [X, Y, Z, RX, RY, RZ]
 * @param weights_ pesos de las traslaciones y las rotaciones
 */ 
Target::Target(const string bodyPart_, const RoboCompBodyInverseKinematics::Pose6D &pose_, const RoboCompBodyInverseKinematics::WeightVector &weights_)
{
	state 		= State::IDLE;
	bodyPart 	= bodyPart_;
	pose 		= QVec::vec6(pose_.x, pose_.y, pose_.z, pose_.rx, pose_.ry, pose_.rz);
	weights 	= QVec::vec6(weights_.x, weights_.y, weights_.z, weights_.rx, weights_.ry, weights_.rz);
}
/**
 * \brief Destructor por defecto
 */ 
Target::~Target()
{
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS PUT												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo SET STATE:
 * Cambia el valor del atributo STATE por el valor del parametro de entrada.
 * @param state_ nuevo estado
 */ 
void Target::setState(Target::State state_)		{	state = state_; }
/**
 * \brief Metodo SET BODY PART:
 * Cambia el valor del atributo BODY PART por el valor del parametro de entrada.
 * @param bodyPart_ nombre de la parte del robot.
 */ 
void Target::setBodyPart(string bodyPart_)		{	bodyPart = bodyPart_; }
/** 
 * \brief: Este metodo asigna el valor de la posicion pose_ al target.
 *@param pose_ es un QVEC
 */ 
void Target::setPose(QVec pose_)				{	pose	= pose_; }
/**
 * \brief Metodo SET WEIGHTS:
 * Cambia el valor del atributo WEIGHTS por el valor del parametro de entrada.
 * @param weights_ nuevo vector de pesos para las coordenadas de traslacion y de orientacion
 */ 
void Target::setWeights(QVec weights_)			{	weights = weights_; }
/** 
 * \brief: Este metodo asigna el valor de la posicion pose_ al target.
 *@param pose_ es un RoboCompBodyInverseKinematics::Pose6D
 */ 
void Target::setPose(RoboCompBodyInverseKinematics::Pose6D pose6D_)
{ 
	pose	= QVec::vec6(pose6D_.x, pose6D_.y, pose6D_.z, pose6D_.rx, pose6D_.ry, pose6D_.rz); 
}
/**
 * \brief Metodo SET WEIGHTS:
 * Cambia el valor del atributo WEIGHTS por el valor del parametro de entrada.
 * @param weights_ nuevo vector de pesos para las coordenadas de traslacion y de orientacion
 */ 
void Target::setWeights(RoboCompBodyInverseKinematics::WeightVector weights_)	
{	
	weights = QVec::vec6(weights_.x, weights_.y, weights_.z, weights_.rx, weights_.ry, weights_.rz); 
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo GET STATE
 * Devuelve el valor del atributo STATE: IDLE, IN PROCESS, RESOLVED
 * @return Target::State valor del state
 */ 
Target::State Target::getState()			{ return state; 	}

/**
 * \brief Metodo GET BODY PART
 * Devuelve el nombre de la parte del robot a la que va destinado el target.
 * @return string nombre de la parte del robot.
 */ 
string Target::getBodyPart()				{ return bodyPart; }

/**
 * \brief Metodo GET POSE 
 * Devuelve las coordenadas de traslacion y orientacion del target.
 * @return QVEC.
 */
QVec Target::getPose()						{ return pose;}
/**
 * \brief Metodo GET WEIGHTS
 * Devuelve el vector de pesos de las coordenadas de traslacion y orientacion del target.
 * @return QVEC
 */ 
QVec Target::getWeights()					{ return weights; }
/**
 * \brief Metodo GET POSE 6D
 * Devuelve las coordenadas de traslacion y orientacion del target.
 * @return POSE6D
 */
RoboCompBodyInverseKinematics::Pose6D Target::getPose6D	()
{
	Pose6D pose6D;
	pose6D.x = pose.x();
	pose6D.y = pose.y();
	pose6D.z = pose.z();
	pose6D.rx = pose.rx();
	pose6D.ry = pose.ry();
	pose6D.rz = pose.rz();
		
	return pose6D;
}
/**
 * \brief Metodo GET WEIGHTS 6D
 * Devuelve el vector de pesos de las coordenadas de traslacion y orientacion del target.
 * @return WeightVector
 */ 
RoboCompBodyInverseKinematics::WeightVector Target::getWeights6D()
{
	WeightVector weights6D;
	weights6D.x = weights.x();
	weights6D.y = weights.y();
	weights6D.z = weights.z();
	weights6D.rx = weights.rx();
	weights6D.ry = weights.ry();
	weights6D.rz = weights.rz();
		
	return weights6D;
}









