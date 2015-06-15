#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto. 
 */ 
VisualHand::VisualHand(InnerModel *im_, QString tip_)
{
	this->im = im_;
	this->x = this->y = this->z = this->rx = this->ry = this->rz = -9999;
	this->tip = tip_;
	this->lastUpdate = new timeval;
}

/**
 * \brief Destructor por defecto
 */ 
VisualHand::~VisualHand()
{

}

void VisualHand::setVisualPose_APRIL(RoboCompAprilTags::tag tag)
{
	QVec tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	
	// Metemos en el InnerModel la marca vista por la RGBD:	
	InnerModelNode *nodeParent = this->im->getNode("rgbd_transform");
	if( this->im->getNode("marca-segun-head") == NULL)
	{
		InnerModelTransform *node = this->im->newTransform("marca-segun-head", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
		InnerModelTransform *node2 = this->im->newTransform("marca-segun-head2", "static", node, 0, 0, 0,       0., 0, 0,      0.);
		node->addChild(node2);
	}
	this->im->updateTransformValues("marca-segun-head", tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	this->im->updateTransformValues("marca-segun-head2", 0,0,0,   -M_PI_2,0,M_PI);
	
	//Pasamos del marca-segun-head al mundo:
	QVec ret = this->im->transform("root", tagPose, "rgbd");
	QVec ret2 = this->im->transform("root", tagPose, "marca-segun-head2");
	this->x = ret(0);
	this->y = ret(1);
	this->z = ret(2);
	
	this->rx = ret2(3);
	this->ry = ret2(4);
	this->rz = ret2(5);
	
		
	/*QVec ret = this->im->transform("root", tagPose, "rgbd");
	std::cout<<"SISTEMA ROOT: "<<ret(0)<<" "<<ret(1)<<" "<<ret(2)<<" "<<ret(3)<<" "<<ret(4)<<" "<<ret(5)<<std::endl;
	this->x = ret(0);
	this->y = ret(1);
	this->z = ret(2);

	if (false)
	{
	// 	const Rot3D offsetRotation(-M_PI_2, 0, -M_PI_2);
		const Rot3D offsetRotation(0,0,0);
		const Rot3D aprilRotation(ret(3), ret(4), ret(5));

		const QMat finalR = offsetRotation * aprilRotation;
		
		const QVec finalAngles = finalR.extractAnglesR_min();
		
		this->rx = finalAngles(0);
		this->ry = finalAngles(1);
		this->rz = finalAngles(2);
	}
	else
	{
		this->rx = ret(3);
		this->ry = ret(4);
		this->rz = ret(5);
	}*/

	gettimeofday(this->lastUpdate, NULL);

}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 												METODOS PUT/SET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * \brief Metodo SET VISUAL POSE
 * Almacena en sus atributos [X, Y, Z, RX, RY, RZ] las coordenadas de traslacion y rotacion
 * de la marcda de apriltags que recibe como parametro de entrada. Actualiza el tiempo de
 * lastUpdate. PASA DE RGBD A ROOT!!!!!!!!!
 * @param tag marca de apriltags con la posicion de la mano.
 */ 
// void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
// {
// 	QVec tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
// 	QVec ret = this->im->transform("root", tagPose, "rgbd");
// 	this->x = ret(0);
// 	this->y = ret(1);
// 	this->z = ret(2);
// 	this->rx = ret(3);
// 	this->ry = ret(4);
// 	this->rz = ret(5);
// 	
// 	gettimeofday(this->lastUpdate, NULL);
// }

/**
 * \brief Metodo SET INTERNAL POSE
 * Almacena en el atributo internalPose las coordenadas de traslacion y de rotacion
 * de la mano en la que cree el robot que esta.
 * @param pose es la pose6D de donde cree el robot que tiene el
 */ 
void VisualHand::setInternalPose(RoboCompBodyInverseKinematics::Pose6D pose)
{
	this->internalPose.x = pose.x;
	this->internalPose.y = pose.y;
	this->internalPose.z = pose.z;
	
	this->internalPose.rx = pose.rx;
	this->internalPose.ry = pose.ry;
	this->internalPose.rz = pose.rz;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo SECONDS ELAPSED
 * Devuelve los segundos que han pasado desde que se actualizo la pose visual por ultima vez.
 * @return double segundos
 */ 
double VisualHand::secondsElapsed()
{
	if(this->x==-9999 and this->y==-9999 and this->z==-9999)
		return 0;

	static timeval currentTimeval;
	gettimeofday(&currentTimeval, NULL);
	
	const double secs  = currentTimeval.tv_sec  - lastUpdate->tv_sec;
	const double usecs = currentTimeval.tv_usec - lastUpdate->tv_usec;
	return secs + usecs/1000000;
}

/**
 * \brief Metodo GET ERROR
 * Calcula el error que existe entre la pose de la mano que la camara esta viendo
 * y la posicion del target al que quiere ir.
 * TODO ARREGLAR EL ERROR ENTRE ANGULOS
 * @param target target al que quiere ir.
 * @return pose6D error
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getError(Pose6D target)
{
	RoboCompBodyInverseKinematics::Pose6D error;
	
	error.x = target.x - this->x;
	error.y = target.y - this->y;
	error.z = target.z - this->z;
	
	//Sacamos los errores de rotacion: El error de rotacion tiene que 
	//estar recogido en la matriz de rotacion del target al tip. El 
	//target esta en el sistema de referencia del mundo.
	QMat matriz_Target_to_Tip = this->im->getRotationMatrixTo("target", "visual_hand");
	QVec angles = matriz_Target_to_Tip.extractAnglesR_min();
	error.rx = angles[0];
	error.ry = angles[1];
	error.rz = angles[2];
	
	return error;
}

/**
 * \brief Metodo GET VISUAL POSE
 * Devuelve las coordenadas de traslacion y de orientacion de la marca vista
 * por la camara del robot (¿hay que poner algun elapsed time?)
 * @return pose6D
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getVisualPose()
{
	RoboCompBodyInverseKinematics::Pose6D poseVisual;
	poseVisual.x = this->x;
	poseVisual.y = this->y;
	poseVisual.z = this->z;
	
	poseVisual.rx = this->rx;
	poseVisual.ry = this->ry;
	poseVisual.rz = this->rz;
	
	return poseVisual;
}

/**
 * \brief Metodo GET INTERNAL POSE
 * Devuelve las coordenadas de traslacion y de rotacion de la marca
 * que el robot cree tener en cierta posicion.
 * @return pose6d
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getInternalPose()
{
	return this->internalPose;
}

