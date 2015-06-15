/*########################################################
 * MARAVILLA DE DOCUMENTACION INTERNA
 * -----------------------------------
 * Clase VISUAL HAND creada para almacenar y manejar las poses
 * de las manos del robot (cuando digo poses me refiero a:
 * 		1) La pose que esta viendo la camara de la mano: this->x, this->y, this->z, this->rx, this->ry, this->rz
 * 		2) La pose en la que el robot cree tener la mano: internalPose->x, internalPose->y, internalPose->z, internalPose->rx, internalPose->ry, internalPose->rz,
 * ATRIBUTOS:
 * 		Los propios de Pose6D, que hereda.(Consulte BodyInverseKinematics.ice)
 * 		internalPose: una pose6D para almacenar la posicion en la que el robot cree tener la mano.
 * 		lastUpdate: ultimo tiempo en el que actualizo sus valores de pose visual
 *########################################################*/ 

#include <BodyInverseKinematics.h>
#include <AprilTags.h>

#include <sys/time.h>

#include <innermodel/innermodel.h>

class VisualHand : public RoboCompBodyInverseKinematics::Pose6D
{
public:
	//CONSTRUCTORES Y DESTRUCTORES DE LA CLASE:
	VisualHand(InnerModel *im_, QString tip_);
	~VisualHand();
	
	void setVisualPose_APRIL(RoboCompAprilTags::tag tag);
	void setInternalPose(RoboCompBodyInverseKinematics::Pose6D pose);
	double secondsElapsed();
	
 	RoboCompBodyInverseKinematics::Pose6D getError(Pose6D target);
	RoboCompBodyInverseKinematics::Pose6D getVisualPose();
	RoboCompBodyInverseKinematics::Pose6D getInternalPose();
	
private:
	RoboCompBodyInverseKinematics::Pose6D internalPose;
	timeval *lastUpdate;
	InnerModel *im;
	QString tip;
};