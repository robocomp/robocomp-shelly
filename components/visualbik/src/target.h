/*########################################################
 * MARAVILLA DE DOCUMENTACION INTERNA
 * -----------------------------------
 * Clase TARGET creada para almacenar y manejar los datos 
 * procedentes de un target tipico de cinematica inversa.
 * ATRIBUTOS:
 * 		active: bandera que indica si el t
 *########################################################*/ 
#ifndef TARGET_H
#define TARGET_H

#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qqueue.h>
#include <BodyInverseKinematics.h>

using namespace std;
using namespace RoboCompBodyInverseKinematics;

class Target
{
public:
	//Atributos, estructuras y enumerados publicos:
	enum class State {IDLE, WAITING, IN_PROCESS, RESOLVED};
	
 private:
	//Atributos privados:
	State 	state;
	string 	bodyPart;
	QVec	pose;
	QVec	weights;
	
public:
	//CONSTRUCTORES Y DESTRUCTORES DE LA CLASE:
	Target	(); //por defecto
	Target	(const string bodyPart_, const RoboCompBodyInverseKinematics::Pose6D &pose6D_, const RoboCompBodyInverseKinematics::WeightVector &weights_); //parametrizado
	~Target	();
	
	//METODOS PUT
	void 	setState	(Target::State state_);
	void 	setBodyPart	(string bodyPart_);
	void 	setPose		(QVec pose_);
	void	setWeights	(QVec weights_);
	void 	setPose		(RoboCompBodyInverseKinematics::Pose6D       pose6D_);
	void 	setWeights	(RoboCompBodyInverseKinematics::WeightVector weights_);
	
	void setRX(float rx) {pose[3] = rx;};
	void setRY(float ry) {pose[4] = ry;};
	void setRZ(float rz) {pose[5] = rz;};

	//METODOS GET:
	Target::State 	getState	();
	string 			getBodyPart	();
	QVec			getPose		();
	QVec			getWeights	();
	RoboCompBodyInverseKinematics::Pose6D 		getPose6D	();
	RoboCompBodyInverseKinematics::WeightVector getWeights6D();

};

#endif // TARGET_H