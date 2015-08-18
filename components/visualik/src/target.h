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

#include <iostream>
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qqueue.h>
#include <InverseKinematics.h>

using namespace std;
using namespace RoboCompInverseKinematics;

class Target
{
public:
	//Atributos, estructuras y enumerados publicos:
	enum class State {IDLE, WAITING, IN_PROCESS, NOT_RESOLVED, RESOLVED};
	
 private:
	//Atributos privados:
	int 	idIK;
	int     idVIK;
	State 	state;
	string 	bodyPart;
	QVec	pose;
	QVec	weights;
	QTime 	runTime;
	
public:
	//CONSTRUCTORES Y DESTRUCTORES DE LA CLASE:
	Target	(); //por defecto
	Target	(const string bodyPart_, const RoboCompInverseKinematics::Pose6D &pose6D_, const RoboCompInverseKinematics::WeightVector &weights_); //parametrizado
	~Target	();
	
	//METODOS PUT
	void	setID_IK	(int id_);
	void	setID_VIK   (int id_);
	void 	setState	(Target::State state_);
	void 	setBodyPart	(string bodyPart_);
	void 	setPose		(QVec pose_);
	void	setWeights	(QVec weights_);
	void 	setPose		(RoboCompInverseKinematics::Pose6D       pose6D_);
	void 	setWeights	(RoboCompInverseKinematics::WeightVector weights_);
	void	setRunTime	();

	//METODOS GET:
	int                             getID_IK	();
	int                             getID_VIK   ();
	Target::State                   getState	();
	string                          getBodyPart	();
	QVec                            getPose		();
	QVec			getWeights	();
	float			getRunTime	();
	RoboCompInverseKinematics::Pose6D 		getPose6D	();
	RoboCompInverseKinematics::WeightVector getWeights6D();

};

#endif // TARGET_H