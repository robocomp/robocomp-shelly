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
	State state;
	string bodyPart;
	RoboCompBodyInverseKinematics::Pose6D pose6D; //vector con la pose del target.
	RoboCompBodyInverseKinematics::WeightVector weights; //vector con los pesos del target.
	
public:
	//CONSTRUCTORES Y DESTRUCTORES DE LA CLASE:
	Target(); //por defecto
	Target(const string bodyPart_, const RoboCompBodyInverseKinematics::Pose6D &pose6D_, const RoboCompBodyInverseKinematics::WeightVector &weights_); //parametrizado
	~Target();
	
	//METODOS PUT
	void changeState(Target::State state_);
	void changeBodyPart(string bodyPart_);
	void changePose6D(RoboCompBodyInverseKinematics::Pose6D pose6D_);
	void changeWeights(RoboCompBodyInverseKinematics::WeightVector weights_);
	
	//METODOS GET:
	Target::State getState();
	string getBodyPart();
	RoboCompBodyInverseKinematics::Pose6D getPose6D();
	RoboCompBodyInverseKinematics::WeightVector getWeights();

};

#endif // TARGET_H