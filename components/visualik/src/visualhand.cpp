#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>
/**
* \brief The VisualHand constructor must receive a pointer to an InnerModel object and the name of the arm's tip.
*/
VisualHand::VisualHand(InnerModel *im_, QString tip_)
{
	im 						= im_;
	tip 					= tip_;
	visualPose 				= QVec::vec6(0,0,0,0,0,0);
	errorInternal_Visual 	= QVec::vec6(0,0,0,0,0,0);
	lastUpdate 				= new timeval;
	gettimeofday(lastUpdate, NULL);
	
	InnerModelNode *nodeParent = im->getNode("rgbd_transform");
	if(im->getNode("marca-" + tip + "-segun-head") == NULL)
	{
		nodeMarca = im->newTransform("marca-" + tip + "-segun-head", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(nodeMarca);
		nodeMarca2 = im->newTransform("marca-" + tip + "-segun-head2", "static", nodeMarca, 0, 0, 0,       0., 0, 0,      0.);
		nodeMarca->addChild(nodeMarca2);
	}
}

/**
* \brief Destructor.
*/
VisualHand::~VisualHand()
{
	delete lastUpdate;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 												METODOS PUT/SET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * \brief Updates the hand's possition according to an April tag and the time.
 * Also, it calculates the error between the visualHand and the internal hand
 */
void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
{
	QVec tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	// Metemos en el InnerModel la marca vista por la RGBD:
	im->updateTransformValues("marca-" + tip + "-segun-head", tag.tx, tag.ty, tag.tz,   tag.rx, tag.ry, tag.rz);
	im->updateTransformValues("marca-" + tip + "-segun-head2", 0,0,0,   -M_PI_2,0,M_PI);
	//Pasamos del marca-segun-head al mundo:
	QVec ret = im->transform6D("root", tagPose, "rgbd");
	visualPose[0] = ret(0);
	visualPose[1] = ret(1);
	visualPose[2] = ret(2);
	QVec ret2 = im->transform("root", tagPose, "marca-" + tip + "-segun-head2");
	visualPose[3] = ret2(3);
	visualPose[4] = ret2(4);
	visualPose[5] = ret2(5);
	//Actualizamos la posicion del nodo visual_hand:
	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());
	gettimeofday(lastUpdate, NULL);
	
	//TODO Calculo el error entre pose interna y pose visual.
	QVec tip_root 		= im->transform("root", QVec::zeros(3), tip);
	QVec visual_root	= im->transform("root", QVec::zeros(3), "visual_hand");
	QVec errorT 		= visual_root-tip_root;
// 	qDebug()<<"TIP in ROOT: "<<tip_root<<"\nVISUAL in ROOT: "<<visual_root<<"\nERROR T: "<<errorT<<"\nCORREGIDA T: "<<tip_root+errorT;
	
	//TODO Calculo del error de rotacion:
	QMat Mvisual_tip= im->getRotationMatrixTo(tip, "visual_hand");
	QMat Mtip_root	= im->getRotationMatrixTo("root", tip);
	QVec Avisual_tip= Mvisual_tip.extractAnglesR_min();
	QVec R1			= Mtip_root * QVec::vec3(Avisual_tip[0], 0, 0);
	QVec R2			= Mtip_root * QVec::vec3(0, Avisual_tip[1], 0);
	QVec R3			= Mtip_root * QVec::vec3(0, 0, Avisual_tip[2]);
	Rot3D M1(R1[0], R1[1],R1[2]);
	Rot3D M2(R2[0], R2[1],R2[2]);
	Rot3D M3(R3[0], R3[1],R3[2]);
	QMat Mfinal		=(M1*M2)*M3;
	QVec errorR 	= Mfinal.extractAnglesR_min();
//  	qDebug()<<"ERROR R: "<<errorR;
	
	//ANTES!!!!!!!!!!!!!!!:
	//const QVec errorInv 	= im->transform6D(tip, "visual_hand");
	//QVec errorInvP      	= QVec::vec3(errorInv(0), errorInv(1), errorInv(2));*/
	//errorInternal_Visual 	= im->getRotationMatrixTo("root", tip)*errorInvP;
	errorInternal_Visual.inject(errorT,0);
	errorInternal_Visual.inject(errorR,3);
// 	qDebug()<<"ERROR: "<<errorInternal_Visual;
// 	qFatal("fary");
}
/**
* \brief Updates the hand's possition according to direct kinematics.
*/
void VisualHand::setVisualPose(const QVec pose_)
{
	visualPose = pose_;
}
/**
 * \brief Actualiza la posicion de la marca visual con la posicion de la marca interna 
 * y el error calculado que existe entre la marca vista y la marca interna. Calcula la
 * pose (trasladandola) y la rotacion la asimila como la rotacion del tip en el mundo.
 */ 
void VisualHand::setVisualPosewithInternal()
{
	qDebug()<<"La camara no ve la marca...";
	
	QVec aux           = im->transform6D("root", tip);
	QVec rotCorregida  = QVec::vec3(aux.rx(), aux.ry(), aux.rz());
	QVec poseCorregida = im->transform("root", QVec::zeros(3), tip) + QVec::vec3(errorInternal_Visual.x(), errorInternal_Visual.y(), errorInternal_Visual.z());
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(rotCorregida,3); 	//Diremos que la rotacion es la misma que el tip

	visualPose = correccionFinal;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
* \brief Metodo SECONDS ELAPSED
* Devuelve los segundos que han pasado desde que se actualizo la pose visual por ultima vez.
* @return double segundos
*/
double VisualHand::getSecondsElapsed()
{
	static timeval currentTimeval;
	gettimeofday(&currentTimeval, NULL);

	static bool first = true;
	if (first)
	{
		first = false;
		return 0;
	}

	const double secs  = currentTimeval.tv_sec  - lastUpdate->tv_sec;
	const double usecs = currentTimeval.tv_usec - lastUpdate->tv_usec;
	return secs + usecs/1000000;
}
/**
* \brief Computes the error from the visual position to a target position
* @param target the target
* @return qvec error
*/
QVec VisualHand::getError()
{
	QVec target_root = im->transform("root", "target");
	QVec visual_root = im->transform("root", "visual_hand");
	QVec errorT      = target_root - visual_root;
	
	QVec aux    = im->transform6D("visual_hand", "target");
	QVec errorR = QVec::vec3(aux.rx(), aux.ry(), aux.rz());
	
	QVec errorFinal = QVec::zeros(6);
	errorFinal.inject(errorT, 0);
	errorFinal.inject(errorR, 3);
	return errorFinal;
	
// 	const QVec error = im->transform6D("target", "visual_hand");
	//QVec target_root = im->transform("root", QVec::Zeros(3), "target");
	//return error;
}
/**
* \brief Computes the inverse of the error from the visual position to a target position
* @param target the target
* @return qvec error
*/
QVec VisualHand::getErrorInverse()
{
	const QVec error = im->transform6D("visual_hand", "target");
	return error;
}
/**
* \brief Metodo GET VISUAL POSE
* Devuelve las coordenadas de traslacion y de orientacion de la marca vista
* por la camara del robot.
* @return QVEC
*/
QVec VisualHand::getVisualPose()
{
	return visualPose;
}
/**
* \brief Metodo GET INTERNAL POSE
* Devuelve las coordenadas de traslacion y de orientacion de la marca que el robot 
* siente internamente (lo que el cree)
* @return QVEC
*/
QVec VisualHand::getInternalPose()
{
	QVec internalPose = im->transform6D("root", tip);
	return internalPose;
}
/**
* \brief returns the name of the hand's tip.
* @return QString tip
*/ 
QString VisualHand::getTip() 
{ 
	return tip; 
}