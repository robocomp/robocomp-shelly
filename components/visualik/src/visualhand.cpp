#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>
/**
* \brief The VisualHand constructor must receive a pointer to an InnerModel object and the name of the arm's tip.
*/
VisualHand::VisualHand(InnerModel *im_, QString tip_)
{
	im                      = im_;
	tip                     = tip_;
	visualPose              = QVec::vec6(0,0,0,0,0,0);
	errorInternal_Visual    = QVec::vec6(0,0,0,0,0,0);
	lastUpdate              = new timeval;
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

/**
 * \brief Updates the hand's possition according to an April tag and the time.
 * Also, it calculates the error between the visualHand and the internal hand
 */
void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
{
	const QString SH1 = "marca-" + tip + "-segun-head";
	const QString SH2 = "marca-" + tip + "-segun-head2";
	
	tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	// Metemos en el InnerModel la marca vista por la RGBD:
	im->updateTransformValues(SH1, tag.tx, tag.ty, tag.tz,    tag.rx, tag.ry, tag.rz);
	im->updateTransformValues(SH2,      0,      0,      0,    -M_PI_2,       0,     0);
	//Pasamos del marca-segun-head al mundo:
	QVec ret = im->transform6D("root", SH2);
	visualPose[0] = ret(0);
	visualPose[1] = ret(1);
	visualPose[2] = ret(2);
	visualPose[3] = ret(3);
	visualPose[4] = ret(4);
	visualPose[5] = ret(5);
	//Actualizamos la posicion del nodo visual_hand:
	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());
	gettimeofday(lastUpdate, NULL);
	
	// Calculo el error de traslacion:
	const QVec errorT = im->transform(tip, "visual_hand");

	// Calculo del error de rotacion:
	QMat error_from_tip = im->getRotationMatrixTo(tip, "visual_hand");

	//Error total
	errorInternal_Visual.inject(errorT, 0);
	errorInternal_Visual.inject(error_from_tip.extractAnglesR_min(),3);

	errorInternalINV_Visual.inject(-errorT, 0);
	errorInternalINV_Visual.inject(error_from_tip.invert().extractAnglesR_min(),3);
	
	error_from_tip.invert().extractAnglesR_min().print("inv err R");
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
// 	const QVec error = im->transform6D("target", "visual_hand");
// 	return error;
	return errorInternal_Visual;
}
/**
* \brief Computes the inverse of the error from the visual position to a target position
* @param target the target
* @return qvec error
*/
QVec VisualHand::getErrorInverse()
{
<<<<<<< HEAD
// 	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());
// 	const QVec error = im->transform6D("visual_hand", "target");
// 	im->transform6D("root",        "target").print("target      pose_in_root");
// 	im->transform6D("root",             tip).print("belief      pose_in_root");
// 	im->transform6D("root",   "visual_hand").print("visual_hand pose_in_root");
// 	error.print("target desde visual");
// 	return error;
	return errorInternalINV_Visual;

=======
	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());
	
	const QVec error = im->transform6D("visual_hand", "target");

	//error.print("target desde visual");
	return error;
>>>>>>> 86fa073abbfe5a190a0cf1d2df1797cb0d3cd062
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

