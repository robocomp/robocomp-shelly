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
	internalError           = QVec::vec6(0,0,0,0,0,0);
	internalErrorINV        = QVec::vec6(0,0,0,0,0,0);
	targetError           = QVec::vec6(0,0,0,0,0,0);
	targetErrorINV        = QVec::vec6(0,0,0,0,0,0);
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
	// tagPose as seen by apriltagscomp (rotation zero faces the camera)
	tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);

	// Update the innermodel pose according to the desired zero position of the hand's april tag (pi/2)
	const QString SH1 = "marca-" + tip + "-segun-head";
	im->updateTransformValues(SH1, tag.tx, tag.ty, tag.tz,    tag.rx, tag.ry, tag.rz);
	const QString SH2 = "marca-" + tip + "-segun-head2";
	im->updateTransformValues(SH2,      0,      0,      0,   -M_PI_2,       0,     0);

	// Obtain the visual pose of the hand from the root reference frame and update innermodel accordingly
	visualPose = im->transform6D("root", SH2);
	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());

	gettimeofday(lastUpdate, NULL);

	updateInternalError();
	updateTargetError();
	
// 	getInternalErrorInverse().print("invErr");
}


void VisualHand::updateTargetError()
{
	getErrors("visual_hand", "target", targetError, targetErrorINV);
}


void VisualHand::updateInternalError()
{
	getErrors("visual_hand", tip, internalError, internalErrorINV);
}

void VisualHand::getErrors(QString visual, QString source, QVec &normal, QVec &inverse)
{
	// Compute translation error
	const QVec errorT = im->transform(source, visual);
	// Compute rotation error
	QMat error_from_tip = im->getRotationMatrixTo(visual, source);
	// Generate resulting vectors
	normal.inject(errorT, 0);
	normal.inject(error_from_tip.extractAnglesR_min(),3);
	inverse.inject(-errorT, 0);
	inverse.inject(error_from_tip.invert().extractAnglesR_min(),3);
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
	QVec poseCorregida = im->transform("root", QVec::zeros(3), tip) + QVec::vec3(internalError.x(), internalError.y(), internalError.z());
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(rotCorregida,3); 	//Diremos que la rotacion es la misma que el tip

	visualPose = correccionFinal;
}


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


QVec VisualHand::getTargetError()
{
	return targetError;
}
QVec VisualHand::getTargetErrorInverse()
{
	return targetErrorINV;
}

QVec VisualHand::getInternalError()
{
	return internalError;
}
QVec VisualHand::getInternalErrorInverse()
{
	return internalErrorINV;
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

