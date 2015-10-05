#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>

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
	
	lastUpdate->tv_sec = 0;
}


VisualHand::~VisualHand()
{
	delete lastUpdate;
}

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
}


void VisualHand::setVisualPosewithInternalError()
{
// 	internalErrorINV.print("setVisualPosewithInternalError:: internalErrorINV");
	// Obtain the visual pose of the hand from the root reference frame and update innermodel accordingly
	visualPose = im->transform6D("root", internalError, "grabPositionHandR");
// 	visualPose.print("me imagino la mano (no la veo) en ");
	im->updateTransformValues("visual_hand", visualPose.x(), visualPose.y(), visualPose.z(), visualPose.rx(), visualPose.ry(), visualPose.rz());

// 	gettimeofday(lastUpdate, NULL);

// 	updateInternalError();
// 	updateTargetError();
}

void VisualHand::updateTargetError()
{
	getErrors("visual_hand", "target", targetError, targetErrorINV/*, true*/);
}

void VisualHand::updateInternalError()
{
	getErrors("visual_hand", tip, internalError, internalErrorINV);
}

void VisualHand::getErrors(QString visual, QString source, QVec &normal, QVec &inverse, bool debug)
{
	// Compute translation error
	const QVec errorT = im->transform(source, visual);
	if (debug)
	{
		qDebug() << "getErrors v(" << visual << ") src(" << source << ")";
		errorT.print("T");
	}

	// Compute rotation error
	QMat error_from_tip = im->getRotationMatrixTo(visual, source);
	
	// Generate resulting vectors
	for (int i=0; i<3; i++) normal(i) = errorT(i);
	if (debug)
		normal.print("nT");
	for (int i=0; i<3; i++) normal(i+3) = error_from_tip.extractAnglesR_min()(i);
	
	for (int i=0; i<3; i++) inverse(i) = -errorT(i);
	if (debug)
		inverse.print("iT");
	for (int i=0; i<3; i++) inverse(i+3) = error_from_tip.invert().extractAnglesR_min()(i);
}


double VisualHand::getSecondsElapsed()
{
	static timeval currentTimeval;
	gettimeofday(&currentTimeval, NULL);

	static bool first = true;
	if (first)
	{
		first = false;
		return 99999999999;
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

QVec VisualHand::getVisualPose()
{
	return visualPose;
}

QVec VisualHand::getInternalPose()
{
	QVec internalPose = im->transform6D("root", tip);
	return internalPose;
}


QString VisualHand::getTip() 
{ 
	return tip; 
}

