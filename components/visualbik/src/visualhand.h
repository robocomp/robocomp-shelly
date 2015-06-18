#include <BodyInverseKinematics.h>
#include <AprilTags.h>
#include <sys/time.h>

#include <innermodel/innermodel.h>

/**
 * \class VisualHand
 * \brief Stores the necessary information and provides some helper methods to work with the visual feedback of the position of the robot's hand
 * 		-visualPose: visual pose of the apriltag.
 * 		-lastUpdate: last time that the visual pose was updated.
 * 		-im:		 InnerModel
 * 		-errorInternal_Visual: error between the visual pose and the internal pose.
 * 		-nodeMarca & nodeMarca2: auxiliar nodes 
 **/
class VisualHand
{
public:

	VisualHand	(InnerModel *im_, QString tip_);
	~VisualHand	();

	
	void setVisualPose				(RoboCompAprilTags::tag tag);
	void setVisualPose				(const QVec pose_);
	void setVisualPosewithInternal	();
	
	double 	getSecondsElapsed	();
 	QVec 	getError			();
	QVec 	getErrorInverse		();
	QVec 	getVisualPose		();
	QVec 	getInternalPose		();
	QString getTip				();

private:
	QVec 				visualPose;
	timeval 			*lastUpdate;
	InnerModel 			*im;
	QString 			tip;
	QVec 				errorInternal_Visual;
	InnerModelTransform *nodeMarca, *nodeMarca2;
};