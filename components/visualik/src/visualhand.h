#include <iostream>
#include <InverseKinematics.h>
#include <AprilTags.h>
#include <sys/time.h>

#include <innermodel/innermodel.h>

using namespace std;


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

	VisualHand(InnerModel *im_, QString tip_);
	~VisualHand();

	
	void setVisualPose             (RoboCompAprilTags::tag tag);
	void setVisualPose             (const QVec pose_);
	void setVisualPosewithInternal ();

	/**
	* \brief Metodo SECONDS ELAPSED
	* Devuelve los segundos que han pasado desde que se actualizo la pose visual por ultima vez.
	* @return double segundos
	*/
	double getSecondsElapsed         ();

	void    updateTargetError();
	QVec    getTargetError           ();
	QVec    getTargetErrorInverse    ();
	
	void    updateInternalError();
	QVec    getInternalError         ();
	QVec    getInternalErrorInverse  ();


	QVec    getVisualPose      ();
	QVec    getInternalPose    ();
	QString getTip             ();

private:
	void        getErrors(QString visual, QString source, QVec &normal, QVec &inverse);

	QVec        visualPose;
	QVec        tagPose;
	timeval     *lastUpdate;
	InnerModel  *im;
	QString     tip;
	QVec        internalError, internalErrorINV;
	QVec        targetError, targetErrorINV;
	InnerModelTransform *nodeMarca, *nodeMarca2;
};


