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
	/**
	* \brief The VisualHand constructor must receive a pointer to an InnerModel object and the name of the arm's tip.
	* @param im_ the pointer to the main innermodel
	* @param tip_ the name of the end effector of the hand
	*/
	VisualHand(InnerModel *im_, QString tip_);
	~VisualHand();

	/**
	* \brief Updates the hand's possition according to an April tag and the time.
	* Also, it calculates the error between the visualHand and the internal hand
	* @param tag the pose of the aprilTag
	*/
	void setVisualPose             (RoboCompAprilTags::tag tag);
	/**
	* \brief Updates the pose of the visual tag with the internal error calculated before, between the visual tag and the internal tip.
	* ALERT HAY QUE PROBARLO
	*/ 
	void setVisualPosewithInternalError ();

	/**
	* \brief Returns the elapsed time in seconds between the last time that the visual pose was calculated and the current time.
	* @return double seconds
	*/
	double getSecondsElapsed         ();

	/** \brief updates the direct and inverse errors between the target and the visual pose. */ 
	void    updateTargetError      ();
	/** 
	 * \brief returns the direct error between the target and the visual pose
	 * @return QVec (6) direct error
	 */
	QVec    getTargetError         ();
	/** 
	 * \brief returns the inverse error between the target and the visual pose
	 * @return QVec (6) inverse error
	 */
	QVec    getTargetErrorInverse  ();

	/** \brief updates the direct and inverse errors between the internal tip and the visual pose. */ 	
	void    updateInternalError();
	/** 
	 * \brief returns the direct error between the internal tip and the visual pose
	 * @return QVec (6) direct error
	 */
	QVec    getInternalError         ();
	/** 
	 * \brief returns the inverse error between the internal tip and the visual pose
	 * @return QVec (6) inverse error
	 */
	QVec    getInternalErrorInverse  ();


	/** 
	 * \brief returns the visual pose
	 * @return QVec (6) visual pose
	 */
	QVec    getVisualPose      ();
	/** 
	 * \brief returns the internal pose of the tip
	 * @return QVec (6) internal pose
	 */
	QVec    getInternalPose    ();
	/**
	* \brief returns the name of the hand's tip.
	* @return QString tip
	*/ 
	QString getTip             ();

private:
	void        getErrors(QString visual, QString source, QVec &normal, QVec &inverse, bool debug=false);

	QVec        visualPose;
	QVec        tagPose;
	timeval     *lastUpdate;
	InnerModel  *im;
	QString     tip;
	QVec        internalError, internalErrorINV;
	QVec        targetError, targetErrorINV;
	InnerModelTransform *nodeMarca, *nodeMarca2;
};


