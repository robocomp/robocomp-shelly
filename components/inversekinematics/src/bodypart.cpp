#include "bodypart.h"
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Parametrized Constructor
 * @param partName_ name of the body's part
 * @param tipName_  name of the part's tip (end effector)
 */ 
BodyPart::BodyPart(QString partName_, QString tipName_,  QStringList	motorList_)
{
	counter 	= 0;
	partName 	= partName_;
	tipName  	= tipName_;
	motorList 	= motorList_;
}
/**
 * \brief default constructor
 */ 
BodyPart::BodyPart()
{

}
/**
 * \brief Default Destructor
 */ 
BodyPart::~BodyPart()
{
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this method returns the name of the robot's part
 * @return QString partName
 */ 
QString BodyPart::getPartName()
{
	return partName;
}
/**
 * \brief this method returns the name of the part's tip.
 * @return QString tipName
 */ 
QString BodyPart::getTipName()
{
	return tipName;
}
/**
 * \brief this method returns a list with all the motor's names that composes the robot's part.
 *@return QStringList motorList 
 */ 
QStringList BodyPart::getMotorList()
{
	return motorList;
}
/**
 * \brief this method returns the queue of the targets
 * @return QQueue targetList
 */ 
QQueue<Target> &BodyPart::getTargetList()
{
	return targetList;
}
/**
 * \brief this method returns the queue of the solved targets
 * @return QQueue solvedList
 */ 
QQueue<Target> &BodyPart::getSolvedList()
{
	return solvedList;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method insert the new target into the list (queue) of targets.
 * @param target the new target.
 */ 
void BodyPart::addTargetToList(Target &target)
{
	target.setTargetIdentifier(counter);
	targetList.enqueue(target);
	counter++;
}
/**
 * \brief This method insert the solved target into the solved list (queue) of solved targets,
 * and delete it from the targetList.
 * @param target the new target.
 */ 
void BodyPart::addSolvedToList()
{
	qDebug()<<"Target resuelto";
	solvedList.enqueue(targetList.dequeue());
}

