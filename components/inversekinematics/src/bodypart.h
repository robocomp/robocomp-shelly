/* ------------------------------------------------------------------
 *  CLASS BODY PART
 * IT REPRESETS ONE PART OF THE ROBOT. WE DIVIDE THE ROBOT IN THREE
 * DIFERENT PARTS:
 * -----> RIGHT ARM: IT'S COMPOSED BY ALL THE MOTORS OF THE RIGHT ARM
 * -----> LEFT ARM: IT'S COMPOSED BY ALL THE MOTORS OF THE LEFT ARM
 * -----> HEAD: IT'S COMPOSED BY ALL THE MOTORS OF THE ROBOT'S HEAD
 * 
 * ONE PART OF THE ROBOT HAS ALWAYS:
 * 		- THE NAME OF THE PART.
 * 		- THE LIST OF PART'S MOTORS.
 *		- THE NAME OF THE TIP.
 * 		- THE LIST OF TARGETS THAT MUST BE EXECUTED
 * ------------------------------------------------------------------*/ 
#ifndef BODYPART_H
#define BODYPART_H

#include <iostream>
#include <QString>
#include <QStringList>
#include "target.h"

using namespace std;

class BodyPart
{
public:
	BodyPart	(QString partName_, QString tipName_, QStringList	motorList_);
	BodyPart	();
	~BodyPart	();
	
	QString			getPartName		();
	QString			getTipName		();
	QStringList		getMotorList	();
	QQueue<Target>	&getTargetList	();
	void 			removeTarget	();
	
	void			addTargetToList	(Target &target);
	void 			reset			();
	
private:
	QString			partName;
	QString			tipName;
	QStringList		motorList;
	QQueue<Target>	targetList;
	//QQueue<Target>	solvedList;
	int				counter;
	
};

#endif // BODYPART_H
