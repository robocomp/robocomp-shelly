/* ------------------------------------------------------------------
 *  CLASS METRIC
 * THIS CLASS IS IN CHARGE OF DOING THE CONVERSION FROM MILIMETERS
 * TO METERS. IN FACT, WE CAN DELETE IT, AND PUT HIS METHOD IN THE
 * SPECIFIC WORKER, BUT THIS METHOD HAVE A LOT OF CODE LINES, AND IT'S
 * REALLY UGLY.... AND HARD TO UNDERSTAND.
 * ------------------------------------------------------------------*/ 
#ifndef METRIC_H
#define METRIC_H

#include <iostream>
#include <innermodel/innermodel.h>
#include <osgDB/ReadFile>
//#include "sampler.h"

using namespace std;

class Metric
{
public:
	Metric();
	~Metric();

	static void moveInnerModelFromMillimetersToMeters(InnerModelNode* node);
};
#endif