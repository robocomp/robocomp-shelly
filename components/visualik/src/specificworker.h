/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	TargetState getTargetState(const string &bodyPart, const int targetID);
	int setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist);
	void goHome(const string &bodyPart);
	void stop(const string &bodyPart);
	int setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights);
	bool getPartState(const string &bodyPart);
	int setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax);
	void newAprilTag(const tagsList &tags);

public slots:
	void compute(); 	

private:
	
};

#endif

