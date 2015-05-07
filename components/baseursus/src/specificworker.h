/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	void computeOdometry(bool forced=false);


private:
	void setWheels(QVec wheelVels_);
	float R, l1, l2;
	QMat M_wheels_2_vels;
	QMat M_vels_2_wheels;
	
	// Odometry control
	QMutex *dataMutex;
	QVec wheelVels;
	float angle, x, z;
	float corrAngle, corrX, corrZ;
	InnerModel *innermodel;
	InnerModelTransform *backPose, *newPose;
	InnerModelTransform *corrBackPose, *corrNewPose;
	QTime lastOdometryUpdate;
	
	void getBaseState(::RoboCompOmniRobot::TBaseState &state);
	void getBasePose(::Ice::Int &x, ::Ice::Int &z, ::Ice::Float &alpha);
	void setSpeedBase(::Ice::Float advx, ::Ice::Float advz, ::Ice::Float rotv);
	void stopBase();
	void resetOdometer();
	void setOdometer(const ::RoboCompOmniRobot::TBaseState &state);
	void setOdometerPose(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha);
	void correctOdometer(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha);
};

#endif
