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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <ui_mainUI.h>

#include <CommonBehavior.h>
#include <InverseKinematics.h>
#include <JointMotor.h>
#include <AprilTags.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompInverseKinematics;
using namespace RoboCompJointMotor;
using namespace RoboCompAprilTags;




class GenericWorker : 
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	

	InverseKinematicsPrx inversekinematics_proxy;
	JointMotorPrx jointmotor_proxy;

	virtual void setFingers(const float d) = 0;
	virtual TargetState getTargetState(const string &bodyPart, const int targetID) = 0;
	virtual int setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist) = 0;
	virtual void goHome(const string &bodyPart) = 0;
	virtual void stop(const string &bodyPart) = 0;
	virtual int setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights) = 0;
	virtual bool getPartState(const string &bodyPart) = 0;
	virtual void setJoint(const string &joint, const float angle, const float maxSpeed) = 0;
	virtual int setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax) = 0;
	virtual void newAprilTag(const tagsList &tags) = 0;


protected:
	QTimer timer;
	int Period;

public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif