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
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	dataMutex = new QMutex(QMutex::Recursive);

	wheelVels = QVec::vec4(0,0,0,0);
	x     = z     = angle     = 0;
	corrX = corrZ = corrAngle = 0;

	/// InnerModel
	innermodel = new InnerModel();
	// raw odometry nodes
	backPose = innermodel->newTransform("backPose", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
	innermodel->getRoot()->addChild(backPose);	
	newPose = innermodel->newTransform("newPose", "static", backPose, 0,0,0, 0,0,0, 0);
	backPose->addChild(newPose);
	// corrected odometry nodes
	corrBackPose = innermodel->newTransform("corrBackPose", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
	innermodel->getRoot()->addChild(corrBackPose);	
	corrNewPose = innermodel->newTransform("corrNewPose", "static", corrBackPose, 0,0,0, 0,0,0, 0);
	corrBackPose->addChild(corrNewPose);
	
	
	lastOdometryUpdate = QTime::currentTime();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(dataMutex);

	// YEP: OMNI-DIRECTIONAL ROBOTS: Abstract: All the robots introduced in chapter 7, with the exception of syncro-drive vehicles...
	// NOPE: http://cdn.intechopen.com/pdfs-wm/465.pdf
	R  = QString::fromStdString(params["UrsusBase.WheelRadius"].value).toFloat(); 
	l1 = QString::fromStdString(params["UrsusBase.DistAxes"].value   ).toFloat();
	l2 = QString::fromStdString(params["UrsusBase.AxesLength"].value ).toFloat();

	printf("l1: %f\n", l1);
	printf("l2: %f\n", l2);
	printf("r:  %f\n", R);

	// inverse kinematics matrix
	const float ill = 1. / (2.*(l1 + l2));
	M_wheels_2_vels = QMat(3, 4);
	M_wheels_2_vels(0,0) = +1./4.;
	M_wheels_2_vels(0,1) = +1./4.;
	M_wheels_2_vels(0,2) = +1./4.;
	M_wheels_2_vels(0,3) = +1./4.;
	M_wheels_2_vels(1,0) = +1./4.;
	M_wheels_2_vels(1,1) = -1./4.;
	M_wheels_2_vels(1,2) = -1./4.;
	M_wheels_2_vels(1,3) = +1./4.;
	M_wheels_2_vels(2,0) = +ill;
	M_wheels_2_vels(2,1) = -ill;
	M_wheels_2_vels(2,2) = +ill;
	M_wheels_2_vels(2,3) = -ill;
	M_wheels_2_vels = M_wheels_2_vels.operator*(R); // R instead of 2*pi*R because we use rads/s instead of rev/s

	// forward kinematics matrix
	const float ll = (l1 + l2)/2.;
	M_vels_2_wheels = QMat(4,3);
	M_vels_2_wheels(0,0) = +1.;
	M_vels_2_wheels(1,0) = +1.;
	M_vels_2_wheels(2,0) = +1.;
	M_vels_2_wheels(3,0) = +1.;
	M_vels_2_wheels(0,1) = +1.;
	M_vels_2_wheels(1,1) = -1.;
	M_vels_2_wheels(2,1) = -1.;
	M_vels_2_wheels(3,1) = +1.;
	M_vels_2_wheels(0,2) = +ll; // In contrast with the paper this code is based on, the
	M_vels_2_wheels(1,2) = -ll; // third column of the matrix is inverted because we use
	M_vels_2_wheels(2,2) = +ll; // the left-hand rule for angles.
	M_vels_2_wheels(3,2) = -ll;
	M_vels_2_wheels = M_vels_2_wheels.operator*(1./(R)); // 1/R instead of 1/(2*pi*R) because we use rads/s instead of rev/s
	M_vels_2_wheels.print("M_vels_2_wheels");
	
	timer.start(Period);
	return true;
}

void SpecificWorker::SpecificWorker::compute()
{
	computeOdometry(false);
}

void SpecificWorker::SpecificWorker::computeOdometry(bool forced)
{
	QMutexLocker locker(dataMutex);
	const float elapsedTime = 0.001*float(lastOdometryUpdate.elapsed());
	lastOdometryUpdate = QTime::currentTime();
	
	if (forced or lastOdometryUpdate.elapsed() > 0.1)
	{
		QVec newP;
		QVec deltaPos = (M_wheels_2_vels * wheelVels).operator*(elapsedTime);
		deltaPos.print("delta");

		// Raw odometry
		innermodel->updateTransformValues("newPose",     deltaPos(0), 0, deltaPos(1),       0,       deltaPos(2), 0);
		newP = innermodel->transform("root", "newPose");
		innermodel->updateTransformValues("backPose",        newP(0), 0,     newP(2),       0, deltaPos(2)+angle, 0);
		innermodel->updateTransformValues("newPose",               0, 0,           0,       0,                 0, 0);
		x = newP(0);
		z = newP(2);
		angle += deltaPos(2);

		// Corrected odometry
		innermodel->updateTransformValues("corrNewPose",    deltaPos(0), 0, deltaPos(1),    0,       deltaPos(2), 0);
		newP = innermodel->transform("root", "corrNewPose");
		innermodel->updateTransformValues("corrBackPose",       newP(0), 0,     newP(2),    0, deltaPos(2)+angle, 0);
		innermodel->updateTransformValues("corrNewPose",              0, 0,           0,    0,                 0, 0);
		corrX = newP(0);
		corrZ = newP(2);
		corrAngle += deltaPos(2);
	}
}


void SpecificWorker::getBaseState(::RoboCompOmniRobot::TBaseState &state)
{
	printf("--------------------\n");
	QMutexLocker locker(dataMutex);

	state.x = x;
	state.z = z;
	state.alpha = angle;
	printf("raw  (%f, %f @ %f)\n", state.x, state.z, state.alpha);

	state.correctedX = corrX;
	state.correctedZ = corrZ;
	state.correctedAlpha = corrAngle;
	printf("corr (%f, %f @ %f)\n", state.correctedX, state.correctedZ, state.correctedAlpha);
}

void SpecificWorker::getBasePose(::Ice::Int &x, ::Ice::Int &z, ::Ice::Float &alpha)
{
	x = x;
	z = z;
	alpha = angle;
}

void SpecificWorker::setSpeedBase(::Ice::Float advx, ::Ice::Float advz, ::Ice::Float rotv)
{
	computeOdometry(true);
	QMutexLocker locker(dataMutex);
// 	printf("Me llega: %f %f %f\n", advx, advz, rotv);
	const QVec v = QVec::vec3(advz, advx, rotv);
	const QVec wheels = M_vels_2_wheels * v;
// 	printf("Mandamos: %f %f %f %f\n", wheels(0), wheels(1), wheels(2), wheels(3));
	setWheels(wheels);
}

void SpecificWorker::stopBase()
{
	setWheels(QVec::vec4(0,0,0,0));
}

void SpecificWorker::resetOdometer()
{
	QMutexLocker locker(dataMutex);
	setOdometerPose(0,0,0);
	correctOdometer(0,0,0);
}

void SpecificWorker::setOdometer(const ::RoboCompOmniRobot::TBaseState &state)
{
	QMutexLocker locker(dataMutex);
	setOdometerPose(state.x,          state.z,          state.alpha);
	correctOdometer(state.correctedX, state.correctedZ, state.correctedAlpha);
}

void SpecificWorker::setOdometerPose(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
{
	QMutexLocker locker(dataMutex);
	this->x = x;
	this->z = z;
	this->angle = alpha;
}

void SpecificWorker::correctOdometer(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
{
	QMutexLocker locker(dataMutex);
	this->corrX = x;
	this->corrZ = z;
	this->corrAngle = alpha;
	
// 	("%f %f    @ %f\n", corrX, corrZ, corrAngle);
}

void SpecificWorker::setWheels(QVec wheelVels_)
{
	QMutexLocker locker(dataMutex);
	wheelVels = wheelVels_;
	static MotorGoalVelocity goalFL, goalFR, goalBL, goalBR;

	goalFL.maxAcc = goalFR.maxAcc = goalBL.maxAcc = goalBR.maxAcc = 0.1;

	goalFL.name = "frontLeft";
	goalFL.velocity = wheelVels(0);
	
	goalFR.name = "frontRight";
	goalFR.velocity = wheelVels(1);
	
	goalBL.name = "backLeft";
	goalBL.velocity = wheelVels(2);
	
	goalBR.name = "backRight";
	goalBR.velocity = wheelVels(3);

	try 
	{
		jointmotor_proxy->setVelocity(goalFL);
		jointmotor_proxy->setVelocity(goalFR);
		jointmotor_proxy->setVelocity(goalBL);
		jointmotor_proxy->setVelocity(goalBR);
	}
	catch(...)
	{
		printf("Error sending motor commands to JointMotor interface\n");
	}
}

