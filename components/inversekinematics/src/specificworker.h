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

#include <iostream>
#include <fstream>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

#include "metric.h"
#include "target.h"
#include "bodypart.h"
#include "inversedkinematic.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT

private:
	struct stTargetsSolved
	{
		QString      part;
		Target       target;
		TargetState  state;
	};
	stTargetsSolved TargetsSolved;
	
	int                       correlativeID;
	bool                      INITIALIZE_READY;
	bool                      UPDATE_READY;
	QMap<QString, BodyPart>   bodyParts;
	QQueue<stTargetsSolved>   targetsSolved;
	QMutex                    *mutexSolved;
	QStringList               availableParts;
	InnerModel                *innermodel;
	InversedKinematic         *inversedkinematic;
	ofstream file;            // EL FICHERO DONDE GUARDAR DATOS

	
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
#endif
	
public:
	     SpecificWorker(MapPrx& mprx);	
	    ~SpecificWorker();
	bool setParams     (RoboCompCommonBehavior::ParameterList params);	
	
	void        stop                 (const string &bodyPart);
	int         setTargetPose6D      (const string &bodyPart, const Pose6D &target, const WeightVector &weights);
	int         setTargetAdvanceAxis (const string &bodyPart, const Axis &ax, const float dist);
	int         setTargetAlignaxis   (const string &bodyPart, const Pose6D &target, const Axis &ax);
	bool        getPartState         (const string &bodyPart);
	TargetState getTargetState       (const string &bodyPart, const int targetID);
	void        goHome               (const string &bodyPart);
	void        setJoint             (const string &joint, const float angle, const float maxSpeed);
	void        setFingers           (const float d);
	void        sendData             (const TData &data);
	
public slots:
	void compute(); 	

private:
	void updateInnerModel       ();
	void updateAngles           (QVec newAngles, BodyPart part);
	void createInnerModelTarget (Target &target);
	void removeInnerModelTarget (Target &target);
	void showInformation        (BodyPart part, Target target);
	void addTargetSolved        (QString part, Target t);
	//TODO QUITAR DESPUES
	//void updateMotors           (BodyPart bp, QVec angles);
};

#endif

