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
#include <innermodel/innermodel.h>
#include "../../inversekinematicsComp/src/target.h"
#include <qt4/Qt/qcheckbox.h>
#include <qt4/Qt/qwidget.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qspinbox.h>
#include <qt4/QtGui/QFrame>
#include <time.h>
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	
	/// CONSTRUCTORES Y DESTRUCTORES ///
					SpecificWorker		(MapPrx& mprx);	
					~SpecificWorker		();
	
	/// MÉTODOS PÚBLICOS ///
	bool 			setParams			(RoboCompCommonBehavior::ParameterList params);
	void  			newAprilTag			(const tagsList& tags);


public slots:
	/// SLOTS ///
 	void 			compute				();
	
private:
	/// ATRIBUTOS PRIVADOS ///
	InnerModel 		*innerModel;								// Puntero para trabajar con el innerModel (pintar el target y obtener valores angulares)
	
	/// MÉTODOS PRIVADOS ///
};

#endif