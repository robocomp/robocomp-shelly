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

//**********************************************************************************************************//
//				CONSTRUCTORES Y DESTRUCTORES																//
//**********************************************************************************************************//
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

//**********************************************************************************************************//
//				MÉTODOS DE INICIALIZACIÓN Y CONEXIÓN?														//
//**********************************************************************************************************//
/**
 * @brief Method SET PARAMS. 
 * It's called for the MONITOR thread, which initialize the component with the 
 * parameters of the correspondig config file.
 * 
 * @return bool
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Guardamos el innerModel que se nos pasa como parámetro de inicialización.
	// ¡CUIDADO CON EL INNERMODEL! Debe ser el mismo que el que utiliza LOKIARM!!!
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel") ;
// 		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel = new InnerModel(par.value);
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
// 		}
// 		else
// 		{	qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
// 			qFatal("Exiting now.");
// 		}
	}
	catch(std::exception e)	{ qFatal("Error reading config params"); }

	timer.start(Period);
	return true;
}

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	// Definimos una variable de tiempo para hacer este método periódico...
	static QTime reloj = QTime::currentTime();
	
	if( reloj.elapsed() > 1000)
	{	
	}
}

//**********************************************************************************************************//
//			SLOTS PÚBLICOS																					//
//**********************************************************************************************************//
void SpecificWorker::compute( )
{
}

