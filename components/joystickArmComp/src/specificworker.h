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

// Get Base data command: Reductora, Radio, Separación entre ruedas
#define DEVICEDATA_GET_COMMAND_STRING "D0:"
#define DEVICEDATA_GET_COMMAND_CRC    "0*"
#define DEVICEDATA_DATA_SIZE   3+2+1+2+1+3+2
#define DEVICEDATA_GETGEAR_DATASIZE 2
#define DEVICEDATA_GETRADIO_DATASIZE 2
#define DEVICEDATA_GETDEVICELINE_DATASIZE 3

#include <string>
using namespace std;

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
		float Get_Valor_Motor(int ID, int valorPotenciometro);
		int translateToJose (QString n);
	
	private:
		void initialize();
		void getData();	
		void sendData( );
		
		bool ascii2dec ( char *buf, int ndigits, int &out );
		bool sendCommand(QString cmd, char *buf, int totalread);	
		void convertToJointMotor();
		QString translate(QString first);
		void checkError(char character );
		QSerialPort device;
		bool send;
		char resultado[70];
		RoboCompJointMotor::MotorGoalPositionList listGoals;
		
		//no entiendo porque atributos de clase
		unsigned char stringEnviado[200], stringRecibido[200];
		int numEnviado, numRecibido;
		bool READY;
		
		struct Limites
		{
			float Max;
			float Min;
		};

	public slots:
		void compute(); 	
		
};

#endif