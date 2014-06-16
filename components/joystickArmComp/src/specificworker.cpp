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
	
	qDebug()<<"constructor";
	
	
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	/// Read interface type from parameters	
	RoboCompCommonBehavior::Parameter par = params.at("Joy.Device");	 
	qDebug()<<"device value"<<QString::fromStdString(par.value);
		
	device.setName (QString::fromStdString(par.value) );
	device.setBaudRate ( QSerialPort::BAUD9600);		
	par =params.at("Joy.MaxAdvance");
	qDebug()<<"Joy maxadvance value"<<QString::fromStdString(par.value);
	
	
	setPeriod(1);		
	initialize();
	timer.start(Period);	
	return true;
}

void SpecificWorker::initialize()
{
	qDebug()<<"init Truncate"<<device.open ( QIODevice::Truncate );
	qDebug()<<"cleaning and close";
	device.close();
	
	qDebug()<<"init ReadOnly"<<device.open ( QIODevice::ReadOnly  );;
	if ( device.open ( QIODevice::ReadOnly ) == false )
	{
		qDebug()<<"Failed to open: "+device.name()<<"check your config";
		qFatal("exit");
	}	
	
}
void SpecificWorker::sendData()
{
	send=false;
	convertToJointMotor();	
	try
	{	
// 			qDebug()<<"motor:"<<i<<QString::fromStdString(listGoals[i].name);
		jointmotor_proxy->setSyncPosition(listGoals);			
		
	}
	catch ( Ice::Exception e )
	{
		qDebug()<<"Exception: error talking to JointMotor_proxy";//<<QString::fromStdString(listGoals[i].name)<<e.what();
	}
	
}
void SpecificWorker::convertToJointMotor()
{
	QString s (resultado);
	
	QStringList l = s.split("* ");
	
// 	qDebug()<< l.size();
	l.removeLast();
//	qDebug()<<"l"<< l;
	listGoals.clear();
	int maximun = l.size()-2;
	qDebug()<<maximun;
	for (int i = 0; i < maximun; ++i) {
		QString n =l.at(i);
		QString name,position;
		
		n=n.remove("*");
//		qDebug()<<"motor segun jose"<<n;
		QStringList motor =n.split(":");

		if (! motor.isEmpty() )
		{
			name=translate(motor.first());
			position =motor.last();
// 			qDebug()<<"position.toInt()"<<position.toInt();
			RoboCompJointMotor::MotorGoalPosition mg;
			mg.name = name.toStdString(); mg.maxSpeed = 1.0; 
			
			int id=translateToJose(motor.first());
			mg.position = Get_Valor_Motor(id, position.toInt());
			
			
///			cout<<mg.name<<" "<<mg.position<<" "<<mg.maxSpeed<<"\n";
			qDebug()<<"--";
			listGoals.push_back( mg );			
		}
	
	}
//	qDebug()<<"---- listGoals.size()"<< listGoals.size()<<"-----" ;
	
	
}
int SpecificWorker::translateToJose(QString n)
{	
	int id=-1;
	
	if (n=="S00")
		id=0;
	if (n=="S01")
		id=1;
	if (n=="S02")
		id=2;
	if (n=="S03")
		id=3;
	if (n=="S04")
		id=4;
	if (n=="S05")
		id=5;
	if (n=="S06")
		id=6;
	return id;
}


float SpecificWorker::Get_Valor_Motor(int ID, int valorPotenciometro){
	
	Limites Limites_Motores[10];
    Limites Limites_Potenciometros[10];
    
	///arm left, limites motores, min y max, representa la posicion del motor cuando el potenciometro esta a su valor Min o su valor Max respectivamente
    Limites_Motores[0].Min = 0;
    Limites_Motores[0].Max = -3.14;
    Limites_Potenciometros[0].Min = 120;
    Limites_Potenciometros[0].Max = 800;
	
    Limites_Motores[1].Min = -3.14;
    Limites_Motores[1].Max = 0.5;	
	Limites_Potenciometros[1].Min = 150;
    Limites_Potenciometros[1].Max = 990;
	
    
    Limites_Motores[2].Min = 1.57;
    Limites_Motores[2].Max =  -1.57;
	
	Limites_Potenciometros[2].Min = 200;
    Limites_Potenciometros[2].Max = 950;
	
    
	///3 munieca
    Limites_Motores[3].Min = 1.57;
    Limites_Motores[3].Max = -1.57;
	Limites_Potenciometros[3].Min = 130;
    Limites_Potenciometros[3].Max = 870;
    
	///4 es el codo
    Limites_Motores[4].Min = 0;
    Limites_Motores[4].Max = 1.57;
	Limites_Potenciometros[4].Min = 500;
    Limites_Potenciometros[4].Max = 800;
	
        
	
	///arm right
//     Limites_Motores[5].Min = 0;
//     Limites_Motores[5].Max = 125952;
		
	Limites_Potenciometros[5].Min = 107;
    Limites_Potenciometros[5].Max = 848;
	Limites_Potenciometros[6].Min = 107;
    Limites_Potenciometros[6].Max = 848;
	
  float  Valor;
  float Salida=0.0;
  if (valorPotenciometro >= 0 && valorPotenciometro<=1023)
  { // comprobamos que esté dentro del rango 0 -> 1023    
    // Asignamos el valor del potenciometro
    Valor = valorPotenciometro;
    // comprobamos los limites de los máximos recorridos del potenciometro asignando los max y min si se superan dichos valores.
    if (valorPotenciometro< Limites_Potenciometros[ID].Min)  Valor = Limites_Potenciometros[ID].Min;
    if (valorPotenciometro > Limites_Potenciometros[ID].Max)  Valor = Limites_Potenciometros[ID].Max;
      
    // Restamos el valor del mínimo para tener un rango de 0 -> max-min	
    //Valor -=Limites_Potenciometros[ID].Min;
  
    //Salida = (Valor * (Limites_Motores[ID].Max-Limites_Motores[ID].Min))/(Limites_Potenciometros[ID].Max-Limites_Potenciometros[ID].Min); 
	Salida=  ( (Limites_Motores[ID].Max - Limites_Motores[ID].Min) /  (Limites_Potenciometros[ID].Max - Limites_Potenciometros[ID].Min) ) * (Valor - Limites_Potenciometros[ID].Min) + Limites_Motores[ID].Min;
   
// 	if (ID==2 || ID==3 || ID==7 || ID==8) // Si se trata de uno de los motores del codo o de la muñeca (giro -90º 0º +90º )
// 		Salida = ((Valor * Limites_Motores[ID].Max * 2)/(Limites_Potenciometros[ID].Max-Limites_Potenciometros[ID].Min))-Limites_Motores[ID].Max;  // es una regla de tres
// 	else
// 		Salida = (Valor * (Limites_Motores[ID].Max-Limites_Motores[ID].Min))/(Limites_Potenciometros[ID].Max-Limites_Potenciometros[ID].Min);  // es una regla de tres
// 
// 		
// 	if (ID==5 ) // invertimos el sentido de giro del motor 5 OK
// 		Salida = (Salida * (-1))+Limites_Motores[ID].Max;
// 	
// 	if (ID==6 ) // invertimos el sentido de giro del motor 6 OK
// 		Salida = ((Salida * (-1))+Limites_Motores[ID].Max);
// 	
// 	if (ID==9 ) // invertimos el sentido de giro del motor
// 		Salida = (Salida * (-1))+Limites_Motores[ID].Max;
  }
 
 return (Salida);
} 

QString SpecificWorker::translate(QString first)
{
	QString id ("SIN HACER");
	if (first=="S00")		
		id = "rightShoulder1";
	if (first=="S01")
		id= "rightShoulder2";
	if (first=="S02")
		id= "rightShoulder3";
	if (first=="S03")
		id= "rightForeArm";
	if (first=="S04")
		id= "rightElbow";
	return id;
}

void SpecificWorker::checkError(char character)
{

	QVector <char> validCharacters;	
	validCharacters.append('S');
	validCharacters.append(':');
	validCharacters.append('*');
	validCharacters.append(' ');
	for (int i=0; i<=9;i++)		
		validCharacters.append((char)(((int)'0')+i));
	
	
	if (validCharacters.contains(character) == false)
	{
		qDebug()<<"validCharacters"<<validCharacters;	
		qDebug()<<"character read:"<< character;
		qDebug()<<"Character invalid reading from the USB. Check your device. Actual device"<<device.name();
		qFatal("qFatal: Exit");	
	}
}


// void SpecificWorker::getData()
// {
// 	send=false;
// 	if (device.isOpen() )
// 	{
// 		char buf[59];		
// 		for (int i=0;i<59;i++)
// 			resultado[i]=' ';
// 		///esto no esta demasiao bien hecho
// 		numRecibido = device.read(buf, 1);
// //		qDebug()<<"buf"<<buf[0];
// 		checkError (buf[0] );
// 		if (buf[0] =='S' )
// 		{
// 			resultado[0]=buf[0];
// 			device.read(buf, 1);
// 			if (buf[0] =='0' )
// 			{
// 				resultado[1]=buf[0];
// 				device.read(buf,1);
// 				if (buf[0] =='0' )
// 				{
// 					resultado[2]=buf[0];
// 					numRecibido = device.read(buf, 56);
// 					if (numRecibido == 56)
// 					{
// 						for (int j=0;j<numRecibido;j++)
// 						{
// 							resultado[j+3]=buf[j];
// 						}
// 						send = true;
// 						
// 					}
// 				}
// 			}
// 		} 
// 	}
// 	else 
// 	{
// 		qDebug()<<device.name();
// 		qFatal("device is not open");		
// 	}
// 
// }

void SpecificWorker::getData()
{
	send=false;
	if (device.isOpen() )
	{
		char buf[140];	
		///solo hay 5 motores 10 caracteres por campo
		for (int i=0;i<70;i++)
			resultado[i]=' ';
		///esto no esta demasiao bien hecho
		numRecibido = device.read(buf, 140);
		qDebug()<<"buf"<<buf<<numRecibido;
		///este metodo me cambia el check error
//		checkError (buf[0] );
		
		for (int i=0;i<70;i++)
		{
			if (buf[i] =='S' && buf[i+1] =='0' && buf[i+2] =='0')
			{
				for (int j=i; j<i+69;j++)
				{
					resultado[j-i]=buf[j];
					
				}
				
				qDebug()<<"resultado:"<<resultado<<"--";
				
				send = true;
				
				return;
			}
				
		}
// 		if (buf[i] =='S' )
// 		{
// 			resultado[0]=buf[0];
// 			device.read(buf, 1);
// 			if (buf[0] =='0' )
// 			{
// 				resultado[1]=buf[0];
// 				device.read(buf,1);
// 				if (buf[0] =='0' )
// 				{
// 					resultado[2]=buf[0];
// 					numRecibido = device.read(buf, 56);
// 					if (numRecibido == 56)
// 					{
// 						for (int j=0;j<numRecibido;j++)
// 						{
// 							resultado[j+3]=buf[j];
// 						}
// 						send = true;
// 						
// 					}
// 				}
// 			}
// 		} 
	}
	else 
	{
		qDebug()<<device.name();
		qFatal("device is not open");		
	}

}




/**
* \brief Send command to the device
* @param cmd Command to send
* @return true if command was sent successfully, else return false
*/
bool SpecificWorker::sendCommand(QString cmd, char *buf, int totalread){
	char *command;
	if(device.isOpen()){
		cmd += 'X';
		cmd += 'X';
		command = cmd.toLatin1().data();
	
		for (int o = 0; o < cmd.size(); ++o) stringEnviado[o] = (uchar)(cmd[o].toAscii());// 	stringEnviado = cmd;
	
		// Send command
		device.write(command,cmd.size());
		numRecibido = device.read(buf, totalread);
		
		
		for (int i = 0; i < numRecibido; ++i) 
			stringRecibido[i] = (unsigned char)buf[i];
	
		if ( numRecibido != totalread ){
		       
			printf("[10] baseComp: Ack error: <comand(%d) ", cmd.size());
			return false;
		}
		else{
			for (int o=0; o<numRecibido; ++o) stringRecibido[o] = buf[o]; // stringRecibido = buf; /// XXX
			return true;
		}
	}
	else{
		rError("Fatal error, motion deviced closed");
		return false;
	}
}
//Utilities
/**
* \brief Utility function. Transform ascii to decimal
* @param buf Ascci string
* @param ndigits Number of digits
* @param out Returned integer
* @return true if transform done successfully
*/
bool SpecificWorker::ascii2dec ( char *buf, int ndigits, int &out ){
	int i;
	out = 0;
	for ( i=0; i<ndigits; i++ )
	{
		if ( buf[i] < 48 || buf[i] > 57 ) return false;
		out += ( buf[i] - 48 ) * ( int ) pow ( 10,ndigits-1-i );
	}
	return true;
}


void SpecificWorker::compute( )
{
 	QMutexLocker l (mutex);	
//	qDebug()<<"compute send"<<send;
// 	qDebug()<<"device"<<device.isReadable();
 	if (!send)
		getData();
 	else
 		sendData();
	
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

///copy to base
// void SpecificWorker::getData()
// {
// 	char buf[128];
// 	int i=0, tmp0=0,tmp1=0,tmp2=0;
// 	bool success=false;
// 
// 	QString cmd = QString ( DEVICEDATA_GET_COMMAND_STRING DEVICEDATA_GET_COMMAND_CRC );
// 	if ( sendCommand ( cmd, buf, DEVICEDATA_DATA_SIZE ) ){
// 		// Read the received data
// 		if ( buf[i] == 'D' && buf[++i] == '0' && buf[++i] == ':' ){
// 			i++;
// 			ascii2dec ( buf+i,DEVICEDATA_GETGEAR_DATASIZE,tmp0);
// 			i += DEVICEDATA_GETGEAR_DATASIZE +1;  // : added
// 			ascii2dec ( buf+i,DEVICEDATA_GETRADIO_DATASIZE,tmp1 );
// 			i += DEVICEDATA_GETRADIO_DATASIZE +1;  // : added
// 			ascii2dec ( buf+i,DEVICEDATA_GETDEVICELINE_DATASIZE,tmp2 );
// 		}
// 	}
// 	qDebug()<<"tmp (0 1 2):"<<tmp0<<tmp1<<tmp2;;
// }
