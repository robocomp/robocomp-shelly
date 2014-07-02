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


#include <iostream>
#include <ostream> 
#include <string>
#include <cstdio>
#include <stdio.h>
#include<arpa/inet.h> //inet_addr
#include<stdlib.h>    //strlen
#include<unistd.h>    //write
#include<pthread.h> //for threading , link with lpthread

#define BUFFSIZE 64000

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>

#include <QObject>


/**
       \brief
       @author authorname
*/

using namespace std;
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newFaceTabletUrsus(const valuesList& values);
	
	int port;	// the port I'm listening on
	int dataport;	// port I listen for datagrams on (can be same or different from port)
	int REVERSE;

	void Client();		// constructor
	void closesocket();		                // close the socket

	void send_string(char *str);		        // send a string to socket
	void send_ints(int *vals, int len);	        // send some integers
	void send_bytes(char *vals, int len);           // send some bytes
	void send_floats(float *vals, int len);		// send some floats
	void send_doubles(double *vals, int len);	// send some doubles
	void send_datagram(char *vals, int len);        // send a datagram
	int recv_string(char *str, int max, char term); // recv a string
	int recv_ints(int *vals, int max);  		// recv ints
	int recv_floats(float *vals, int max);  	// recv floats
	int recv_doubles(double *vals, int max);  	// recv doubles
	int recv_bytes(char *vals, int max);  		// recv bytes
	int recv_datagram(char *vals, int len);        // recv a datagram
	
	void send_Position(string T_rcy, string T_rcd, string T_gb, string T_ab, string T_tb, string T_eb, string T_poiX, string T_poiY, string T_podX, string T_podY);
	
	valuesList values2;


public slots:
 	void compute(); 	
	
protected:		
	int new_fd;	               // new connection on new_fd
	int datasockfd;	             
	int senddatasockfd;	              
    struct sockaddr_in their_addr; // connector's address information
    struct sockaddr_in my_addr;    // my address information
	struct hostent *he;
	double buffer[BUFFSIZE];	// reuse the same memory for buffer
	double buffer2[BUFFSIZE];

private:

	void recv_ack();
	void send_ack();

 
};

#endif