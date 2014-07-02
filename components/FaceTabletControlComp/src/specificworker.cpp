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

 
  #define socklet_t int
 #define BACKLOG 10      // how many pending connections queue will hold 
#define VERBOSE 1      // turn on or off debugging output

#define SIZE 10		  /* how many items per packet */
#define NUM_PACKS 3	  /* number of times we'll do it */
/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
  
  
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
   

}
void SpecificWorker::compute( )
{
	 
	for (uint i=0; i<values2.size(); i++)
	{
	 
	port = 9999;
	dataport = 33012;
	int rev = 1;

        if (VERBOSE) printf("Server: opening socket to %s on port = %d, datagrams on port = %d\n", "158.49.247.247", port, dataport);

	if ((he=gethostbyname("158.49.247.247"))==NULL) 
		perror("gethostbyname");

        if ((new_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
            perror("socket");

	if ((datasockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            perror("datagram socket");
            exit(1);
        }

        if ((senddatasockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            perror("datagram send socket");
            exit(1);
        }
	
        my_addr.sin_family = AF_INET;        
        my_addr.sin_port = htons(dataport); 
        my_addr.sin_addr.s_addr = INADDR_ANY; 
        bzero(&(my_addr.sin_zero), 8);        

        their_addr.sin_family = AF_INET;        
        their_addr.sin_port = htons(port);      
        their_addr.sin_addr = *((struct in_addr *) he->h_addr); 
        bzero(&(their_addr.sin_zero), 8);        

        if (::connect(new_fd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
            perror("connect new_fd");
            exit(1);
        }

        if (bind(datasockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
            perror("bind datagram socket");
	    exit(1);
        }
	
	string Srcy = static_cast<ostringstream*>( &(ostringstream() << values2[0].rcy) )->str();
	string Srcd = static_cast<ostringstream*>( &(ostringstream() << values2[0].rcd) )->str();
	string Sgb  = static_cast<ostringstream*>( &(ostringstream() << values2[0].gb) )->str();
	string Sab  = static_cast<ostringstream*>( &(ostringstream() << values2[0].ab) )->str();
	string Stb  = static_cast<ostringstream*>( &(ostringstream() << values2[0].tb) )->str();
	string Seb  = static_cast<ostringstream*>( &(ostringstream() << values2[0].eb) )->str();
	string SpoiX = static_cast<ostringstream*>( &(ostringstream() << values2[0].poiX) )->str();
	string SpoiY = static_cast<ostringstream*>( &(ostringstream() << values2[0].poiY) )->str();
	string SpodX = static_cast<ostringstream*>( &(ostringstream() << values2[0].podX) )->str();
	string SpodY = static_cast<ostringstream*>( &(ostringstream() << values2[0].podY) )->str();
	

	  cout<<"id emotional state"<<values2[0].idES<<endl;
	  cout<<"flag"<<values2[0].flag<<endl;
	  
	  // 1 anger - 2 fear- 3 sad - 4 Happy - 5 Neutral - 6 Confused
	  
	  if (values2[0].idES == 1)
	  {
	    cout<<"Anger state"<<endl;
	    send_Position("20","-20","0","10","90","-1","0","0","0","0");
	    
	  } else if (values2[0].idES == 2)
	  {
	    cout<<"Fear state"<<endl;

	    send_Position("-10","10","0","60","90","0","5","-10","5","-10");
	  } else   if (values2[0].idES == 3)
	  {
	    cout<<"Sad state"<<endl;
	    send_Position("-20","20","0","40","90","-1","0","40","0","40");
	    
	  } else  if (values2[0].idES == 4)
	  {
	    cout<<"Happy state"<<endl;
	    send_Position("0","0","0","60","99","1","0","10","0","10");
	    
	  }   else  if (values2[0].idES == 5)
	  {
	    cout<<"Neutral state"<<endl;
	    send_Position("0","0","0","10","80","0","5","10","5","3");
	  } else  if (values2[0].idES == 6)
	  {
	    cout<<"Confused state"<<endl;

	    send_Position("-10","-10","20","40","50","0","-30","35","30","-10");
	  } else {
	    cout<<"control manual"<<endl;
	    send_Position(Srcy, Srcd, Sgb, Sab, Stb, Seb, SpoiX, SpodY, SpodX, SpodY);
	  }
	  values2.clear();

 	shutdown(new_fd, SHUT_RDWR);

	close(new_fd);
	//closesocket();
	
 	close(datasockfd);
 	close(senddatasockfd);
	}

}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};


void SpecificWorker::send_Position(string T_rcy, string T_rcd, string T_gb, string T_ab, string T_tb, string T_eb, string T_poiX, string T_poiY, string T_podX, string T_podY)
{

    std::string reply= "{\"cara\": [{\"gradoCejaIzq\": \"" + T_rcy + "\",\"gradoCejaDer\": \"" + T_rcd + "\",\"gradoBoca\": \"" + T_gb + "\",\"aperturaBoca\": \"" + T_ab + "\",\"tamBoca\": \"" + T_tb + "\",\"estadoBoca\": \"" + T_eb + "\",\"posOjoIzqX\": \"" + T_poiX + "\",\"posOjoIzqY\": \"" + T_poiY + "\",\"posOjoDerX\": \"" + T_podX + "\",\"posOjoDerY\": \"" + T_podY + "\"}]}";  
    char *cstr = new char[reply.length() + 1];
    strcpy(cstr, reply.c_str());
	
    send_string(cstr);
}


void SpecificWorker::newFaceTabletUrsus(const valuesList& values)
{
 values2 = values;
}

// send a string to the socket
void  SpecificWorker::send_string(char *str)
{
        if (send(new_fd, (char *) str, strlen(str), 0) == -1)
            perror("send");              

	if (VERBOSE) printf("Server: sending string '%s'\n", str);                       

//	recv_ack();
	send_ack();
	
	close(new_fd);
};

// send some integers over the wire
void  SpecificWorker::send_ints(int *val, int len)
{
	int *buff;
	char *ptr, *valptr;
	int i,j;

	// we may need to reverse the byte order, oh joy

	if (REVERSE)
	{
		// set the buff pointer to our previously allocated spot
		buff = (int *) buffer;

		ptr = (char *) buff;
		valptr = (char *) val;

		// we need to reverse the order of each set of bytes
		for (i = 0; i < len; i++)
		{
			for (j=0; j<sizeof(int); j++)
				ptr[i*sizeof(int)+j] = (char)
					valptr[(i+1)*sizeof(int)-j-1];
		}
        	if (send(new_fd, (char *) buff, sizeof(int)*len, 0) == -1)
            		perror("send ints");              		
	}
	else
        	if (send(new_fd, (char *) val, sizeof(int)*len, 0) == -1)
            		perror("send ints");              

	if (VERBOSE)
	{ 
		printf("Server: sending %d ints - ", len);                       
		for (i=0; i<len; i++)
			printf("%d ", val[i]);
		printf("\n");
	}

	// don't continue until we receive an ack from client

	recv_ack();
	send_ack();
};

// send some floats over the wire
void  SpecificWorker::send_floats(float *val, int len)
{
	float *buff;
	char *ptr, *valptr;
	int i,j;

	// we may need to reverse the byte order, oh joy

	if (REVERSE)
	{
		buff = (float *) buffer;

		ptr = (char *) buff;
		valptr = (char *) val;

		// we need to reverse the order of each set of bytes
		for (i = 0; i < len; i++)
		{
			for (j=0; j<sizeof(float); j++)
				ptr[i*sizeof(float)+j] = (char)
					valptr[(i+1)*sizeof(float)-j-1];
		}
        	if (send(new_fd, (char *) buff, sizeof(float)*len, 0) == -1)
            		perror("send floats");              		
	}
	else
        	if (send(new_fd, (char *) val, sizeof(float)*len, 0) == -1)
            		perror("send floats");              

	if (VERBOSE)
	{ 
		printf("Server: sending %d floats - ", len);                       
		for (i=0; i<len; i++)
			printf("%0.3f ", val[i]);
		printf("\n");
	}

	recv_ack();
	send_ack();
};

// send some doubles over the wire
void  SpecificWorker::send_doubles(double *val, int len)
{
	double *buff;
	char *ptr, *valptr;
	int i,j;

	// we may need to reverse the byte order, oh joy

	if (REVERSE)
	{
		// allocate a temporary array to hold the reversed bytes
		buff = (double *) buffer;

		ptr = (char *) buff;
		valptr = (char *) val;

		// we need to reverse the order of each set of bytes
		for (i = 0; i < len; i++)
		{
			for (j=0; j<sizeof(double); j++)
				ptr[i*sizeof(double)+j] = (char)
					valptr[(i+1)*sizeof(double)-j-1];
		}
        	if (send(new_fd, (char *) buff, sizeof(double)*len, 0) == -1)
            		perror("send doubles");              		
	}
	else
        	if (send(new_fd, (char *) val, sizeof(double)*len, 0) == -1)
            		perror("send doubles");              

	if (VERBOSE)
	{ 
		printf("Server: sending %d doubles - ", len);                       
		for (i=0; i<len; i++)
			printf("%0.3f ", val[i]);
		printf("\n");
	}

	recv_ack();
	send_ack();
};

// send some bytes over the wire
void  SpecificWorker::send_bytes(char *val, int len)
{
        int i;
        
        if (send(new_fd, (char *) val, len, 0) == -1)
                        perror("send bytes");
        
        if (VERBOSE)
        {
                printf("Server: sending %d bytes - ", len);

                for (i=0; i<len; i++)
                        printf("%d ", val[i]);
                printf("\n");
        }
        
        // don't continue until we receive an ack from client
        
        recv_ack(); 
        send_ack();
};

// send a packet of bytes using a datagram
void  SpecificWorker::send_datagram(char *val, int len)
{
        int i, numbytes;

        if ((numbytes=sendto(senddatasockfd, val, len, 0,
                (struct sockaddr *) &their_addr, sizeof(struct sockaddr)))==-1)
                        perror("send datagram bytes");

        if (VERBOSE)
        {
                printf("Server: sending datagram of %d bytes - ", len);
                for (i=0; i<len; i++)
                        printf("%d ", val[i]);
                printf("\n");
        }
};

// receive a string, returns num of bytes received
int  SpecificWorker::recv_string(char *str, int maxlen, char term)
{
	int numbytes = 0;
	int end = 0;
	int i, j;
	char *temp;

	// set the temp buffer to our already allocated spot
	temp = (char *) buffer;

	j = 0;

	// this is annoying, but the java end is sending a char
	// at a time, so we recv some chars (probably 1), append
	// it to our str string, then carry on until we see
	// the terminal character
	while (!end)
	{
		if ((numbytes=recv(new_fd, temp, BUFFSIZE, 0))==-1)
			perror("recv");
		for (i=0; i<numbytes; i++)
		{
			str[j] = temp[i];	
			j++;
		}
		if ((temp[i-1]==term) || (j==maxlen-1))
			end = 1;
	}


	str[j] = '\0';

	if (VERBOSE) printf("Server: received '%s'\n", str);                       	

	send_ack();
	recv_ack();
	
	return numbytes;
};


// receive some ints, returns num of ints received
int  SpecificWorker::recv_ints(int *val, int maxlen)
{
	int numbytes = 0;
	int i, j;
	char *temp;
        char *result;
	int end = 0;
	int total_bytes = 0;

	temp = (char *) buffer;
	result = (char *) buffer2;

	j = 0;

	// we receiving the incoming ints one byte at a time
	// oh cross language sockets are so much fun...

	while (!end)
	{
		if ((numbytes=recv(new_fd, temp, BUFFSIZE, 0))==-1)
			perror("recv");
		for (i=0; i<numbytes; i++)
		{
			result[j] = temp[i];	
			j++;
		}

		total_bytes = total_bytes + numbytes;
		if (total_bytes==maxlen*sizeof(int))
			end = 1;
	}

	// now we need to put the array of bytes into the array of ints

	char *ptr;
	int num = j/sizeof(int);

	ptr = (char *) val;

	// the significance order depends on the platform

	if (REVERSE)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < num; i++)
		{
			for (j=0; j<sizeof(int); j++)
				ptr[i*sizeof(int)+j] = (char)
					result[(i+1)*sizeof(int)-j-1];
		}
	}
	else
	{
		// leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = result[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d ints - ", num);             	
		for (i=0; i<num; i++)
			printf("%d ", val[i]);
		printf("\n");
	}

	send_ack();
	recv_ack();

	return num;
};

// receive some floats, returns num of floats received
int  SpecificWorker::recv_floats(float *val, int maxlen)
{
	int numbytes = 0;
	int i, j;
	char *temp;
        char *result;
	int end = 0;
	int total_bytes = 0;

	temp = (char *) buffer;
	result = (char *) buffer2;

	j = 0;

	// we receiving the incoming ints one byte at a time
	// oh cross language sockets are so much fun...

	while (!end)
	{
		if ((numbytes=recv(new_fd, temp, BUFFSIZE, 0))==-1)
			perror("recv");
		for (i=0; i<numbytes; i++)
		{
			result[j] = temp[i];	
			j++;
		}

		total_bytes = total_bytes + numbytes;
		if (total_bytes==maxlen*sizeof(float))
			end = 1;

	}

	// now we need to put the array of bytes into the array of floats

	char *ptr;
	int num = j/sizeof(float);

	ptr = (char *) val;

	// the significance order depends on the platform

	if (REVERSE)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < num; i++)
		{
			for (j=0; j<sizeof(float); j++)
				ptr[i*sizeof(float)+j] = (char)
					result[(i+1)*sizeof(float)-j-1];
		}
	}
	else
	{
		// leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = result[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d floats - ", num);             	
		for (i=0; i<num; i++)
			printf("%f ", val[i]);
		printf("\n");
	}

	send_ack();
	recv_ack();

	return num;

};

// receive some doubles, returns num of doubles received
int  SpecificWorker::recv_doubles(double *val, int maxlen)
{
	int numbytes = 0;
	int i, j;
	char *temp;
        char *result;
	int end = 0;
	int total_bytes = 0;

	temp = (char *) buffer;
	result = (char *) buffer2;

	j = 0;

	// we receiving the incoming ints one byte at a time
	// oh cross language sockets are so much fun...

	while (!end)
	{
		if ((numbytes=recv(new_fd, temp, BUFFSIZE, 0))==-1)
			perror("recv");
		for (i=0; i<numbytes; i++)
		{
			result[j] = temp[i];	
			j++;
		}

		total_bytes = total_bytes + numbytes;
		if (total_bytes==maxlen*sizeof(double))
			end = 1;

	}

	// now we need to put the array of bytes into the array of floats

	char *ptr;
	int num = j/sizeof(double);

	ptr = (char *) val;

	// the significance order depends on the platform

	if (REVERSE)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < num; i++)
		{
			for (j=0; j<sizeof(double); j++)
				ptr[i*sizeof(double)+j] = (char)
					result[(i+1)*sizeof(double)-j-1];
		}
	}
	else
	{
		// leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = result[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d doubles - ", num);             	
		for (i=0; i<num; i++)
			printf("%e ", val[i]);
		printf("\n");
	}

	send_ack();
	recv_ack();

	return num;
};

// receive some ints, returns num of ints received
int  SpecificWorker::recv_bytes(char *val, int maxlen)
{
        int numbytes = 0;
        int i, j;
        char *temp; 
        int end = 0;
        int total_bytes = 0;
                        
        temp = (char *) buffer;

        j = 0;
 
        // we receiving the incoming ints one byte at a time
        // oh cross language sockets are so much fun...
        
        while (!end)
        {
                if ((numbytes=recv(new_fd, temp, BUFFSIZE, 0))==-1)
                        perror("recv");
                for (i=0; i<numbytes; i++)
                {
                        val[j] = temp[i];
                        j++;
                }
                
                total_bytes = total_bytes + numbytes;
                if (total_bytes==maxlen)
                        end = 1;

                if (total_bytes==maxlen)
                        end = 1;
        
        }
        
        if (VERBOSE)
        {
                printf("Server: received %d bytes - ", maxlen);
                for (i=0; i<maxlen; i++)
                        printf("%d ", val[i]);
                printf("\n");
        }
                 
        send_ack();
        recv_ack();
                 
        return maxlen;
};

// receive a datagram
int  SpecificWorker::recv_datagram(char *val, int maxlen)
{
        int numbytes = 0;
        //int addr_len, i;
	int i;
	
	struct sockaddr_in from_addr;

	socklen_t addr_len;
        addr_len = sizeof(struct sockaddr);
        if ((numbytes=recvfrom(datasockfd, val, maxlen, 0,
                (struct sockaddr *) &from_addr, &addr_len)) == -1)
        {
                perror("recvfrom datagram");
        }

        if (VERBOSE)
        {
                printf("Server: received datagram of %d bytes - ", numbytes); 
                for (i=0; i<numbytes; i++)
                        printf("%d ", val[i]);
                printf("\n");
        }

        return numbytes;
};

                
// shut down the socket
void  SpecificWorker::closesocket()
{
	close(new_fd);
};

// recv a short ack from the client 
void  SpecificWorker::recv_ack()
{
	char temp[1];
	int total = 0;

	if (VERBOSE)
		printf("Waiting for ack...\n");

	while (total<1)
	   total += recv(new_fd, temp, 1, 0);	

	if (VERBOSE)
		printf("Ack recieved.\n");

};

// send a short ack to the client 
void  SpecificWorker::send_ack()
{
	char temp[1];
	temp[0] = 42;

	if (VERBOSE)
		printf("Sending ack...\n");
      
	send(new_fd, temp, 1, 0);
};


