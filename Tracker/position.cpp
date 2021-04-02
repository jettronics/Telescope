
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <termios.h>
#include <vector>
#include <array>

#include "position.h"

Position::Position()
    : filestream(-1)
    , fixedRate(1)
    , lastDirVarAzm(6)
    , lastDirVarAlt(6)
    , lastDirFixAzm(36)
    , lastDirFixAlt(36)
{
    //strSend.clear();
    arrSend.clear();
}

Position::~Position()
{

}


void Position::init()
{
    cout << "Position init" << endl;
    
    filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (filestream == -1) 
    {
        cout << "UART open error" << endl;
    }

    struct termios options;
    tcgetattr(filestream, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    options.c_cflag |=  (CLOCAL | CREAD); 
    options.c_cflag &=  ~CSIZE; 
    options.c_cflag &=  ~CSTOPB; 
    options.c_cflag &=  ~PARENB; 
    options.c_cflag |=  CS8; 
    
    /*options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; 
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;*/
        
    //tcflush(filestream, TCIFLUSH);
    tcsetattr(filestream, TCSANOW, &options);
    return;
}

void Position::deInit()
{
    cout << "Position deinit" << endl;
    
    close(filestream);
    
    return;
}

void Position::setFixedAzm( int azm )
{
    array<char, 8> chrRate; 
    
    chrRate[0] = (char)'P';
    chrRate[1] = (char)2;
    chrRate[2] = (char)16;
    if( azm > 0 )
    {
        chrRate[3] = (char)36;
        chrRate[4] = (char)fixedRate;
    }
    else
    if( azm < 0 )
    {
        chrRate[3] = (char)37;
        chrRate[4] = (char)fixedRate;
    }
    else
    {
        chrRate[3] = lastDirFixAzm;
        chrRate[4] = (char)0;
    }
    chrRate[5] = (char)0;
    chrRate[6] = (char)0;
    chrRate[7] = (char)0;
        
    arrSend.push_back(chrRate);
    
    lastDirFixAzm = chrRate[3];
    
    return;
}

void Position::setFixedAlt( int alt )
{
    array<char, 8> chrRate; 
    
    chrRate[0] = (char)'P';
    chrRate[1] = (char)2;
    chrRate[2] = (char)17;
    if( alt > 0 )
    {
        chrRate[3] = (char)36;
        chrRate[4] = (char)fixedRate;
    }
    else
    if( alt < 0 )
    {
        chrRate[3] = (char)37;
        chrRate[4] = (char)fixedRate;
    }
    else
    {
        chrRate[3] = lastDirFixAlt;
        chrRate[4] = (char)0;
    }
    chrRate[5] = (char)0;
    chrRate[6] = (char)0;
    chrRate[7] = (char)0;
        
    arrSend.push_back(chrRate);
    
    lastDirFixAlt = chrRate[3];
    
    return;
}

void Position::setVariableAzm( int azm )
{
    array<char, 8> chrRate; 
    char rh = 0, rl = 0;
    
    chrRate[0] = (char)'P';
    chrRate[1] = (char)3;
    chrRate[2] = (char)16;
    if( azm > 0 )
    {
        rh = (char)(azm >> 6);
        rl = (char)((azm << 2) % 256);
        chrRate[3] = (char)6;
        chrRate[4] = (char)rh;
        chrRate[5] = (char)rl;
    }
    else
    if( azm < 0 )
    {
        int azmpos = -azm;
        rh = (char)(azmpos >> 6);
        rl = (char)((azmpos << 2) % 256);
        chrRate[3] = (char)7;
        chrRate[4] = (char)rh;
        chrRate[5] = (char)rl;
    }
    else
    {
        chrRate[3] = lastDirVarAzm;
        chrRate[4] = (char)0;
        chrRate[5] = (char)0;
    }
    
    //cout << "Variable azm: " << azm << ", RH: " << (int)rh << ", RL: " << (int)rl << endl;
    
    chrRate[6] = (char)0;
    chrRate[7] = (char)0;
        
    arrSend.push_back(chrRate);
    
    lastDirVarAzm = chrRate[3];
    
    return;
}

void Position::setVariableAlt( int alt )
{
    array<char, 8> chrRate; 
    char rh = 0, rl = 0;    
    
    chrRate[0] = (char)'P';
    chrRate[1] = (char)3;
    chrRate[2] = (char)17;
    if( alt > 0 )
    {
        rh = (char)(alt >> 6);
        rl = (char)((alt << 2) % 256);
        chrRate[3] = (char)6;
        chrRate[4] = (char)rh;
        chrRate[5] = (char)rl;
    }
    else
    if( alt < 0 )
    {
        int altpos = -alt;
        rh = (char)(altpos >> 6);
        rl = (char)((altpos << 2) % 256);
        chrRate[3] = (char)7;
        chrRate[4] = (char)rh;
        chrRate[5] = (char)rl;
    }
    else
    {
        chrRate[3] = lastDirVarAlt;
        chrRate[4] = (char)0;
        chrRate[5] = (char)0;
    }
    
    //cout << "Variable alt: " << alt << ", RH: " << (int)rh << ", RL: " << (int)rl << endl;
    
    chrRate[6] = (char)0;
    chrRate[7] = (char)0;
        
    arrSend.push_back(chrRate);
    
    lastDirVarAlt = chrRate[3];
    
    return;    
}

void Position::process()
{
    if( filestream != -1 )
    {
        if( arrSend.size() > 0 )
        {
            //cout << "Position send:";
            for( int i=0; i < 8; i++ )
            {
                arrBuf[i] = arrSend.at(0)[i];
                //cout << " " << (int)arrBuf[i];
            }
            //cout << endl;
            int rettx = write(filestream, arrBuf, 8);
            arrSend.erase(arrSend.begin()+0);
            if (rettx < 0) 
            {
                cout << "UART TX error" << endl;
            }
            cout.flush();
        }
                
        int retrx = read(filestream, rxBuffer, 1000);

        if( retrx < 0 ) 
        {
            rxBuffer[0] = '\0';
        } 
        else 
        if( retrx >= 0) 
        {
            rxBuffer[retrx] = '\0';
            string rxRead = rxBuffer;
            cout << "UART RX length: " << retrx << endl;
            cout << "UART RX data: " << rxRead << endl;
            cout.flush();
        } 
        
    }
    return;
}



