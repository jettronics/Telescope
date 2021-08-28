
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
    , msgReceived(true)
    , sendAzm(false)
    , waitReceived(0)
    , waitTurnAzm(false)
    , waitTurnAzmCount(0)
    , waitTurnAlt(false)
    , waitTurnAltCount(0)
{
    //strSend.clear();
    //arrSend.clear();
    arrBufAlt[0] = (char)'P';
    arrBufAlt[1] = (char)2;
    arrBufAlt[2] = (char)17;
    arrBufAlt[3] = lastDirFixAlt;
    arrBufAlt[4] = (char)0;
    arrBufAlt[5] = (char)0;
    arrBufAlt[6] = (char)0;
    arrBufAlt[7] = (char)0;
    
    arrBufAzm[0] = (char)'P';
    arrBufAzm[1] = (char)2;
    arrBufAzm[2] = (char)16;
    arrBufAzm[3] = lastDirFixAzm;
    arrBufAzm[4] = (char)0;
    arrBufAzm[5] = (char)0;
    arrBufAzm[6] = (char)0;
    arrBufAzm[7] = (char)0;
}

Position::~Position()
{

}


void Position::init()
{
    cout << "Position init" << endl;
    
    //arrSend.clear();
    
    filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (filestream == -1) 
    {
        cout << "UART open error" << endl;
    }

    struct termios options;
    tcgetattr(filestream, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    /*options.c_cflag |=  (CLOCAL | CREAD); 
    options.c_cflag &=  ~CSIZE; 
    options.c_cflag &=  ~CSTOPB; 
    options.c_cflag &=  ~PARENB; 
    options.c_cflag |=  CS8; */
    
    options.c_cflag &=  ~CSIZE; 
    options.c_cflag &=  ~CSTOPB; 
    options.c_cflag &=  ~PARENB; 
    options.c_cflag |=  CS8; 
    options.c_cflag |=  (CLOCAL | CREAD);  
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag = IGNPAR;           /* Parity-Fehler ignorieren */
    options.c_oflag &= ~OPOST;          /* setze "raw" Input */
    options.c_cc[VMIN]  = 0;            /* warten auf min. 0 Zeichen */
    options.c_cc[VTIME] = 0;            /* Timeout */
    tcflush(filestream,TCIOFLUSH);              /* Puffer leeren */  
    tcsetattr(filestream, TCSAFLUSH, &options);     
    
    //tcsetattr(filestream, TCSANOW, &options);
    
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
    //array<char, 8> chrRate; 
    
    arrBufAzm[0] = (char)'P';
    arrBufAzm[1] = (char)2;
    arrBufAzm[2] = (char)16;
    if( azm > 0 )
    {
        arrBufAzm[3] = (char)36;
        arrBufAzm[4] = (char)fixedRate;
    }
    else
    if( azm < 0 )
    {
        arrBufAzm[3] = (char)37;
        arrBufAzm[4] = (char)fixedRate;
    }
    else
    {
        arrBufAzm[3] = lastDirFixAzm;
        arrBufAzm[4] = (char)0;
    }
    arrBufAzm[5] = (char)0;
    arrBufAzm[6] = (char)0;
    arrBufAzm[7] = (char)0;
        
    //arrSend.push_back(chrRate);
    
    lastDirFixAzm = arrBufAzm[3];
    
    return;
}

void Position::setFixedAlt( int alt )
{
    //array<char, 8> chrRate; 
    
    arrBufAlt[0] = (char)'P';
    arrBufAlt[1] = (char)2;
    arrBufAlt[2] = (char)17;
    if( alt > 0 )
    {
        arrBufAlt[3] = (char)36;
        arrBufAlt[4] = (char)fixedRate;
    }
    else
    if( alt < 0 )
    {
        arrBufAlt[3] = (char)37;
        arrBufAlt[4] = (char)fixedRate;
    }
    else
    {
        arrBufAlt[3] = lastDirFixAlt;
        arrBufAlt[4] = (char)0;
    }
    arrBufAlt[5] = (char)0;
    arrBufAlt[6] = (char)0;
    arrBufAlt[7] = (char)0;
        
    //arrSend.push_back(chrRate);
    
    lastDirFixAlt = arrBufAlt[3];
    
    return;
}

void Position::setVariableAzm( int azm )
{
    //array<char, 8> chrRate; 
    char rh = 0, rl = 0;
    if( ((lastDirVarAzm == 6) && (azm < 0)) && (waitTurnAzm == false) )
    {
        waitTurnAzmCount = 0;
        waitTurnAzm = true;
        cout << "setVariableAzm dir change wait 1" << endl;
    }
    else
    if( ((lastDirVarAzm == 7) && (azm > 0)) && (waitTurnAzm == false) )
    {
        waitTurnAzmCount = 0;
        waitTurnAzm = true;
        cout << "setVariableAzm dir change wait 2" << endl;
    }

    if( waitTurnAzm == true )
    {
        if( waitTurnAzmCount > 8 )
        {
            waitTurnAzm = false;
            cout << "setVariableAzm dir change allowed" << endl;
        }
        else
        {
            azm = 0;
        }
    }
    
    arrBufAzm[0] = (char)'P';
    arrBufAzm[1] = (char)3;
    arrBufAzm[2] = (char)16;
    if( azm > 0 )
    {
        rh = (char)(azm >> 6);
        rl = (char)((azm << 2) % 256);
        arrBufAzm[3] = (char)6;
        arrBufAzm[4] = (char)rh;
        arrBufAzm[5] = (char)rl;
    }
    else
    if( azm < 0 )
    {
        int azmpos = -azm;
        rh = (char)(azmpos >> 6);
        rl = (char)((azmpos << 2) % 256);
        arrBufAzm[3] = (char)7;
        arrBufAzm[4] = (char)rh;
        arrBufAzm[5] = (char)rl;
    }
    else
    {
        arrBufAzm[3] = lastDirVarAzm;
        arrBufAzm[4] = (char)0;
        arrBufAzm[5] = (char)0;
    }
    
    //cout << "Variable azm: " << azm << ", RH: " << (int)rh << ", RL: " << (int)rl << endl;
    
    arrBufAzm[6] = (char)0;
    arrBufAzm[7] = (char)0;
        
    //arrSend.push_back(chrRate);
    
    lastDirVarAzm = arrBufAzm[3];
   
    return;
}

void Position::setVariableAlt( int alt )
{
    //array<char, 8> chrRate; 
    char rh = 0, rl = 0;   
    if( ((lastDirVarAlt == 6) && (alt < 0)) && (waitTurnAlt == false) )
    {
        waitTurnAltCount = 0;
        waitTurnAlt = true;
        cout << "setVariableAlt dir change wait 1" << endl;
    }
    else
    if( ((lastDirVarAlt == 7) && (alt > 0)) && (waitTurnAlt == false) )
    {
        waitTurnAltCount = 0;
        waitTurnAlt = true;
        cout << "setVariableAlt dir change wait 2" << endl;
    }
    
    if( waitTurnAlt == true )
    {
        if( waitTurnAltCount > 8 )
        {
            waitTurnAlt = false;
            cout << "setVariableAlt dir change allowed" << endl;
        }
        else
        {
            alt = 0;
        }
    }
    
    arrBufAlt[0] = (char)'P';
    arrBufAlt[1] = (char)3;
    arrBufAlt[2] = (char)17;
    if( alt > 0 )
    {
        rh = (char)(alt >> 6);
        rl = (char)((alt << 2) % 256);
        arrBufAlt[3] = (char)6;
        arrBufAlt[4] = (char)rh;
        arrBufAlt[5] = (char)rl;
    }
    else
    if( alt < 0 )
    {
        int altpos = -alt;
        rh = (char)(altpos >> 6);
        rl = (char)((altpos << 2) % 256);
        arrBufAlt[3] = (char)7;
        arrBufAlt[4] = (char)rh;
        arrBufAlt[5] = (char)rl;
    }
    else
    {
        arrBufAlt[3] = lastDirVarAlt;
        arrBufAlt[4] = (char)0;
        arrBufAlt[5] = (char)0;
    }
    
    //cout << "Variable alt: " << alt << ", RH: " << (int)rh << ", RL: " << (int)rl << endl;
    
    arrBufAlt[6] = (char)0;
    arrBufAlt[7] = (char)0;
        
    //arrSend.push_back(chrRate);
    
    lastDirVarAlt = arrBufAlt[3];

    return;    
}

void Position::process()
{
    if( filestream != -1 )
    {
        //if( arrSend.size() > 0 )
        {
            if( (msgReceived == true) && (waitReceived >= 2) )
            {
                //cout << "Position send:";
                if( sendAzm == false )
                {
                    sendAzm = true;
                    for( int i=0; i < 8; i++ )
                    {
                        //arrBuf[i] = arrSend.at(0)[i];
                        arrBuf[i] = arrBufAzm[i];
                        //cout << hex << " " << (int)arrBuf[i];
                    }
                }
                else
                {
                    sendAzm = false;
                    for( int i=0; i < 8; i++ )
                    {
                        //arrBuf[i] = arrSend.at(0)[i];
                        arrBuf[i] = arrBufAlt[i];
                        //cout << hex << " " << (int)arrBuf[i];
                    }
                }
                //cout << endl;
                //cout.flush();
                int rettx = write(filestream, arrBuf, 8);
                //arrSend.erase(arrSend.begin()+0);
                if (rettx < 0) 
                {
                    cout << "UART TX error" << endl;
                }
                else
                {
                    msgReceived = false;
                }
                //cout.flush();
            }
        }
//#if 0                
        int retrx = read(filestream, rxBuffer, 1000);

        if( retrx < 0 ) 
        {
            rxBuffer[0] = '\0';
            cout << "UART RX Error:";
            cout << hex << " 0x" << retrx << endl;            
        } 
        else 
        if( retrx >= 0) 
        {
            rxBuffer[retrx] = '\0';
            //string rxRead = rxBuffer;
            //cout << "UART RX length: " << retrx << endl;
            //cout << "UART RX data: " << rxRead << endl;
            //cout << dec << "UART RX length: " << retrx << endl;
            /*for( int i=0; i < retrx; i++ )
            {
                cout << hex << " 0x" << (int)rxBuffer[i];
            }*/

            if( retrx > 0 )
            {
                //cout << endl;
                msgReceived = true;
                waitReceived = 0;
                //cout << "msg received" << endl;
                //cout.flush();
            }
        } 
        
        if( msgReceived == true )
        {
            waitReceived++;
        }
        if( waitTurnAzm == true )
        {
            waitTurnAzmCount++;
        }
        if( waitTurnAlt == true )
        {
            waitTurnAltCount++;
        }
//#endif        
    }
    return;
}



