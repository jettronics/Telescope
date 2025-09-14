
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <termios.h>
#include <vector>
#include <array>
#include <string.h>

#include "setup.h"
#include "position.h"

Position::Position()
    : filestream(-1)
    , fixedRate(1)
    , fixedRateAzm(1)
    , fixedRateAlt(1)
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
    , slewingActive(0)
    , gotoActive(0)
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
    
    arrBufGoto[0] = (char)'b';
    arrBufGoto[1] = (char)'0';
    arrBufGoto[2] = (char)'0';
    arrBufGoto[3] = (char)'0';
    arrBufGoto[4] = (char)'0';
    arrBufGoto[5] = (char)'0';
    arrBufGoto[6] = (char)'0';
    arrBufGoto[7] = (char)'0';
    arrBufGoto[8] = (char)'0';
    arrBufGoto[9] = (char)',';
    arrBufGoto[10] = (char)'0';
    arrBufGoto[11] = (char)'0';
    arrBufGoto[12] = (char)'0';
    arrBufGoto[13] = (char)'0';
    arrBufGoto[14] = (char)'0';
    arrBufGoto[15] = (char)'0';
    arrBufGoto[16] = (char)'0';
    arrBufGoto[17] = (char)'0';
}

Position::~Position()
{
}

PositionUsb::PositionUsb()
    : Position()
{
}

PositionUsb::~PositionUsb()
{
}

void Position::init()
{
#if 0    
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
    
    options.c_cflag &=  ~CSIZE; 
    options.c_cflag &=  ~CSTOPB; 
    options.c_cflag &=  ~PARENB; 
    options.c_cflag &=  ~CRTSCTS;
    options.c_cflag |=  CS8; 
    options.c_cflag |=  (CLOCAL | CREAD);  
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_iflag = IGNPAR;           /* Parity-Fehler ignorieren */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    options.c_oflag &= ~OPOST;          /* setze "raw" Input */
    options.c_oflag &= ~ONLCR;
    options.c_cc[VMIN]  = 0;            /* warten auf min. 0 Zeichen */
    options.c_cc[VTIME] = 0;            /* Timeout */
    tcflush(filestream,TCIOFLUSH);              /* Puffer leeren */  
    tcsetattr(filestream, TCSAFLUSH, &options);     
    
    //tcsetattr(filestream, TCSANOW, &options);
#endif
    return;
}

void PositionUsb::init()
{
    cout << "Position usb init" << endl;

#if 0    
    //arrSend.clear();
    
    filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    //filestream = open("/dev/ttyUSB0", O_RDWR);
    if (filestream == -1) 
    {
        cout << "UART open error" << endl;
    }

    struct termios options;
    tcgetattr(filestream, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    options.c_cflag &=  ~CSIZE; 
    options.c_cflag &=  ~CSTOPB; 
    options.c_cflag &=  ~PARENB; 
    options.c_cflag &=  ~CRTSCTS;
    options.c_cflag |=  CS8; 
    options.c_cflag |=  (CLOCAL | CREAD);  
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    //options.c_iflag = IGNPAR;           /* Parity-Fehler ignorieren */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    options.c_oflag &= ~OPOST;          /* setze "raw" Input */
    options.c_oflag &= ~ONLCR;
    options.c_cc[VMIN]  = 0;            /* warten auf min. 0 Zeichen */
    options.c_cc[VTIME] = 0;            /* Timeout */
    tcflush(filestream,TCIOFLUSH);              /* Puffer leeren */  
    tcsetattr(filestream, TCSAFLUSH, &options);     
    
    //tcsetattr(filestream, TCSANOW, &options);
#endif    
    
    
    filestream = open("/dev/ttyUSB0", O_RDWR );

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(filestream, &tty) != 0) 
    {
        cout << "UART error: " << errno  << ", " << strerror(errno) << endl;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 50;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(filestream, TCSANOW, &tty) != 0) 
    {
        cout << "tcsetattr error: " << errno  << ", " << strerror(errno) << endl;
    }

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
    char rh = 0, rl = 0;
    
    cout << "setFixedAzm: " << azm << endl;
    
    arrBufAzm[0] = (char)'P';
    //arrBufAzm[1] = (char)2;
    arrBufAzm[1] = (char)3;
    arrBufAzm[2] = (char)16;
    if( azm > 0 )
    {
        //arrBufAzm[3] = (char)36;
        //arrBufAzm[4] = (char)fixedRate;
        rh = (char)(azm >> 6);
        rl = (char)((azm << 2) % 256);
        arrBufAzm[3] = (char)6;
        arrBufAzm[4] = (char)rh;
        arrBufAzm[5] = (char)rl;        
    }
    else
    if( azm < 0 )
    {
        //arrBufAzm[3] = (char)37;
        //arrBufAzm[4] = (char)fixedRate;
        int azmpos = -azm;
        rh = (char)(azmpos >> 6);
        rl = (char)((azmpos << 2) % 256);
        arrBufAzm[3] = (char)7;
        arrBufAzm[4] = (char)rh;
        arrBufAzm[5] = (char)rl;        
    }
    else
    {
        //arrBufAzm[3] = lastDirFixAzm;
        //arrBufAzm[4] = (char)0;
        arrBufAzm[3] = lastDirVarAzm;
        arrBufAzm[4] = (char)0;
        arrBufAzm[5] = (char)0;
    }
/*
    arrBufAzm[5] = (char)0;
    arrBufAzm[6] = (char)0;
    arrBufAzm[7] = (char)0;
        
    lastDirFixAzm = arrBufAzm[3];
*/    
    arrBufAzm[6] = (char)0;
    arrBufAzm[7] = (char)0;
        
    lastDirVarAzm = arrBufAzm[3];
        
    return;
}

void Position::setFixedAlt( int alt )
{
    char rh = 0, rl = 0;   
    
    cout << "setFixedAlt: " << alt << endl;
    
    arrBufAlt[0] = (char)'P';
    //arrBufAlt[1] = (char)2;
    arrBufAlt[1] = (char)3;
    arrBufAlt[2] = (char)17;
    if( alt > 0 )
    {
        //arrBufAlt[3] = (char)36;
        //arrBufAlt[4] = (char)fixedRate;
        rh = (char)(alt >> 6);
        rl = (char)((alt << 2) % 256);
        arrBufAlt[3] = (char)6;
        arrBufAlt[4] = (char)rh;
        arrBufAlt[5] = (char)rl;        
    }
    else
    if( alt < 0 )
    {
        //arrBufAlt[3] = (char)37;
        //arrBufAlt[4] = (char)fixedRate;
        int altpos = -alt;
        rh = (char)(altpos >> 6);
        rl = (char)((altpos << 2) % 256);
        arrBufAlt[3] = (char)7;
        arrBufAlt[4] = (char)rh;
        arrBufAlt[5] = (char)rl;        
    }
    else
    {
        //arrBufAlt[3] = lastDirFixAlt;
        //arrBufAlt[4] = (char)0;
        arrBufAlt[3] = lastDirVarAlt;
        arrBufAlt[4] = (char)0;
        arrBufAlt[5] = (char)0;        
    }
/*    
    arrBufAlt[5] = (char)0;
    arrBufAlt[6] = (char)0;
    arrBufAlt[7] = (char)0;
        
    lastDirFixAlt = arrBufAlt[3];
*/    
    arrBufAlt[6] = (char)0;
    arrBufAlt[7] = (char)0;
        
    lastDirVarAlt = arrBufAlt[3];
    
    return;
}

void Position::setVariableAzm( int azm )
{
    char rh = 0, rl = 0;
    if( ((lastDirVarAzm == 6) && (azm < 0)) && (waitTurnAzm == false) )
    {
        waitTurnAzmCount = 0;
        waitTurnAzm = true;
        //cout << "setVariableAzm dir change wait 1" << endl;
    }
    else
    if( ((lastDirVarAzm == 7) && (azm > 0)) && (waitTurnAzm == false) )
    {
        waitTurnAzmCount = 0;
        waitTurnAzm = true;
        //cout << "setVariableAzm dir change wait 2" << endl;
    }

    if( waitTurnAzm == true )
    {
        if( waitTurnAzmCount > 8 )
        {
            waitTurnAzm = false;
            //cout << "setVariableAzm dir change allowed" << endl;
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
        
    lastDirVarAzm = arrBufAzm[3];
   
    return;
}

void Position::setVariableAlt( int alt )
{
    char rh = 0, rl = 0;   
    if( ((lastDirVarAlt == 6) && (alt < 0)) && (waitTurnAlt == false) )
    {
        waitTurnAltCount = 0;
        waitTurnAlt = true;
        //cout << "setVariableAlt dir change wait 1" << endl;
    }
    else
    if( ((lastDirVarAlt == 7) && (alt > 0)) && (waitTurnAlt == false) )
    {
        waitTurnAltCount = 0;
        waitTurnAlt = true;
        //cout << "setVariableAlt dir change wait 2" << endl;
    }
    
    if( waitTurnAlt == true )
    {
        if( waitTurnAltCount > 8 )
        {
            waitTurnAlt = false;
            //cout << "setVariableAlt dir change allowed" << endl;
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
        
    lastDirVarAlt = arrBufAlt[3];

    return;    
}

void Position::setGotoAzmAlt( double azm, double alt )
{
    cout << "Goto Azm: " << azm << ", Alt: " << alt << endl;

    int azmConv = 0; 
    if( azm > 0.0 )
    {
        azmConv = (int)((azm * (double)46603.38) + (double)0.5);
    }
    else
    {
        azmConv = (int)((azm * (double)46603.38) - (double)0.5);
    }
    int altConv = (int)((alt * (double)46603.38) + (double)0.5);
    stringstream azmConvHex;
    azmConvHex << uppercase << setfill('0') << setw(6) << hex << azmConv;
    stringstream altConvHex;
    altConvHex << uppercase << setfill('0') << setw(6) << hex << altConv;
    cout << "Azm hex: " << azmConvHex.str() << ", Alt hex: " << altConvHex.str() << endl;
    for( int i = 0; i < 6; i++ )
    {
        arrBufGoto[i+1] = (char)azmConvHex.str()[i];
        arrBufGoto[i+10] = (char)altConvHex.str()[i];
    }
    gotoActive = 1;

    return;
}


void Position::process()
{
    if( filestream != -1 )
    {
        if( (msgReceived == true) && (waitReceived >= 2) )
        {
            //cout << "Position send:";
            if( sendAzm == false )
            {
                sendAzm = true;
                for( int i=0; i < 8; i++ )
                {
                    arrBuf[i] = arrBufAzm[i];
                    /*cout << hex << " " << (int)arrBuf[i];
                    if( i == 7 )
                    {
                        cout << endl;
                    }*/
                }
            }
            else
            {
                sendAzm = false;
                for( int i=0; i < 8; i++ )
                {
                    arrBuf[i] = arrBufAlt[i];
                    /*cout << hex << " " << (int)arrBuf[i];
                    if( i == 7 )
                    {
                        cout << endl;
                    }*/
                }
            }
            //cout << endl;
            //cout.flush();
            int rettx = write(filestream, arrBuf, 8);
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
            //cout << "UART RX data: " << rxRead << endl;
            //cout << dec << "UART RX length: " << retrx << endl;
            /*for( int i=0; i < retrx; i++ )
            {
                cout << hex << " 0x" << (int)rxBuffer[i];
                if( i == retrx-1 )
                {
                    cout << endl;
                }
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

void PositionUsb::process()
{
    if( filestream != -1 )
    {
        if( (msgReceived == true) && (waitReceived >= 2) )
        {
            waitReceived = 0;
            
            int azmActive = (int)arrBufAzm[4] + (int)arrBufAzm[5];
            int altActive = (int)arrBufAlt[4] + (int)arrBufAlt[5];
            if( ((azmActive != 0) || (altActive != 0)) && (slewingActive == 0) )
            {
                slewingActive = 2;
                if( azmActive != 0 )
                {
                    sendAzm = true;
                }
                cout << "Slewing active" << endl;
            }
            
            if( slewingActive > 0 )
            {
                for( int i=0; i < 8; i++ )
                {
                    if( sendAzm == false )
                    {
                        arrBuf[i] = arrBufAlt[i];
                    }
                    else
                    {
                        arrBuf[i] = arrBufAzm[i];
                    }
                }
                if( sendAzm == false )
                {
                    sendAzm = true;
                }
                else
                {
                    sendAzm = false;
                }
                int rettx = write(filestream, arrBuf, 8);
                if (rettx < 0) 
                {
                    cout << "UART usb TX error" << endl;
                } 
                
                if( ((azmActive == 0) && (altActive == 0)) && (slewingActive > 0) )
                {
                    slewingActive--;
                    cout << "Slewing stopping: " << slewingActive << endl;
                }
            }
            else
            if( gotoActive > 0 )
            {
                gotoActive--;
                int rettx = write(filestream, arrBufGoto, 18);
                if (rettx < 0) 
                {
                    cout << "UART usb TX error" << endl;
                }
            }
            //cout.flush();
        }

//#if 0                
        msgReceived = true;

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

