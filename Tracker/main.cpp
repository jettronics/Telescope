
#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <sys/un.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/plot.hpp>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>

#include "tcpsocketcom.h"
#include "procmessage.h"
#include "focus.h"
#include "position.h"
#include "objectcontrol.h"
#include "camera.h"


using namespace std;


// "./%e"

int main(int argc, char *argv[]) 
{
    //pid_t pid; 
    pid_t pid[2];
    int stat;
    
    cout << "Tracker" << endl; 
    cout.flush();
    
    ProcMessage msgFocCam; // Server = focus, Client = camera
    ProcMessage msgPosCam; // Server = position, Client = camera
    
    msgFocCam.init(ProcMessage::UnblockingAll, ProcMessage::UnblockingAll);
    msgPosCam.init(ProcMessage::UnblockingRead, ProcMessage::UnblockingWrite);
    
    //pid = fork(); 
    pid[0] = fork();
    pid[1] = fork();
    
    //if (pid > 0)
    if (pid[0] > 0 && pid[1] > 0)
    {
        TcpSocketCom tcpsocketcom( 51717 );
        TcpSocketCom tcpsocketstream( 52717 );
        
        //Position position;
        //Camera camera( &tcpsocketcom, &tcpsocketstream, &msgFocCam, &position );
        Camera camera( &tcpsocketcom, &tcpsocketstream, &msgFocCam, &msgPosCam );
        
        //position.init();
        
        while( tcpsocketcom.init() < 0 )
        {
            sleep(1);
        }
        while( tcpsocketstream.init() < 0 )
        {
            sleep(1);
        }
     
        while( 1 )
        {
            if( camera.process() < 0 )
            {
                break;
            }
            //position.process();
        }

        //position.deInit();
        tcpsocketcom.deInit();
        tcpsocketstream.deInit();
        
        //waitpid(pid, &stat, 0);
        waitpid(pid[1], &stat, 0);
        waitpid(pid[0], &stat, 0);
    }
    //else
    else if (pid[0] == 0 && pid[1] > 0) 
    {
        int exitCond;
        Position position;
        Position *posUsb = new PositionUsb;
        ObjectControl objectcontrol(&position, posUsb, &msgPosCam);
        
        position.init();
        posUsb->init();
        
        while( 1 )
        {
            exitCond = objectcontrol.processMsg();
            position.process();
            posUsb->process();
            
            if( exitCond < 0 )
            {
                sleep(2);
                break;
            }
        }
        
        position.deInit();
        posUsb->deInit();
        
        delete posUsb;
        
        exit(0);        
    }
    else if (pid[1] == 0 && pid[0] > 0) 
    {
        Focus focus(&msgFocCam);
        
        focus.init();
        
        while( 1 )
        {
            if( focus.processMsg() < 0 )
            {
                break;
            }
        }
        
        focus.deInit();
        
        exit(0);
    }
    else
    {
        exit(0);
    }
    
    cout << "Tracker exit" << endl;
    cout.flush();
     
    return 0; 
}
