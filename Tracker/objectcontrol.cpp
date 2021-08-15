
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "procmessage.h"
#include "position.h"
#include "objectcontrol.h"


#define CONTROL_CYCLE_TIME 0.25


ObjectControl::ObjectControl()
{

}

ObjectControl::ObjectControl(Position *position)
    : position(position)
    , ctrlPos(0,0)
    , Kp(0.4)//Kp(0.5)
    , Ti(0.015)//Ti(0.009)
    , Td(0.5)
    , width(0)
    , height(0)
    , uKiOld(0,0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , speedAvail(false)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPosBuf = Point2d(0,0);
    speedObj = Point2d(0,0);
}

ObjectControl::ObjectControl(Position *position, ProcMessage *proc)
    : position(position)
    , procMsg(proc)
    , ctrlPos(0,0)
    , Kp(0.4)//Kp(0.5)
    , Ti(0.015)//Ti(0.009)
    , Td(0.5)
    , width(0)
    , height(0)
    , uKiOld(0,0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , speedAvail(false)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPosBuf = Point2d(0,0);
    speedObj = Point2d(0,0);
}


ObjectControl::~ObjectControl()
{

}

void ObjectControl::init(double width, double height)
{
    cout << "init = " << width << "x" << height << endl;
    uKiOld = Point2d(0,0);
    
    //arcsecondPerPixel = 9802.0 / height;
    arcsecondPerPixel = 857.0 / height;
    cout << "arcsecondPerPixel = " << arcsecondPerPixel << endl;
    
    ctrlPos.x = 1143.0 * 0.5;
    ctrlPos.y = 857.0 * 0.5;
    
    inPosBuf.x = width * 0.5;
    inPosBuf.y = height * 0.5;
    
    arcsecondsSpeedLimitedOld = Point2i(0,0);
    
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    initFlag = true;
}

void ObjectControl::deInit()
{
    cout << "deInit" << endl;
    speedAvail = false;
    position->setVariableAzm( 0 );
    position->setVariableAlt( 0 );
}

void ObjectControl::process( Point2d inPos )
{
    Point2d inPosArc = inPos * arcsecondPerPixel;
    
    if( initFlag == true )
    {
        for( int i=0; i<8; i++ )
        {
            deltaInPos[i] = inPosArc;
        }
        initFlag = false;
    }
    Point2d uDiff = inPosArc - ctrlPos;
    
    Point2d uKp = Kp * uDiff;
    
    Point2d uKi = (Ti * uDiff)+uKiOld;
    if( uKi.x >= 35.7 )
    {
        uKi.x = 35.7;
    }
    else
    if( uKi.x <= -35.7 )
    {
        uKi.x = -35.7;
    }
    
    if( uKi.y >= 35.7 )
    {
        uKi.y = 35.7;
    }
    else
    if( uKi.y <= -35.7 )
    {
        uKi.y = -35.7;
    }
    
    if( (uDiff.x < 3.6) && (uDiff.x > -3.6) )
    {
        uKiOld.x = 0.0;
    }
    else
    {
        uKiOld.x = uKi.x;
    }
    
    if( (uDiff.y < 3.6) && (uDiff.y > -3.6) )
    {
        uKiOld.y = 0.0;
    }
    else
    {
        uKiOld.y = uKi.y;
    }
            
    for( int i=0; i<7; i++ )
    {
        deltaInPos[7-i] = deltaInPos[6-i];
    }
    deltaInPos[0] = inPosArc;
    Point2d uKd = Td * (deltaInPos[0]-deltaInPos[7]); 
    //cout << "uKd = " << uKd << "''/s" << endl;
    
    Point2d uPID = uKp + uKi + uKd;
    //cout << "uPID = " << uPID << "''/s" << endl;
    
    Point2i arcsecondsSpeed = static_cast<Point2i>(uPID);   
    //cout << "arcsec/s = " << arcsecondsSpeed << "''/s" << endl;
    
    Point2i arcsecondsSpeedLimited;
    arcsecondsSpeedLimited = speedLimit( arcsecondsSpeed );
    
    if( speedAvail == true )
    {
        if( (uDiff.y < 3.6) && (uDiff.y > -3.6) && (uDiff.x < 3.6) && (uDiff.x > -3.6) )
        {
            speedAvail = false;
            //arcsecondsSpeedLimited = (speedObj * arcsecondPerPixel);
            cout << "speedAvail arcsecLim/s = " << speedObj * arcsecondPerPixel << "''/s" << endl;
        }
    }
    
    if( arcsecondsSpeedLimitedOld.x != arcsecondsSpeedLimited.x )
    {
        position->setVariableAzm(arcsecondsSpeedLimited.x);
        cout << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    
    if( arcsecondsSpeedLimitedOld.y != arcsecondsSpeedLimited.y )
    {
        position->setVariableAlt(arcsecondsSpeedLimited.y);
        cout << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
        
    arcsecondsSpeedLimitedOld = arcsecondsSpeedLimited;
    
    return;
}

int ObjectControl::processMsg()
{
    int ret = 0;
    
    controlCycleTime();
    
    string rec = procMsg->receiveServerFromClient();
    if( rec.length() > 1 )
    {
        size_t pos;
        bool deInitFlag = false;
        
        //cout << "rec: " << rec << endl;
        cout.flush();
        
        if( (pos = rec.find("speed")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            size_t startchar = sub.find('=');
            if( startchar != string::npos )
            {
                size_t xchar = sub.find('x');
                if( xchar != string::npos )
                {
                    size_t endchar = sub.find(';');
                    if( endchar != string::npos )
                    {
                        string vx = sub.substr(startchar+1, xchar-1); 
                        string vy = sub.substr(xchar+1, endchar-1); 
                        speedObj.x = (double)stod(vx);
                        speedObj.y = (double)stod(vy);
                        speedAvail = true;
                        cout << "speed = " << speedObj << endl; 
                    }
                }
            }           
        }
        
        if( (pos = rec.find("init")) != string::npos )
        {
            string sub = rec.substr(pos+4);
            size_t startchar = sub.find('=');
            if( startchar != string::npos )
            {
                size_t xchar = sub.find('x');
                if( xchar != string::npos )
                {
                    size_t endchar = sub.find(';');
                    if( endchar != string::npos )
                    {
                        string width = sub.substr(startchar+1, xchar-1); 
                        string height = sub.substr(xchar+1, endchar-1); 
                        double dwidth = (double)stoi(width);
                        double dheight = (double)stoi(height);
                        init(dwidth, dheight);
                        trackFlag = true;
                    }
                }
            }
        }
        else
        if( (pos = rec.find("roipt")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            size_t startchar = sub.find('=');
            if( startchar != string::npos )
            {
                size_t xchar = sub.find('x');
                if( xchar != string::npos )
                {
                    size_t endchar = sub.find(';');
                    if( endchar != string::npos )
                    {
                        if( trackFlag == true )
                        {
                            string width = sub.substr(startchar+1, xchar-1); 
                            string height = sub.substr(xchar+1, endchar-1); 
                            inPosBuf.x = (double)stoi(width);
                            inPosBuf.y = (double)stoi(height);
                        }
                    }
                }
            }
        }
        else
        if( (pos = rec.rfind("rate=")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            cout << "rate: " << (int)stoi(sub) << endl;
            position->setFixedRate( (char)stoi(sub) );
        }  
        else
        if( (pos = rec.rfind("alt=1")) != string::npos )
        {
            cout << "alt=1" << endl;
            position->setFixedAlt( 1 );
            trackFlag = false;
        }  
        else
        if( (pos = rec.rfind("alt=-1")) != string::npos )
        {
            cout << "alt=-1" << endl;
            position->setFixedAlt( -1 );
            trackFlag = false;
        }  
        else
        if( (pos = rec.rfind("azm=-1")) != string::npos )
        {
            cout << "Position: left" << endl;
            position->setFixedAzm( -1 );
            trackFlag = false;
        }  
        else
        if( (pos = rec.rfind("azm=1")) != string::npos )
        {
            cout << "azm=1" << endl;
            position->setFixedAzm( 1 );
            trackFlag = false;
        }  
        else
        if( (pos = rec.rfind("alt=0")) != string::npos )
        {
            cout << "alt=0" << endl;
            position->setFixedAlt( 0 );
        }  
        else
        if( (pos = rec.rfind("azm=0")) != string::npos )
        {
            cout << "azm=0" << endl;
            position->setFixedAzm( 0 );
        }  
        else
        if( (pos = rec.find("exit")) != string::npos )
        {
            deInit();
            trackFlag = false;
            ret = -1;
        }
        // no else
        if( (pos = rec.find("deInit")) != string::npos )
        {
            deInit();
            trackFlag = false;
        }

    }
    
    if( trackFlag == true )
    {
        process(inPosBuf);
    }
    
    return ret;
}

Point2i ObjectControl::speedLimit(Point2i speed)
{
    Point2i retSpeed;
    // Limit and Raster
    int speedField[2];
    
    speedField[0] = speed.x;
    if( speed.x < 0 ) speedField[0] = -speed.x;
    speedField[1] = speed.y;
    if( speed.y < 0 ) speedField[1] = -speed.y;
    
    for( int i=0; i<2; i++ )
    {
        if( speedField[i] >= 100 )
        {
            speedFieldOut[i] = 100;
        }
        else
        if( (speedField[i] <= 90) && (speedField[i] >= 80) )
        {
            speedFieldOut[i] = 80;
        }
        else
        if( (speedField[i] <= 70) && (speedField[i] >= 60) ) 
        {
            speedFieldOut[i] = 60;
        }
        else
        if( (speedField[i] <= 50) && (speedField[i] >= 40) ) 
        {
            speedFieldOut[i] = 40;
        }
        else
        if( (speedField[i] <= 30) && (speedField[i] >= 20) ) 
        {
            speedFieldOut[i] = 20;
        }
        else
        if( speedField[i] <= 10 )
        {
            speedFieldOut[i] = 0;
        }
    }
        
    retSpeed.x = speedFieldOut[0];
    if( speed.x < 0 ) retSpeed.x = -speedFieldOut[0];
    retSpeed.y = speedFieldOut[1];
    if( speed.y > 0 ) retSpeed.y = -speedFieldOut[1];
    
    return retSpeed;
}

void ObjectControl::controlCycleTime()
{
    // Call Tracking Control algorithm
    clock_t cycleTimeStop = clock();
    double cycleTimeDiff = double(cycleTimeStop - cycleTimeStart)/CLOCKS_PER_SEC;
    if( cycleTimeDiff < (double)CONTROL_CYCLE_TIME )
    {
        unsigned int waitCycle = (CONTROL_CYCLE_TIME - cycleTimeDiff) * 1000000;
        //cout << "cycleTimeDiff: " << cycleTimeDiff << endl;
        //cout << "wait: " << waitCycle << endl;
        //cout.flush();
        usleep( waitCycle );
    }
    //if( cycleTimeDiff >= (double)CONTROL_CYCLE_TIME )
    {
        cycleTimeStart = clock();
        //cout << "cycleTimeDiff: " << cycleTimeDiff << endl;
        //cout << "Object: x: " << roipt.x << ", y: " << roipt.y << endl;
        //cout.flush();
    }
    
    return;
}
