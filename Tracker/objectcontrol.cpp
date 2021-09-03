
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
    , Kp(0.2)//Kp(0.4) Kp(0.5)
    , Ti(0.005)//Ti(0.015) Ti(0.009)
    , Td(0.5)
    , width(0)
    , height(0)
    , uKiOld(0,0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(0,0)
    , arcsecondsSpeedPT1(0,0)
    , predictCalc(false)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPosBuf = Point2d(0.0,0.0);
    speedObj = Point2d(0,0);
}

ObjectControl::ObjectControl(Position *position, ProcMessage *proc)
    : position(position)
    , procMsg(proc)
    , ctrlPos(0,0)
    , Kp(0.2)//Kp(0.4) Kp(0.5)
    , Ti(0.005)//Ti(0.015) Ti(0.009)
    , Td(0.5) //Object speed -> 8 entries every 0.25s = 2s -> 0.5 1/s
    , width(0)
    , height(0)
    , uKiOld(0,0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(0,0)
    , arcsecondsSpeedPT1(0,0)    
    , predictCalc(false)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPosBuf = Point2d(0.0,0.0);
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
    cout << dec << "Init inPosBuf = " << inPosBuf << endl;
    
    arcsecondsSpeedLimitedOld = Point2i(0,0);
    arcsecondsSpeedPredict = Point2i(0,0);
    arcsecondsSpeedPT1 = Point2i(0,0);
    
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    initFlag = true;
}

void ObjectControl::deInit()
{
    cout << "deInit" << endl;
    predictCalc = false;
    position->setVariableAzm( 0 );
    position->setVariableAlt( 0 );
    //position->setFixedAzm( 0 );
    //position->setFixedAlt( 0 );
}

void ObjectControl::process()
{
    Point2d inPosArc = arcsecondPerPixel * inPosBuf;
    //cout << dec << "inPosArc = " << inPosArc << "''" << endl;
    
    if( initFlag == true )
    {
        for( int i=0; i<8; i++ )
        {
            deltaInPos[i] = inPosArc;
        }
        initFlag = false;
    }
    Point2d uDiff = inPosArc - ctrlPos;
    //cout << "uDiff = " << uDiff << "''" << endl;
    
    Point2d uKp = Kp * uDiff;
    //cout << "uKp = " << uKp << "''/s" << endl;
    
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
    //cout << "uKi = " << uKi << "''/s" << endl;
    Point2d uKd = Td * (deltaInPos[0]-deltaInPos[7]); 
    //cout << "uKd = " << uKd << "''/s" << endl;
    
    Point2d uPID = uKp + uKi + uKd;
    //cout << "uPID = " << uPID << "''/s" << endl;
    
    Point2i arcsecondsSpeed = static_cast<Point2i>(uPID);  
    arcsecondsSpeed.y = -arcsecondsSpeed.y; 
    //cout << "arcsec/s = " << arcsecondsSpeed << "''/s" << endl;
    
    Point2i arcsecondsSpeedLimited;
    arcsecondsSpeedLimited = speedMax( arcsecondsSpeed );
    //arcsecondsSpeedLimited = speedLimit( arcsecondsSpeed );
    
    //cout << "uDiff = " << uDiff << "''" << endl;
    if( predictCalc == false )
    {
        Point2i arcsecondsSpeedTemp;
        arcsecondsSpeedTemp.x = arcsecondsSpeedPredict.x + (0.02 * (arcsecondsSpeedLimited.x - arcsecondsSpeedPredict.x));
        arcsecondsSpeedTemp.y = arcsecondsSpeedPredict.y + (0.02 * (arcsecondsSpeedLimited.y - arcsecondsSpeedPredict.y));
        arcsecondsSpeedPredict = arcsecondsSpeedTemp;
        //cout << "Mean arcsecLim/s = " << arcsecondsSpeedPredict << "''/s" << endl;
        //cout << "Act  arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    else
    {
        arcsecondsSpeedLimited = arcsecondsSpeedPredict;
        //cout << "Predict arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    
    //if( arcsecondsSpeedLimitedOld.x != arcsecondsSpeedLimited.x )
    {
        position->setVariableAzm(arcsecondsSpeedLimited.x);
        //cout << dec << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    
    //if( arcsecondsSpeedLimitedOld.y != arcsecondsSpeedLimited.y )
    {
        position->setVariableAlt(arcsecondsSpeedLimited.y);
        //cout << dec << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
        
    //arcsecondsSpeedLimitedOld = arcsecondsSpeedLimited;
    
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
        
        if( (pos = rec.find("notrack")) != string::npos )
        {
            cout << "notrack: " << arcsecondsSpeedPredict << endl;
            predictCalc = true;
        }
        else
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
                        string width = sub.substr(startchar+1, xchar-1); 
                        string height = sub.substr(xchar+1, endchar-1); 
                        inPosBuf.x = (double)stoi(width);
                        inPosBuf.y = (double)stoi(height);
                        trackFlag = true;
                        predictCalc = false;
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
            cout << "azm=-1" << endl;
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
        process();
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

Point2i ObjectControl::speedMax(Point2i speed)
{
    Point2i retSpeed;
    
    retSpeed = speed;
    if( abs(speed.x) > 100 )
    {
        if( speed.x < 0 ) 
        {
            retSpeed.x = -100;
        }
        else
        {
            retSpeed.x = 100;
        }
    }
    if( abs(speed.y) > 100 )
    {
        if( speed.y < 0 ) 
        {
            retSpeed.y = -100;
        }
        else
        {
            retSpeed.y = 100;
        }
    }
        
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
    
    cycleTimeStart = clock();
    //cout << "cycleTimeDiff: " << cycleTimeDiff << endl;
    //cout << "waitTimeDiff: " << waitTimeDiff << endl;
    //cout << "Object: x: " << roipt.x << ", y: " << roipt.y << endl;
    //cout.flush();

    
    return;
}
