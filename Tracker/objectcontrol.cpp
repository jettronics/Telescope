
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "procmessage.h"
#include "position.h"
#include "objectcontrol.h"


#define CONTROL_CYCLE_TIME 0.25
#define CONTROL_FIXED_RATE 40


ObjectControl::ObjectControl()
{

}

ObjectControl::ObjectControl(Position *position, Position *position2)
    : position(position)
    , position2(position2)
    , ctrlPos(0,0)
    , Kp(0.205)//Kp(0.4) Kp(0.5)
    , Td(0.5)
    , width(0)
    , height(0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(15,0)
    , predictCalc(false)
    , manualPos(false)
    , manualPos2(false)    
    , selector(0)
    , dt(0.25)
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
    clock_gettime(CLOCK_REALTIME, &start);
}

ObjectControl::ObjectControl(Position *position, Position *position2, ProcMessage *proc)
    : position(position)
    , position2(position2)    
    , procMsg(proc)
    , ctrlPos(0,0)
    , Kp(0.205)//Kp(0.4) Kp(0.5)
    , Td(0.5) //Object speed -> 8 entries every 0.25s = 2s -> 0.5 1/s
    , width(0)
    , height(0)
    , arcsecondPerPixel(0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(0,0)
    , predictCalc(false)
    , manualPos(false) 
    , manualPos2(false)        
    , selector(0)   
    , dt(0.25)
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
    clock_gettime(CLOCK_REALTIME, &start);
}


ObjectControl::~ObjectControl()
{

}

void ObjectControl::init(double width, double height)
{
    cout << "init = " << width << "x" << height << endl;
    
    //arcsecondPerPixel = 9802.0 / height;
    arcsecondPerPixel = 857.0 / height;
    cout << "arcsecondPerPixel = " << arcsecondPerPixel << endl;
    
    ctrlPos.x = 1143.0 * 0.5;
    ctrlPos.y = 857.0 * 0.5;
    
    inPosBuf.x = width * 0.5;
    inPosBuf.y = height * 0.5;
    cout << dec << "Init inPosBuf = " << inPosBuf << endl;
    
    // Just for testing
    //inPosArc = arcsecondPerPixel * inPosBuf;
    
    arcsecondsSpeedLimitedOld = Point2i(0,0);
    arcsecondsSpeedPredict = Point2i(15,0);
    
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
    position2->setVariableAzm( 0 );
    position2->setVariableAlt( 0 );
    //position->setFixedAzm( 0 );
    //position->setFixedAlt( 0 );
}

void ObjectControl::process()
{
    Point2d inPosArc = arcsecondPerPixel * inPosBuf;
    // Just for testing
    /*if( inPosArc.x < 2000 )
    {
        inPosArc.x += 3.75;
    }*/

    //cout << dec << "inPosArc = " << inPosArc << "''" << endl;
    
    if( initFlag == true )
    {
        for( int i=0; i<8; i++ )
        {
            deltaInPos[i] = inPosArc;
        }
        for( int i=0; i<8; i++ )
        {
            dtPos[i] = dt;
        }
        initFlag = false;
    }
        
    Point2d uDiff = inPosArc - ctrlPos;
    //cout << "uDiff = " << uDiff << "''" << endl;
    
    Point2d uKp = Kp * uDiff;
    //cout << "uKp = " << uKp << "''/s" << endl;
    
    double dtSum = dt;
    for( int i=0; i<7; i++ )
    {
       dtPos[7-i] = dtPos[6-i];
       dtSum += dtPos[6-i];
    }
    dtPos[0] = dt;
    Td = ((double)1.0) / dtSum;
    //cout << "Td = " << Td << "1/s" << ", dt = " << dt << "s" << endl;

    for( int i=0; i<7; i++ )
    {
        deltaInPos[7-i] = deltaInPos[6-i];
    }
    deltaInPos[0] = inPosArc;

    //cout << "uKi = " << uKi << "''/s" << endl;
    Point2d uKd = Td * (deltaInPos[0]-deltaInPos[7]); 
    //cout << "uKd = " << uKd << "''/s" << endl;
    
    Point2d uPID = uKp + uKd;
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
        Point2d arcsecondsSpeedTemp;
        arcsecondsSpeedTemp.x = arcsecondsSpeedPredict.x + (0.01 * ((double)arcsecondsSpeedLimited.x - arcsecondsSpeedPredict.x));
        arcsecondsSpeedTemp.y = arcsecondsSpeedPredict.y + (0.01 * ((double)arcsecondsSpeedLimited.y - arcsecondsSpeedPredict.y));
        arcsecondsSpeedPredict.x = arcsecondsSpeedTemp.x;
        arcsecondsSpeedPredict.y = arcsecondsSpeedTemp.y;
        //cout << "Mean arcsecLim/s = " << arcsecondsSpeedPredict << "''/s" << endl;
        //cout << "Act  arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    else
    {
        arcsecondsSpeedLimited = arcsecondsSpeedPredict;
        //cout << "Predict arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    
    if( manualPos == false )
    {
        position->setVariableAzm(arcsecondsSpeedLimited.x);
        //cout << dec << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
        position->setVariableAlt(arcsecondsSpeedLimited.y);
        //cout << dec << "arcsecLim/s = " << arcsecondsSpeedLimited << "''/s" << endl;
    }
    
    if( manualPos2 == false )
    {   
        position2->setVariableAzm(arcsecondsSpeedLimited.x);
        position2->setVariableAlt(arcsecondsSpeedLimited.y); 
    }
        
    //arcsecondsSpeedLimitedOld = arcsecondsSpeedLimited;
    
    return;
}

int ObjectControl::processMsg()
{
    int ret = 0;
    
    //controlCycleTime();
    measureCycleTime();
    
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
        
        if( (pos = rec.rfind("sel=")) != string::npos )
        {
            string sub = rec.substr(pos+4);
            cout << "sel: " << (int)stoi(sub) << endl;
            selector = (int)stoi(sub);
        }
        else
        if( (pos = rec.rfind("rate=")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            cout << "rate: " << (int)stoi(sub) << endl;
            if( selector == 0 )
            {
                position->setFixedRate( (char)stoi(sub) );
            }
            else
            {
                position2->setFixedRate( (char)stoi(sub) );
            }
        }  
        else
        if( (pos = rec.rfind("alt=1")) != string::npos )
        {
            cout << "alt=1" << endl;
            //position->setFixedAlt( 1 );
            if( selector == 0 )
            {
                position->setFixedAlt( (int)position->getFixedRate() * CONTROL_FIXED_RATE );
                manualPos = true;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAlt( position2->getFixedRate() * CONTROL_FIXED_RATE );
                manualPos2 = true;
                //trackFlag = false;
            }
        }  
        else
        if( (pos = rec.rfind("alt=-1")) != string::npos )
        {
            cout << "alt=-1" << endl;
            //position->setFixedAlt( -1 );
            if( selector == 0 )
            {
                position->setFixedAlt( position->getFixedRate() * (-CONTROL_FIXED_RATE) );
                manualPos = true;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAlt( position2->getFixedRate() * (-CONTROL_FIXED_RATE) );
                manualPos2 = true;
                //trackFlag = false;
            }            
        }  
        else
        if( (pos = rec.rfind("azm=-1")) != string::npos )
        {
            cout << "azm=-1" << endl;
            //position->setFixedAzm( -1 );
            if( selector == 0 )
            {
                position->setFixedAzm( position->getFixedRate() * (-CONTROL_FIXED_RATE) ); 
                manualPos = true;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAzm( position2->getFixedRate() * (-CONTROL_FIXED_RATE) ); 
                manualPos2 = true;
                //trackFlag = false;
            }            
        }  
        else
        if( (pos = rec.rfind("azm=1")) != string::npos )
        {
            cout << "azm=1" << endl;
            //position->setFixedAzm( 1 );
            if( selector == 0 )
            {
                position->setFixedAzm( position->getFixedRate() * CONTROL_FIXED_RATE );   
                manualPos = true;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAzm( position2->getFixedRate() * CONTROL_FIXED_RATE );  
                manualPos2 = true;
                //trackFlag = false;
            }             
        }  
        else
        if( (pos = rec.rfind("alt=0")) != string::npos )
        {
            cout << "alt=0" << endl;
            //position->setFixedAlt( 0 );
            if( selector == 0 )
            {
                position->setFixedAlt( 0 );   
                manualPos = false;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAlt( 0 ); 
                manualPos2 = false;
                //trackFlag = false;
            }   
        }  
        else
        if( (pos = rec.rfind("azm=0")) != string::npos )
        {
            cout << "azm=0" << endl;
            //position->setFixedAzm( 0 );
            if( selector == 0 )
            {
                position->setFixedAzm( 0 );   
                manualPos = false;
                //trackFlag = false;
            }
            else
            {
                position2->setFixedAzm( 0 ); 
                manualPos2 = false;
                //trackFlag = false;
            }              
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

void ObjectControl::measureCycleTime()
{
   usleep(245000);

   clock_gettime(CLOCK_REALTIME, &end);
   dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) * 0.000000001;
   clock_gettime(CLOCK_REALTIME, &start);

   return;
}
