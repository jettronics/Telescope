
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "setup.h"
#include "procmessage.h"
#include "position.h"
#include "objectcontrol.h"


#define CONTROL_FIXED_RATE 20
#define CONTROL_EXP_RATE 1.9
#define SPEED_MAX_LIMIT 400


ObjectControl::ObjectControl()
{

}

/*
ObjectControl::ObjectControl(Position *position, Position *position2)
    : position(position)
    , position2(position2)
    , ctrlPos(0.0,0.0)
    , Kp(0.205)//Kp(0.4) Kp(0.5)
    , Td(0.5)
    , width(0)
    , height(0)
    , arcsecondPerPixel(0.0)
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(15,0)
    , predictCalc(false)
    , manualPos(false)
    , manualPos2(false)    
    , selector(0)
    , dt(0.25)
    , normFactor(1.0)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPos = Point2d(0.0,0.0);
    speedObj = Point2d(0,0);
    clock_gettime(CLOCK_REALTIME, &start);
}
*/

ObjectControl::ObjectControl(Position *position, Position *position2, ProcMessage *proc)
    : position(position)
    , position2(position2)    
    , procMsg(proc)
    , ctrlPos(0,0)
    , Kp(0.205)//Kp(0.4) Kp(0.5)
    , Td(0.5) //Object speed -> 8 entries every 0.25s = 2s -> 0.5 1/s
    , width(0)
    , height(0)
    , arcsecondPerPixel(4.1,3.8) //31.0,28.9
    , arcsecondsSpeedLimitedOld(0,0)
    , initFlag(true)
    , trackFlag(false)
    , arcsecondsSpeedPredict(0,0)
    , predictCalc(false)
    , arcToPixelMeasurement(false)
    , manualPos(false) 
    , manualPos2(false)        
    , selector(0)   
    , dt(0.25)
    , arcToPixelTime(0.0)
    , speedUpdateTime(0.0)
    , normFactor(1.0)
    , speedObjectMeasured(false)
{
    speedFieldOut[0] = 0;
    speedFieldOut[1] = 0;
    
    for( int i=0; i<8; i++ )
    {
        deltaInPos[i] = Point2d(0,0);
    }
    cycleTimeStart = clock();
    inPos = Point2d(0.0,0.0);
    inPosStart = Point2d(0.0,0.0);
    inPosOld = Point2d(0.0,0.0);
    speedObj = Point2d(0.0,0.0);
    inArcDiffOld = Point2d(0.0,0.0);
    inArcDiff = inArcDiffOld;
    inDiffOld = Point2d(0.0,0.0);;
    speedTelescope = Point2d(0.0,0.0);
    speedCentre = Point2d(0.0,0.0);
    speedObject = Point2d(0.0,0.0);
    clock_gettime(CLOCK_REALTIME, &start);
}


ObjectControl::~ObjectControl()
{

}

void ObjectControl::init(double width, double height)
{
    cout << "init = " << width << "x" << height << endl;
    
    ctrlPos.x = width * 0.5;
    ctrlPos.y = height * 0.5;
    
    inPos.x = width * 0.5;
    inPos.y = height * 0.5;
        
    inPosStart = inPos;
    inPosOld = inPos;
    
    speedUpdateTime = 0.0;
    inArcDiffOld = Point2d(0.0,0.0);
    speedTelescope = Point2d(0.0,0.0);
    speedCentre = Point2d(0.0,0.0);
    speedObject = Point2d(0.0,0.0);
    speedObjectMeasured = false;
    
    cout << "arcsecondPerPixel = " << arcsecondPerPixel << endl;
    cout << dec << "Init inPos = " << inPos << endl;
    
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

void ObjectControl::controlPosition()
{
	Point2d inPosArc = arcsecondPerPixel.y * inPos;
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
		double invTau = dt / (20.0 + dt);
		arcsecondsSpeedTemp.x = arcsecondsSpeedPredict.x + (invTau * ((double)arcsecondsSpeedLimited.x - arcsecondsSpeedPredict.x));
		arcsecondsSpeedTemp.y = arcsecondsSpeedPredict.y + (invTau * ((double)arcsecondsSpeedLimited.y - arcsecondsSpeedPredict.y));
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

	if( manualPos2 == false )
	{
		position2->setVariableAzm(arcsecondsSpeedLimited.x);
		position2->setVariableAlt(arcsecondsSpeedLimited.y);
	}

	//arcsecondsSpeedLimitedOld = arcsecondsSpeedLimited;

    return;
}

void ObjectControl::controlPositionExt()
{
    Point2d inPosNew;
    Point2d inDiff;
    
    if( initFlag == true )
    {
        initFlag = false;
        inArcDiffOld = Point2d(0.0,0.0);
        inArcDiff = Point2d(0.0,0.0);
        ctrlPos = inPos;
        inPosOld = inPos;
        inDiff = Point2d(0.0,0.0);
        inDiffOld = inDiff;
        speedTelescope = Point2d(0.0,0.0);
        speedObject = Point2d(0.0,0.0);
        speedCentre = Point2d(0.0,0.0);
        speedUpdateTime = 0.0;
        
        for( int i=0; i<8; i++ )
		{
			deltaInPos[i] = inArcDiff;
		}
		for( int i=0; i<8; i++ )
		{
			dtPos[i] = dt;
		}
        
        cout << "Tracking initialized" << endl;
    }
    else
    {
        if( (inPos.x < 1.0) && (inPos.y < 1.0) )
        {
            inPosNew = inPosOld;
        }
        else
        {
            //inPosNew = inPosOld + (0.5 * (inPos - inPosOld));
            //Try Median
            //inPosNew.x = medianFilter( medianInPosX, inPos.x);
            //inPosNew.y = medianFilter( medianInPosY, inPos.y);
            inPosNew = inPos;
        }
        if( manualPos2 == true )
        {
            ctrlPos = inPosNew;
        }
        inDiff = inPosNew - ctrlPos;
        
        Point2d diffObject = inPosNew - inPosOld;
        Point2d diffArcObject;
        diffArcObject.x = arcsecondPerPixel.x * diffObject.x;
        diffArcObject.y = arcsecondPerPixel.y * diffObject.y;
        
        double diffObjectAbs = (diffArcObject.x * diffArcObject.x) + (diffArcObject.y * diffArcObject.y);
        diffObjectAbs = sqrt(diffObjectAbs);
        
        double diffTelescopeMaxThres = 2.0 * ((double)SPEED_MAX_LIMIT * (double)SPEED_MAX_LIMIT);
        diffTelescopeMaxThres = sqrt(diffTelescopeMaxThres);
        diffTelescopeMaxThres *= dt;
        diffTelescopeMaxThres *= 6.0;
                
        if( diffObjectAbs > diffTelescopeMaxThres )
        {
            if( manualPos2 == false )        
            {
                cout << "Target jump: " << diffObjectAbs << ", diffTelescopeMaxThres: " << diffTelescopeMaxThres << endl;
                ctrlPos = inPos - inDiffOld;
                inDiff = inDiffOld;
            }
        }
                
        inArcDiff.x = arcsecondPerPixel.x * inDiff.x;
        inArcDiff.y = arcsecondPerPixel.y * inDiff.y;
        
        speedCentre = 0.1 * inArcDiff;
              
        inDiffOld = inDiff;
        inPosOld = inPosNew;
        
        
        double dtSum = dt;
        for( int i=0; i<(DT_SAMPLES-1); i++ )
        {
           dtPos[(DT_SAMPLES-1)-i] = dtPos[(DT_SAMPLES-2)-i];
           dtSum += dtPos[(DT_SAMPLES-2)-i];
        }
        dtPos[0] = dt;
        speedUpdateTime = dtSum;

        for( int i=0; i<(DT_SAMPLES-1); i++ )
        {
            deltaInPos[(DT_SAMPLES-1)-i] = deltaInPos[(DT_SAMPLES-2)-i];
        }
        deltaInPos[0] = inArcDiff;
        inArcDiffOld = deltaInPos[7];

        speedObject = (inArcDiff - inArcDiffOld)/speedUpdateTime;

#if 0     
        ----
        speedUpdateTime += dt;
        if( speedUpdateTime >= 2.0 ) //1.0
        {
            speedObject = (inArcDiff - inArcDiffOld)/speedUpdateTime;
            inArcDiffOld = inArcDiff;
            speedUpdateTime = 0.0;
            cout << "speedObject: " << speedObject << endl;
            cout << "inDiff: " << inDiff << endl;
            cout << "diffObjectAbs: " << diffObjectAbs << ", diffTelescopeMaxThres: " << diffTelescopeMaxThres << endl;
        }
#endif
        
        speedTelescope = speedCentre + speedObject;
        //cout << "Telescope speed update: " << speedTelescope << endl;
                
        Point2i arcsecondsSpeed = static_cast<Point2i>(speedTelescope);
        arcsecondsSpeed.y = -arcsecondsSpeed.y;
        
        Point2i arcsecondsSpeedLimited;
        arcsecondsSpeedLimited = speedMax( arcsecondsSpeed );

        if( manualPos2 == false )
        {
            position2->setVariableAzm(arcsecondsSpeedLimited.x);
            position2->setVariableAlt(arcsecondsSpeedLimited.y);
        }
    }

    return;
}


void ObjectControl::controlSpeed()
{
    Point2d inDiff = inPos - ctrlPos;
    
    // arcs to pixel measurement
    if( arcToPixelMeasurement == true )
    {
        speedTelescope.x = 150.0;
        double startMeasure = inPos.x - inPosStart.x;
        if( (fabs(inDiff.x) > 10.0) && (fabs(startMeasure) > 10.0) )
        {
            arcToPixelTime += dt;
        }
        else
        if( (fabs(inDiff.x) < 10.0) )
        {
            double arcToPixelDistance = (fabs(inPosStart.x)-10.0) - fabs(inPos.x);
            cout << dec << "arcs to pixel time = " << arcToPixelTime << "s" << endl;
            cout << dec << "arcs to pixel distance = " << arcToPixelDistance << "px" << endl;
            arcsecondPerPixel.x = (arcToPixelTime / arcToPixelDistance) * speedTelescope.x;
            cout << dec << "arc seconds per pixel = " << arcsecondPerPixel.x << "arcs/px" << endl;
            arcToPixelTime = 0.0;
            arcToPixelDistance = 0.0;
            arcToPixelMeasurement = false;
            speedTelescope = Point2d(0.0,0.0);
        } 
    }
    else
    {
        if( initFlag == true )
        {
            initFlag = false;
            inArcDiffOld = Point2d(0.0,0.0);
            ctrlPos = inPos;
            inDiff = Point2d(0.0,0.0);
            speedTelescope = Point2d(0.0,0.0);
            speedObject = Point2d(0.0,0.0);
            speedCentre = Point2d(0.0,0.0);
            speedUpdateTime = 0.0;
            speedObjectMeasured = false;
            cout << "Tracking initialized" << endl;
        }
        
        inArcDiff.x = arcsecondPerPixel.x * inDiff.x;
        inArcDiff.y = arcsecondPerPixel.y * inDiff.y;
        // Moving average
        //double invN = dt / (0.5 + dt);
        //inArcDiff =  inArcDiff + ((inArcDiffLoc - inArcDiff)*invN);
        //inArcDiff = inArcDiffLoc;
        
    	speedUpdateTime += dt;
        if( speedObjectMeasured == false )
        {
            Point2d distDiffPnt = inArcDiff - inArcDiffOld;
            double distDiffArc = (distDiffPnt.x * distDiffPnt.x) + (distDiffPnt.y * distDiffPnt.y);
            distDiffArc = sqrt(distDiffArc);
            //if( distDiffArc > 2500.0 ) //3000 -> 100px
            {
                speedObjectMeasured = true;
                speedObject = ((inArcDiff - inArcDiffOld)/speedUpdateTime);
                
                //Test only
                speedObject.x = 100.0;
                speedObject.y = 0.0;
                /*ctrlPos.x = 400.0;
                ctrlPos.y = 500.0;
                inDiff = inPos - ctrlPos;
                inArcDiff.x = arcsecondPerPixel.x * inDiff.x;
                inArcDiff.y = arcsecondPerPixel.y * inDiff.y;*/
                
                inArcDiffOld = inArcDiff;
                speedCentre.x = fmin(inArcDiffOld.x, (double)SPEED_MAX_LIMIT);
                speedCentre.y = fmin(inArcDiffOld.y, (double)SPEED_MAX_LIMIT);
                speedTelescope = speedObject + speedCentre;
                speedUpdateTime = 0.0;
                cout << "Object speed measured: " << speedObject << endl;
                cout << "Telescope speed: " << speedTelescope << endl;
            }
        }
        else
        {
            if( speedUpdateTime >= 5.0 ) //1.0
            {
                //Point2d realDiff = (inArcDiff - inArcDiffOld)/speedUpdateTime;
                Point2d inArcDiffGuess = inArcDiffOld + speedObject - speedTelescope - speedCentre;
                //speedObject = realDiff + (inArcDiffOld/8.0) + speedTelescope;
                
                //v_o_out(i) = (p_o_out(i) - p_o_out(i-1) + (v_t(i)*T))/T;
                //v_t(i+1) = v_o_out(i)+(p_o_out(i)/T_c);
                //speedObject = ((inArcDiff - inArcDiffOld)/speedUpdateTime);// + speedTelescope;
                //speedTelescope = speedObject + (inArcDiff/6.0); //5.0
                //inArcDiffOld = inArcDiff;
                speedUpdateTime = 0.0;
                inArcDiffOld = inArcDiff;
                speedCentre.x = fmin(inArcDiffOld.x, (double)SPEED_MAX_LIMIT);
                speedCentre.y = fmin(inArcDiffOld.y, (double)SPEED_MAX_LIMIT);
                cout << dec << "inArcDiff: " << inArcDiff << ", inArcDiffGuess: " << inArcDiffGuess << endl;
                cout << "Object speed update: " << speedObject << endl;
                
                Point2d speedTelescopeLoc = speedObject + speedCentre;
#if 0                
                if( (speedTelescopeLoc.x > 0.0) && (speedTelescope.x < 0.0) )
                {
                    speedTelescopeLoc.x = -5.0;
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
                else
                if( (speedTelescopeLoc.x < 0.0) && (speedTelescope.x > 0.0) )
                {
                    speedTelescopeLoc.x = 5.0;
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
                else
                if( fabs(speedTelescopeLoc.x) < 5.0 )
                {
                    if( speedTelescopeLoc.x > 0.0 )
                    {
                        speedTelescopeLoc.x = 5.0;
                    }
                    else
                    {
                        speedTelescopeLoc.x = -5.0;
                    } 
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
                
                if( (speedTelescopeLoc.y > 0.0) && (speedTelescope.y < 0.0) )
                {
                    speedTelescopeLoc.y = -5.0;
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
                else
                if( (speedTelescopeLoc.x < 0.0) && (speedTelescope.x > 0.0) )
                {
                    speedTelescopeLoc.y = 5.0;
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
                else
                if( fabs(speedTelescopeLoc.y) < 5.0 )
                {
                    if( speedTelescopeLoc.y > 0.0 )
                    {
                        speedTelescopeLoc.y = 5.0;
                    }
                    else
                    {
                        speedTelescopeLoc.y = -5.0;
                    } 
                    //cout << "Telescope speed limit: " << speedTelescopeLoc << endl;
                }
#endif                
                speedTelescope = speedTelescopeLoc;
                cout << "Telescope speed update: " << speedTelescope << endl;
            }
        }
    }
            
    Point2i arcsecondsSpeed = static_cast<Point2i>(speedTelescope);
	arcsecondsSpeed.y = -arcsecondsSpeed.y;
    
    Point2i arcsecondsSpeedLimited;
	arcsecondsSpeedLimited = speedMax( arcsecondsSpeed );

    if( manualPos2 == false )
	{
		position2->setVariableAzm(arcsecondsSpeedLimited.x);
		position2->setVariableAlt(arcsecondsSpeedLimited.y);
	}

    return;
}

void ObjectControl::process()
{
    controlPositionExt();
	return;
}

int ObjectControl::processMsg()
{
    int ret = 0;
    
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
                        inPos.x = (double)stod(width);
                        inPos.y = (double)stod(height);
                        //cout << "inPos: " << inPos << endl;
                        trackFlag = true;
                        predictCalc = false;
                    }
                }
            }
        }

        if( (pos = rec.rfind("rate=")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            cout << "rate: " << (int)stoi(sub) << endl;
            position2->setFixedRate( (char)stoi(sub) );
        } 
        
        if( (pos = rec.rfind("rateAzm=")) != string::npos )
        {
            string sub = rec.substr(pos+8);
            cout << "rateAzm: " << (int)stoi(sub) << endl;
            position2->setFixedRateAzm( (char)stoi(sub) );
        }   
        
        
        if( (pos = rec.rfind("rateAlt=")) != string::npos )
        {
            string sub = rec.substr(pos+8);
            cout << "rateAlt: " << (int)stoi(sub) << endl;
            position2->setFixedRateAlt( (char)stoi(sub) );
        }  
        
        if( (pos = rec.rfind("alt=1")) != string::npos )
        {
            cout << "alt=1" << endl;
            position2->setFixedAlt( pow(position2->getFixedRateAlt(), CONTROL_EXP_RATE) * CONTROL_FIXED_RATE );
            manualPos2 = true;
        }  
        else
        if( (pos = rec.rfind("alt=-1")) != string::npos )
        {
            cout << "alt=-1" << endl;
            position2->setFixedAlt( pow(position2->getFixedRateAlt(), CONTROL_EXP_RATE) * (-CONTROL_FIXED_RATE) );
            manualPos2 = true;
        }  
        
        if( (pos = rec.rfind("azm=-1")) != string::npos )
        {
            cout << "azm=-1" << endl;
            position2->setFixedAzm( pow(position2->getFixedRateAzm(), CONTROL_EXP_RATE) * (-CONTROL_FIXED_RATE) );
            manualPos2 = true;
        }  
        else
        if( (pos = rec.rfind("azm=1")) != string::npos )
        {
            cout << "azm=1" << endl;
            position2->setFixedAzm( pow(position2->getFixedRateAzm(), CONTROL_EXP_RATE) * CONTROL_FIXED_RATE );
            manualPos2 = true;
        }  
        
        if( (pos = rec.rfind("alt=0")) != string::npos )
        {
            cout << "alt=0" << endl;
            position2->setFixedAlt( 0 );
            position2->setFixedRateAlt( 1 );
            manualPos2 = false;
        }  
        
        if( (pos = rec.rfind("azm=0")) != string::npos )
        {
            cout << "azm=0" << endl;
            position2->setFixedAzm( 0 );
            position2->setFixedRateAzm( 1 );
            manualPos2 = false;
        }  
        
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
    if( abs(speed.x) > SPEED_MAX_LIMIT )
    {
        if( speed.x < 0 ) 
        {
            retSpeed.x = -SPEED_MAX_LIMIT;
        }
        else
        {
            retSpeed.x = SPEED_MAX_LIMIT;
        }
    }
    if( abs(speed.y) > SPEED_MAX_LIMIT )
    {
        if( speed.y < 0 ) 
        {
            retSpeed.y = -SPEED_MAX_LIMIT;
        }
        else
        {
            retSpeed.y = SPEED_MAX_LIMIT;
        }
    }
        
    return retSpeed;
}

void ObjectControl::measureCycleTime()
{
   usleep(245000);

   clock_gettime(CLOCK_REALTIME, &end);
   dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) * 0.000000001;
   clock_gettime(CLOCK_REALTIME, &start);

   return;
}
