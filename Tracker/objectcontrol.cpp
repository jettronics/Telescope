
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
    , arcsecondPerPixel(3.85, 3.85) //31.0,28.9
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
    , followFlag(false)
    , followUpdateTime(0.0)
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
    solarObj = "";
    spaceObj = "";
    objRaDec.ra = 0.0;
    objRaDec.dec = 0.0;
    location.lat = 0.0; /* N */
	location.lng = 0.0; /* E */
    orientation.alt = 0.0;
    orientation.az = 0.0;
    positionAzmAlt.alt = 0.0;
    positionAzmAlt.az = 0.0;
    positionAzmAltPrev.alt = 0.0;
    positionAzmAltPrev.az = 0.0;
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
    
    //initFlag = true;
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
#if 0        
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
#endif                
        inArcDiff.x = arcsecondPerPixel.x * inDiff.x;
        inArcDiff.y = arcsecondPerPixel.y * inDiff.y;
        
        //speedCentre = 0.1 * inArcDiff;
        speedCentre = 0.2 * inArcDiff;
              
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

void ObjectControl::followPositionExt()
{
    
    if( followUpdateTime >= 2.0 )
    {
        convertRaDec2AzmAlt();
        double deltaAzm = positionAzmAlt.az - positionAzmAltPrev.az;
        double deltaAlt = positionAzmAlt.alt - positionAzmAltPrev.alt;
        cout << "Delta Azm: " << deltaAzm << ", Alt: " << deltaAlt << endl;
        
        orientation.az = positionAzmAlt.az;
        orientation.alt = positionAzmAlt.alt;
        
        cout << "Orientation: azm=" << orientation.az << ", alt=" << orientation.alt << endl;
        
        double arcsecsAzmD = deltaAzm * (double)3600.0/followUpdateTime;
        double arcsecsAltD = deltaAlt * (double)3600.0/followUpdateTime;
        int arsecsAzm = (int)((arcsecsAzmD)+(double)0.5);
        int arsecsAlt = (int)((arcsecsAltD)+(double)0.5);
        if( arcsecsAzmD < 0.0 )
        {
            arsecsAzm = (int)((arcsecsAzmD)-(double)0.5);
        }
        if( arcsecsAltD < 0.0 )
        {
            arsecsAlt = (int)((arcsecsAltD)-(double)0.5);
        }
        
        positionAzmAltPrev.alt = positionAzmAlt.alt;
        positionAzmAltPrev.az = positionAzmAlt.az;
        followUpdateTime = 0.0;
        
        Point2i arcsecondsSpeed;
        arcsecondsSpeed.x = (int)(arsecsAzm + (double)0.5);
        arcsecondsSpeed.y = (int)(arsecsAlt + (double)0.5);
        
        cout << "arcsec/s Azm: " << arcsecondsSpeed.x << ", Alt: " << arcsecondsSpeed.y << endl;
        
        if( manualPos2 == false )
        {
            position2->setVariableAzm(arcsecondsSpeed.x);
            position2->setVariableAlt(arcsecondsSpeed.y);
        }
        
    }
    
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
                        int len = 0;
                        len = xchar - (startchar+1);
                        string width = sub.substr(startchar+1, len); 
                        len = endchar - (xchar+1);
                        string height = sub.substr(xchar+1, len); 
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
                        int len = 0;
                        len = xchar - (startchar+1);
                        string width = sub.substr(startchar+1, len); 
                        len = endchar - (xchar+1);
                        string height = sub.substr(xchar+1, len); 
                        inPos.x = (double)stod(width);
                        inPos.y = (double)stod(height);
                        //cout << "inPos: " << inPos << endl;
                        if( trackFlag == false )
                        {
                            initFlag = true;
                            trackFlag = true;
                            predictCalc = false;
                        }
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
        
        if( (pos = rec.rfind("Goto")) != string::npos )
        {
            if( (pos = rec.rfind("GotoOrAzm=")) != string::npos )
            {
                string sub = rec.substr(pos+10);
                double azm = (double)stod(sub);
                cout << "GotoOrAzm: " << azm << endl;
                // SkyView app 0° = North, 180° = South, -90° = West,  90° = East
                // Libnova   180° = North,   0° = South,  90° = West, 270° = East
                /*double conv = azm + 180.0;
                if( conv >= 360.0 )
                {
                    conv = 0.0;
                }*/
                
                // AltAz app   0° = North, 180° = South, 270° = West,  90° = East
                // Libnova   180° = North,   0° = South,  90° = West, 270° = East
                double conv = 0.0;
                if( (azm >= 0.0) && (azm < 180.0) )
                {
                    conv = azm + 180.0;
                }
                else
                if( (azm >= 180.0) && (azm < 360.0) )
                {
                    conv = azm - 180.0;
                }
                orientation.az = conv;
                //telscpoffset.az = conv;
                cout << "orientation.az: " << orientation.az << endl;
            }   
            else
            if( (pos = rec.rfind("GotoOrAlt=")) != string::npos )
            {
                string sub = rec.substr(pos+10);
                double alt = (double)stod(sub);
                cout << "GotoOrAlt: " << alt << endl;
                orientation.alt = alt;
                //telscpoffset.alt = alt;
            }  
            else
            if( (pos = rec.rfind("GotoObj1=")) != string::npos )
            {
                size_t endchar = rec.find(';');
                if( endchar != string::npos )
                {
                    int len = endchar - (pos+9);
                    string sub = rec.substr(pos+9, len);
                    cout << "GotoObj1: " << sub << endl;
                    calcRaDecFromSolarObj(sub);
                    convertRaDec2AzmAlt();
                    followUpdateTime = 0.0;
                }
            } 
            else
            if( (pos = rec.rfind("GotoObj2=")) != string::npos )
            {
                size_t endchar = rec.find(';');
                if( endchar != string::npos )
                {
                    int len = endchar - (pos+9);
                    string sub = rec.substr(pos+9, len);
                    cout << "GotoObj2: " << sub << endl;
                    calcRaDecFromSpaceObj(sub);
                    convertRaDec2AzmAlt();
                    followUpdateTime = 0.0;
                }
            }
            else
            if( (pos = rec.rfind("GotoState=start")) != string::npos )
            {
                cout << "GotoState: start" << endl;
                positionAzmAltPrev.alt = positionAzmAlt.alt;
                positionAzmAltPrev.az = positionAzmAlt.az;
                
                double altDelta = positionAzmAlt.alt - orientation.alt;
                double azmDelta = positionAzmAlt.az - orientation.az;
                
                orientation.az = positionAzmAlt.az;
                orientation.alt = positionAzmAlt.alt;
                cout << "New orientation: azm=" << orientation.az << ", alt=" << orientation.alt << endl;
                
                // To Do: Calculate Telescope Azm/Alt coordinates
                                
                position2->setGotoAzmAlt( azmDelta, altDelta );
                //followUpdateTime = 0.0;
                followFlag = false;
            }
            else
            if( (pos = rec.rfind("GotoState=follow")) != string::npos )
            {
                cout << "GotoState: follow" << endl;
                //followUpdateTime = 0.0;
                followFlag = true;
            }
            else
            if( (pos = rec.rfind("GotoState=stop")) != string::npos )
            {
                cout << "GotoState: stop" << endl;
                position2->setFixedAlt( 0 );
                position2->setFixedRateAlt( 1 );
                position2->setFixedAzm( 0 );
                position2->setFixedRateAzm( 1 );
                manualPos2 = false;
                followFlag = false;
            }
        } 
        
        if( (pos = rec.rfind("LocN=")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            double north = (double)stod(sub);
            cout << "LocN: " << north << endl;
            location.lat = north;
        }   
        
        if( (pos = rec.rfind("LocE=")) != string::npos )
        {
            string sub = rec.substr(pos+5);
            double east = (double)stod(sub);
            cout << "LocE: " << east << endl;
            location.lng = east;
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
        controlPositionExt();
    }
    followUpdateTime += dt;
    if( followFlag == true )
    {
        followPositionExt();
    }    
    
    return ret;
}


void ObjectControl::calcRaDecFromSolarObj(string obj)
{
    double JD;
	
    cout << "calcRaDecFromSolarObj: " << obj << endl;
     
	/* get the julian day from the local system time */
	JD = ln_get_julian_from_sys();
	cout << "Julian day: " << JD << endl;
	
	if (obj == "Moon")
    {
        ln_get_lunar_equ_coords(JD, &objRaDec);
    }
    else
    if (obj == "Mars") 
    {
        ln_get_mars_equ_coords(JD, &objRaDec);
    }
    else
    if (obj == "Jupiter")
    {
        ln_get_jupiter_equ_coords(JD, &objRaDec); 
    }
    else
    if (obj == "Saturn")
    {
        ln_get_saturn_equ_coords(JD, &objRaDec);
    }
    else
    if (obj == "Merkur")
    {
        ln_get_mercury_equ_coords(JD, &objRaDec);
    }
    else
    {
        objRaDec.ra = 0.0;
        objRaDec.dec = 0.0;
    }
    
    cout << "Solar object Ra: " << objRaDec.ra << ", Dec: " << objRaDec.dec << endl;
    
    return;
}

void ObjectControl::calcRaDecFromSpaceObj(string obj)
{
    size_t midchar = obj.find(',');
    if( midchar != string::npos )
    {
        int len = 0;
        len = midchar;
        string Ra = obj.substr(0, len); 
        len = obj.length() - (midchar+1);
        string Dec = obj.substr(midchar+1, len); 
        objRaDec.ra = (double)stod(Ra);
        objRaDec.dec = (double)stod(Dec);
        
        cout << "Space object Ra: " << objRaDec.ra << ", Dec: " << objRaDec.dec << endl;
    }
    return;
}

void ObjectControl::convertRaDec2AzmAlt()
{
    double JD;
	
    cout << "convertRaDec2AzmAlt: " << endl;
     
	/* get the julian day from the local system time */
	JD = ln_get_julian_from_sys();
    
    ln_get_hrz_from_equ(&objRaDec, &location, JD, &positionAzmAlt);
    
    cout << "Object Azm: " << positionAzmAlt.az << ", Alt: " << positionAzmAlt.alt << endl;

    return;
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
