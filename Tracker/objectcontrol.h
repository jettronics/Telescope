#ifndef OBJECTCONTROL_H
#define OBJECTCONTROL_H

using namespace std;
using namespace cv;

#include <libnova/lunar.h>
#include <libnova/mars.h>
#include <libnova/jupiter.h>
#include <libnova/saturn.h>
#include <libnova/mercury.h>
#include <libnova/julian_day.h>
#include <libnova/rise_set.h>
#include <libnova/transform.h>

#define DT_SAMPLES   8

class ObjectControl
{
   
public:
   ObjectControl();
   //ObjectControl(Position *position, Position *position2);
   ObjectControl(Position *position, Position *position2, ProcMessage *proc);
   virtual ~ObjectControl();

public:
   void init(double width, double height);
   void deInit();
   int processMsg();

private:
   Point2i speedLimit(Point2i speed);
   Point2i speedMax(Point2i speed);
   void measureCycleTime();
   void controlPosition();
   void controlPositionExt();
   void convertRaDec2AzmAlt();
   void calcRaDecFromSolarObj(string obj);
   void calcRaDecFromSpaceObj(string obj);
   void followPositionExt();
   
private:
   Position *position;
   Position *position2;
   ProcMessage *procMsg;
   Point2d ctrlPos;
   double Kp, Td;
   double width, height;
   Point2d arcsecondPerPixel;
   Point2i arcsecondsSpeedLimitedOld;
   int speedFieldOut[2];
   Point2d deltaInPos[DT_SAMPLES];
   bool initFlag;
   clock_t cycleTimeStart;
   bool trackFlag;
   Point2d inPos, inPosStart, inPosOld;
   Point2d speedObj;
   Point2d arcsecondsSpeedPredict, speedTelescope, speedObject, speedCentre;
   Point2d inArcDiff, inArcDiffOld, inDiffOld;
   bool predictCalc, arcToPixelMeasurement;
   bool manualPos;
   bool manualPos2;   
   int selector;
   double dt, arcToPixelTime, speedUpdateTime;
   double dtPos[DT_SAMPLES];
   struct timespec start, end;
   double normFactor;
   bool speedObjectMeasured;
   String solarObj;
   String spaceObj;
   ln_equ_posn objRaDec;
   ln_lnlat_posn location;
   ln_hrz_posn positionTelescope, positionTelescopeOffset;
   ln_hrz_posn positionAzmAlt, positionAzmAltPrev;
   bool followFlag;
   double followUpdateTime;
};


#endif
