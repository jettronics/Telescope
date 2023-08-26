#ifndef OBJECTCONTROL_H
#define OBJECTCONTROL_H

using namespace std;
using namespace cv;

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
   void process();
   int processMsg();

private:
   Point2i speedLimit(Point2i speed);
   Point2i speedMax(Point2i speed);
   void measureCycleTime();
   void controlPosition();
   void controlSpeed();
   
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
   Point2d deltaInPos[8];
   bool initFlag;
   clock_t cycleTimeStart;
   bool trackFlag;
   Point2d inPos, inPosStart;
   Point2d speedObj;
   Point2d arcsecondsSpeedPredict, speedTelescope, speedObject;
   Point2d inArcDiff, inArcDiffOld;
   bool predictCalc, arcToPixelMeasurement;
   bool manualPos;
   bool manualPos2;   
   int selector;
   double dt, arcToPixelTime, speedUpdateTime;
   double dtPos[8];
   struct timespec start, end;
   double normFactor;
   bool speedObjectMeasured;
};


#endif
