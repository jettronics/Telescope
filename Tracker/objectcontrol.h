#ifndef OBJECTCONTROL_H
#define OBJECTCONTROL_H

using namespace std;
using namespace cv;

class ObjectControl
{
   
public:
   ObjectControl();
   ObjectControl(Position *position);
   ObjectControl(Position *position, ProcMessage *proc);
   virtual ~ObjectControl();

public:
   void init(double width, double height);
   void deInit();
   void process( Point2d inPos );
   int processMsg();

private:
   Point2i speedLimit(Point2i speed);
   void controlCycleTime();
   
private:
   Position *position;
   ProcMessage *procMsg;
   Point2d ctrlPos;
   double Kp, Ti, Td;
   double width, height;
   Point2d uKiOld;
   double arcsecondPerPixel;
   Point2i arcsecondsSpeedLimitedOld;
   int speedFieldOut[2];
   Point2d deltaInPos[8];
   bool initFlag;
   clock_t cycleTimeStart;
   bool trackFlag;
   Point2d inPosBuf;
};


#endif
