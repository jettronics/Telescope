#ifndef CAMERA_H
#define CAMERA_H

using namespace std;
using namespace cv;

#define MEDIAN_FILTER_SIZE  5 //7 //9 //11

class CameraProperties
{
public:
   CameraProperties();
   virtual ~CameraProperties(); 


public:
   double widthVideo;
   double widthVideoOld;
   double heightVideo;
   double heightVideoOld;
   double widthImage;
   double heightImage;
   double brightness;
   double contrast;
   double saturation;
   double gain;
   double exposure;
   double fpsVideo;
   double fpsImage;
 
};


class Camera
{
public:
   Camera();
   //Camera( TcpSocketCom *control, TcpSocketCom *stream, Focus *focus, Position *position );
   //Camera( TcpSocketCom *control, TcpSocketCom *stream, ProcMessage *proc, Position *position );
   Camera( TcpSocketCom *control, TcpSocketCom *stream, ProcMessage *proc, ProcMessage *posmsg );
   virtual ~Camera(); 

public:
   int process();
   
private:
   struct DotTrackingType {int index; double distance; double area; Point2d pnt; Point2d ctrl;};
    
private:
   int setControl( string prop );
   void processStreamMjpeg( Mat image );
   void initStreamMjpeg();
   void initRoi(Point2d pnt);
   string getDateAndTime();
   void stopVideoRecord();
   void dotDetection();
   void objectFlowbySubPixels();
   void sendFocus();
   void calcFocus();
   void changeZoom();
   float medianFilter(float *medArr, float in);
   int handleJoystickEvents(string *msgEvents);
    
private:
   CameraProperties camProps;
   raspicam::RaspiCam_Cv cam; 
   VideoWriter *writer;
   TcpSocketCom *control;
   TcpSocketCom *stream;
   ProcMessage *procMsg;
   ProcMessage *posMsg;
   Position *position;
   bool videoMode;
   int photoStable;
   int cameraState;
   Mat imageout, imagein, imageshot, imagetrack, imagefocus, imagegray, imageproc, imageprev, imagemorph, imagefloat;
   string writeMjpegHeader;
   string writeMjpegContent;
   bool displayByWindow;
   bool enableTracker, initTracker, runTracker, runControl, dotTracker, dotFound;
   bool recordVideo;
   Rect2d roi;
   Point2d roipt;
   Ptr<Tracker> tracker;
   vector<vector<Point>> contours, contoursLoc;
   ObjectControl *objectControl;
   Scalar roiColor;
   bool enAutoFocus; 
   double meanFocus;
   double drawScale;
   Scalar focusColor;
   int focusPos;
   Rect2d zoom;
   double zoomFactor;
   double focusLineLength;
   double roiSize;
   DotTrackingType dotTracking;
   float medianInPosX[MEDIAN_FILTER_SIZE];
   float medianInPosY[MEDIAN_FILTER_SIZE];
   int joystHndl;
   struct js_event joystEvent;
   int joystVertState, joystHorState;
   int joystPosSpeedY, joystPosSpeedX;
   int joystButtonXState;
   int joystButtonSelectState;
   bool initObjectFlow;
   bool manCentreCtrl;
   //vector<double> osci1;
   //Ptr<plot::Plot2d> plot1;
};


#endif 
