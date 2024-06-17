#ifndef CAMERA_H
#define CAMERA_H

using namespace std;
using namespace cv;

#define MEDIAN_FILTER_SIZE  11

class CameraProperties
{
public:
   CameraProperties();
   virtual ~CameraProperties(); 


public:
   double widthVideo;
   double widthVideoOld;
   double heightVideo;
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
   struct DotTrackingType {int index; double distance; double area; Point2d pnt;};
    
private:
   int setControl( string prop );
   void processStreamMjpeg( Mat image );
   void initStreamMjpeg();
   void initRoi(Point2d pnt);
   string getDateAndTime();
   void stopVideoRecord();
   void dotDetection();
   void sendFocus();
   void calcFocus();
   void changeZoom();
   int medianFilter(int *medArr, int in);
    
private:
   CameraProperties camProps;
   raspicam::RaspiCam_Cv cam; 
   VideoWriter *writer;
   TcpSocketCom *control;
   TcpSocketCom *stream;
   Focus *focus;
   ProcMessage *procMsg;
   ProcMessage *posMsg;
   Position *position;
   bool videoMode;
   int photoStable;
   int cameraState;
   Mat imageout, imagein, imageshot, imagetrack, imagefocus, imagegray, imageproc;
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
   int medianInPosX[MEDIAN_FILTER_SIZE];
   int medianInPosY[MEDIAN_FILTER_SIZE];
   //vector<double> osci1;
   //Ptr<plot::Plot2d> plot1;
};


#endif 
