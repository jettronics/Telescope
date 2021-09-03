#ifndef CAMERA_H
#define CAMERA_H

using namespace std;
using namespace cv;

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
   Camera( TcpSocketCom *control, TcpSocketCom *stream, Focus *focus, Position *position );
   Camera( TcpSocketCom *control, TcpSocketCom *stream, ProcMessage *proc, Position *position );
   Camera( TcpSocketCom *control, TcpSocketCom *stream, ProcMessage *proc, ProcMessage *posmsg );
   virtual ~Camera(); 

public:
   int process();
    
private:
   int setControl( string prop );
   void processStreamMjpeg( Mat image );
   void initStreamMjpeg();
   void initRoi(Point2d pnt);
   string getDateAndTime();
   void stopVideoRecord();
   void sendFocus();
   void calcFocus();
   void changeZoom();
    
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
   Mat imageout, imagein, imageshot, imagetrack, imagefocus;
   string writeMjpegHeader;
   string writeMjpegContent;
   bool displayByWindow;
   bool enableTracker;
   bool initTracker;
   bool runTracker, runControl;
   bool recordVideo;
   Rect2d roi;
   Point2d roipt;
   Ptr<Tracker> tracker;
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
   //vector<double> osci1;
   //Ptr<plot::Plot2d> plot1;
};


#endif 
