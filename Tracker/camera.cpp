

#include <ctime>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/plot.hpp>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <linux/joystick.h>

#include "tcpsocketcom.h"
#include "procmessage.h"
#include "position.h"
#include "objectcontrol.h"
#include "camera.h"

#define NOT_SHOW_CAMERA_WINDOW
#define OPENCV_WAIT_FOR_KEY 20
#define STABLE_PHOTO_SHOT (100/OPENCV_WAIT_FOR_KEY)
#define ROI_WIDTH_RATIO 0.8 //0.15
//#define CONTROL_CYCLE_TIME 0.25
#define TRACKING_METHOD "KCF"
//#define TRACKING_METHOD "MIL"


CameraProperties::CameraProperties()
    : widthVideo(1280) //1441 
    , widthVideoOld(1280)
    , heightVideo(960) //1080
    , heightVideoOld(960) //1080
    , widthImage(4056)
    , heightImage(3040)
    , brightness(50) //[0,100]
    , contrast(50) //[0,100]
    , saturation(50) //[0,100]
    , gain(50) //[0,100]
    , exposure(-1) //[1,100] shutter speed from 0 to 33ms
    , fpsVideo(20)
    , fpsImage(3)
{
    
}

CameraProperties::~CameraProperties()
{
}

Camera::Camera()
    : writer(nullptr)
    , control(nullptr)
    , stream(nullptr)
    , procMsg(nullptr)
    , videoMode(true)
    , photoStable(STABLE_PHOTO_SHOT)
    , cameraState(0)
    , displayByWindow(false)
    , enableTracker(false)
    , initTracker(false)
    , runTracker(false)
    , runControl(false)
    , dotTracker(true)
    , dotFound(false)
    , recordVideo(false)
    , roi(0,0,0,0)
    , roipt(0,0)
    , roiColor(Scalar(255,255,255))
    , enAutoFocus(false)   
    , meanFocus(0.0) 
    , drawScale(1.0)
    , focusColor(Scalar(0,165,255))    
    , focusPos(30)
    , zoom(0,0,0,0)  
    , zoomFactor(1)  
    , focusLineLength(0.0)
    , roiSize(ROI_WIDTH_RATIO)
    , joystHndl(-1)
    , joystVertState(0)
    , joystHorState(0)
    , joystPosSpeedY(1)
    , joystPosSpeedX(1)
    , joystButtonXState(0)
    , joystButtonSelectState(0)
{
    objectControl = new ObjectControl;
    dotTracking.area = 0.0;
    dotTracking.distance = 0.0;
    dotTracking.index = 0;
    dotTracking.pnt = Point2d(0.0,0.0);
}

Camera::Camera( TcpSocketCom *control, TcpSocketCom *stream, ProcMessage *proc, ProcMessage *posmsg )
    : writer(nullptr)
    , control(control)
    , stream(stream)
    , procMsg(proc)
    , posMsg(posmsg)
    , position(nullptr)
    , videoMode(true)
    , photoStable(STABLE_PHOTO_SHOT)
    , cameraState(0)
    , displayByWindow(false)
    , enableTracker(false)
    , initTracker(false)
    , runTracker(false)
    , runControl(false)
    , dotTracker(true)
    , dotFound(false)
    , recordVideo(false)
    , roi(0,0,0,0)
    , roipt(0,0)
    , roiColor(Scalar(255,255,255))
    , enAutoFocus(false)     
    , meanFocus(0.0)  
    , drawScale(1.0)   
    , focusColor(Scalar(0,165,255))  
    , focusPos(30)   
    , zoom(0,0,0,0)   
    , zoomFactor(1)  
    , focusLineLength(0.0)
    , roiSize(ROI_WIDTH_RATIO)  
    , joystHndl(-1)
    , joystVertState(0)
    , joystHorState(0)
    , joystPosSpeedY(1)
    , joystPosSpeedX(1)
    , joystButtonXState(0)
    , joystButtonSelectState(0)
    , initObjectFlow(true)
{
    objectControl = nullptr;
    dotTracking.area = 0.0;
    dotTracking.distance = 0.0;
    dotTracking.index = 0;
    dotTracking.pnt = Point2d(0.0,0.0);

/*        
    osci1.clear();

    for( int t = 0; t < 400; t++ )
    { 
        osci1.push_back(0);
    }

    namedWindow( "Plot1", CV_WINDOW_NORMAL );

    Mat data1( osci1 );
    plot1 = plot::createPlot2d( data1 );

    plot1->setMaxY(200);
    plot1->setMinY(-5);
*/

}

Camera::~Camera()
{
    cout << "Delete objectControl" << endl;
    if( objectControl != nullptr )
    {
        delete objectControl;
    }
    if( writer != nullptr )
    {
        delete writer;
    }
}

int Camera::handleJoystickEvents(string *msgEvents)
{
    int ret = -1;
    
    if( joystHndl >= 0 )
    {
        ssize_t bytes;

        bytes = read(joystHndl, &joystEvent, sizeof(joystEvent));

        if (bytes == sizeof(joystEvent))
        {
            ret = 0;
            if( joystEvent.type == JS_EVENT_BUTTON )
            {
                //cout << "Joystick Button: " << (int)joystEvent.number << ", State: " << joystEvent.value << endl;
                if( ((int)joystEvent.number == 0) && ((int)joystEvent.value != 0) ) 
                {
                    // Button X -> ROI on/off
                    if( joystButtonXState == 0 )
                    {
                        joystButtonXState = 1;
                        string posStr = "Tracker=init;";
                        (*msgEvents) += posStr;
                    }
                    else
                    {
                        joystButtonXState = 0;
                        string posStr = "Tracker=deinit;";
                        (*msgEvents) += posStr;
                    }
                    cout << "Button X -> ROI state: " << joystButtonXState << endl;
                }
                else
                if( ((int)joystEvent.number == 1) && ((int)joystEvent.value != 0) ) 
                {
                    // Button A -> ROI size up
                    string posStr = "Position=roiup;";
                    (*msgEvents) += posStr;
                    cout << "Button A -> ROI size up" << endl;
                }
                else
                if( ((int)joystEvent.number == 2) && ((int)joystEvent.value != 0) ) 
                {
                    // Button B -> ROI size down
                    string posStr = "Position=roidn;";
                    (*msgEvents) += posStr;
                    cout << "Button B -> ROI size down" << endl;
                }
                else
                if( ((int)joystEvent.number == 3) && ((int)joystEvent.value != 0) ) 
                {
                    // Button Y -> Stop all axis
                    joystHorState = 0;
                    joystVertState = 0;
                    joystPosSpeedX = 1;
                    joystPosSpeedY = 1;
                    string pos1Str = "Position=stoplr;";
                    (*msgEvents) += pos1Str;
                    string pos2Str = "Position=stopud;";
                    (*msgEvents) += pos2Str;
                    cout << "Position Horizontal and Vertical stop" << endl;
                }
                else
                if( ((int)joystEvent.number == 8) && ((int)joystEvent.value != 0) ) 
                {
                    // Button Select -> Find Object
                    if( joystButtonSelectState == 0 )
                    {
                        joystButtonSelectState = 1;
                        string posStr = "Tracker=run;";
                        (*msgEvents) += posStr;
                    }
                    else
                    {
                        joystButtonSelectState = 0;
                        string posStr = "Tracker=stop;";
                        (*msgEvents) += posStr;
                    }
                    
                    cout << "Button Select -> Object state: " << joystButtonSelectState << endl;
                }
                else
                if( ((int)joystEvent.number == 9) && ((int)joystEvent.value != 0) ) 
                {
                    // Button Start -> Tracking Start/Stop
                    string posStr = "Tracker=ctrl;";
                    (*msgEvents) += posStr;
                    
                    cout << "Button Start -> Tracker control" << endl;
                }
            }
            else
            if( joystEvent.type == JS_EVENT_AXIS )
            {
                //cout << "Joystick Axis: " << (int)joystEvent.number << ", State: " << joystEvent.value << endl;
                if( ((int)joystEvent.number == 0) && ((int)joystEvent.value != 0) )
                {
                    if( joystHorState != 0 )
                    {
                        // Increase Speed
                        joystPosSpeedX++;
                        if( joystPosSpeedX > 15 )
                        {
                            joystPosSpeedX = 15;
                        }
                        string posStr = "PositionspeedX=" + to_string(joystPosSpeedX) + ";";
                        (*msgEvents) += posStr;
                    }
                                        
                    // Left, Right
                    if( joystEvent.value > 0 )
                    {
                        if( (joystHorState != joystEvent.value) && (joystHorState != 0) )
                        {
                            // Stop
                            joystHorState = 0;
                            string posStr = "Position=stoplr;";
                            (*msgEvents) += posStr;
                            cout << "Position Horizontal stop" << endl;
                        }
                        else
                        {
                            // Right
                            joystHorState = joystEvent.value;
                            string posStr = "Position=right;";
                            (*msgEvents) += posStr;
                            cout << "Position right" << endl;
                        }
                    }
                    else
                    if( joystEvent.value < 0 )
                    {
                        if( (joystHorState != joystEvent.value) && (joystHorState != 0) )
                        {
                            // Stop
                            joystHorState = 0;
                            string posStr = "Position=stoplr;";
                            (*msgEvents) += posStr;
                            cout << "Position Horizontal stop" << endl;
                        }
                        else
                        {
                            // Left
                            joystHorState = joystEvent.value;
                            string posStr = "Position=left;";
                            (*msgEvents) += posStr;
                            cout << "Position left" << endl;
                        }
                    }
                    
                    if( joystHorState == 0 )
                    {
                        joystPosSpeedX = 1;
                        string posStr = "PositionspeedX=" + to_string(joystPosSpeedX) + ";";
                        (*msgEvents) += posStr;
                    }
                    
                    cout << "Horizontal speed: " << joystPosSpeedX << endl;
                }
                else
                if( ((int)joystEvent.number == 1) && ((int)joystEvent.value != 0) )
                {
                    if( joystVertState != 0 )
                    {
                        // Increase Speed
                        joystPosSpeedY++;
                        if( joystPosSpeedY > 15 )
                        {
                            joystPosSpeedY = 15;
                        }
                        string posStr = "PositionspeedY=" + to_string(joystPosSpeedY) + ";";
                        (*msgEvents) += posStr;
                    }
                    
                    // Up, Down
                    if( joystEvent.value > 0 )
                    {
                        if( (joystVertState != joystEvent.value) && (joystVertState != 0) )
                        {
                            // Stop
                            joystVertState = 0;
                            string posStr = "Position=stopud;";
                            (*msgEvents) += posStr;
                            cout << "Position Vertical stop" << endl;
                        }
                        else
                        {
                            // Down
                            joystVertState = joystEvent.value;
                            string posStr = "Position=down;";
                            (*msgEvents) += posStr;
                            cout << "Position down" << endl;
                        }
                    }
                    else
                    if( joystEvent.value < 0 )
                    {
                        if( (joystVertState != joystEvent.value) && (joystVertState != 0) )
                        {
                            // Stop
                            joystVertState = 0;
                            string posStr = "Position=stopud;";
                            (*msgEvents) += posStr;
                            cout << "Position Vertical stop" << endl;
                        }
                        else
                        {
                            // Up
                            joystVertState = joystEvent.value;
                            string posStr = "Position=up;";
                            (*msgEvents) += posStr;
                            cout << "Position up" << endl;
                        }
                    }
                    
                    if( joystVertState == 0 )
                    {
                        joystPosSpeedY = 1;
                        string posStr = "PositionspeedY=" + to_string(joystPosSpeedY) + ";";
                        (*msgEvents) += posStr;
                    }
                    
                    cout << "Vertical speed: " << joystPosSpeedY << endl;
                }
            }
        }
    }
    
    return ret;
}


int Camera::process( void )
{
    int ret = 0;
    
    if( cameraState == 0 )
    { 
        cout << "Initializing ..." << endl;
        cam.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
        if( videoMode == true )
        {
            cam.set( CV_CAP_PROP_FRAME_WIDTH, camProps.widthVideo );
            cam.set( CV_CAP_PROP_FRAME_HEIGHT, camProps.heightVideo );
            cam.set( CV_CAP_PROP_FPS, camProps.fpsVideo );
        }
        else
        {
            cam.set( CV_CAP_PROP_FRAME_WIDTH, camProps.widthImage );
            cam.set( CV_CAP_PROP_FRAME_HEIGHT, camProps.heightImage );
            cam.set( CV_CAP_PROP_FPS, camProps.fpsImage );
        }
        cam.set( CV_CAP_PROP_BRIGHTNESS, camProps.brightness );
        cam.set( CV_CAP_PROP_CONTRAST, camProps.contrast );
        cam.set( CV_CAP_PROP_SATURATION, camProps.saturation );
	 	cam.set( CV_CAP_PROP_GAIN, camProps.gain );
        cam.set( CV_CAP_PROP_EXPOSURE, camProps.exposure );
        
        joystHndl = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        cout << "Open Joystick: " << joystHndl << endl;
        joystVertState = 0;
        joystHorState = 0;
        joystPosSpeedX = 1;
        joystPosSpeedY = 1;
        joystButtonXState = 0;
        joystButtonSelectState = 0;
        
        if( videoMode == true )
        {
            if( runTracker == false )
            {
                cout<<"Tracker init"<<endl;
                cout<<"ROI init"<<endl;
                roipt.x = camProps.widthVideo / 2;
                roipt.y = camProps.heightVideo / 2;
                dotTracking.pnt = roipt;
                roiColor = Scalar(255,255,255);
            }
            else
            {
                cout<<"ROI update"<<endl;
                // Calculate roi based on central point
                roipt.x = roipt.x * camProps.widthVideo / camProps.widthVideoOld;
                roipt.y = roipt.y * camProps.heightVideo / camProps.heightVideoOld;
                dotTracking.pnt = roipt;
                runTracker = false;
                initTracker = true;
                roiColor = Scalar(0,255,0);
            }
            if( runControl == true )
            {
                //objectControl->init( camProps.widthVideo, camProps.heightVideo );
                string strSend = "init=" + to_string((int)camProps.widthVideo) + "x" + 
                                 to_string((int)camProps.heightVideo) + ";";
                posMsg->sendClientToServer(strSend);
            }
            drawScale = camProps.widthVideo / 640.0;
            focusPos = 30*drawScale;
            meanFocus = 0.0;
            initRoi(roipt);
            changeZoom();
            tracker = Tracker::create( TRACKING_METHOD );
            cout<<"ROI Point x: " << roipt.x << ", y: " << roipt.y << endl;
        }
        
        camProps.widthVideoOld = camProps.widthVideo;
        camProps.heightVideoOld = camProps.heightVideo;
        
        VideoCapture cap(CV_CAP_ANY);
                
        if( cam.open() )
        {
            cout<<"Camera opended"<<endl;
            if( displayByWindow == true )
            {
                //#ifdef SHOW_CAMERA_WINDOW
                namedWindow( "Camera input", CV_WINDOW_NORMAL );
                //#endif
            }
            else
            {
                initStreamMjpeg();
            }
            cameraState = 1;
        }
        else
        {
            cout<<"Camera couldn't be opended"<<endl;
            cam.release();
            cameraState = 0;
            ret = -1;
        }
    }
    else
    if( cameraState == 1 )
    {
        char c = 0;
        int retCtrl = 0;
                
        cam.grab();
        cam.retrieve(imagein);
        // Mirror image y-axis
        //flip(imagein, imageout, -1);
        imageout = imagein.clone();
        //bitwise_not(imageout, imageout);
        
        imagetrack = imageout.clone();
        //imageproc = imagetrack.clone();
        
        if( videoMode == true )
        {
            if( initTracker == true )
            {
                cout << "Initialize Tracker" << endl;
                if( dotTracker == false )
                {
                    tracker->init( imagetrack, roi );
                }
                runTracker = true;
                initTracker = false;
                roiColor = Scalar(0,255,0);
                roipt.x = ((int)roi.width >> 1) + roi.x;
                roipt.y = ((int)roi.height >> 1) + roi.y; 
                dotFound = false;
                cout << "Run Tracker" << endl;
            }
            if( runTracker == true )
            {
                if( ((roi.x > 20) && ((roi.x + roi.width) < (camProps.widthVideo - 20))) &&
                    ((roi.y > 20) && ((roi.y + roi.height) < (camProps.heightVideo - 20))) )
                {
                    if( dotTracker == false )
                    {   
                        tracker->update( imagetrack, roi );
                        roipt.x = ((int)roi.width >> 1) + roi.x;
                        roipt.y = ((int)roi.height >> 1) + roi.y;
                    }
                    else
                    {
                        //dotDetection();
                        objectFlowbySubPixels();
                    }
                }
                else
                {
                    initTracker = false;
                    runTracker = false;
                    if( runControl == true )
                    {
                        posMsg->sendClientToServer("deInit");
                    }
                    runControl = false;
                    roipt.x = camProps.widthVideo / 2;
                    roipt.y = camProps.heightVideo / 2;
                    initRoi(roipt);
                    roiColor = Scalar(255,255,255);
                }
            }
            else
            {
                initObjectFlow = true;
            }
            
            if( runControl == true )
            {
                string strSend, strx, stry;
                if( dotTracker == false )
                {
                    // Call deinit after roipt reached closed to mid of image and start prediction
                    stringstream streamx, streamy;
                    streamx << fixed << setprecision(2) << roipt.x;
                    strx = streamx.str(); 
                    streamy << fixed << setprecision(2) << roipt.y;
                    stry = streamy.str(); 
                }
                else
                {
                    stringstream streamx, streamy;
                    streamx << fixed << setprecision(2) << dotTracking.pnt.x;
                    strx = streamx.str(); 
                    streamy << fixed << setprecision(2) << dotTracking.pnt.y;
                    stry = streamy.str(); 
                }
                strSend = "roipt=" + strx + "x" + stry + ";";
                //cout << "strSend: " << strSend << endl;
                posMsg->sendClientToServer(strSend);
            }
            
            if( recordVideo == true )
            {
                if( writer != nullptr )
                {
                    writer->write(imageout);
                }
                rectangle( imagetrack, roi, Scalar(0,0,255), 2*drawScale, 1 );
            }
            else
            {
                rectangle( imagetrack, roi, roiColor, 2*drawScale, 1 );
            }
        }
        
        if( displayByWindow == true )
        {
            //#ifdef SHOW_CAMERA_WINDOW            
            if( (enableTracker == true) || (recordVideo == true) )
            {
                imshow("Camera input", imagetrack(zoom));
            }
            else
            {
                imshow("Camera input", imageout(zoom));
            }
            //#endif
        }
        
        c = (char)waitKey(OPENCV_WAIT_FOR_KEY);
        
        string recData = control->getData();
        handleJoystickEvents(&recData);
        
        if( recData != "Empty" )
        {
            size_t found = recData.find("Empty");
            if( found != string::npos )
            {
                recData.erase(found, 5);
            }
            cout << "\nData received: " << recData << endl;
            retCtrl = setControl(recData); 
            cout.flush();                                                
        }  
        
        if( videoMode == true )
        {
            if( displayByWindow == false )
            {
                if( (enableTracker == true) || (recordVideo == true) )
                {
                    processStreamMjpeg( imagetrack(zoom) );
                    //processStreamMjpeg( imageproc );
                }
                else
                {
                    processStreamMjpeg( imageout(zoom) );
                }
            }
            
            if( (c == 27) || (retCtrl < 0) ) //ESC
            {
                // End tracker
                cam.release();
                ret = -1;
                destroyAllWindows();
            }
        }
        else
        {
            if( photoStable > 0 )
            {
                photoStable--;
            }
            else
            if( photoStable > (-10) )
            {    
                string picturename;
                                
                imageshot = imageout.clone();
                photoStable--;

                picturename = "/media/pi/INTENSO/Pictures/tracker_" + getDateAndTime() + "_" + to_string(-photoStable) + ".jpg";
                cout << "saving picture: " << picturename << endl;
                imwrite( picturename, imageshot );
                cout.flush();
                //resize(imageout, imageshot, cv::Size(), 0.25, 0.25);
            }
            else
            if( photoStable <= -10 )
            {   
                cout << "Picture: false" << endl;
                cameraState = 0;
                photoStable = STABLE_PHOTO_SHOT;
                videoMode = true;
                stopVideoRecord();
                cam.release();
                /*
                if( displayByWindow == false )
                {
                    processStreamMjpeg( imageshot );
                    usleep(20000);
                }*/
            }
        }
    }
           
    return ret;
}

void Camera::dotDetection()
{
    cvtColor(imagetrack(roi), imagegray, CV_RGB2GRAY);
    //threshold(imagegray, imageproc, 0, 255, THRESH_BINARY | THRESH_OTSU);
    adaptiveThreshold(imagegray, imageproc, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 2);
    findContours(imageproc, contoursLoc, RETR_LIST, CHAIN_APPROX_SIMPLE);
    if( contoursLoc.size() > 0 )
    {
        contours = contoursLoc;
        double maxArea = 30000.0; //500.0
        int dotContourIndex = -1;
        Point2f dotContour;
        
        vector<DotTrackingType> vecDotTrack;
        vecDotTrack.clear();
        
        if( dotFound == false )
        {
            dotTracking.area = 0.0;
            
            for( size_t i = 0; i < contours.size(); i++ )
            {
                double actArea = contourArea(contours[i]);
                
                if( actArea < maxArea ) 
                {
                    if( actArea > dotTracking.area )
                    {
                        //cout << "actArea: " << actArea << ",maxContourIndex: " << i << endl;
                        dotTracking.area = actArea;
                        dotContourIndex = (int)i;
                        float dotRadius;
                        minEnclosingCircle(	contours[dotContourIndex], dotContour, dotRadius );
                        Point2d roiDot;
                        roiDot.x = roi.x + (double)dotContour.x;
                        roiDot.y = roi.y + (double)dotContour.y;
                        dotTracking.pnt = roiDot;
                        dotFound = true;
                    }
                }
            }
            if( dotFound == true )
            {
                for( int i=0; i<MEDIAN_FILTER_SIZE; i++ )
                {
                    medianInPosX[i] = dotTracking.pnt.x;
                    medianInPosY[i] = dotTracking.pnt.y;
                }
            }
        }
        else
        {
            for( size_t i = 0; i < contours.size(); i++ )
            {
                double actArea = contourArea(contours[i]);
                DotTrackingType dotTrack;
                                
                //cout << "actArea: " << actArea << ",maxContourIndex: " << i << endl;
                dotContourIndex = (int)i;
                float dotRadius;
                minEnclosingCircle(	contours[dotContourIndex], dotContour, dotRadius );
                dotTrack.index = dotContourIndex;
                Point2d roiDot;
                roiDot.x = roi.x + (double)dotContour.x;
                roiDot.y = roi.y + (double)dotContour.y;
                Point2d roiDist = dotTracking.pnt-roiDot;
                double squaredSum = (roiDist.x * roiDist.x) + (roiDist.y * roiDist.y);
                dotTrack.distance = sqrt(squaredSum);
                //cout << "dist: " << dist << endl;
                dotTrack.pnt = roiDot;
                dotTrack.area = actArea;                  
                vecDotTrack.push_back(dotTrack);
            }
            double comb = 1500.0;
            dotTracking.index = -1;
            double minComb = 0.0;
            for( int i = 0; i < vecDotTrack.size(); i++ )
            {
                double diffArea = fabs(dotTracking.area - vecDotTrack[i].area);
                minComb = diffArea + vecDotTrack[i].distance;
                
                if( minComb < comb )
                {
                    dotContourIndex = vecDotTrack[i].index;
                    comb = minComb;
                    dotTracking.index = i;
                }
            }
            
            if( dotTracking.index >= 0 )
            {
                dotTracking.pnt = vecDotTrack[dotTracking.index].pnt;
            }
            else
            {
                dotTracking.pnt = Point2d(0.0, 0.0);
                dotFound = false;
            }
            
        }
        if( dotContourIndex >= 0 )
        {
            //cout << "dotArea: " << dotArea << endl;
            drawContours( imagetrack(roi), contours, dotContourIndex, Scalar(0,255,0), 2*drawScale, 1);
            
            dotTracking.pnt.x = medianFilter( medianInPosX, dotTracking.pnt.x );
            dotTracking.pnt.y = medianFilter( medianInPosY, dotTracking.pnt.y );
            drawMarker( imagetrack, dotTracking.pnt, roiColor, MARKER_CROSS, (int)(50.0*drawScale), 2, 1 );
            
            //dotTracking.pnt = vecDot[dotVecIndex];
            //cout << "roi tracked: " << roiptDot << endl;
        }
        else
        {
            dotTracking.pnt = Point2d(0.0, 0.0);
        }
    }
    
    return;
}

void Camera::objectFlowbySubPixels()
{
    cvtColor(imagetrack(roi), imagegray, CV_RGB2GRAY);
    if( initObjectFlow == false ) 
    {
        adaptiveThreshold(imagegray, imagemorph, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 27, -10); // 15, -5 // 21, 2 
        //morphologyEx(imagemorph, imageproc, cv::MORPH_OPEN, cv::Mat::ones(3,3,CV_8U));
        morphologyEx(imagemorph, imageproc, cv::MORPH_CLOSE, cv::Mat::ones(10,10,CV_8U));
        findContours(imageproc, contoursLoc, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //RETR_EXTERNAL //RETR_LIST
        //findContours(imageproc, contoursLoc, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //RETR_EXTERNAL //RETR_LIST
        
        if( contoursLoc.size() > 0 )
        {
            contours = contoursLoc;
            double maxArea = 100000.0; //30000.0; //500.0
            int dotContourIndex = -1;
            Point2f dotContour;
            
            vector<DotTrackingType> vecDotTrack;
            vecDotTrack.clear();
            
            if( dotFound == false )
            {
                dotTracking.area = 0.0;
                //cout << "start" << endl;
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    double actArea = contourArea(contours[i]);
                    //cout << "actArea: " << actArea << ", i: " << i << endl;
                    if( actArea < maxArea ) 
                    {
                        if( actArea > dotTracking.area )
                        {
                            //cout << "actArea: " << actArea << ",maxContourIndex: " << i << endl;
                            dotTracking.area = actArea;
                            dotContourIndex = (int)i;
                            float dotRadius;
                            minEnclosingCircle(	contours[dotContourIndex], dotContour, dotRadius );
                            Point2d roiDot;
                            roiDot.x = roi.x + (double)dotContour.x;
                            roiDot.y = roi.y + (double)dotContour.y;
                            dotTracking.pnt = roiDot;
                            dotFound = true;
                        }
                    }
                }
                if( dotFound == true )
                {
                    for( int i=0; i<MEDIAN_FILTER_SIZE; i++ )
                    {
                        medianInPosX[i] = (float)dotTracking.pnt.x;
                        medianInPosY[i] = (float)dotTracking.pnt.y;
                    }
                }
            }
            else
            {
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    double actArea = contourArea(contours[i]);
                    DotTrackingType dotTrack;
                                    
                    //cout << "actArea: " << actArea << ",maxContourIndex: " << i << endl;
                    dotContourIndex = (int)i;
                    float dotRadius;
                    minEnclosingCircle(	contours[dotContourIndex], dotContour, dotRadius );
                    dotTrack.index = dotContourIndex;
                    Point2d roiDot;
                    roiDot.x = roi.x + (double)dotContour.x;
                    roiDot.y = roi.y + (double)dotContour.y;
                    Point2d roiDist = dotTracking.pnt-roiDot;
                    double squaredSum = (roiDist.x * roiDist.x) + (roiDist.y * roiDist.y);
                    dotTrack.distance = sqrt(squaredSum);
                    //cout << "dist: " << dist << endl;
                    dotTrack.pnt = roiDot;
                    dotTrack.area = actArea;                  
                    vecDotTrack.push_back(dotTrack);
                }
                double comb = 1500.0;
                dotTracking.index = -1;
                double minComb = 0.0;
                for( int i = 0; i < vecDotTrack.size(); i++ )
                {
                    double diffArea = fabs(dotTracking.area - vecDotTrack[i].area);
                    minComb = diffArea + vecDotTrack[i].distance;
                    
                    if( minComb < comb )
                    {
                        dotContourIndex = vecDotTrack[i].index;
                        comb = minComb;
                        dotTracking.index = i;
                    }
                }
                
                if( dotTracking.index >= 0 )
                {
                    dotTracking.pnt = vecDotTrack[dotTracking.index].pnt;
                }
                else
                {
                    dotTracking.pnt = Point2d(0.0, 0.0);
                    dotFound = false;
                }
                
            }
            if( dotContourIndex >= 0 )
            {
                drawContours( imagetrack(roi), contours, dotContourIndex, Scalar(0,255,0), 2*drawScale, 1);
                
                Point2f pnt2f;
                vector<Point2f> dots;
                pnt2f.x = (float)dotTracking.pnt.x;
                pnt2f.y = (float)dotTracking.pnt.y;
                dots.push_back(pnt2f);
                
                imagegray.convertTo(imagefloat, CV_32F);
                TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01);
                cornerSubPix(imagegray, dots, Size(5, 5), Size(-1, -1), criteria);
                              
                dotTracking.pnt.x = medianFilter( medianInPosX, dots[0].x);
                dotTracking.pnt.y = medianFilter( medianInPosY, dots[0].y );
                drawMarker( imagetrack, dotTracking.pnt, roiColor, MARKER_CROSS, (int)(50.0*drawScale), 2, 1 );
                //cout << "dotTracking: " << dotTracking.pnt << endl;
                
                //dotTracking.pnt = vecDot[dotVecIndex];
                //cout << "roi tracked: " << roiptDot << endl;
            }
            else
            {
                dotTracking.pnt = Point2d(0.0, 0.0);
            }
        }
    }
    else
    {
        initObjectFlow = false;
    }
    imageprev = imagegray.clone();
    
    return;
}

#if 0
void Camera::objectFlowbySubPixels() 
{
    cvtColor(imagetrack(roi), imagegray, CV_RGB2GRAY);
    
    if( initObjectFlow == false ) 
    {
        adaptiveThreshold(imagegray, imageproc, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, -5); // 21, 2
        morphologyEx(imageproc, imagemorph, cv::MORPH_OPEN, cv::Mat::ones(3,3,CV_8U));
        findContours(imagemorph, contoursLoc, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //RETR_LIST
        if( contoursLoc.size() > 0 )
        {
            contours = contoursLoc;
            double maxArea = 30000.0; //500.0
            dotTracking.area = 0.0;
            int dotContourIndex = -1;
            Point2f dotContour;
            vector<Point2f> dots;
           
            for( size_t i = 0; i < contours.size(); i++ )
            {
                double actArea = contourArea(contours[i]);
                
                if( actArea < maxArea ) 
                {
                    //cout << "actArea: " << actArea << endl;
                    //if( actArea > dotTracking.area )
                    if( actArea > 16.0 )
                    {
                        //cout << "actArea: " << actArea << ",maxContourIndex: " << i << endl;
                        dotTracking.area = actArea;
                        dotContourIndex = (int)i;
                        float dotRadius;
                        minEnclosingCircle(	contours[dotContourIndex], dotContour, dotRadius );
                        //Rect2d r = boundingRect(contours[i]);
                        Point2d roiDot;
                        roiDot.x = roi.x + (double)dotContour.x;
                        roiDot.y = roi.y + (double)dotContour.y;
                        //roiDot.x = roi.x + r.x + (r.width/2.0);
                        //roiDot.y = roi.y + r.y + (r.height/2.0);
                        dotTracking.pnt = roiDot;
                        if( dots.size() < 5 )
                        {
                            Point2f pnt2f;
                            pnt2f.x = (float)dotTracking.pnt.x;
                            pnt2f.y = (float)dotTracking.pnt.y;
                            dots.push_back(pnt2f);
                        }
                    }
                }
            }
            
            //if( dotTracking.area > 0.1 )
            if( dots.size() > 0 )
            {
                imagegray.convertTo(imagefloat, CV_32F);
                TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01);
                cornerSubPix(imagegray, dots, Size(5, 5), Size(-1, -1), criteria);
                //cout << "cornerSubPix: x=" << dots[0].x << ", y=" << dots[0].y << endl;
                
                
                //if( dotContourIndex >= 0 )
                for( int i = 0; i < dots.size(); i++ )
                {
                    //cout << "dotArea: " << dotArea << endl;
                    //drawContours( imagetrack(roi), contours, dotContourIndex, Scalar(0,255,0), 2*drawScale, 1);
                    
                    //dotTracking.pnt.x = medianFilter( medianInPosX, dotTracking.pnt.x );
                    //dotTracking.pnt.y = medianFilter( medianInPosY, dotTracking.pnt.y );
                    //drawMarker( imagetrack, dotTracking.pnt, roiColor, MARKER_CROSS, (int)(50.0*drawScale), 2, 1 );
                    drawMarker( imagetrack, dots[i], roiColor, MARKER_CROSS, (int)(50.0*drawScale), 2, 1 );
                }
                
                // -- Optical Flow Lucas-Kanade --
                vector<Point2f> dotsMoved;
                vector<uchar> status;
                vector<float> err;
 
                calcOpticalFlowPyrLK(imagegray, imageprev, dots, dotsMoved, status, err, Size(21, 21), 3, criteria);

                if (status[0]) 
                {
                    Point2f meandelta;
                    meandelta.x = 0.0f;
                    meandelta.y = 0.0f;
                    for( int i = 0; i < dotsMoved.size(); i++ )
                    {
                        Point2f delta = dotsMoved[i] - dots[i];
                        meandelta.x += delta.x;
                        meandelta.y += delta.y;
                        cout << "Moved " << i << ": dx=" << delta.x << ", dy=" << delta.y << endl;
                    }
                    meandelta.x /= (float)dotsMoved.size();
                    meandelta.y /= (float)dotsMoved.size();
                    cout << "Mean moved: mx=" << meandelta.x << ", my=" << meandelta.y << endl;
                } 
                else 
                {
                    cout << "Tracking fehlgeschlagen." << endl;
                }
            }
            dots.clear();
        }
    }
    else
    {
        initObjectFlow = false;
    }
    imageprev = imagegray.clone();
    
#if 0    
    // -- Bild laden (Graustufen) --
    cv::Mat img1 = cv::imread("frame1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("frame2.png", cv::IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Fehler beim Laden der Bilder." << std::endl;
        return -1;
    }

    // -- adaptiveThreshold zur Sternsegmentierung --
    cv::Mat thresh;
    cv::adaptiveThreshold(img1, thresh, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, 15, -5);

    // -- Rauschen filtern --
    cv::Mat morph;
    cv::morphologyEx(thresh, morph, cv::MORPH_OPEN, cv::Mat::ones(3,3,CV_8U));

    // -- Konturen finden --
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // -- Größten "Stern" finden --
    double maxArea = 0;
    cv::Rect bestRect;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area > maxArea) {
            cv::Rect r = cv::boundingRect(c);
            if (r.width > 4 && r.height > 4) {
                maxArea = area;
                bestRect = r;
            }
        }
    }

    if (maxArea == 0) {
        std::cerr << "Kein geeigneter Stern gefunden." << std::endl;
        return -1;
    }

    // -- Mittelpunkt mit Moments berechnen (grob) --
    cv::Mat roi = img1(bestRect);
    cv::Moments m = cv::moments(roi, true);
    double cx = bestRect.x + m.m10 / m.m00;
    double cy = bestRect.y + m.m01 / m.m00;
    std::cout << "Zentrum (Momente): x=" << cx << ", y=" << cy << std::endl;

    // -- Punkt als cv::Point2f vorbereiten --
    std::vector<cv::Point2f> p0 = { cv::Point2f((float)cx, (float)cy) };

    // -- Subpixel-Verfeinerung mit cornerSubPix --
    cv::Mat img1_float;
    img1.convertTo(img1_float, CV_32F); // cornerSubPix braucht float

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);
    cv::cornerSubPix(img1, p0, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    std::cout << "Nach cornerSubPix: x=" << p0[0].x << ", y=" << p0[0].y << std::endl;

    // -- Optical Flow Lucas-Kanade --
    std::vector<cv::Point2f> p1;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(img1, img2, p0, p1, status, err,
                             cv::Size(21, 21), 3, criteria);

    if (status[0]) {
        float dx = p1[0].x - p0[0].x;
        float dy = p1[0].y - p0[0].y;
        std::cout << "Bewegung: dx=" << dx << ", dy=" << dy << std::endl;
    } else {
        std::cout << "Tracking fehlgeschlagen." << std::endl;
    }

    // -- Anzeige --
    cv::Mat img1_c, img2_c;
    cv::cvtColor(img1, img1_c, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, img2_c, cv::COLOR_GRAY2BGR);

    cv::circle(img1_c, p0[0], 5, cv::Scalar(0, 255, 0), 2);
    if (status[0]) {
        cv::circle(img2_c, p1[0], 5, cv::Scalar(0, 0, 255), 2);
        cv::line(img2_c, p0[0], p1[0], cv::Scalar(255, 0, 0), 1);
    }

    cv::imshow("Bild 1 - Zentrum", img1_c);
    cv::imshow("Bild 2 - Optical Flow", img2_c);
    cv::waitKey(0);
#endif    
    return;
} 
#endif

void Camera::sendFocus()
{
    string strSend = "mean=" + to_string(meanFocus) + ";";
    procMsg->sendClientToServer(strSend);
    return;
}

void Camera::calcFocus()
{
    Mat dst;
    cvtColor(imageout(roi), imagefocus, CV_RGB2GRAY);
    
    Laplacian(imagefocus, dst, CV_64F);

    Scalar mu, sigma;
    meanStdDev(dst, mu, sigma);

    double meanFocusLoc = sigma.val[0] * sigma.val[0];
    
    //Mat imageSobel;
    //Sobel(imagefocus, imageSobel, CV_16U, 1, 1);
    //double meanFocusLoc = mean(imageSobel)[0];
    
    meanFocus = meanFocus + (0.3 * (meanFocusLoc - meanFocus)); //0.09 0.3
    
/*
    osci1.erase( osci1.begin()+0 );
    osci1.push_back( meanFocus );
    cv::Mat image1;
    plot1->render( image1);
    cv::imshow( "Plot1", image1 );
*/        
    putText(imagetrack(zoom), to_string((int)meanFocus), Point(focusPos,focusPos), FONT_HERSHEY_SIMPLEX, 1*drawScale, focusColor, 2, LINE_AA);
    //line(imagetrack(zoom), Point(focusPos,focusPos), Point(focusPos+(meanFocus*focusLineLength),focusPos), 
    //     focusColor, 5*drawScale, LINE_8);
    
    return;
}

int Camera::setControl( string prop )
{
    size_t pos;
    int ret = 0;

    if( (pos = prop.rfind("Position")) != string::npos )
    {
        if( (pos = prop.rfind("Positionspeed=")) != string::npos )
        {
            string sub = prop.substr(pos+14);
            cout << "Positionspeed: " << (int)stoi(sub) << endl;
            //position->setFixedRate( (char)stoi(sub) );
            string strSend = "rate=" + sub;
            posMsg->sendClientToServer(strSend);
        }    
        
        if( (pos = prop.rfind("PositionspeedX=")) != string::npos )
        {
            string sub = prop.substr(pos+15);
            cout << "PositionspeedX: " << (int)stoi(sub) << endl;
            //position->setFixedRate( (char)stoi(sub) );
            string strSend = "rateAzm=" + sub;
            posMsg->sendClientToServer(strSend);
        }  
        
        if( (pos = prop.rfind("PositionspeedY=")) != string::npos )
        {
            string sub = prop.substr(pos+15);
            cout << "PositionspeedY: " << (int)stoi(sub) << endl;
            //position->setFixedRate( (char)stoi(sub) );
            string strSend = "rateAlt=" + sub;
            posMsg->sendClientToServer(strSend);
        }
        
        if( (pos = prop.rfind("Position=up")) != string::npos )
        {
            cout << "Position: up" << endl;
            //position->setFixedAlt( 1 );
            posMsg->sendClientToServer("alt=1");
        }  
        
        if( (pos = prop.rfind("Position=down")) != string::npos )
        {
            cout << "Position: down" << endl;
            //position->setFixedAlt( -1 );
            posMsg->sendClientToServer("alt=-1");
        }  
        
        if( (pos = prop.rfind("Position=left")) != string::npos )
        {
            cout << "Position: left" << endl;
            //position->setFixedAzm( -1 );
            posMsg->sendClientToServer("azm=-1");
        }  
        
        if( (pos = prop.rfind("Position=right")) != string::npos )
        {
            cout << "Position: right" << endl;
            //position->setFixedAzm( 1 );
            posMsg->sendClientToServer("azm=1");
        }  
        
        if( (pos = prop.rfind("Position=stopud")) != string::npos )
        {
            cout << "Position: stop up/down" << endl;
            //position->setFixedAlt( 0 );
            posMsg->sendClientToServer("alt=0");
        }  
        
        if( (pos = prop.rfind("Position=stoplr")) != string::npos )
        {
            cout << "Position: stop left/right" << endl;
            //position->setFixedAzm( 0 );
            posMsg->sendClientToServer("azm=0");
        }  
        
        if( (pos = prop.rfind("Position=roiup")) != string::npos )
        {
            
            roiSize += 0.01;
            if( roiSize > 0.8 ) //0.5
            {
                roiSize = 0.8;  //0.5
            }
            cout << "Position: roi size up: " << roiSize << endl;
            initRoi(roipt);
            if( dotTracker == false )
            {
                runTracker = false;
                initTracker = false;
            }
        } 
        
        if( (pos = prop.rfind("Position=roidn")) != string::npos )
        {
            roiSize -= 0.01;
            if( roiSize < 0.01 )
            {
                roiSize = 0.01;
            }
            cout << "Position: roi size down: " << roiSize << endl;
            initRoi(roipt);
            if( dotTracker == false )
            {
                runTracker = false;
                initTracker = false;
            }
        } 
    }
    else
#ifdef PICTURE_yes
    if( (pos = prop.rfind("Display")) != string::npos )
    {
        if( (pos = prop.rfind("Display=false")) != string::npos )
        {
            cout << "Display: false" << endl;
            cameraState = 0;
            videoMode = true;
            displayByWindow = false;
            stopVideoRecord();
            cam.release();
        }   
        else
        if( (pos = prop.rfind("Display=true")) != string::npos )
        {
            cout << "Display: true" << endl;
            cameraState = 0;
            videoMode = true;
            displayByWindow = true;
            stopVideoRecord();
            cam.release();
        }
    }
    else
    if( (pos = prop.rfind("Picture")) != string::npos )
    {
        if( (pos = prop.rfind("Picture=false")) != string::npos )
        {
            // Not used anymore
            cout << "Picture: false" << endl;
            cameraState = 0;
            photoStable = STABLE_PHOTO_SHOT;
            videoMode = true;
            stopVideoRecord();
            cam.release();
        }   
        else
        if( (pos = prop.rfind("Picture=true")) != string::npos )
        {
            cout << "Picture: true" << endl;
            //posMsg->sendClientToServer("deInit");
            posMsg->sendClientToServer("notrack");
            cameraState = 0;
            photoStable = STABLE_PHOTO_SHOT;
            videoMode = false;
            stopVideoRecord();
            cam.release();
        }  
        else
        if( (pos = prop.rfind("Picture=record")) != string::npos )
        {
            cout << "Picture: record" << endl;
            if( recordVideo == false )
            {
                recordVideo = true;
                string videoname;
                videoname = "/media/pi/INTENSO/Videos/tracker_" + getDateAndTime() + ".avi";
                cout << "Video name: " << videoname << endl;
                if( writer == nullptr )
                {
                    writer = new VideoWriter;
                }
                writer->open(videoname, 
                             CV_FOURCC('M', 'J', 'P', 'G'), 
                             camProps.fpsVideo, imagetrack.size(), true);
                cout << "Video writer open: " << writer->isOpened() << endl;
            }
            else
            {
                stopVideoRecord();
                cout << "Video writer close" << endl;
            }
        }  
    }
    else
#endif
    if( (pos = prop.rfind("Tracker")) != string::npos )
    {
        if( (pos = prop.rfind("Tracker=end")) != string::npos )
        {
            cout << "Tracker: end" << endl;
            procMsg->sendClientToServer("exit");
            posMsg->sendClientToServer("exit");
            ret = -1;
        } 
        else
        if( (pos = prop.rfind("Tracker=init")) != string::npos )
        {
            cout << "Tracker: init" << endl;
            enableTracker = true;
        }
        else
        if( (pos = prop.rfind("Tracker=deinit")) != string::npos )
        {
            cout << "Tracker: deinit" << endl;
            enableTracker = false;
        }
        else
        if( (pos = prop.rfind("Tracker=run")) != string::npos )
        {
            cout << "Tracker: run" << endl;
            initTracker = true;
            runTracker = false;
            tracker = Tracker::create( TRACKING_METHOD );
        }
        else
        if( (pos = prop.rfind("Tracker=stop")) != string::npos )
        {
            cout << "Tracker: stop" << endl;
            initTracker = false;
            runTracker = false;
            if( runControl == true )
            {
                posMsg->sendClientToServer("deInit");
            }
            runControl = false;
            roipt.x = camProps.widthVideo / 2;
            roipt.y = camProps.heightVideo / 2;
            initRoi(roipt);
            roiColor = Scalar(255,255,255);
        }
        else
        if( (pos = prop.rfind("Tracker=ctrl")) != string::npos )
        {
            cout << "Tracker: ctrl" << endl;
            if( runTracker == true ) 
            {
                if( runControl == false )
                {
                    roiColor = Scalar(255,0,0);
                    runControl = true;
                    string strSend = "init=" + to_string((int)camProps.widthVideo) + "x" + 
                                     to_string((int)camProps.heightVideo) + ";";
                    posMsg->sendClientToServer(strSend);
                }
                else
                {
                    roiColor = Scalar(0,255,0);
                    runControl = false;
                    posMsg->sendClientToServer("deInit");
                }
            }
        }
        else
        if( (pos = prop.rfind("Tracker=kcf")) != string::npos )
        {
            cout << "Tracker: kcf" << endl;
            dotTracker = false;
        }
        else
        if( (pos = prop.rfind("Tracker=dot")) != string::npos )
        {
            cout << "Tracker: dot" << endl;
            dotTracker = true;
        }
    }
    else
    if( (pos = prop.rfind("Goto")) != string::npos )
    {
        if( (pos = prop.rfind("GotoOrAzm=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "GotoOrAzm: " << (int)stoi(sub) << endl;
            posMsg->sendClientToServer(sub);
        }   
        else
        if( (pos = prop.rfind("GotoOrAlt=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "GotoOrAzm: " << (int)stoi(sub) << endl;
            posMsg->sendClientToServer(sub);
        }  
        else
        if( (pos = prop.rfind("GotoObj1=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "GotoObj1: " << sub << endl;
            posMsg->sendClientToServer(sub);
        } 
        else
        if( (pos = prop.rfind("GotoObj2=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "GotoObj2: " << sub << endl;
            posMsg->sendClientToServer(sub);
        }
        else
        if( (pos = prop.rfind("GotoState=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "GotoState: " << sub << endl;
            posMsg->sendClientToServer(sub);
        }
    }
    else
    {
        if( (pos = prop.rfind("Zoom=")) != string::npos )
        {
            string sub = prop.substr(pos+5);
            zoomFactor = stoi(sub);
            changeZoom();
            cout << "Zoom: " << zoomFactor << endl;
        }

        if( (pos = prop.rfind("Brightness=")) != string::npos )
        {
            string sub = prop.substr(pos+11);
            camProps.brightness = (double)stoi(sub);
            cout << "Brightness: " << camProps.brightness << endl;
            cam.set( CV_CAP_PROP_BRIGHTNESS, camProps.brightness );
        }
        
        if( (pos = prop.rfind("Contrast=")) != string::npos )
        {
            string sub = prop.substr(pos+9);
            camProps.contrast = (double)stoi(sub);
            cout << "Contrast: " << camProps.contrast << endl;
            cam.set( CV_CAP_PROP_CONTRAST, camProps.contrast );
        }  
                    
        if( (pos = prop.rfind("Saturation=")) != string::npos )
        {
            string sub = prop.substr(pos+11);
            camProps.saturation = (double)stoi(sub);
            cout << "Saturation: " << camProps.saturation << endl;
            cam.set( CV_CAP_PROP_SATURATION, camProps.saturation );
        }
        
        if( (pos = prop.rfind("Gain=")) != string::npos )
        {
            string sub = prop.substr(pos+5);
            camProps.gain = (double)stoi(sub);
            cout << "Gain: " << camProps.gain << endl;
            cam.set( CV_CAP_PROP_GAIN, camProps.gain );
        } 
                     
        if( (pos = prop.rfind("Exposure=")) != string::npos )
        {
            string sub = prop.substr(pos+9);
            camProps.exposure = (double)stoi(sub);
            cout << "Exposure: " << camProps.exposure << endl;
            cam.set( CV_CAP_PROP_EXPOSURE, camProps.exposure );
        }
        
        if( (pos = prop.rfind("Fps=")) != string::npos )
        {
            string sub = prop.substr(pos+4);
            camProps.fpsVideo = (double)stoi(sub);
            cout << "Fps: " << camProps.fpsVideo << endl;
            cameraState = 0;
            stopVideoRecord();
            cam.release();
        }   
        
        if( (pos = prop.rfind("Resolution")) != string::npos )
        {
            string sub = prop.substr(pos+10); 
            string width = sub.substr(sub.find('=')+1, sub.find('x')-1); 
            string height = sub.substr(sub.find('x')+1, sub.find(';')-1); 
            camProps.widthVideo = (double)stoi(width);
            camProps.heightVideo = (double)stoi(height);
            cout << "Resolution: " << camProps.widthVideo << "x" << camProps.heightVideo << endl;
            cameraState = 0;
            stopVideoRecord();
            cam.release();
        } 
        
        if( (pos = prop.rfind("LocN=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "LocN: " << (double)stod(sub) << endl;
            posMsg->sendClientToServer(sub);
        }   
        
        if( (pos = prop.rfind("LocE=")) != string::npos )
        {
            string sub = prop.substr(pos);
            //cout << "LocE: " << (double)stod(sub) << endl;
            posMsg->sendClientToServer(sub);
        }   
    }
    
    return ret;
}

void Camera::initRoi(Point2d pnt)
{
    roi.height = (double)((int)(camProps.heightVideo * roiSize));
    roi.width = (double)((int)(camProps.widthVideo * roiSize));
    roi.x = (double)((int)(pnt.x - (roi.width * 0.5)));
    roi.y = (double)((int)(pnt.y - (roi.height * 0.5)));
    cout << "ROI rect: " << roi << endl;
}

void Camera::changeZoom()
{
    if( zoomFactor == 1 )
    {
        zoom.width = camProps.widthVideo;
        zoom.height = camProps.heightVideo;
        zoom.x = 0.0;
        zoom.y = 0.0;        
    }
    else
    {
        zoom.width = camProps.widthVideo / (double)zoomFactor;
        zoom.height = camProps.heightVideo / (double)zoomFactor;
        zoom.x = (camProps.widthVideo - zoom.width) * 0.5;
        zoom.y = (camProps.heightVideo - zoom.height) * 0.5;         
    }
    //focusLineLength = 100.0 * drawScale / zoomFactor;
    focusLineLength = 5.0 * drawScale / zoomFactor;
}

float Camera::medianFilter( float *medArr, float in )
{
    float sortArr[MEDIAN_FILTER_SIZE];
    for( int i=0; i<MEDIAN_FILTER_SIZE; i++ )
	{
		medArr[(MEDIAN_FILTER_SIZE-1)-i] = medArr[(MEDIAN_FILTER_SIZE-2)-i];
        sortArr[(MEDIAN_FILTER_SIZE-1)-i] = medArr[(MEDIAN_FILTER_SIZE-1)-i];
	}
	medArr[0] = in;
    sortArr[0] = in;
    
    int len = sizeof(sortArr)/sizeof(sortArr[0]);
    sort( sortArr, sortArr + len, greater<float>() );
    
    int indArr = (MEDIAN_FILTER_SIZE>>1);
    //cout << "indArr: " << indArr << endl;
    /*cout << "sortArr: ";
    for( int i=0; i<MEDIAN_FILTER_SIZE; i++ )
    {
        cout << sortArr[i] << ", ";
    }
    cout << endl;
    */
    return sortArr[indArr];
}


string Camera::getDateAndTime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char bufDateTime[80];
    
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(bufDateTime, 80, "%F_%H%M%S", timeinfo);
    
    return bufDateTime;
}

void Camera::stopVideoRecord()
{
    recordVideo = false;
    if( writer != nullptr )
    {
        delete writer;
        writer = nullptr;
    }
}

void Camera::initStreamMjpeg()
{
   writeMjpegHeader = string("HTTP/1.0 200 OK\r\n")
               + "Server: robot\r\n"
               + "Connection: close\r\n" 
               + "Max-Age: 0\r\n" 
               + "Expires: 0\r\n"
               + "Cache-Control: no-cache, private\r\n" 
               + "Pragma: no-cache\r\n" 
               + "Content-Type: multipart/x-mixed-replace; " 
               + "boundary=--BoundaryString\r\n\r\n";
       
   stream->sendData(&writeMjpegHeader); 
}

void Camera::processStreamMjpeg( Mat image )
{
    vector<uchar> buf;
    imencode(".jpg", image, buf, vector<int>());
    string imagebuffer(buf.begin(), buf.end());

    writeMjpegContent = string("--BoundaryString\r\n") 
                      + "Content-type: image/jpg\r\n" 
                      + "Content-Length: " 
                      + to_string(imagebuffer.length())
                      + "\r\n\r\n"
                      + imagebuffer
                      + "\r\n\r\n";
    stream->sendData(&writeMjpegContent); 
}
