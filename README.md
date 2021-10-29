# Telescope

## Introduction
<!-- What the project does --> 
<!-- Why the project is useful -->
The Telescope project has been introduced to track astronomy objects e.g. moon, planets, etc. and to watch them by mobile device like smart phone or tablet without adjusting the object position through the Telescope manually. There is the possibility to take object image snap shots and recording video stream of the object. But based on the Altitude-Azimuth-mounting it is not suitable for longer exposure times since the object will rotate during that time. Better would be an equatorial mount for this.  

Another purpose is to use the Tracker application running on one Telescope and control another Telescope as a slave to follow the master Telescope. The slave Telescope can then be used to watch the astronomy objects without adjusting the object position manually and without using Star alignment.

The Telescope NexStar 4 will be used as master and NexStar 8 SE will be used as slave.

## Electrical setup and wiring
### Telescope power
There is the possibility to connect external 12V Power sources instead of using battery compartment of the Telescope.
The connector is shown below:  
![Telescope Power](/Images/TelescopePower.jpg)  
The inner diameter of the plug is 2.1mm.
### NexStar Telescope data interface
There is a HW port on the Hand controller of NexStar Telescopes controlling Telescope from external devices. For older versions the HW port is a RS232 serial port and for newer versions the serial port is realized by USB.
#### RS232 Interface for NexStar 4
The following picture shows the Pin wiring for RS232 port on Hand controller of NexStar 4.   
![RJ11 Hand Controller](/Images/RJ11HandController.png)  
Note that there is a need of a RS232 voltage level shifter to communicate with an external PC.
#### USB Interface to NexStar 8 SE
The USB port can be used directly to communicate with external devices.
#### Slewing commands
The following commands allow you to slew (move) the telescope at variable rates.
For variable rates, multiply the desired rate by 4 and then separate it into a high and low byte. For example if the
desired tracking rate is 150 arcseconds/second, then:  
* trackRateHigh = (150 * 4) \ 256 = 2, and  
* trackRateLow = (150 * 4) mod 256 = 88 

| Function | Command | Response | 
| --- | --- | --- | 
| Variable rate Azm (or RA) slew in positive direction | 'P', 3, 16, 6, trackRateHigh, trackRateLow, 0, 0 | '#' |
| Variable rate Azm (or RA) slew in negative direction | 'P', 3, 16, 7, trackRateHigh, trackRateLow, 0, 0 | '#' |
| Variable rate Alt (or Dec) slew in positive direction | 'P', 3, 17, 6, trackRateHigh, trackRateLow, 0, 0 | '#' | 
| Variable rate Alt (or Dec) slew in negative direction | 'P', 3, 17, 7, trackRateHigh, trackRateLow, 0, 0 | '#' |
### Optical focus motor
Manually focusing by hand causes wiggle in the observing object. To avoid this a small motor is mounted on the Telescope focus.

![Focus Motor](/Images/FocusMotor.jpg)  
  
Motor will be controlled by SW. It can be distuinguished between Auto Focus and Manual Focus. 
### Tracking Camera 
For tracking and taking images the Raspberry PI HQ camera is used. Following an abstraction of data sheet
* Sony IMX477R Sensor
* 7.9mm Sensor diagonal
* Pixelsize 1.55µm x 1.55µm
* Integrated IR cut filter with following Transmission curve:
![IR Filter Transmission](/Images/IRFilterTransmission.png)  
Astronomy imaging requests a IR cut filter with passing wavelengths from 400nm to 700nm with a transmission of 97...99% to block IR light which causes halos around all objects.
For tracking the integrated IR cut filter is fine enough.
### Main controller
The controller used here for the Telescope is a Raspberry PI 3B+. The wiring overview is shown below.
![Telescope Wiring](/Images/TelescopeWiring.jpg)  
The camera is connected through the Raspberry's Camera port. Slewing commands will be send over RS232 interface with Voltage converter for older Telescopes. For newer Telescopes the communication is realized by USB connection. Focus motor driver works with 3.3V driven by RPi directly. To control the motor speed PWM Pins will be used. 
## Operation
The Telescope tracker can be controlled by any web browser calling http://RPi-hostname:1880/ui. As following showed the tracker tabs will be explained in more detail.
### Tracker
This tab controls starting, finishing the application and shutdowns the controller.  
  
![Tracker Screen Shot](/Images/TrackerScreenShot.jpg)  
- Green button: Starts the application
- Orange button: Finishes the application
- Red button: Controller Shutdown 
### Display
In this tab all relevant Display functions are collected. 

![Display Screen Shot](/Images/DisplayScreenShot.jpg)  
- Camera stream (Main area)
- Camera stream zoom (Slider)
- Switch between Camera stream over http or over Rasbperry HDMI port (Monitor)
- Taking pictures or recording stream (Camera and Video symbol)
- Show/Hide ROI (Region Of Interest) for tracking (Square)
- Track object (Arrow)
- Follow object by Telescope slewing (Circle with Dot)
### Position
With following tab the Telescope can be slewed to the desired object position by the arrows in following tab.

![Position Screen Shot](/Images/PositionScreenShot.jpg)  
- With the squared icon buttons the slewing can be stopped
- With the Speed slider the slewing rate can be set
- With the selector the Telescope can be selected which needs to be slewed
### Focus
The Focus tab is used to focus the object shown in camera stream.

![Focus Screen Shot](/Images/FocusScreenShot.jpg)  
- Slow focus motor speed can be controlled by one arrow buttons manually
- Fast focus motor speed can be controlled by two arrow buttons manually
- The button with opposite arrows is the auto focus. Here it is important to focus the object by manual control buttons above as good as possible.
To fine tune focus automatically press this button.
### Properties
With Properties tab the camera properties as followed can be set:

![Properties Screen Shot](/Images/PropertiesScreenShot.jpg)  
- Brightness
- Contrast
- Saturation
- Gain
- Exposure
- Frames Per Second (FPS)
- Resolution
## Software function description
### Remote Control application
#### Node-RED flow overview
#### Major flow nodes
### Main Control application
#### Tracking
#### Object Control
#### Focus
1. Manual focus
2. Autofocus


