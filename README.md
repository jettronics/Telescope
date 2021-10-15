# Telescope

## Introduction
<!-- What the project does --> 
<!-- Why the project is useful -->
The Telescope project has been introduced to track astronomy objects e.g. moon, planets, etc. and to watch them by mobile device like smart phone or tablet without adjusting the object position through the Telescope manually. There is the possibility to take object image snap shots and recording video stream of the object. But based on the Altitude-Azimuth-mounting it is not suitable for longer exposure times since the object will rotate during that time. Better would be an equatorial mount for this.  

Another purpose is to use the Tracker application running on one Telescope and control another Telescope as a slave to follow the master Telescope. The slave Telescope can then be used to watch the astronomy objects without adjusting the object position manually and without using Star alignment.

The Telescope NexStar 4 will be used as master and NexStar 8 SE will be used as slave.

## Electrical setup and wiring
### Telescope power
### Telescope data interface
#### Slewing commands
The following commands allow you to slew (move) the telescope at variable rates.
For variable rates, multiply the desired rate by 4 and then separate it into a high and low byte. For example if the
desired tracking rate is 150 arcseconds/second, then:  
trackRateHigh = (150 * 4) \ 256 = 2, and  
trackRateLow = (150 * 4) mod 256 = 88 

| Function | Command | Response | 
| --- | --- | --- | 
| Variable rate Azm (or RA) slew in positive direction | 'P', 3, 16, 6, trackRateHigh, trackRateLow, 0, 0 | '#' |
| Variable rate Azm (or RA) slew in negative direction | 'P', 3, 16, 7, trackRateHigh, trackRateLow, 0, 0 | '#' |
| Variable rate Alt (or Dec) slew in positive direction | 'P', 3, 17, 6, trackRateHigh, trackRateLow, 0, 0 | '#' | 
| Variable rate Alt (or Dec) slew in negative direction | 'P', 3, 17, 7, trackRateHigh, trackRateLow, 0, 0 | '#' |

#### RS232 Interface for NexStar 4
#### USB Interface to NexStar 8 SE
### Optical focus motor
### Tracking Camera 
### Main controller

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

## Operation
### Tracker
### Focus
### Camera
### ...



