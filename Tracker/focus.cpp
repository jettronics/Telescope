
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <vector>

#include "setup.h"
#include "procmessage.h"
#include "focus.h"

#define MAX_WAIT_COUNT  7
#define MAX_WAIT_TURN   6

Focus::Focus()
    : turnPre(TurnStop) 
    , turnPost(TurnStop) 
    , turnLast(TurnStop)
    , steptimeout(0)
    , procMsg(nullptr)
    , state(StateIdle)
    , timecnt(0)
    , aFocus(false)
    , afMeank(0.0)
    , afMeanStart(0.0)
    , afMeanMax(0.0)    
    , afDiffInt(0.0)
    , autoFocusState(AutoFocusStopped)
    , waitCnt(0) 
    , waitTurnThres(MAX_WAIT_TURN) 
    , waitTurn(0)
{
    findFocus.clear();
}

Focus::Focus(ProcMessage *proc)
    : turnPre(TurnStop) 
    , turnPost(TurnStop) 
    , turnLast(TurnStop)
    , steptimeout(0)
    , procMsg(proc)
    , state(StateIdle)
    , timecnt(0)
    , aFocus(false)
    , afMeank(0.0)
    , afMeanStart(0.0)
    , afMeanMax(0.0)    
    , afDiffInt(0.0)    
    , autoFocusState(AutoFocusStopped)  
    , waitCnt(0)
    , waitTurnThres(MAX_WAIT_TURN)  
    , waitTurn(0)  
{
    findFocus.clear();
}


Focus::~Focus()
{

}


void Focus::init()
{
    cout << "Focus init" << endl;
    // Export the desired pin by writing to /sys/class/gpio/export
    int fd;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    write(fd, "12", 2);
    close(fd);
    fd = open("/sys/class/gpio/export", O_WRONLY);
    write(fd, "18", 2);
    close(fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpioxx/direction
    fd = open("/sys/class/gpio/gpio12/direction", O_WRONLY);
    write(fd, "out", 3);
    close(fd);
    fd = open("/sys/class/gpio/gpio18/direction", O_WRONLY);
    write(fd, "out", 3);
    close(fd);

    // Set the pin to a level
    fd1 = open("/sys/class/gpio/gpio12/value", O_WRONLY);
    write(fd1, "0", 1);
    //close(fd);
    fd2 = open("/sys/class/gpio/gpio18/value", O_WRONLY);
    write(fd2, "0", 1);
    //close(fd);
    
}

void Focus::deInit()
{
    close(fd1);
    close(fd2);
}

void Focus::process()
{
    if( turnPre != turnPost )
    {
        //steptimeout = 20000;
        steptimeout = 0;
        if( turnPre != TurnStop )
        {
            gettimeofday(&tvStart, 0);
            //cout << "Focus time start: " << tvStart.tv_sec << "s, " << tvStart.tv_usec << "us" << endl;
            if( ((turnPre == TurnStepRight) &&
                 ((turnLast == TurnStepLeft) || (turnLast == TurnRunLeft))) ||
                ((turnPre == TurnStepLeft) &&
                 ((turnLast == TurnStepRight) || (turnLast == TurnRunRight))) ) 
            {
                steptimeout = 100000;
            }
        }
        turnPost = turnPre;
        driver(turnPost);
    }
    else
    if( (turnPost == TurnStepRight) ||
        (turnPost == TurnStepLeft) )
    {
        gettimeofday(&tvEnd, 0);
        
        turnLast = turnPost;
                
        int timeoutus = tvStart.tv_usec+steptimeout;
        int timeouts = tvStart.tv_sec;
        if( timeoutus > 900000 )
        {
            //timeoutus = timeoutus - 999999;
            timeoutus = 0;
            timeouts += 1;
            //cout << "Focus crit time limit" << endl;
        }
        if( ((tvEnd.tv_usec >= timeoutus) && 
             (tvEnd.tv_sec >= timeouts)) ||
            (steptimeout == 0) )
        {
            turnPre = TurnStop;
            turnPost = TurnStop;
            driver(turnPost);
            //cout << "Focus time stop step: " << timeouts << "s, " << timeoutus << "us" << endl;
        }
    }
    else
    if( (turnPost == TurnRunRight) ||
        (turnPost == TurnRunLeft) )
    {
        gettimeofday(&tvEnd, 0);
        
        turnLast = turnPost;
        
        if( tvEnd.tv_sec >= tvStart.tv_sec+5 )
        {
            turnPre = TurnStop;
            turnPost = TurnStop;
            driver(turnPost);
            //cout << "Focus time stop run: " << tvEnd.tv_sec << "s, " << tvEnd.tv_usec << "us" << endl;
        }
    }
    
    return;
}


int Focus::processMsg()
{
    int ret = 0;
    string rec = procMsg->receiveServerFromClient();
    if( rec.length() > 1 )
    {
        size_t pos;
        
        if( (pos = rec.find("autooff")) != string::npos )
        {
            aFocus = false;
            if( turnPre != Focus::TurnStop )
            {
                turnLast = turnPre;
            }
            turnPre = Focus::TurnStop;
            turnPost = Focus::TurnStop;
            driver(Focus::TurnStop); 
            autoFocusState = AutoFocusStopped;
            cout << "autoFocusState: Stop" << endl;
        }
        else
        if( (pos = rec.find("autoon")) != string::npos )
        {
            aFocus = true;
            afMeank = 0.0;
            afMeanStart = 0.0;
            afMeanMax = 0.0;
            afDiffInt = 0.0;
            autoFocusState = AutoFocusStart;              
            turnPre = Focus::TurnStop;
            turnPost = Focus::TurnStop;
            cout << "autoFocusState: Start" << endl;
        }        
        else
        if( (pos = rec.find("mean=")) != string::npos )
        {
            //cout << "rec: " << rec << endl;
            string mean = rec.substr(pos+5);
            afMeank = (double)stod(mean);
            //cout << "mean: " << afMeank << endl;
        }
        else
        if( (pos = rec.find("runleft")) != string::npos )
        {
            if( turnPre == Focus::TurnRunLeft )
            {
                turnPre = Focus::TurnStop;
            }
            else
            {
                turnPre = Focus::TurnRunLeft;
            }
        }
        else
        if( (pos = rec.find("runright")) != string::npos )
        {
            if( turnPre == Focus::TurnRunRight )
            {
                turnPre = Focus::TurnStop;
            }
            else
            {
                turnPre = Focus::TurnRunRight;
            }
        }
        else
        if( (pos = rec.find("stepleft")) != string::npos )
        {
            if( turnPre == Focus::TurnStepLeft )
            {
                turnPre = Focus::TurnStop;
            }
            else
            {
                turnPre = Focus::TurnStepLeft;
            }
        }
        else
        if( (pos = rec.find("stepright")) != string::npos )
        {
            if( turnPre == Focus::TurnStepRight )
            {
                turnPre = Focus::TurnStop;
            }
            else
            {
                turnPre = Focus::TurnStepRight;
            }
        }
        else
        if( (pos = rec.find("stop")) != string::npos )
        {
            turnPre = Focus::TurnStop;
        }
        
        if( (pos = rec.find("exit")) != string::npos )
        {
            turnPre = Focus::TurnStop;
            ret = -1;
        }
    }
    
    if( aFocus == true )
    {
        autoFocus();
    }
    else
    {
        manualFocus();
    }
    
    return ret;
}

void Focus::autoFocus()
{
    switch( autoFocusState )
    {
        case AutoFocusStart:
        {
            waitCnt = 0;
            afDiffInt = 0.0;
            afDiffOld = 0.0;
            waitTurnThres = MAX_WAIT_TURN;
            waitTurn = 0;
            findFocus.clear();

            // continue with turnLast
            if( turnLast == Focus::TurnStop) 
            {
                turnPre = Focus::TurnStepRight;
            }
            else
            {
                turnPre = turnLast;
            }
            
            driver(turnPre);
            usleep(800000);
            driver(Focus::TurnStop);
            usleep(100000);
            // change dir
            if( turnPre == Focus::TurnStepRight )
            {
                turnPre = Focus::TurnStepLeft;
            }                        
            else
            {
                turnPre = Focus::TurnStepRight;
            }
            driver(turnPre);
            usleep(50000);
            driver(Focus::TurnStop);
            usleep(100000);            
            autoFocusState = AutoFocusScan;
            cout << "autoFocusState: Scan" << endl;            
        }
        break;
        case AutoFocusScan:
        {
            waitCnt++;
            findFocus.push_back(afMeank);
            //if( waitCnt >= MAX_WAIT_COUNT )
            if( waitCnt >= 120 )
            {
                waitCnt = 0;
                
                int indexMax = 0;
                afMeanMax = 0.0;
                for( int i=0; i < findFocus.size(); i++ )
                {
                    if( afMeanMax < findFocus.at(i) )
                    {
                        afMeanMax = findFocus.at(i);
                        indexMax = i;
                    }
                }
                cout << "autoFocus: Max value: " << afMeanMax << ", Max index: " << indexMax << endl;
                
                cout << "autoFocus: Size: " << findFocus.size() << endl;
                
                if( turnPre == Focus::TurnStepRight )
                {
                    turnPre = Focus::TurnStepLeft;
                }                        
                else
                {
                    turnPre = Focus::TurnStepRight;
                }
                driver(turnPre);
                usleep(50000);
                driver(Focus::TurnStop);
                usleep(100000);     
                cout << "autoFocusState: Find Max" << endl;
                autoFocusState = AutoFocusFindMax;
            }
            else
            {
                driver(turnPre);
                usleep(16000);
                driver(Focus::TurnStop);
                usleep(100000);            
            }
        }
        break;
        case AutoFocusFindMax:
        {
            if( afMeank >= (afMeanMax-4.0) )
            {
                cout << "autoFocus: Max value: " << afMeanMax << ", Act value: " << afMeank << endl;                
                cout << "autoFocusState: Stopped" << endl;
                autoFocusState = AutoFocusStopped;
                turnLast = turnPre;
                turnPre = Focus::TurnStop;
                procMsg->sendServerToClient("autodone");                    
            }
            else
            {
                //cout << "autoFocus: Max value: " << afMeanMax << ", Act value: " << afMeank << endl;
                driver(turnPre);
                usleep(8000);
                driver(Focus::TurnStop);
                usleep(100000);    
            }
        }
        break;        
        case AutoFocusStopped:
        {
            usleep(100000);
        }
        break;        
        default:
        break;
    }
}

void Focus::manualFocus()
{
    if( turnPre != turnPost )
    {
        if( turnPre != TurnStop )
        {
            if( ((turnPre == TurnStepRight) &&
                 ((turnLast == TurnStepLeft) || (turnLast == TurnRunLeft))) ||
                ((turnPre == TurnStepLeft) &&
                 ((turnLast == TurnStepRight) || (turnLast == TurnRunRight))) ) 
            {
                driver(turnPre);        
                usleep(100000);
                driver(Focus::TurnStop); 
                state = StateStep;
                cout << "Focus turn" << endl;
            }
            else
            {
                if( (turnPre == TurnStepRight) || (turnPre == TurnStepLeft) ) 
                {
                    state = StateStep;
                    cout << "Focus step" << endl;
                }
                else
                {
                    state = StateRun;
                    driver(turnPre); 
                    cout << "Focus run" << endl;
                }
            }
            turnLast = turnPre;
        }
        else
        {
            driver(Focus::TurnStop); 
            state = StateIdle;
            timecnt = 0;
            cout << "Focus stop" << endl;
        }
        turnPost = turnPre;
    }
    if( (state == StateIdle) || (state == StateRun) )
    {
        usleep( 100000 );
        if( state == StateRun )
        {
            timecnt++;
            if( timecnt >= 30 )
            {
                turnPre = Focus::TurnStop;
                timecnt = 0;
            }
        }
    }
    else
    {
        driver(turnPre);
        usleep( 8000 );
        //usleep( 50000 );
        driver(TurnStop);
        usleep( 100000 );
    }
    
    return;
}

void Focus::driver( TurnType turn )
{
    int fd;
    switch( turn )
    {
        case TurnRunRight:
        case TurnStepRight:
            //fd = open("/sys/class/gpio/gpio12/value", O_WRONLY);
            write(fd1, "1", 1);
            //close(fd);
            //fd = open("/sys/class/gpio/gpio18/value", O_WRONLY);
            write(fd2, "0", 1);
            //close(fd);        
        break;
        case TurnRunLeft:
        case TurnStepLeft:
            //fd = open("/sys/class/gpio/gpio12/value", O_WRONLY);
            write(fd1, "0", 1);
            //close(fd);
            //fd = open("/sys/class/gpio/gpio18/value", O_WRONLY);
            write(fd2, "1", 1);
            //close(fd);
        break;
        case TurnStop:
        default:
            //fd = open("/sys/class/gpio/gpio12/value", O_WRONLY);
            write(fd1, "0", 1);
            //close(fd);
            //fd = open("/sys/class/gpio/gpio18/value", O_WRONLY);
            write(fd2, "0", 1);
            //close(fd);        
        break;
    }
    
    return;
}

