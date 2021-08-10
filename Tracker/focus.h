#ifndef FOCUS_H
#define FOCUS_H

using namespace std;

class Focus
{
   
public:
   enum TurnType { TurnStop = 0, TurnRunRight, TurnRunLeft, 
                   TurnStepRight, TurnStepLeft};
   enum AutoFocusType { AutoFocusStopped = 0, AutoFocusStart, 
                        AutoFocusTurn, AutoFocusCheckDir, AutoFocusFindMax };
   
public:
   Focus();
   Focus(ProcMessage *proc);
   virtual ~Focus();

public:
   void init();
   void deInit();
   void process();
   int processMsg();
   
   void run( TurnType turn ) { turnPre = turn; }

private:
   enum StateType { StateIdle = 0, StateRun, StateStep };
                   
private:
   void driver( TurnType turn );
   void manualFocus();
   void autoFocus();
    
private:
   TurnType turnPre;
   TurnType turnPost;
   TurnType turnLast;
   struct timeval tvStart; 
   struct timeval tvEnd; 
   int steptimeout;
   int fd1, fd2;
   ProcMessage *procMsg;
   StateType state;
   int timecnt;
   bool aFocus;
   double afMeank;
   double afMeanStart;
   double afMeanMax;
   double afMeanFirst;
   double afDiffInt; 
   double afDiffOld; 
   AutoFocusType autoFocusState;
   int waitCnt;
   int checkTurn;
};


#endif
