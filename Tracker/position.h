#ifndef POSITION_H
#define POSITION_H

using namespace std;

class Position
{
   
public:
   Position();
   virtual ~Position();

public:
   virtual void init();
   void deInit();
   virtual void process();
   void setFixedAzm( int azm );
   void setFixedAlt( int alt );
   void setFixedRate( char rate ) { fixedRate = rate; }
   char getFixedRate() { return fixedRate; }
   void setFixedRateAzm( char rate ) { fixedRateAzm = rate; }
   char getFixedRateAzm() { return fixedRateAzm; }
   void setFixedRateAlt( char rate ) { fixedRateAlt = rate; }
   char getFixedRateAlt() { return fixedRateAlt; }
   void setVariableAzm( int azm );
   void setVariableAlt( int alt );
   
protected:
   int filestream;
   char rxBuffer[1024];
   char arrBuf[8];
   char arrBufAlt[8];
   char arrBufAzm[8];
   char fixedRate;
   char fixedRateAlt;
   char fixedRateAzm;
   char lastDirVarAzm;
   char lastDirVarAlt;
   char lastDirFixAzm;
   char lastDirFixAlt;
   bool msgReceived;
   bool sendAzm;
   int waitReceived;
   bool waitTurnAzm;
   int waitTurnAzmCount;
   bool waitTurnAlt;
   int waitTurnAltCount;    
};

class PositionUsb : public Position
{
   
public:
   PositionUsb();
   virtual ~PositionUsb();

public:
   void init();
   void process();
   
};

#endif
