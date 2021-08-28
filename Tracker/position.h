#ifndef POSITION_H
#define POSITION_H

using namespace std;

class Position
{
   
public:
   
   
public:
   Position();
   virtual ~Position();

public:
   void init();
   void deInit();
   void process();
   void setFixedAzm( int azm );
   void setFixedAlt( int alt );
   void setFixedRate( char rate ) { fixedRate = rate; }
   void setVariableAzm( int azm );
   void setVariableAlt( int alt );
   
private:
   int filestream;
   //vector<string> strSend;
   //vector<array<char, 8>> arrSend;
   char rxBuffer[1024];
   char arrBuf[8];
   char arrBufAlt[8];
   char arrBufAzm[8];
   char fixedRate;
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


#endif
