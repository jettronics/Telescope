#ifndef PROCMESSAGE_H
#define PROCMESSAGE_H

using namespace std;

#define BUFSIZE 1024

class ProcMessage
{
   
public:
   ProcMessage();
   virtual ~ProcMessage();

public:
   typedef enum
   {
     PipeRead = 0,
     PipeWrite,
     PipeMax
   }
   pipes_type;

   typedef enum
   {
     BlockingAll = 0,
     UnblockingRead,
     UnblockingWrite,
     UnblockingAll
   }
   blocking_type;
   
   void init(blocking_type clienttoserver, blocking_type servertoclient);
   void deInit();
   
   void sendClientToServer( string data );
   void sendServerToClient( string data );
   string receiveClientFromServer();
   string receiveServerFromClient();
   
private:
   int pipeClientToServer[PipeMax];
   int pipeServerToClient[PipeMax];
   char bufferClientFromServer[BUFSIZE];
   char bufferServerFromClient[BUFSIZE];
};


#endif
