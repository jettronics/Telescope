
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>

#include "procmessage.h"


ProcMessage::ProcMessage()
{

}

ProcMessage::~ProcMessage()
{

}


void ProcMessage::init(blocking_type clienttoserver, blocking_type servertoclient)
{
    pipe(pipeClientToServer);
    pipe(pipeServerToClient);
    
    if( clienttoserver == UnblockingRead )
    {
        fcntl(pipeClientToServer[PipeRead], F_SETFL, O_NONBLOCK);
    }
    if( clienttoserver == UnblockingWrite )
    {
        fcntl(pipeClientToServer[PipeWrite], F_SETFL, O_NONBLOCK);
    }    
    if( clienttoserver == UnblockingAll )
    {
        fcntl(pipeClientToServer[PipeRead], F_SETFL, O_NONBLOCK);
        fcntl(pipeClientToServer[PipeWrite], F_SETFL, O_NONBLOCK);
    }
    if( servertoclient == UnblockingRead )
    {
        fcntl(pipeServerToClient[PipeRead], F_SETFL, O_NONBLOCK);
    }
    if( servertoclient == UnblockingWrite )
    {
        fcntl(pipeServerToClient[PipeWrite], F_SETFL, O_NONBLOCK);
    }
    if( servertoclient == UnblockingAll )
    {
        fcntl(pipeServerToClient[PipeRead], F_SETFL, O_NONBLOCK);
        fcntl(pipeServerToClient[PipeWrite], F_SETFL, O_NONBLOCK);
    }
}
   
void ProcMessage::deInit()
{
    close(pipeClientToServer[PipeRead]);
    close(pipeClientToServer[PipeWrite]);
    close(pipeServerToClient[PipeRead]);
    close(pipeServerToClient[PipeWrite]);    
}
   
void ProcMessage::sendClientToServer( string data )
{
    //cout << "sendClientToServer: " << data << " length: " << data.length() << endl;
    cout.flush();
    if( data.length() > 0 )
    {
        write(pipeClientToServer[PipeWrite], data.data(), data.length());
    }
    else
    {
        string empty = "empty";
        write(pipeClientToServer[PipeWrite], empty.data(), empty.length());
    }
}

void ProcMessage::sendServerToClient( string data )
{
    //cout << "sendServerToClient: " << data << " length: " << data.length() << endl;
    cout.flush();
    if( data.length() > 0 )
    {
        write(pipeServerToClient[PipeWrite], data.data(), data.length());
    }
    else
    {
        string empty = "empty";
        write(pipeServerToClient[PipeWrite], empty.data(), empty.length());
    }    
}

string ProcMessage::receiveClientFromServer()
{
    bufferClientFromServer[0] = 0;
    int iPipeRead = read(pipeServerToClient[PipeRead], bufferClientFromServer, BUFSIZE-1);
    if( (iPipeRead > 0) && (iPipeRead < BUFSIZE) )
    {
        bufferClientFromServer[iPipeRead] = 0;
        //cout << "receiveClientFromServer: Pipe read: " << bufferClientFromServer << endl;
    }
    cout.flush();
    return bufferClientFromServer;
}

string ProcMessage::receiveServerFromClient()
{
    bufferServerFromClient[0] = 0;
    int iPipeRead = read(pipeClientToServer[PipeRead], bufferServerFromClient, BUFSIZE-1);
    if( (iPipeRead > 0) && (iPipeRead < BUFSIZE) )
    {
        bufferServerFromClient[iPipeRead] = 0;
        //cout << "receiveServerFromClient: Pipe read: " << bufferServerFromClient << endl;
    }
    cout.flush();
    return bufferServerFromClient;
}

