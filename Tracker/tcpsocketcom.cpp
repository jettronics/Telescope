
#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <fcntl.h>

#include "tcpsocketcom.h"


TcpSocketCom::TcpSocketCom(int port)
    : sockfd(0)
    , newsockfd(0) 
    , portno(port) //51717
    , clilen(0)
    , initfailed(false)
{
 
}


TcpSocketCom::~TcpSocketCom()
{

}

int TcpSocketCom::init()
{
    cout << "Using port " << portno  << endl;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        cout << "Socket Error: " << sockfd << endl;
        initfailed = true;
        return -1;
    }
    
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( portno );
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
    {
        cout << "Bind Error" << endl;
        initfailed = true;
        return -1;
    }
    listen(sockfd,5);
    clilen = sizeof(cli_addr); 
    
    cout << "Waiting for new client..." << endl;
    if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
    {
        cout << "Accept Error: " << newsockfd  << endl;
        cout.flush();
        return -1;
    }
    
    cout << "Opened new communication with client"  << endl;
    
    cout << "None blocking TCP handler"  << endl;
    fcntl(newsockfd, F_SETFL, O_NONBLOCK);
    
    initfailed = false;   
    return 0;
}

void TcpSocketCom::process()
{
    if( initfailed == false )
    {
        string recData = getData();
        
        if( recData != "Empty" )
        {
            cout << "Data received: " << recData << endl;;
            cout.flush();
            //procMsg->sendClientToServer(recData);
        }
        else
        {
            // Do nothing                      
        }
    }
    return;
}

void TcpSocketCom::deInit()
{
    close( newsockfd );
}

string TcpSocketCom::getData() 
{
    char buffer[100];
    int n = 0;

    if( initfailed == false )
    {
        n = read(newsockfd, buffer, 100);
    }
    
    if ( n < 0 )
    {
        //cout << "\nRead Error: " << n;
        //cout.flush();
        strcpy(buffer,"Empty\0");
    }
    else
    if( n == 0 )
    {
        strcpy(buffer,"Empty\0");
    }
    else
    {
        buffer[n] = '\0';
        cout << "Socket read: " << buffer << endl;
    }

    return buffer;    
}

void TcpSocketCom::sendData(string *send)
{
    int n;

    //cout << "\nsend: " << *send << "\n";
    if ( (n = write( newsockfd, send->data(), send->length() ) ) < 0 )
    {
        cout << "Write Error: " << n << endl;
    }
    return;
}
