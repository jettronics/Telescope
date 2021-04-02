#ifndef TCPSOCKETCOM_H
#define TCPSOCKETCOM_H

using namespace std;

class TcpSocketCom
{
public:
    TcpSocketCom(int port);
    virtual ~TcpSocketCom(); 

public:
    int init();
    void process();
    void deInit();
   
public:    
    string getData();
    void sendData(string *send);

private:
    int sockfd; 
    int newsockfd; 
    int portno; 
    int clilen;
    struct sockaddr_in serv_addr;
    struct sockaddr_in cli_addr;
    bool initfailed;
};


#endif 
