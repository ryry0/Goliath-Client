/*
  Author: Ryan - David Reyes
*/
#ifndef TCPIPNIX_H_ 
#define TCPIPNIX_H_
#include <unistd.h>

/*
    The socket that you do data operations on if you're the client and
    have used connectToHost is servSock.

    If you're the server the socket you use for data operations is
    the one returned from acceptConnection().
*/

class TCP
{
  public:
    TCP();
    ~TCP();
    bool connectToHost(const int portNo, const char * IPAddress);
    bool listenToPort(const int portNo);
    unsigned int acceptConnection();

    int sendData(unsigned int writeTo, char * data, const int len);
    int receiveData(unsigned int readFrom, 
        char * data, 
        const int len);
      
    /*
      The folowing functions send and receive data 
      framed with a 4 byte packet that specifies
      the size of that data
    */

    int sendFramedData(unsigned int writeTo, char * data, const int len);
    int receiveFramedData(unsigned int readFrom, char * data);

    int getSocket() const {return servSock;}
    int closeSocket(unsigned int sockArg) {return close(sockArg);}

  private:
    unsigned int servSock; //socket handle
};

#endif // TCPIPNIX_H_
