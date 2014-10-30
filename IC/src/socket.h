#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */
#include <arpa/inet.h>        /*  inet (3) funtions         */


/*  Global constants  */
#define MAX_LINE           (1000)
#define LISTENQ        (1024)   /*  Backlog for listen()   */

struct ICDataPackage {
    int avgdisp_gt;
    int avgdisp_nn;
    char endl;             // endl fix :)
};
extern struct ICDataPackage video_impl;

class Socket{
private:
    /*  Global variables  */
    int       list_s;                /*  listening socket          */
    int       conn_s;                /*  connection socket         */
    struct    sockaddr_in servaddr;  /*  socket address structure  */
    char      buffer[MAX_LINE];      /*  character buffer          */
    char     *endptr;                /*  for strtol()              */

public:
    /*  Function declarations  */
	int initSocket(unsigned int port) ;
	ssize_t Readline_socket(void *vptr, size_t maxlen);
    bool Read_socket(char * c, size_t maxlen);
    bool Write_socket(char * c, size_t n) ;
	ssize_t Writeline_socket(const char * text, size_t n);
	int closeSocket(void);
    int shutdownSocket(void);




};
#endif  /*  SOCKET_H  */

