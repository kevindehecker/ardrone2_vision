#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <mutex>
#include <thread>

/*  Global constants  */
#define MAX_LINE           (1000)
#define LISTENQ        (1024)   /*  Backlog for listen()   */
#define TCPPORT 6969
struct ICDataPackage {
    int avgdisp_gt;
    int avgdisp_gt_stdev;
    int avgdisp_nn;
    float fps;
    char endl;             // endl fix, makes it worker nicer in terminal for debugging :)
};
extern struct ICDataPackage video_impl;

/*
 * This class will interface with to the IC pprz module over tcp/ip
 *
 */
class Socket{
private:
    /*  Global variables  */
    int       list_s;                /*  listening socket          */
    int       conn_s;                /*  connection socket         */
    struct    sockaddr_in servaddr;  /*  socket address structure  */
    char      buffer[MAX_LINE];      /*  character buffer          */
    char     *endptr;                /*  for strtol()              */
    std::thread thread_comm;
    std::mutex g_lockComm;
    bool connectionAccepted;
    char *key;
    bool *cams_are_running;
    bool closeThreads;


    /*  Function declarations  */
    int initSocket(unsigned int port) ;
    ssize_t Readline_socket(void *vptr, size_t maxlen);
    bool Read_socket(char * c, size_t maxlen);
    bool Write_socket(char * c, size_t n) ;
    ssize_t Writeline_socket(const char * text, size_t n);
    int closeSocket(void);
    int shutdownSocket(void);
    void commInThread();
    void commOutThread();

public:
    /*  Global variables  */
    int commdata_gt;
    int commdata_gt_stdev;
    int commdata_nn;
    float commdata_fps;

    /*  Function declarations  */
    void Close();
    void Init(char *key, bool *cams_are_running);
    void Unlock();


};
#endif  /*  SOCKET_H  */

