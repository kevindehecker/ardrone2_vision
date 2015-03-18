#include "socket.h"
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>
#include <string.h> 		/* memset */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "defines.h"


int Socket::closeSocket(void) {
	return close(conn_s);
}

int Socket::shutdownSocket(void) {
    return shutdown(conn_s,1);
}

int Socket::initSocket(unsigned int tcpport) {

    /*  Create the listening socket  */
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        fprintf(stderr, "tcp server: Error creating listening socket.\n");
        return -1;
    }


    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(tcpport);

	/*  Bind our socket addresss to the 
	listening socket, and call listen()  */

    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) {
        fprintf(stderr, "tcp server: Error calling bind()\n");
        exit(EXIT_FAILURE);
    }

    if ( listen(list_s, LISTENQ) < 0 ) {
        fprintf(stderr, "tcp server: Error calling listen()\n");
        exit(EXIT_FAILURE);
    }

	/*  Wait for a connection, then accept() it  */	
	if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 ) {
	    fprintf(stderr, "tcp server: Error calling function accept()\n");
	    return -1;
	}
	printf("Connected!\n");
	return 1;
}

/*  Read a line from a socket  */


bool Socket::Read_socket(char * c, size_t maxlen) {
    int n = 0;
    while (n<maxlen) {
        int tmpn = read(conn_s, c+n, maxlen-n);
        n+=tmpn;
    }
    return true;
}



bool Socket::Write_socket(char * c, size_t n) {

    while ( n > 0 ) {
        int nwritten = 0;
        if ( (nwritten = write(conn_s, c, n)) <= 0 ) {
            if ( errno == EINTR )
                nwritten = 0;
            else
                return false;
        }
        n -= nwritten;
        c += nwritten;
    }
}

void Socket::Unlock() {
    g_lockComm.unlock();
}

void Socket::Init(char *keyp, bool *cams_are_running) {
    closeThreads = false;
    thread_comm  = std::thread(&Socket::commOutThread,this);
    g_lockComm.unlock();
	this->cams_are_running = cams_are_running;
    key = keyp;

}

void Socket::Close() {
    std::cout << "Closing socket\n";

    g_lockComm.unlock();
    usleep(10000);
    if (closeSocket()) {
        std::cout << "Waiting socket tread\n";
        thread_comm.join();
        std::cout << "Socket thread closed\n";
    }
    else { // probably was never connected and hanging on initSocket()
        thread_comm.detach();	//accept is blocking if no connection
        std::cout << "Socket thread detached\n";
    }
}

void Socket::commInThread() {
    char data[1];
    while (*cams_are_running && connectionAccepted ) {
        int n = Read_socket(data,1);
        if (n>0 && data[0] != '\r' && data[0] != '\n') {
            std::cout << "TCP received " << data[0] << std::endl;
            if (data[0]>0) {
                *key = data[0];
                if (*key==120 || *key==113) {
                    *key=27; // translate x to esc
                    *cams_are_running = false;
                    std::cout << "Exiting\n";
                }
            }
        } else {
            usleep(100);
        }
    }
    std::cout << "CommInThread exiting.\n";
}

void Socket::commOutThread() {
    if (!initSocket(TCPPORT)) {
        std::cout << "Error initialising connection\n";
        return;
    }
    std::cout << "Opened socket @" << TCPPORT << std::endl;
    connectionAccepted = true;

    std::thread thread_commIn(&Socket::commInThread,this);

    while (*cams_are_running) {

        g_lockComm.lock();

        ICDataPackage out;
        out.avgdisp_gt = commdata_gt;
        out.avgdisp_gt_stdev = commdata_gt_stdev;
		out.avgdisp_est = commdata_est;
        out.fps = commdata_fps;
        out.endl = 0;
		std::cout << "gt: " << out.avgdisp_gt << " est: " << out.avgdisp_est << std::endl;

        char * c = (char *) (void *) &out; // struct in c++ will not come out of the kast.
        Write_socket(c, sizeof(out));
    }

    connectionAccepted = false;
    closeSocket();
    thread_commIn.detach(); // join seems to block


    std::cout << "CommOutThread exiting.\n";
}



