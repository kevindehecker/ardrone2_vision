#include "socket.h"
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>
#include <string.h> 		/* memset */

#include <stdlib.h>
#include <stdio.h>



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

ssize_t Socket::Readline_socket(void *vptr, size_t maxlen) {
    size_t n, rc;
    char    c, *buffer;
  
    buffer = (char*)vptr;

    for ( n = 1; n < maxlen; n++ ) {
	
	if ( (rc = read(conn_s, &c, 1)) == 1 ) {
	    *buffer++ = c;
	    if ( c == '\n' )
		break;
	}
	else if ( rc == 0 ) {
	    if ( n == 1 )
		return 0;
	    else
		break;
	}
	else {
	    if ( errno == EINTR )
		continue;
	    return -1;
	}
    }

    *buffer = 0;
    return n;
}


/*  Write a line to a socket  */
ssize_t Socket::Writeline_socket(const char * text, size_t n) {
    size_t      nleft;
    ssize_t     nwritten;
	nleft  = n+1; //also send \0
	
    while ( nleft > 0 ) {
	if ( (nwritten = write(conn_s, text, nleft)) <= 0 ) {
	    if ( errno == EINTR )
		nwritten = 0;
	    else
		return -1;
	}
	nleft  -= nwritten;
	text += nwritten;
    }

    return n;
	
}




