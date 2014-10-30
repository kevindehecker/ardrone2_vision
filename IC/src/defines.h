#ifndef DEFINES_H
#define DEFINES_H

//#define ARDRONEFRONTCAM
#define DELFLY_COLORMODE

#define RUNMODE none

#ifdef _PC

#define DELFLY
//#define DUOWEBCAM

//#define DELFLY_WIFI
//#define FILECAM

#ifndef DELFLY_COLORMODE


#define GEIGER
//#define SGM
//#define BM currently not completely implemented
//#define LONGSEC
#endif

#define HASSCREEN // dont disable in qt debugger!
#ifndef FILECAM
//#define VIDEORAW
#endif

#ifndef DELFLY_COLORMODE
//#define VIDEORESULTS
#endif
#define VIDEOFPS 10

#define USE_SOCKET
#define TCPPORT 6969

#else // drone

#define DELFLY

#define VIDEORAW

#ifndef DELFLY_COLORMODE
//#define VIDEORESULTS

//#define SGM
#define LONGSEC

#endif // DELFLY_COLORMODE

#define VIDEOFPS 8 // setting this lower than 5 will crash the drone...
#define USE_SOCKET
#define TCPPORT 6969
#endif // pc/drone

#endif //DEFINES_H
