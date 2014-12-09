#ifndef DEFINES_H
#define DEFINES_H


//#define ARDRONEFRONTCAM
//#define DELFLY_COLORMODE
//#define DELFLY_DISPMODE

#ifdef _PC /*************************     PC     ******************/

#define RUNMODE stereo_textons
#define DELFLY
//#define DUOWEBCAM

//#define DELFLY_WIFI
#define FILECAM

#ifndef DELFLY_COLORMODE

#define VIDEORESULTS
//#define DRAWHIST

//#define GEIGER
#define SGM
//#define BM currently not completely implemented
//#define LONGSEC
#endif

#define HASSCREEN // dont disable in qt debugger!
#ifndef FILECAM
#define VIDEORAW
#endif

#define VIDEOFPS 10

//#define USE_TERMINAL_INPUT // using this will conflict with qt debugging
#define USE_SOCKET

#else /*************************    DRONE     ******************/
#define RUNMODE stereo_textons
#define DELFLY

//#define VIDEORAW

#ifndef DELFLY_COLORMODE

//#define VIDEORESULTS

#define SGM
//#define LONGSEC

#endif // DELFLY_COLORMODE

#define VIDEOFPS 10 // setting this lower than 5 will crash the drone...

//#define USE_TERMINAL_INPUT // using this disables running in background
#define USE_SOCKET
#endif // pc/drone

#endif //DEFINES_H
