#ifndef DEFINES_H
#define DEFINES_H


//#define ARDRONEFRONTCAM // switch to single camera mode, ARDrone2 front cam
//#define DELFLY_COLORMODE // Switch to single camera mode, the Delflycamera in color mode (needs to be programmed!)
//#define DELFLY_DISPMODE // Switch to single camera mode, the Delflycamera in disparity output mode (needs to be programmed!)

#ifdef _PC /*************************     PC     ******************/

#define RUNMODE stereo_textons // start with this mode (enum defined in IC.cpp)
#define DELFLY //use the Delfly stereo cam over usb2serial
//#define DUOWEBCAM // use the double webcam stereo set up

//#define DELFLY_WIFI // use the delfly stereo cam, while it is connected to the ardrone2, streaming over wifi
#define FILECAM // use a video as source instead of the camera. The file name is defined in filecam.cpp

#ifndef DELFLY_COLORMODE

//#define VIDEORESULTS // show the main results window
//#define DRAWVIZS    //show secundairy results (histograms and texton visulaisations
#define EXPORT //create export.txt and seperate stereo pair png images

//#define GEIGER // use libelas Geiger stereo algorithm
#define SGM  //use OpenCV Semi Global Matching stereo algorithm
//#define BM currently not completely implemented
//#define LONGSEC // use Kirk's implementation of LONGSEC stereo algorithm
#endif

//#define HASSCREEN // dont disable in qt debugger!
#ifndef FILECAM
#define VIDEORAW // write the raw video footage from the camera to a video file
#endif

#define VIDEOFPS 10 // the estimated frame rate of the video used for creating output videos

//#define USE_TERMINAL_INPUT // using this will conflict with qt debugging
#define USE_SOCKET // communication to the pprz IC module

#else /*************************    DRONE     ******************/
#define RUNMODE stereo_textons
#define DELFLY

#define VIDEORAW

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
