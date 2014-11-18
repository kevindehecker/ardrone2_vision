#include <mutex>
#include <thread>

#include "defines.h"
#include "StereoAlg.h"
#include "smoother.h"
#include "socket.h"
#include "textons.h"
#ifdef FILECAM
#include "Filecam.h"
#endif
#ifdef DUOWEBCAM
#include "DuoWebcam.h"
#endif
#ifdef DELFLY_WIFI
#include "Delfly_wifi.h"
#endif
#ifdef DELFLY
#include "Delfly.h"
#endif
#ifdef ARDRONEFRONTCAM
#include "ARDroneCam.h"
#endif

#include "stopwatch.h"

/***********Enums****************/
enum modus_t {none, stereo_only, textons_only, stereo_textons};


/***********Variables****************/
bool connectionAccepted;
std::mutex g_lockComm;
int commdata_gt;
int commdata_nn;
char key = 0;
std::string msg;
cv::Mat resFrame;
cv::VideoWriter outputVideo;
cv::VideoWriter outputVideoResults;
stopwatch_c stopWatch;
modus_t mode;
Socket tcp;


Socket tcp_test;

#ifdef FILECAM
FileCam svcam;
#else
#ifdef DUOWEBCAM
DuoWebCam svcam;
#endif
#ifdef DELFLY_WIFI
Delfly_WiFi svcam;
#else
#ifdef DELFLY
DelFly svcam;
#endif
#endif
#endif

#ifdef ARDRONEFRONTCAM
ARDronecam ARfrontcam;
#endif

stereoAlg stereo;
Textons textonizer;

/*******Private prototypes*********/
void process_video();
void TerminalInputThread();
int main( int argc, char **argv);
void saveStereoPair();

void combineImage(cv::Mat resFrame, cv::Mat smallsourceimage, int x, int y,int width, int height, bool convertRGB);
void commOutThread();
void commInThread();

/************ code ***********/
void combineAllImages() {

#ifndef DELFLY_COLORMODE
#if defined(DELFLY)
    combineImage(resFrame,stereo.DisparityMat,svcam.im_width*4,0,stereo.DisparityMat.cols*2,stereo.DisparityMat.rows*2,false);
    combineImage(resFrame,svcam.frameL_mat,0,0,svcam.frameL_mat.cols*2,svcam.frameL_mat.rows*2,true);
    combineImage(resFrame,svcam.frameR_mat,svcam.im_width*2,0,svcam.im_width*2,svcam.im_height*2,true);
    combineImage(resFrame,textonizer.graphFrame,0,svcam.im_height*2,svcam.im_width*6,svcam.im_height*2,false);
#else
    combineImage(resFrame,stereo.DisparityMat,svcam.im_width,0,stereo.DisparityMat.cols,stereo.DisparityMat.rows,false);
    combineImage(resFrame,svcam.frameL_mat,0,0,svcam.frameL_mat.cols/2,svcam.frameL_mat.rows/2,true);
    combineImage(resFrame,svcam.frameR_mat,svcam.im_width/2,0,svcam.im_width/2,svcam.im_height/2,true);
    combineImage(resFrame,textonizer.graphFrame,0,svcam.im_height/2,svcam.im_width*1.5,svcam.im_height/2,false);
#endif
#endif

}

void combineImage(cv::Mat resFrame, cv::Mat smallsourceimage, int x, int y,int width, int height, bool convertRGB) {

    cv::Point p1(x, y);
    cv::Point p2(x+width, y+height);
    cv::Point size(width, height);

    cv::Mat rgbsmallsourceimage(smallsourceimage.cols,smallsourceimage.rows,CV_8UC3);
    if (convertRGB) {
        cvtColor(smallsourceimage,rgbsmallsourceimage,CV_GRAY2RGB,0);
    } else {
        rgbsmallsourceimage = smallsourceimage;
    }


    cv::Mat roi = cv::Mat(resFrame, cv::Rect(p1, p2));

    if (rgbsmallsourceimage.size().width != width || rgbsmallsourceimage.size().height != height) {
        cv::resize(rgbsmallsourceimage,roi,size);
    } else {
        rgbsmallsourceimage.copyTo(roi);
    }
}

/*
 * process_video: retrieves frames from camera, does the processing and handles the IO.
 * Skips frames if processing takes too long.
 * the camera.
 */
#ifdef DELFLY_COLORMODE
void process_video() {
    stopWatch.Start();
    int frames = 0;

    while (key != 27 && svcam.cams_are_running) // ESC
    {

        svcam.waitForImage();  //synchronized grab stereo frame:
#ifdef VIDEORAW
        outputVideo.write(svcam.frameC_mat);
#endif
#ifdef HASSCREEN
        cv::imshow("Results",svcam.frameC_mat);
        key = cv::waitKey(10);
#endif

        if (key==114) {frames=0;stopWatch.Restart();msg="Reset";key=0;} //[r]: reset stopwatch

        frames++;
        float time = stopWatch.Read()/1000;
        std::cout << "FPS: " << frames /(time) << "\n";

    } // main while loop
    
#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}
#else // stereo vision instead of color
void process_video() {
    stopWatch.Start();
    int frames = 0;
    //main while loop:
    while (key != 27 && svcam.cams_are_running) // ESC
    {

        svcam.waitForImage();  //synchronized grab stereo frame:

        bool stereoOK=true;
        float avgDisparity=0;
        if (mode==stereo_only || mode==stereo_textons) {
            //stereo is turned on
            stereoOK = stereo.calcDisparityMap(svcam.frameL_mat,svcam.frameR_mat); // calc the stereo groundtruth
            avgDisparity=stereo.avgDisparity;
            commdata_gt = avgDisparity; // tmp for comm testing
        }

        if ((mode==textons_only || mode==stereo_textons) && stereoOK) {
            textonizer.getTextonDistributionFromImage(svcam.frameL_mat,avgDisparity);  //perform the texton stuff
        }


#ifdef VIDEORAW
        //combine stereo pair if needed
        if (mode==none  || mode==textons_only) {
            stereo.combineImage(svcam.frameL_mat,svcam.frameR_mat);
        } else {
#ifndef LONGSEC
            stereo.combineImage(svcam.frameL_mat,svcam.frameR_mat);
#endif // LONGSEC

        }
        outputVideo.write(stereo.frameC_mat);

#endif //VIDEORAW

#if defined(HASSCREEN) || defined(VIDEORESULTS)
        textonizer.drawGraph(msg);
        combineAllImages();
#ifdef HASSCREEN
        cv::imshow("Results", resFrame);
#endif
#ifdef VIDEORESULTS
        outputVideoResults.write(resFrame);
#endif
#ifdef HASSCREEN
        key = cv::waitKey(10);
#endif
#endif

        switch ( mode) {
        case none:
            msg= "none";
            break;
        case textons_only:
            msg= "textons";
            break;
        case stereo_only:
            msg= "stereo";
            break;
        case stereo_textons:
            msg= "textons+stereo";
            break;
        }
        if (key==10) {textonizer.retrainAll();key=0;msg="Learn";} //      [enter]: perform learning
        if (key==115) {textonizer.saveRegression();key=0;msg="Save";} //  [s]: save
        if (key==99) {textonizer.initLearner(true);key=0;msg="Clear";} // [c]: clear
        if (key==108) {textonizer.reload();key=0;msg="Reload";} //        [l]: reload
        if (key==48) {mode=none;key=0;} //                                [0]: switch stereo and texton calculation off
        if (key==49) {mode=textons_only;key=0;} //                        [1]: switch stereo mode off, textons on
        if (key==50) {mode=stereo_only;key=0;} //                         [2]: switch stereo mode on, textons off
        if (key==51) {mode=stereo_textons;key=0;} //                      [3]: switch both stereo and textons calucation on
        if (key==32) {saveStereoPair();key=0;} //                         [ ]: save stereo image to bmp
        if (key==114) {frames=0;stopWatch.Restart();msg="Reset";key=0;} //[r]: reset stopwatch


#ifdef USE_SOCKET        
        g_lockComm.unlock();
#endif

        frames++;
        float time = stopWatch.Read()/1000;
        std::cout << "FPS: " << frames /(time) << "\n";

    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}
#endif

int saveid=1;
void saveStereoPair() {
    char str[64];

    sprintf(str,"left%d.png", saveid);
    cv::imwrite( str, svcam.frameL_mat );
    sprintf(str,"right%d.png", saveid);
    cv::imwrite( str, svcam.frameR_mat );
    saveid++;
}

void TerminalInputThread() {
#ifndef HASSCREEN
    usleep(1000000); // let the qt debug output pass through. Hmm doesnt work.
    while(svcam.cams_are_running) {
        std::cin >> key;
        if (key==120 || key==113) {
            key=27; // translate x to esc
            svcam.cams_are_running = false;
            std::cout << "Exiting\n";
        }
    }
#endif
}

void commInThread() {
    char data[1];
    while (svcam.cams_are_running && connectionAccepted ) {
        int n = tcp.Read_socket(data,1);
        if (n>0 && data[0] != '\r' && data[0] != '\n') {
            if (data[0]>0) {
                key = data[0];
                if (key==120 || key==113) {
                    key=27; // translate x to esc
                    svcam.cams_are_running = false;
                    std::cout << "Exiting\n";
                }
            }
        } else {
            usleep(100);
        }
    }
}

void commOutThread() {
#ifdef USE_SOCKET
    if (!tcp.initSocket(TCPPORT)) {
        std::cout << "Error initialising connection\n";
        return;
    }
    std::cout << "Opened socket @" << TCPPORT << std::endl;
    connectionAccepted = true;

    std::thread thread_commIn(commInThread);

    while (svcam.cams_are_running) {

        g_lockComm.lock();

        ICDataPackage out;
        out.avgdisp_gt = commdata_gt;
        out.avgdisp_nn = textonizer.getLast_nn();
        out.endl = 0;
        std::cout << "gt: " << out.avgdisp_gt << " nn: " << out.avgdisp_nn << std::endl;

        char * c = (char *) (void *) &out; // struct in c++ will not come out of the kast.
        tcp.Write_socket(c, sizeof(out));
    }
    connectionAccepted = false;
    tcp.closeSocket();
    thread_commIn.join();

    std::cout << "CommOutThread exiting.\n";
#endif
}

int main( int argc, char **argv )
{

    /*****init the camera*****/
#ifdef DUOWEBCAM
    int cam_left_id;
    int cam_right_id;

    if (argc != 3) {
        std::cout << "Usage: DisplayImage cam_left, cam_right\n";
        std::cout << "Now using default values.\n";

        cam_left_id=0;
        cam_right_id=2;

        //return -1;
    } else {

        cam_left_id= atoi(argv[1]);
        cam_right_id = atoi(argv[2]);
    }
    if (!svcam.init(cam_left_id,cam_right_id)) {return 0;}
#else
    if (!svcam.init()) {return 0;}
#endif

    /*****init the visual bag of words texton methode*****/
    std::cout << "Initialising textonizer\n";
    if (!textonizer.init()) {return 0;}

    /*****Start capturing images*****/
    std::cout << "Start svcam\n";
    svcam.start();
#ifdef ARDRONEFRONTCAM
    std::cout << "Starting ARDRone cam\n";
    ARfrontcam.start();
#endif

    /***init the stereo vision (groundtruth) algorithm ****/
    std::cout << "Initialising stereo algorithm\n";
    stereo.init(svcam.im_width, svcam.im_height); //sv initialisation can only happen after cam start, because it needs the im dims

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
#endif

    std::thread thread_TerminalInput(TerminalInputThread);

#ifdef USE_SOCKET
    std::thread thread_comm(commOutThread);
    g_lockComm.unlock();
#endif
#if defined(HASSCREEN) || defined(VIDEORESULTS)
#ifdef DUOWEBCAM
    resFrame = cv::Mat::zeros(svcam.im_height, svcam.im_width*1.5,CV_8UC3);
#else
    resFrame = cv::Mat::zeros(4*svcam.im_height, 6*svcam.im_width,CV_8UC3);
#endif
#endif

    /*****init the video writer*****/
#ifdef VIDEORAW

#ifdef DELFLY_COLORMODE
    cv::Size size(svcam.im_width,svcam.im_height);
#ifdef pc
    //outputVideo.open("appsrc ! ffmpegcolorspace ! ffenc_mpeg4 ! avimux ! filesink location=testvid.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,true);
    outputVideo.open("video.avi",CV_FOURCC('M','P','E','G'),VIDEOFPS,size,true);
#else
    //outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc ! avimux ! filesink location=testvid.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,true);
    outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.2 port=5000",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,true);
#endif

#else // stereo vision mode
    cv::Size size(svcam.im_width*2,svcam.im_height); // for dsp encoding, ensure multiples of 16
#ifdef pc
    //outputVideo.open("appsrc ! ffmpegcolorspace ! ffenc_mpeg4 ! avimux ! filesink location=video_dsp.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
    outputVideo.open("video.avi",CV_FOURCC('M','P','E','G'),VIDEOFPS,size,false);
#else
    //outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc mode=1 ! avimux ! filesink location=video_dsp.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
    outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc mode=1 ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.2 port=5000",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
#endif

#endif

   if (!outputVideo.isOpened())
   {
           std::cout << "!!! Output stereo video could not be opened" << std::endl;
           return 0;
   }
#endif
#ifdef VIDEORESULTS
   cv::Size sizeRes(resFrame.cols,resFrame.rows);
#ifdef pc
   outputVideoResults.open("videoResults.avi",CV_FOURCC('F','M','P','4'),VIDEOFPS,sizeRes,true);
#else
   outputVideoResults.open("appsrc ! ffmpegcolorspace ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.2 port=5000",CV_FOURCC('H','O','E','R'),VIDEOFPS,sizeRes,true);
#endif

   if (!outputVideoResults.isOpened())
   {
           std::cout << "!!! Output result video could not be opened" << std::endl;
           return 0;
   }
#endif

    /***Go and run forever! ****/
   mode = RUNMODE;
   msg="";
   process_video();
    
    /*****Close everything down*****/
    svcam.close();
#ifdef ARDRONEFRONTCAM
    ARfrontcam.close();
#endif

#ifdef USE_SOCKET
    std::cout << "Closing socket\n";
    if (tcp.closeSocket()) {
        std::cout << "Waiting socket tread\n";
        thread_comm.join();
        std::cout << "Socket thread closed\n";
    }
    else {
        thread_comm.detach();	//accept is blocking if no connection
        std::cout << "Socket thread detached\n";
    }
#endif
    thread_TerminalInput.detach();	//cin is blocking
    return 0;
}


