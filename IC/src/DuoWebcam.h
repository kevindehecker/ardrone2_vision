
#ifndef DUOCAM_H
#define DUOCAM_H

#include "defines.h"
#include <opencv2/contrib/contrib.hpp>

#include <thread>
#include <mutex>

/*
 * This class will interface with a Webcam stereocam over USB (2 seperate usb webcams that is)
 *
 */
class DuoWebCam{


private:
    //Stereo vision camera handling stuff:
    std::mutex g_lockL;
    std::mutex g_lockWaitForImageL;
    std::mutex g_lockR;
    std::mutex g_lockWaitForImageR;
    bool copyNewImage;

    //calibration matrices
    CvMat *_mx1;
    CvMat *_my1;
    CvMat *_mx2;
    CvMat *_my2;
    int darksize = 80;
    int extracaliboffset = 4;
    int camLeft_id;
    int camRight_id;

    std::thread tL;
    std::thread tR;

    IplImage* make_it_gray(IplImage* frame);
    void camThreadL(void);
    void camThreadR(void);


public:
    int im_width = 640;
    int im_height = 480;
    int im_fps =VIDEOFPS;
    bool cams_are_running;
    cv::Mat frameL_mat;
    cv::Mat frameR_mat;

    bool init ( int cam_left_id,  int cam_right_id);
    void start (void) ;
    void waitForImage(void);
    void close (void);

};

#endif //DUOCAM_H
