
#ifndef DELFLYWIFI_H
#define DELFLYWIFI_H

#include <mutex>
#include <thread>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "defines.h"

class Delfly_WiFi{


private:
    std::mutex g_lockWaitForImage;
    bool copyNewImage;
    std::thread thread_cam;
    void workerThread(void);
    void splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR );

    cv::VideoCapture cam;

public:
    int im_width;
    int im_height;
    int darksize = 0;
    int extracaliboffset = 0;

    bool cams_are_running;
#ifdef DELFLY_COLORMODE
    cv::Mat frameC_mat;
#else
    cv::Mat frameL_mat;
    cv::Mat frameR_mat;

#endif

    bool init (void);
    void start (void) ;
    void waitForImage(void);
    void close (void);



};


#endif //DELFLYWIFI_H
