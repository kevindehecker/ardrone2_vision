#include "delfly_wifi.h"
#include <iostream>
#include "rs232.h"

#include "stopwatch.h"

bool Delfly_WiFi::init () {
	darksize = 0;
	extracaliboffset = 0;
      cam = cv::VideoCapture ("udpsrc uri=udp://0.0.0.0:5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)MP4V-ES, payload=(int)96, clock-base=(uint)2693349430\" ! gstrtpjitterbuffer drop-on-latency=false latency=0 ! rtpmp4vdepay ! ffdec_mpeg4 ! ffmpegcolorspace ! appsink");
      if(!cam.isOpened())
      {
          std::cerr << "ERROR: Could not open ARDrone camera\n";
          return false;
      } else {return true;}
}

void Delfly_WiFi::start () {
    cams_are_running=true;
    thread_cam = std::thread(&Delfly_WiFi::workerThread,this);
    waitForImage(); // make sure to receive an image before proceeding, to have the width/height variables set
    waitForImage();
    std::cout << "Delfly wifi camera thread started!\n";
}

void Delfly_WiFi::waitForImage() {
    copyNewImage = true;
    g_lockWaitForImage.lock();
}

void Delfly_WiFi::close () {
    cams_are_running = false;
    g_lockWaitForImage.unlock();
    thread_cam.join();
    cam.release();
}

void Delfly_WiFi::workerThread() {
    int sv_width=256;
    im_width = sv_width/2;
    im_height = 96;


#ifdef DELFLY_COLORMODE
    cv::Mat frameYUYV = cv::Mat::zeros(im_height,im_width, CV_8UC2);
#else
    cv::Mat frameL = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(im_height,im_width, CV_8UC1);
#endif

    cv::Mat frame_mat;

    while (cams_are_running)  {

        bool res =  cam.read(frame_mat);
        std::cout << "Frame read\n";
        if(!res)
        {
            fprintf( stderr, "!!! cvQueryFrame ARDrone failed!\n" );
            return;
        }

        //cv::imshow("ARDrone", frame_mat); // for testing!

        if (copyNewImage) { // if the main thread asks for a new image
#ifdef DELFLY_COLORMODE
            cv::cvtColor(frameYUYV,frameC_mat,  CV_YUV2RGB_YVYU );
#else
            //cv::cvtColor(frame_mat,frame_mat,  CV_RGB2GRAY,CV_8UC1 );

            splitIm(frame_mat,&frameL,&frameR);


            cv::cvtColor(frameL,frameL,  CV_RGB2GRAY,CV_8UC1 );
            cv::cvtColor(frameR,frameR,  CV_RGB2GRAY,CV_8UC1 );
            //  cv::imshow("ARDroneL", frameL); // for testing!
            // cv::imshow("ARDroneR", frameR); // for testing!

            frameL.copyTo(frameL_mat);
            frameR.copyTo(frameR_mat);

#endif

            copyNewImage = false;
            g_lockWaitForImage.unlock();
        }


    } // while loop

    //std::cout << "Exit delfly cam thread\n";
}

void Delfly_WiFi::splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR ) {
    *frameL = cv::Mat(frameC,cv::Rect(0,0,frameC.cols/2,frameC.rows));
    *frameR = cv::Mat(frameC,cv::Rect(frameC.cols/2,0,frameC.cols/2,frameC.rows));
}

int Delfly_WiFi::getImHeight() {
    return im_height;
}

int Delfly_WiFi::getImWidth() {
    return im_width;
}
