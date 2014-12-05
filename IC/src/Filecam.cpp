#include "Filecam.h"
#ifdef FILECAM
#include <iostream>
#include <unistd.h> // sleep

#include <opencv2/contrib/contrib.hpp>



bool FileCam::init () {

    video = cv::VideoCapture("/home/houjebek/AfstudeerData/Experiments/CyberZoo/AutonomousFlightGroundtruth/video_dsp.avi");
    //video = cv::VideoCapture("/home/houjebek/grive/biebfootage/biebwalk1/video.avi");

    if (!video.isOpened()) {
        std::cerr << "Error opening video file!\n";
        return false;
    } else {
        im_width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH)/2;
        im_height = (int)video.get(CV_CAP_PROP_FRAME_HEIGHT);
        return true;
    }
}

void FileCam::start () {
    cams_are_running=true;
    thread_cam = std::thread(&FileCam::workerThread,this);    
    waitForImage();
    waitForImage();
    std::cout << "File opened!\n";
}

void FileCam::waitForImage() {

    g_lockWaitForImage2.lock();
    g_lockWaitForImage1.unlock();

}

void FileCam::close () {
    cams_are_running = false;
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage1.unlock();
	thread_cam.join();    
    video.release();
}


void FileCam::splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR ) {
    *frameL = cv::Mat(frameC,cv::Rect(0,0,frameC.cols/2,frameC.rows));
    *frameR = cv::Mat(frameC,cv::Rect(frameC.cols/2,0,frameC.cols/2,frameC.rows));
}

void FileCam::workerThread() {

    cv::Mat frameC = cv::Mat::zeros(im_height,im_width*2, CV_8UC1);
    cv::Mat frameL = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    stopWatch.Start();

    int skipstart = 150;
    //skip start
    for (int i =0; i < skipstart;i++)
        video >> frameC;

    while (cams_are_running)  {

#ifdef HASSCREEN
        //arrange speed to be ~10fps
        float time = stopWatch.Read();
        time = 100 - time/1000;
        if (time > 0)  {usleep((int)time*1000);}
        stopWatch.Restart();
#endif
        g_lockWaitForImage1.lock();

        video >> frameC;

        if (frameC.empty())
        {
            cams_are_running=false;
            g_lockWaitForImage2.unlock();
            break;
        }

        cvtColor(frameC,frameC,CV_RGB2GRAY,CV_8UC1);
        splitIm(frameC,&frameL,&frameR);

        frameL.copyTo(frameL_mat);
        frameR.copyTo(frameR_mat);


        g_lockWaitForImage2.unlock();




    } // while loop


}

#endif
