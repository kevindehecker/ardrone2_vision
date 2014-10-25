#include "ARDroneCam.h"
#ifdef ARDRONEFRONTCAM
#include <iostream>


void ARDronecam::start () {
    std::cout << "Cam starting\n";
    cams_are_running=true;

    thread_cam = std::thread(&ARDronecam::captureThread,this);
    //captureThread();

    std::cout << "ARDrone camera link started!\n";
}

void ARDronecam::close () {
    cams_are_running = false;
 //   g_lockWaitForImage.unlock();
	thread_cam.join();
}

void ARDronecam::captureThread() {

    std::cout << "ARDroneCam opening\n";
#ifdef drone
    cv::VideoCapture cam("v4l2src device=/dev/video1 ! videorate ! video/x-raw-yuv,framerate=15/1 ! videoscale ! video/x-raw-yuv, width=544, height=304 ! ffmpegcolorspace ! video/x-raw-rgb ! appsink");
#else
    //cv::VideoCapture cam(0);
    cv::VideoCapture cam("udpsrc uri=udp://0.0.0.0:5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)MP4V-ES, payload=(int)96, clock-base=(uint)2693349430\" ! gstrtpjitterbuffer drop-on-latency=false latency=0 ! rtpmp4vdepay ! ffdec_mpeg4 ! ffmpegcolorspace ! appsink");
#endif

    std::cout << "ARDroneCam opened\n";

    if(!cam.isOpened())
    {
        std::cerr << "ERROR: Could not open ARDrone camera\n";
        return;
    }

    bool res =  cam.read(frame_mat);
    std::cout << "Frame read\n";
    if(!res)
    {
        fprintf( stderr, "!!! cvQueryFrame ARDrone failed!\n" );
        return;
    }

    im_height = frame_mat.rows;
    im_width = frame_mat.cols;
    std::cout << "ARDrone im_width: " << frame_mat.cols << ", im_height: " << frame_mat.rows << std::endl;

    cv::Size size(im_width,im_height);
    outputVideo.open("videoFrontCam.avi",CV_FOURCC('M','P','E','G'),15,size);

    if (!outputVideo.isOpened() )
    {
            std::cout << "!!! Output ARdrone video could not be opened" << std::endl;
            return ;
    }

    int framecounter = 0;
    while (cams_are_running)
    {
        outputVideo.write(frame_mat);
#ifdef HASSCREEN
        cv::imshow("ARDrone", frame_mat);
#endif
        res = cam.read(frame_mat);
        if (!res) {
            cams_are_running = false;
            std::cout << "error ARDrone\n";
            return;
        } else {

            std::cout << "ARDrone im_width: " << frame_mat.cols << ", im_height: " << frame_mat.rows << " #" << framecounter << std::endl;
        }

        framecounter++;

    }
    std::cout << "ARDrone camThread releasing.\n";
    cam.release();
    std::cout << "ARDrone camThread exiting.\n";

}

#endif
