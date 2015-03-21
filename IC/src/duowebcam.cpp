
#include "DuoWebcam.h"

#include <stdio.h>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>




bool DuoWebCam::init (int cam_left_id,  int cam_right_id) {
	darksize = 80;
	extracaliboffset = 4;
	im_width = 640;
	im_height = 480;
	im_fps =VIDEOFPS;
	camLeft_id = cam_left_id;
    camRight_id = cam_right_id;
    const char* calibDir = "../duocamcalib/";

    char mx1fn[256];
    strcpy(mx1fn,calibDir);
    strcat(mx1fn,"mx1.xml");
    char mx2fn[256];
    strcpy(mx2fn,calibDir);
    strcat(mx2fn,"mx2.xml");
    char my1fn[256];
    strcpy(my1fn,calibDir);
    strcat(my1fn,"my1.xml");
    char my2fn[256];
    strcpy(my2fn,calibDir);
    strcat(my2fn,"my2.xml");

    _mx1 =	(CvMat *)cvLoad(mx1fn,NULL,NULL,NULL);
    _mx2 =	(CvMat *)cvLoad(mx2fn,NULL,NULL,NULL);
    _my1 =	(CvMat *)cvLoad(my1fn,NULL,NULL,NULL);
    _my2 =	(CvMat *)cvLoad(my2fn,NULL,NULL,NULL);

    if (_mx1 != 0 && _mx2 != 0 && _my1 != 0 && _my2 != 0) {
        std::cout << "finished loading\n";
        return true;
    }
    else {
        fprintf(stderr,"\nError loading calib matrices!");
        return false;
    }
}

void DuoWebCam::start () {
    cams_are_running=true;
    tL =std::thread(&DuoWebCam::camThreadL,this);
    tR =std::thread(&DuoWebCam::camThreadR,this);
    waitForImage();
    waitForImage();
    std::cout << "Webcam link started!";
}

void DuoWebCam::waitForImage() {
    g_lockR.unlock();
    g_lockL.unlock();
    copyNewImage = true;
    g_lockWaitForImageL.lock();
    g_lockWaitForImageR.lock();
	
}

void DuoWebCam::close () {
    cams_are_running = false;
    g_lockR.unlock();
    g_lockL.unlock();
    tL.join();
    tR.join();
    g_lockWaitForImageL.unlock();
    g_lockWaitForImageR.unlock();
}

/* 
 * make_it_gray: custom callback to convert a colored frame to its grayscale version.
 * Remember that you must deallocate the returned IplImage* yourself after calling this function.
*/
IplImage* DuoWebCam::make_it_gray(IplImage* frame) {
    // Allocate space for a new image
 	IplImage* gray_frame = 0;
 	gray_frame = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 1);
 	if (!gray_frame)
 	{
 		fprintf(stderr, "!!! cvCreateImage failed!\n" );
 		return NULL;
 	}

 	cvCvtColor(frame, gray_frame, CV_RGB2GRAY);
 	return gray_frame; 
 }

void DuoWebCam::camThreadL() {

    cv::VideoCapture capLeft(camLeft_id);
	capLeft.set(CV_CAP_PROP_FRAME_WIDTH,im_width);
	capLeft.set(CV_CAP_PROP_FRAME_HEIGHT,im_height);
	capLeft.set(CV_CAP_PROP_FPS,im_fps);

	if(!capLeft.isOpened())
	{
		std::cerr << "ERROR: Could not open left camera\n";		
		return;
	}

	bool res =  capLeft.read(frameL_mat);	
	if(!res) 
	{
		cams_are_running = false;
		fprintf( stderr, "!!! cvQueryFrame Left failed!\n" );
		return;
	}  

	im_height = frameL_mat.rows;
	im_width = frameL_mat.cols;

	std::cout << "Left im_width: " << frameL_mat.cols << ", im_height: " << frameL_mat.rows << std::endl;


	while (cams_are_running)
	{
		g_lockL.lock();
        res = capLeft.read(frameL_mat);
		if (!res) {
			cams_are_running = false;
			std::cout << "error Left\n";
			return;
		} else {
			// std::cout << "Left\n";
		}

		IplImage frameL = frameL_mat; // TODO: convert make it gray to use c++ mat 
    	IplImage* processed_frameL = make_it_gray(&frameL);

    	//rectify and calibrate the stereo image
    	cvRemap( processed_frameL, processed_frameL, _mx1, _my1 );

		//cut off corners that are dark due to rectification
		char* tmpL = (*processed_frameL).imageData;
		processed_frameL=cvCreateImage(cvSize(im_width,im_height-darksize),IPL_DEPTH_8U,1);
    	(*processed_frameL).imageData =  tmpL + im_width*(extracaliboffset+darksize/2);

		frameL_mat = cv::Mat(processed_frameL);

		g_lockWaitForImageL.unlock();
	}
	std::cout << "Left camThread releasing.\n";
	capLeft.release();
	std::cout << "Left camThread exiting.\n";
}

void DuoWebCam::camThreadR() {

    cv::VideoCapture capRight(camRight_id);
	capRight.set(CV_CAP_PROP_FRAME_WIDTH,im_width);
	capRight.set(CV_CAP_PROP_FRAME_HEIGHT,im_height);
	capRight.set(CV_CAP_PROP_FPS,im_fps);

	if(!capRight.isOpened())
	{
		std::cerr << "ERROR: Could not open right camera\n";		
		return;
	}

	bool res = capRight.read(frameR_mat);
	if(!res) 
	{				
		cams_are_running = false;
		fprintf( stderr, "!!! cvQueryFrame right failed!\n" );
		return;
	} 
	im_height = frameR_mat.rows;
	im_width = frameR_mat.cols;

	std::cout << "Right im_width: " << frameR_mat.cols << ", im_height: " << frameR_mat.rows << std::endl;


	while (cams_are_running)
	{
		g_lockR.lock();
		res = capRight.read(frameR_mat);	
		if (!res) {
			cams_are_running = false;
			std::cout << "error Right\n";
			return;
		} 


		IplImage frameR = frameR_mat; // TODO: convert make to use of c++ mat 
    	IplImage* processed_frameR = make_it_gray(&frameR);

    	//rectify and calibrate the stereo image
    	cvRemap( processed_frameR, processed_frameR, _mx2, _my2 );

		//cut off corners that are dark due to rectification
		char* tmpR = (*processed_frameR).imageData;
     	processed_frameR=cvCreateImage(cvSize(im_width,im_height-darksize),IPL_DEPTH_8U,1);
     	(*processed_frameR).imageData =  tmpR + im_width*(darksize/2);
  
		frameR_mat = cv::Mat(processed_frameR);

		g_lockWaitForImageR.unlock();
	}

	std::cout << "Right camThread releasing.\n";
	capRight.release();
	std::cout << "Right camThread exiting.\n";

}
