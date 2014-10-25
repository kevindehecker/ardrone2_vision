
// #ifndef DUOCAM_H
// #define DUOCAM_H

// #include <stdio.h>
// #include <fstream>
// #include <string>
// #include <iostream>
// #include <vector>

// class DuoWebcam{


// private:
// //Stereo vision camera handling stuff:
// std::mutex g_lockL;
// std::mutex g_lockWaitForImageL;
// std::mutex g_lockR;
// std::mutex g_lockWaitForImageR;
// bool copyNewImage;

// //calibration matrices
// CvMat *_mx1;
// CvMat *_my1;
// CvMat *_mx2;
// CvMat *_my2; 
// int darksize = 80;
// int extracaliboffset = 4;


// IplImage* make_it_gray(IplImage* frame);
// void camThreadL(int id) ;
// void camThreadR(int id);
// bool initStereoCalibration();

// public:
// int im_width = 640;
// int im_height = 480;
// int im_fps =10;
// bool cams_are_running;
// cv::Mat frameL_mat;
// cv::Mat frameR_mat;

// };

// #endif //DUOCAM_H