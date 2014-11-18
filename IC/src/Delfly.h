
#ifndef DELFLY_H
#define DELFLY_H

#include <mutex>
#include <thread>
#include <opencv2/contrib/contrib.hpp>
#include "defines.h"

class DelFly{


private:
    //calibration matrices
    CvMat *_mx1;
    CvMat *_my1;
    CvMat *_mx2;
    CvMat *_my2;

	std::mutex g_lockWaitForImage;
	bool copyNewImage;
	std::thread thread_cam;
	void workerThread(void);

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

#endif //DELFLY_H
