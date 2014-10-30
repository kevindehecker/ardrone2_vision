
#ifndef ARDRONECAM_H
#define ARDRONECAM_H

#include <thread>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "defines.h"

#ifdef ARDRONEFRONTCAM
class ARDronecam{


private:		
	std::thread thread_cam;
    cv::VideoWriter outputVideo;
    void captureThread(void);
    cv::Mat frame_mat;

public:
	int im_width;
	int im_height;
	bool cams_are_running;

	void start (void) ;
	void close (void);



};

#endif //ARDRONEFRONTCAM
#endif //ARDRONECAM_H
