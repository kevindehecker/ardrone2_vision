
#ifndef FILECAM_H
#define FILECAM_H
#include "defines.h"
#ifdef FILECAM
#include <mutex>
#include <thread>
#include <opencv2/highgui/highgui.hpp>

#include "stopwatch.h"


/*
 * This class will open a stereo video file and stream the data as if it was an Delfly stereo camera
 *
 */
class FileCam{


private:
	int im_width;
	int im_height;
	int scaleFactor;
	std::mutex g_lockWaitForImage1;
    std::mutex g_lockWaitForImage2;
    bool copyNewImage;
    std::thread thread_cam;
    void workerThread(void);    
    void splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR );
    cv::VideoCapture video;
    stopwatch_c stopWatch;

public:

    int nFrames;

    int fastforward;
    int rewind;

    int CurrentFrame;
    bool cams_are_running;
    cv::Mat frameL_mat;
    cv::Mat frameR_mat;

	bool init (void);
	void start (void) ;
	void waitForImage(void);
	void close (void);
	int getImWidth();
	int getImHeight();


};
#endif //FILECAM
#endif //FILECAM_H
