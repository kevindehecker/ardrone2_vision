
#ifndef DELFLY_H
#define DELFLY_H

#include <mutex>
#include <thread>
#include <opencv2/contrib/contrib.hpp>
#include "defines.h"



#define HEADERCOLOR  0
#define HEADERSTEREO 1
#define HEADERDISPARITY 2

#ifndef DELFLY_COLORMODE
#define HEADERBYTE HEADERSTEREO
#else
#define HEADERBYTE HEADERCOLOR
#endif

/*
 * This class will interface with a Delfly stereocam over USB
 *
 */
class DelFly{


private:
    //calibration matrices
    CvMat *_mx1;
    CvMat *_my1;
    CvMat *_mx2;
    CvMat *_my2;
    cv::Mat frameL_2ndbuf;
    cv::Mat frameR_2ndbuf;

	std::mutex g_lockWaitForImage;
	bool copyNewImage;
	std::thread thread_cam;
	void workerThread(void);

public:
	int im_width;
	int im_height;
	int darksize;
	int extracaliboffset;

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
    int getImHeight();
    int getImWidth();


};

#endif //DELFLY_H
