
#ifndef STEREOALG_H
#define STEREOALG_H

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

#include "defines.h"

#ifdef GEIGER
//lib elas, Andreas Geiger:
#include "elas.h"
#endif

class stereoAlg{


private:
    int totdisppixels;
    float dispScale;
    int darksize;
    int32_t dims[3];

    void performLongSec(cv::Mat bw,cv::Mat * DisparityMat);

#ifdef GEIGER
// //Geiger stereovision class
	Elas::parameters param;
 	//Elas elas; 	
 	int GeigerSubSampling;
 	int GeigerHeatScaling;
	float* D1_data;
	float* D2_data;
 	void initGeigerParam(void);
#endif

#ifdef SGM
    cv::StereoSGBM SGBM;
#endif

public:
	cv::Mat DisparityMat;
	float avgDisparity;
    float stddevDisparity;
    cv::Mat frameC_mat;

	bool init (int im_width,int im_height);
    void combineImage(cv::Mat iml,cv::Mat imr);
    bool calcDisparityMap(cv::Mat frameL,cv::Mat frameR);
	void Destroy();


}; 


#endif //STEREOALG_H
