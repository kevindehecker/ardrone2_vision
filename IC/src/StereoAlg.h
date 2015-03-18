
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

/*
 * This class provides access to several stereo algoritmes
 *
 */
class stereoAlg{


private:
    int totdisppixels;
    float dispScale;
    int darksize;
    int32_t dims[3];


    void performLongSec(cv::Mat bw,cv::Mat * DisparityMat);
#ifdef FILESTEREO
	cv::Mat flst[4];
	int count_filestereo;
#endif
#ifdef GEIGER
// //Geiger stereovision class
	Elas::parameters param;
 	//Elas elas; 	
 	int GeigerSubSampling;
 	int GeigerHeatScaling;
	float* D1_data;
	float* D2_data; 	
#endif
void initGeigerParam(void);

#ifdef SGM
    cv::StereoSGBM SGBM;
#endif

public:
	cv::Mat DisparityMat;
	float avgDisparity;
	cv::Mat avgs;
    float stddevDisparity;
    cv::Mat frameC_mat;

	bool init (int im_width,int im_height);
    void combineImage(cv::Mat iml,cv::Mat imr);
    bool calcDisparityMap(cv::Mat frameL,cv::Mat frameR);
	void Destroy();


}; 


#endif //STEREOALG_H
