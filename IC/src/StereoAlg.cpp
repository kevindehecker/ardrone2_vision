#include "StereoAlg.h"

#include <stdio.h>
#include <fstream>
#include <iostream>

//opencv
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

bool stereoAlg::init (int im_width,int im_height ) {

#ifdef SGM
    #ifdef DELFLY
    	SGBM = cv::StereoSGBM (5, 32, 5) ; // ,50,500,10,100,10,0,0,false);
		dispScale = 512.0;
	#else
	    SGBM = cv::StereoSGBM (5, 256, 5) ; // ,50,500,10,100,10,0,0,false);
		dispScale = 4096.0;
	#endif
#endif

#ifdef BM
	CvStereoBMState *BMState = cvCreateStereoBMState();
	BMState->preFilterSize 		= 5;
	BMState->preFilterCap 		= 1;
	BMState->SADWindowSize 		= 5;
	BMState->minDisparity 		= 0;
	BMState->numberOfDisparities= 256;
	BMState->textureThreshold 	= 0;
	BMState->uniquenessRatio 	= 0;
	BMState->speckleWindowSize 	= 0;
	BMState->speckleRange		= 0;
#endif

#ifdef LONGSEC
    dispScale = 32;
#endif

    dims[0] = im_width;
    dims[1] = im_height-darksize;
    dims[2] = im_width; // bytes per line = width

#ifdef GEIGER
	initGeigerParam();
    // allocate memory for geiger disparity images


    D1_data = (float*)malloc(im_width*(im_height-darksize)*sizeof(float)/(GeigerSubSampling*GeigerSubSampling));
    D2_data = (float*)malloc(im_width*(im_height-darksize)*sizeof(float)/(GeigerSubSampling*GeigerSubSampling));
    dispScale = 4096.0;
    if ( D1_data == NULL ||  D2_data == NULL) {
    	std::cerr << "Mem peop\n";        
    	return 0;
    }

    param=Elas::parameters(Elas::ROBOTICS);
   	totdisppixels = im_width*(im_height-darksize)/(GeigerSubSampling*GeigerSubSampling);
#endif

    return 1;

}


#ifdef GEIGER
void stereoAlg::initGeigerParam() {
#if defined(DELFLY)
	param.subsampling = false;
	param.disp_min = 1;
	param.disp_max = 64;
    darksize = 0;
#else
	param.subsampling = true;
	param.disp_min = 5;
	param.disp_max = 255;
    darksize = 40; // TODO: find out this value
#endif
	GeigerHeatScaling = 256/param.disp_max;

	if (param.subsampling) {
		GeigerSubSampling = 2;
	}
	else {
		GeigerSubSampling =1;
	}

}
#endif


bool stereoAlg::calcDisparityMap(cv::Mat frameL_mat,cv::Mat frameR_mat) {

#ifdef GEIGER
	Elas elas(param);  //hmm moving this to init gives weird deleted function compile error...
	DisparityMat = cv::Mat::zeros((dims[1])/GeigerSubSampling,dims[0]/GeigerSubSampling, cv::DataType<uint8_t>::type);
	int res = elas.process((uint8_t*)frameL_mat.data,(uint8_t*)frameR_mat.data,D1_data,D2_data,dims);
        // fix map and compute average disparity
	avgDisparity=0;
	if (res)  {

            int okcount = 0; //keeps track how many pixels are OK (not 0 = NaN) in the disparity map
            for (int i =0; i<dims[0]*(dims[1])/(GeigerSubSampling*GeigerSubSampling); i++) {

            	if (D1_data[i] <0) {
            		D1_data[i] = 0;
            	}
            	else {
            		okcount++;
            		avgDisparity+=D1_data[i];
            	}
                //(*Disparity).imageData[i]=D1_data[i];
            	DisparityMat.data[i] = D1_data[i]*GeigerHeatScaling;
            }
            avgDisparity /= okcount;

            res = (10*okcount > totdisppixels); //% of good pixels needed before ignoring the frame
            if (!res) {	std::cout << "Blocked: #" << okcount << "/" << totdisppixels << "\n";   }

        } else { return false;}

#else

#ifdef SGM 	//Semi global matching
    DisparityMat = cv::Mat::zeros((dims[1]),dims[0], cv::DataType<uint16_t>::type);
    SGBM.operator()(frameL_mat,frameR_mat,DisparityMat);
#endif

#ifdef BM 	//Block matching
    IplImage* Disparity_OC=cvCreateImage(cvSize(svcam.im_width,(svcam.im_height-svcam.darksize)),IPL_DEPTH_32F,1);
    cvFindStereoCorrespondenceBM (processed_frameL,	processed_frameR, Disparity_OC, BMState);
    cv::Mat DisparityMat(Disparity_OC);
    res = false; // ignore output, not implemented yet
#endif

#ifdef LONGSEC    
    DisparityMat = cv::Mat::zeros((dims[1]),dims[0], cv::DataType<uint16_t>::type);
    combineImage(frameL_mat,frameR_mat);
    performLongSec(frameC_mat,&DisparityMat );
#endif

    avgDisparity = cv::mean(DisparityMat)(0);
#if defined(HASSCREEN) || defined(VIDEORESULTS)
    double min,max;
    cv::minMaxIdx(DisparityMat, &min, &max);
    std::cout << " min/max: " << min << " / " << max << std::endl;
    DisparityMat.convertTo(DisparityMat,CV_8UC1, 256.0 / dispScale, 0.0); // expand range to 0..255.
#endif
#endif

#if defined(HASSCREEN) || defined(VIDEORESULTS)
    cv::applyColorMap(DisparityMat,DisparityMat,2);
#endif

    return true;

}

void stereoAlg::Destroy(){
#ifdef GEIGER
    free(D1_data);
    free(D2_data);
#endif
}

//combines a sperate left and right image into one combined concenated image
void stereoAlg::combineImage(cv::Mat iml,cv::Mat imr) {

    frameC_mat = cv::Mat(dims[1],dims[0]*2,CV_8UC1);
    cv::Point pl1(0, 0);
    cv::Point pl2(dims[0], dims[1]);
    cv::Mat roil = cv::Mat(frameC_mat, cv::Rect(pl1, pl2));
    iml.copyTo(roil);

    cv::Point pr1(dims[0], 0);
    cv::Point pr2(dims[0]*2, dims[1]);
    cv::Mat roir = cv::Mat(frameC_mat, cv::Rect(pr1, pr2));
    imr.copyTo(roir);

}

void stereoAlg::performLongSec(cv::Mat grayframe,cv::Mat * DisparityMat)
{


    //uint _height = image_height;
    uint _width = dims[0];
    uint _height = dims[1];
    uint disparity_min, disparity_max, disparity_range;
    uint thr1;
    disparity_min=0;
    disparity_range=16; // do not change this without also changing the array bounds below
    disparity_max =  disparity_range - disparity_min-1;	// TODO: check this
    thr1 = 7;

    // define working variables
    int p_left, p_right;
    uint check_left_max,check_right_max;
    uint upd_disps1[512] = {};
    uint seqLength;

    for ( uint l = 0; l < _height; l++ )
    {
        uint check_left[16][512] = {};
        uint check_right[16][512]= {};
        uint check_temp[16] = {};

        for ( uint i = disparity_max; i < _width; i++) // for each pixel of the image line
        {
            p_left = grayframe.at<uint8_t>(l, i);

            for ( uint d = disparity_min; d <=disparity_max; d++) // for disparity range
            {
                p_right = grayframe.at<uint8_t>(l, i+_width - d);

                if ( i == _width-1 || std::abs(p_left - p_right) > thr1 ) // check if pixel cost exceeds error threshold
                {
                    seqLength = check_temp[d];
                    // increment sequence length of all previous pixels in sequence
                    while (check_temp[d] > 0)
                    {
                        check_left[d][i-check_temp[d]] = seqLength;
                        check_right[d][i-d-check_temp[d]] = seqLength;
                        check_temp[d]--;
                    }
                }
                check_temp[d]++;
            }
        }

        uint max_disps_left[512] = {};
        uint max_disps_right[512] = {};
        for (uint i = disparity_max, i2 = 0; i<_width; i++, i2++) // for each pixel of the image line
        {
            check_left_max = check_right_max = 0;
            for ( uint d = disparity_min; d <= disparity_max; d++) // for each pixel of the image line
            {
                // check if last sequence was longer
                if (check_left[d][i] > check_left_max)
                {
                    check_left_max = check_left[d][i];
                    max_disps_left[i] = d;
                }
                if (check_right[d][i2] > check_right_max)
                {
                    check_right_max = check_right[d][i2];
                    max_disps_right[i2] = d;
                }
            }
            // reset udp for next loop
            upd_disps1[i] = disparity_range;
        }

        for (uint i = disparity_max; i<_width-disparity_max; i++) // for each pixel of the image line
        {
            if ( upd_disps1[i+max_disps_right[i]] == disparity_range ) // project the disparity map of the second image using initial disparities on the first image
                upd_disps1[i+max_disps_right[i]] = max_disps_right[i];

            if ( max_disps_left[i] < upd_disps1[i] ) // compare the initial disparity map of the first image to the projection of the second image, choose smalles disparity
                upd_disps1[i] = max_disps_left[i];

            (*DisparityMat).at<uint16_t>(l, i) = upd_disps1[i]*8;
        }
    }

}


