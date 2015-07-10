#include "stereoalg.h"

#include <stdio.h>
#include <fstream>
#include <iostream>

//opencv
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

bool stereoAlg::init (int im_width,int im_height) {

	DisparityMat = cv::Mat::zeros(im_height,im_width, CV_8UC3);

#ifdef FILESTEREO
	count_filestereo=0;
	//create filename
	std::string filename = "../images/export.txt";

	//get length of the file
	std::ifstream sfile;
	sfile.open(filename);
	std::ifstream f(filename);
	std::string line;	
	int flength;
	for (flength = 0; std::getline(sfile, line); ++flength)
		;
	sfile.close();


	if (flength==0) {return 0;}
	//read the file into flst
	flst = cv::Mat(flength ,1,CV_32S);
	sfile.open(filename);
	for (int i = SKIPFRAMES_START; i<flength; ++i) {

		std::getline(sfile, line);
		auto start = 0U;
		auto end = line.find(";" );
		//std::cout << line.substr(start, end - start) << std::endl;
		int d;
		d = std::stoi(line, NULL,10);
		//std::cout << d << std::endl;
		if (i-SKIPFRAMES_START >= 0)
		flst.at<int>(i-SKIPFRAMES_START)  =d;
	}
	sfile.close();
	//std::cout << flst << std::endl;

#endif

#ifdef SGM
    #ifdef DELFLY
        SGBM = cv::StereoSGBM (5, 32, 5 ,50,500,10,100,10,0,0,true);
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
    dispScale = 256;
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

void stereoAlg::initGeigerParam() {
	#ifdef GEIGER
#if defined(DELFLY)
	param.subsampling = false;
    param.disp_min = 5;
	param.disp_max = 256;
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
#endif
}


bool stereoAlg::calcDisparityMap(cv::Mat frameL_mat,cv::Mat frameR_mat) {
#ifdef FILESTEREO		
	avgDisparity = flst.at<int>(count_filestereo);

#if defined(HASSCREEN) || defined(VIDEORESULTS)
	std::stringstream s;
	s <<  "../images/disp" << count_filestereo+1 << ".png";	
	DisparityMat = cv::imread(s.str());
#endif

	count_filestereo++;
	if (avgDisparity<5) // minimum disparity
		return false;
	else
		return true;
#endif
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

			int stereoOK = (10*okcount > totdisppixels); //% of good pixels needed before ignoring the frame
			if (!stereoOK) {	std::cout << "Blocked: #" << okcount << "/" << totdisppixels << "\n";
				return false;
			}

            avgDisparity /= okcount;
            avgDisparity *=5; // heuristic scaling for better visualisation and smoother thresh config

			avgDisparity = (int) avgDisparity;

			//std::cout << "#" << count_filestereo << ", GT: " << avgDisparity << std::endl;


		} else {return false;}

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
    performSparseMatching(frameC_mat,&DisparityMat );

    //create histogram of the disparities
    cv::Mat hist = cv::Mat::zeros(15,1,CV_16UC1);
    for (int i=0; i<DisparityMat.cols;i++ ) {
        for (int j=0; j<DisparityMat.rows;j++ ) {

            int tmp = DisparityMat.at<uint16_t>(j, i);
            tmp = tmp>>4;
            hist.at<uint16_t>(tmp)++;
        }
    }
    // avgDisparity = hist.at<uint16_t>(14)*8 + hist.at<uint16_t>(13)*4 + hist.at<uint16_t>(12)*2 + hist.at<uint16_t>(11)*2+ hist.at<uint16_t>(10)*1+ hist.at<uint16_t>(9)*1;
    // stddevDisparity=0;
    int sum = 0;
    avgDisparity=0;
    for (int i=14; i>0;i-- ) {
        sum +=  hist.at<uint16_t>(i);
        if (sum>75) {
            avgDisparity = i;
            break;
        }
    }



//    avgDisparity=0;
//    int okcount = 0; //keeps track how many pixels are OK (not 0 = NaN) in the disparity map
//    for (int i=0; i<DisparityMat.cols;i++ ) {
//        for (int j=0; j<DisparityMat.rows;j++ ) {
//            int tmp = DisparityMat.at<uint16_t>(j, i);
//            if (tmp >0) {
//                okcount++;
//                avgDisparity+=tmp;
//            }
//        }
//    }

//    avgDisparity /= okcount;
//    avgDisparity *=5; // heuristic scaling for better visualisation and smoother thresh config
//    avgDisparity = (int) avgDisparity;

#else
    //avgDisparity = cv::mean(DisparityMat)(0);

    cv::Scalar mean,stddev;
    cv::meanStdDev(DisparityMat,mean,stddev,cv::Mat());

    stddevDisparity = stddev(0);
    avgDisparity = mean(0);
    stddevDisparity = (stddevDisparity/avgDisparity ) * 100;
#endif



#ifdef SGM
    if (stddevDisparity  > 55) {
        avgDisparity =0;
    }
#endif


	//calc depth in mm from disparity
	// z = (b*F) / (d*s)
	// z = depth in mm
	// b = baseline in mm
	// F = focal length in mm
	// d = depth in pixel
	// s = sensor size in mm/pixel.
	//or
	// z = b*F / d
	// mm = mm * pixel / pixel


	// DisparityMat = 500000  / DisparityMat;



#if defined(HASSCREEN) || defined(VIDEORESULTS)
    double min,max;
    cv::minMaxIdx(DisparityMat, &min, &max);

    std::cout << " mean/std: " << avgDisparity << " / " << stddevDisparity << std::endl;
    //std::cout << " min/max: " << min << " / " << max << std::endl;
    DisparityMat.convertTo(DisparityMat,CV_8UC1, 256.0 / dispScale, 0.0); // expand range to 0..255.
#endif
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

void stereoAlg::performSparseMatching(cv::Mat grayframe,cv::Mat * DisparityMat) {
    //uint32_t image_width_bytes = image_width * 2; 					// number of bytes of 2 interlaced image lines
    // TODO check if disparity_min is still required

    uint _width = dims[0];
    uint _height = dims[1];

    int vertical_block_size = 5; // vertical size of SAD-window
    int horizontal_block_size = 5; // horizontal size of SAD-window
    int GRADIENT_THRESHOLD = 3; // defines if image gradient indicates sufficient texture
    int PKRN_THRESHOLD = 130; // defines if best match is significantly better than second best match [in % to deal with fixed point (120 means a difference of 20%)]

    int half_vertical_block_size = (vertical_block_size - 1)/2;     // = 2
    int half_horizontal_block_size = (horizontal_block_size - 1)/2; // = 2

    // Stereo parameters:
    uint32_t disparity_range = 16; // at a distance of 1m, disparity is 7-8
    uint32_t disparity_min = 0;
    uint32_t disparity_max =  disparity_range - disparity_min - 1;	// set disparity_range = 15;
    uint32_t disparity_scale = 255/disparity_range;

    uint sum_abs_diff;
    uint min1;
    uint min1_d;
    uint min2;

    // loop over all image lines (excluding borders)
    for ( uint l = half_vertical_block_size; l < _height - half_vertical_block_size; l++ ) {
        uint8_t p1,p2,p3;
        uint diff_left,diff_mid,diff_right;

        // start by computing first two gradient terms for this image line (taking into account borders)
        p1 = grayframe.at<uint8_t>(l, half_horizontal_block_size+disparity_max-1);
        p2 = grayframe.at<uint8_t>(l, half_horizontal_block_size+disparity_max);
        p3 = grayframe.at<uint8_t>(l, half_horizontal_block_size+disparity_max+1);

        diff_left = std::abs( p1-p2 );
        diff_mid =  std::abs( p2-p3 );

        // loop over image line (excluding borders)
        for (uint p = half_horizontal_block_size+disparity_max; p < _width-half_horizontal_block_size ; p++)  {
            // compute the third image gradient
            diff_right = abs( grayframe.at<uint8_t>(l,p+1) - grayframe.at<uint8_t>(l,p+2));

            // now check if the gradient for this pixel p (diff_mid) is a local peak (higher than diff_left and diff_right) and exceeds minimum-threshold
            if ( diff_mid > diff_left     &&     diff_mid > diff_right     &&     diff_mid > GRADIENT_THRESHOLD ) {
                min1 = 10000;
                min2 = 10000;

                // perform SAD calculation for pixel p
                for ( uint d = 0; d < disparity_range; d++ ) {
                    sum_abs_diff = 0;
                    for (uint x = p - half_horizontal_block_size; x < p + half_horizontal_block_size; x++)  {
                        for (uint y = l - half_vertical_block_size; y < l + half_vertical_block_size; y++) {
                            sum_abs_diff += abs( grayframe.at<uint8_t>(y,  x + _width) - grayframe.at<uint8_t>(y,  x + d) );
                        }
                    }

                    // keep track of minimum cost (+ corresponding index) and also of second best minimum cost
                    if ( sum_abs_diff < min1 ) {
                        min2  = min1;
                        min1_d = d;
                        min1 = sum_abs_diff;
                    } else if ( sum_abs_diff < min2 ) {
                        min2 = sum_abs_diff;
                    }
                }

                // check if the ratio between first and second minimum cost exceeds minimum ratio
                if ( min2*100/(min1+1) > PKRN_THRESHOLD ) {
                    (*DisparityMat).at<uint16_t>(l, p) =  min1_d*disparity_scale; // scaling is purely for visualization
                }
            }

            diff_left = diff_mid;
            diff_mid = diff_right;
        }
    }
}
