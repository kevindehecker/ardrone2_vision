
#ifndef TEXTONS_H
#define TEXTONS_H

#include <ml.h>

//own stuff:
#include "defines.h"
#include "smoother.h"




class Textons{


private:
	//visual words parameters
    #define n_samples 100
	unsigned char n_textons ;
	unsigned char patch_size;
	unsigned char patch_square_size;
	std::vector<std::vector<unsigned char> > textons;
    int filterwidth = 5; // moving average filter
    int k = 5;
//    std::vector<cv::Scalar> colors2;
//    cv::Scalar colors[30];

//    uint8_t colorsr[30];
//    uint8_t colorsg[30];
//    uint8_t colorsb[30];



	//regression learning parameters
    cv::Mat distribution_buffer;
	cv::Mat groundtruth_buffer;
	cv::Mat graph_buffer;
	int lastLearnedPosition;
    int distribution_buf_size = 2000;
    int distribution_buf_pointer =0;
	CvKNearest knn;

	//moving average filters:
	Smoother knn_smoothed;
	Smoother gt_smoothed;

	double getEuclDistance(unsigned char sample[], int texton_id);
	int initTextons();
    void drawTextonColoredImage(cv::Mat grayframe);
	

public:
    int threshold_nn = 29;
    int threshold_gt = 90;
    float avgdisp_smoothed;


	bool init (void);
	bool close(void);	
    cv::Mat drawHistogram(cv::Mat hist,int bins);
    void drawGraph(std::string msg);
    void getTextonDistributionFromImage(cv::Mat grayframeL, float avgdisp);
	void saveRegression();
	void retrainAll();
	void getCommData(float* s);
    bool initLearner(bool nulltrain);
	int loadPreviousRegression();
    void reload();
	cv::Mat graphFrame;
    int getLast_nn();
    int getLast_gt();
    void drawMeanHists(cv::Mat histimage);



}; 
#endif // TEXTONS_H
