
#ifndef TEXTONS_H
#define TEXTONS_H

#include <ml.h>

//own stuff:
#include "defines.h"
#include "smoother.h"




class Textons{


private:
	//visual words parameters
    int n_samples = 50;
    int filterwidth = 10; // moving average filter
    int k = 5;

    // to be loaded from textons*.dat dictionary files:
    unsigned char n_textons;
    unsigned char n_textons_gradient;
    unsigned char n_textons_intensity;
	unsigned char patch_size;
	unsigned char patch_square_size;
    std::vector<std::vector<int16_t> > textons;


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

    double getEuclDistance(int16_t sample[], int texton_id);
	int initTextons();
    void drawTextonColoredImage(cv::Mat grayframe);
    cv::Scalar getColor(int id);

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
