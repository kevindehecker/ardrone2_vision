
#ifndef TEXTONS_H
#define TEXTONS_H

#include <ml.h>

//own stuff:
#include "defines.h"
#include "smoother.h"

class Textons{


private:
	//visual words parameters
	int n_samples;
	float hist_step; // 1/n_samples for histogram sum ==1
	int filterwidth; // moving average filter
	int k;

    // to be loaded from textons*.dat dictionary files:
    unsigned char n_textons;
    unsigned char n_textons_gradient;
    unsigned char n_textons_intensity;
    unsigned char patch_size;
    unsigned char patch_square_size;

	int countsincelearn;

#define CUMULATIVE  0
#define MINIMUM_DISTANCE 1
	int method;
    std::vector<std::vector<int16_t> > textons;


	//regression learning parameters
    cv::Mat distribution_buffer;
    cv::Mat groundtruth_buffer;
    cv::Mat graph_buffer;
    int lastLearnedPosition;
	int distribution_buf_size;
	int distribution_buf_pointer;
    CvKNearest knn;

	//moving average filters:
    Smoother knn_smoothed;
    Smoother gt_smoothed;

    double getEuclDistance(int16_t sample[], int texton_id);
    int initTextons();
    void drawTextonAnotatedImage(cv::Mat grayframe);	
    cv::Scalar getColor(int id);

public:
	int threshold_nn;
	int threshold_gt;
	int *result_input2Mode;

	float tpr_threshold;
	float fpr_best;
    float avgdisp_smoothed;


    cv::Mat frame_Itextoncolor;
    cv::Mat frame_Itextontexton;
    cv::Mat frame_Gtextoncolor;
    cv::Mat frame_Gtextontexton;
    cv::Mat frame_currentHist;
	cv::Mat frame_ROC;


	Textons() {

		n_samples = 1000;
		hist_step = 1/(float)n_samples;
		filterwidth = 5;
		k = 5;
		countsincelearn =0;
		method = MINIMUM_DISTANCE;
		distribution_buf_size = 2000;
		distribution_buf_pointer =0;
		threshold_nn = 150;
		threshold_gt = 200;

		tpr_threshold = 0.98f;
		fpr_best = 0.7f;


	}

	bool init (int * result_input2Mode);
    bool close(void);
    cv::Mat drawHistogram(cv::Mat hist,int bins, int maxY);
    void drawGraph(std::string msg);
    void getTextonDistributionFromImage(cv::Mat grayframe, float avgdisp, bool activeLearning);
    void saveRegression();
    void retrainAll();

    bool initLearner(bool nulltrain);
    int loadPreviousRegression();
    void reload();
    cv::Mat graphFrame;
    int getLast_nn();
    int getLast_gt();
    void drawMeanHists(cv::Mat histimage);
	void setAutoThreshold();
	void updateMSE();


}; 
#endif // TEXTONS_H
