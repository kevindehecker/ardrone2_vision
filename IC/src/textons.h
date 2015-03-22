
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
	int n_samples_sqrt;
	int filterwidth; // moving average filter
	int k;

    // to be loaded from textons*.dat dictionary files:
    unsigned char n_textons;
    unsigned char n_textons_gradient;
    unsigned char n_textons_intensity;
    unsigned char patch_size;
    unsigned char patch_square_size;

	int countsincelearn;

	float _mse_tst;
	float _mse_trn;
	int _mse_tst_cnt;
	int _mse_trn_cnt;
	float _tpr_trn;
	float _fpr_trn;
	float _tpr_tst;
	float _fpr_tst;

    std::vector<std::vector<int16_t> > textons;
#define TEXTON_CUMULATIVE_DISTANCE  0
#define TEXTON_MINIMUM_DISTANCE 1
	int method;

	//regression learning parameters
    cv::Mat distribution_buffer;
    cv::Mat groundtruth_buffer;
    cv::Mat graph_buffer;
    int lastLearnedPosition;
	int distribution_buf_size;
	int distribution_buf_pointer;
	int graph_buf_size;
    CvKNearest knn;

	//moving average filters:
	//Smoother knn_smoothed;
	Smoother est_smoothers[4];
	//Smoother gt_smoothed;

    double getEuclDistance(int16_t sample[], int texton_id);
    int initTextons();
    void drawTextonAnotatedImage(cv::Mat grayframe);	
    cv::Scalar getColor(int id);

public:
	int threshold_est;
	int threshold_gt;
	int *result_input2Mode;
	int quad_VizChannel;

	float tpr_threshold;
	//float avgdisp_smoothed;

    cv::Mat frame_Itextoncolor;
    cv::Mat frame_Itextontexton;
    cv::Mat frame_Gtextoncolor;
    cv::Mat frame_Gtextontexton;
    cv::Mat frame_currentHist;
	cv::Mat frame_ROC;
	cv::Mat frame_regressGraph;

	Textons() {

		n_samples = 20;
		n_samples_sqrt = round(sqrt(n_samples ));
		n_samples = n_samples_sqrt *n_samples_sqrt;		
		filterwidth = 5;
		k = 5;
		countsincelearn =0;
		method = TEXTON_MINIMUM_DISTANCE;
		distribution_buf_size = 2435*4;
		graph_buf_size = distribution_buf_size/4;
		distribution_buf_pointer =0;
		threshold_est = 150;
		threshold_gt = 200;

		tpr_threshold = 0.98f;
		quad_VizChannel=0;

	}

	int init (int * result_input2Mode);
    bool close(void);
    cv::Mat drawHistogram(cv::Mat hist,int bins, int maxY);
    void drawGraph(std::string msg);
	void getTextonDistributionFromImage(cv::Mat grayframe, cv::Mat avgdisps, bool activeLearning, int pauseVideo);
	void getTextonDistributionFromImage(cv::Mat grayframe, float avgdisp, bool activeLearning, int pauseVideo, bool stereoOK,int q);
    void saveRegression();
    void retrainAll();
	void printReport(float fps);

	int initLearner(bool nulltrain);
    int loadPreviousRegression();
    void reload();    
	int getLast_est();
    int getLast_gt();
    void drawMeanHists(cv::Mat histimage);
	void setAutoThreshold();


}; 
#endif // TEXTONS_H
