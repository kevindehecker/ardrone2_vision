
#ifndef TEXTONS_H
#define TEXTONS_H

#include <ml.h>

//own stuff:
#include "defines.h"
#include "smoother.h"




class Textons{


private:
	//visual words parameters
    #define n_samples 50
	unsigned char n_textons ;
	unsigned char patch_size;
	unsigned char patch_square_size;
	std::vector<std::vector<unsigned char> > textons;
    int filterwidth = 10; // moving average filter

	//regression learning parameters
	cv::Mat distribtuion_buffer;
	cv::Mat groundtruth_buffer;
	cv::Mat graph_buffer;
	int lastLearnedPosition;
    int distribution_buf_size = 2000;
    int distribtuion_buf_pointer =0;
	CvKNearest knn;

	//moving average filters:
	Smoother knn_smoothed;
	Smoother gt_smoothed;

	double getEuclDistance(unsigned char sample[], int texton_id);
	int initTextons();

	

public:
    int threshold_nn = 60;
	bool init (void);
	bool close(void);
	void drawHistogram(cv::Mat hist,int bins);
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

}; 
#endif // TEXTONS_H
