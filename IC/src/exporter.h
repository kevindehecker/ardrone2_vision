#ifndef Exporter_H
#define Exporter_H
#include "defines.h"
#ifdef EXPORT

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

/*
 * This class exports data to csv txt file, which may be imported in Matlab etc
 *
 */
class Exporter{
private:
    /*  Global variables  */
    std::ofstream myfile;
    std::ofstream caffefile;
    int saveid=1;

    /*  Function declarations  */


public:
    /*  Global variables  */


    /*  Function declarations  */
    void close();
    void init();
	void write(int avgdisp_gt,int avgdisp_gt_stdev, int avgdisp_nn,cv::Mat quadavgdisp_gt);
    void saveStereoPair(cv::Mat frameL_mat,cv::Mat frameR_mat,cv::Mat DisparityMat);
};
#endif // EXPORT
#endif  /*  Exporter_H  */

