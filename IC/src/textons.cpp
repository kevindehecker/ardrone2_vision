#include "textons.h"
#include <string>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#ifdef _PC
#include <boost/math/special_functions/round.hpp>
#endif

//from HSV colorspace:
float r[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526, 0.8571428571428568, 0.7619047619047614, 0.6666666666666665, 0.5714285714285716, 0.4761904761904763, 0.3809523809523805, 0.2857142857142856, 0.1904761904761907, 0.0952380952380949, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09523809523809557, 0.1904761904761905, 0.2857142857142854, 0.3809523809523809, 0.4761904761904765, 0.5714285714285714, 0.6666666666666663, 0.7619047619047619, 0.8571428571428574, 0.9523809523809523, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
float g[] = { 0, 0.09523809523809523, 0.1904761904761905, 0.2857142857142857, 0.3809523809523809, 0.4761904761904762, 0.5714285714285714, 0.6666666666666666, 0.7619047619047619, 0.8571428571428571, 0.9523809523809523, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526, 0.8571428571428577, 0.7619047619047619, 0.6666666666666665, 0.5714285714285716, 0.4761904761904767, 0.3809523809523814, 0.2857142857142856, 0.1904761904761907, 0.09523809523809579, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float b[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09523809523809523, 0.1904761904761905, 0.2857142857142857, 0.3809523809523809, 0.4761904761904762, 0.5714285714285714, 0.6666666666666666, 0.7619047619047619, 0.8571428571428571, 0.9523809523809523, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9523809523809526, 0.8571428571428577, 0.7619047619047614, 0.6666666666666665, 0.5714285714285716, 0.4761904761904767, 0.3809523809523805, 0.2857142857142856, 0.1904761904761907, 0.09523809523809579, 0};

/*
 *Loads texton dictionary from  file, loads previously results from file, and learns them
 */
bool Textons::init () {

    if (!initTextons()) {return false;}
    if (!initLearner(true)) {return false;}
    loadPreviousRegression();
    return true;
}

/*
*Calculates the Eucladian distance between a patch and a texton
*/
double Textons::getEuclDistance(int16_t sample[], int texton_id) {
    int sum =0;

    std::vector<int16_t> v =textons[texton_id];

    std::vector<int16_t> sample_;
    sample_.assign(sample, sample + patch_square_size);

    for(int i = 0; i < patch_square_size; i++)
    {
        sum += pow(sample_[i] - v[i],2);

    }
    float distance = sqrt((float)sum);

    return distance;
}

/*
 * Creates an histogram image
 */
cv::Mat Textons::drawHistogram(cv::Mat hist,int bins, int maxY) {

    cv::Mat canvas;
    int line_width = 10;


    // Display each histogram in a canvas
	canvas = cv::Mat::zeros(maxY, bins*line_width, CV_8UC3);

    int rows = canvas.rows;
	for (int j = 0; j < bins; j++)
    {
        cv::Scalar c = getColor(j);
		cv::line(canvas,cv::Point(j*line_width, rows),cv::Point(j*line_width, rows - (150*hist.at<float>(j))),c, line_width, 8, 0);
	}


	//copy the normal histogram to the big image
	std::stringstream s;
	s << "Entropy: " << hist.at<float>(n_textons) ;
	putText(canvas,s.str(),cv::Point(0, 40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));

    return canvas;
}

/*
 * Retrieves the bin color for id
 */
cv::Scalar Textons::getColor(int id) {
    if (id<n_textons_intensity) {

    } else {
        id -= n_textons_intensity;
    }
	if (id==4) { // fix double green...
		return 	cv::Scalar(255,255,255);
	}

    return cv::Scalar(b[id*6]*255.0,g[id*6]*255.0,r[id*6]*255.0);
}

/*
 * Draws a range of histograms, averaged over several disparity ranges. Except for the first histogram which is for the current frame.
 */
void Textons::drawMeanHists(cv::Mat histimage) {

    int nHists= 5;
    int hist_width = n_textons*10; // also in drawHistogram -> TODO: change
    int hist_height= 200; // idem

    cv::Mat canvas_allHists = cv::Mat::zeros(hist_height*2,hist_width*3,CV_8UC3);
	cv::Mat meanhists = cv::Mat::zeros(nHists, 20, cv::DataType<float>::type);
    cv::Mat amounts = cv::Mat::zeros(nHists, 1, cv::DataType<int>::type);

    //get the maximum disparity (range), in order to determine the histogram borders (of the 5 histograms)
    double maxgt;
    cv::minMaxIdx(groundtruth_buffer,NULL,&maxgt);
    float f  = maxgt/(nHists); //hist disparity size range

    for (int j=0;j<distribution_buf_size;j++) {
        cv::Mat hist;
        distribution_buffer.row(j).copyTo(hist); // distribution_buffer is in float, so copy like this needed -> not any more TODO: fix
        int id = ceil((groundtruth_buffer.at<float>(j) / f))-1; // calc which hist this frame belongs to
		//std::cout << groundtruth_buffer.at<float>(j) << "\n";
        if (id<0){id=0;} // catch gt = 0;
		else if (id>nHists-1){id=nHists-1;} // catch gt = maxgt, can happen due to float/double difference, with some compilers at least... weird;

        if (!(groundtruth_buffer.at<float>(j) > 6.199 && groundtruth_buffer.at<float>(j) < 6.2001)) { // exclude unused buffer
            amounts.at<int>(id)++; // keep track of amount of frames in a each hist range

            //add hist of current frame to the cumulator of the specific range
			cv::Mat tmp = meanhists.row(id);
            for (int k = 0; k<n_textons; k++) {
                tmp.at<float>(k) = tmp.at<float>(k) + hist.at<float>(k);
            }
        }

    }

    for (int i=0; i<nHists;i++) {

        // select a hist:
        cv::Mat tmp = meanhists.row(i);

        //calc average for each hist
        for (int j = 0; j<n_textons; j++) {
            if (amounts.at<int>(i) > 0 ) {
				tmp.at<float>(j) = (tmp.at<float>(j) / (float)amounts.at<int>(i));
            } else {
				tmp.at<float>(j) = 0;
            }

        }



        //create an image of it
		cv::Mat canvas =  drawHistogram(tmp,n_textons,200);
		std::stringstream s;
		s << "Disps: " << (int)(f*i) << " - " << (int)(f*(i+1)) << ", #" << amounts.at<int>(i);
		putText(canvas,s.str(),cv::Point(0, 20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));

		//and copy it to the big image
		int x,y;
		if (i <2) {
			x = (i+1)*hist_width;
			y=0;
		} else {
			x = (i-2)*hist_width;
			y=hist_height;
		}
		cv::Point p1(x, y);
		cv::Point p2(x+hist_width, y+hist_height);
		cv::Mat roi = cv::Mat(canvas_allHists, cv::Rect(p1, p2));
		canvas.copyTo(roi);

    }

	//copy the normal histogram to the big image
	putText(histimage,"Current frame",cv::Point(0, 20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
	cv::Mat roi_o = cv::Mat(canvas_allHists, cv::Rect(cv::Point(0, 0), cv::Point(hist_width, hist_height)));
	histimage.copyTo(roi_o);

	cv::imshow("Histograms", canvas_allHists);

}

void Textons::drawTextonAnotatedImage(cv::Mat grayframe) {

    int middleid = floor((float)patch_size/2.0); // for gradient, asumes, texton size is odd!
    int16_t sample[patch_square_size];
    int16_t sample_dx[patch_square_size]; // gradient;

    int line_width =1;

    cv::cvtColor(grayframe, frame_Gtextoncolor, CV_GRAY2BGR);
    cv::cvtColor(grayframe, frame_Itextoncolor, CV_GRAY2BGR);
    frame_Gtextontexton = cv::Mat::zeros(grayframe.rows,grayframe.cols,CV_8UC1);
	//    grayframe.copyTo(frame_Gtextontexton);
    grayframe.copyTo(frame_Itextontexton);

    // for all possible patches (non overlapping)
    for(int x=0;x<grayframe.cols-patch_size;x=x+patch_size){
        for(int y=0;y<grayframe.rows-patch_size;y=y+patch_size){

            //extract the next patch to a temporary vector
            for (int xx=0;xx<patch_size;xx++) {
                for (int yy=0;yy<patch_size;yy++) {

                    //copy data to sample
                    sample[yy*patch_size+xx] = grayframe.at<uint8_t>(y+yy,x+xx);

                    //calculate x gradient into sample_dx:
                    if (xx>middleid ) {
                        sample_dx[yy*patch_size+xx] = (int)(0x00ff &grayframe.at<uint8_t>(y+yy,x+xx)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
                    } else if ( xx < middleid ) {
                        sample_dx[yy*patch_size+xx] = (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx));
                    } else {
                        sample_dx[yy*patch_size+xx] = (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx));
                        sample_dx[yy*patch_size+xx] += (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
                        sample_dx[yy*patch_size+xx] /=2;
                    }
					// grayframe.at<uint8_t>(y+yy,x+xx) = 255; // uncomment to visualise picking
                }
            }

            //get the distances to this patch to the textons...
            std::vector<float> distances_gr(n_textons_gradient); // distances
            std::vector<float> distances_i(n_textons_intensity); // distances

            for(int j=0;j<n_textons;j++) {
                if (j < n_textons_intensity) {
                    distances_i.at(j) = getEuclDistance(sample,j);
                } else {
                    distances_gr.at(j-n_textons_intensity) = getEuclDistance(sample_dx,j);
                }
            }

            //...and find out the closest texton:
			int min_id_i = std::min_element(distances_i.begin(), distances_i.end()) - distances_i.begin(); //jàààhoor
			int min_id_gr = std::min_element(distances_gr.begin(), distances_gr.end()) - distances_gr.begin() + n_textons_intensity; //jàààhoor

            //draw colored rectangle:
            cv::rectangle(frame_Gtextoncolor,cv::Point(x, y),cv::Point(x+5, y+5),getColor(min_id_gr), line_width, 8, 0);
            cv::rectangle(frame_Itextoncolor,cv::Point(x, y),cv::Point(x+5, y+5),getColor(min_id_i), line_width, 8, 0);

            //encode the texton
            //copy the closest patch into the image
            std::vector<int16_t> v_gr =textons[min_id_gr];
            std::vector<int16_t> v_i =textons[min_id_i];
            for (int i=0;i<patch_size;i++) {
                for (int j=0;j<patch_size;j++) {
                    // sample[i*patch_size+j] = grayframe.at<uint8_t>(y+j,x+i);
                    frame_Gtextontexton.at<uint8_t>(y+j,x+i) = v_gr[i*patch_size+j];
                    frame_Itextontexton.at<uint8_t>(y+j,x+i) = v_i[i*patch_size+j];
                }
            }


        }
    }

    cv::applyColorMap(frame_Gtextontexton,frame_Gtextontexton,2);
    

	//    cv::imshow("TextonColors gradient", frame_Gtextoncolor);
	//    cv::imshow("TextonColors intensity", frame_Itextoncolor);
	//    cv::imshow("TextonEncoded gradient", frame_Gtextontexton);
	//    cv::imshow("TextonEncoded intensity", frame_Itextontexton);

}

void Textons::drawGraph(std::string msg) {

    cv::Scalar color_gt= cv::Scalar(255,0,0); // blue
    cv::Scalar color_nn= cv::Scalar(0,0,255); // red
    cv::Scalar color_vert= cv::Scalar(0,255,255); // orange
    cv::Scalar color_invert= cv::Scalar(255,255,0); // light blue
    int line_width = 1;

    int stepX = distribution_buf_size/700;
    int rows = 300;

    graphFrame = cv::Mat::zeros(rows, (distribution_buf_size/stepX), CV_8UC3);

    double max;
    cv::minMaxIdx(graph_buffer,NULL,&max,NULL,NULL);
    float scaleY = rows/max;

    float nn,gt;
    float prev_nn = 0, prev_gt = 0;


    int positive_true=0;
    int positive_false=0;
    int negative_true=0;
    int negative_false=0;

    countsincelearn++; // keep track of training/test data
	std::cout << "csll: " << countsincelearn << " \n";

    int learnborder =  (lastLearnedPosition+(distribution_buf_size-distribution_buf_pointer)) % distribution_buf_size; // make a sliding graph
    if ( countsincelearn > distribution_buf_size) {
        learnborder=0;
    }

    for (int j = filterwidth; j < distribution_buf_size ; j++)
    {

        int jj = (j+distribution_buf_pointer) % distribution_buf_size; // make it a sliding graph
        nn = graph_buffer.at<float>(jj,0);
        gt = graph_buffer.at<float>(jj,1);

        if (j > learnborder ) { // change color according to training/test data
            color_nn= cv::Scalar(0,0,255); // red
        } else {

            color_nn= cv::Scalar(0,255,0); // green
        }


        if (!(groundtruth_buffer.at<float>(jj) < 6.201 && groundtruth_buffer.at<float>(jj) > 6.199)) {

			//draw a small colored line above to indicate what the drone will do:
			if (nn < threshold_nn && gt > threshold_gt) {
				//false negative; drone should stop according to stereo, but didn't if textons were used
				//white
				negative_false++;
				cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(255,255,255), line_width, 8, 0);
			} else if (nn > threshold_nn && gt < threshold_gt) {
				//false positive; drone could have proceeded according to stereo, but stopped if textons were used
				//black
				positive_false++;
				cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,0,0), line_width, 8, 0);
			} else if (nn > threshold_nn && gt > threshold_gt) {
				//true positive; both stereo and textons agree, drone should stop
				//red
				negative_true++;
				cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,0,255), line_width, 8, 0);
			} else {
				//both stereo and textons agree, drone may proceed
				//green
				positive_true++;
				cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,255,0), line_width, 8, 0);
			}

        }


        nn = graph_buffer.at<float>(jj,0)*scaleY;
        gt = graph_buffer.at<float>(jj,1)*scaleY;


        if (j==filterwidth) { // fixes discontinuty at the start of the graph
            prev_nn = nn;
            prev_gt = gt;
        }

        //draw knn result:
        cv::line(graphFrame, cv::Point(j/stepX, rows- prev_nn), cv::Point((j+1)/stepX, rows -  nn), color_nn, line_width, 8, 0);
        //draw stereo vision groundtruth:
        cv::line(graphFrame, cv::Point(j/stepX, rows- prev_gt), cv::Point((j+1)/stepX, rows -  gt),color_gt, line_width, 8, 0);
        //draw stereo vision threshold:
        //cv::line(graphFrame, cv::Point(j/stepX, rows- 90), cv::Point((j+1)/stepX, rows -  90),color_gt, line_width, 8, 0);


        prev_nn = nn;
        prev_gt = gt;
    }


	//TODO: move the following draw function out of the loop
	//draw nn vision threshold:
	cv::line(graphFrame, cv::Point(0, rows- threshold_nn*scaleY), cv::Point(graphFrame.cols, rows -  threshold_nn*scaleY),color_invert, line_width, 8, 0);
	putText(graphFrame,"est.thresh.",cv::Point(0, rows- threshold_nn*scaleY+10),cv::FONT_HERSHEY_SIMPLEX,0.4,color_invert);
	//draw gt vision threshold:
	cv::line(graphFrame, cv::Point(0, rows- threshold_gt*scaleY), cv::Point(graphFrame.cols, rows -  threshold_gt*scaleY),color_vert, line_width, 8, 0);
	putText(graphFrame,"gt.thresh.",cv::Point(0, rows- threshold_gt*scaleY-10),cv::FONT_HERSHEY_SIMPLEX,0.4,color_vert);



#ifdef _PC
    //calculate fp/fn ratio
    float tpr = (float)positive_true /(float)(positive_true+negative_false);
	float fpr = (float)positive_false /(float)(negative_true+positive_false);

    std::stringstream s;
	s << msg << " TPR: " << boost::format("%.2f")%tpr << " --> FPR: " << boost::format("%.2f")%fpr;
    msg = s.str();

#endif


    //draw text to inform about the mode and ratios or to notify user a key press was handled
	putText(graphFrame,msg,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,0.7,color_vert);

}

void Textons::setAutoThreshold() {


	int imsize = 400;
#ifdef DRAWVIZS
	frame_ROC = cv::Mat::zeros(imsize,imsize,CV_8UC3);

	/*** create axis and labels ***/
	//Y
	std::string ylabel = "TPR";
	putText(frame_ROC,ylabel,cv::Point(10, 20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
	cv::line(frame_ROC,cv::Point(0, 0),cv::Point(0, imsize),cv::Scalar(255,255,255), 1, 8, 0);
	//draw tpr threshold (which is fixed)
	cv::line(frame_ROC,cv::Point(0,imsize- tpr_threshold*imsize),cv::Point(imsize,imsize - tpr_threshold*imsize),cv::Scalar(127,127,255), 1, 8, 0);

	//X
	std::string xlabel = "FPR";
	putText(frame_ROC,xlabel,cv::Point(imsize-30, imsize-20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
	cv::line(frame_ROC,cv::Point(0,imsize-1),cv::Point(imsize,imsize-1 ),cv::Scalar(255,255,255), 1, 8, 0);



	cv::Scalar c = getColor(2);
	int line_width=2;
#endif

	bool done = false;
	cv::Mat tprs(threshold_gt,1,CV_32F); // these two arrays could be optimised away, or used for nicer graph
	cv::Mat fprs(threshold_gt,1,CV_32F);

	int best = 0;
	float fpr_best_tmp = 99999999;

	for (int i = threshold_gt; i > 0; i-- ) {

		int positive_true=0;
		int positive_false=0;
		int negative_true=0;
		int negative_false=0;

		for (int j = filterwidth; j < distribution_buf_size ; j++)
		{
			float nn,gt;
			int jj = (j+distribution_buf_pointer) % distribution_buf_size; // make it a sliding graph
			nn = graph_buffer.at<float>(jj,0);
			gt = graph_buffer.at<float>(jj,1);


			if (!(groundtruth_buffer.at<float>(jj) < 6.201 && groundtruth_buffer.at<float>(jj) > 6.199)) {

				//draw a small colored line above to indicate what the drone will do:
				if (nn < i && gt > threshold_gt) {
					//false negative; drone should stop according to stereo, but didn't if textons were used (miss)
					negative_false++;
				} else if (nn > i && gt < threshold_gt) {
					//false positive; drone could have proceeded according to stereo, but stopped if textons were used (false alarm)
					positive_false++;
				} else if (nn > i && gt > threshold_gt) {
					//true positive; both stereo and textons agree, drone should stop
					negative_true++;
				} else {
					//both stereo and textons agree, drone may proceed
					positive_true++;
				}
			}
		}

		//calculate fp/fn ratio
		float tpr = (float)positive_true /(float)(positive_true+negative_false);
		float fpr = (float)positive_false /(float)(negative_true+positive_false);

		tprs.at<float>(i) = tpr;
		fprs.at<float>(i) = fpr;


		if (tpr > tpr_threshold && fpr < fpr_best_tmp) {
			best = i;
			fpr_best_tmp = fprs.at<float>(best);
		}


#ifdef DRAWVIZS		
		cv::line(frame_ROC,cv::Point(fpr*imsize,imsize- tpr*imsize),cv::Point(fpr*imsize, imsize-tpr*imsize),c, line_width, 8, 0);		
#endif


	}

	threshold_nn = best;
	fpr_best = fpr_best_tmp;
#ifdef DRAWVIZS
	std::stringstream s;
	s << "est.thresh. = " << best;
	putText(frame_ROC,s.str(),cv::Point(fpr_best*imsize+5, imsize/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(127,127,255));
	//draw threshold
	cv::line(frame_ROC,cv::Point(fpr_best*imsize, 0),cv::Point(fpr_best*imsize, imsize),cv::Scalar(127,127,255), 1, 8, 0);
#endif


}

//calculates the histogram/distribution
void Textons::getTextonDistributionFromImage(cv::Mat grayframe, float avgdisp, bool activeLearning) {

    int middleid = floor((float)patch_size/2.0); // for gradient, asumes, texton size is odd!
    int16_t sample[patch_square_size];
    int16_t sample_dx[patch_square_size]; // gradient
    cv::Mat hist;
	hist = cv::Mat::zeros(1, n_textons+1, cv::DataType<float>::type); // +1 -> entropy

    for(int n=0;n<n_samples;n++){

        //extract a random patch to a temporary vector
        int x = rand() % (grayframe.cols-patch_size);
        int y = rand() % (grayframe.rows-patch_size);
        for (int xx=0;xx<patch_size;xx++) {
            for (int yy=0;yy<patch_size;yy++) {

                //copy data to sample
                sample[yy*patch_size+xx] = grayframe.at<uint8_t>(y+yy,x+xx);

                //calculate x gradient into sample_dx:
                if (xx>middleid ) {
                    sample_dx[yy*patch_size+xx] = (int)(0x00ff &grayframe.at<uint8_t>(y+yy,x+xx)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
                } else if ( xx < middleid ) {
                    sample_dx[yy*patch_size+xx] = (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx));
                } else {
                    sample_dx[yy*patch_size+xx] = (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx));
                    sample_dx[yy*patch_size+xx] += (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
                    sample_dx[yy*patch_size+xx] /=2;
                }
				// grayframe.at<uint8_t>(y+yy,x+xx) = 255; // uncomment to visualise picking
            }
        }

		//        if (n==0) { // visualise a patch
		//            cv::Mat test((int)patch_size,(int)patch_size,CV_8UC1, *sample);
		//            imshow("patch", test );
		//        }

		if (method==CUMULATIVE) {
			//get the and sum distances to this patch to the textons...
			std::vector<float> distances_gr(n_textons_gradient); // distances, TODO: float does not seem necessary -> int?
			std::vector<float> distances_i(n_textons_intensity); // distances
			float sum_i=0;
			float sum_gr=0;

			for(int j=0;j<n_textons;j++) {
				if (j < n_textons_intensity) {
					distances_i.at(j) = getEuclDistance(sample,j);
					hist.at<float>(j) = distances_i.at(j);
					sum_i +=hist.at<float>(j);
				} else {
					distances_gr.at(j-n_textons_intensity) = getEuclDistance(sample_dx,j);
					hist.at<float>(j) = distances_gr.at(j-n_textons_intensity);
					sum_gr +=hist.at<float>(j);
				}
			}

			//normalize
			for(int j=0;j<n_textons;j++) {
				if (j < n_textons_intensity) {
					hist.at<float>(j) /=sum_i;
				} else {
					hist.at<float>(j) /=sum_gr;
				}
			}

		}  else {

			cv::Mat distances_gr = cv::Mat::zeros(n_textons_gradient,1,CV_32F);
			cv::Mat distances_i = cv::Mat::zeros(n_textons_intensity,1,CV_32F);

			for(int j=0;j<n_textons;j++) {
				if (j < n_textons_intensity) {
					distances_i.at<float>(j,0) = getEuclDistance(sample,j);
				} else {
					distances_gr.at<float>(j-n_textons_intensity,0) = getEuclDistance(sample_dx,j);
				}
			}
			cv::Point min_element_gr,min_element_i;
			cv::minMaxLoc(distances_i,NULL, NULL, &min_element_i,NULL);
			cv::minMaxLoc(distances_gr,NULL, NULL, &min_element_gr,NULL);


			hist.at<float>(min_element_i.y) = hist.at<float>(min_element_i.y) + hist_step;
			hist.at<float>(min_element_gr.y + n_textons_intensity) = hist.at<float>(min_element_gr.y + n_textons_intensity) + hist_step;


		}
    }

	//float sum = cv::sum(hist)(0);
	float entropy =0;
	for (int i = 0 ; i < n_textons; i++) {
		float f = hist.at<float>(i); // sum;
		if (f!=0) {
			entropy  = entropy  - f * log(f)/log(2);
		}
	}
	hist.at<float>(n_textons) = entropy;
	std:: cout << "Entropy: " << entropy << std::endl;


    //copy new data into learning buffer:
    cv::Mat M1 = distribution_buffer.row((distribution_buf_pointer+0) % distribution_buf_size) ;
    hist.convertTo(M1,cv::DataType<float>::type,1,0); // floats are needed for knn
    groundtruth_buffer.at<float>((distribution_buf_pointer)% distribution_buf_size) = avgdisp_smoothed;
    //run knn
    float nn = knn.find_nearest(M1,k,0,0,0,0); // if segfault here, clear xmls!
    //perform smoothing:
    nn = knn_smoothed.addSample(nn);

    //smooth gt, but exclude std filter from smoothing:
    if (avgdisp >0.1 ) {
        avgdisp_smoothed = gt_smoothed.addSample(avgdisp); // perform smoothing
    }
    else {
        avgdisp_smoothed = 0;
    }
    avgdisp_smoothed = avgdisp; // nah, nm


    if (!activeLearning) {        //if not active learning, learn all samples
        distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
    } else {            //otherwise, only learn errornous samples
        if (nn < threshold_nn && avgdisp_smoothed > threshold_gt) {
            //false negative; drone should stop according to stereo, but didn't if textons were used
            distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
        } else if (nn > threshold_nn && avgdisp_smoothed < threshold_gt) {
            //false positive; drone could have proceeded according to stereo, but stopped if textons were used
            distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
        }
    }

	//std::cout << "hist" << distribution_buf_pointer << ": " << hist << std::endl;



    std::cout << "knn.disp.: " << nn << "  |  truth: " << avgdisp_smoothed << std::endl;

    //save values for visualisation	in graph
    graph_buffer.at<float>((distribution_buf_pointer+0) % distribution_buf_size,0) = nn;
    graph_buffer.at<float>(distribution_buf_pointer,1) = avgdisp_smoothed; // groundtruth

#ifdef DRAWVIZS
	frame_currentHist = drawHistogram(hist,n_textons,200);
	//drawMeanHists(frame_currentHist);
    drawTextonAnotatedImage(grayframe);
#endif
}

/*
 * Retrieves and returns the last added ground truth
 */
int Textons::getLast_gt() {
    int gt = graph_buffer.at<float>(distribution_buf_pointer,1);
    return gt;
}

/*
 * Retrieves and returns the last added texton result
 */
int Textons::getLast_nn() {
    int nn = graph_buffer.at<float>(distribution_buf_pointer,0);
    return nn;
}

inline bool checkFileExist (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

/*
 * Loads the texton dictionaries from file
 */
int Textons::initTextons() {
    std::cout << "Opening textons file\n";

    if (!checkFileExist("../textons10_gradient.dat")) {std::cerr << "Error: gradient textons not available\n";return 0;}
    if (!checkFileExist("../textons10_intensity.dat")) {std::cerr << "Error: intensity textons not available\n";return 0;}

    std::ifstream input_gr("../textons10_gradient.dat", std::ios::binary );
    std::ifstream input_i("../textons10_intensity.dat", std::ios::binary );
    // copies all data into buffer
    std::vector<unsigned char> buffer_gr(( std::istreambuf_iterator<char>(input_gr)),(std::istreambuf_iterator<char>()));
    std::vector<unsigned char> buffer_i(( std::istreambuf_iterator<char>(input_i)),(std::istreambuf_iterator<char>()));

    n_textons_gradient = buffer_gr[0];
    n_textons_intensity = buffer_i[0];
    n_textons =  n_textons_gradient  + n_textons_intensity;

    if (buffer_gr[1] != buffer_i[1]) {std::cerr << "Error: patch sizes don't match\n";return 0;}
    patch_size = buffer_gr[1];
    patch_square_size = patch_size*patch_size;


    textons.resize(n_textons);
    printf("#textons: %d, Patch size: %d, #samples: %d\n",n_textons,patch_size,n_samples );

    int counter =2; // skip first two bytes, as they contain other info
    for (int i = 0;i<n_textons_intensity;i++) {
        std::vector<int16_t> v(patch_square_size);
		//        printf("texton[%d]:", i);
        for (int j=0;j<patch_square_size;j++) {

            uint8_t t0 = buffer_i[counter];
            uint8_t t1 = buffer_i[counter+1];
            int16_t t = ((t1 << 8) & 0xff00) |  (t0 & 0x00ff);
            counter +=2;
            v[j] = t;
			//            if (j>0) {printf(", %d", v[j]);} else {printf(" %d", v[j]);}
        }
        textons[i] = v;
        input_i.close();
		//        std::cout << std::endl;
    }


    counter =2; // skip first two bytes, as they contain other info
    for (int i = 0;i<n_textons_gradient;i++) {
        std::vector<int16_t> v(patch_square_size);
		//        printf("texton_gr[%d]:", i+buffer_i[0]);
        for (int j=0;j<patch_square_size;j++) {

            uint8_t t0 = buffer_gr[counter];
            uint8_t t1 = buffer_gr[counter+1];
            int16_t t = ((t1 << 8) & 0xff00) |  (t0 & 0x00ff);
            counter +=2;
            v[j] = t;
			//            if (j>0) {printf(", %d", v[j]);} else {printf(" %d", v[j]);}
        }
        textons[i+n_textons_intensity] = v;
        input_gr.close();
		//        std::cout << std::endl;
    }

    return 1;
}

/*
 * Clears and initialises the learned buffer. Can also train on null data, actively reseting knn
 */
bool Textons::initLearner(bool nulltrain) {
    srand (time(NULL));
	distribution_buffer = cv::Mat::zeros(distribution_buf_size, n_textons+1, cv::DataType<float>::type);
    groundtruth_buffer = cv::Mat::zeros(distribution_buf_size, 1, cv::DataType<float>::type);
    graph_buffer = cv::Mat::zeros(distribution_buf_size, 2, cv::DataType<float>::type);
    groundtruth_buffer = groundtruth_buffer +6.2; //for testing...
    distribution_buf_pointer = 0;
    knn_smoothed.init(filterwidth);
    gt_smoothed.init(filterwidth);
    lastLearnedPosition = 0;
    countsincelearn=0;
    if (nulltrain) {
        return knn.train(distribution_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );
    } else {
        return true;
    }
}

/*
 *  Retrains knn on all available data accumulated in the buffer
 */
void Textons::retrainAll() {
    std::cout << "Training knn regression:\n";
    knn.train(distribution_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );
    lastLearnedPosition = (distribution_buf_pointer +1 )% distribution_buf_size;
    countsincelearn=0;

#ifdef _PC
    //redraw the graph and reinit the smoother
    //may take significant time if learning buffer is big, so don't perform onboard drone
    std::cout << "Initialising smoother:\n";
    for (int i=0; i<distribution_buf_size; i++) {

        int jj = (i+distribution_buf_pointer) % distribution_buf_size;
        cv::Mat M1 = distribution_buffer.row(jj); // prevent smoothing filter discontinuity
        graph_buffer.at<float>(jj,0) = knn_smoothed.addSample(knn.find_nearest(M1,k,0,0,0,0));
        gt_smoothed.addSample(graph_buffer.at<float>(jj,1)); // prepare the gt filter, not really necessary
    }
#endif
}

/*
 *  Loads the learning buffer from xml file previously saved, also performs retrain
 */
int Textons::loadPreviousRegression() {
    try {
        std::cout << "Opening memory files\n";


        if (!checkFileExist("../distribution_buffer.xml")) {std::cout << "Warning: no previous data found\n";return 0;}
        cv::FileStorage dist_fs("../distribution_buffer.xml", cv::FileStorage::READ);
        dist_fs["distribution_buffer"] >>distribution_buffer;
        cv::FileStorage ground_fs("../groundtruth_buffer.xml", cv::FileStorage::READ);
        ground_fs["groundtruth_buffer"] >> groundtruth_buffer;
        cv::FileStorage graph_fs("../graph_buffer.xml", cv::FileStorage::READ);
        graph_fs["graph_buffer"] >> graph_buffer;

        cv::FileStorage where_fs("../distribution_buf_pointer.xml", cv::FileStorage::READ);
        where_fs["distribution_buf_pointer"] >> distribution_buf_pointer;

        std::cout << "Training...\n";
        //draw the training set results:
        retrainAll();

    } catch (int e) {
        std::cout << "No previous regression memory could be openend.\n";
    }
}

/*
 * Save the current learning buffer to xml files. Also train on the currently available data.
 */
void Textons::saveRegression() {
    cv::FileStorage dist_fs("../distribution_buffer.xml", cv::FileStorage::WRITE);
    dist_fs << "distribution_buffer" << distribution_buffer;

    cv::FileStorage ground_fs("../groundtruth_buffer.xml", cv::FileStorage::WRITE);
    ground_fs << "groundtruth_buffer" << groundtruth_buffer;

    cv::FileStorage graph_fs("../graph_buffer.xml", cv::FileStorage::WRITE);
    graph_fs << "graph_buffer" << graph_buffer;

    cv::FileStorage where_fs("../distribution_buf_pointer.xml", cv::FileStorage::WRITE);
    where_fs << "distribution_buf_pointer" << distribution_buf_pointer;

    cv::FileStorage size_fs("../distribution_buf_size.xml", cv::FileStorage::WRITE);
    size_fs << "distribution_buf_size" << distribution_buf_size;

    retrainAll();
}

/*
 * Reloads the learning buffer from file and retrains the learner with that data,
 * effectively reseting the learner to save point
 */
void Textons::reload() {
    initLearner(false);
    loadPreviousRegression();
}



