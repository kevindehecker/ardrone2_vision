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
int Textons::init (int * result_input2Mode) {
	this->result_input2Mode = result_input2Mode;

	if (initTextons()) {return 1;}
	if (initLearner(true)) {return 1;}
    loadPreviousRegression();


#ifdef HASSCREEN
	frame_ROC = cv::Mat::zeros(400,400,CV_8UC3);
#endif

	return 0;
}

/*
*Calculates the Eucladian distance between a patch and a texton
*/
double Textons::getEuclDistance(int16_t sample[], int texton_id) {
    int sum =0;
    std::vector<int16_t> v =textons[texton_id];
    std::vector<int16_t> sample_;
    sample_.assign(sample, sample + patch_square_size);

	for(int i = 0; i < patch_square_size; i++) {
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
	int line_width_half = 5;


    // Display each histogram in a canvas
	canvas = cv::Mat::zeros(maxY, bins*line_width, CV_8UC3);

    int rows = canvas.rows;
	for (int j = 0; j < bins; j++) {
        cv::Scalar c = getColor(j);
		cv::line(canvas,cv::Point(j*line_width+line_width_half , rows),cv::Point(j*line_width+line_width_half , rows - (150*hist.at<float>(j))),c, line_width, 8, 0);
	}


	//copy the normal histogram to the big image
	std::stringstream s;
	s << "Entropy: " << hist.at<float>(n_textons) ;
	putText(canvas,s.str(),cv::Point(0, 20),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(255,255,255));
	cv::line(canvas,cv::Point(canvas.cols/2,45),cv::Point(canvas.cols/2, canvas.rows ),cv::Scalar(180,180,180), 1, 8, 0);

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

void Textons::drawRegressionGraph(std::string msg) {

    cv::Scalar color_gt= cv::Scalar(255,0,0); // blue
	cv::Scalar color_est= cv::Scalar(0,0,255); // red
    cv::Scalar color_vert= cv::Scalar(0,255,255); // orange
    cv::Scalar color_invert= cv::Scalar(255,255,0); // light blue
    int line_width = 1;



	const int fsizex = 1280;
	const int fsizey = 300;
	const int barsize =10;
	double max;
	cv::minMaxIdx(graph_buffer,NULL,&max,NULL,NULL);

	const float scaleX = (float)((fsizex))/(distribution_buf_size);
	const int rows = (fsizey-barsize);
	const float scaleY = (rows)/max;

	//frame_regressGraph = cv::Mat::zeros(300, ((distribution_buf_size)*scaleX ), CV_8UC3);
	frame_regressGraph = cv::Mat::zeros(fsizey, fsizex, CV_8UC3);

	cv::Point p1(0,barsize);
	cv::Point p2(frame_regressGraph.cols, frame_regressGraph.rows);
	cv::Mat graphFrame = cv::Mat(frame_regressGraph, cv::Rect(p1, p2));

	float prev_est = 0, prev_gt = 0;
    int positive_true=0;
    int positive_false=0;
    int negative_true=0;
    int negative_false=0;

	_mse_tst = 0;
	_mse_tst_cnt = 0;
	_mse_trn = 0;
	_mse_trn_cnt = 0;

	int learnborder =  ((lastLearnedPosition)+((distribution_buf_size)-(distribution_buf_pointer))) % (distribution_buf_size); // make a sliding graph
    if ( countsincelearn > (distribution_buf_size)) {
        learnborder=0;
    }

    for (int j = filterwidth; j < distribution_buf_size ; j++)
    {

        int jj = (j+(distribution_buf_pointer)) % (distribution_buf_size); // make it a sliding graph
		float est =0,gt =0;
		est = graph_buffer.at<float>(jj,0);
		gt = graph_buffer.at<float>(jj,1);
		

		if (j==filterwidth) { // fixes discontinuty at the start of the graph
			prev_est = est;
			prev_gt = gt;
		}

        if (!(groundtruth_buffer.at<float>(jj) < 6.201 && groundtruth_buffer.at<float>(jj) > 6.199)) {

			if (j > learnborder ) { // change color according to training/test data
				color_est= cv::Scalar(0,0,255); // red
				_mse_tst += (gt - est) * (gt - est);
				_mse_tst_cnt++;
			} else {
				color_est= cv::Scalar(0,255,0); // green
				_mse_trn += (gt - est) * (gt - est);
				_mse_trn_cnt++;
			}


			//draw a small colored line above to indicate what the drone will do:
			if (est < threshold_est && gt > threshold_gt) {
				//false negative; drone should stop according to stereo, but didn't if textons were used
				//white
				if (j > learnborder ) {negative_false++;}
				cv::line(frame_regressGraph,cv::Point(j*scaleX , 0),cv::Point(j*scaleX , barsize),cv::Scalar(255,255,255), line_width, 8, 0);
			} else if (est > threshold_est && gt < threshold_gt) {
				//false positive; drone could have proceeded according to stereo, but stopped if textons were used
				//black
				if (j > learnborder ) {positive_false++;}
				cv::line(frame_regressGraph,cv::Point(j*scaleX , 0),cv::Point(j*scaleX  , barsize),cv::Scalar(0,0,0), line_width, 8, 0);
			} else if (est > threshold_est && gt > threshold_gt) {
				//true positive; both stereo and textons agree, drone should stop
				//red
				if (j > learnborder ) {negative_true++;}
				cv::line(frame_regressGraph,cv::Point(j*scaleX , 0),cv::Point(j*scaleX , barsize),cv::Scalar(0,0,255), line_width, 8, 0);
			} else {
				//both stereo and textons agree, drone may proceed
				//green
				if (j > learnborder ) {positive_true++;}
				cv::line(frame_regressGraph,cv::Point(j*scaleX , 0),cv::Point(j*scaleX , barsize),cv::Scalar(0,255,0), line_width, 8, 0);
			}





			//draw knn result:
			cv::line(graphFrame, cv::Point(j*scaleX , rows- prev_est), cv::Point((j+1)*scaleX , rows -  est), color_est, line_width, CV_AA, 0);
			//draw stereo vision groundtruth:
			if (gt>5) { // ignore instances with unknown groundtruth (minDisparity >5). TODO: make minDispairty a const
				cv::line(graphFrame, cv::Point(j*scaleX , rows- prev_gt), cv::Point((j+1)*scaleX , rows -  gt),color_gt, line_width, CV_AA, 0);
				prev_gt = gt;
			}
			prev_est = est;
		}

    }

	// calc mean square errors
	_mse_tst /= _mse_tst_cnt;
	_mse_trn /= _mse_trn_cnt;
	//std::stringstream s_mse;
	//s_mse << "mse trn: " << (int)_mse_trn << ", tst: " << (int)_mse_tst;
	//putText(frame_regressGraph,s_mse.str(),cv::Point(0, 50),cv::FONT_HERSHEY_SIMPLEX,0.5,color_vert);


	//draw est vision threshold:
	cv::line(graphFrame, cv::Point(0, rows- threshold_est*scaleY), cv::Point(frame_regressGraph.cols, rows -  threshold_est*scaleY),color_invert, line_width, 8, 0);
	putText(frame_regressGraph,"est",cv::Point(0, rows- threshold_est*scaleY+10+barsize),cv::FONT_HERSHEY_SIMPLEX,0.4,color_invert);
	//draw gt vision threshold:
	cv::line(graphFrame, cv::Point(0, rows- threshold_gt*scaleY), cv::Point(frame_regressGraph.cols, rows -  threshold_gt*scaleY),color_vert, line_width, 8, 0);
	putText(frame_regressGraph,"gt.",cv::Point(0, rows- threshold_gt*scaleY-10+barsize),cv::FONT_HERSHEY_SIMPLEX,0.4,color_vert);



#ifdef _PC
    //calculate fp/fn ratio
    float tpr = (float)positive_true /(float)(positive_true+negative_false);
	float fpr = (float)positive_false /(float)(negative_true+positive_false);

    std::stringstream s;
	s << msg << " TPR: " << boost::format("%.2f")%tpr << " --> FPR: " << boost::format("%.2f")%fpr;
	s << ". MSE trn: " << (int)_mse_trn << ", tst: " << (int)_mse_tst;
    msg = s.str();

#endif


    //draw text to inform about the mode and ratios or to notify user a key press was handled
	putText(frame_regressGraph,msg,cv::Point(0, rows+barsize-2),cv::FONT_HERSHEY_SIMPLEX,0.5,color_vert);

}

void Textons::setAutoThreshold() {

#ifdef DRAWVIZS
	int imsize = 400;
	cv::Mat graphframe;
	if (*result_input2Mode == VIZ_ROC ) {

		int border = 20;

		cv::Point p1(border,0);
		cv::Point p2(imsize+border, imsize);

		frame_ROC = cv::Mat::zeros(imsize+border,imsize+border,CV_8UC3);
		graphframe = cv::Mat(frame_ROC, cv::Rect(p1, p2));

		//graphframe =cv::Scalar(255,255,255);

		/*** create axis and labels ***/
		//Y
		putText(frame_ROC,"T",cv::Point(4, imsize/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		putText(frame_ROC,"P",cv::Point(4, 15+imsize/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		putText(frame_ROC,"R",cv::Point(4, 30+imsize/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		cv::line(graphframe,cv::Point(0, 0),cv::Point(0, imsize),cv::Scalar(255,255,255), 1, 8, 0);
		//draw tpr threshold (which is fixed)
		cv::line(graphframe,cv::Point(0,imsize- tpr_threshold*imsize),cv::Point(imsize,imsize - tpr_threshold*imsize),cv::Scalar(127,127,255), 1, 8, 0);

		//X
		std::string xlabel = "FPR";
		putText(frame_ROC,xlabel,cv::Point(imsize/2, imsize+border-7),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		cv::line(graphframe,cv::Point(0,imsize-1),cv::Point(imsize,imsize-1 ),cv::Scalar(255,255,255), 1, 8, 0);

		// 0 scale
		putText(frame_ROC,"0",cv::Point(4, imsize+border-7),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		// 1 scale
		putText(frame_ROC,"1",cv::Point(4, 12),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
		putText(frame_ROC,"1",cv::Point(imsize+border-10, imsize+border-7),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));



	}
	cv::Scalar c = getColor(2);
	int line_width=2;
#endif

	bool done = false;
	cv::Mat tprs_trn(threshold_gt,1,CV_32F); // these two arrays could be optimised away, or used for nicer graph
	cv::Mat fprs_trn(threshold_gt,1,CV_32F);
	cv::Mat tprs_tst(threshold_gt,1,CV_32F);
	cv::Mat fprs_tst(threshold_gt,1,CV_32F);

	int best = 0;
	float fpr_tst_best_tmp = 99999999; //smaller is better, so start the search extremely high
	float fpr_trn_best_tmp = 99999999; //smaller is better, so start the search extremely high

	int learnborder =  (lastLearnedPosition+(distribution_buf_size-distribution_buf_pointer)) % distribution_buf_size; // make a sliding graph
	if ( countsincelearn > distribution_buf_size) {
		learnborder=0;
	}

	for (int i = threshold_gt; i > 0; i-- ) {

		int positive_true_trn=0;
		int positive_false_trn=0;
		int negative_true_trn=0;
		int negative_false_trn=0;


		int positive_true_tst=0;
		int positive_false_tst=0;
		int negative_true_tst=0;
		int negative_false_tst=0;

		for (int j = filterwidth; j < distribution_buf_size ; j++) {
			float est,gt;
			int jj = (j+distribution_buf_pointer) % distribution_buf_size; // make it a sliding graph
			est = graph_buffer.at<float>(jj,0);
			gt = graph_buffer.at<float>(jj,1);



			if (!(groundtruth_buffer.at<float>(jj) < 6.201 && groundtruth_buffer.at<float>(jj) > 6.199) && gt > 5) {
				if (j > learnborder ) { //tst/trn
					if (est < i && gt > threshold_gt) {
						//false negative; drone should stop according to stereo, but didn't if textons were used (miss)
						negative_false_tst++;
					} else if (est > i && gt < threshold_gt) {
						//false positive; drone could have proceeded according to stereo, but stopped if textons were used (false alarm)
						positive_false_tst++;
					} else if (est > i && gt > threshold_gt) {
						//true positive; both stereo and textons agree, drone should stop
						negative_true_tst++;
					} else {
						//both stereo and textons agree, drone may proceed
						positive_true_tst++;
					}
				} else {
					if (est < i && gt > threshold_gt) {
						//false negative; drone should stop according to stereo, but didn't if textons were used (miss)
						negative_false_trn++;
					} else if (est > i && gt < threshold_gt) {
						//false positive; drone could have proceeded according to stereo, but stopped if textons were used (false alarm)
						positive_false_trn++;
					} else if (est > i && gt > threshold_gt) {
						//true positive; both stereo and textons agree, drone should stop
						negative_true_trn++;
					} else {
						//both stereo and textons agree, drone may proceed
						positive_true_trn++;
					}
				}
			}
		}

		//calculate fp/fn ratio
		float tpr_trn = (float)positive_true_trn /(float)(positive_true_trn+negative_false_trn);
		float fpr_trn = (float)positive_false_trn /(float)(negative_true_trn+positive_false_trn);
		float tpr_tst = (float)positive_true_tst /(float)(positive_true_tst+negative_false_tst);
		float fpr_tst = (float)positive_false_tst /(float)(negative_true_tst+positive_false_tst);

		tprs_trn.at<float>(i) = tpr_trn;
		fprs_trn.at<float>(i) = fpr_trn;
		tprs_tst.at<float>(i) = tpr_tst;
		fprs_tst.at<float>(i) = fpr_tst;


		if (tpr_trn > tpr_threshold && fpr_trn < fpr_trn_best_tmp) {
			best = i;
			fpr_trn_best_tmp = fprs_trn.at<float>(best);
			fpr_tst_best_tmp = fprs_tst.at<float>(best);
		}


#ifdef DRAWVIZS
		if (*result_input2Mode == VIZ_ROC ) {
			cv::line(graphframe,cv::Point(fpr_trn*imsize,imsize- tpr_trn*imsize),cv::Point(fpr_trn*imsize, imsize-tpr_trn*imsize),cv::Scalar(0,255,0), line_width, 8, 0);
			cv::line(graphframe,cv::Point(fpr_tst*imsize,imsize- tpr_tst*imsize),cv::Point(fpr_tst*imsize, imsize-tpr_tst*imsize),cv::Scalar(0,0,255), line_width, 8, 0);
		}
#endif


	}

	threshold_est = best;

	_tpr_trn = tprs_trn.at<float>(best);
	_fpr_trn = fprs_trn.at<float>(best);
	_tpr_tst = tprs_tst.at<float>(best);
	_fpr_tst = fprs_tst.at<float>(best);
#ifdef DRAWVIZS
	if (*result_input2Mode == VIZ_ROC ) {
		std::stringstream s;
		s << "est.thresh. = " << best;
		putText(graphframe,s.str(),cv::Point(fpr_trn_best_tmp*imsize+5, imsize/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(127,127,255));
		//draw fpr resulting threshold
		cv::line(graphframe,cv::Point(fpr_trn_best_tmp*imsize, 0),cv::Point(fpr_trn_best_tmp*imsize, imsize),cv::Scalar(127,127,255), 1, 8, 0);

		std::stringstream s_trn;
		if (fpr_trn_best_tmp <= 1) {
			s_trn << "TPR: " << boost::format("%.2f")%tprs_trn.at<float>(best) << " -> FPR: " << boost::format("%.2f")%fprs_trn.at<float>(best);
		} else {
			s_trn << "TPR: " << boost::format("%.2f")%tpr_threshold << " -> FPR: -";
		}		
		std::stringstream s_tst;
		s_tst << "TPR: " << boost::format("%.2f")%tprs_tst.at<float>(best) << " <> FPR: " << boost::format("%.2f")%fprs_tst.at<float>(best);
		putText(graphframe,s_tst.str(),cv::Point(imsize-200 , imsize-12),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
		putText(graphframe,s_trn.str(),cv::Point(imsize-200 , imsize-34),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0));

	}
#endif



}

//calculates the histogram/distribution
void Textons::getTextonDistributionFromImage(cv::Mat grayframe, float gt, bool activeLearning, int pauseVideo, bool stereoOK) {

    int middleid = floor((float)patch_size/2.0); // for gradient, asumes, texton size is odd!
    int16_t sample[patch_square_size];
    int16_t sample_dx[patch_square_size]; // gradient
    cv::Mat hist;
	hist = cv::Mat::zeros(1, n_textons+1, cv::DataType<float>::type); // +1 -> entropy

	int gridsize_x = (grayframe.cols-patch_size) / n_samples_sqrt;
	int gridsize_y = (grayframe.rows-patch_size) / n_samples_sqrt;
	for(int nx=0;nx<n_samples_sqrt;nx++){
		for(int ny=0;ny<n_samples_sqrt;ny++){

			//extract a random patch to a temporary vector
			//int x = rand() % (grayframe.cols-patch_size);
			//int y = rand() % (grayframe.rows-patch_size);

			//extract a patch from a grid to a temporary vector
			int x = gridsize_x * nx + (gridsize_x>>1);
			int y = gridsize_y * ny + (gridsize_y>>1);

			for (int xx=0;xx<patch_size;xx++) {
				for (int yy=0;yy<patch_size;yy++) {

					//copy data to sample
					sample[yy*patch_size+xx] = grayframe.at<uint8_t>(y+yy,x+xx);

					//calculate x gradient into sample_dx:

					if (xx==patch_size-1) {
						sample_dx[yy*patch_size+xx] = (int)(0x00ff &grayframe.at<uint8_t>(y+yy,x+xx)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
						sample_dx[yy*patch_size+xx] <<=1; // increase integer precision instead of just dividing
					} else if ( xx == 0 ) {
						sample_dx[yy*patch_size+xx] = (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx));
						sample_dx[yy*patch_size+xx] <<=1; // increase integer precision instead of just dividing
					} else {
						//this should normaly be divided by 2, but this is done through integer precision <<1 above, and divide by after distance calculation
						sample_dx[yy*patch_size+xx] = (int)(0x00ff &grayframe.at<uint8_t>(y+yy,x+xx+1)) - (int)(0x00ff & grayframe.at<uint8_t>(y+yy,x+xx-1));
					}
#ifdef DRAWVIZS
					if (*result_input2Mode == VIZ_histogram) {
						grayframe.at<uint8_t>(y+yy,x+xx) = 255; // visualize sampling
					}
#endif					
				}
			}

			//        if (n==0) { // visualise a patch
			//            cv::Mat test((int)patch_size,(int)patch_size,CV_8UC1, *sample);
			//            imshow("patch", test );
			//        }




			if (method==TEXTON_CUMULATIVE_DISTANCE) {
				//get the and sum distances to this patch to the textons...

				for(int j=0;j<n_textons;j++) {
					if (j < n_textons_intensity) {
						float dis = getEuclDistance(sample,j);
						hist.at<float>(j) = hist.at<float>(j) + dis;
					} else {
						float dis = getEuclDistance(sample_dx,j)/4; // /4 = integer precision used in case of gradient patches
						hist.at<float>(j) = hist.at<float>(j) + dis;
					}
				}
				//move normalize uit for loop
			}  else {

				cv::Mat distances_gr = cv::Mat::zeros(n_textons_gradient,1,CV_32F);
				cv::Mat distances_i = cv::Mat::zeros(n_textons_intensity,1,CV_32F);

				for(int j=0;j<n_textons;j++) {
					if (j < n_textons_intensity) {
						distances_i.at<float>(j,0) = getEuclDistance(sample,j);
					} else {
						distances_gr.at<float>(j-n_textons_intensity,0) = getEuclDistance(sample_dx,j) /4; // /4 = integer precision used in case of gradient patches
					}
				}
				cv::Point min_element_gr,min_element_i;
				cv::minMaxLoc(distances_i,NULL, NULL, &min_element_i,NULL);
				cv::minMaxLoc(distances_gr,NULL, NULL, &min_element_gr,NULL);


				hist.at<float>(min_element_i.y)++;
				hist.at<float>(min_element_gr.y + n_textons_intensity)++;


			}
		}
	}



	//normalize
	//float sum = cv::sum(hist)(0);
	float sum_i = 0;
	for(int j=0;j<n_textons_intensity;j++) {
		sum_i += hist.at<float>(j);
	}
	for(int j=0;j<n_textons_intensity;j++) {
		hist.at<float>(j) /= 2*sum_i; // /2 because two histrograms
	}


	float sum_gr = 0;
	for(int j=n_textons_intensity;j<n_textons;j++) {
		sum_gr += hist.at<float>(j);
	}
	for(int j=n_textons_intensity;j<n_textons;j++) {
		hist.at<float>(j) /= 2*sum_gr; // /2 because two histrograms
	}

	//calculate and concatenate entropy distribution
	float entropy =0;
	for (int i = 0 ; i < n_textons; i++) {
		float f = hist.at<float>(i);
		if (f!=0) {
			entropy  = entropy  - f * (log(f)/log(2));
		}
	}
	hist.at<float>(n_textons) = entropy;
	//std:: cout << "Entropy: " << entropy << std::endl;

    //copy new data into learning buffer:
    cv::Mat M1 = distribution_buffer.row((distribution_buf_pointer+0) % distribution_buf_size) ;
    hist.convertTo(M1,cv::DataType<float>::type,1,0); // floats are needed for knn
	groundtruth_buffer.at<float>((distribution_buf_pointer)% distribution_buf_size) = gt;
    //run knn
	float est = knn.find_nearest(M1,k,0,0,0,0); // if segfault here, clear xmls!
    //perform smoothing:
	est = est_smoother.addSample(est);

	if (!pauseVideo && stereoOK) {
		if (!activeLearning) {        //if not active learning, learn all samples
			distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
		} else {            //otherwise, only learn errornous samples
			if (est < threshold_est && gt > threshold_gt) {
				//false negative; drone should stop according to stereo, but didn't if textons were used
				distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
			} else if (est > threshold_est && gt < threshold_gt) {
				//false positive; drone could have proceeded according to stereo, but stopped if textons were used
				distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;
			}
		}
	}
	//std::cout << "hist" << distribution_buf_pointer << ": " << hist << std::endl;



	//std::cout << "knn.disp.: " << est << "  |  truth: " << avgdisp_smoothed << std::endl;

    //save values for visualisation	in graph
	 if (stereoOK) {
		 graph_buffer.at<float>((distribution_buf_pointer+0) % distribution_buf_size,0) = est;
		 graph_buffer.at<float>(distribution_buf_pointer,1) = gt;
	 }

#ifdef DRAWVIZS
	if (*result_input2Mode == VIZ_histogram || *result_input2Mode == VIZ_texton_intensity_color_encoding || *result_input2Mode == VIZ_texton_gradient_color_encoding ) {
		frame_currentHist = drawHistogram(hist,n_textons,200);
	}
	//drawMeanHists(frame_currentHist);
	if (*result_input2Mode == VIZ_texton_intensity_color_encoding || *result_input2Mode == VIZ_texton_gradient_color_encoding || *result_input2Mode == VIZ_texton_intensity_texton_encoding || *result_input2Mode == VIZ_texton_gradient_texton_encoding) {
		drawTextonAnotatedImage(grayframe);
	}
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
int Textons::getLast_est() {
	int est = graph_buffer.at<float>(distribution_buf_pointer,0);
	return est;
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

	if (!checkFileExist("../textons10_gradient_cubicle.dat")) {std::cerr << "Error: gradient textons not available\n";return 1;}
	if (!checkFileExist("../textons10_intensity_cubicle.dat")) {std::cerr << "Error: intensity textons not available\n";return 1;}

	std::ifstream input_gr("../textons10_gradient_cubicle.dat", std::ios::binary );
	std::ifstream input_i("../textons10_intensity_cubicle.dat", std::ios::binary );
    // copies all data into buffer
    std::vector<unsigned char> buffer_gr(( std::istreambuf_iterator<char>(input_gr)),(std::istreambuf_iterator<char>()));
    std::vector<unsigned char> buffer_i(( std::istreambuf_iterator<char>(input_i)),(std::istreambuf_iterator<char>()));

    n_textons_gradient = buffer_gr[0];
    n_textons_intensity = buffer_i[0];
    n_textons =  n_textons_gradient  + n_textons_intensity;

	if (buffer_gr[1] != buffer_i[1]) {std::cerr << "Error: patch sizes don't match\n";return 1;}
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
			v[j] = t *2; // *2 is for integer precision
			//            if (j>0) {printf(", %d", v[j]);} else {printf(" %d", v[j]);}
        }
		textons[i+n_textons_intensity] = v;
        input_gr.close();
		//        std::cout << std::endl;
    }

	return 0;
}

/*
 * Clears and initialises the learned buffer. Can also train on null data, actively reseting knn
 */
int Textons::initLearner(bool nulltrain) {
    srand (time(NULL));
	distribution_buffer = cv::Mat::zeros(distribution_buf_size, n_textons+1, cv::DataType<float>::type); // +1 for entropy
    groundtruth_buffer = cv::Mat::zeros(distribution_buf_size, 1, cv::DataType<float>::type);
    graph_buffer = cv::Mat::zeros(distribution_buf_size, 2, cv::DataType<float>::type);
    groundtruth_buffer = groundtruth_buffer +6.2; //for testing...
    distribution_buf_pointer = 0;
	est_smoother.init(filterwidth);
	//gt_smoothed.init(filterwidth);
    lastLearnedPosition = 0;
    countsincelearn=0;
    if (nulltrain) {
		knn.train(distribution_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );		
	}
	return 0;

}

/*
 *  Retrains knn on all available data accumulated in the buffer
 */
void Textons::retrainAll() {
	std::cout << "Training knn regression\n";
//	std::cout << distribution_buffer << std::endl;
//	std::cout << groundtruth_buffer << std::endl;
    knn.train(distribution_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );	
    lastLearnedPosition = (distribution_buf_pointer +1 )% distribution_buf_size;
    countsincelearn=0;

#ifdef _PC
    //redraw the graph and reinit the smoother
    //may take significant time if learning buffer is big, so don't perform onboard drone
	//std::cout << "Initialising smoother\n";
    for (int i=0; i<distribution_buf_size; i++) {
        int jj = (i+distribution_buf_pointer) % distribution_buf_size;
        cv::Mat M1 = distribution_buffer.row(jj); // prevent smoothing filter discontinuity
		graph_buffer.at<float>(jj,0) = est_smoother.addSample(knn.find_nearest(M1,k,0,0,0,0));
		//gt_smoothed.addSample(graph_buffer.at<float>(jj,1)); // prepare the gt filter, not really necessary
    }	
#endif
}

/*
 *  Loads the learning buffer from xml file previously saved, also performs retrain
 */
int Textons::loadPreviousRegression() {
    try {
        std::cout << "Opening memory files\n";


		if (!checkFileExist("../distribution_buffer.xml")) {std::cout << "Warning: no previous data found\n";return 1;}
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

void Textons::printReport(float fps) {

	drawRegressionGraph(""); // calc mse

	int tst_prc = (int)round(((((float)_mse_tst_cnt)/((float)(_mse_trn_cnt+_mse_tst_cnt)))*100.0f));
	std::cout << "\n*************************Report*************************\n";		
	std::cout << "@fps;                     : " << fps << std::endl;
	std::cout << "kNN;                     k: " << k << std::endl;
	std::cout << "Moving average width;     : " << filterwidth << std::endl;
	std::cout << "Texton patch size;        : " << (int)patch_size << std::endl;
	std::cout << "#patches per image;       : " << n_samples << std::endl;
	std::cout << "#textons;        intensity: " << (int)n_textons_intensity << "  \tgradient: " <<  (int)n_textons_gradient << std::endl;
	std::cout << "#Samples in buffer;  train: " << _mse_trn_cnt << "\ttest: " << _mse_tst_cnt << " --> " << tst_prc << "%" << std::endl ;
	std::cout << "Mean square error;   train: " << _mse_trn << "\ttest: " << _mse_tst << std::endl;
	std::cout << "True positive rate;  train: " << _tpr_trn << "\ttest: " << _tpr_tst << std::endl;
	std::cout << "False positive rate; train: " << _fpr_trn << "\ttest: " << _fpr_tst << std::endl;
	std::cout << "*********************************************************\n\n";

}

