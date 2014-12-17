#include "textons.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

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
    double distance =0;

    std::vector<int16_t> v =textons[texton_id];

    std::vector<int16_t> sample_;
    sample_.assign(sample, sample + patch_square_size);

    for(int i = 0; i < patch_square_size; i++)
    {
        distance += pow(sample_[i] - v[i],2);

    }
    distance = sqrt(distance);

    return distance;
}

/*
 * Creates an histogram image
 */
cv::Mat Textons::drawHistogram(cv::Mat hist,int bins) {

    cv::Mat canvas;
    int line_width = 10;


    // Display each histogram in a canvas
    canvas = cv::Mat::ones(200, bins*line_width, CV_8UC3);

    int rows = canvas.rows;
    for (int j = 0; j < bins-1; j++)
    {
        cv::Scalar c = getColor(j);
        cv::line(canvas,cv::Point(j*line_width, rows),cv::Point(j*line_width, rows - (hist.at<int>(j) * rows/n_samples)),c, line_width, 8, 0);
    }    
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
    cv::Mat meanhists = cv::Mat::zeros(nHists, n_textons, cv::DataType<int>::type);
    cv::Mat amounts = cv::Mat::zeros(nHists, 1, cv::DataType<int>::type);

    //get the maximum disparity (range), in order to determine the histogram borders (of the 5 histograms)
    double maxgt;
    cv::minMaxIdx(groundtruth_buffer,NULL,&maxgt);
    float f  = maxgt/(nHists); //hist disparity size range

    for (int j=0;j<distribution_buf_size;j++) {
        cv::Mat hist;
        distribution_buffer.row(j).convertTo(hist,cv::DataType<int>::type,1,0); // distribution_buffer is in float, so copy like this needed
        int id = ceil((groundtruth_buffer.at<float>(j) / f))-1; // calc which hist this frame belongs to
        if (id<0){id=0;} // catch gt = 0;

        if (!(groundtruth_buffer.at<float>(j) > 6.199 && groundtruth_buffer.at<float>(j) < 6.2001)) { // exclude unused buffer
            amounts.at<int>(id)++; // keep track of amount of frames in a each hist range

            //add hist of current frame to the cumulator of the specific range
            cv::Mat tmp = meanhists.row(id);
            for (int k = 0; k<n_textons; k++) {
                tmp.at<int>(k) = tmp.at<int>(k) + hist.at<int>(k);
            }
        }

    }

    for (int i=0; i<nHists;i++) {
        //calc average for each hist
        cv::Mat tmp = meanhists.row(i);
        for (int j = 0; j<n_textons; j++) {
            if (amounts.at<int>(i) > 0 ) {
                tmp.at<int>(j) = (tmp.at<int>(j) / amounts.at<int>(i));
            } else {
                 tmp.at<int>(j) = 0;
            }

        }

        //create an image of it
       cv::Mat canvas =  drawHistogram(tmp,n_textons);
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
    cv::Mat canvas_color_i;
    cv::Mat canvas_encoded_i;
    cv::Mat canvas_color_gr;
    cv::Mat canvas_encoded_gr;
    cv::cvtColor(grayframe, canvas_color_gr, CV_GRAY2BGR);
    cv::cvtColor(grayframe, canvas_color_i, CV_GRAY2BGR);
    grayframe.copyTo(canvas_encoded_gr);
    grayframe.copyTo(canvas_encoded_i);

    // for all possible patches (non overlapping)
    for(int x=0;x<grayframe.cols-patch_size;x=x+patch_size){
        for(int y=0;y<grayframe.rows-patch_size;y=y+patch_size){

            //extract a random patch to a temporary vector
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
            std::vector<float> distances_gr(n_textons_gradient); // distances, TODO: float does not seem necessary -> int?
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
            cv::rectangle(canvas_color_gr,cv::Point(x, y),cv::Point(x+5, y+5),getColor(min_id_gr), line_width, 8, 0);
            cv::rectangle(canvas_color_i,cv::Point(x, y),cv::Point(x+5, y+5),getColor(min_id_i), line_width, 8, 0);

            //encode the texton
            //copy the closest patch into the image
            std::vector<int16_t> v_gr =textons[min_id_gr];
            std::vector<int16_t> v_i =textons[min_id_i];
            for (int i=0;i<patch_size;i++) {
                for (int j=0;j<patch_size;j++) {
                    // sample[i*patch_size+j] = grayframe.at<uint8_t>(y+j,x+i);
                    canvas_encoded_gr.at<uint8_t>(y+j,x+i) = v_gr[i*patch_size+j];
                    canvas_encoded_i.at<uint8_t>(y+j,x+i) = v_i[i*patch_size+j];
                }
            }


        }
    }


    cv::imshow("TextonColors gradient", canvas_color_gr);
    cv::imshow("TextonColors intensity", canvas_color_i);
    cv::imshow("TextonEncoded gradient", canvas_encoded_gr);
    cv::imshow("TextonEncoded intensity", canvas_encoded_i);

}

int countsincelearn =0;
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

    countsincelearn++;

    int learnborder =  (lastLearnedPosition+(distribution_buf_size-distribution_buf_pointer)) % distribution_buf_size; // make a sliding graph
    if ( countsincelearn > distribution_buf_size) {
        learnborder=0;
    }

    for (int j = filterwidth; j < distribution_buf_size ; j++)
    {

        int jj = (j+distribution_buf_pointer) % distribution_buf_size; // make a sliding graph
        nn = graph_buffer.at<float>(jj,0);
        gt = graph_buffer.at<float>(jj,1);

        if (j > learnborder ) { // change color according to training/test data
            color_nn= cv::Scalar(0,0,255); // red
        } else {

            color_nn= cv::Scalar(0,255,0); // green
        }


        //draw a small colored line above to indicate what the drone will do:
        if (nn < threshold_nn && gt > threshold_gt) {
            //false negative; drone should stop according to stereo, but didn't if textons were used
            //white
            cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(255,255,255), line_width, 8, 0);
        } else if (nn > threshold_nn && gt < threshold_gt) {
            //false positive; drone could have proceeded according to stereo, but stopped if textons were used
            //black
            cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,0,0), line_width, 8, 0);
        } else if (nn > threshold_nn && gt > threshold_gt) {
            //both stereo and textons agree, drone should stop
            //red
            cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,0,255), line_width, 8, 0);
        } else {
            //both stereo and textons agree, drone may proceed
            //green
            cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 10),cv::Scalar(0,255,0), line_width, 8, 0);
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
        //draw nn vision threshold:
        cv::line(graphFrame, cv::Point(j/stepX, rows- threshold_nn*scaleY), cv::Point((j+1)/stepX, rows -  threshold_nn*scaleY),color_invert, line_width, 8, 0);
        //draw gt vision threshold:
        cv::line(graphFrame, cv::Point(j/stepX, rows- threshold_gt*scaleY), cv::Point((j+1)/stepX, rows -  threshold_gt*scaleY),color_vert, line_width, 8, 0);

        prev_nn = nn;
        prev_gt = gt;
    }

    //draw text to notify user a key press was handled
    putText(graphFrame,msg,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,1,color_vert);

}

//calculates the histogram/distribution
void Textons::getTextonDistributionFromImage(cv::Mat grayframe, float avgdisp) {

    int middleid = floor((float)patch_size/2.0); // for gradient, asumes, texton size is odd!
    int16_t sample[patch_square_size];
    int16_t sample_dx[patch_square_size]; // gradient
    cv::Mat hist;
    hist = cv::Mat::zeros(1, n_textons, CV_32SC1);

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

        //get the distances to this patch to the textons...
        std::vector<float> distances_gr(n_textons_gradient); // distances, TODO: float does not seem necessary -> int?
        std::vector<float> distances_i(n_textons_intensity); // distances

        for(int j=0;j<n_textons;j++) {
            if (j < n_textons_intensity) {
                distances_i.at(j) = getEuclDistance(sample,j);                
            } else {
                distances_gr.at(j-n_textons_intensity) = getEuclDistance(sample_dx,j);
            }
        }

        //...and find out the closest texton:
       int min_id_i = std::min_element(distances_i.begin(), distances_i.end()) - distances_i.begin();
       int min_id_gr = std::min_element(distances_gr.begin(), distances_gr.end()) - distances_gr.begin() +n_textons_intensity;

        //increase the bin of closest texton
        hist.at<int>(min_id_gr)++;
        hist.at<int>(min_id_i)++;

    }

    //copy new data into learning buffer:
    cv::Mat M1 = distribution_buffer.row((distribution_buf_pointer+0) % distribution_buf_size) ;
    hist.convertTo(M1,cv::DataType<float>::type,1,0);

    //smooth avg, but exclude std filter from smoothing:
    if (avgdisp >0.1 ) {
        avgdisp_smoothed = gt_smoothed.addSample(avgdisp); // perform smoothing
    }
    else {
        avgdisp_smoothed = 0;
    }
//avgdisp_smoothed = avgdisp;


    groundtruth_buffer.at<float>(distribution_buf_pointer) = avgdisp_smoothed;
    distribution_buf_pointer = (distribution_buf_pointer+1) % distribution_buf_size;

     //std::cout << "hist" << distribution_buf_pointer << ": " << hist << std::endl;

    //run knn
   float nn = knn.find_nearest(M1,k,0,0,0,0); // if segfault here, clear xmls!

    //perform smoothing:
    nn = knn_smoothed.addSample(nn);

    std::cout << "knn.disp.: " << nn << "  |  truth: " << avgdisp_smoothed << std::endl;

    //save values for visualisation	in graph
    graph_buffer.at<float>((distribution_buf_pointer+0) % distribution_buf_size,0) = nn;
    graph_buffer.at<float>(distribution_buf_pointer,1) = avgdisp_smoothed; // groundtruth

#ifdef DRAWVIZS
    cv::Mat histimage = drawHistogram(hist,n_textons);
    drawMeanHists(histimage);
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

    if (!checkFileExist("../textons_gradient.dat")) {std::cerr << "Error: gradient textons not available\n";return 0;}
    if (!checkFileExist("../textons_intensity.dat")) {std::cerr << "Error: intensity textons not available\n";return 0;}

    std::ifstream input_gr("../textons_gradient.dat", std::ios::binary );
    std::ifstream input_i("../textons_intensity.dat", std::ios::binary );
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
    distribution_buffer = cv::Mat::zeros(distribution_buf_size, n_textons, cv::DataType<float>::type);
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
 * Reloads the learning buffer from file and retrains the learner with that data, effectively reseting the learner to then
 */
void Textons::reload() {
    initLearner(false);
    loadPreviousRegression();
}



