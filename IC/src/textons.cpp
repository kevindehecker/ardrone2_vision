#include "textons.h"
#include <opencv2/highgui/highgui.hpp>

void Textons::getCommData(float* s){
        s[0] = 66.0; // header
        s[1] = graph_buffer.at<float>(distribtuion_buf_pointer,0); // nn
        s[2] = graph_buffer.at<float>(distribtuion_buf_pointer,1); // stereo
}

bool Textons::init () {
    if (!initTextons()) {return false;}
    if (!initLearner(false)) {return false;}
    loadPreviousRegression();
    return true;
}

/*
*Calculates the Eucladian distance between a patch and a texton
*/
double Textons::getEuclDistance(unsigned char sample[], int texton_id) {    
    double distance =0;

    std::vector<unsigned char> v =textons[texton_id];

    for(int i = 0; i < patch_square_size; i++)
    {
        distance += pow(sample[i] - v[i],2);

    }
    distance = sqrt(distance);

    return distance;
}

void Textons::drawHistogram(cv::Mat hist,int bins) {

    cv::Mat canvas;
    cv::Scalar color= cv::Scalar(255,0,0); // blue
    int line_width = 10;

    // Display each histogram in a canvas
    canvas = cv::Mat::ones(200, bins*line_width, CV_8UC3);

    int rows = canvas.rows;
    for (int j = 0; j < bins-1; j++)
    {
        cv::line(
                    canvas,
                    cv::Point(j*line_width, rows),
                    cv::Point(j*line_width, rows - (hist.at<int>(j) * rows/n_samples)),
                    color, line_width, 8, 0
                    );
    }
    cv::imshow("Histogram", canvas);
}

void Textons::drawGraph(std::string msg) {

    cv::Scalar color_gt= cv::Scalar(255,0,0); // blue
    cv::Scalar color_nn= cv::Scalar(0,0,255); // red
    cv::Scalar color_vert= cv::Scalar(0,180,255); // orange
    int line_width = 1;

    int stepX = distribution_buf_size/700;
    int rows = 300;

    graphFrame = cv::Mat::zeros(rows, (distribution_buf_size/stepX), CV_8UC3);

    double max;
    cv::minMaxIdx(graph_buffer,NULL,&max,NULL,NULL);
    float scaleY = rows/max;

    float nn,gt;
    float prev_nn = 0, prev_gt = 0;


    for (int j = filterwidth; j < distribution_buf_size ; j++)
    {

        int jj = (j+distribtuion_buf_pointer) % distribution_buf_size; // make a sliding graph
        nn = graph_buffer.at<float>(jj,0)*scaleY;
        gt = graph_buffer.at<float>(jj,1)*scaleY;
        //if ((lastLearnedPosition % distribution_buf_size) < jj && j > jj) {
        if (lastLearnedPosition < jj && j > jj) {
            //draw a small colored line above none learned part:
            cv::line(graphFrame,cv::Point(j/stepX, 0),cv::Point(j/stepX, 3),color_vert, line_width, 8, 0);
            color_nn= cv::Scalar(0,0,255); // red
        } else {

            color_nn= cv::Scalar(0,255,0); // green
        }

        if (j==filterwidth) { // fixes discontinuty at the start of the graph
            prev_nn = nn;
            prev_gt = gt;
        }

        //draw knn result:
        cv::line(graphFrame, cv::Point(j/stepX, rows- prev_nn), cv::Point((j+1)/stepX, rows -  nn), color_nn, line_width, 8, 0);
        //draw stereo vision groundtruth:
        cv::line(graphFrame, cv::Point(j/stepX, rows- prev_gt), cv::Point((j+1)/stepX, rows -  gt),color_gt, line_width, 8, 0);

        prev_nn = nn;
        prev_gt = gt;
    }

    //draw text to notify user a key press was handled
    putText(graphFrame,msg,cv::Point(0, 30),cv::FONT_HERSHEY_SIMPLEX,1,color_vert);
}

//calculates the histogram/distribution
void Textons::getTextonDistributionFromImage(cv::Mat grayframe, float avgdisp) {
    //printf("im height: %d\n",(*grayframe).height);

    unsigned char sample[patch_square_size];
    std::vector<int> distribution(n_textons);
    cv::Mat hist;
    hist = cv::Mat::zeros(1, n_textons, CV_32SC1);



    for(int n=0;n<n_samples;n++){

        int x = rand() % (grayframe.cols-patch_size);
        int y = rand() % (grayframe.rows-patch_size);
        //printf("x: %d, y: %d\n",x,y);

        //extract a random patch to a temporary vector
        for (int i=0;i<patch_size;i++) {
            for (int j=0;j<patch_size;j++) {
                sample[i*patch_size+j] = grayframe.at<uint8_t>(y+j,x+i);
            }
        }

//        if (n==0) { // visualise a patch
//            cv::Mat test((int)patch_size,(int)patch_size,CV_8UC1, *sample);
//            imshow("patch", test );
//        }

        //get the distances to this patch to the textons...
        std::vector<double> distances(n_textons); // distances
        for(int j=0;j<n_textons;j++) {
            distances.at(j) = getEuclDistance(sample,j);
        }
        //...and find out the closest texton:
        int min_id = std::min_element(distances.begin(), distances.end()) - distances.begin(); //jàààhoor

        //increase the bin of closest texton
        distribution.at(min_id)++;
        hist.at<int>(min_id)++; // for visualisation

    }

    //copy new data into learning buffer:
    cv::Mat M1 = distribtuion_buffer.row(distribtuion_buf_pointer ) ;
    hist.convertTo(M1,cv::DataType<float>::type,1,0);
    avgdisp = gt_smoothed.addSample(avgdisp); // perform smoothing
    groundtruth_buffer.at<float>(distribtuion_buf_pointer) = avgdisp;
    distribtuion_buf_pointer = (distribtuion_buf_pointer+1) % distribution_buf_size;

    // std::cout << "hist" << distribtuion_buf_pointer << ": " << hist << std::endl;

    //run knn
    float nn = knn.find_nearest(M1,5,0,0,0,0);

    //perform smoothing:
    nn = knn_smoothed.addSample(nn);

    std::cout << "knn.disp.: " << nn << "  |  truth: " << avgdisp << std::endl;

    //save values for visualisation	in graph
    graph_buffer.at<float>(distribtuion_buf_pointer,0) = nn;
    graph_buffer.at<float>(distribtuion_buf_pointer,1) = avgdisp; // groundtruth

#ifdef DRAWHIST
    drawHistogram(hist,n_textons);
#endif
}

int Textons::getLast_gt() {
    int gt = graph_buffer.at<float>(distribtuion_buf_pointer,1);
    return gt;
}
int Textons::getLast_nn() {
    int nn = graph_buffer.at<float>(distribtuion_buf_pointer,0);
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

int Textons::initTextons() {
    std::cout << "Opening textons file\n";

    if (!checkFileExist("../textons.dat")) {std::cerr << "Error: textons not available\n";return 0;}

    std::ifstream input("../textons.dat", std::ios::binary );
    // copies all data into buffer
    std::vector<char> buffer(( std::istreambuf_iterator<char>(input)),(std::istreambuf_iterator<char>()));
    n_textons = buffer[0];
    patch_size = buffer[1];
    patch_square_size = patch_size*patch_size;
    textons.resize(n_textons);
    printf("#textons: %d, Patch size: %d, #samples: %d\n",n_textons,patch_size,n_samples );

    int counter =2; // skip first two bytes, as they contain other info
    for (int i = 0;i<n_textons;i++) {
        std::vector<unsigned char> v(patch_square_size);
        //printf("texton[%d]:", i);
        for (int j=0;j<patch_square_size;j++) {
            v[j] = buffer[counter++];
            //	if (j>0) {printf(", %d", v[j]);} else {printf(" %d", v[j]);}
        }
        textons[i] = v;
        input.close();
        //printf("\n");
    }
    return 1;
}

bool Textons::initLearner(bool nulltrain) {
    srand (time(NULL));
    distribtuion_buffer = cv::Mat::zeros(distribution_buf_size, n_textons, cv::DataType<float>::type);
    groundtruth_buffer = cv::Mat::zeros(distribution_buf_size, 1, cv::DataType<float>::type);
    graph_buffer = cv::Mat::zeros(distribution_buf_size, 2, cv::DataType<float>::type);
    groundtruth_buffer = groundtruth_buffer +6.2; //for testing...
    distribtuion_buf_pointer = 0;
    knn_smoothed.init(filterwidth);
    gt_smoothed.init(filterwidth);
    if (nulltrain) {
        return knn.train(distribtuion_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );
    } else {
        return true;
    }
}

void Textons::retrainAll() {
    std::cout << "Training knn regression:\n";
    knn.train(distribtuion_buffer, groundtruth_buffer, cv::Mat(), true, 32, false );
    lastLearnedPosition = (distribtuion_buf_pointer +1 )% distribution_buf_size;

#ifdef _PC
    std::cout << "Initialising smoother:\n";
    for (int i=0;i<distribution_buf_size; i++) {

        int jj = (i+distribtuion_buf_pointer) % distribution_buf_size;
        cv::Mat M1 = distribtuion_buffer.row(jj); // prevent smoothing filter discontinuity
        graph_buffer.at<float>(jj,0) = knn_smoothed.addSample(knn.find_nearest(M1,5,0,0,0,0));
        gt_smoothed.addSample(graph_buffer.at<float>(i,1)); // prepare the gt filter, not really necessary
    }
#endif
}

int Textons::loadPreviousRegression() {
    try {
        std::cout << "Opening memory files\n";


        if (!checkFileExist("../distribtuion_buffer.xml")) {std::cout << "Warning: no previous data found\n";return 0;}
        cv::FileStorage dist_fs("../distribtuion_buffer.xml", cv::FileStorage::READ);
        dist_fs["distribtuion_buffer"] >>distribtuion_buffer;
        cv::FileStorage ground_fs("../groundtruth_buffer.xml", cv::FileStorage::READ);
        ground_fs["groundtruth_buffer"] >> groundtruth_buffer;
        cv::FileStorage graph_fs("../graph_buffer.xml", cv::FileStorage::READ);
        graph_fs["graph_buffer"] >> graph_buffer;

        std::cout << "Training...\n";
        //draw the training set results:
        retrainAll();

    } catch (int e) {
        std::cout << "No previous regression memory could be openend.\n";
    }
}

void Textons::saveRegression() {
    cv::FileStorage dist_fs("../distribtuion_buffer.xml", cv::FileStorage::WRITE);
    dist_fs << "distribtuion_buffer" << distribtuion_buffer;

    cv::FileStorage ground_fs("../groundtruth_buffer.xml", cv::FileStorage::WRITE);
    ground_fs << "groundtruth_buffer" << groundtruth_buffer;

    cv::FileStorage graph_fs("../graph_buffer.xml", cv::FileStorage::WRITE);
    graph_fs << "graph_buffer" << graph_buffer;

    retrainAll();
}

void Textons::reload() {
    initLearner(false);
    loadPreviousRegression();
}
