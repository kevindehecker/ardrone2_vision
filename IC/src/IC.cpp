#include <mutex>
#include <thread>

#include "defines.h"
#include "StereoAlg.h"
#include "smoother.h"
#include "socket.h"
#include "textons.h"
#ifdef FILECAM
#include "Filecam.h"
#endif
#ifdef DUOWEBCAM
#include "DuoWebcam.h"
#endif
#ifdef DELFLY_WIFI
#include "Delfly_wifi.h"
#endif
#ifdef DELFLY
#include "Delfly.h"
#endif
#ifdef ARDRONEFRONTCAM
#include "ARDroneCam.h"
#endif
#ifdef EXPORT
#include "exporter.h"
#endif

#include "stopwatch.h"

/*
#include "caffe/caffe.hpp"

using caffe::Blob;
using caffe::Caffe;
using caffe::Net;
using caffe::Layer;
using caffe::shared_ptr;
using caffe::Timer;
using caffe::vector;
*/

/***********Enums****************/
enum modus_t {none, stereo_only, textons_only, stereo_textons, stereo_textons_active};


/***********Variables****************/
char key = 0;
std::string msg;
cv::Mat resFrame;
cv::VideoWriter outputVideo;
cv::VideoWriter outputVideoResults;
stopwatch_c stopWatch;
modus_t mode;

#ifdef USE_TERMINAL_INPUT
std::thread thread_TerminalInput;
#endif

Socket tcp;
#ifdef EXPORT
Exporter exporter;
#endif
int countmsgclear=0;
int pauseVideo=0;
int result_input2Mode = VIZ_ROC;

#ifdef FILECAM
FileCam svcam;
#else
#ifdef DUOWEBCAM
DuoWebCam svcam;
#endif
#ifdef DELFLY_WIFI
Delfly_WiFi svcam;
#else
#ifdef DELFLY
DelFly svcam;
#endif
#endif
#endif

#ifdef ARDRONEFRONTCAM
ARDronecam ARfrontcam;
#endif

stereoAlg stereo;
Textons textonizer;

/*******Private prototypes*********/
void process_video();
#ifdef USE_TERMINAL_INPUT
void TerminalInputThread();
#endif
int main( int argc, char **argv);

void changeThresh_gt(int value);
void changeThresh_nn(int value);

void combineImage(cv::Mat resFrame, cv::Mat smallsourceimage, int x, int y,int width, int height, bool convertRGB);
void commOutThread();
void commInThread();
void handlekey();

/************ code ***********/

/*
 * Combines the stereo image and the learning graph to one big image
 */
void combineAllImages() {

#if defined(DELFLY)

	int im_width = resFrame.cols;
	int im_height = resFrame.rows;
	int sub_width = im_width/3;
	int sub_height = im_height/2;

	combineImage(resFrame,svcam.frameL_mat,0,0,sub_width,sub_height,true);


	//select which result image will be shown next to input image:
	if (result_input2Mode == VIZ_right_input_image) { //right input image
		combineImage(resFrame,svcam.frameR_mat,sub_width,0,sub_width,sub_height,true);
		if (!pauseVideo) {cv::applyColorMap(stereo.DisparityMat,stereo.DisparityMat,2);}
		combineImage(resFrame,stereo.DisparityMat,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_texton_intensity_color_encoding) { //texton intensity color encoding
		combineImage(resFrame,textonizer.frame_Itextoncolor,sub_width,0,sub_width,sub_height,false);
		combineImage(resFrame,textonizer.frame_currentHist,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_texton_intensity_texton_encoding) {// texton intensity texton encoding
		combineImage(resFrame,textonizer.frame_Itextontexton,sub_width,0,sub_width,sub_height,true);
		if (!pauseVideo) {cv::applyColorMap(stereo.DisparityMat,stereo.DisparityMat,2);}
		combineImage(resFrame,stereo.DisparityMat,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_texton_gradient_color_encoding) { //texton gradient color encoding
		combineImage(resFrame,textonizer.frame_Gtextoncolor,sub_width,0,sub_width,sub_height,false);
		combineImage(resFrame,textonizer.frame_currentHist,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_texton_gradient_texton_encoding) {// texton gradient,  press shift 6
		combineImage(resFrame,textonizer.frame_Gtextontexton,sub_width,0,sub_width,sub_height,false);
		if (!pauseVideo) {cv::applyColorMap(stereo.DisparityMat,stereo.DisparityMat,2);}
		combineImage(resFrame,stereo.DisparityMat,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_histogram) {// histogram       , but press shift 7!
		combineImage(resFrame,textonizer.frame_currentHist,sub_width,0,sub_width,sub_height,false);
		if (!pauseVideo) {cv::applyColorMap(stereo.DisparityMat,stereo.DisparityMat,2);}
		combineImage(resFrame,stereo.DisparityMat,sub_width*2,0,sub_width,sub_height,false);
	} else if (result_input2Mode == VIZ_ROC) {// ROC       , but press shift 8!
		combineImage(resFrame,textonizer.frame_ROC,sub_width,0,sub_width,sub_height,false);
		if (!pauseVideo) {cv::applyColorMap(stereo.DisparityMat,stereo.DisparityMat,2);}
		combineImage(resFrame,stereo.DisparityMat,sub_width*2,0,sub_width,sub_height,false);
	}

	combineImage(resFrame,textonizer.graphFrame,0,sub_height,im_width,sub_height,false);
#else
	combineImage(resFrame,stereo.DisparityMat,svcam.getImWidth(),0,stereo.DisparityMat.cols,stereo.DisparityMat.rows,false);
	combineImage(resFrame,svcam.frameL_mat,0,0,svcam.frameL_mat.cols/2,svcam.frameL_mat.rows/2,true);
	combineImage(resFrame,svcam.frameR_mat,svcam.getImWidth()/2,0,svcam.getImWidth()/2,svcam.getImHeight()/2,true);
	combineImage(resFrame,textonizer.graphFrame,0,svcam.getImHeight()/2,svcam.getImWidth()*1.5,svcam.getImHeight()/2,false);
#endif

}

/*
 * Puts one image into the big image
 */
void combineImage(cv::Mat resFrame, cv::Mat smallsourceimage, int x, int y,int width, int height, bool convertRGB) {


	if (smallsourceimage.cols == 0 || smallsourceimage.rows == 0) {
		smallsourceimage = cv::Mat(height,width, CV_8UC3);
		convertRGB=false;
	}


	cv::Point p1(x, y);
	cv::Point p2(x+width, y+height);
	cv::Point size(width, height);

	cv::Mat rgbsmallsourceimage(smallsourceimage.cols,smallsourceimage.rows,CV_8UC3);
	if (convertRGB) {
		cvtColor(smallsourceimage,rgbsmallsourceimage,CV_GRAY2RGB,0);
	} else {
		rgbsmallsourceimage = smallsourceimage;
	}


	cv::Mat roi = cv::Mat(resFrame, cv::Rect(p1, p2));

	if (rgbsmallsourceimage.size().width != width || rgbsmallsourceimage.size().height != height) {
		cv::resize(rgbsmallsourceimage,roi,size);
	} else {
		rgbsmallsourceimage.copyTo(roi);
	}
}


/*
 * process_video: retrieves frames from camera, does the processing and handles the IO.
 * Skips frames if processing takes too long.
 * the camera.
 */
void process_video() {
	stopWatch.Start();
	int frames = 0;
	//main while loop:
	while (key != 27 && svcam.cams_are_running) // ESC
	{
		if (!pauseVideo) {
			svcam.waitForImage();  //synchronized grab stereo frame:
		}

		bool stereoOK=true;

		if ((mode==stereo_only || mode==stereo_textons || mode==stereo_textons_active) && !pauseVideo) {
			//stereo is turned on
			stereoOK = stereo.calcDisparityMap(svcam.frameL_mat,svcam.frameR_mat); // calc the stereo groundtruth
		}
		if ((mode==stereo_textons_active || mode==stereo_textons && stereoOK) || mode==textons_only) {
			textonizer.getTextonDistributionFromImage(svcam.frameL_mat,stereo.avgDisparity,mode==stereo_textons_active,pauseVideo);  //perform the texton stuff
			tcp.commdata_nn = textonizer.getLast_nn();
		}
		if (mode==stereo_only || mode==stereo_textons || stereo_textons_active) {
			tcp.commdata_gt = stereo.avgDisparity;
			tcp.commdata_gt_stdev = stereo.stddevDisparity;
		}

		if (mode==stereo_textons || mode==stereo_textons_active) {
			textonizer.setAutoThreshold();
		}

#ifdef VIDEORAW
		//combine stereo pair if needed
		if (mode==none  || mode==textons_only) {
			stereo.combineImage(svcam.frameL_mat,svcam.frameR_mat);
		} else {
#ifndef LONGSEC
			stereo.combineImage(svcam.frameL_mat,svcam.frameR_mat);
#endif // LONGSEC

		}
		outputVideo.write(stereo.frameC_mat);

#endif //VIDEORAW

#if defined(HASSCREEN) || defined(VIDEORESULTS)
		textonizer.drawGraph(msg);
		combineAllImages();
#ifdef HASSCREEN
		cv::imshow("Results", resFrame);
#endif
#ifdef VIDEORESULTS
		outputVideoResults.write(resFrame);
#endif
#endif

		handlekey();

		if (!pauseVideo) {
			frames++;
		}
		//		if (frames == 1500) {
		//			textonizer.retrainAll();
		//		}

		//		if ((frames % 100) == 99) {
		//			textonizer.retrainAll();
		//			//textonizer.saveRegression();
		//			std::cout << "mod: " << frames % 100 << "\n|" ;
		//		}



		float time = stopWatch.Read()/1000;
		tcp.commdata_fps = frames /(time);
		std::cout << "#" << frames << ", fps: " << tcp.commdata_fps << ", GT: " << tcp.commdata_gt << std::endl;

#ifdef USE_SOCKET
		tcp.Unlock();
#endif
#ifdef EXPORT
		exporter.write(tcp.commdata_gt,tcp.commdata_gt_stdev,tcp.commdata_nn);
		exporter.saveStereoPair(svcam.frameL_mat,svcam.frameR_mat,stereo.DisparityMat);
#endif

	} // main while loop

#ifdef HASSCREEN
	cv::destroyAllWindows();
#endif
}

void handlekey() {

#ifdef HASSCREEN
	key = cv::waitKey(10);
	key = key & 0xff;
	if (key == 27) {  //esc
		svcam.cams_are_running = false;
		return; // don't clear key, just exit
	}
#endif

	switch(key) {
	case 10: // [enter]: perform learning
		textonizer.retrainAll();
		msg="Learn";
		break;
	case 105:// [i]: idem (perform learning)
		textonizer.retrainAll();
		msg="Learn";
		break;
	case 115: // [s]: save
		textonizer.saveRegression();
		msg="Save";
		break;
	case 99: // [c]: clear
		textonizer.initLearner(true);
		msg="Clear";
		break;
	case 108: // [l]: reload
		textonizer.reload();
		msg="Reload";
		break;
	case 48: // [0]: switch stereo and texton calculation off
		mode=none;
		break;
	case 49: // [1]: switch stereo mode off, textons on
		mode=textons_only;
		break;
	case 50: // [2]: switch stereo mode on, textons off
		mode=stereo_only;
		break;
	case 51: // [3]: switch both stereo and textons calucation on
		mode=stereo_textons;
		break;
	case 52: // [4]: switch both stereo and textons calucation on, use active learning
		mode=stereo_textons_active;
		break;
		//    case 92: // [\]: save stereo image to bmp
		//	  exporter.saveStereoPair();
		//	  break;
		//    case 114: // [r]: reset stopwatch
		//	  frames=0;stopWatch.Restart();msg="Reset";
		//	  break;
	case 97: // [a]: increase threshold nn
		changeThresh_nn(1);
		break;
	case 122: // [z]: decrease threshold nn
		changeThresh_nn(-1);
		break;
	case 65: // [A]: increase threshold gt
		changeThresh_gt(1);
		break;
	case 90: // [Z]: decrease threshold gt
		changeThresh_gt(-1);
		break;
	case 33: // [!]: show right input image
		result_input2Mode=VIZ_right_input_image;
		break;
	case 64: // [@]: show intensity texton color encoded left input image
		result_input2Mode=VIZ_texton_intensity_color_encoding;
		break;
	case 35: // [#]: show intensity texton texton encoded left input image
		result_input2Mode=VIZ_texton_intensity_texton_encoding;
		break;
	case 36: // [$]: show gradient texton color encoded left input image
		result_input2Mode=VIZ_texton_gradient_color_encoding;
		break;
	case 37: // [%]: show gradient texton texton encoded left input image
		result_input2Mode=VIZ_texton_gradient_texton_encoding;
		break;
	case 38: // [&]: show histogram
		result_input2Mode=VIZ_histogram;
		break;
	case 42: // [*]: show ROC curve
		result_input2Mode=VIZ_ROC;;
		break;
#ifdef HASSCREEN
	case '>': // fast forward filecam
		svcam.fastforward=1;
		svcam.rewind=0;
		break;
#ifndef FILESTEREO //currently not supported by FILESTEREO
	case '<': // rewind filecam
		svcam.rewind=1;
		svcam.fastforward=0;
		break;
#endif
	case ' ': // pause
		svcam.rewind=0;
		svcam.fastforward=0;
		if (pauseVideo) {
			pauseVideo=0;
		}else {
			pauseVideo=1;
		}

		//		while (pauseVideo==1) {
		//			usleep(1000);
		//			key = cv::waitKey();
		//			if (key == 32) { // next frame
		//				key=0;
		//				break;
		//			}else if (key != 0) {
		//				pauseVideo=0;
		//				key=0;
		//			}
		//			break;
		//		}
#endif
	} // end switch key

	//display a message on the image, informing which key has been pressed.
	//if no key was pressed for some time, display the current mode
	countmsgclear ++;
	if (countmsgclear>10) {
		countmsgclear=0;

		switch ( mode) {
		case none:
			msg= "none";
			break;
		case textons_only:
			msg= "textons";
			break;
		case stereo_only:
			msg= "stereo";
			break;
		case stereo_textons:
			msg= "textons+stereo";
			break;
		case stereo_textons_active:
			msg= "stereo+textons active learning";
			break;
		}
	}


#ifndef VIDEORESULTS
	if (key!=0) {
		std::cout << "Terminal: "  << msg << std::endl;

	}
#endif
	key=0;

}

void changeThresh_nn(int value) {
	textonizer.threshold_nn = textonizer.threshold_nn + value;
#ifdef _PC
	std::stringstream s;
	s << "nn thresh: " << (textonizer.threshold_nn);
	msg=s.str();
#endif
}
void changeThresh_gt(int value) {
	textonizer.threshold_gt = textonizer.threshold_gt + value;
#ifdef _PC
	std::stringstream s;
	s << "gt thresh: " << (textonizer.threshold_gt);
	msg=s.str();
#endif
}

void TerminalInputThread() {
#ifdef USE_TERMINAL_INPUT
	usleep(1000000); // let the qt debug output pass through. Hmm doesnt work.
	while(svcam.cams_are_running) {
		std::cin >> key;
		if (key==120 || key==113) {
			key=27; // translate x to esc
			svcam.cams_are_running = false;
			std::cout << "Exiting\n";
		}
	}
#endif
}

int init(int argc, char **argv) {

	/*****init the camera*****/
#ifdef DUOWEBCAM
	int cam_left_id;
	int cam_right_id;

	if (argc != 3) {
		std::cout << "Usage: DisplayImage cam_left, cam_right\n";
		std::cout << "Now using default values.\n";

		cam_left_id=0;
		cam_right_id=2;

		//return 1;
	} else {

		cam_left_id= atoi(argv[1]);
		cam_right_id = atoi(argv[2]);
	}
	if (!svcam.init(cam_left_id,cam_right_id)) {return 1;}
#else
	if (!svcam.init()) {return 1;}
#endif

	/*****init the visual bag of words texton methode*****/
	std::cout << "Initialising textonizer\n";
	if (!textonizer.init(&result_input2Mode)) {return 1;}

	/*****Start capturing images*****/
	std::cout << "Start svcam\n";
	svcam.start();
#ifdef ARDRONEFRONTCAM
	std::cout << "Starting ARDRone cam\n";
	ARfrontcam.start();
#endif

	/***init the stereo vision (groundtruth) algorithm ****/
	std::cout << "Initialising stereo algorithm\n";
	stereo.init(svcam.getImWidth(), svcam.getImHeight()); //sv initialisation can only happen after cam start, because it needs the im dims

	/*****init the (G)UI*****/
#ifdef HASSCREEN
	cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
	//cv::resizeWindow("Results", 1100, 550); //makes it slower
#endif
#ifdef USE_TERMINAL_INPUT
	thread_TerminalInput = std::thread(TerminalInputThread);
#endif
#ifdef USE_SOCKET
	tcp.Init(&key, &(svcam.cams_are_running));
#endif
#ifdef EXPORT
	exporter.init();
#endif
#if defined(HASSCREEN) || defined(VIDEORESULTS)
#ifdef DUOWEBCAM
	resFrame = cv::Mat::zeros(svcam.getImHeight(), svcam.getImWidth()*1.5,CV_8UC3);
#else
	resFrame = cv::Mat::zeros(500, 1100,CV_8UC3);
#endif
#endif

	/*****init the video writer*****/
#ifdef VIDEORAW

	cv::Size size(svcam.getImWidth()*2,svcam.getImHeight()); // for dsp encoding, ensure multiples of 16
#ifdef _PC
	//outputVideo.open("appsrc ! ffmpegcolorspace ! ffenc_mpeg4 ! avimux ! filesink location=video_wifi.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
	outputVideo.open("video_wifi.avi",CV_FOURCC('M','P','E','G'),VIDEOFPS,size,false);
#else
	//outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc mode=1 ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.2 port=5000",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
	outputVideo.open("appsrc ! ffmpegcolorspace ! dspmp4venc mode=0 ! avimux ! filesink location=video_dsp.avi",CV_FOURCC('H','O','E','R'),VIDEOFPS,size,false);
#endif

	if (!outputVideo.isOpened())
	{
		std::cout << "!!! Output stereo video could not be opened" << std::endl;
		return 1;
	}
#endif
#ifdef VIDEORESULTS
	cv::Size sizeRes(resFrame.cols,resFrame.rows);
#ifdef _PC
	outputVideoResults.open("videoResults.avi",CV_FOURCC('F','M','P','4'),VIDEOFPS,sizeRes,true);
#else
	outputVideoResults.open("appsrc ! ffmpegcolorspace ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.2 port=5000",CV_FOURCC('H','O','E','R'),VIDEOFPS,sizeRes,true);
#endif

	if (!outputVideoResults.isOpened())
	{
		std::cout << "!!! Output result video could not be opened" << std::endl;
		return 1;
	}
#endif

	mode = RUNMODE;
	result_input2Mode = VIZ_right_input_image;
	msg="";


	//    caffe::Caffe::set_mode(caffe::Caffe::CPU);
	//
	//    caffe::SolverParameter solver_param;
	//    caffe::ReadProtoFromTextFileOrDie("/home/goggles/caffe/examples/cifar10/cifar10_quick_solver.prototxt", &solver_param);

	return 0;
}

void close() {

	/*****Close everything down*****/
	svcam.close();
#ifdef ARDRONEFRONTCAM
	ARfrontcam.close();
#endif

#ifdef USE_SOCKET
	tcp.Close();
#endif
#ifdef USE_TERMINAL_INPUT
	thread_TerminalInput.detach();	//cin is blocking
#endif
#ifdef EXPORT
	exporter.close();
#endif

}

int main( int argc, char **argv )
{
	if (init(argc,argv)) {return 1;}

	/* clear learning buffer instead of using old stuff */
	textonizer.initLearner(true);

	process_video();
	close();

	/* auto save at the end */
	textonizer.retrainAll();
	textonizer.saveRegression();

	return 0;
}


