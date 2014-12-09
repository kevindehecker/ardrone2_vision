#include "Delfly.h"
#include <iostream>
#include "rs232.h"

#include "stopwatch.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv.h>

bool DelFly::init () {
    int res = RS232_OpenComport(3000000);  //3000000 is the maximum of the usb bus ftdi serial port
	if (res != 0) {
		std::cerr << "Error opening COM port. Is the camera connected?\n";
        return false;
    }


//    const char* calibDir = "../calib10_2b/";

//    char mx1fn[256];
//    strcpy(mx1fn,calibDir);
//    strcat(mx1fn,"mx1.xml");
//    char mx2fn[256];
//    strcpy(mx2fn,calibDir);
//    strcat(mx2fn,"mx2.xml");
//    char my1fn[256];
//    strcpy(my1fn,calibDir);
//    strcat(my1fn,"my1.xml");
//    char my2fn[256];
//    strcpy(my2fn,calibDir);
//    strcat(my2fn,"my2.xml");

//    _mx1 =	(CvMat *)cvLoad(mx1fn,NULL,NULL,NULL);
//    _mx2 =	(CvMat *)cvLoad(mx2fn,NULL,NULL,NULL);
//    _my1 =	(CvMat *)cvLoad(my1fn,NULL,NULL,NULL);
//    _my2 =	(CvMat *)cvLoad(my2fn,NULL,NULL,NULL);

//    if (_mx1 != 0 && _mx2 != 0 && _my1 != 0 && _my2 != 0) {
//        std::cout << "finished loading\n";
//        return true;
//    }
//    else {
//        fprintf(stderr,"\nError loading calib matrices!");
//        return false;
//    }

    return true;

}

void DelFly::start () {
    cams_are_running=true;
    thread_cam = std::thread(&DelFly::workerThread,this);
    copyNewImage = false;
    waitForImage(); // make sure to receive an image before proceeding, to have the width/height variables set
    waitForImage();
    std::cout << "Delfly camera link started!\n";
}

void DelFly::waitForImage() {	
    while (!copyNewImage ) {
        usleep(1000);
    }
    g_lockWaitForImage.lock();
    frameL_2ndbuf.copyTo(frameL_mat);
    frameR_2ndbuf.copyTo(frameR_mat);    
    copyNewImage = false; // hmm ugly, but misisng resetevent in linux...
    g_lockWaitForImage.unlock();
}

void DelFly::close () {
    cams_are_running = false;
    copyNewImage=true;
    g_lockWaitForImage.unlock();
	thread_cam.join();
    RS232_CloseComport();
}


#if !(defined(DELFLY_COLORMODE) && defined(DELFLY_DISPMODE))

void DelFly::workerThread() {
    unsigned char buffer[25348];
    int bufsize = 25348; //=128*96*2 + 4 (start of frame header) + 96*4 *2 ( 96 * SOL en EOL)
    int i=0, sv_width=256;
    int current_line = 0;
    im_width = sv_width/2;
    im_height = 96;

    //comport resetter
    stopwatch_c stopWatch_comportAlive;
    stopWatch_comportAlive.Start();

    int tmpjets ;

    /* perfect red detector
    const uint8_t min_U = 123; // u=pink, v=yellow, u+v = blue
    const uint8_t min_V = 140;
    const uint8_t max_U = 138;
    const uint8_t max_V = 170;
    */

    //perfect blue
//    const uint8_t min_U = 130; // u=pink, v=yellow, u+v = blue // 131
//    const uint8_t min_V = 100; // 0
//    const uint8_t max_U = 200; // 255
//    const uint8_t max_V = 140; // 125

//    const uint8_t min_Y = 0; // 110
//    const uint8_t max_Y = 200; // 160




#ifdef DELFLY_COLORMODE
    cv::Mat frameYUYV = cv::Mat::zeros(im_height,im_width, CV_8UC2);
#else
    cv::Mat frameL = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(im_height,im_width, CV_8UC1);
#endif

    unsigned char prevbuf3,prevbuf2,prevbuf1;
    int tmpsize = 0;

    while (cams_are_running)  {

        //try to receive (the rest of) a full image into the buffer:
        while (bufsize> tmpsize && cams_are_running) {
            int res = RS232_PollComport(buffer+tmpsize,bufsize-tmpsize);
            //std::cout << "bytecounter: " << res << std::endl;
            if (res<0) {
                cams_are_running = false;
                std::cerr << "Serial port read error, is the camera connected?\n";
                g_lockWaitForImage.unlock();
                return;
            }
            tmpsize+=res;
            if (res<4095) {
                //in order to prevent 100% cpu usage, sleep when the comport buffer wasn full.
                usleep(1000);
            }

            //In order to fix an issue with the drone comport driver; behold for this horrible hack:
            //If no bytes were received for more then 1 second, reinitialise the comport.
            //On the ARDrone2 this happens after about ~0-30 frames, except for if the program was started after a clean boot.
            //Also, if the usb bus is used for other things, this may occur. (the camera maxes out the bandwidth)
            //On the PC, this does not happen at all.
            if (res>0) {
                stopWatch_comportAlive.Restart();
            } else if (stopWatch_comportAlive.Read() > 1000) {
                std::cerr << "Restart comport!\n";
                RS232_CloseComport();
                init();
            }
        }

        //loop through the buffer in search of hearders, and handle them accordingly
        for (i=0;i < bufsize-sv_width-4;i++) {

            if (prevbuf3 == 255 && prevbuf2 == 0 && prevbuf1==HEADERBYTE) { //detect a start of frame header
               // uint8_t headerbyte = prevbuf1;

                if ((buffer[i] == 0x80 || buffer[i] == 0xc7) && (buffer[i+4+sv_width] == 0x9d || buffer[i+4+sv_width] == 0xda)) { //Start of Line && End of Line (not checking for header of EOL)

                    //line detected, copy it to the image buffer
                    if (current_line < im_height) { // sometimes bytes get lost, compensate by dropping lines.
                        for (int j = 0; j< im_width;j++) {
                            int jj = j * 2 + i+1;


//                            if (headerbyte == HEADERCOLOR) {
//                                frameYUYV.at<uint8_t>(current_line,j*2+1) = buffer[jj];
//                                frameYUYV.at<uint8_t>(current_line,j*2) = buffer[jj+1];
//                            } else if (headerbyte==HEADERSTEREO) {
//                                frameL.at<uint8_t>(current_line,j) = buffer[jj++];
//                                frameR.at<uint8_t>(current_line,j) = buffer[jj];
//                            } else if (headerbyte==HEADERDISPARITY){

//                            }



#ifdef DELFLY_COLORMODE
                            frameYUYV.at<uint8_t>(current_line,j*2+1) = buffer[jj];
                            frameYUYV.at<uint8_t>(current_line,j*2) = buffer[jj+1];
#else
                            int caliblineR = current_line +3; // for sv cam 10, this seems to work ok
                            int caliblineL = current_line -2;

                            if (caliblineL >= 0 && caliblineL < im_height )
                                frameL.at<uint8_t>(caliblineL,j) = buffer[jj++];

                            if (caliblineR >= 0 && caliblineR < im_height )
                                frameR.at<uint8_t>(caliblineR,j) = buffer[jj];
#endif
                        }

                        i+=sv_width+4;
                        current_line++ ;

                    } else {std::cout << "End Of Frame err \n" ;}

                } else if (buffer[i] == 0xec || buffer[i] == 0xab) { //end of frame

                    // i should converge to 2* sv_width, since that's the for loop boundry
                    // at the start of the program, of just after a stream corruption, i will vary
                    //std::cout << "current_line: " << current_line << ", at " << i << "\n";





                    if (true ) { // if the main thread asks for a new image
#ifdef DELFLY_COLORMODE

//                        uint32_t mean_u = 0;
//                        uint32_t mean_v = 0;
//                        uint32_t mean_y = 0;

//                        uint32_t min_u = 0;
//                        uint32_t min_v = 0;
//                        uint32_t min_y = 0;
//                        uint32_t max_u = 0;
//                        uint32_t max_v = 0;
//                        uint32_t max_y = 0;
//                        min_u--;
//                        min_v--;


//                        uint32_t cocnt = 0;
//                        for (int jjj = 0; jjj < 2*im_width-4; jjj+=4) {
//                            for (int iii = 0; iii < im_height; iii++) {
//                                uint8_t u = frameYUYV.at<uint8_t>(iii,jjj+1);
//                                uint8_t v = frameYUYV.at<uint8_t>(iii,jjj+3);
//                                uint8_t y1 = frameYUYV.at<uint8_t>(iii,jjj+0);
//                                //uint8_t y2 = frameYUYV.at<uint8_t>(iii,jjj+2);


//                                if (u<min_u) {
//                                    min_u = u;
//                                }
//                                if (v<min_v) {
//                                    min_v = v;
//                                }
//                                if (y1<min_y) {
//                                    min_y = y1;
//                                }

//                                if (u>max_u) {
//                                    max_u = u;
//                                }
//                                if (v>max_v) {
//                                    max_v = v;
//                                }
//                                if (y1>max_y) {
//                                    max_y = y1;
//                                }

//                                mean_u+=u;
//                                mean_v+=v;
//                                mean_y+=y1;

                                if (u>max_u) {
                                    max_u = u;
                                }
                                if (v>max_v) {
                                    max_v = v;
                                }
                                if (y1>max_y) {
                                    max_y = y1;
                                }

                                mean_u+=u;
                                mean_v+=v;
                                mean_y+=y1;

//                                // Color Check:
//                                if ( (u >= min_U) && (u <= max_U)
//                                     && (v >= min_V) && (v <= max_V) && (y1 >= min_Y) && (y1 <= max_Y)) {
//                                    cocnt ++;


//                                    //make it blue:
//                                    frameYUYV.at<uint8_t>(iii,jjj+1) = 0; // U
//                                   // frameYUYV.at<uint8_t>(iii,jjj+0);     // Y1
//                                    frameYUYV.at<uint8_t>(iii,jjj+3) = 0;  // V
//                                    //frameYUYV.at<uint8_t>(iii,jjj+2);     // Y1
//                                 } else if ((u >= min_U) && (u <= max_U)) {
//                                    //make it pink:
//                                    frameYUYV.at<uint8_t>(iii,jjj+1) = 255; // U
//                                    frameYUYV.at<uint8_t>(iii,jjj+0) = 50;  // Y1
//                                    frameYUYV.at<uint8_t>(iii,jjj+3) = 255; // V
//                                    frameYUYV.at<uint8_t>(iii,jjj+2) = 50;  // Y2
//                                } else if((v >= min_V) && (v <= max_V)) {
//                                    //make it ?:
//                                    frameYUYV.at<uint8_t>(iii,jjj+1) = 0;      // U
//                                    frameYUYV.at<uint8_t>(iii,jjj+0) = 255;    // Y1
//                                    frameYUYV.at<uint8_t>(iii,jjj+3) = 255;    // V
//                                    frameYUYV.at<uint8_t>(iii,jjj+2) = 255;    // Y2
//                                }
                            }
                        }

////                                } else if((y1 >= min_Y) && (y1 <= max_Y)) {
////                                    //make it ?:
////                                    frameYUYV.at<uint8_t>(iii,jjj+1) = 0;      // U
////                                    frameYUYV.at<uint8_t>(iii,jjj+0) = 128;    // Y1
////                                    frameYUYV.at<uint8_t>(iii,jjj+3) = 255;    // V
////                                    frameYUYV.at<uint8_t>(iii,jjj+2) = 128;    // Y2
////                                }
//                            }
//                        }

                        mean_u /= (128*46);
                        mean_v /= (128*46);
                        mean_y /= (128*46);

//                        mean_u /= (128*46);
//                        mean_v /= (128*46);
//                        mean_y /= (128*46);

//                        std::cout << "cnt: " << cocnt << std::endl;
//                        std::cout << "Y; min: " << min_y << "\tmax: " << max_y << "\tmean: " << mean_y << "\trange: " << max_y-min_y << std::endl;
//                        std::cout << "U; min: " << min_u << "\tmax: " << max_u << "\tmean: " << mean_u << "\trange: " << max_y-min_u << std::endl;
//                        std::cout << "V; min: " << min_v << "\tmax: " << max_v << "\tmean: " << mean_v << "\trange: " << max_y-min_v << std::endl;

                        cv::cvtColor(frameYUYV,frameC_mat,  CV_YUV2RGB_YVYU );
#else

                        //rectify and calibrate the stereo image
                        /* as it happens, this gives worst results due to the very small images
                        cv::Mat bigL (480,640, CV_8UC1);
                        cv::Mat bigR (480,640, CV_8UC1);
                        cv::resize(frameL,bigL,bigL.size(),0,0,CV_INTER_LINEAR);
                        cv::resize(frameR,bigR,bigR.size(),0,0,CV_INTER_LINEAR);
                        cv::remap(bigL, bigL, (cv::Mat) _mx1, (cv::Mat) _my1 ,CV_INTER_LINEAR);
                        cv::remap(bigR, bigR, (cv::Mat) _mx2, (cv::Mat) _my2 ,CV_INTER_LINEAR);

                        cv::resize(bigL,frameL_mat,frameL.size(),0,0,CV_INTER_LINEAR);
                        cv::resize(bigR,frameR_mat,frameR.size(),0,0,CV_INTER_LINEAR);
                        */



                       // tmpjets = (tmpjets+1) % 30;
                       // if (tmpjets==1) {
                        g_lockWaitForImage.lock();
                        frameL.copyTo(frameL_2ndbuf);
                        frameR.copyTo(frameR_2ndbuf);
                        copyNewImage =true;
                        g_lockWaitForImage.unlock();

                   // }
//                        frameL.copyTo(frameL_mat);
//                        frameR.copyTo(frameR_mat);
#endif



                    }
                    current_line = 0;
                }

            } // if header

            //remember the 3 previous bytes in seperate variables in order to check for headers:
            prevbuf3 = prevbuf2;
            prevbuf2 = prevbuf1;
            prevbuf1 = buffer[i];

        } // for buffer loop

        //copy the end of the (current) buffer to the start of the (newly to be received) buffer:
        //(this is necessary because we want to check the SOL as well as the EOL from one if statement)
        //((it could be avoided using a double buffer, but this will need extra logic making the code less clear)
        prevbuf3 = prevbuf2;
        prevbuf2 = prevbuf1;
        prevbuf1 = buffer[i];
        i++;
        for (int j = i; j< bufsize; j++) {
            buffer[j-i] = buffer[j];
        }
        tmpsize=bufsize-i;

    } // while loop

    std::cout << "Exit delfly cam thread\n";
}



#else

void DelFly::workerThread() {
    unsigned char buffer[25348];
    int bufsize = 25348; //=128*96*2 + 4 (start of frame header) + 96*4 *2 ( 96 * SOL en EOL)
    int i=0, sv_width=128;
    int current_line = 0;
    im_width = sv_width;
    im_height = 96;

    //comport resetter
    stopwatch_c stopWatch_comportAlive;
    stopWatch_comportAlive.Start();

#ifdef DELFLY_COLORMODE
    //cv::Mat frameYUYV = cv::Mat::zeros(im_height,im_width, CV_8UC2);
    cv::Mat frameDisp = cv::Mat::zeros(im_height,im_width, CV_8UC1);
#else
    cv::Mat frameL = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(im_height,im_width, CV_8UC1);
#endif

    unsigned char prevbuf3,prevbuf2,prevbuf1;
    int tmpsize = 0;

    while (cams_are_running)  {

        //try to receive (the rest of) a full image into the buffer:
        while (bufsize> tmpsize && cams_are_running) {
            int res = RS232_PollComport(buffer+tmpsize,bufsize-tmpsize);
            //std::cout << "bytecounter: " << res << std::endl;
            if (res<0) {
                cams_are_running = false;
                std::cerr << "Serial port read error, is the camera connected?\n";
                return;
            }
            tmpsize+=res;
            if (res<4095) {
                //in order to prevent 100% cpu usage, sleep when the comport buffer wasn full.
                usleep(1000);
            }

            //In order to fix an issue with the drone comport driver; behold for this horrible hack:
            //If no bytes were received for more then 1 second, reinitialise the comport.
            //On the ARDrone this happens after about ~0-30 frames, except for if the program was started after a clean boot.
            //On the PC, this does not happen at all.
            if (res>0) {
                stopWatch_comportAlive.Restart();
            } else if (stopWatch_comportAlive.Read() > 1000) {
                std::cerr << "Restart comport!\n";
                RS232_CloseComport();
                init();
            }
        }

        //loop through the buffer in search of hearders, and handle them accordingly
        for (i=0;i < bufsize-sv_width-2;i++) {

            if (prevbuf3 == 255 && prevbuf2 == 0 && prevbuf1 == 2) { //detect a header


                if ((buffer[i] == 0x80 || buffer[i] == 0xc7) && (buffer[i+4+sv_width] == 0x9d || buffer[i+4+sv_width] == 0xda)) { //Start of Line && End of Line (not checking for header of EOL)

                    //line detected, copy it to the image buffer
                    if (current_line < im_height) { // sometimes bytes get lost, compensate by dropping lines.
                        for (int j = 0; j< im_width;j++) {
                            frameDisp.at<uint8_t>(current_line,j) = buffer[j+i+1] * 12;
                        }

                        i+=sv_width+4;
                        current_line++ ;

                    } else {std::cout << "End Of Frame err \n" ;}

                } else if (buffer[i] == 0xec || buffer[i] == 0xab) { //end of frame

                    // i should converge to 2* sv_width, since that's the for loop boundry
                    // at the start of the program, of just after a stream corruption, i will vary
                    std::cout << "current_line: " << current_line << ", at " << i << "\n";

                    if (copyNewImage) { // if the main thread asks for a new image

                        int cnt = 0;
                        for (int y = 0; y < im_height;y++) {
                            for (int x = 0; x < im_width;x++) {
                                if (frameDisp.at<uint8_t>(y,x) > 5*12) {
                                    cnt++;
                                }
                            }
                        }

                        std::cout << "dispcnt: " << (cnt>>8) << std::endl;



                        cv::applyColorMap(frameDisp,frameC_mat,2);

                        copyNewImage = false;
                        g_lockWaitForImage.unlock();
                    }
                    current_line = 0;
                }

            } // if header

            //remember the 3 previous bytes in seperate variables in order to check for headers:
            prevbuf3 = prevbuf2;
            prevbuf2 = prevbuf1;
            prevbuf1 = buffer[i];

        } // for buffer loop

        //copy the end of the (current) buffer to the start of the (newly to be received) buffer:
        //(this is necessary because we want to check the SOL as well as the EOL from one if statement)
        //((it could be avoided using a double buffer, but this will need extra logic making the code less clear)
        prevbuf3 = prevbuf2;
        prevbuf2 = prevbuf1;
        prevbuf1 = buffer[i];
        i++;
        for (int j = i; j< bufsize; j++) {
            buffer[j-i] = buffer[j];
        }
        tmpsize=bufsize-i;

    } // while loop

    std::cout << "Exit delfly cam thread\n";
}

#endif
