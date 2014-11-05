#include "Delfly.h"
#include <iostream>
#include "rs232.h"

#include "stopwatch.h"


bool DelFly::init () {
    int res = RS232_OpenComport(3000000);  //3000000 is the maximum of the usb bus ftdi serial port
	if (res != 0) {
		std::cerr << "Error opening COM port. Is the camera connected?\n";
	} else {return true;}
}

void DelFly::start () {
    cams_are_running=true;
    thread_cam = std::thread(&DelFly::workerThread,this);
    waitForImage(); // make sure to receive an image before proceeding, to have the width/height variables set
    waitForImage();
    std::cout << "Delfly camera link started!\n";
}

void DelFly::waitForImage() {
	copyNewImage = true;
	g_lockWaitForImage.lock();
}

void DelFly::close () {
    cams_are_running = false;
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
            //On the ARDrone this happens after about ~0-30 frames, except for if the program was started after a clean boot.
            //On the PC, this does not happen at all.
            if (res>0) {
                stopWatch_comportAlive.Restart();
            } else if (stopWatch_comportAlive.Read() > 100) {
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
                            frameL.at<uint8_t>(current_line,j) = buffer[jj++];
                            frameR.at<uint8_t>(current_line,j) = buffer[jj];
#endif
                        }

                        i+=sv_width+4;
                        current_line++ ;

                    } else {std::cout << "End Of Frame err \n" ;}

                } else if (buffer[i] == 0xec || buffer[i] == 0xab) { //end of frame

                    // i should converge to 2* sv_width, since that's the for loop boundry
                    // at the start of the program, of just after a stream corruption, i will vary
                    //std::cout << "current_line: " << current_line << ", at " << i << "\n";

                    if (copyNewImage ) { // if the main thread asks for a new image
#ifdef DELFLY_COLORMODE

                  /*
                        uint32_t totU = 0;
                        uint32_t totV = 0;
                        uint32_t totY = 0;
                        uint32_t totYY = 0;

                        for (int jjj = 0; jjj < 2*im_width-4; jjj+=4) {
                            for (int iii = 0; iii < im_height; iii++) {
                                totY+= frameYUYV.at<uint8_t>(iii,jjj+0);
                                totU+= frameYUYV.at<uint8_t>(iii,jjj+1);
                                totYY+= frameYUYV.at<uint8_t>(iii,jjj+2);
                                totV+= frameYUYV.at<uint8_t>(iii,jjj+3);
                            }
                        }

                        std::cout << "TotU: " << totU/(128*48) << std::endl;
                        std::cout << "TotV: " << totV/(128*48) << std::endl;
                        std::cout << "TotY: " << totY/(128*48) << std::endl;
                        std::cout << "TotYY: " << totYY/(128*48) << std::endl;
                        */

                        cv::cvtColor(frameYUYV,frameC_mat,  CV_YUV2RGB_YVYU );
#else
                        frameL.copyTo(frameL_mat);
                        frameR.copyTo(frameR_mat);
#endif

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