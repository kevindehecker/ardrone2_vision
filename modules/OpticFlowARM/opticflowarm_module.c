/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


/*
 * Author: Wim 
 * Runs Maxime's OpticFlow written for DSP completely on ARM side
 */

// flag 1 23/09/2014 13:27


// Own header
#include "opticflowarm_module.h"

// Semaphore to control memory acces
#include <semaphore.h>

// UDP RTP Images
#include "udp/socket.h"

// Timing
#include <sys/time.h>

// Calculations
#include <math.h>

// Threaded computer vision
#include <pthread.h>

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization.h"


//Values from optitrack system
#include "subsystems/gps.h"
// Video
#include "v4l/video.h"
#include "resize.h"

#include "encoding/jpeg.h"
#include "encoding/rtp.h"

// Own definitions
#define VIDEO_IN_W	320
#define VIDEO_IN_H	240

// Standard headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> // added for usleep


// Optical flow code
#include "opticflow_sobel_2.h"
#include "calcFlowXYZ.h"

// Downlink
//#ifndef DOWNLINK_DEVICE
//#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
//#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Variables
// filtered optic_flow_velocity
float Vx_filt = 0.0, Vy_filt=0.0, Vz_filt=0.0;

static sem_t sem_results;

float OFXInt = 0;
float OFYInt = 0;
unsigned int pGainHover = 1000;
unsigned int iGainHover = 0;

#define SAT	1500 // as int32 angle, so angle in rad = 1500/(1<<INT32_ANGLE_FRAC) = 1500/(2^12) = 0.36 rad (21 deg)
unsigned char saturateX = 0, saturateY = 0;
unsigned char first = 1;

volatile struct Int32Eulers cmd_euler;
volatile uint8_t computervision_thread_has_results = 0;

void opticflow_module_run(void) {
  
	if(optic_flow_ctrl == 1)
	{
	  	
		// Read Latest Vision Module Results
		if (computervision_thread_has_results)
		{
			computervision_thread_has_results = 0;

			sem_wait(&sem_results);

			if(saturateX==0)
			{
				OFYInt -= iGainHover*Vy_filt; // minus sign: positive Y velocity = negative phi command
			}
			if(saturateY==0)
			{
				OFXInt += iGainHover*Vx_filt;
			}

			cmd_euler.phi = (pGainHover*-Vy_filt + OFYInt); // minus sign: positive Y velocity = negative phi command
			cmd_euler.theta = (pGainHover*Vx_filt + OFXInt);

			sem_post(&sem_results);

			saturateX = 0; saturateY = 0;
			
			/*// if saturation limits are reached, integrator should not integrate extra
			if(cmd_euler.phi<-SAT){cmd_euler.phi = -SAT; saturateX = 1;}
			else if(cmd_euler.phi>SAT){cmd_euler.phi = SAT; saturateX = 1;}
			if(cmd_euler.theta<-SAT){cmd_euler.theta = -SAT; saturateY = 1;}
			else if(cmd_euler.theta>SAT){cmd_euler.theta = SAT;saturateY = 1;}
			*/
		}
		/*else
		{
			cmd_euler.phi = 0;
			cmd_euler.theta = 0;
		}*/

		if(first)
		{
			cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
			first = 0;
		}

		// for downlink only
		int32_t dl_cmd_phi = cmd_euler.phi;
		int32_t dl_cmd_theta = cmd_euler.theta;
		int32_t dl_cmd_psi = cmd_euler.psi;
		
		float dl_cmd_phi_f = ANGLE_FLOAT_OF_BFP(cmd_euler.phi);
		float dl_cmd_theta_f = ANGLE_FLOAT_OF_BFP(cmd_euler.theta);	
		float dl_cmd_psi_f = ANGLE_FLOAT_OF_BFP(cmd_euler.psi);		
		
		stabilization_attitude_set_rpy_setpoint_i(&cmd_euler); // wait with this line until testing is justifiable
		DOWNLINK_SEND_OPTICFLOW_CTRL(DefaultChannel, DefaultDevice, &dl_cmd_phi, &dl_cmd_theta, &dl_cmd_psi, &dl_cmd_phi_f, &dl_cmd_theta_f, &dl_cmd_psi_f);
	}
	else
	{
		// when optic flow inactive, do not increase integrator
		OFXInt = 0;
		OFYInt = 0;
	}

}


// COMPUTER VISION THREAD
void send_this_image(struct img_struct* img_new);

// Timers
struct timeval start_time;
struct timeval end_time;

#define USEC_PER_SEC 1000000
volatile long time_elapsed (struct timeval *t1, struct timeval *t2);
volatile long time_elapsed (struct timeval *t1, struct timeval *t2)
{
	long sec, usec;
	sec = t2->tv_sec - t1->tv_sec;
	usec = t2->tv_usec - t1->tv_usec;
	if (usec < 0) {
	--sec;
	usec = usec + USEC_PER_SEC;
	}
	return sec*USEC_PER_SEC + usec;
}
void start_timer() {
	gettimeofday(&start_time, NULL);
}
long end_timer() {
	gettimeofday(&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}

struct timeval start_time_rates;
struct timeval end_time_rates;

void start_timer_rates(void) {
	gettimeofday(&start_time_rates, NULL);
}
long end_timer_rates(void) {
	gettimeofday(&end_time_rates, NULL);
	return time_elapsed(&start_time_rates, &end_time_rates);
}

//Camera parameters
#define Fx		343.1211
#define Fy		348.5053

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 1;



void *computervision_thread_main(void* data)
{
  
  printf("computervision_thread_main started\n");
  
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video2";
  vid.w=320;
  vid.h=240;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }
  
  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // declare and initialise parameters for calcFlowXYZ
  unsigned int profileX[320] = {}; 	// feature histograms
  unsigned int prevProfileX[320] = {};
  unsigned int profileY[240] = {};
  unsigned int prevProfileY[240] = {};
  
  //unsigned int *histTempX,*histTempY; 	// hist Temp

  unsigned int threshold = 1000; 	// initial feature detection threshold
  
  unsigned int nbSkipped = 0;
  unsigned int framesskip = 5;
  unsigned int skip_counter = 0;
  int Txp=0,Typ=0,Tzp=0;			// optical flow in percent px
  float Tx=0.0,Ty=0.0,Tz=0.0;			// optical flow in px
  
  unsigned int erreur;
  unsigned int percentDetected;
  
  float FPS;
  
  struct FloatRates* body_rate;
  float dt=0.0;
  long diffTime;
  float p_temp=0.0,	q_temp=0.0;		// originially, these variables were defined outside computer_vision_thread
  float p_corr=0.0, 	q_corr=0.0;
  float Tx_corr = 0.0, 	Ty_corr=0.0;
  
  float Vx=0.0, Vy=0.0, Vz=0.0; 	// velocity in body frame
  
  // flow filter
  float alpha_v = 0.16;
  
  // sonar height
  float h_sonar_prev = 0.0;
  float h_sonar = 0.0;
  float h_sonar_m = 0.0;
  float h_sonar_raw = 0.0;
  float h_gps_corr = 0.0;
  float h = 0.0;
  float sonar_scaling = 0.68; // rough estimate of the scaling 
  float alpha_h = 0.5;
  
  uint8_t msg_id = 0;
  
  // temporary
  sem_post(&sem_results);
  uint8_t profileYpart1[120] = {}; // for downlink
  uint8_t profileYpart2[120] = {};
  uint8_t part_id = 0;
  int img_counter = 0;
  float Txp_slow=0.0;
  float Typ_slow=0.0;
  float Tnp_slow=0.0;
  
  
  while (computer_vision_thread_command > 0)
  {
    
    
     // Grab image from camera
     video_grab_image(&vid, img_new);
     
      
     printf("new loop\n");
     img_counter++;
     if (img_counter == 60) {
       printf("starting image writing\n");
       //send_this_image(img_new);
       img_counter=0;
       printf("image written\n");
     }
     
     // ----------------------------
     // CALCULATE FEATURE HISTOGRAMS
     // ----------------------------
     percentDetected = ApplySobelFilter2(img_new, profileX, profileY, &threshold); 
     
     
     // --------------
     // CALCULATE FLOW
     // --------------
     // Tx/Ty/Tz are (probably) integers of 0.01 px
     
     erreur = calcFlowXYZ(&Txp,&Typ,&Tzp,profileX,prevProfileX,profileY,prevProfileY,&nbSkipped,&framesskip);
     
     // Convert from [percent px] to [px] (this is also a cast from int to float)
     // watch out!! from image axes to body axes conversion!!
     Tx = (float)Typ/100/(framesskip+1); 	// 10 because we are skipping 9! shouldn't be hardcoded.
     Ty = -(float)Txp/100/(framesskip+1);

     Txp_slow = 0.05*(float)Txp/100 + 0.95*Txp_slow;
     Typ_slow = 0.05*(float)Typ/100 + 0.95*Typ_slow;
     Tnp_slow = sqrt(Txp_slow*Txp_slow+Typ_slow*Typ_slow);

     // framesskip control
     if (nbSkipped==0) { // only adjust framesskip after just calculating new flow
	if (abs(Typ)==1200 || abs(Txp)==1200) {
	  if (framesskip > 5) {
	    framesskip=framesskip-3;
	  }
	  else if (framesskip>0){
	    framesskip--;
	    
	  }
	}
	if (Tnp_slow < 2 && framesskip < 9) {
	  skip_counter++;
	  if (skip_counter == 60) {
	    skip_counter=0;
	    framesskip++;
	  }
	  
	}
     }
       
     
     // ------------------
     // CALCULATE VELOCITY
     // ------------------
     // first, wait to proceed
     sem_wait(&sem_results);
     
     // delta t (for FPS and for micro-rotation)
     diffTime = end_timer();
     start_timer();
     dt = (float)(diffTime)/USEC_PER_SEC;
     
     // frames per second
     FPS = 1/dt;
          
     // MICRO ROTATION (adapted code)			
     // current body rates
     body_rate = stateGetBodyRates_f();
     p_temp = body_rate->p;
     q_temp = body_rate->q;     
     
     //micro roation
     p_corr = p_temp*dt; // this is actually not a rate but an angle (RAD)
     q_corr = q_temp*dt; // this is actually not a rate but an angle (RAD)
     
     // calculate corrected flow with current roll/pitch movements (in px)
     // watch out: Fy,Fx are in image axes!
     Tx_corr = (float)Tx;// - q_corr*(float)Fy;
     Ty_corr = (float)Ty;// + p_corr*(float)Fx;
     
     // SONAR HEIGHT [m]
     h_sonar_prev = h_sonar;
     h_sonar_raw = (float)ins_impl.sonar_z/1000*sonar_scaling;// sonar_z is an integer with unit [mm]
     
     // delete outliers
     if (abs(h_sonar-h_sonar_prev)>3)
       h_sonar = h_sonar_prev;
     else
       h_sonar = h_sonar_raw;
     
     // corrected gps height
     h_gps_corr = stateGetPositionEnu_f()->z - 0.32; // 0.32 is the standard optitrack offset
     
     // which height do you use?
     h = h_sonar;

     // ACTUAL VELOCITY CALCULATION
     // velocity calculation from optic flow
     // watch out: Fy,Fx are in image axes
     Vx = h*FPS*(float)Tx_corr/(Fy); // [m/s]
     Vy = h*FPS*(float)Ty_corr/(Fx); // [m/s]
     Vz = h*FPS*(float)Tz; 
     
     // filter V
     Vx_filt = alpha_v*Vx + (1-alpha_v)*Vx_filt;
     Vy_filt = alpha_v*Vy + (1-alpha_v)*Vy_filt;
     
    
     sem_post(&sem_results);
 
     // AUTOPILOT BODY VELOCITY
     // Enu velocity
     struct NedCoor_f* V_ned;
     V_ned = stateGetSpeedNed_f();
     
     
     // attitude
     struct FloatQuat* BodyQuaternions = stateGetNedToBodyQuat_f();
     
     struct FloatRMat Rmat_Ned2Body;
     
     // rotation matrix from quaternions
     FLOAT_RMAT_OF_QUAT(Rmat_Ned2Body,*BodyQuaternions);
     
     struct FloatVect3 vect_ned;
     vect_ned.x = V_ned->x;
     vect_ned.y = V_ned->y;
     vect_ned.z = V_ned->z;
     
     struct FloatVect3 V_body;

     // multiply
     FLOAT_RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, vect_ned);
     
     
     // DOWNLINK
     if (msg_id > 99)
       msg_id = 1;	
     else
       msg_id++;
  
     
     for (int i=0;i<240;i++) {
       
       if (i<120) {
	 profileYpart1[i] = profileY[i];
       }
       else {
	 profileYpart2[i-120] = profileY[i];
       }
     }
    
     uint32_t current_modus = framesskip;
     
     DOWNLINK_SEND_OF_VELOCITIES(DefaultChannel,DefaultDevice,&Txp_slow,&Typ_slow,&Tnp_slow,&(V_body.x),&(V_body.y),&(V_body.z),&(vect_ned.x),&(vect_ned.y),&(vect_ned.z));
     DOWNLINK_SEND_OF_DEBUG(DefaultChannel,DefaultDevice,&Tx,&Ty,&Tz, &Vx, &Vy, &Vz, &Vx_filt, &Vy_filt, &Vz_filt, &p_corr, &q_corr, &percentDetected, &current_modus, &erreur, &FPS, &h_sonar, &h_sonar_raw, &h, &msg_id);
     
     //send a LOT of hist data
     part_id=1;
     DOWNLINK_SEND_OF_HIST(DefaultChannel,DefaultDevice,&msg_id,&part_id,&Typ,&erreur,120,profileYpart1);// warning from this line (truncation of integer in profileX)
     part_id=2;
     DOWNLINK_SEND_OF_HIST(DefaultChannel,DefaultDevice,&msg_id,&part_id,&Typ,&erreur,120,profileYpart2);// warning from this line (truncation of integer in profileX)
     
   
     // report new results
     computervision_thread_has_results++;
     
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;    
  }
  
  


void opticflow_module_start(void)
{
  
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void opticflow_module_stop(void)
{
  computer_vision_thread_command = 0;
}


int optic_flow_ctrl_start(void) {
  optic_flow_ctrl = 1;
  return 0;
}

int optic_flow_ctrl_stop(void) {
  optic_flow_ctrl = 0;
  return 0;
}







void send_this_image(struct img_struct* img_new) {

    // Video Compression
    uint8_t* jpegbuf = (uint8_t*)malloc(img_new->h*img_new->w*2);

    // Network Transmit
    struct UdpSocket* vsock;
    vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);  
  
    // Video Resizing
    #define DOWNSIZE_FACTOR   1
    uint8_t quality_factor  = 99; // From 0 to 99 (99=high)
    uint8_t dri_jpeg_header = 0;
  
    
    
    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (img_new->buf, jpegbuf, quality_factor, image_format, img_new->w, img_new->h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
    
        send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        img_new->w, img_new->h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );
    
}

  
  



