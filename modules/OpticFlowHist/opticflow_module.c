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
#include "opticflow_module.h"

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
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"


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
#define Fx		343.1211 // Camera focal length (px/rad)
#define Fy		348.5053 // Camera focal length (px/rad)

// Standard headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> // added for usleep


// Optical flow code
#include "opticflow_sobelfilter.h"
#include "calcflow.h"
#include "opticflow_kalmanfilt.h"

// Downlink
//#ifndef DOWNLINK_DEVICE
//#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
//#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// defines
#ifndef STABILIZATION_ATTITUDE_DEADBAND_A
#define STABILIZATION_ATTITUDE_DEADBAND_A 0
#endif

#ifndef STABILIZATION_ATTITUDE_DEADBAND_E
#define STABILIZATION_ATTITUDE_DEADBAND_E 0
#endif

#ifndef OPTICFLOW_HOVER
#define OPTICFLOW_HOVER 	0
#endif

#ifndef OPTICFLOW_HORIZONTAL
#define OPTICFLOW_HORIZONTAL 	1
#endif

// optic flow control
uint8_t opticflow_control_mode = OPTICFLOW_HORIZONTAL;
float Vx_ctrl = 0.0, Vy_ctrl=0.0, Vz_ctrl=0.0;
float Vx_sp = 0.0, Vy_sp = 0.0, Vz_sp = 0.0;
float Vx_ctrl_err = 0.0, Vy_ctrl_err = 0.0, Vz_ctrl_err = 0.0;

float OFXInt = 0;
float OFYInt = 0;

// default settings
uint32_t pGainHover = 500;
uint32_t iGainHover = 5;
uint32_t dGainHover = 300;


#define SAT	1500 // as int32 angle, so angle in rad = 1500/(1<<INT32_ANGLE_FRAC) = 1500/(2^12) = 0.36 rad (21 deg)
unsigned char saturateX = 0, saturateY = 0;
unsigned char first = 1;
int32_t cmd_phi0 	= 0;
int32_t cmd_theta0 	= 0;
int32_t cmd_psi0 	= 0;

float rc_phi = 0.0;
float rc_theta = 0.0;
float rc_psi = 0.0;

struct Int32Eulers cmd_euler;
volatile uint8_t computervision_thread_has_results = 0;

// acceleration in horizontal plane
struct NedCoor_f* A_ned;
void UpdateAccel(void);
float Ahx = 0.0;
float Ahy = 0.0;
float Ahx_m = 0.0;
float Ahy_m = 0.0;

// Complementary filter
uint8_t first_compl = 1;
float Vx_err_m = 0.0;
float Vy_err_m = 0.0;
float Vx_err_int_m = 0.0;
float Vy_err_int_m = 0.0;
float Ahx_bias_m = 0.0;
float Ahy_bias_m = 0.0;
float Vx_err_f = 0.0;
float Vy_err_f = 0.0;
float Vx_err_int_f = 0.0;
float Vy_err_int_f = 0.0;
float Ahx_bias_f = 0.0;
float Ahy_bias_f = 0.0;
float Vx_compl_m = 0.0;
float Vy_compl_m = 0.0;
float Vx_compl_f = 0.0;
float Vy_compl_f = 0.0;
float pGainCompl = 40;
float iGainCompl = 6.28;

// optic flow
float Vx=0.0,    Vy=0.0,    	Vz=0.0; 	// velocity in body frame
float Vx_filt = 0.0, 		Vy_filt=0.0, 		Vz_filt=0.0;
float Vx_corr_angle=0.0, 	Vy_corr_angle=0.0;
float Vx_filt_corr_angle=0.0, 	Vy_filt_corr_angle=0.0;  

// body velocity
struct FloatVect3 V_body;
void UpdateAutopilotBodyVel(void);

void initflowdata(FILE *fp);
void saveflowdata(FILE *fp,int msg_id,int Txp,int Typ,unsigned int *histX,unsigned int *prevhistX,unsigned int *histY,unsigned int *prevhistY,float *errormapx,float *errormapy,unsigned int window);
void savehistXY(FILE *fp,int Txp,int Typ,unsigned int *histx, unsigned int *histy, int counter);
void saveThisImage(unsigned char *frame_buf, int width, int height, int counter);
void saveSingleImageDataFile(unsigned char *frame_buf, int width, int height, char filename[100]);

static float get_rc_roll_f(void);
static float get_rc_pitch_f(void);
static float get_rc_yaw_f(void);



void opticflow_module_run(void) {
  
 
  printf("phi, theta, psi: %.2f, %.2f, %.2f\n",rc_phi,rc_theta,rc_psi);
  
	if(autopilot_mode == AP_MODE_OPTIC_FLOW) {
	  
	  switch (opticflow_control_mode) {
	    case OPTICFLOW_HOVER:
	      opticflow_hover_run();
	      break;
	    case OPTICFLOW_HORIZONTAL:
	      opticflow_horizontal_run();
	      break;
	    default:
	      opticflow_hover_run();
	      break;
	  }
	  
	}
	else {
	  opticflow_h_control_stop();
	}  
}


 



void opticflow_hover_run(void) {
  
  Vx_sp = 0;
  Vy_sp = 0;
  
  // run control loop with zero setpoints
  opticflow_h_control_run();
}

void opticflow_horizontal_run(void) {
  
  // read pilot inputs (in radians)
  rc_phi = get_rc_roll_f();
  rc_theta = get_rc_pitch_f();
  rc_psi = get_rc_yaw_f();
  
  // calculate velocity setpoints
  Vx_sp = -rc_theta*10;
  Vy_sp = rc_phi*10;
  
  printf("Vx_sp, Vy_sp: %f,%f\n",Vx_sp,Vy_sp);
  
  // run control loop with setpoints
  opticflow_h_control_run(); 
  
}

void opticflow_h_control_run(void) {
  
  

  if(first)
  {
	  
	  // last known setpoints (from RC/NAV mode) are the trim setpoints:
	  cmd_phi0 	= stab_att_sp_euler.phi;
	  cmd_theta0 	= stab_att_sp_euler.theta;
	  cmd_psi0  	= stab_att_sp_euler.psi;
	  
	  // psi command is current heading
	  cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
	  first = 0;
  }	  	
  // Read Latest Vision Module Results
  if (computervision_thread_has_results)
  {
    
	  Vx_ctrl_err = Vx_sp - Vx_ctrl;
	  Vy_ctrl_err = Vy_sp - Vy_ctrl;	  
	    
	  computervision_thread_has_results = 0;

	  if(saturateX==0)
	  {
		  OFYInt += iGainHover*Vy_ctrl_err; // minus sign: positive Y velocity = negative phi command
	  }
	  if(saturateY==0)
	  {
		  OFXInt += iGainHover*Vx_ctrl_err;
	  }

	  cmd_euler.phi 	= cmd_phi0   + (pGainHover* Vy_ctrl_err + OFYInt + dGainHover*-Ahy); // minus sign: positive Y velocity = negative phi command
	  cmd_euler.theta 	= cmd_theta0 + (pGainHover*-Vx_ctrl_err - OFXInt + dGainHover*Ahx);
	  cmd_euler.psi 	= cmd_psi0;
	  
	  saturateX = 0; saturateY = 0;
	  
	  // if saturation limits are reached, integrator should not integrate extra
	  if(cmd_euler.phi < -SAT) {
	    cmd_euler.phi = -SAT; 
	    saturateX = 1; 
	  }
	  else if(cmd_euler.phi > SAT) {
	    cmd_euler.phi = SAT; 
	    saturateX = 1;
	  }
	  if(cmd_euler.theta < -SAT) { 
	    cmd_euler.theta = -SAT; 
	    saturateY = 1;
	  }
	  else if(cmd_euler.theta > SAT) { 
	    cmd_euler.theta = SAT;
	    saturateY = 1; 
	  }
	  
  }

  // for downlink only
  int32_t dl_cmd_phi = cmd_euler.phi;
  int32_t dl_cmd_theta = cmd_euler.theta;
  int32_t dl_cmd_psi = cmd_euler.psi;

  stabilization_attitude_set_rpy_setpoint_i(&cmd_euler); // wait with this line until testing is justifiable
  DOWNLINK_SEND_OF_CTRL(DefaultChannel, DefaultDevice, &dl_cmd_phi, &dl_cmd_theta, &dl_cmd_psi, &cmd_phi0, &cmd_theta0, &cmd_psi0, &pGainHover, &iGainHover, &dGainHover);
}

void opticflow_h_control_stop(void) {
  // when optic flow inactive, reset integrator
  first=1;
  OFXInt = 0;
  OFYInt = 0;
}

// COMPUTER VISION THREAD


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

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 1;




void UpdateAccel(void) {
  
    
    // acceleration measurement
    A_ned = stateGetAccelNed_f();
    
    
    
    
    // attitude
    float psi = stateGetNedToBodyEulers_f()->psi;
    
    // horizontal acceleration in body-x, body-y
    Ahx_m = cosf(psi)*A_ned->x  + sinf(psi)*A_ned->y;
    Ahy_m = -sinf(psi)*A_ned->x + cosf(psi)*A_ned->y;
    
    // filter
    float alpha_a = 0.05;
    Ahx = alpha_a*Ahx_m + (1-alpha_a)*Ahx;
    Ahy = alpha_a*Ahy_m + (1-alpha_a)*Ahy;
}

void ComplementaryFilter_run(float dt);
void ComplementaryFilter_run(float dt) {
    
    if (first_compl) {
      first_compl = 0;
      Vx_compl_m = Vx_filt_corr_angle;
      Vy_compl_m = Vx_filt_corr_angle;      
      Vx_compl_f = Vx_filt_corr_angle;
      Vy_compl_f = Vx_filt_corr_angle;
    }
    
    // raw accelerometer
    Vx_err_m = Vx_compl_m - Vx_filt_corr_angle;
    Vy_err_m = Vy_compl_m - Vx_filt_corr_angle;

    Vx_err_int_m += iGainCompl*Vx_err_m*(dt);
    Vy_err_int_m += iGainCompl*Vy_err_m*(dt);

    Ahx_bias_m = pGainCompl*Vx_err_m + Vx_err_int_m;
    Ahy_bias_m = pGainCompl*Vy_err_m + Vy_err_int_m;  
  
    Vx_compl_m = Vx_compl_m + (Ahx_m-Ahx_bias_m)*(dt);
    Vy_compl_m = Vy_compl_m + (Ahx_m-Ahy_bias_m)*(dt); 
    
    // filtered accelerometer
    Vx_err_f = Vx_compl_f - Vx_filt_corr_angle;
    Vy_err_f = Vy_compl_f - Vx_filt_corr_angle;

    Vx_err_int_f += iGainCompl*Vx_err_f*(dt);
    Vy_err_int_f += iGainCompl*Vy_err_f*(dt);

    Ahx_bias_f = pGainCompl*Vx_err_f + Vx_err_int_f;
    Ahy_bias_f = pGainCompl*Vy_err_f + Vy_err_int_f;      
    
    Vx_compl_f = Vx_compl_f + (Ahx  -Ahx_bias_f)*(dt);
    Vx_compl_f = Vy_compl_f + (Ahx  -Ahy_bias_f)*(dt);
}


void UpdateAutopilotBodyVel(void) {
     
     // Ned velocity
     struct NedCoor_f* V_ned;
     V_ned = stateGetSpeedNed_f();
     
     
     // attitude
     struct FloatQuat* BodyQuaternions = stateGetNedToBodyQuat_f();
     
     struct FloatRMat Rmat_Ned2Body;
     
     // rotation matrix from quaternions_filt_corr
     FLOAT_RMAT_OF_QUAT(Rmat_Ned2Body,*BodyQuaternions);
     
     struct FloatVect3 vect_ned;
     vect_ned.x = V_ned->x;
     vect_ned.y = V_ned->y;
     vect_ned.z = V_ned->z;

     // multiply
     FLOAT_RMAT_VMULT(V_body, Rmat_Ned2Body, vect_ned);  
}




void *computervision_thread_main(void* data)
{
  
  // identification for checking calculations
  uint8_t msg_id = 0;
  
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

  // declare and initialise parameters for calcflow
  unsigned int histX[WIDTH] = {}; 	// feature histograms
  unsigned int prevhistX[WIDTH] = {};
  unsigned int histY[HEIGHT] = {};
  unsigned int prevhistY[HEIGHT] = {};
  
  unsigned int window = 12;
  unsigned int error;  
  float errormapx[2*window+1];
  float errormapy[2*window+1];

 
  unsigned int curskip = 0;
  unsigned int framesskip = 5;
  unsigned int lowflow_counter = 0;
  int 	Txp=0,     Typ=0,     Tzp=0;			// optical flow in percent px
  int 	prevTxp=0, prevTyp=0, prevTzp=0;
  float Tx=0.0,	   Ty=0.0,    Tz=0.0;			// optical flow in px
  
  float FPS;
  struct FloatEulers* body_angle;
  
  float phi_corr 	= 0.0;
  float phi_temp 	= 0.0;
  float phi_temp_prev 	= 0.0;
  float theta_corr 	= 0.0;
  float theta_temp 	= 0.0;
  float theta_temp_prev = 0.0;
  
  float dt	=0.0;
  float dt_total=0.0;
  long 	diffTime;
  
  float Tx_slow=0.0;
  float Ty_slow=0.0;
  float Tn_slow=0.0;
  float Tx_corr_angle = 0.0, 	Ty_corr_angle=0.0;

  // flow velocity filter
  float alpha_v = 0.16;
  
  // sonar height
  // float h_sonar 	= 0.0;
  // float h_ap 		= 0.0;
  float h 		= 0.0;
  
  // kalman filter
  int first_kalman = 1;
  float opticflow_noise = 0.01;	   // 0.01 = low, 10 = high
  float opticflow_uncertainty = 0; // 0 = min, 1=max
  float Vx_kalman 	= 0.0;
  float Vy_kalman 	= 0.0;  
  float Vxm_kalman 	= 0.0;
  float Vym_kalman 	= 0.0;
  float Axm_kalman 	= 0.0;
  float Aym_kalman 	= 0.0;  
  
  // downlink kalman filter
  float P00 = 0.0;
  float P11 = 0.0;
  float P22 = 0.0;
  float P33 = 0.0;
  float Q00 = 0.0;
  float Q11 = 0.0;
  float R00 = 0.0;
  float R11 = 0.0;
  
  // old variables (not deleted because they're downlinked)
  unsigned int threshold = 1000; 	// initial feature detection threshold
  unsigned int percentDetected;
  float p_corr;
  float q_corr;
  
  // filenames for saving data
  FILE *fp_flowdata;
  char filename2[100];
  sprintf(filename2, "%sflowdata.csv","/data/video/"); 
  fp_flowdata=fopen(filename2, "w");  
  initflowdata(fp_flowdata);
  
  while (computer_vision_thread_command > 0)
  {
    
    
     printf("cv thread loop\n");
     //
     msg_id++;
    
     // Grab image from camera
     video_grab_image(&vid, img_new);
     
     ////////////////////////////////
     // CALCULATE FLOW
     ////////////////////////////////
     // Txp/Typ/Tzp are integers of 0.01 px
     
    // skip?
    curskip++;
    if(curskip < framesskip) {
      Txp = prevTxp;
      Typ = prevTyp;
      Tzp = prevTzp;
    }
    // or use calculated values
    else {
      ApplySobelFilter(img_new, histX, histY);
      
      // calculate flow from current and previous histograms
      error = calcFlow2(&Txp,&Typ,&Tzp,histX,prevhistX,histY,prevhistY,&curskip,&framesskip,&window,errormapx,errormapy);

      prevTxp = Txp;
      prevTyp = Typ;
      prevTzp = Tzp;
      
      // write flowdata to file (only if motors are on)
      if (autopilot_in_flight) 
	saveflowdata(fp_flowdata,msg_id,Txp,Typ,histX,prevhistX,histY,prevhistY,errormapx,errormapy,window);
      
      curskip=0; // set curskip to zero if you have just calculated new flow
      
      // prevhist = hist
      memcpy(prevhistX,histX,WIDTH*sizeof(unsigned int));
      memcpy(prevhistY,histY,HEIGHT*sizeof(unsigned int));	



      
    }     

     
     // Convert from [percent px] to [px] (this is also a cast from int to float)
     // watch out!! from image axes to body axes conversion!!
     // x_body =  y_image
     // y_body = -x_image
     Tx = (float)Typ/FLOWFACT/(framesskip+1); 
     Ty = -(float)Txp/FLOWFACT/(framesskip+1);
     
    
     // slow flow for frameskip control
     float alpha_T = 0.05;
     Tx_slow = alpha_T*(float)Txp/FLOWFACT + (1-alpha_T)*Tx_slow; // divide by 100 because of the per-cent scaling of the flow
     Ty_slow = alpha_T*(float)Typ/FLOWFACT + (1-alpha_T)*Ty_slow;
     Tn_slow = sqrt(Tx_slow*Tx_slow+Ty_slow*Ty_slow);
     
     // delta t (for FPS and for micro-rotation)
     diffTime = end_timer();
     start_timer();
     dt = (float)(diffTime)/USEC_PER_SEC;
     
     // frames per second
     dt_total = dt_total+dt; // delta t since last calculated flow
     if (curskip==0) {
      FPS = ((float)framesskip+1)/dt_total; // literally frames-per-second
      dt_total=0;
     }
     
    
     ////////////////////////////////
     // MICRO ROTATION
     ////////////////////////////////
     
     // calculate microroation with body angle difference
     if (curskip==0) {
       body_angle 	= stateGetNedToBodyEulers_f();
       phi_temp 	= body_angle->phi;
       theta_temp 	= body_angle->theta;
       
       phi_corr 	= (phi_temp-phi_temp_prev)/(framesskip+1); // correction angle
       theta_corr 	= (theta_temp-theta_temp_prev)/(framesskip+1); // correction angle
     
       phi_temp_prev 	= phi_temp;
       theta_temp_prev 	= theta_temp;
     }

     // calculate corrected flow with current roll/pitch movements (in px)
     // watch out: Fy,Fx are in image axes!
     
     // flow correction for microrotation
     Tx_corr_angle = (float)Tx - theta_corr*(float)Fy;
     Ty_corr_angle = (float)Ty + phi_corr*(float)Fx;     
      
     ////////////////////////////////
     // FRAMESKIP CONTROL
     ////////////////////////////////
     #define TMIN 	5
     #define TMIN_STEPS 20
     #define MAX_SKIP 	9
     // only adjust framesskip after just calculating new flow
     if (curskip==0) { 
       
	// if flow is equal to max window
	if (abs(Typ)==window*FLOWFACT || abs(Txp)==window*FLOWFACT) {   
	  if (framesskip > 5) {
	    framesskip=framesskip-3;
	  }
	  else if (framesskip>0){
	    framesskip--;
	    
	  }
	}
	// if filtered flow stays low
	if (Tn_slow < TMIN && framesskip < MAX_SKIP) {
	  lowflow_counter++;
	  if (lowflow_counter > TMIN_STEPS/(framesskip+1)) {
	    lowflow_counter=0;
	    framesskip++;
	  }
	  
	}
     }
     
     ////////////////////////////////
     // HEIGHT
     ////////////////////////////////
     
     
#ifdef USE_SONAR     
     // SONAR HEIGHT
     h = (float)ins_impl.sonar_z/1000;// sonar_z is an integer with unit [mm]
#else
     // Autopilot state
     h = stateGetPositionEnu_f()->z;
#endif
     
     ////////////////////////////////
     // CALCULATE VELOCITY
     ////////////////////////////////

     // velocity calculation from optic flow
     // watch out: Fy,Fx are in image axes
     
     // V not corrected for microrotation
     Vx = h*FPS*(float)Tx/(Fy); // [m/s]
     Vy = h*FPS*(float)Ty/(Fx); // [m/s]
     Vz = h*FPS*(float)Tz; 
     
     // V corrected for microrotation
     Vx_corr_angle = h*FPS*(float)Tx_corr_angle/(Fy);
     Vy_corr_angle = h*FPS*(float)Ty_corr_angle/(Fx);     
     
     ////////////////////////////////
     // FILTERED VELOCITY
     ////////////////////////////////
     
     alpha_v = 0.16;
     
     // uncorrected, filtered velocity
     Vx_filt = alpha_v*Vx + (1-alpha_v)*Vx_filt;
     Vy_filt = alpha_v*Vy + (1-alpha_v)*Vy_filt;
     
     // corrected, filtered velocity
     Vx_filt_corr_angle = alpha_v*Vx_corr_angle + (1-alpha_v)*Vx_filt_corr_angle;
     Vy_filt_corr_angle = alpha_v*Vy_corr_angle + (1-alpha_v)*Vy_filt_corr_angle;     
     
     ////////////////////////////////
     // ACCELERATION UPDATE
     ////////////////////////////////
     UpdateAccel(); 
     
     ////////////////////////////////
     // COMPLEMENTARY FILTER
     ////////////////////////////////
     ComplementaryFilter_run(dt);
     
     ////////////////////////////////
     // KALMAN FILTER
     ////////////////////////////////
     
     if (first_kalman) {
       first_kalman=0;
       KalmanOpticFlow_reset();
     }
     
     // with maximum uncertainty, set optic flow noise really high
     if (opticflow_uncertainty==1) {
       opticflow_noise = 10;
       KalmanOpticFlow_set_R(opticflow_noise,opticflow_noise);
     }
     
     // measurement
     Vxm_kalman = Vx_corr_angle;
     Vym_kalman = Vy_corr_angle;
     Axm_kalman = Ahx;
     Aym_kalman = Ahy;
     
     // update kalman filter
     UpdateKalman(Vxm_kalman, Vym_kalman, Axm_kalman, Aym_kalman, dt);
     
     // new velocity
     Vx_kalman = Vkalmanx;
     Vy_kalman = Vkalmany;
     
     // Which velocity do you use for the control?
     Vx_ctrl = Vx_filt_corr_angle;
     Vy_ctrl = Vy_filt_corr_angle;     
     
     ////////////////////////////////
     // AUTOPILOT BODY VELOCITY
     ////////////////////////////////
     UpdateAutopilotBodyVel();

     ////////////////////////////////
     // DOWNLINK
     ////////////////////////////////
     
     P00 = P[0];
     P11 = P[5];
     P22 = P[10];
     P33 = P[15];
     Q00 = Q[0];
     Q11 = Q[3];
     R00 = R[0];
     R11 = R[3];
     
     DOWNLINK_SEND_OF_VELOCITIES(DefaultChannel,DefaultDevice,&Vx,&Vy,&Vz,&(V_body.x),&(V_body.y),&(V_body.z),&Vx_corr_angle,&Vy_corr_angle,&Vx_corr_angle,&Vy_corr_angle,&Vx_compl_m,&Vx_compl_m,&Vx_compl_f,&Vx_compl_f,&Vx_kalman,&Vy_kalman);
     DOWNLINK_SEND_OF_DEBUG(DefaultChannel,DefaultDevice,&Tx,&Ty,&Tz, &Vx, &Vy, &Vz, &Vx_ctrl, &Vy_ctrl, &Vz_ctrl, &p_corr, &q_corr, &percentDetected, &threshold, &error, &FPS, &h, &msg_id, &framesskip, &window, &autopilot_mode, &Ahx_m, &Ahy_m,&Vx_kalman,&Vy_kalman,&Biasx,&Biasy,&Vkalmanx_pred,&Vkalmany_pred,&Vxm_kalman,&Vym_kalman,&Axm_kalman,&Aym_kalman,&dt,&P00,&P11,&P22,&P33,&Q00,&Q11,&R00,&R11);

     // report new results
     computervision_thread_has_results++;
     
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;    
  }

void opticflow_module_start(void){
  
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

void initflowdata(FILE *fp) {
	fprintf(fp,"%s;%s;%s;%s;%s;%s;%s;%s;%s;\n","msg_id","Txp","Typ","histX","prevhistX","histY","prevhistY","errormapX","errormapY");
}
	
void saveflowdata(FILE *fp,int msg_id, int Txp,int Typ,unsigned int *histX,unsigned int *prevhistX,unsigned int *histY,unsigned int *prevhistY,float *errormapx,float *errormapy, unsigned int window) {
	int j;
	
	// msg id
	fprintf(fp, "%d;",msg_id);
	
	// flow results
	fprintf(fp, "%d;",Txp);
	fprintf(fp, "%d;",Typ);
	
	// hist x
	for(j=0; j<WIDTH; j++) 
	{
	  fprintf(fp, "%d;",histX[j]); 
	}
	
	// prev hist x
	for(j=0; j<WIDTH; j++) 
	{
	  fprintf(fp, "%d;",prevhistX[j]); 
	}

	
	// hist y
	for(j=0; j<HEIGHT; j++) 
	{
	  fprintf(fp, "%d;",histY[j]); 
	}
	
	// prev hist y
	for(j=0; j<HEIGHT; j++) 
	{
	  fprintf(fp, "%d;",prevhistY[j]); 
	}
	
	
	// error map x
	for(j=0; j<(2*(window)+1); j++) 
	{
	  fprintf(fp, "%f;",errormapx[j]); 
	}
	
	// error map y
	for(j=0; j<(2*(window)+1); j++) 
	{
	  fprintf(fp, "%f;",errormapy[j]); 
	}
	fprintf(fp,"\n");		
}


static float get_rc_roll_f(void) {
  int32_t roll = radio_control.values[RADIO_ROLL];
#if STABILIZATION_ATTITUDE_DEADBAND_A
  DeadBand(roll, STABILIZATION_ATTITUDE_DEADBAND_A);
  return roll * STABILIZATION_ATTITUDE_SP_MAX_PHI / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_A);
#else
  return roll * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ;
#endif
}

static float get_rc_pitch_f(void) {
  int32_t pitch = radio_control.values[RADIO_PITCH];
#if STABILIZATION_ATTITUDE_DEADBAND_E
  DeadBand(pitch, STABILIZATION_ATTITUDE_DEADBAND_E);
  return pitch * STABILIZATION_ATTITUDE_SP_MAX_THETA / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_E);
#else
  return pitch * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ;
#endif
}

static inline float get_rc_yaw_f(void) {
  int32_t yaw = radio_control.values[RADIO_YAW];
  DeadBand(yaw, STABILIZATION_ATTITUDE_DEADBAND_R);
  return yaw * STABILIZATION_ATTITUDE_SP_MAX_R / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_R);
}  
  



