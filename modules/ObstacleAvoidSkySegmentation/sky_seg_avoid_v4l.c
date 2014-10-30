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


// Own header
#include "sky_seg_avoid.h"

// UDP Message with GST vision
#include "udp/socket.h"
#include "video_message_structs.h"

// Navigate Based On Vision
#include "avoid_navigation.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude

// Threaded computer vision
#include <pthread.h>


struct UdpSocket *sock;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;
int sky_seg_avoid_vision_adjust_factor;


void sky_seg_avoid_init(void) {
  // Give unique ID's to messages TODO: check that received messages are correct (not from an incompatable gst plugin)
  ppz2gst.ID = 0x0003;
  gst2ppz.ID = 0x0004;
  sky_seg_avoid_vision_adjust_factor = 4;

  // Navigation Code
  init_avoid_navigation();
}


volatile uint8_t computervision_thread_has_results = 0;

void sky_seg_avoid_run(void) {

  // Send Attitude To GST Module
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();
  ppz2gst.counter++; // 512 Hz
  ppz2gst.roll = att->phi;
  ppz2gst.pitch = att->theta;
  ppz2gst.adjust_factor = sky_seg_avoid_vision_adjust_factor;



  // Read Latest GST Module Results
  if (computervision_thread_has_results)
  {
    computervision_thread_has_results = 0;
    run_avoid_navigation_onvision();
  }
  else
  {
    // Play annimation
    static uint8_t nr = 0;
    gst2ppz.obstacle_bins[nr] ++;
    nr ++;
    if (nr >= N_BINS)
      nr = 0;
  }
}

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/video.h"
#include "resize.h"

// Payload Code
#include "obstacleavoidskysegmentation_code.h"

#define DOWNLINK_VIDEO 1

#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

#include <stdio.h>

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  #define DOWNSIZE_FACTOR   8
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

#ifdef DOWNLINK_VIDEO

  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);

#endif

  // First Apply Settings before init
  imgWidth = small.w;
  imgHeight = small.h;
  verbose = 2;
  my_plugin_init();

  while (computer_vision_thread_command > 0)
  {
    video_grab_image(&vid, img_new);

    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);

    static uint8_t toggle = 1;
    toggle = 1-toggle;


    // Process Before sending
    if (toggle == 1)
      my_plugin_run(small.buf);

#ifdef DOWNLINK_VIDEO

    // JPEG encode the image:
    uint32_t quality_factor = 60; // quality factor from 1 (high quality) to 8 (low quality)
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
    uint32_t delta_t_per_frame = 0; // 0 = use drone clock
    send_rtp_frame(vsock, jpegbuf,size, small.w, small.h,0, quality_factor, dri_header, delta_t_per_frame);
#endif

    // Else process after sending raw
    if (toggle != 1)
      my_plugin_run(small.buf);

    computervision_thread_has_results++;
  }

  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void sky_seg_avoid_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void sky_seg_avoid_stop(void)
{
  computer_vision_thread_command = 0;
}



