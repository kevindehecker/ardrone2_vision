/*
 * Author: Maxime
 * Edited by Wim, November 2014
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "image.h"
#include "opticflow_sobelfilter.h"
#include "opticflow_module.h"

#define LONG_MAX	2147483647
#define GNOISE 		15 // noise gradient threshold

// imgIn is in U-Y-V-Y format (exactly that order)
// http://en.wikipedia.org/wiki/YUV

void ApplySobelFilter(struct img_struct* imgIn, unsigned int *histX, unsigned int *histY)
// http://en.wikipedia.org/wiki/Sobel_operator
// - Gtot = sqrt(Gx^2 and Gy^2) instead of Gtot = Gx^2 + Gy^2
// - furthermore: only return Gtot if it is above the NOISE threshold (set as define GNOISE)
// - no return variable

{


	int i,j;
	int Gx,Gy;
	long Gtot;

	unsigned short widthStep;
	unsigned char* imageData;
		
	widthStep = 2*WIDTH;
	imageData = imgIn->buf;
	
	unsigned char *img = imageData+1;

	unsigned char *in =(img + widthStep); 		// "current" line of the image
	unsigned char *inPrevLine = img; 		// previous line of the image
	unsigned char *inNextLine = (img + 2*widthStep);// next line of the image

	unsigned int *histSobelX = histX+1; // initialise the position of the histogram X and Y
	unsigned int *histSobelY = histY+1;
	
	memset(histX,0,WIDTH*sizeof(unsigned int)); // initialise histX with 0
	memset(histY,0,HEIGHT*sizeof(unsigned int));

	// for all lines in the image (start with 1)
	for(j=1;j<HEIGHT-1;j++) //for(j=1;j<height-1;j++) 
	{
		histSobelX = histX+1; // initialise the position of the histogram X
		
		// over the entire width of the image
		for(i=1;i<WIDTH-1;i++)
		{
		    
		    Gx = 0;
		    Gy = 0;
		    // convolution filter
		    // inPrevline
		    Gx += Gx - 1 * (*inPrevLine);
		    Gy += Gy - 1 * (*inPrevLine);
		    
		    // inPrevLine +1
		    inPrevLine+=2;
		    Gx = Gx + 0 * (*inPrevLine);
		    Gy = Gy - 2 * (*inPrevLine);  
		    
		    // inPrevLine +2
		    inPrevLine+=2;
		    Gx = Gx + 1 * (*inPrevLine);
		    Gy = Gy - 1 * (*inPrevLine);  
		    
		    // in
		    Gx = Gx - 2 * (*in);
		    Gy = Gy + 0 * (*in);
		    
		    // in +1 
		    in+=2;
		    Gx = Gx + 0 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    // in + 2
		    in+=2;
		    Gx = Gx + 2 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    
		    // in Nextline
		    Gx = Gx - 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    // in Nextline +1
		    inNextLine+=2;
		    Gx = Gx + 0 * (*inNextLine);
		    Gy = Gy + 2 * (*inNextLine);
		      
		    // in Nextline +2
		    inNextLine+=2;
		    Gx = Gx + 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    Gtot = Gx*Gx + Gy*Gy;
		    Gtot = sqrt(Gtot);
		    

		    if(Gtot < GNOISE) {
		      Gtot=0;
		    }
		    
		    // add gradient tot histograms
		    *histSobelX += Gtot; 
		    *histSobelY += Gtot;
	    
		    histSobelX++; // move to next entry of the x histogram
		    
		    // Move to next pixel
		    inPrevLine-=2;
		    in-=2;
		    inNextLine-=2;
		 
		}
		in += 4;
		inPrevLine += 4;
		inNextLine += 4;
		histSobelY++; // move to the next entry of the x histogram
		
	}
	// result is 2 histograms with over its elements a sum of the convolution

}

