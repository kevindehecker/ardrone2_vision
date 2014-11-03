/*
 * Author: Maxime
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "calcflow.h"
#include "opticflow_module.h"



unsigned int histTempX[320];
//extern unsigned int *histTempX;
unsigned int histTempY[240];
//extern unsigned int *histTempY;

int prevTx = 0;
int prevTy = 0;
int prevTz = 0;

float errorHists(unsigned int *hist1, unsigned int *hist2, unsigned int start, unsigned int end)
{
	int j;
	float erreur = 0;

	
	
	for(j=start;j<end;j++)
	{
		erreur += (hist1[j]-hist2[j])*(hist1[j]-hist2[j]);

	}
	//erreur *= 512; // multiply with 512.. why?
	
	// finally, divide error by number of columns that are tested
	erreur = erreur/(end-start);
	
	
	return erreur; 
}

unsigned int calcFlow(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *histY, unsigned int *prevHistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window)
{
// x,y in image plane!! image plane is rotated 90 degrees with respect to body frame: x_body = y_image, y_body =  -x_image
// in this function, the flow Tx,Ty,Tz is in percent, so Tx [px percent] = 100*flow[px];  
  
    int Tx=0,Ty=0,Tz=0;
    //int Tz_inf,Tz_sup; // unused
    int TyPrev=-255;
    short start=-1,end=0;
    float erreur=0,erreurX=0,erreurY=0;
    unsigned int shorterror;
    float erreurvec[25];
    memset(erreurvec,999,25*sizeof(unsigned long));
    
    float min=99999999999; // 99 miljard
    short first=1;
    int idx_shifted;
    short i;
    
        
    int wdw_pct = *window*FLOWFACT;
    
      // try for all possible Tx,Ty,Tz around Tz=0,Tx=prevTx,Ty=prevTy with a window defined by *window in [px]
      for(Tz=0;Tz<=0;Tz++)
      {
	
	  for(Ty=0-wdw_pct;Ty<=0+wdw_pct;Ty+=FLOWFACT)
	  {
		  for(Tx=0-wdw_pct;Tx<=0+wdw_pct;Tx+=FLOWFACT)
		  {
		    
				  // set histTempX to zero
				  start = -1;
				  memset(histTempX,0,WIDTH*sizeof(unsigned int));
				  for(i=-HALFWIDTH;i<HALFWIDTH;i++)
				  {
					  //idxGros = i*(FLOWFACT+Tz)-Tx;				  
					  //idx = idxGros/FLOWFACT + HALFWIDTH;
					  idx_shifted = HALFWIDTH+i*(1+Tz/FLOWFACT)-Tx/FLOWFACT;
					  if(idx_shifted>=0 && idx_shifted<WIDTH)
					  {
						  if(start==-1)
							  start = idx_shifted;
						  end = idx_shifted;
						  
						  // on Temporary Hist X, fill it with the correct values of previous Hist X on the correct locations (x-shift in px)
						  histTempX[HALFWIDTH+i] = prevHistX[idx_shifted];
					  }
				  }
				  
				  // Calculate, with the given Tx,Ty,Tz, the error with the current Hist
				  erreurX = errorHists(histTempX,histX,start,end+1);

			  if(TyPrev != Ty)
			  {
					  start = -1;
					  memset(histTempY,0,HEIGHT*sizeof(unsigned int));
					  for(i=-HALFHEIGHT;i<HALFHEIGHT;i++)
					  {
						  //idxGros = i*(100+Tz)-Ty;
						  //idx = idxGros/100 + 120;
						  idx_shifted = HALFHEIGHT+i*(1+Tz/FLOWFACT)-Ty/FLOWFACT;
						  if(idx_shifted>=0 && idx_shifted<HEIGHT)
						  {
							  if(start==-1)
								  start = idx_shifted;
							  end = idx_shifted;
							  histTempY[HALFHEIGHT+i] = prevHistY[idx_shifted];
						  }
					  }
					  erreurY = errorHists(histTempY,histY,start,end+1);
					  TyPrev = Ty;
			  }

 				  erreur = erreurX + erreurY;
// 				  erreurvec[25*k+j] = erreur;
			  
				  if(first)
				  {
					  min = erreur;
					  *Tx_min = Tx;
					  *Ty_min = Ty;
					  *Tz_min = Tz;
					  first = 0;
				  }
				  else if(erreur<min)
				  {
					  

					  min = erreur;
					  *Tx_min = Tx;
					  *Ty_min = Ty;
					  *Tz_min = Tz;
				  }
		  }
	    }
	}      
    

   
    shorterror = min;
    return shorterror;	
}


unsigned int calcFlow2(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *histY, unsigned int *prevHistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window, float *errormapx, float *errormapy) {
  

  unsigned int errorX = calcFlow_single( Tx_min, histX, prevHistX, window, WIDTH, errormapx);
  unsigned int errorY = calcFlow_single( Ty_min, histY, prevHistY, window, HEIGHT, errormapy);
  unsigned int shorterror= errorX + errorY;
  
  return shorterror;
}


unsigned int calcFlow_single(int *T_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *window, unsigned int width, float *errormap)
{
// calculate flow x and y independently
  
    int T=0;
    //int Tz_inf,Tz_sup; // unused
    short start=-1,end=0;
    float error=0;
    unsigned int shorterror;
    
    
    int halfwidth = width/2;
    
    float min=99999999999; // 99 miljard
    short first=1;
    int idx_shifted;
    short i;
    
    int wdw_pct = *window*FLOWFACT;
    
    unsigned int histTemp[width];
    
    // declare and initialise error map
    memset(errormap,0,(2*(*window)+1)*sizeof(float));
    int j = 0;
    
      // try for all possible Tx,Ty,Tz around Tz=0,Tx=prevTx,Ty=prevTy with a window defined by *window in [px]
      for(T=0-wdw_pct;T<=0+wdw_pct;T+=FLOWFACT)
      {
	
	  // set histTempX to zero
	  start = -1;
	  memset(histTemp,0,width*sizeof(unsigned int));
	  for(i=-halfwidth;i<halfwidth;i++)
	  {
		  //idxGros = i*(FLOWFACT+Tz)-Tx;				  
		  //idx = idxGros/FLOWFACT + HALFWIDTH;
		  idx_shifted = i+halfwidth-T/FLOWFACT;
		  
		  if(idx_shifted>=0 && idx_shifted<width)
		  {
			  
			  if(start==-1)
				  start = idx_shifted;	
			  end = idx_shifted;
			  
			  // on Temporary Hist X, fill it with the correct values of previous Hist X on the correct locations (x-shift in px)
			  histTemp[halfwidth+i] = prevHistX[idx_shifted];
		  }
	  }

	  // Calculate, with the given Tx,Ty,Tz, the error with the current Hist
	  
	  error = errorHists(histTemp,histX,start,end+1);
	  errormap[j] = error;
	  j++;
	  	  
	  if(first)
	  {
		  min = error;
		  *T_min = T;
		  first = 0;
	  }
	  else if(error<min)
	  {
		  min = error;
		  *T_min = T;
	  }
	 
	  
      }  
    shorterror = min;
    return shorterror;	
}