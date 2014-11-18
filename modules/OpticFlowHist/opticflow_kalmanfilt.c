#include "opticflow_kalmanfilt.h"
#include "matrixcalc.h"

void KalmanOpticFlow_reset(void) {
  
  // Kalman Filter
  float P_initial = 10;
  float Q_initial = 100; 		// accelerometer noise
  float R_initial = 0.01;		// optic flow velocity noise
  KalmanOpticFlow_set_P(P_initial,P_initial,P_initial,P_initial);
  KalmanOpticFlow_set_Q(Q_initial,Q_initial);
  KalmanOpticFlow_set_R(R_initial,R_initial);
  
  // initial optimal state  
  Vkalmanx = 0;
  Vkalmany = 0;
  Biasx = 0;
  Biasy = 0;
}

void KalmanOpticFlow_set_P(float p1, float p2, float p3, float p4) {
  
  P[0]=p1;	P[1]=0;		P[2]=0;		P[3]=0;
  P[4]=0;	P[5]=p2;	P[6]=0;		P[7]=0;
  P[8]=0;	P[9]=0;		P[10]=p3;	P[11]=0;
  P[12]=0;	P[13]=0;	P[14]=0;	P[15]=p4;
  
}

void KalmanOpticFlow_set_Q(float q1, float q2) {
  // acceleration noise
  Q[0]=q1;	Q[1]=0;			
  Q[2]=0;	Q[3]=q2;		
}
  

void KalmanOpticFlow_set_R(float r1, float r2) {
  // acceleration noise
  R[0]=r1;	R[1]=0;			
  R[2]=0;	R[3]=r2;		
}

void UpdateKalman(float Vmeasx, float Vmeasy, float Ameasx, float Ameasy, float dt) {


      // measurement outputs
      float y_meas[2];
      y_meas[0] = Vmeasx;
      y_meas[1] = Vmeasy;

      
      // current optimal state estimation
      float x_opt[4];
      x_opt[0] = Vkalmanx;
      x_opt[1] = Vkalmany;
      x_opt[2] = Biasx;
      x_opt[3] = Biasy;

      float Phi[16] =   {1,	0,	-dt,	0,
		      0,	1,	0,	-dt,
		      0,	0,	1,	0,
		      0,	0,	0,	1};     
      float Gamma[8] = {dt,	0,
		      0,	dt,
		      0,	0,
		      0,	0};	   
		    
      float K[8];

      float Hx[8] = { 1,	0,	0,	0,
		      0,	1,	0,	0};

      // --------------------------
      // Prediction
      // --------------------------
      float x_pred[4];
      float y_pred[2];  

      x_pred[0] = x_opt[0] + (Ameasx - x_opt[2])*dt;
      x_pred[1] = x_opt[1] + (Ameasy - x_opt[3])*dt;		
      x_pred[2] = x_opt[2];
      x_pred[3] = x_opt[3];

      y_pred[0] = x_pred[0];
      y_pred[1] = x_pred[1];

      // PrintMat("x_pred",x_pred,4,1);
      // PrintMat("y_pred",y_pred,2,1);

      // --------------------------
      // Covariance Update
      // --------------------------

      float P1[16];
      float Phi_P_Phi[16];
      float Gam_Q_Gam[16];
      float PhiT[16];
      float GammaT[8];
      float P_Phi[16];
      float Q_Gam[8];

      // PrintMat("P",P,4,4);
      // PrintMat("Phi",Phi,4,4);
      // Phi*P*Phi
      TranspMat(PhiT,Phi,4,4);
      MultMat(P_Phi,P,PhiT,4,4,4,4);
      MultMat(Phi_P_Phi,Phi,P_Phi,4,4,4,4);

      // Gamma*Q*Gamma
      // PrintMat("Q",Q,2,2);
      // PrintMat("Gamma",Gamma,4,2);
      TranspMat(GammaT,Gamma,4,2);
      // PrintMat("GammaT",GammaT,2,4);
      MultMat(Q_Gam,Q,GammaT,2,2,2,4);
      // PrintMat("Q_Gam",Q_Gam,2,4);
      MultMat(Gam_Q_Gam,Gamma,Q_Gam,4,2,2,4);
      // PrintMat("Gam_Q_Gam",Gam_Q_Gam,4,4);

      // P = Phi_P_Phi + Gam_Q_Gam
      AddMat(P1,Phi_P_Phi,Gam_Q_Gam,4,4);
      CpyMat(P,P1,4,4);

      // PrintMat("P",P,4,4);

      // --------------------------
      // Kalman Gain
      // --------------------------
      float HxT[8];
      float P_HxT[8];
      float Hx_P_HxT[4];
      float B[4];
      float invB[4];

      // K = P*HxT / (Hx*P*HxT + R) 
      //   = P*HxT / B
      // PrintMat("Hx",Hx,2,4);
      // PrintMat("R",R,4,2);
      TranspMat(HxT,Hx,2,4);
      MultMat(P_HxT,P,HxT,4,4,4,2);
      // PrintMat("P_HxT",P_HxT,4,2);
      MultMat(Hx_P_HxT,Hx,P_HxT,2,4,4,2);
      // PrintMat("Hx_P_HxT",Hx_P_HxT,2,2);
      AddMat(B,Hx_P_HxT,R,2,2);
      // PrintMat("B",B,2,2);
      InvMat22(invB,B);
      // PrintMat("invB",invB,2,2);

      MultMat(K,P_HxT,invB,4,2,2,2);

      // PrintMat("K",K,4,2);

      // --------------------------
      // New Optimal state
      // --------------------------

      // state_opt = state_pred + K*(y_meas - y_pred);
      float err_pred[2];
      SubtMat(err_pred,y_meas,y_pred,2,1);
      // PrintMat("y_meas",y_meas,2,1);
      // PrintMat("y_pred",y_pred,2,1);
      // PrintMat("err_pred",err_pred,2,1);

      float K_err_pred[4];
      MultMat(K_err_pred,K,err_pred,4,2,2,1);


      AddMat(x_opt,x_pred,K_err_pred,4,1);

      // x_opt[0] = x_pred[0] + K[0]*(y_meas[0]-y_pred[0]) + K[1]*(y_meas[1]-y_pred[1]);
      // x_opt[1] = x_pred[1] + K[2]*(y_meas[0]-y_pred[0]) + K[3]*(y_meas[1]-y_pred[1]);
      // x_opt[2] = x_pred[2] + K[4]*(y_meas[0]-y_pred[0]) + K[5]*(y_meas[1]-y_pred[1]);
      // x_opt[3] = x_pred[3] + K[6]*(y_meas[0]-y_pred[0]) + K[7]*(y_meas[1]-y_pred[1]);

      // PrintMat("x_opt",x_opt,4,1);

      // --------------------------
      // New Covariance
      // --------------------------

      float K_Hx[16];
      float I_min_K_Hx[16];

      MultMat(K_Hx,K,Hx,4,2,2,4);
      // PrintMat("K_Hx",K_Hx,4,4);

      I_min_K_Hx[0] = 1 - K_Hx[0];
      I_min_K_Hx[1] =   - K_Hx[1];
      I_min_K_Hx[2] =   - K_Hx[2];
      I_min_K_Hx[3] =   - K_Hx[3];

      I_min_K_Hx[4] =   - K_Hx[4];
      I_min_K_Hx[5] = 1 - K_Hx[5];
      I_min_K_Hx[6] =   - K_Hx[6];
      I_min_K_Hx[7] =   - K_Hx[7];

      I_min_K_Hx[8] =   - K_Hx[8];
      I_min_K_Hx[9] =   - K_Hx[9];
      I_min_K_Hx[10] = 1 - K_Hx[10];
      I_min_K_Hx[11] =   - K_Hx[11];

      I_min_K_Hx[12] =   - K_Hx[12];
      I_min_K_Hx[13] =   - K_Hx[13];
      I_min_K_Hx[14] =   - K_Hx[14];
      I_min_K_Hx[15] = 1 - K_Hx[15];

      MultMat(P1,I_min_K_Hx,P,4,4,4,4);
      // PrintMat("P1",P1,4,4);
      CpyMat(P,P1,4,4);
      // PrintMat("P",P,4,4);

      // Outputs:
      Vkalmanx 	= x_opt[0];
      Vkalmany 	= x_opt[1];
      Biasx 		= x_opt[2];
      Biasy 		= x_opt[3];

      Vkalmanx_pred 	= x_pred[0];
      Vkalmany_pred 	= x_pred[1];

}

  