
// Kalman Filter
float P[16];
float Q[4];
float R[4];
float Vkalmanx;
float Vkalmany;
float Biasx;
float Biasy;
float Vkalmanx_pred;
float Vkalmany_pred;

void KalmanOpticFlow_reset(void);
void KalmanOpticFlow_run(void);
void KalmanOpticFlow_set_P(float p1, float p2, float p3, float p4);
void KalmanOpticFlow_set_Q(float q1, float q2);
void KalmanOpticFlow_set_R(float r1, float r2);
void UpdateKalman(float Vmeasx, float Vmeasy, float Ameasx, float Ameasy, float dt);
