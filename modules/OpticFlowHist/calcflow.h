/*
 * Author: Maxime, Wim
 */
float errorhists(unsigned int *hist1, unsigned int *hist2, unsigned int start, unsigned int end);
unsigned int calcFlow(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevhistX, unsigned int *histY, unsigned int *prevhistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window);
unsigned int calcFlow_f(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevhistX, unsigned int *histY, unsigned int *prevhistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window);

unsigned int calcFlow2(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevhistX, unsigned int *histY, unsigned int *prevhistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window, float *errormapx, float *errormapy);
unsigned int calcFlow_single(int *Tx_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *window, unsigned int width, float *errormap);