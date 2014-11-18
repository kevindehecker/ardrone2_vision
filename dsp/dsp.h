#ifndef _DSP_H_
#define _DSP_H_

#include "../cv/image.h"

//Application sepcific attributes
#define ARGSIZE             32

#define ALIGN_4K(truc)	((truc%4096 == 0)?truc:(truc+(4096 - truc%4096)))

/* Messaging DSP commands */
#define VISION_SETUPBUFFERS    	0xABCD
#define VISION_SETUPTEMPBUFFER    0xBCDE
#define VISION_SETUPMALLOCBUFFER    0xBCDF
#define VISION_WRITEREADY      0xADDD
#define VISION_SETUPIMAGESIZE		0xABCE

//We have to reserve space in the memory for DSP dynamic allocations
#define DSP_MALLOC_BUFFER_SIZE		0x80000	//512ko

extern UINT g_dwDSPWordSize;

/* VISION task context data structure. */
struct VISION_TASK {
	DSP_HPROCESSOR hProcessor;	/* Handle to processor. */
	DSP_HNODE hNode;	/* Handle to node. */
	unsigned char* bufferSend;	//To DSP
	unsigned char* bufferReceive;	//From DSP
	unsigned char* bufferTemp;
	PVOID dspAddrSend;
	PVOID dspAddrRecv;
	PVOID dspAddrTemp;
	int shmidSend;		//Shared memory identifier
	int shmidRecv;
	int shmidTemp;
};

/* VISION_TI_uuid = 38BA464F_9C3E_484E_990F_48305B183848 */
static const struct DSP_UUID VISION_TI_uuid = {
	0x38ba464f, 0x9c3e, 0x484e, 0x99, 0x0f, { 0x48, 0x30, 0x5b, 0x18, 0x38,0x48}};

int DSPVisionOpen(struct VISION_TASK *task, struct img_struct* img2Dsp, struct img_struct* img2Mpu);
void DSPVisionClose(struct VISION_TASK *task);

/* DSP Initialization and cleanup routines. */
int InitializeProcessor(struct VISION_TASK *copyTask);
int InitializeNode(struct VISION_TASK *copyTask);
int InitializeStreams(struct VISION_TASK *copyTask);
int CleanupProcessor(struct VISION_TASK *copyTask);
int CleanupNode(struct VISION_TASK *copyTask);
int CleanupStreams(struct VISION_TASK *copyTask);
#endif

