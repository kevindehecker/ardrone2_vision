#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/shm.h>

#include "inc/dbapi.h"

#include "../cv/image.h"
#include "dsp.h"

UINT g_dwDSPWordSize = 1;	// default for 2430

static int shmidMalloc;
static unsigned char* bufferMalloc;
static PVOID dspAddrMalloc;

int DSPVisionOpen(struct VISION_TASK *task, struct img_struct* img2Dsp, struct img_struct* img2Mpu)
{
	int status = 0;

	DspManager_Open(status, NULL);

	task->hProcessor = NULL;
	task->hNode = NULL;

	if (DSP_SUCCEEDED(status)) 
	{
		/* Perform processor level initialization. */
		status = InitializeProcessor(task);
	}
	if (DSP_SUCCEEDED(status)) 
	{
		/* Perform node level initialization. */
		status = InitializeNode(task);
		if (DSP_SUCCEEDED(status)) 
		{
			//Initialize image buffers

			/* Actual MPU Buffer addresses */
			img2Dsp->buf = 0;
			img2Mpu->buf = 0;
			//task->bufferTemp = 0;

			/* Base addresses of reserved DSP virtual address ranges (bytes) */
			task->dspAddrSend = NULL;
			task->dspAddrRecv = NULL;
			task->dspAddrTemp = NULL;

			/* Reserved buffer DSP virtual addresses (bytes) */
			PVOID aDspSendBuffer = NULL;
			PVOID aDspRecvBuffer = NULL;
			PVOID aDspTempBuffer = NULL;
			PVOID aDspMallocBuffer = NULL;

			/* MPU buffer size */
			ULONG ulSendBufferSize = img2Dsp->w * img2Dsp->h * 2;
			ULONG ulRecvBufferSize = img2Mpu->w * img2Mpu->h * 2;

			/* DSP reserve size = buffersize + 4K page size */
			ULONG ulSendResv = ALIGN_4K(ulSendBufferSize) + 0x1000;
			ULONG ulRecvResv = ALIGN_4K(ulRecvBufferSize) + 0x1000;
			ULONG ulTempResv = ALIGN_4K(ulRecvBufferSize) + 0x1000;

			/* Messaging used for GPP/DSP synchronization */
			struct DSP_MSG msgToDsp;

			//Allocate MPU Buffers in the shared memory segment
			//Create Send segment
			key_t key = 42;
			task->shmidSend = shmget(key,ulSendBufferSize,IPC_CREAT|666);
			if(task->shmidSend==-1)
			{
				perror("shmget");
				exit(1);
			}

			//Attach segment
			img2Dsp->buf = shmat(task->shmidSend,NULL,0);
			if (img2Dsp->buf == (unsigned char *)(-1)) 
			{
				perror("shmat");
				exit(1);
			}
			printf("cpu2dsp: 0x%x\n",(unsigned int)(img2Dsp->buf));

			//Create receive segment
			key = 43;
			task->shmidRecv = shmget(key,ulRecvResv,IPC_CREAT|666);
			if(task->shmidRecv==-1)
			{
				perror("shmget");
				exit(1);
			}

			//Attach segment
			img2Mpu->buf = shmat(task->shmidRecv,NULL,0);
			if (img2Mpu->buf == (unsigned char *)(-1)) 
			{
				perror("shmat");
				exit(1);
			}
			printf("dsp2cpu: 0x%x\n",(unsigned int)(img2Mpu->buf));

			//Create temp segment
			key = 44;
			task->shmidTemp = shmget(key,ulTempResv,IPC_CREAT|666);
			if(task->shmidTemp==-1)
			{
				perror("shmget");
				exit(1);
			}

			//Attach segment
			task->bufferTemp = shmat(task->shmidTemp,NULL,0);
			if (task->bufferTemp == (unsigned char *)(-1)) 
			{
				perror("shmat");
				exit(1);
			}
			printf("temp: 0x%x\n",(unsigned int)(task->bufferTemp));
			
			//Create DSP MALLOC segment
			key = 45;
			shmidMalloc = shmget(key,DSP_MALLOC_BUFFER_SIZE,IPC_CREAT|666);
			if(shmidMalloc==-1)
			{
				perror("shmget");
				exit(1);
			}

			//Attach segment
			bufferMalloc = shmat(shmidMalloc,NULL,0);
			if (task->bufferTemp == (unsigned char *)(-1)) 
			{
				perror("shmat");
				exit(1);
			}
			printf("malloc: 0x%x\n",(unsigned int)(bufferMalloc));

			if (!img2Dsp->buf || !img2Mpu->buf || !task->bufferTemp) 
			{
				fprintf(stdout, "Failed to allocate MPU buffers.\n");
				status = -ENOMEM;
			} 
			else 
			{
				/* Reserve DSP virtual memory for send buffer */
				status = DSPProcessor_ReserveMemory(task->hProcessor, ulSendResv, &(task->dspAddrSend));
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout, "DSPProcessor_ReserveMemory succeeded. dspAddrSend = 0x%x \n",(UINT)(task->dspAddrSend));
				} else {
					fprintf(stdout, "DSPProcessor_ReserveMemory failed. Status = 0x%x\n", (UINT)status);
				}
			}

			if (DSP_SUCCEEDED(status)) {
				/* Reserve DSP virtual memory for receive buffer */
				status = DSPProcessor_ReserveMemory(task->hProcessor, ulRecvResv, &(task->dspAddrRecv));
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout,"DSPProcessor_ReserveMemory succeeded. dspAddrRecv = 0x%x \n",(UINT)(task->dspAddrRecv));
				} else {
					fprintf(stdout, "DSPProcessor_ReserveMemory failed. Status = 0x%x\n", (UINT)status);
				}
			}
			if (DSP_SUCCEEDED(status)) {
				// Reserve DSP virtual memory for temp buffer, same size as output buffer 
				status = DSPProcessor_ReserveMemory(task->hProcessor, ulTempResv, &(task->dspAddrTemp));
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout,"DSPProcessor_ReserveMemory succeeded. dspAddrTemp = 0x%x \n",(UINT)(task->dspAddrTemp));
				} else {
					fprintf(stdout, "DSPProcessor_ReserveMemory failed. Status = 0x%x\n", (UINT)status);
				}
			}
			
			if (DSP_SUCCEEDED(status)) {
				// Reserve DSP virtual memory for malloc buffer 
				status = DSPProcessor_ReserveMemory(task->hProcessor, DSP_MALLOC_BUFFER_SIZE, &dspAddrMalloc);
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout,"DSPProcessor_ReserveMemory succeeded. dspAddrMalloc = 0x%x \n",(UINT)(dspAddrMalloc));
				} else {
					fprintf(stdout, "DSPProcessor_ReserveMemory failed. Status = 0x%x\n", (UINT)status);
				}
			}
			
			if (DSP_SUCCEEDED(status)) {
				/* Map MPU "send buffer" to DSP "receive buffer" virtual address */
				status = DSPProcessor_Map(task->hProcessor, (PVOID)(img2Dsp->buf),  ulSendBufferSize, task->dspAddrRecv, &aDspRecvBuffer, 0);
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout, "DSPProcessor_Map succeeded.\n");
				} else {
					fprintf(stdout, "DSPProcessor_Map failed. Status = 0x%x\n",(UINT)status);
				}
			}
			if (DSP_SUCCEEDED(status)) {
				/* Map MPU "receive buffer" to DSP "send buffer" virtual address */
				status = DSPProcessor_Map(task->hProcessor, (PVOID)(img2Mpu->buf),  ulRecvBufferSize, task->dspAddrSend, &aDspSendBuffer,0);
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout, "DSPProcessor_Map succeeded.\n");
				} else {
					fprintf(stdout, "DSPProcessor_Map failed. Status = 0x%x\n",(UINT)status);
				}
			}
			if (DSP_SUCCEEDED(status)) {
				// Map MPU "temp buffer" to DSP "temp buffer" virtual address
				status = DSPProcessor_Map(task->hProcessor, (PVOID)(task->bufferTemp),  ulRecvBufferSize, task->dspAddrTemp, &aDspTempBuffer,0);
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout, "DSPProcessor_Map succeeded.\n");
				} else {
					fprintf(stdout, "DSPProcessor_Map failed. Status = 0x%x\n",(UINT)status);
				}
			}		
			
			if (DSP_SUCCEEDED(status)) {
			//Map dsp malloc buffer
				status = DSPProcessor_Map(task->hProcessor, (PVOID)(bufferMalloc),  DSP_MALLOC_BUFFER_SIZE, dspAddrMalloc, &aDspMallocBuffer,0);
				if (DSP_SUCCEEDED(status)) {
					fprintf(stdout, "DSPProcessor_Map succeeded.\n");
				} else {
					fprintf(stdout, "DSPProcessor_Map failed. Status = 0x%x\n",(UINT)status);
				}
			}
			
			if (DSP_SUCCEEDED(status)) {
				/* Notify DSP of word-adjusted buffer addresses */
				msgToDsp.dwCmd = VISION_SETUPBUFFERS;
				msgToDsp.dwArg1 = (DWORD)(aDspRecvBuffer) / g_dwDSPWordSize;
				msgToDsp.dwArg2 = (DWORD)(aDspSendBuffer) / g_dwDSPWordSize;
				status = DSPNode_PutMessage(task->hNode, &msgToDsp, DSP_FOREVER);
				fprintf(stdout, "Sending VISION BUFs to DSP cmd=SETUP, DspRecvBuf=0x%x, DspSendBuf=0x%x \n",(UINT)(aDspRecvBuffer), (UINT)(aDspSendBuffer));
				if (DSP_FAILED(status)) {
					fprintf(stdout, "DSPProcessor_PutMessage failed. Status = 0x%x\n", (UINT)status);
				}
			}

			if (DSP_SUCCEEDED(status)) {
				// Notify DSP of word-adjusted temp buffer addresses
				msgToDsp.dwCmd = VISION_SETUPTEMPBUFFER;
				msgToDsp.dwArg1 = (DWORD)(aDspTempBuffer) / g_dwDSPWordSize;
				status = DSPNode_PutMessage(task->hNode, &msgToDsp, DSP_FOREVER);
				fprintf(stdout, "Sending TEMP BUF to DSP cmd=SETUPTEMP, DspTempBuf=0x%x\n",(UINT)(task->dspAddrTemp));
				if (DSP_FAILED(status)) {
					fprintf(stdout, "DSPProcessor_PutMessage failed. Status = 0x%x\n", (UINT)status);
				}
			}
			
			if (DSP_SUCCEEDED(status)) {
				// Notify DSP of word-adjusted malloc buffer addresses
				msgToDsp.dwCmd = VISION_SETUPMALLOCBUFFER;
				msgToDsp.dwArg1 = (DWORD)(aDspMallocBuffer) / g_dwDSPWordSize;
				msgToDsp.dwArg2 = DSP_MALLOC_BUFFER_SIZE;
				status = DSPNode_PutMessage(task->hNode, &msgToDsp, DSP_FOREVER);
				fprintf(stdout, "Sending MALLOC BUF to DSP cmd=SETUPMALLOC, DspMallocBuf=0x%x\n",(UINT)(dspAddrMalloc));
				if (DSP_FAILED(status)) {
					fprintf(stdout, "DSPProcessor_PutMessage failed. Status = 0x%x\n", (UINT)status);
				}
			}

			//Save DSP Virtual addresses for unmapping
			task->dspAddrRecv = aDspRecvBuffer;
			task->dspAddrSend = aDspSendBuffer;
			task->dspAddrTemp = aDspTempBuffer;
			dspAddrMalloc = aDspMallocBuffer;

			//Save shared memory addresses
			task->bufferSend = img2Dsp->buf;
			task->bufferReceive = img2Mpu->buf;

			DSPProcessor_FlushMemory(task->hProcessor,(PVOID)(img2Dsp->buf), ulSendBufferSize,0);
			DSPProcessor_FlushMemory(task->hProcessor,(PVOID)(img2Mpu->buf), ulRecvBufferSize,0);
			DSPProcessor_FlushMemory(task->hProcessor,(PVOID)(task->bufferTemp), ulRecvBufferSize,0);

			//Send images dimensions to dsp
			//printf("sizeof(int) = %d\n", sizeof(int));
			if (DSP_SUCCEEDED(status)) {
				msgToDsp.dwCmd = VISION_SETUPIMAGESIZE;
				
				*((unsigned int*)(&img2Dsp->buf[0])) = img2Dsp->w;
				*((unsigned int*)(&img2Dsp->buf[4])) = img2Dsp->h;
				*((unsigned int*)(&img2Dsp->buf[8])) = img2Mpu->w;
				*((unsigned int*)(&img2Dsp->buf[12])) = img2Mpu->h;

				DSPProcessor_FlushMemory(task->hProcessor, (PVOID)(img2Dsp->buf),16,0);
				status = DSPNode_PutMessage(task->hNode, &msgToDsp, DSP_FOREVER);
				fprintf(stdout, "Sending img2Dsp dimensions to DSP cmd=INPUTIMAGESIZE, w=%d, h=%d \n",img2Dsp->w, img2Dsp->h);
				if (DSP_FAILED(status)) {
					fprintf(stdout, "DSPProcessor_PutMessage failed. Status = 0x%x\n", (UINT)status);
				}
			}
		}
	}
	return (DSP_SUCCEEDED(status) ? 0 : -1);
}

void DSPVisionClose(struct VISION_TASK *task)
{
	/* Unmap DSP Recv buffer from MPU receive buffer */
	int status = DSPProcessor_UnMap(task->hProcessor, (PVOID)(task->dspAddrRecv));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnMap succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnMap failed. Status = 0x%x\n", (UINT)status);
	}
	/* Upmap DSP Send buffer from MPU receive buffer */
	status = DSPProcessor_UnMap(task->hProcessor,(PVOID)(task->dspAddrSend));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnMap succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnMap failed. Status = 0x%x\n", (UINT)status);
	}
	// Upmap DSP Temp buffer from MPU receive buffer
	status = DSPProcessor_UnMap(task->hProcessor,(PVOID)(task->dspAddrTemp));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnMap succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnMap failed. Status = 0x%x\n", (UINT)status);
	}
	// Upmap DSP Malloc buffer
	status = DSPProcessor_UnMap(task->hProcessor,(PVOID)(dspAddrMalloc));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnMap succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnMap failed. Status = 0x%x\n", (UINT)status);
	}
	/* Unreserve DSP virtual memory */
	status = DSPProcessor_UnReserveMemory(task->hProcessor, (PVOID)(task->dspAddrSend));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnReserveMemory succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnReserveMemory failed.  Status = 0x%x\n", (UINT)status);
	}
	/* Unreserve DSP virtual memory */
	status = DSPProcessor_UnReserveMemory(task->hProcessor,  (PVOID)(task->dspAddrRecv));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnReserveMemory succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnReserveMemory failed. Status = 0x%x\n", (UINT)status);
	}
	// Unreserve DSP virtual memory
	status = DSPProcessor_UnReserveMemory(task->hProcessor,  (PVOID)(task->dspAddrTemp));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnReserveMemory succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnReserveMemory failed. Status = 0x%x\n", (UINT)status);
	}
	// Unreserve DSP virtual memory
	status = DSPProcessor_UnReserveMemory(task->hProcessor,  (PVOID)(dspAddrMalloc));
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, "DSPProcessor_UnReserveMemory succeeded.\n");
	} else {
		fprintf(stdout, "DSPProcessor_UnReserveMemory failed. Status = 0x%x\n", (UINT)status);
	}

	/* Free MPU buffers */
	shmdt(task->bufferSend);
	shmdt(task->bufferReceive);
	shmdt(task->bufferTemp);
	shmdt(bufferMalloc);
	//Delete shared memory segments
	shmctl(task->shmidRecv, IPC_RMID, NULL);
	shmctl(task->shmidRecv, IPC_RMID, NULL);
	shmctl(task->shmidTemp, IPC_RMID, NULL);
	shmctl(shmidMalloc, IPC_RMID, NULL);

	CleanupNode(task);
	CleanupProcessor(task);
	DspManager_Close(0, NULL);
}

/*
 *  ======== InitializeProcessor ========
 *  Perform processor related initialization.
 */
int InitializeProcessor(struct VISION_TASK *task)
{
	int status = -EPERM;
	struct DSP_PROCESSORINFO dspInfo;
	UINT numProcs;
	UINT indexDSP = 0;
	INT procId = 0;
	/* Attach to DSP */
	while (DSP_SUCCEEDED(DSPManager_EnumProcessorInfo(indexDSP,&dspInfo, (UINT)sizeof(struct DSP_PROCESSORINFO),&numProcs))) {
		if ((dspInfo.uProcessorType == DSPTYPE_55) ||  (dspInfo.uProcessorType == DSPTYPE_64)) {
			if (dspInfo.uProcessorType == DSPTYPE_55) {
				g_dwDSPWordSize = 2;
			} else {
				g_dwDSPWordSize = 1;
			}
			printf("DSP device detected !! \n");
			procId = indexDSP;
			status = 0;
			break;
		}
		indexDSP++;
	}
	/* Attach to an available DSP (in this case, the 1st DSP). */
	if (DSP_SUCCEEDED(status)) {
		status = DSPProcessor_Attach(procId, NULL,&task->hProcessor);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPProcessor_Attach succeeded.\n");
		} else {
			fprintf(stdout, "DSPProcessor_Attach failed. Status = 0x%x\n", (UINT)status);
		}
	} else
		fprintf(stdout, "Failed to get the desired processor \n");
#ifdef PROC_LOAD
	if (DSP_SUCCEEDED(status)) {
		fprintf(stdout, " DSP Image: %s.\n", argv);
		status = DSPProcessor_Load(task->hProcessor, argc, &argv, NULL);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPProcessor_Load succeeded.\n");
		} else {
			fprintf(stdout, "DSPProcessor_Load failed.\n");
		}
		status = DSPProcessor_Start(task->hProcessor);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPProcessor_Start succeeded.\n");
		} else {
			fprintf(stdout, "DSPProcessor_Start failed.\n");
		}
	}
#endif				/* PROC_LOAD */
	return (status);
}

/*
 *  ======== InitializeNode ========
 *  Perform node related initialization.
 */
int InitializeNode(struct VISION_TASK *task)
{
	BYTE argsBuf[ARGSIZE + sizeof(ULONG)];
	struct DSP_CBDATA *pArgs;
	struct DSP_NODEATTRIN nodeAttrIn;
	struct DSP_UUID uuid;
	int status = 0;

	uuid = VISION_TI_uuid;
	nodeAttrIn.uTimeout = DSP_FOREVER;
	nodeAttrIn.iPriority = 5;
	pArgs = (struct DSP_CBDATA *)argsBuf;
	pArgs->cbData = ARGSIZE;
	/* Allocate the VISION node. */
	if (DSP_SUCCEEDED(status)) {
		status = DSPNode_Allocate(task->hProcessor, &uuid, pArgs, &nodeAttrIn, &task->hNode);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPNode_Allocate succeeded.\n");
		} else {
			fprintf(stdout,"DSPNode_Allocate failed. Status = 0x%x\n", (UINT)status);
		}
	}
	/* Create the VISION node on the DSP. */
	if (DSP_SUCCEEDED(status)) {
		status = DSPNode_Create(task->hNode);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPNode_Create succeeded.\n");
		} else {
			fprintf(stdout, "DSPNode_Create failed. Status = 0x%x\n", (UINT)status);
		}
	}
	/* Start the VISION node on the DSP. */
	if (DSP_SUCCEEDED(status)) {
		status = DSPNode_Run(task->hNode);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPNode_Run succeeded.\n");
		} else {
			fprintf(stdout, "DSPNode_Run failed. Status = 0x%x\n",(UINT)status);
		}
	}
	return (status);
}

/*
 *  ======== CleanupNode ========
 *  Perform node related cleanup.
 */
int CleanupNode(struct VISION_TASK *task)
{
	int exitStatus;
	int status = 0;

	if (task->hNode) {
		/* Terminate DSP node */
		status = DSPNode_Terminate(task->hNode, &exitStatus);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPNode_Terminate succeeded.\n");
		} else {
			fprintf(stdout, "DSPNode_Terminate failed: 0x%x\n", (UINT)status);
		}
		/* Delete DSP node */
		status = DSPNode_Delete(task->hNode);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPNode_Delete succeeded.\n");
		} else {
			fprintf(stdout, "DSPNode_Delete failed: 0x%x\n", (UINT)status);
		}
		task->hNode = NULL;
	}
	return (status);
}

/*
 *  ======== CleanupProcessor ========
 *  Perform processor related cleanup.
 */
int CleanupProcessor(struct VISION_TASK *task)
{
	int status = 0;

	if (task->hProcessor) {
		/* Detach from processor. */
		status = DSPProcessor_Detach(task->hProcessor);
		if (DSP_SUCCEEDED(status)) {
			fprintf(stdout, "DSPProcessor_Detach succeeded.\n");
		} else {
			fprintf(stdout, "DSPProcessor_Detach failed.\n");
		}
		task->hProcessor = NULL;
	}

	return (status);
}


