#include "FTCardLib.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char const *argv[]) {

	unsigned int dw;

	//step 1: open device
	if (dw = FT_OpenDevice(), dw != FT_STATUS_SUCCESS) {
		printf("Failed to open device (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to open device\n");

	//step 2: set parameters
	dw = FT_SetParam(FT_PARAMID_FIBRECHL, 1);
	dw = FT_SetParam(FT_PARAMID_TRIGGER, 0);
	dw = FT_SetParam(FT_PARAMID_LEAD,1);
	if (dw != FT_STATUS_SUCCESS) {
		printf("Failed to set fibre channel 1 (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to set parameters (fibre channel 1, inner trigger, send lead)\n");

    //step 3: send control command via synchronized serial port
	unsigned short cmd = 0x7f00;
	if (dw = FT_SendSynData(2,FT_SYNFREQ_1M,&cmd,16), dw != FT_STATUS_SUCCESS) {
		printf("Failed to send synchronization data (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to send mode 1 switch matrix command\n");

    //step 4: start task
	if (dw = FT_StartTask(), dw != FT_STATUS_SUCCESS) {
		printf("Failed to start task (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to start task\n");

    //step 5: send fibre data
	unsigned short pData[2048] = {0};//prepare
	pData[0] = 0x7fff; pData[1] = 0xbc1c; pData[2] = 0xaaaa; pData[3] = 0xbbbb; pData[4] = 0x3c1c; pData[5] = 0x7fff; //add header
	for(int i=6;i<2048-6;i++) {
		pData[i] = 1024-(i-6)%1024;//fill content
    }
    for(int index=0;index<6;index++) {//set tail
        pData[2048-1-index] = pData[index];
    }
	if (dw = FT_SendFibreData(pData,4096), dw != FT_STATUS_SUCCESS) {
		printf("Failed to send fibre data (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to send 4K data\n");

    //step 6: receive fibre data
	unsigned char buffer[16*1024]; int remaining = 16*1024; int cnt = 0;
	while((remaining>0)&&(cnt<200)) {
		if(FT_GetPendingFibreSize()>0) {
			printf("Receive %u fibre data\n",FT_GetPendingFibreSize());
			remaining -= FT_ReceiveFibreData(buffer+16*1024-remaining,remaining);
		}
		cnt++;
		usleep(1000);
	}
	printf("Received total %u fibre data\n",16*1024-remaining);

    //step 7: receive synchronized serial port data
	unsigned char synRecv[512]; remaining = 512; cnt = 0;
	while((remaining>0)&&(cnt<200)) {
		if(FT_GetPendingSynSize()>0) {
			printf("Receive %u synchronization data\n",FT_GetPendingSynSize());
			remaining -= FT_ReceiveSynData(synRecv,remaining);
		}
		cnt++;
		sleep(5);
	}
	printf("Received total %u synchronization data\n",512-remaining);

    //step 8: finish task
	if (dw = FT_StopTask(), dw != FT_STATUS_SUCCESS) {
		printf("Failed to stop task (return value %u)\n", dw);
		return 1;
	}
	FT_ClearPendingFibreData();
	printf("Succeed to stop task\n");

/************************************************************************/
/*******************************TTL operations***************************/
	//set TTL port D11 direction
	if (dw = FT_SetD11Direction(1), dw != FT_STATUS_SUCCESS) {
		printf("Failed to set D11 output (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to set D11 output\n");
	//read from TTL port D9 
	unsigned int val;
	if (dw = FT_ReadTTLPort(9,&val), dw != FT_STATUS_SUCCESS) {
		printf("Failed to read D9 (return value %u)\n", dw);
		return 1;
	}
	printf("Read D9: %u\n",val);
	//write data to TTL port D11
	val = 0x55aa;
	if (dw = FT_WriteTTLPort(11,val), dw != FT_STATUS_SUCCESS) {
		printf("Failed to write D11 (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to write D11 by 0x55aa\n");
/************************************************************************/

	sleep(2);
	//close device
	FT_CloseDevice();
	return 0;
}
