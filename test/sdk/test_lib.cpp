#include "FTCardLib.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>

int main(int argc, char const *argv[]) {

	unsigned int dw;

	//step 1: open device
	if (dw = FT_OpenDevice(), dw != FT_STATUS_SUCCESS) {
		printf("Failed to open device (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to open device\n");

	//step 2: set parameters
	int chl = 1;
	dw = FT_SetParam(FT_PARAMID_FIBRECHL, chl);
	dw = FT_SetParam(FT_PARAMID_TRIGGER, 0);
	dw = FT_SetParam(FT_PARAMID_LEAD,1);
	if (dw != FT_STATUS_SUCCESS) {
		printf("Failed to set fibre channel %d (return value %u)\n", chl, dw);
		return 1;
	}
	printf("Succeed to set parameters (fibre channel %d, inner trigger, send lead)\n", chl);

    //step 3: send control command via synchronized serial port
	unsigned short cmd = 0x7f00;
	if (dw = FT_SendSynData(2,FT_SYNFREQ_1M,&cmd,16), dw != FT_STATUS_SUCCESS) {
		printf("Failed to send synchronization data (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to send mode 1 switch matrix command\n");

    //step 4: start task
	if (dw = FT_StartTask(0), dw != FT_STATUS_SUCCESS) {
		printf("Failed to start task (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to start task\n");

    //step 5: send fibre data
	uint16_t pData[2048] = {0};//prepare
	pData[0] = 0x7fff; pData[1] = 0xbc1c; pData[2] = 0xaaaa; pData[3] = 0xbbbb; pData[4] = 0x3c1c; pData[5] = 0x7fff; //add header
	for(int i=6;i<2048-6;i++) {
	    pData[i] = 1024-((i-6)%1024);//fill content
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
	const int size = 128*1024*1024;
	unsigned char *buffer = new unsigned char[size];
   int remaining = size; int cnt = 0;
	struct timespec start,stop;
	double time_val = 0.0;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
	while((remaining>0)&&(cnt<5e5)) {
		int count = FT_ReceiveFibreData(buffer+size-remaining,remaining);
		cnt++;
		remaining -= count;
		if(count==0) {
			usleep(10);
		}
	}
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop);
	time_val = (stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec) * 1.0e-9;
	printf("Received total %u fibre data in %f s\n",size-remaining,time_val);

    //step 7: finish task
	if (dw = FT_StopTask(), dw != FT_STATUS_SUCCESS) {
		printf("Failed to stop task (return value %u)\n", dw);
		return 1;
	}
	printf("Succeed to stop task\n");

	//print data
	if((size-remaining)>0) {
		uint16_t *recvData = (uint16_t*)buffer;
		int size_u16 = (size-remaining)/2;
		printf("10 Received data\n");
		for(int i=0;i<10;i++) {
			printf("0x%x\n",recvData[i]);
		}
		int start_index = -1;
		for(int i=0;i<size_u16;i++) {
			if(start_index<0) {
				if((i<(size_u16-10)) && (recvData[i]==pData[0]) && (recvData[i+6]==pData[6])) {
					start_index = i;
					printf("match transmit data\n");
					printf("begin index: %d\n",start_index);
					printf("0x%x\n",recvData[i]);
				}
			}
			else {
				if(i<(start_index+10)) {
					printf("0x%x\n",recvData[i]);
				}
				else {
					break;
				}
			}
		}
		if(start_index<0) {
			printf("no match transmit data\n");
		}
	}
	//save to file
	int fd = open("result.dat",O_WRONLY|O_CREAT|O_TRUNC,0670);
	if(fd>=0) {
		write(fd,buffer,128*1024);
		close(fd);
	}

    //step 8: receive synchronized serial port data
	unsigned char synRecv[512]; remaining = 512; cnt = 0;
	while((remaining>0)&&(cnt<20)) {
		if(FT_GetPendingSynSize()>0) {
			printf("Receive %u synchronization data\n",FT_GetPendingSynSize());
			remaining -= FT_ReceiveSynData(synRecv,remaining);
		}
		cnt++;
		usleep(5000);
	}
	printf("Received total %u synchronization data\n",512-remaining);

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
