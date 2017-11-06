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
    int chl = 4;
    dw = FT_SetParam(FT_PARAMID_FIBRECHL, chl);
    if (dw != FT_STATUS_SUCCESS) {
        printf("Failed to set fibre channel %d (return value %u)\n", chl, dw);
        return 1;
    }
    printf("Succeed to set parameters (fibre channel %d)\n", chl);

    //step 3: start task
    if (dw = FT_StartTask(0), dw != FT_STATUS_SUCCESS) {
        printf("Failed to start task (return value %u)\n", dw);
        return 1;
    }
    printf("Succeed to start task\n");

    //step 4: send fibre data
    const int size = 100*1024*1024;
#if 1
    uint32_t pData[8*1024/4] = {0};//prepare
    //FT_ReceiveFibreData(pData,4096*2);
    int tx_cnt = 0;
    for(uint32_t i=0;i<((size+4096)/(1024*8));i++) {
        for(uint32_t j=0;j<(8*1024/4);j++) {
            pData[j] = (i*(8*1024/4))+j;//fill content
        }
        if (dw = FT_SendFibreData(pData,1024*8), dw != FT_STATUS_SUCCESS) {
            printf("Failed to send fibre data (return value %u)\n", dw);
            return 1;
        }
        else {
            tx_cnt += 1024*8;
        }
        usleep(1);
    }
    printf("Succeed to send %d data\n",tx_cnt);
    //usleep(1);
#endif

    //step 5: receive fibre data
    unsigned char *buffer = new unsigned char[size];
    int remaining = size; int cnt = 0;
    struct timespec start,stop;
    double time_val = 0.0;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    while((remaining>0)&&(cnt<100000)) {
        int count = FT_ReceiveFibreData(buffer+size-remaining,remaining);
        remaining -= count;
        if(count==0) {
            cnt++;
            usleep(1);
        }
        else {
            cnt = 0;
        }
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop);
    time_val = (stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec) * 1.0e-9;
    printf("Received total %u fibre data in %f s\n",size-remaining,time_val);

    //print data
    if((size-remaining)>0) {
        uint16_t *recvData = (uint16_t*)buffer;
        printf("10 Received data\n");
        for(int i=0;i<10;i++) {
            printf("0x%x\n",recvData[i]);
        }
        //save to file
        int fd = open("result.dat",O_WRONLY|O_CREAT|O_TRUNC,0666);
        if(fd>=0) {
            write(fd,buffer,size-remaining);
            close(fd);
            printf("Received data are saved in file: result.dat\n");
        }
        else {
            printf("Open result file failed.\n");
        }
    }

    //step 6: finish task
    if (dw = FT_StopTask(), dw != FT_STATUS_SUCCESS) {
        printf("Failed to stop task (return value %u)\n", dw);
        return 1;
    }
    printf("Succeed to stop task\n");

    sleep(2);
    //close device
    FT_CloseDevice();
    return 0;
}
