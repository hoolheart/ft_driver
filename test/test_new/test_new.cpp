#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>

int main(int argc, char const *argv[]) {

    unsigned int dw;

    //step 1: open device
    int FID = open("/dev/pcie_ft4", O_RDWR);
    if (FID<0) {
        printf("Failed to open device (return value %u)\n", FID);
        return 1;
    }
    printf("Succeed to open device\n");
    sleep(2);

    //prepare sending data
    unsigned char sendData[16*1024];
    for(int i=0;i<16*1024;i++) {
    	sendData[i] = 0;
    }

    //step 5: receive fibre data
    const int size = 100*1024*1024;
    unsigned char *buffer = new unsigned char[size];
    int remaining = size; int cnt = 0;
    struct timespec start,stop;
    double time_val = 0.0;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    while((remaining>0)&&(cnt<100000)) {
        //receive
        int count = read(FID,buffer,remaining);
        remaining -= count;
        if(count<=0) {
            //send
            //write(FID,sendData,8*1024);
            cnt++;
            usleep(1);
        }
        else {
            printf("Received data %d\n",count);
            usleep(100);
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

    //close device
    close(FID);
    return 0;
}
