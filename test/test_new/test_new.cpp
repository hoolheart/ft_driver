#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main(int argc, char const *argv[]) {

    unsigned int dw;

    //step 1: open device
    int FID = open("/dev/pcie_ft1", O_RDWR);
    if (FID<0) {
        printf("Failed to open device (return value %u)\n", FID);
        return 1;
    }
    printf("Succeed to open device\n");
    sleep(2);

    //step 5: receive fibre data
    printf("Start to receive data\n");
    const int size = 1024*1024*1024;
    const int count = 128;
    int single_size = size/count;
    unsigned char *buffer = new unsigned char[single_size];
    int remaining = size;
    struct timespec start,stop;
    double time_val = 0.0;
    bool isFirst = true;
    while(true) {
        //save to file
        int fd = open(isFirst?"result0.dat":"result1.dat",O_WRONLY|O_CREAT|O_TRUNC,0666);
        isFirst = !isFirst;
        if(fd<0) {
            continue;
        }
        //record time
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
        for(int i=0;i<count;i++) {
            int remaining = single_size;
            while(remaining>0) {
                //receive
                int len = read(FID,buffer+single_size-remaining,remaining);
                remaining -= len;
            }
            //save to file
            write(fd,buffer,single_size);
        }
        close(fd);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop);
        //print
        time_val = (stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec) * 1.0e-9;
        printf("Received total %u fibre data in %f s and saved to %s\n",size,time_val,isFirst?"result1.dat":"result0.dat");
    }

    //close device
    close(FID);
    return 0;
}
