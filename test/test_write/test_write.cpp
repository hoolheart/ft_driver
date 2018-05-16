#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main(int argc, char const *argv[]) {

    unsigned int dw;

    //open device
    int FID = open("/dev/pcie_ft1", O_RDWR);
    if (FID<0) {
        printf("Failed to open device (return value %u)\n", FID);
        return 1;
    }
    printf("Succeed to open device\n");

    //send fibre data
    const int send_size = 32*1024;
    unsigned char *send_data = new unsigned char[send_size];
    //TODO prepare send data here
    printf("Start sending data.\n");
    int send_len = write(FID, send_data, send_size);
    printf("Succeed to send data: %d\n", send_len);

    //receive fibre data
    printf("Start to receive data\n");
    const int size = 8*1024*1024;
    unsigned char *buffer = new unsigned char[size];
    //receive
    int len = read(FID,buffer,size);
    printf("Succeed to receive data: %d\n",len);

    //close device
    close(FID);
    return 0;
}
