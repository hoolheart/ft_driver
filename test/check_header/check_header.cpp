#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char const *argv[]) {

    //open device
    int FID = open("/dev/pcie_ft1", O_RDWR);
    if (FID<0) {
        cout<<"Failed to open device (return value "<<FID<<")"<<endl;
        return 1;
    }
    cout<<"Succeed to open device"<<endl;
    sleep(1);
    
    //prepare record file
    ofstream f_record("record.txt",ios::out|ios::app);
    if(!f_record.is_open()) {
        cout<<"Failed to open record file"<<endl;
        close(FID);
        return 2;
    }
    cout<<"Succeed to open record file"<<endl;

    //receive fibre data
    cout<<"Start to receive data"<<endl;
    const int size = 128*1024*1024;
    const int int_size = size/sizeof(unsigned int);
    int remaining = size;
    unsigned char *buffer = new unsigned char[size];
    unsigned int  *int_buf = (unsigned int*)buffer;
    struct timespec start,stop;
    double time_val = 0.0;
    unsigned int head0 = 0xaaaaaaaa, head1 = 0x55555555;
    int last_head = -1, repeat = 0;
    while(true) {
        //record time
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
        remaining = size;
        while(remaining>0) {
            //receive
            int len = read(FID,buffer+size-remaining,remaining);
            remaining -= len;
        }
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop);
        //print
        time_val = (stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec) * 1.0e-9;
        cout<<"Received total "<<size<<" fibre data in "<<time_val<<" s"<<endl;
        //find head
        bool found = false;
        for(int i=0;i<int_size-1;i++) {
            if((int_buf[i]==head0) && (int_buf[i+1]==head1)) {
                if(last_head>=0) {
                    //calculate range
                    int range = (i-last_head-2)*sizeof(unsigned int);
                    if(!found) {
                        range += repeat*size;
                        repeat = 0;
                    }
                    //record
                    f_record<<range<<endl;
                }
                last_head = i;
                found = true;
            }
        }
        repeat++;
        f_record.flush();
    }

    //close device
    f_record.close();
    close(FID);
    return 0;
}

