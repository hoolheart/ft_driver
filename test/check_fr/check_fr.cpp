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

enum CHK_STEP {
    CHK_HEAD0 = 0,
    CHK_HEAD1,
    CHK_WAIT_CNT,
    CHK_CNT_B0,
    CHK_CNT_B1,
    CHK_CNT_M0,
    CHK_CNT_M1,
    
    CHK_STEP_END
};

const int head_len = 6;
const unsigned int head0 = 0xaaaaaaaa, head1 = 0x55555555;
const int ignore_cnt = 10;

int main(int argc, char const *argv[]) {
    
    //prepare record file
    ofstream f_record("record.txt",ios::out|ios::app);
    if(!f_record.is_open()) {
        cout<<"Failed to open record file"<<endl;
        return 2;
    }
    cout<<"Succeed to open record file"<<endl;

    //receive fibre data
    const int size = 128*1024*1024;
    const int int_size = size/sizeof(unsigned int);
    int remaining = size;
    unsigned char *buffer = new unsigned char[size];
    unsigned int  *int_buf = (unsigned int*)buffer;
    struct timespec start,stop;
    double time_val = 0.0;
    
    //prepare check
    bool started = false;
    CHK_STEP step = CHK_HEAD0;
    int head_cnt = 0;
    unsigned int last_cnt_b = 0, last_cnt_m = 0, cnt_b = 0, cnt_m = 0;

    //open device
    int FID = open("/dev/pcie_ft1", O_RDWR);
    if (FID<0) {
        cout<<"Failed to open device (return value "<<FID<<")"<<endl;
        f_record.close();
        return 1;
    }
    cout<<"Succeed to open device"<<endl;
    //sleep(1);
    
    //start receiving data
    cout<<"Start to receive data"<<endl;
    int repeat_cnt = 0;
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
        if(repeat_cnt<ignore_cnt) {
            repeat_cnt++;
            continue;
        }
        //process data
        for(int i=0;i<int_size;i++) {
            switch (step) {
            case CHK_HEAD0:
                //head 0
                if(int_buf[i]==head0) {
                    step = CHK_HEAD1;
                }
                break;
            case CHK_HEAD1:
                //head 1
                if(int_buf[i]==head1) {
                    step = CHK_WAIT_CNT;
                    head_cnt = 2;
                }
                else if(int_buf[i]!=head0) {
                    step = CHK_HEAD0;
                }
                break;
            case CHK_WAIT_CNT:
                //wait head finished
                head_cnt++;
                if(head_cnt==head_len) {
                    step = CHK_CNT_B0;
                }
                else if(head_cnt>head_len) {
                    step = CHK_HEAD0;
                }
                break;
            case CHK_CNT_B0:
                //first part of big count
                cnt_b = (int_buf[i]&0xffff);
                step = CHK_CNT_B1;
                break;
            case CHK_CNT_B1:
                //second part of big count
                cnt_b = (cnt_b<<16)+(int_buf[i]&0xffff);
                step = CHK_CNT_M0;
                break;
            case CHK_CNT_M0:
                //first part of minor count
                cnt_m = (int_buf[i]&0xffff);
                step = CHK_CNT_M1;
                break;
            case CHK_CNT_M1:
                //secord part of minor count
                cnt_m = (cnt_m<<16)+(int_buf[i]&0xffff);
                step = CHK_HEAD0;
                //record to file
                f_record<<cnt_b<<"\t"<<cnt_m<<endl;
                //check count
                if(started) {
                    if(last_cnt_b==cnt_b) {
                        //check minor count
                        if((last_cnt_m+1)!=cnt_m) {
                            f_record<<"error on minor count:"<<last_cnt_m<<"\t"<<cnt_m<<endl;
                            cout<<"error on minor count:"<<last_cnt_m<<"\t"<<cnt_m<<endl;
                        }
                    }
                    else {
                        //check big count
                        if((last_cnt_b+1)!=cnt_b) {
                            f_record<<"error on big count:"<<last_cnt_b<<"\t"<<cnt_b<<endl;
                            cout<<"error on big count:"<<last_cnt_b<<"\t"<<cnt_b<<endl;
                        }
                    }
                }
                else {
                    started = true;
                }
                //record count
                last_cnt_b = cnt_b;
                last_cnt_m = cnt_m;
                break;
            default:
                step = CHK_HEAD0;
                break;
            }
        }
        f_record.flush();
    }

    //close device
    f_record.close();
    close(FID);
    return 0;
}

