// Copyright 2017
// Original Author: Huide Zhou <prettyage.new@gmail.com>

#include <iostream>

#include <random>
#include <chrono>
#include <vector>

//Use Linux file functions for the device
#include <fcntl.h>
#include <unistd.h> // opening flags 

#include "../../src/driver/ft_macros.h"

using std::cout;
using std::endl;

void test_bite(int FID) {
    //prepare iocmd
    uint32_t tmp;
    IOCmd_t iocmd = {BAR_IO,0,FT_BITE,(void *)&tmp};
    //read
    read(FID, &iocmd, sizeof(uint32_t));
    //print
    cout << "Using read interface" << endl;
    cout << "PCI-e state: " << (tmp&0x10000)?"OK":"Error" << endl;
    cout << "DDR2 state: " << (tmp&0x20000)?"OK":"Error" << endl;
    for (int i=0;i<4;i++) {
        cout << "Fibre Reset " << i+1 << ": " << (tmp&(1<<(20+i)))?"OK":"Error" << endl;
        cout << "Fibre Signal " << i+1 << ": " << (tmp&(1<<(24+i)))?"ON":"OFF" << endl;
    }
    //prepare Bar0Cmd
    Bar0Cmd_t barCmd = {FT_BITE,&tmp};
    //read
    ioctl(FID,FT_READ_BAR0_U32,&barCmd);
    //print
    cout << "Using ioctl interface" << endl;
    cout << "PCI-e state: " << (tmp&0x10000)?"OK":"Error" << endl;
    cout << "DDR2 state: " << (tmp&0x20000)?"OK":"Error" << endl;
    for (int i=0;i<4;i++) {
        cout << "Fibre Reset " << i+1 << ": " << (tmp&(1<<(20+i)))?"OK":"Error" << endl;
        cout << "Fibre Signal " << i+1 << ": " << (tmp&(1<<(24+i)))?"ON":"OFF" << endl;
    }
}

void test_cpi_freq(int FID) {
    //prepare iocmd
    uint32_t tmp = 100;
    IOCmd_t iocmd = {BAR_IO,0,CPI_FREQ_BASE,(void *)&tmp};
    //write
    write(FID, &iocmd, sizeof(uint32_t));
    //print
    cout << "Using write interface" << endl;
    cout << "Set CPI1 FREQ: " << tmp << endl;
    //read
    read(FID, &iocmd, sizeof(uint32_t));
    //print
    cout << "Using read interface" << endl;
    cout << "Get CPI1 FREQ: " << tmp << endl;
    //prepare Bar0Cmd
    tmp = 200;
    Bar0Cmd_t barCmd = {CPI_FREQ_BASE,&tmp};
    //write
    ioctl(FID,FT_WRITE_BAR0_U32,&barCmd);
    //print
    cout << "Using ioctl interface" << endl;
    cout << "Set CPI1 FREQ: " << tmp << endl;
    //read
    ioctl(FID,FT_READ_BAR0_U32,&barCmd);
    //print
    cout << "Using ioctl interface" << endl;
    cout << "Get CPI1 FREQ: " << tmp << endl;
}

int main(int argc, char const *argv[]) {
    cout << "Starting test of FIBRE_TEST kernel module." << endl;

    //Open the device
    int FID = open("/dev/pcie_ft1", O_RDWR | O_NONBLOCK);
    if (FID < 0){
        cout << "Could not open device handle!" << endl;
        return -1;
    }
    
    //testing
    test_bite();
    test_cpi_freq();

    //close file
    close(FID);

    return 0;
}