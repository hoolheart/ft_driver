// Copyright 2017
// Original Author: Huide Zhou <prettyage.new@gmail.com>

#include <iostream>
using namespace std;

//Use Linux file functions for the device
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> // opening flags
#include <stdint.h>

#include "ft_macros.h"

uint32_t readRegister(int fid, uint32_t addr) {
    //prepare Bar0Cmd
    uint32_t tmp;
    Bar0Cmd_t barCmd = {addr,&tmp};
    //read
    ioctl(fid,FT_READ_BAR0_U32,&barCmd);
    return tmp;
}

void writeRegister(int fid, uint32_t addr, uint32_t value) {
    //prepare Bar0Cmd
    Bar0Cmd_t barCmd = {addr,&value};
    //write
    ioctl(fid,FT_WRITE_BAR0_U32,&barCmd);
}

int main(int argc, char const *argv[]) {
    cout << "Starting test of FIBRE_TEST kernel module." << endl;

    //Open the device
    int FID = open("/dev/pcie_ft1", O_RDWR | O_NONBLOCK);
    if (FID < 0){
        cout << "Could not open device handle!" << endl;
        return -1;
    }
    cout << endl;

    //test read
    uint32_t tmp = readRegister(FID, FT_BITE);
    cout << "Read BITE register" << endl;
    cout << "state register " << tmp << endl;
    cout << "PCI-e state: " << ((tmp&0x10000)?"OK":"Error") << endl;
    cout << "DDR2 state: " << ((tmp&0x20000)?"OK":"Error") << endl;
    for (int i=0;i<4;i++) {
        cout << "Fibre Reset " << i+1 << ": " << ((tmp&(1<<(20+i)))?"OK":"Error") << endl;
        cout << "Fibre Signal " << i+1 << ": " << ((tmp&(1<<(24+i)))?"ON":"OFF") << endl;
    }
    cout << endl;

    //test write
    tmp = 100;
    writeRegister(FID, CPI_FREQ_BASE, tmp);
    cout << "Write CPI_FREQ_BASE register" << endl;
    cout << "Set CPI1 FREQ: " << tmp << endl;
    //read
    tmp = readRegister(FID, CPI_FREQ_BASE);
    cout << "Get CPI1 FREQ: " << tmp << endl;

    //close file
    close(FID);

    return 0;
}
