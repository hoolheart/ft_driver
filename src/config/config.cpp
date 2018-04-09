// Copyright 2017
// Original Author: Huide Zhou <prettyage.new@gmail.com>


#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdio>
using namespace std;

//Use Linux file functions for the device
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> // opening flags
#include <stdint.h>

#include "ft_macros.h"

#define ADDR_CPLL      0X3000
#define ADDR_QPLL      0X3004
#define ADDR_PLL_SRC   0X3008
#define ADDR_CHL_RESET 0X300C
#define CHL_RANGE      0X800
#define OFFSET_CPLL1   0X5E
#define OFFSET_CPLL2   0X88
#define ADDR_SYN_SET   0X3040
#define ADDR_SYN_PARA  0X3044
#define ADDR_IDLE_SET  0X3050
#define ADDR_IDLE_PARA 0X3054

struct speed_setting {
    uint32_t clock;
    uint32_t para0;
    uint32_t para1;
};

struct speed_setting predefines[10] = {
    {3,0x1002,0x11},{1,0x1002,0x11},{2,0x1082,0x11},{1,0x1082,0x11},{1,0x1083,0x11},
    {3,0x1002,0},{1,0x1002,0},{2,0x1082,0},{1,0x1082,0},{1,0x1083,0}
};

int32_t speed_code = 1,
    syn_length = 2,
    syn_kcode  = 3,
    syn_value  = 0x5a5a,
    idle_length = 2,
    idle_kcode  = 3,
    idle_value  = 0xbcbc;

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

void applySetting(int fid) {
    //speed
    if((speed_code<1) || (speed_code>10)) {//check code
        cout<<"Wrong speed code"<<speed_code<<endl;
        return;
    }
    cout<<"Setting..."<<endl;
    struct speed_setting &s_setting = predefines[speed_code-1];//get setting
    writeRegister(fid,ADDR_CPLL,s_setting.clock);
    for(int i=0;i<4;i++) {
        uint32_t chl_base = CHL_RANGE*(i+1);
        writeRegister(fid,chl_base+OFFSET_CPLL1,s_setting.para0);
        writeRegister(fid,chl_base+OFFSET_CPLL2,s_setting.para1);
    }
    //synchronized head
    uint32_t syn_para = (syn_length<<4) + (syn_kcode&0xf);
    writeRegister(fid,ADDR_SYN_PARA,syn_para);
    writeRegister(fid,ADDR_SYN_SET,syn_value);
    //idle code
    uint32_t idle_para = (idle_length<<4) + (idle_kcode&0xf);
    writeRegister(fid,ADDR_IDLE_PARA,idle_para);
    writeRegister(fid,ADDR_IDLE_SET,idle_value);
    //reset channels
    cout<<"Reset channels..."<<endl;
    writeRegister(fid,ADDR_CHL_RESET,0);
    writeRegister(fid,ADDR_CHL_RESET,1);
    writeRegister(fid,ADDR_CHL_RESET,0);
    cout<<"Succeed to configure board"<<endl;
}

bool parseLine(string &line) {
    //prepare string
    istringstream s(line);
    //parse key
    string key,eq;
    if(!(s>>key)) {
        return false;
    }
    else if(key[0] == '#') {
        return true;
    }
    else if((!(s>>eq)) || (eq!="=")) {
        return false;
    }
    //parse value
    if(key=="speed_code") {
        return s>>speed_code;
    }
    else if(key=="syn_length") {
        return s>>syn_length;
    }
    else if(key=="syn_kcode") {
        return s>>syn_kcode;
    }
    else if(key=="syn_value") {
        return s>>std::hex>>syn_value;
    }
    else if(key=="idle_length") {
        return s>>idle_length;
    }
    else if(key=="idle_kcode") {
        return s>>idle_kcode;
    }
    else if(key=="idle_value") {
        return s>>std::hex>>idle_value;
    }
    else {
        return false;
    }
}

bool parseFile(const string &f_name) {
    cout<<"parsing file: "<<f_name<<endl;
    //prepare file stream
    ifstream f(f_name,ios::in);
    if(f.is_open()) {
        //parse file
        string line;
        bool flag = true;
        while(getline(f,line)) {
            //parse line
            if(!parseLine(line)) {
                cout<<"failed to parse line: "<<line<<endl;
                flag = false;
                break;
            }
        }
        f.close();
        return flag;
    }
    else {
        cout<<"failed to open file: "<<f_name<<endl;
        return false;
    }
}

void printSetting() {
    cout<<"speed code: "<<speed_code<<endl;
    cout<<"syn length: "<<syn_length<<endl;
    cout<<"syn kcode: "<<syn_kcode<<endl;
    cout<<"syn value: "<<std::hex<<syn_value<<endl;
    cout<<"idle length: "<<idle_length<<endl;
    cout<<"idle kcode: "<<idle_kcode<<endl;
    cout<<"idle value: "<<std::hex<<idle_value<<endl;
}

int main(int argc, char const *argv[]) {
    
    cout << "Starting configure FIBRE_TEST board." << endl;

    //Open the device
    int FID = open("/dev/pcie_ft1", O_RDWR | O_NONBLOCK);
    if (FID < 0){
        cout << "Could not open device handle!" << endl;
        return -1;
    }
    cout << endl;

    //Read configure file
    if(parseFile("ft.conf")) {
        //print
        printSetting();
        //apply setting
        applySetting(FID);
    }

    //close file
    close(FID);

    return 0;
}
