#include "FTCardDriver_class.h"
#include "ft_macros.h"
#include <pthread.h>
#include <string.h>

//Use Linux file functions for the device
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> // opening flags
#include <stdint.h>

enum FT_CARD_PARAM {
    FT_PARAM_FIBRECHL = 0,/**< fibre channel */
    FT_PARAM_TRIGGER,/**< trigger source, 0:inner, 1:outer */
    FT_PARAM_LEAD,/**< fibre lead signal, 0:don't send, 1:send */
    FT_PARAM_CPINUM,/**< cpi number 1-5 */
    FT_PARAM_PRF1,/**< pulse repeat frequency */
    FT_PARAM_PRF2,
    FT_PARAM_PRF3,
    FT_PARAM_PRF4,
    FT_PARAM_PRF5,
    FT_PARAM_CPI1_PULSENUM,/**< pulse number of one cpi */
    FT_PARAM_CPI2_PULSENUM,
    FT_PARAM_CPI3_PULSENUM,
    FT_PARAM_CPI4_PULSENUM,
    FT_PARAM_CPI5_PULSENUM,
    FT_PARAM_DELAY1,/**< delay */
    FT_PARAM_DELAY2,
    FT_PARAM_DELAY3,
    FT_PARAM_DELAY4,
    FT_PARAM_DELAY5,
    FT_PARAM_DELAY6,
    FT_PARAM_DELAY7,
    FT_PARAM_DELAY8,
    FT_PARAM_DELAY9,
    FT_PARAM_DELAY10,
    FT_PARAM_DELAY11,
    FT_PARAM_DELAY12,
    FT_PARAM_DELAY13,
    FT_PARAM_DELAY14,
    FT_PARAM_PULSEWIDTH1,/**< pulse width */
    FT_PARAM_PULSEWIDTH2,
    FT_PARAM_PULSEWIDTH3,
    FT_PARAM_PULSEWIDTH4,
    FT_PARAM_PULSEWIDTH5,
    FT_PARAM_PULSEWIDTH6,
    FT_PARAM_PULSEWIDTH7,
    FT_PARAM_PULSEDIR,/**< pulse direction */

    FT_PARAM_NUM
};

struct FT_CARD_DATA {
    enum{DEV_CLOSE,DEV_OPEN,TASK_GOING} deviceStatus;/**< device status */
    pthread_mutex_t mutexDev;/**< device mutex */
    //HANDLE eventInt;/**< events of device interrupts */

    unsigned char *transmitData;/**< transmit fibre data */

    bool stopSignalFibre, stopSignalSyn;/**< signal to stop threads */
    pthread_t hThreadFibre, hThreadSyn;/**< handles of threads */

    unsigned char *fibreData;/**< fibre data pool */
    int iFibreStart, iFibreStop;/**< indexes of fibre data */
    pthread_mutex_t mutexFibre;/**< mutex of fibre data */

    unsigned char *synData;/**< synchronization data pool */
    int iSynStart, iSynStop;/**< indexes of synchronization data */
    pthread_mutex_t mutexSyn;/**< mutex of synchronization data */

    unsigned int params[FT_PARAM_NUM];/**< parameters */
};

#define TRANS_DATA_SIZE (8*1024)//transmit data size
#define SINGLE_RECV_SIZE (4*1024)//single receive fibre data size
#define FIBRE_DATA_SIZE (10*1024*1024)//fibre data pool size
#define SYN_DATA_SIZE 512//synchronization data pool size

FTCardDriver* FTCardDriver::instance = 0;

FTCardDriver* FTCardDriver::getInstance()
{
    if (instance == 0)
        instance = new FTCardDriver();//create instance
    return instance;
}

FTCardDriver::FTCardDriver()
{
    //prepare data
    d = new FT_CARD_DATA;
    //events
    //d->eventInt = CreateEvent(NULL, FALSE, FALSE, NULL);

    //setup driver
    fid = -1;
    d->deviceStatus = FT_CARD_DATA::DEV_CLOSE;
    pthread_mutex_init(&d->mutexDev,0);//mutex

    //initialize parameters
    memset(d->params, 0, sizeof(unsigned int)*FT_PARAM_NUM);
    d->params[FT_PARAM_FIBRECHL] = 1;//fibre channel
    d->params[FT_PARAM_CPINUM] = 1;//one cpi
    for (int i = 0; i < 5; i++) {
        d->params[FT_PARAM_PRF1 + i] = 20000000 / 2000;//PRF:2000Hz
        d->params[FT_PARAM_CPI1_PULSENUM + i] = 1;//one pulse per CPI
    }
    for (int i = FT_PARAM_PULSEWIDTH1; i <= FT_PARAM_PULSEWIDTH7; i++) {
        d->params[i] = 1;//default pulse width: 1us
    }

    //transmit data
    d->transmitData = new unsigned char[TRANS_DATA_SIZE];//create

    //fibre data
    d->fibreData = new unsigned char[FIBRE_DATA_SIZE];//create pool
    d->iFibreStart = 0; d->iFibreStop = 0;//initialize index
    pthread_mutex_init(&d->mutexFibre,0);//mutex

    //synchronization data
    d->synData = new unsigned char[SYN_DATA_SIZE];//create pool
    d->iSynStart = 0; d->iSynStop = 0;//initialize index
    pthread_mutex_init(&d->mutexSyn,0);//mutex

    //threads
    d->stopSignalFibre = false; d->stopSignalSyn = false;//no stop signal
    //create threads
    d->hThreadFibre = 0;
    pthread_create(&d->hThreadSyn,0,FTCardDriver::receiveSyn,0);
}

FTCardDriver::~FTCardDriver()
{
    //close device
    close_device();

    //stop threads
    d->stopSignalFibre = true; d->stopSignalSyn = true;//set signal
    //wait threads to finish
    if (d->hThreadFibre) {
        pthread_join(d->hThreadFibre,0);
    }
    pthread_join(d->hThreadSyn,0);

    //delete data pools
    delete d->transmitData;
    delete d->fibreData; delete d->synData;
    pthread_mutex_destroy(&d->mutexFibre); pthread_mutex_destroy(&d->mutexSyn);

    //delete mutex
    pthread_mutex_destroy(&d->mutexDev);
}

unsigned int FTCardDriver::open_device()
{
    if (fid>=0) {//check device
        return FT_STATUS_SUCCESS;
    }
    pthread_mutex_lock(&d->mutexDev);//lock driver
    //open device
    fid = open("/dev/pcie_ft1", O_RDWR | O_NONBLOCK);
    if (fid<0) {
        pthread_mutex_unlock(&d->mutexDev);//release driver
        return FT_STATUS_OPEN_FAILED;
    }
    d->deviceStatus = FT_CARD_DATA::DEV_OPEN;//set status
    //initialize events
    pthread_mutex_unlock(&d->mutexDev);//release driver
    //initialize send command
    writeBAR0(SEND_ADDR, 0);
    //initialize TTL direction
    writeBAR0(TTL_DIR_ADDR,0x02);
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::close_device()
{
    if (d->deviceStatus == FT_CARD_DATA::TASK_GOING) {//check task tatus
        stopTask();
    }

    if (fid<0) {//check device
        return FT_STATUS_SUCCESS;
    }
    pthread_mutex_lock(&d->mutexDev);//lock driver
    //close device
    close(fid);
    fid = -1;
    d->deviceStatus = FT_CARD_DATA::DEV_CLOSE;//set satus
    pthread_mutex_unlock(&d->mutexDev);//release driver
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::reset()
{
    if (fid>=0) {//check device
        pthread_mutex_lock(&d->mutexDev);//lock driver
        ioctl(fid,FT_RESET,0);//reset device
        pthread_mutex_unlock(&d->mutexDev);//release driver
        return FT_STATUS_SUCCESS;
    }
    return FT_STATUS_NOTOPEN;
}

unsigned int FTCardDriver::setParam(unsigned char id, int value)
{
    //check status
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE) {
        return FT_STATUS_NOTOPEN;
    }
    else if (d->deviceStatus == FT_CARD_DATA::TASK_GOING) {
        return FT_STATUS_TASKINGOING;
    }
    //set parameter
    if (id == FT_PARAMID_FIBRECHL) {//fibre channel
        if ((value < 1) || (value>4)) {//check value
            return FT_STATUS_PARAMVAL_INVALID;
        }
        d->params[FT_PARAM_FIBRECHL] = (unsigned int)value;//record
    }
    else if (id == FT_PARAMID_TRIGGER) {//trigger
        if ((value < 0) || (value>1)) {//check value
            return FT_STATUS_PARAMVAL_INVALID;
        }
        d->params[FT_PARAM_TRIGGER] = (unsigned int)value;//record
    }
    else if (id == FT_PARAMID_LEAD) {//fibre lead
        if ((value < 0) || (value>1)) {//check value
            return FT_STATUS_PARAMVAL_INVALID;
        }
        d->params[FT_PARAM_LEAD] = (unsigned int)value;//record
    }
    else if (id == FT_PARAMID_CPINUM) {//cpi number
        if((value<1) || (value>5)) {
            return FT_STATUS_PARAMVAL_INVALID;
        }
        d->params[FT_PARAM_CPINUM] = (unsigned int)value;//record
    }
    else if (id==FT_PARAMID_PRF) {
        if((value<1) || (value>20e6)) {
            return FT_STATUS_PARAMVAL_INVALID;
        }
        double prf = 20e6/(double)value;
        for(int i=0;i<5;i++) {
            d->params[FT_PARAM_PRF1+i] = (unsigned int)(prf+0.5);//record
        }
    }
    else if (id==FT_PARAMID_CPI_PULSENUM) {
        for(int i=0;i<5;i++) {
            d->params[FT_PARAM_CPI1_PULSENUM+i] = (unsigned int)value;//record
        }
    }
    else if ((id>=FT_PARAMID_DELAY1) && (id<=FT_PARAMID_DELAY14)) {
        d->params[FT_PARAM_DELAY1+(id-FT_PARAMID_DELAY1)] = (unsigned int)value;//record
    }
    else if ((id>=FT_PARAMID_PULSEWIDTH1) && (id<=FT_PARAMID_PULSEWIDTH5)) {
        d->params[FT_PARAM_PULSEWIDTH1+(id-FT_PARAMID_PULSEWIDTH1)] = (unsigned int)value;//record
    }
    else if ((id>=FT_PARAMID_PULSEDIR1) && (id<=FT_PARAMID_PULSEDIR23)) {
        if ((value < 0) || (value>1)) {//check value
            return FT_STATUS_PARAMVAL_INVALID;
        }
        unsigned int cur = d->params[FT_PARAM_PULSEDIR];//current value
        unsigned int mask = 0x01<<(id-FT_PARAMID_PULSEDIR1);//mask
        if(value==1) {
            cur |= mask;
        }
        else {
            cur &= (~mask);
        }
        d->params[FT_PARAM_PULSEDIR] = cur;//record
    }
    else {
        return FT_STATUS_PARAMID_INVALID;
    }
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::startTask()
{
    //check status
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE) {
        return FT_STATUS_NOTOPEN;
    }
    else if (d->deviceStatus == FT_CARD_DATA::TASK_GOING) {
        return FT_STATUS_TASKINGOING;
    }

    //reset
    reset();

    //set parameters
    unsigned int addr, in;
    //lead
    in = (d->params[FT_PARAM_LEAD] > 0) ? (0X01 << (d->params[FT_PARAM_FIBRECHL] - 1)) : 0;//get value
    addr = FIBRE_LEAD_ADDR;//get addr
    writeBAR0(addr, in);//write
    //PRF
    for (int i = 0; i<5; i++) {
        in = d->params[FT_PARAM_PRF1 + i];//get value
        addr = CPI_FREQ_BASE + 4 * i;//get addr
        writeBAR0(addr, in);//write
    }
    //cpi pulse num
    for (int i = 0; i<5; i++) {
        in = d->params[FT_PARAM_CPI1_PULSENUM + i];//get value
        addr = CPI_PULSE_NUM_BASE + 4 * i;//get addr
        writeBAR0(addr, in);//write
    }
    //cpi num
    in = d->params[FT_PARAM_CPINUM];//get value
    addr = CPI_NUM_ADDR;//get addr
    writeBAR0(addr, in);//write
    //delay
    for (int i = 0; i<14; i++) {
        in = d->params[FT_PARAM_DELAY1 + i];//get value
        addr = DELAY_BASE + 4 * i;//get addr
        writeBAR0(addr, in);//write
    }
    //pulse width
    for (int i = 0; i<7; i++) {
        in = d->params[FT_PARAM_PULSEWIDTH1 + i];//get value
        addr = PULSE_WIDTH_BASE + 4 * i;//get addr
        writeBAR0(addr, in);//write
    }
    //pulse direction
    in = d->params[FT_PARAM_PULSEDIR];//get value
    addr = PULSE_DIR_ADDR;//get addr
    writeBAR0(addr, in);//write
    //trigger source
    in = (d->params[FT_PARAM_TRIGGER]>0) ? 0X03 : 0;//get value
    addr = TRIGGER_ADDR;//get addr
    writeBAR0(addr, in);//write

    //config channel
    unsigned int reg, val;
    val = ((d->params[FT_PARAM_FIBRECHL] - 1) << 3) + 0x03;//channel
    readBAR0(0x38, reg);//read existing state
    writeBAR0(0x38, (reg & 0x03) | val);//set channel

    //start fibre receiving thread
    d->stopSignalFibre = false;
    pthread_create(&d->hThreadFibre, 0, FTCardDriver::receiveFibre, 0);

    //set status
    d->deviceStatus = FT_CARD_DATA::TASK_GOING;
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::sendFibreData(void * ptr, unsigned int len)
{
    //check status
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE) {
        return FT_STATUS_NOTOPEN;
    }
    else if (d->deviceStatus == FT_CARD_DATA::DEV_OPEN) {
        return FT_STATUS_TASKNOTSTARTED;
    }

    //check length
    if (len > TRANS_DATA_SIZE) {
        return FT_STATUS_FIBREDATA_TOOLONG;
    }

    //prepare
    memcpy(d->transmitData, ptr, len);//copy data to buffer
    if ((TRANS_DATA_SIZE-len)>0) {
        memset((char *)d->transmitData + len, 0, TRANS_DATA_SIZE-len);//fill 0
    }
    IOCmd_t iocmd = {DMA_DATA,0,0,(void *)d->transmitData};
    //send data
    int count = write(fid, &iocmd, TRANS_DATA_SIZE);
    if (count > 0) {//check result
        if (d->params[FT_PARAM_TRIGGER]) {//outer
            writeBAR0(SEND_ADDR, 1);//send
            writeBAR0(SEND_ADDR, 0);
        }
        else {//inner
            writeBAR0(INNER_TRIGGER_ADDR, 1);//open sequence
        }
        return FT_STATUS_SUCCESS;
    }
    return FT_STATUS_TIMEOUT;
}

unsigned int FTCardDriver::getPendingFibreSize()
{
    if (d->deviceStatus == FT_CARD_DATA::TASK_GOING) {//check status
        pthread_mutex_lock(&d->mutexFibre);//lock data
        unsigned int size = (d->iFibreStop >= d->iFibreStart) ? (d->iFibreStop - d->iFibreStart) : (d->iFibreStop + FIBRE_DATA_SIZE - d->iFibreStart);//get size
        pthread_mutex_unlock(&d->mutexFibre);//release data
        return size;
    }
    return 0;
}

unsigned int FTCardDriver::receiveFibreData(void * ptr, unsigned int max_len)
{
    if (d->deviceStatus == FT_CARD_DATA::TASK_GOING) {//check status
        pthread_mutex_lock(&d->mutexFibre);//lock data
        unsigned int size = (d->iFibreStop >= d->iFibreStart) ? (d->iFibreStop - d->iFibreStart) : (d->iFibreStop + FIBRE_DATA_SIZE - d->iFibreStart);//get size
        if (size > max_len) size = max_len;
        unsigned char *pData = (unsigned char *)ptr;
        for (unsigned int i = 0; i < size; i++) {
            pData[i] = d->fibreData[d->iFibreStart];//fill data
            d->iFibreStart = (d->iFibreStart + 1) % FIBRE_DATA_SIZE;//increase index
        }
        pthread_mutex_unlock(&d->mutexFibre);//release data
        return size;
    }
    return 0;
}

void FTCardDriver::clearPendingFibreData()
{
    pthread_mutex_lock(&d->mutexFibre);//lock data
    d->iFibreStart = 0; d->iFibreStop = 0;//initialize index
    pthread_mutex_unlock(&d->mutexFibre);//release data
}

unsigned int FTCardDriver::stopTask()
{
    //check status
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE) {
        return FT_STATUS_NOTOPEN;
    }
    else if (d->deviceStatus == FT_CARD_DATA::DEV_OPEN) {
        return FT_STATUS_SUCCESS;
    }

    //stop task
    writeBAR0(INNER_TRIGGER_ADDR, 0);//stop inner trigger
    //writeBAR0(0x38, 0);//clear channel setting
    d->stopSignalFibre = true;//set signal
    //wait threads to finish
    if (d->hThreadFibre) {
        pthread_join(d->hThreadFibre,0);
        d->hThreadFibre = 0;
    }
    d->deviceStatus = FT_CARD_DATA::DEV_OPEN;
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::sendSynData(unsigned char chl, SYN_FREQ freq, void * ptr, unsigned int len)
{
    //check status
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE)
        return FT_STATUS_NOTOPEN;
    //check channel
    if ((chl < 1) || (chl>3))
        return FT_STATUS_SYNCHL_INVALID;
    //check length
    if ((len < 1) || (len > 256))
        return FT_STATUS_SYNDATA_SIZEWRONG;
    //check frequency
    if ((freq<FT_SYNFREQ_500K) || (freq>FT_SYNFREQ_20M))
        return FT_STATUS_SYNFREQ_INVALID;
    //send data
    unsigned int baseList[3] = { SYN1_TX_CYCLE_ADDR, SYN2_TX_CYCLE_ADDR, SYN3_TX_CYCLE_ADDR };
    chl--;
    //set cycle
    unsigned int cycles[FT_SYNFREQ_20M + 1] = {40,20,4,2,1};
    writeBAR0(baseList[chl], cycles[freq]);
    //set length
    writeBAR0(baseList[chl] + 4, len-1);
    //fill data
    len = (len + 7) / 8;//change len in bit to unsigned char
    unsigned int dat[8] = { 0 }; memcpy(dat, ptr, len);//prepare data
    for (unsigned int i = 0; i<(len + 3) / 4; i++) {
        writeBAR0(baseList[chl] + 8, dat[i]);//write data
    }
    //trigger
    writeBAR0(baseList[chl]+0xC,1);
    usleep(100);
    writeBAR0(baseList[chl]+0xC,0);
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::getPendingSynSize()
{
    if (d->deviceStatus != FT_CARD_DATA::DEV_CLOSE) {//check status
        pthread_mutex_lock(&d->mutexSyn);//lock data
        unsigned int size = (d->iSynStop >= d->iSynStart) ? (d->iSynStop - d->iSynStart) : (d->iSynStop + SYN_DATA_SIZE - d->iSynStart);//get size
        pthread_mutex_unlock(&d->mutexSyn);//release data
        return size;
    }
    return 0;
}

unsigned int FTCardDriver::receiveSynData(void * ptr, unsigned int max_len)
{
    if (d->deviceStatus != FT_CARD_DATA::DEV_CLOSE) {//check status
        pthread_mutex_lock(&d->mutexSyn);//lock data
        unsigned int size = (d->iSynStop >= d->iSynStart) ? (d->iSynStop - d->iSynStart) : (d->iSynStop + SYN_DATA_SIZE - d->iSynStart);//get size
        if (size > max_len) size = max_len;
        unsigned char *pData = (unsigned char *)ptr;
        for (unsigned int i = 0; i < size; i++) {
            pData[i] = d->synData[d->iSynStart];//fill data
            d->iSynStart = (d->iSynStart + 1) % SYN_DATA_SIZE;//increase index
        }
        pthread_mutex_unlock(&d->mutexSyn);//release data
        return size;
    }
    return 0;
}

void FTCardDriver::clearPendingSynData()
{
    //clear index
    d->iSynStart = 0; d->iSynStop = 0;
}

unsigned int FTCardDriver::setD11Direction(bool dir)
{
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE) {//check status
        return FT_STATUS_NOTOPEN;
    }
    writeBAR0(TTL_DIR_ADDR, dir ? 0x06 : 0x02);//set direction
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::writeTTLPort(unsigned char port, unsigned int value)
{
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE)//check status
        return FT_STATUS_NOTOPEN;
    if ((port < 9) || (port>12))//check port
        return FT_STATUS_PORT_INVALID;
    if (port == 9) {//D9
        value = (value & 0xffff);
        writeBAR0(TTL_OUT_BASE, value);
    }
    else if (port == 10) {//D10
        value = (value & 0xff) << 24;
        writeBAR0(TTL_OUT_BASE, value);
    }
    else if (port == 11) {//D11
        value = (value & 0xffff);
        writeBAR0(TTL_OUT_BASE + 4, value);
    }
    else if (port == 12) {//D12
        value = (value & 0xff) << 16;
        writeBAR0(TTL_OUT_BASE + 4, value);
    }
    return FT_STATUS_SUCCESS;
}

unsigned int FTCardDriver::readTTLPort(unsigned char port, unsigned int &value)
{
    if (d->deviceStatus == FT_CARD_DATA::DEV_CLOSE)//check status
        return FT_STATUS_NOTOPEN;
    if ((port < 9) || (port>12))//check port
        return FT_STATUS_PORT_INVALID;
    if (port == 9) {//D9
        readBAR0(TTL_IN_BASE, value);
        value = (value & 0xffff);
    }
    else if (port == 10) {//D10
        readBAR0(TTL_IN_BASE, value);
        value = (value & 0xff000000) >> 24;
    }
    else if (port == 11) {//D11
        readBAR0(TTL_IN_BASE + 4, value);
        value = (value & 0xffff);
    }
    else if (port == 12) {//D12
        readBAR0(TTL_IN_BASE + 4, value);
        value = (value & 0xff0000) >> 16;
    }
    return FT_STATUS_SUCCESS;
}

bool FTCardDriver::readBAR0(unsigned int offset, unsigned int &out)
{
    if (fid>=0) {//check device
        pthread_mutex_lock(&d->mutexDev);//lock mutex
        //prepare Bar0Cmd
        uint32_t tmp = 0;
        Bar0Cmd_t barCmd = {(uint32_t)offset,&tmp};
        //read
        int dw = ioctl(fid,FT_READ_BAR0_U32,&barCmd);
        out = tmp;
        pthread_mutex_unlock(&d->mutexDev);//release mutex
        return (dw == 0);
    }
    return false;
}

bool FTCardDriver::writeBAR0(unsigned int offset, unsigned int in)
{
    if (fid>=0) {//check device
        pthread_mutex_lock(&d->mutexDev);//lock mutex
        //prepare Bar0Cmd
        uint32_t tmp = (uint32_t)in;
        Bar0Cmd_t barCmd = {(uint32_t)offset,&tmp};
        //write
        int dw = ioctl(fid,FT_WRITE_BAR0_U32,&barCmd);
        pthread_mutex_unlock(&d->mutexDev);//release mutex
        return (dw == 0);
    }
    return false;
}

void *FTCardDriver::receiveFibre(void *lpara)
{
    FTCardDriver * pDriver = getInstance();//get driver instance

    //prepare
    unsigned char buffer[SINGLE_RECV_SIZE];
    IOCmd_t iocmd = {DMA_DATA,0,0,(void *)buffer};

    while (!pDriver->d->stopSignalFibre) {
        int count = read(pDriver->fid, &iocmd, SINGLE_RECV_SIZE);
        if (count>0) {
            //copy dma data to buffer
            pthread_mutex_lock(&pDriver->d->mutexFibre);//lock data
            unsigned int size = (pDriver->d->iFibreStop >= pDriver->d->iFibreStart) ? (pDriver->d->iFibreStop - pDriver->d->iFibreStart) : (pDriver->d->iFibreStop + FIBRE_DATA_SIZE - pDriver->d->iFibreStart);//get size
            if ((FIBRE_DATA_SIZE-size) > (unsigned int)count) {
                size = count;
            }
            else {
                size = FIBRE_DATA_SIZE - size-1;//can't fulfill buffer
            }
            for (unsigned int i = 0; i < size; i++) {
                pDriver->d->fibreData[pDriver->d->iFibreStop] = buffer[i];//copy data
                pDriver->d->iFibreStop = (pDriver->d->iFibreStop + 1) % FIBRE_DATA_SIZE;//increase index
            }
            pthread_mutex_unlock(&pDriver->d->mutexFibre);//release data
        }
    }

    return 0;
}

void *FTCardDriver::receiveSyn(void *lpara)
{
    FTCardDriver * pDriver = getInstance();//get driver instance

    //prepare
    unsigned int depth,dat;
    unsigned int buffer[1024];

    //read by dma
    while (!pDriver->d->stopSignalSyn) {
        if (pDriver->readBAR0(SYN_RX_CURNUM_ADDR, depth)) {
            if ((depth>0)&&(depth<=1024)) {
                for (unsigned int i = 0; i<depth; i++) {
                    //read data
                    if (pDriver->readBAR0(SYN_RX_DATA_ADDR, dat)) {
                        buffer[i] = dat;
                    }
                }
                //copy to buffer
                pthread_mutex_lock(&pDriver->d->mutexSyn);//lock data
                unsigned int size = (pDriver->d->iSynStop >= pDriver->d->iSynStart) ? (pDriver->d->iSynStop - pDriver->d->iSynStart) : (pDriver->d->iSynStop + SYN_DATA_SIZE - pDriver->d->iSynStart);//get size
                if ((SYN_DATA_SIZE - size) > (depth * 4)) size = depth * 4;
                else size = SYN_DATA_SIZE - size - 1;//can't fulfill buffer
                unsigned char *pData = (unsigned char *)buffer;
                for (unsigned int i = 0; i < size; i++) {
                    pDriver->d->synData[pDriver->d->iSynStop] = pData[i];//copy data
                    pDriver->d->iSynStop = (pDriver->d->iSynStop + 1) % SYN_DATA_SIZE;//increase index
                }
                pthread_mutex_unlock(&pDriver->d->mutexSyn);//release data
            }
        }
        usleep(10*1000);
    }

    return 0;
}
