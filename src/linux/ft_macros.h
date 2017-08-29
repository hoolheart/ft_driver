#ifndef __FT_MACROS_H__
#define __FT_MACROS_H__
#include <stdint.h>

//driver name
#define DRIVER_NAME "FIBRE_TEST"
#define BOARD_NAME "pcie_ft1"

//IO COMMAND TYPE
enum FPGA_IO_CMD {
    BAR_IO = 0,
    BAR_IO_REPEAT,
    DMA_DATA,
    
    FPGA_IO_CMD_END
};

/*I/0 - should move to separate file at some point */
struct IOCmd_t {
    uint32_t cmd; //command word see @FPGA_IO_CMD
    uint8_t barNum; //which bar we are read/writing to
    uint32_t devAddr; // address relative to BAR that we are read/writing to
    void * userAddr; // virtual address in user space to read/write from
};

//address
#define CPI_FREQ_BASE       0X210//task parameters
#define CPI_PULSE_NUM_BASE  0X230
#define CPI_NUM_ADDR        0X250
#define DELAY_BASE          0X260
#define PULSE_WIDTH_BASE    0X2B0
#define PULSE_DIR_ADDR      0X2CC//pulse direction
#define FIBRE_LEAD_ADDR     0X134//lead
#define TRIGGER_ADDR        0X138//trigger setting
#define INNER_TRIGGER_ADDR  0X200//inner trigger
#define SEND_ADDR           0X204//send command
#define SYN1_TX_CYCLE_ADDR  0X100//synchronization serial port 1 transmit
#define SYN1_TX_NUM_ADDR    0X104
#define SYN1_TX_DATA_ADDR   0X108
#define SYN1_TX_TRIG_ADDR   0X10C
#define SYN2_TX_CYCLE_ADDR  0X110//synchronization serial port 2 transmit
#define SYN2_TX_NUM_ADDR    0X114
#define SYN2_TX_DATA_ADDR   0X118
#define SYN2_TX_TRIG_ADDR   0X11C
#define SYN3_TX_CYCLE_ADDR  0X150//synchronization serial port 3 transmit
#define SYN3_TX_NUM_ADDR    0X154
#define SYN3_TX_DATA_ADDR   0X158
#define SYN3_TX_TRIG_ADDR   0X15C
#define SYN_RX_NUM_ADDR     0X120//synchronization serial port receive
#define SYN_RX_STATE_ADDR   0X124
#define SYN_RX_CURNUM_ADDR  0X128
#define SYN_RX_DATA_ADDR    0X12C
#define SYN_RX_CLEAR_ADDR   0X130
#define TTL_DIR_ADDR        0X13C//ttl
#define TTL_IN_BASE         0X140
#define TTL_OUT_BASE        0X148

//signal
#define TRIGGER_SIGNAL (SIGRTMIN+1)

//ioctl commands
enum FT_IOCTL_CMD {
    FT_RESET = 0,//reset FPGA
    FT_READ_BAR0_U32,//read an u32 data from bar0
    FT_WRITE_BAR0_U32,//write an u32 data to bar0

    FT_IOCTL_CMD_END
};

//struct for FT_READ_BAR0_U32 and FT_WRITE_BAR0_U32
struct Bar0Cmd_t {
    uint32_t addr;
    uint32_t *value;
};

#endif//__FT_MACROS_H__
