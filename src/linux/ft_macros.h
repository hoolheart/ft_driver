#ifndef __FT_MACROS_H__
#define __FT_MACROS_H__

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

#endif//__FT_MACROS_H__
