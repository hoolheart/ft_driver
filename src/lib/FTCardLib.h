#ifndef __FT_CARD_LIB_H__
#define __FT_CARD_LIB_H__

/** export macro */
#ifdef _WIN32//windows
#ifdef FTCARDDRIVER_EXPORTS
#define FTCARDDRIVER_API __declspec(dllexport)
#else
#define FTCARDDRIVER_API __declspec(dllimport)
#endif
#define CALL_METHOD _stdcall
#else//linux
#define FTCARDDRIVER_API
#define CALL_METHOD
#endif//systems

/** return status */
enum FT_CARD_STATUS {
	FT_STATUS_SUCCESS = 0,/**< operation success */
	FT_STATUS_WDCERR,/**< failed to open WinDriver in debug mode */
	FT_STATUS_DEVICENOTFOUND,/**< device cannot be found */
	FT_STATUS_OPEN_FAILED,/**< failed to open device */
	FT_STATUS_ENABLEINT_FAILED,/**< failed to enable interrupt */
	FT_STATUS_NOTOPEN,/**< device is not opened */
	FT_STATUS_PARAMID_INVALID,/**< parameter id is invalid (see #FT_CARD_PARAM_ID) */
	FT_STATUS_PARAMVAL_INVALID,/**< parameter value is invalid */
	FT_STATUS_TASKINGOING,/**< task is currently going */
	FT_STATUS_TASKNOTSTARTED,/**< task has not been started */
	FT_STATUS_FIBREDATA_TOOLONG,/**< output fibre data is too long (bigger than 8KB) */
	FT_STATUS_TIMEOUT,/**< operation time-out */
	FT_STATUS_SYNCHL_INVALID,/**< synchronization serial port channel is invalid (1-3) */
	FT_STATUS_SYNDATA_SIZEWRONG,/**< output synchronization data size if wrong (1-256) */
	FT_STATUS_SYNFREQ_INVALID,/**< frequency setting of synchronization serial port is invalid (see #SYN_FREQ) */
	FT_STATUS_PORT_INVALID,/**< TTL port is invalid (9-12) */

	FT_STATUS_END
};

/** parameter id */
enum FT_CARD_PARAM_ID {
	FT_PARAMID_FIBRECHL = 0,/**< fibre channel, default:1, valid:1-4 */
	FT_PARAMID_TRIGGER,/**< trigger source, 0:inner, 1:outer */
	FT_PARAMID_LEAD,/**< fibre lead, 0:don't send, 1:send */
	FT_PARAMID_CPINUM,/**< cpi number 1-5 */
	FT_PARAMID_PRF,/**< pulse repeat frequency (20e6/prf) */
	FT_PARAMID_CPI_PULSENUM,/**< pulse number of one cpi */
	FT_PARAMID_DELAY1,/**< FR delay (0.1us) */
	FT_PARAMID_DELAY2,/**< CPI delay (0.1us) */
	FT_PARAMID_DELAY3,/**< BW delay (0.1us) */
	FT_PARAMID_DELAY4,/**< TRT delay (0.1us) */
	FT_PARAMID_DELAY5,/**< TRW delay (0.1us) */
	FT_PARAMID_DELAY6,/**< Reserved */
	FT_PARAMID_DELAY7,/**< Reserved */
	FT_PARAMID_DELAY8,/**< Fibre1 delay (0.1us) */
	FT_PARAMID_DELAY9,/**< Fibre2 delay (0.1us) */
	FT_PARAMID_DELAY10,/**< Fibre3 delay (0.1us) */
	FT_PARAMID_DELAY11,/**< Fibre4 delay (0.1us) */
	FT_PARAMID_DELAY12,/**< Serial1 delay (0.1us) */
	FT_PARAMID_DELAY13,/**< Serial2 delay (0.1us) */
	FT_PARAMID_DELAY14,/**< Serial3 delay (0.1us) */
	FT_PARAMID_PULSEWIDTH1,/**< FR pulse width */
	FT_PARAMID_PULSEWIDTH2,/**< CPI pulse width */
	FT_PARAMID_PULSEWIDTH3,/**< BW pulse width */
	FT_PARAMID_PULSEWIDTH4,/**< TRT pulse width */
	FT_PARAMID_PULSEWIDTH5,/**< TRW pulse width */
	FT_PARAMID_PULSEDIR1,/**< FR pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR2,/**< CPI pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR3,/**< BW1 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR4,/**< ENABLE1 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR5,/**< SD1 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR6,/**< SC1 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR7,/**< TRT pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR8,/**< TRR pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR9,/**< BW2 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR10,/**< ENABLE2 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR11,/**< SD2 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR12,/**< SC2 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR13,/**< InputTrigger pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR14,/**< ENABLE3 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR15,/**< SD3 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR16,/**< SC3 pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR17,/**< SD pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR18,/**< SC pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR19,/**< END pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR20,/**< SYN pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR21,/**< TRT pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR22,/**< TRW pulse direction, 0:negative, 1:positive */
	FT_PARAMID_PULSEDIR23,/**< DarkRoom pulse direction, 0:negative, 1:positive */

	FT_PARAMID_END
};

/** frequency setting of synchronization serial port */
enum SYN_FREQ {
	FT_SYNFREQ_500K = 0,
	FT_SYNFREQ_1M,
	FT_SYNFREQ_5M,
	FT_SYNFREQ_10M,
	FT_SYNFREQ_20M,

	FT_SYNFREQ_END
};

/**********************interface functions ************************/

/** 
 * @brief open device
 * @return see #FT_CARD_STATUS
 */
FTCARDDRIVER_API unsigned int CALL_METHOD FT_OpenDevice();
/**
* @brief close device
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_CloseDevice();
/**
* @brief reset device
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_ResetDevice();

/**
* @brief set fibre testing parameter
* @param id - id of parameter, see #FT_CARD_PARAM_ID
* @param value - value of parameter
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_SetParam(unsigned char id, int value);
/**
* @brief start task
* @param simulation_enable - enable flag of simulation, 0 for fibre mode, 1 for simulation mode
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_StartTask(int simulation_enable);
/**
* @brief send fibre data
* @param ptr - pointer of data
* @param len - length of data, unit:Byte (<8K)
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_SendFibreData(void * ptr, unsigned int len);
/**
* @brief receive fibre data
* @param ptr - pointer of data
* @param max_len - maximum length of data, unit:Byte
* @return actually read data length, unit:Byte
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_ReceiveFibreData(void * ptr, unsigned int max_len);
/**
* @brief stop task
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_StopTask();

/**
* @brief send synchronization serial port data
* @param chl - synchronization serial port channel, valid only for 1-3
* @param freq - frequency setting
* @param ptr - pointer of data
* @param len - length of data, unit:bit (1-256)
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_SendSynData(unsigned char chl, unsigned int freq, void * ptr, unsigned int len);
/**
* @brief get size of pending synchronization serial port data
* @return size of pending synchronization serial port data, unit:Byte
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_GetPendingSynSize();
/**
* @brief receive synchronization serial port data
* @param ptr - pointer of data
* @param max_len - maximum length of data, unit:Byte
* @return actually read data length, unit:Byte
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_ReceiveSynData(void * ptr, unsigned int max_len);
/**
* @brief clear pending synchronization serial port data
*/
FTCARDDRIVER_API void CALL_METHOD FT_ClearPendingSynData();

/**
* @brief set TTL port D11 direction
* @param dir - direction, 0:input, 1:output
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_SetD11Direction(unsigned int dir);
/**
* @brief write TTL port
* @param port - TTL port, valid only for 9-12
* @param value - output value
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_WriteTTLPort(unsigned char port, unsigned int value);
/**
* @brief read TTL port
* @param port - TTL port, valid only for 9-12
* @param value - pointer of input value
* @return see #FT_CARD_STATUS
*/
FTCARDDRIVER_API unsigned int CALL_METHOD FT_ReadTTLPort(unsigned char port, unsigned int *value);

#endif /* __FT_CARD_LIB_H__ */
