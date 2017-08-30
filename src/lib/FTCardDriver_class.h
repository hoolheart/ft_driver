#ifndef FT_CARD_DRIVER_CLASS_H__
#define FT_CARD_DRIVER_CLASS_H__

#include "FTCardLib.h"

struct FT_CARD_DATA;

/** driver api of fibre-test card */
class FTCardDriver {

private:
    int fid;
	struct FT_CARD_DATA * d;
	static FTCardDriver * instance;/**< singleton instance */

private:
	FTCardDriver();

public:
	~FTCardDriver();

	static FTCardDriver * getInstance();/**< get driver interface instance */

	unsigned int open();/**< open fibre-test card */
	unsigned int close();/**< close fibre-test card */
	unsigned int reset();/**< reset fibre-test card */

	unsigned int setParam(unsigned char id, int value);/**< set parameter */
	unsigned int startTask();/**< start task */
	unsigned int sendFibreData(void * ptr, unsigned int len);/**< send fibre data */
	unsigned int getPendingFibreSize();/**< get size of pending fibre data */
	unsigned int receiveFibreData(void * ptr, unsigned int max_len);/**< receive fibre data */
	void clearPendingFibreData();/**< clear pending fibre data */
	unsigned int stopTask();/**< stop task */

	unsigned int sendSynData(unsigned char chl, SYN_FREQ freq, void * ptr, unsigned int len);/**< send synchronization serial port data */
	unsigned int getPendingSynSize();/**< get size of pending synchronization serial port data */
	unsigned int receiveSynData(void * ptr, unsigned int max_len);/**< receive synchronization serial port data */
	void clearPendingSynData();/**< clear pending synchronization serial port data */

	unsigned int setD11Direction(bool dir);/**< set TTL port D11 direction, false:input, true:output */
	unsigned int writeTTLPort(unsigned char port, unsigned int value);/**< write TTL port */
	unsigned int readTTLPort(unsigned char port, unsigned int &value);/**< read TTL port */

private:
	/************ functions to operate device *************/
	bool readBAR0(unsigned int offset, unsigned int& out);
	bool writeBAR0(unsigned int offset, unsigned int in);
	/************ functions of receiving threads **********/
	static void *receiveFibre(void* lpara);
	static void *receiveSyn(void* lpara);
};

#endif //FT_CARD_DRIVER_CLASS_H__
