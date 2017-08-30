#include "FTCardDriver.h"
#include "FTCardDriver_class.h"

unsigned int CALL_METHOD FT_OpenDevice()
{
	return FTCardDriver::getInstance()->open();
}

unsigned int CALL_METHOD FT_CloseDevice()
{
	return FTCardDriver::getInstance()->close();
}

unsigned int CALL_METHOD FT_ResetDevice()
{
	return FTCardDriver::getInstance()->reset();
}

unsigned int CALL_METHOD FT_SetParam(unsigned char id, int value)
{
	return FTCardDriver::getInstance()->setParam(id, value);
}

unsigned int CALL_METHOD FT_StartTask()
{
	return FTCardDriver::getInstance()->startTask();
}

unsigned int CALL_METHOD FT_SendFibreData(void * ptr, unsigned int len)
{
	return FTCardDriver::getInstance()->sendFibreData(ptr, len);
}

unsigned int CALL_METHOD FT_GetPendingFibreSize()
{
	return FTCardDriver::getInstance()->getPendingFibreSize();
}

unsigned int CALL_METHOD FT_ReceiveFibreData(void * ptr, unsigned int max_len)
{
	return FTCardDriver::getInstance()->receiveFibreData(ptr, max_len);
}

void CALL_METHOD FT_ClearPendingFibreData()
{
	FTCardDriver::getInstance()->clearPendingFibreData();
}

unsigned int CALL_METHOD FT_StopTask()
{
	return FTCardDriver::getInstance()->stopTask();
}

unsigned int CALL_METHOD FT_SendSynData(unsigned char chl, unsigned int freq, void * ptr, unsigned int len)
{
	return FTCardDriver::getInstance()->sendSynData(chl, (SYN_FREQ)freq, ptr, len);
}

unsigned int CALL_METHOD FT_GetPendingSynSize()
{
	return FTCardDriver::getInstance()->getPendingSynSize();
}

unsigned int CALL_METHOD FT_ReceiveSynData(void * ptr, unsigned int max_len)
{
	return FTCardDriver::getInstance()->receiveSynData(ptr, max_len);
}

void CALL_METHOD FT_ClearPendingSynData()
{
	FTCardDriver::getInstance()->clearPendingSynData();
}

unsigned int CALL_METHOD FT_SetD11Direction(unsigned int dir)
{
	return FTCardDriver::getInstance()->setD11Direction(dir != 0);
}

unsigned int CALL_METHOD FT_WriteTTLPort(unsigned char port, unsigned int value)
{
	return FTCardDriver::getInstance()->writeTTLPort(port, value);
}

unsigned int CALL_METHOD FT_ReadTTLPort(unsigned char port, unsigned int *value)
{
	return FTCardDriver::getInstance()->readTTLPort(port, (*value));
}
