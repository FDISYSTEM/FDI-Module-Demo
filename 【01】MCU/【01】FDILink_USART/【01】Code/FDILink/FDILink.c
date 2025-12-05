#include "FDILink.h"
#include "string.h"

//----------------------------------------------------------------------//
//				          - FDI_LINK Function file                     -//
//----------------------------------------------------------------------//
extern uint8_t CRC8_Table(uint8_t* p, uint8_t counter);
extern uint16_t CRC16_Table(uint8_t* p, uint8_t counter);
extern void fdiDecodeBuffer(int PACKET_ID,void* buf);

#ifndef FDI_ASSERT
#define FDI_ASSERT(x)
#endif

/**
  * @brief Initialize the FDILink status structure.
	* @return return 0 (always).
  */
int fdiComProtocolInit(FDILink_Status_t* FDILink)
{
	FDILink->BufferIndex = 0;
	FDILink->BootStatus = FDILink_Status_Running;
	FDILink->RxStatus = FDILink_Frame_Start;
	for (int i = 0; i < 256; i++)
	{
		FDILink->Buffer[i] = 0;
	}
	return 0;
}

/**
  * @brief Pack the data into the FDILink buffer for transmission.
  * @param[out] buffer Pointer to the buffer to store the packed data.
	* @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] len Length of the data buffer.	(0-255)
	* @param[in] type Data type.
	* @param[in] buf Pointer to the data buffer.
	*	@param[in] len Length of the data buffer.
	* @return Length of the packed data 
  */
int fdiComBufferTrans(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len)
{
	FDI_ASSERT(len < 248);
	buffer[FDILink_Frame_Start] = FDILink_STX_Flag;
	buffer[FDILink_Frame_CMD] = type;
	buffer[FDILink_Frame_Length] = len;
	buffer[FDILink_Frame_SerialNumber] = FDILink->TxNumber++;
	uint8_t CRC8 = CRC8_Table(buffer, FDILink_Frame_CRC8);
	buffer[FDILink_Frame_CRC8] = CRC8;

	uint8_t* buf_data = buffer + FDILink_Frame_Data;
	memcpy(buf_data,buf,len);
	uint16_t CRC16 = CRC16_Table(buf_data, len);
	buffer[FDILink_Frame_CRC16H] = (CRC16 >> 8);
	buffer[FDILink_Frame_CRC16L] = (CRC16 & 0xff);
	buffer[FDILink_Frame_End + len - 1] = FDILink_EDX_Flag;
	return FDILink_Frame_End + len;
}

/**
  * @brief Reset the FDILink status.
  * @param[in] FDILink Pointer to the FDILink status structure.
  */
void fdiResetAll(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
	FDILink->RxDataLeft = 0;
	FDILink->RxType = 0;
	FDILink->BufferIndex = 0;
	FDILink->TxNumber = 0;
}

/**
  * @brief Reset the FDILink error status.
  * @param[in] FDILink Pointer to the FDILink status structure.
  */
void fdiErrorOccurred(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
}

/**
  * @brief Insert a value into the FDILink buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] value Value to be inserted.
  */
void fdiInsertBuffer(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxDataLeft <= 0)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	if (FDILink->RxStatus != FDILink_Frame_Data)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	FDILink->Buffer[FDILink->BufferIndex++] = value;
	if (FDILink->BufferIndex >= 256)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	FDILink->RxDataLeft--;
}

/**
  * @brief Handle the running data received in the FDILink communication.
  * @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] value Received value.
	*	@return 0 for success, -3 if invalid RxStatus, -1 for error, 1 for complete frame received.
  */
int fdiRuningReceiveData(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxStatus < FDILink_Frame_Start || FDILink->RxStatus > FDILink_Frame_End)
	{
		fdiErrorOccurred(FDILink);
		return -3;
	}
	FDILink->FDILink_Frame_Buffer[FDILink->RxStatus] = value;
	switch (FDILink->RxStatus)
	{
		case FDILink_Frame_Start:
			fdiResetAll(FDILink);
			if (value != FDILink_STX_Flag)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_CMD;
			break;
		case FDILink_Frame_CMD:
			FDILink->RxType = value;
			FDILink->RxStatus = FDILink_Frame_Length;
			break;
		case FDILink_Frame_Length:
			FDILink->RxDataLeft = value;
			FDILink->RxStatus = FDILink_Frame_SerialNumber;
			break;
		case FDILink_Frame_SerialNumber:
			FDILink->RxNumber = value;
			FDILink->RxStatus = FDILink_Frame_CRC8;
			break;
		case FDILink_Frame_CRC8:
			FDILink->CRC8_Verify = value;
			if (CRC8_Table(FDILink->FDILink_Frame_Buffer, FDILink_Frame_CRC8) != FDILink->CRC8_Verify)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			if(FDILink->RxDataLeft == 0)
			{
				FDILink->RxStatus = FDILink_Frame_Start;
				return 1;
			}
			FDILink->RxStatus = FDILink_Frame_CRC16H;
			break;
		case FDILink_Frame_CRC16H:
			FDILink->CRC16_Verify = value;
			FDILink->RxStatus = FDILink_Frame_CRC16L;
			break;
		case FDILink_Frame_CRC16L:
			FDILink->CRC16_Verify = (FDILink->CRC16_Verify << 8) | value;
			FDILink->RxStatus = FDILink_Frame_Data;
			break;
		case FDILink_Frame_Data:
			if (FDILink->RxDataLeft)
			{
				fdiInsertBuffer(FDILink,value);
				if (FDILink->RxDataLeft == 0)
				{
					FDILink->RxStatus = FDILink_Frame_End;
				}
				break;
			}
			else
			{
				FDILink->RxStatus = FDILink_Frame_End;
			}

		case FDILink_Frame_End:
		{
			if (value != FDILink_EDX_Flag)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			uint16_t CRC16 = CRC16_Table(FDILink->Buffer, FDILink->BufferIndex);
			if (CRC16 != FDILink->CRC16_Verify)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_Start;
			return 1;
		}
		default:
			fdiErrorOccurred(FDILink);
			return -1;
	}
	return 0;
}

/**
  * @brief Pack the data into a FDI link frame.
  * @param[out] buffer Pointer to the output buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
	* @param[in] type Frame type.
	* @param[in] buf Pointer to the data buffer.
	*	@param[in] len Length of the data buffer.
	* @return  Size of the packed frame.
  */
static inline int fdiPackRecelveData(FDILink_Status_t* FDILink, uint8_t value)
{
	FDI_ASSERT(FDILink->BootStatus == FDILink_Status_Running);
	uint8_t result = fdiRuningReceiveData(FDILink, value);
	if (result == 1)
	{
		fdiDecodeBuffer(FDILink->RxType,FDILink->Buffer);
	}
	return result;
}

/**
  * @brief Receive data into the FDILink buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
	* @param[in] buf Pointer to the received data buffer.
	*	@param[in] len Length of the received data buffer.
	* @return return 0 (always).
  */
int fdiComProtocolReceive(FDILink_Status_t* FDILink, uint8_t * buf, int len)
{
	for(int i = 0;i < len;i++)
	{
		fdiPackRecelveData(FDILink, buf[i]);
	}
	return 0;
}

