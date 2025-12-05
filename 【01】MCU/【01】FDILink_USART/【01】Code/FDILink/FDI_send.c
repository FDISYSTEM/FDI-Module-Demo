#include "stm32f1xx_hal.h"
#include "string.h"
#include "fdilink_decode.h"
#include "FDILink.h"
#include "FDI_send.h"

//----------------------------------------------------------------------//
//				          - FDI_LINK Packet sending protocol                 -//
//----------------------------------------------------------------------//
//--TEST_2025
extern UART_HandleTypeDef huart1;
static FDILink_Status_t _FDILink;

/*!
 *  请求数据帧并解析，ID为要请求的数据帧ID编号。系统会返回当前时刻对应数据输出，如果该帧被设置为固定频率输出，则会持续输出。
 *  使用此指令会自动解析要获得的数据至构建好的结构体中，fdiDecodeBuffer中配置了部分数据包，用户可自行添加需要解析的数据包
 *	Request and analysis data frame.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiGetPacket(uint8_t ID)
{
	uint8_t p_buff[256];
	uint8_t buffer[4];
	buffer[0] = ID;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	fdiComBufferTrans(p_buff, &_FDILink, 0xA0, buffer, sizeof(buffer));
	for(int i = 0;i < 10;i++)
	{
		//请求ID发送
		HAL_Delay(100);
		HAL_UART_Transmit_IT(&huart1, p_buff, 12);
	}
	return 0;
}

/*!
 *  提供一个示例，演示用户创建外部输入数据帧并向模块发送
 *	Configure external SLAM1 packets. All external packets are configured in this format.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		None
 */
void fdiSendSLAM1Datapacke(void)
{
	External_SLAM1_Packet_t SLAMData;
	
	SLAMData.Position_X 				   = 1.23456;
	SLAMData.Position_Y 				   = 2.23456;
	SLAMData.Position_Z 				   = 3.23456;
	SLAMData.Velocity_X 				   = 4.23456;
	SLAMData.Velocity_Y 				   = 5.23456;
	SLAMData.Velocity_Z					   = 6.23456;
	SLAMData.Roll       				   = 7.23456;
	SLAMData.Pitch     					   = 8.23456;
	SLAMData.Yaw        				   = 9.23456;
	SLAMData.Position_X_standard_deviation = 0.23456;
	SLAMData.Position_Y_standard_deviation = 0.23456;
	SLAMData.Position_Z_standard_deviation = 0.23456;
	SLAMData.Velocity_X_standard_deviation = 0.23456;
	SLAMData.Velocity_Y_standard_deviation = 0.23456;
	SLAMData.Velocity_Z_standard_deviation = 0.23456;
	SLAMData.Roll_standard_deviation       = 2.23456;
	SLAMData.Pitch_standard_deviation      = 2.23456;
	SLAMData.Yaw_standard_deviation        = 2.23456;
	
	fdiSendExternalData(External_SLAM1_Packet_ID, &SLAMData, sizeof(External_SLAM1_Packet_t));
}

/*!
 * 	封装外部输入数据帧，通过串口发送到模组
 *	Send configured external packets.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiSendExternalData(int type, void* buffer, int length)
{
	uint8_t buf[256];
	fdiComBufferTrans(buf, &_FDILink, type, buffer, length);
	HAL_Delay(10);
	HAL_UART_Transmit_IT(&huart1, buf, strlen((char*)buf));
	HAL_Delay(100);
	return FDI_NO_ERROR;
}
