/*!
 *	\file		FDILink.h
 *  \author		FDISYSTEMS 
 *	\date		05 February 2023
 *
 *	\brief		Contains main FDILink methods.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2018-2023, FDISYSTEMS  . All rights reserved.
 *	
 *	This source code is intended for use only by FDISYSTEMS and
 *	those that have explicit written permission to use it from
 *	FDISYSTEMS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */

/*!
 *	\mainpage FDISYSTEMS Communication library documentation
 *	Welcome to the FDILink library documentation.<br>
 *	This documentation describes all functions implemented in the FDILink library.
 */
#ifndef __SERIAL_BOOT_H
#define __SERIAL_BOOT_H
#include <stdint.h>
//未初始化
#define FDILink_Status_Uninitialized 		0
//运行
#define FDILink_Status_Running 					2
//正忙
#define FDILink_Status_Busy 				3

#define FDILink_Frame_Start					0
#define FDILink_Frame_CMD						1
#define FDILink_Frame_Length				2
#define FDILink_Frame_SerialNumber	3
#define FDILink_Frame_CRC8					4
#define FDILink_Frame_CRC16H				5
#define FDILink_Frame_CRC16L				6
#define FDILink_Frame_Data					7
#define FDILink_Frame_End						8


/***************************************************************
*	帧符号
***************************************************************/

#define FDILink_STX_Flag 0xFC
#define FDILink_EDX_Flag 0xFD

typedef struct
{
	uint8_t Start;
	uint8_t CMD;
	uint8_t Length;
	uint8_t SerialNumber;
	uint8_t CRC8;
	union
	{
		uint16_t CRC16;
		struct
		{
			uint8_t CRC16L;
			uint8_t CRC16H;
		};
	};
	uint8_t Data[0];
}__attribute__((packed)) FDILink_Frame_t;

typedef struct FDILink_Status
{
	int 				BootStatus;
	int					RxStatus;
	int 				RxType;
	int 				RxDataLeft;
	int		 			RxNumber;
	int		 			TxNumber;
	int 				CRC8_Verify;
	int 				CRC16_Verify;
	uint32_t 			BufferIndex;
	uint8_t 			FDILink_Frame_Buffer[12];
	uint8_t 			Buffer[256];
}FDILink_Status_t;

#define FDI_NO_ERROR 0
#define FDI_ERROR    1

int fdiComProtocolInit(FDILink_Status_t* FDILink);
/*
fdiComBufferTrans函数：
介绍：结构体封装函数，将数据封装成FDILink可以解析的帧格式，存放在buffer里面，返回帧的总长度
参数列表：
buffer		---	缓冲区，一般是256字节的数组(uint8_t Buffer[256])，函数处理完后存放FDILink帧
FDILink		---	FDILink类型的结构体，可以定义一个全局变量
type			---	数据类型，可以在手册或者fdilink_decode.h的输入部分查找自己需要使用的数据帧
buf				---	载荷，存放有效数据
len				---	载荷长度
*/
int fdiComBufferTrans(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len);
/*
fdiComProtocolReceive函数：
介绍：处理接收到的数据块，逐字节遍历并判断其是否符合FDILink帧格式，一般放在回调函数里面
函数调度：fdiComProtocolReceive ---> fdiPackRecelveData ---> fdiRuningReceiveData
*/
int fdiComProtocolReceive(FDILink_Status_t* FDILink, uint8_t * buf, int len);

#endif
