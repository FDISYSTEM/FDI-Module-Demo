/*!
 *	\file		CAN_receive.h
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
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd

#define TYPE_IMU 			0x40
#define TYPE_AHRS 		0x41
#define TYPE_INSGPS 	0x42
#define TYPE_SYSSTATE 0x50
#define TYPE_FORTIME  0x52


#define IMU_LEN  			0x38   //56+8  8组数据
#define AHRS_LEN		  0x30   //48+8  7组数据
#define INSGPS_LEN	  0x48 	 //72+8  10组数据
#define SYS_STATE_LEN 0x64
#define FORTIME_LEN 	0x0E

#define SYS_STATE_CAN 15
#define IMU_CAN 9
#define AHRS_CAN 8
#define INSGPS_CAN 11
#define FORTIME_CAN 5


#endif

