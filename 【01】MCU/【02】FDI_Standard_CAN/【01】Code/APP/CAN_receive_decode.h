/*!
 *	\file		CAN_receive_decode.h
 *  \author		FDISYSTEMS 
 *	\date		05 February 2023
 *
 *	\brief		Contains main FDI_CAN methods.
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
 *	Welcome to the FDI_Standard_CAN  documentation.<br>
 *	This documentation describes all functions implemented in the FDI_CAN library.
 */

#ifndef CAN_RECEIVE_DECODE_H
#define CAN_RECEIVE_DECODE_H

#include "stm32f4xx_hal.h"
#include "stdint.h"  // 包含这个头文件以使用 int16_t


//进行结构体取消缺省对齐
#pragma pack(push, 1)
//1
__packed typedef struct
{
	int16_t		ANGLE_TRACK;
	int16_t 	ANGLE_SLIP;
	uint16_t 	CURVATURE_RADIUS;
	uint8_t 	AUTO_STATUS;
	uint8_t 	rev;
}FDI_ECAN_MSG_TRACK_SLIP_CURVATURE_t;
#define Fdican_Packet_ID_544		0x00000544

//2
__packed typedef struct
{
	uint16_t	TIME_OFFSET_0;
	uint16_t	TIME_OFFSET_1;
	uint16_t	TIME_OFFSET_2;
	uint16_t	TIME_OFFSET_3;
}FDI_ECAN_MSG_EVENT_TIME_E_t;
#define Fdican_Packet_ID_521		0x00000521

//3
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint32_t	STATUS;
}FDI_ECAN_MSG_EVENT_INFO_E_t;
#define Fdican_Packet_ID_520		0x00000520

//4
__packed typedef struct
{
	uint16_t	TIME_OFFSET_0;
	uint16_t	TIME_OFFSET_1;
	uint16_t	TIME_OFFSET_2;
	uint16_t	TIME_OFFSET_3;
}FDI_ECAN_MSG_EVENT_TIME_D_t;
#define Fdican_Packet_ID_519		0x00000519

//5
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint32_t	STATUS;
}FDI_ECAN_MSG_EVENT_INFO_D_t;
#define Fdican_Packet_ID_518		0x00000518

//6
__packed typedef struct
{
	uint16_t	TRUE_HEADING;
	uint16_t	TRUE_HEADING_ACC;
	int16_t		PITCH;
	uint16_t	PITCH_ACC;
}FDI_ECAN_MSG_GPS2_HDT_t;
#define Fdican_Packet_ID_393		0x00000393

//7
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	STATUS;
	uint16_t	rev;
}FDI_ECAN_MSG_GPS2_HDT_INFO_t;
#define Fdican_Packet_ID_392		0x00000392

//8
__packed typedef struct
{
	uint16_t	LATITUDE_ACC;
	uint16_t	LONGITUDE_ACC;
	uint16_t	ALTITUDE_ACC;
	uint16_t	BASE_STATION_ID;
}FDI_ECAN_MSG_GPS2_POS_ACC_t;
#define Fdican_Packet_ID_391		0x00000391

//9
__packed typedef struct
{
	int32_t		Altitude;
	int16_t		UNDULATION;
	uint8_t		NUM_SV;
	uint8_t		DIFF_CORR_AGE;

}FDI_ECAN_MSG_GPS2_POS_ALT_t;
#define Fdican_Packet_ID_390		0x00000390


//10
__packed typedef struct
{
	int32_t		LATITUDE;
	int32_t		LONGITUDE;
}FDI_ECAN_MSG_GPS2_POS_t;
#define Fdican_Packet_ID_389		0x00000389

//11
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint32_t	STATUS;
}FDI_ECAN_MSG_GPS2_POS_INFO_t;
#define Fdican_Packet_ID_388		0x00000388

//12
__packed typedef struct
{
	uint16_t	COURSE;
	uint16_t	COURSE_ACC;
	uint32_t	rev;
}FDI_ECAN_MSG_GPS2_VEL_ACC_t;
#define Fdican_Packet_ID_387		0x00000387

//13
__packed typedef struct
{
	int16_t		VELOCITY_ACC_N;
	int16_t		VELOCITY_ACC_E;
	int16_t		VELOCITY_ACC_D;
	int16_t		rev;
}FDI_ECAN_MSG_GPS2_VEL_COURSE_t;
#define Fdican_Packet_ID_386		0x00000386

//14
__packed typedef struct
{
	int16_t		VELOCITY_N;
	int16_t		VELOCITY_E;
	int16_t		VELOCITY_D;
	int16_t		rev;
}FDI_ECAN_MSG_GPS2_VEL_t;
#define Fdican_Packet_ID_385		0x00000385

//15
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint32_t	STATUS;
}FDI_ECAN_MSG_GPS2_VEL_INFO_t;
#define Fdican_Packet_ID_384		0x00000384

//16
__packed typedef struct
{
	uint16_t	TRUE_HEADING;
	uint16_t	TRUE_HEADING_ACC;
	int16_t		PITCH;
	uint16_t	PITCH_ACC;
}FDI_ECAN_MSG_GPS1_HDT_t;
#define Fdican_Packet_ID_377		0x00000377

//17
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	STATUS;
	uint16_t	rev;
}FDI_ECAN_MSG_GPS1_HDT_INFO_t;
#define Fdican_Packet_ID_376		0x00000376

//18
__packed typedef struct
{
	uint16_t	LATITUDE_ACC;
	uint16_t	LONGITUDE_ACC;
	uint16_t	ALTITUDE_ACC;
	uint16_t	BASE_STATION_ID;
}FDI_ECAN_MSG_GPS1_POS_ACC_t;
#define Fdican_Packet_ID_375		0x00000375

//19
__packed typedef struct
{
	int32_t		altitute;
	int16_t		UNDULATION;
	uint8_t		NUM_SV;
	uint8_t		DIFF_CORR_AGE;
}FDI_ECAN_MSG_GPS1_POS_ALT_t;
#define Fdican_Packet_ID_374		0x00000374

//20
__packed typedef struct
{
	int32_t		LATITUDE;
	int32_t		LONGITUDE;
}FDI_ECAN_MSG_GPS1_POS_t;
#define Fdican_Packet_ID_373		0x00000373

//21
__packed typedef struct
{
	uint32_t		TIME_STAMP;
	uint32_t		STATUS;
}FDI_ECAN_MSG_GPS1_POS_INFO_t;
#define Fdican_Packet_ID_372		0x00000372

//22
__packed typedef struct
{
	uint16_t		COURSE;
	uint16_t		COURSE_ACC;
}FDI_ECAN_MSG_GPS1_VEL_COURSE_t;
#define Fdican_Packet_ID_371		0x00000371

//23
__packed typedef struct
{
	int16_t		VELOCITY_ACC_N;
	int16_t		VELOCITY_ACC_E;
	int16_t		VELOCITY_ACC_D;
}FDI_ECAN_MSG_GPS1_VEL_ACC_t;
#define Fdican_Packet_ID_370		0x00000370

//24
__packed typedef struct
{
	int16_t		VELOCITY_N;
	int16_t		VELOCITY_E;
	int16_t		VELOCITY_D;
}FDI_ECAN_MSG_GPS1_VEL_t;
#define Fdican_Packet_ID_369		0x00000369

//25
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint32_t	STATUS;
}FDI_ECAN_MSG_GPS1_VEL_INFO_t;
#define Fdican_Packet_ID_368		0x00000368

//26
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint8_t		DEPTH_STATUS;
}FDI_ECAN_MSG_DEPTH_INFO_t;
#define Fdican_Packet_ID_358		0x00000358

//27
__packed typedef struct
{
	int32_t	PRESSURE_DIFF;
	int16_t	AIRSPEED;
}FDI_ECAN_MSG_AIR_DATA_AIRSPEED_t;
#define Fdican_Packet_ID_356		0x00000356

//28
__packed typedef struct
{
	uint32_t	PRESSURE_ABS;
	int32_t		Altitute;
}FDI_ECAN_MSG_AIR_DATA_ALTITUDE_t;
#define Fdican_Packet_ID_355		0x00000355

//29
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint8_t		AIR_DATA_STATUS;
	int16_t		AIR_TEMPERATURE;
}FDI_ECAN_MSG_AIR_DATA_INFO_t;
#define Fdican_Packet_ID_354		0x00000354

//30
__packed typedef struct
{
	int16_t		VELOCITY;
}FDI_ECAN_MSG_ODO_VEL_t;
#define Fdican_Packet_ID_353		0x00000353

//31
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	ODO_STATUS;
}FDI_ECAN_MSG_ODO_INFO_t;
#define Fdican_Packet_ID_352		0x00000352

//32
__packed typedef struct
{
	int16_t		ACCEL_X;
	int16_t		ACCEL_Y;
	int16_t		ACCEL_Z;
}FDI_ECAN_MSG_MAG_2_t;
#define Fdican_Packet_ID_338		0x00000338

//33
__packed typedef struct
{
	int16_t	MAG_X;
	int16_t	MAG_Y;
	int16_t	MAG_Z;
}FDI_ECAN_MSG_MAG_1_t;
#define Fdican_Packet_ID_337		0x00000337

//34
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	STATUS;
}FDI_ECAN_MSG_MAG_0_t;
#define Fdican_Packet_ID_336		0x00000336

//35
__packed typedef struct
{
	int16_t		VEL_X;
	int16_t		VEL_Y;
	int16_t		VEL_Z;
}FDI_ECAN_MSG_SHIP_MOTION_HP_2_t;
#define Fdican_Packet_ID_333		0x00000333

//36
__packed typedef struct
{
	int16_t		ACCEL_X;
	int16_t		ACCEL_Y;
	int16_t		ACCEL_Z;
}FDI_ECAN_MSG_SHIP_MOTION_HP_1_t;
#define Fdican_Packet_ID_332		0x00000332

//37
__packed typedef struct
{
	int16_t	SURGE;
	int16_t	SWAY;
	int16_t	HEAVE;
}FDI_ECAN_MSG_SHIP_MOTION_HP_0_t;
#define Fdican_Packet_ID_331		0x00000331

//38
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	PERIOD;
	uint16_t	STATUS;
}FDI_ECAN_MSG_SHIP_MOTION_HP_INFO_t;
#define Fdican_Packet_ID_330		0x00000330

//39
__packed typedef struct
{
	int16_t	VEL_X;
	int16_t	VEL_Y;
	int16_t	VEL_Z;
}FDI_ECAN_MSG_SHIP_MOTION_2_t;
#define Fdican_Packet_ID_329		0x00000329

//40
__packed typedef struct
{
	int16_t	ACCEL_X;
	int16_t	ACCEL_Y;
	int16_t	ACCEL_Z;
}FDI_ECAN_MSG_SHIP_MOTION_1_t;
#define Fdican_Packet_ID_325		0x00000325

//41
__packed typedef struct
{
	int16_t	SURGE;
	int16_t	SWAY;
	int16_t	HEAVE;
}FDI_ECAN_MSG_SHIP_MOTION_0_t;
#define Fdican_Packet_ID_321		0x00000321

//42
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	PERIOD;
	uint16_t	STATUS;
}FDI_ECAN_MSG_SHIP_MOTION_INFO_t;
#define Fdican_Packet_ID_320		0x00000320

//43
__packed typedef struct
{
	int16_t	VELOCITY_X;
	int16_t	VELOCITY_Y;
	int16_t	VELOCITY_Z;
}FDI_ECAN_MSG_EKF_VEL_BODY_t;
#define Fdican_Packet_ID_313		0x00000313

//44
__packed typedef struct
{
	int16_t	VELOCITY_ACC_N;
	int16_t	VELOCITY_ACC_E;
	int16_t	VELOCITY_ACC_D;
}FDI_ECAN_MSG_EKF_VEL_NED_ACC_t;
#define Fdican_Packet_ID_312		0x00000312

//45
__packed typedef struct
{
	int16_t	VELOCITY_N;
	int16_t	VELOCITY_E;
	int16_t	VELOCITY_D;
}FDI_ECAN_MSG_EKF_VEL_NED_t;
#define Fdican_Packet_ID_311		0x00000311

//46
__packed typedef struct
{
	int16_t	LATITUDE_ACC;
	int16_t	LONGITUDE_ACC;
	int16_t	ALTITUDE_ACC;
}FDI_ECAN_MSG_EKF_POS_ACC_t;
#define Fdican_Packet_ID_310		0x00000310

//47
__packed typedef struct
{
	int32_t	Altitute;
	int16_t	UNDULATION;
}FDI_ECAN_MSG_EKF_ALTITUDE_t;
#define Fdican_Packet_ID_309		0x00000309

//48
__packed typedef struct
{
	int32_t	LATITUDE;
	int32_t	LONGITUDE;
}FDI_ECAN_MSG_EKF_POS_t;
#define Fdican_Packet_ID_308		0x00000308

//49
__packed typedef struct
{
	int16_t	ROLL_ACC;
	int16_t	PITCH_ACC;
	int16_t	YAW_ACC;
}FDI_ECAN_MSG_EKF_ORIENTATION_ACC_t;
#define Fdican_Packet_ID_307		0x00000307

//50
__packed typedef struct
{
	int16_t	ROLL;
	int16_t	PITCH;
	int16_t	YAW;
}FDI_ECAN_MSG_EKF_EULER_t;
#define Fdican_Packet_ID_306		0x00000306

//51
__packed typedef struct
{
	int16_t	Q0;
	int16_t	Q1;
	int16_t	Q2;
	int16_t	Q3;
}FDI_ECAN_MSG_EKF_QUAT_t;
#define Fdican_Packet_ID_305		0x00000305

//52
__packed typedef struct
{
	int32_t	TIME_STAMP;
}FDI_ECAN_MSG_EKF_INFO_t;
#define Fdican_Packet_ID_304		0x00000304

//53
__packed typedef struct
{
	int16_t	DELTA_ANGLE_X;
	int16_t	DELTA_ANGLE_Y;
	int16_t	DELTA_ANGLE_Z;
}FDI_ECAN_MSG_IMU_DELTA_ANGLE_t;
#define Fdican_Packet_ID_292		0x00000292

//54
__packed typedef struct
{
	int16_t	DELTA_VEL_X;
	int16_t	DELTA_VEL_Y;
	int16_t	DELTA_VEL_Z;
}FDI_ECAN_MSG_IMU_DELTA_VEL_t;
#define Fdican_Packet_ID_291		0x00000291

//55
__packed typedef struct
{
	int16_t	GYRO_X;
	int16_t	GYRO_Y;
	int16_t	GYRO_Z;
}FDI_ECAN_MSG_IMU_GYRO_t;
#define Fdican_Packet_ID_290		0x00000290

//56
__packed typedef struct
{
	int16_t	ACCEL_X;
	int16_t	ACCEL_Y;
	int16_t	ACCEL_Z;
}FDI_ECAN_MSG_IMU_ACCEL_t;
#define Fdican_Packet_ID_289		0x00000289

//57
__packed typedef struct
{
	uint32_t	TIME_STAMP;
	uint16_t	STATUS;
	int16_t		TEMPERATURE;
}FDI_ECAN_MSG_IMU_INFO_t;
#define Fdican_Packet_ID_288		0x00000288

//58
__packed typedef struct
{
	uint8_t YEAR;
	uint8_t MONTH;
	uint8_t DAY;
	uint8_t HOUR;
	uint8_t MIN;
	uint8_t SEC;
	uint16_t MICRO_SEC;
}FDI_ECAN_MSG_UTC_1_t;
#define Fdican_Packet_ID_273		0x00000273

//59
__packed typedef struct
{
	uint32_t TIME_STAMP;
	uint32_t GPS_TOW;
}FDI_ECAN_MSG_UTC_0_t;
#define Fdican_Packet_ID_272		0x00000272

//60
__packed typedef struct
{
	uint32_t SOLUTION;
	uint32_t HEAVE_STATUS;
}FDI_ECAN_MSG_STATUS_03_t;
#define Fdican_Packet_ID_258		0x00000258

//61
__packed typedef struct
{
	uint32_t COM;
	uint32_t AIDING;
}FDI_ECAN_MSG_STATUS_02_t;
#define Fdican_Packet_ID_257		0x00000257

//62
__packed typedef struct
{
	uint32_t TIME_STAMP;
	uint16_t GENERAL;
	uint16_t CLOCK;
}FDI_ECAN_MSG_STATUS_01_t;
#define Fdican_Packet_ID_256		0x00000256
//释放缺省对其
#pragma pack(pop)
#endif

