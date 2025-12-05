/**
  ******************************************************************************
  * @file           : CAN_receive.c
  * @brief          : CAN program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023   FDI company
  * All rights reserved.
  *
  * CAN communication routine file, changes can be added.
  *
  ******************************************************************************
  */
	
#include "main.h"
#include "CAN_receive_decode.h"
#include "CAN_receive.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void FDI_Extended_CAN_CallBack(uint32_t ExtId , uint8_t *Buffer);
void FDI_Extended_CAN_Request(int request_ID_1,int request_ID_2,int request_ID_3,int request_ID_4);

//接收数据结构体
FDI_ECAN_MSG_TRACK_SLIP_CURVATURE_t		FDI_ECAN_MSG_TRACK_SLIP_CURVATURE;
FDI_ECAN_MSG_EVENT_TIME_E_t		FDI_ECAN_MSG_EVENT_TIME_E;
FDI_ECAN_MSG_EVENT_INFO_E_t		FDI_ECAN_MSG_EVENT_INFO_E;
FDI_ECAN_MSG_EVENT_TIME_D_t		FDI_ECAN_MSG_EVENT_TIME_D;

uint8_t Request_ID_list[8]; 
#define FDI_REQUREST_ID 0x000000A0

void FDI_Extended_CAN_Request(int request_ID_1,int request_ID_2,int request_ID_3,int request_ID_4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Request_tx_message;          

	Request_tx_message.ExtId = FDI_REQUREST_ID;
	Request_tx_message.IDE	 = CAN_ID_EXT;
	Request_tx_message.RTR 	 = CAN_RTR_DATA;
  Request_tx_message.DLC   = 0x08;
	
	Request_ID_list[0]=request_ID_1;
	Request_ID_list[1]=request_ID_2;
	Request_ID_list[2]=request_ID_3;
	Request_ID_list[3]=request_ID_4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Request_tx_message, Request_ID_list, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RX_Header;																			
	uint8_t RX_BUFFER[8];
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RX_Header,RX_BUFFER);					
	
	if(RX_Header.IDE == CAN_ID_STD)
	{
		FDI_Extended_CAN_CallBack(RX_Header.StdId , RX_BUFFER);
	}else if(RX_Header.IDE == CAN_ID_EXT)
	{
		uint16_t decoded_user_ID = (RX_Header.ExtId >> 16) & 0xFFFF;
		if(decoded_user_ID == USER_DEFINE_CAN_ID)
			FDI_Extended_CAN_CallBack(RX_Header.ExtId &= 0x0000FFFF , RX_BUFFER);
	}

}
															
void FDI_Extended_CAN_CallBack(uint32_t ExtId , uint8_t *Buffer)
{
	switch (ExtId)
	{
		case Fdican_Packet_ID_256 :
		{
			FDI_ECAN_MSG_STATUS_01_t		FDI_ECAN_MSG_STATUS_01;
			memcpy(&FDI_ECAN_MSG_STATUS_01 , Buffer , sizeof(FDI_ECAN_MSG_STATUS_01_t));
			break;
		}
		case Fdican_Packet_ID_257 :
		{
			FDI_ECAN_MSG_STATUS_02_t		FDI_ECAN_MSG_STATUS_02;
			memcpy(&FDI_ECAN_MSG_STATUS_02 , Buffer , sizeof(FDI_ECAN_MSG_STATUS_02_t));
			break;
		}
		case Fdican_Packet_ID_258 :
		{
			FDI_ECAN_MSG_STATUS_03_t		FDI_ECAN_MSG_STATUS_03;
			memcpy(&FDI_ECAN_MSG_STATUS_03 , Buffer , sizeof(FDI_ECAN_MSG_STATUS_03_t));
			break;
		}
		case Fdican_Packet_ID_272 :
		{
			FDI_ECAN_MSG_UTC_0_t			FDI_ECAN_MSG_UTC_0;
			memcpy(&FDI_ECAN_MSG_UTC_0 , Buffer , sizeof(FDI_ECAN_MSG_UTC_0_t));
			break;
		}
		case Fdican_Packet_ID_273 :
		{
			FDI_ECAN_MSG_UTC_1_t			FDI_ECAN_MSG_UTC_1;
			memcpy(&FDI_ECAN_MSG_UTC_1 , Buffer , sizeof(FDI_ECAN_MSG_UTC_1_t));
			break;
		}
		case Fdican_Packet_ID_289 :
		{
			FDI_ECAN_MSG_IMU_ACCEL_t		FDI_ECAN_MSG_IMU_ACCEL;	
			memcpy(&FDI_ECAN_MSG_IMU_ACCEL , Buffer , sizeof(FDI_ECAN_MSG_IMU_ACCEL_t));
			break;
		}
		case Fdican_Packet_ID_290 :
		{
			FDI_ECAN_MSG_IMU_GYRO_t			FDI_ECAN_MSG_IMU_GYRO;
			memcpy(&FDI_ECAN_MSG_IMU_GYRO , Buffer , sizeof(FDI_ECAN_MSG_IMU_GYRO_t));
			break;
		}
		case Fdican_Packet_ID_291 :
		{
			FDI_ECAN_MSG_IMU_DELTA_VEL_t		FDI_ECAN_MSG_IMU_DELTA_VEL;
			memcpy(&FDI_ECAN_MSG_IMU_DELTA_VEL , Buffer , sizeof(FDI_ECAN_MSG_IMU_DELTA_VEL_t));
			break;
		}
		case Fdican_Packet_ID_292 :
		{
			FDI_ECAN_MSG_IMU_DELTA_ANGLE_t		FDI_ECAN_MSG_IMU_DELTA_ANGLE;	
			memcpy(&FDI_ECAN_MSG_IMU_DELTA_ANGLE , Buffer , sizeof(FDI_ECAN_MSG_IMU_DELTA_ANGLE_t));
			break;
		}
		case Fdican_Packet_ID_304 :
		{
			FDI_ECAN_MSG_EKF_INFO_t		FDI_ECAN_MSG_EKF_INFO;
			memcpy(&FDI_ECAN_MSG_EKF_INFO , Buffer , sizeof(FDI_ECAN_MSG_EKF_INFO_t));
			break;
		}
		case Fdican_Packet_ID_305 :
		{
			FDI_ECAN_MSG_EKF_QUAT_t			FDI_ECAN_MSG_EKF_QUAT;	
			memcpy(&FDI_ECAN_MSG_EKF_QUAT , Buffer , sizeof(FDI_ECAN_MSG_EKF_QUAT_t));
			break;
		}
		case Fdican_Packet_ID_306 :
		{
			FDI_ECAN_MSG_EKF_EULER_t 		FDI_ECAN_MSG_EKF_EULER;	
			memcpy(&FDI_ECAN_MSG_EKF_EULER , Buffer , sizeof(FDI_ECAN_MSG_EKF_EULER_t));
			break;
		}
		case Fdican_Packet_ID_307 :
		{
			FDI_ECAN_MSG_EKF_ORIENTATION_ACC_t		FDI_ECAN_MSG_EKF_ORIENTATION_ACC;	
			memcpy(&FDI_ECAN_MSG_EKF_ORIENTATION_ACC , Buffer , sizeof(FDI_ECAN_MSG_EKF_ORIENTATION_ACC_t));
			break;
		}
		case Fdican_Packet_ID_308 :
		{
			FDI_ECAN_MSG_EKF_POS_t			FDI_ECAN_MSG_EKF_POS;
			memcpy(&FDI_ECAN_MSG_EKF_POS , Buffer , sizeof(FDI_ECAN_MSG_EKF_POS_t));
			break;
		}
		case Fdican_Packet_ID_309 :
		{
			FDI_ECAN_MSG_EKF_ALTITUDE_t		FDI_ECAN_MSG_EKF_ALTITUDE;
			memcpy(&FDI_ECAN_MSG_EKF_ALTITUDE , Buffer , sizeof(FDI_ECAN_MSG_EKF_ALTITUDE_t));
			break;
		}
		case Fdican_Packet_ID_310 :
		{
			FDI_ECAN_MSG_EKF_POS_ACC_t		FDI_ECAN_MSG_EKF_POS_ACC;
			memcpy(&FDI_ECAN_MSG_EKF_POS_ACC , Buffer , sizeof(FDI_ECAN_MSG_EKF_POS_ACC_t));
			break;
		}
		case Fdican_Packet_ID_311 :
		{
			FDI_ECAN_MSG_EKF_VEL_NED_t		FDI_ECAN_MSG_EKF_VEL_NED;
			memcpy(&FDI_ECAN_MSG_EKF_VEL_NED , Buffer , sizeof(FDI_ECAN_MSG_EKF_VEL_NED_t));
			break;
		}	
		case Fdican_Packet_ID_312 :
		{
			FDI_ECAN_MSG_EKF_VEL_NED_ACC_t	FDI_ECAN_MSG_EKF_VEL_NED_ACC;
			memcpy(&FDI_ECAN_MSG_EKF_VEL_NED_ACC , Buffer , sizeof(FDI_ECAN_MSG_EKF_VEL_NED_ACC_t));
			break;
		}
		case Fdican_Packet_ID_313 :
		{
			FDI_ECAN_MSG_EKF_VEL_BODY_t		FDI_ECAN_MSG_EKF_VEL_BODY;
			memcpy(&FDI_ECAN_MSG_EKF_VEL_BODY , Buffer , sizeof(FDI_ECAN_MSG_EKF_VEL_BODY_t));
			break;
		}
		case Fdican_Packet_ID_320 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_INFO_t		FDI_ECAN_MSG_SHIP_MOTION_INFO;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_INFO , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_INFO_t));
			break;
		}
		case Fdican_Packet_ID_321 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_0_t		FDI_ECAN_MSG_SHIP_MOTION_0;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_0 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_0_t));
			break;
		}
		case Fdican_Packet_ID_325 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_1_t		FDI_ECAN_MSG_SHIP_MOTION_1;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_1 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_1_t));
			break;
		}
		case Fdican_Packet_ID_329 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_2_t		FDI_ECAN_MSG_SHIP_MOTION_2;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_2 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_2_t));
			break;
		}
		case Fdican_Packet_ID_330 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_HP_INFO_t		FDI_ECAN_MSG_SHIP_MOTION_HP_INFO;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_HP_INFO , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_HP_INFO_t));
			break;
		}
		case Fdican_Packet_ID_331 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_HP_0_t		FDI_ECAN_MSG_SHIP_MOTION_HP_0;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_HP_0 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_HP_0_t));
			break;
		}
		case Fdican_Packet_ID_332 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_HP_1_t		FDI_ECAN_MSG_SHIP_MOTION_HP_1;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_HP_1 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_HP_1_t));
			break;
		}
		case Fdican_Packet_ID_333 :
		{
			FDI_ECAN_MSG_SHIP_MOTION_HP_2_t		FDI_ECAN_MSG_SHIP_MOTION_HP_2;
			memcpy(&FDI_ECAN_MSG_SHIP_MOTION_HP_2 , Buffer , sizeof(FDI_ECAN_MSG_SHIP_MOTION_HP_2_t));
			break;
		}
		case Fdican_Packet_ID_336 :
		{
			FDI_ECAN_MSG_MAG_0_t		FDI_ECAN_MSG_MAG_0;
			memcpy(&FDI_ECAN_MSG_MAG_0 , Buffer , sizeof(FDI_ECAN_MSG_MAG_0_t));
			break;
		}
		case Fdican_Packet_ID_337 :
		{
			FDI_ECAN_MSG_MAG_1_t		FDI_ECAN_MSG_MAG_1;
			memcpy(&FDI_ECAN_MSG_MAG_1 , Buffer , sizeof(FDI_ECAN_MSG_MAG_1_t));
			break;
		}
		case Fdican_Packet_ID_338 :
		{
			FDI_ECAN_MSG_MAG_2_t		FDI_ECAN_MSG_MAG_2;
			memcpy(&FDI_ECAN_MSG_MAG_2 , Buffer , sizeof(FDI_ECAN_MSG_MAG_2_t));
			break;
		}
		case Fdican_Packet_ID_352 :
		{
			FDI_ECAN_MSG_ODO_INFO_t		FDI_ECAN_MSG_ODO_INFO;
			memcpy(&FDI_ECAN_MSG_ODO_INFO , Buffer , sizeof(FDI_ECAN_MSG_ODO_INFO_t));
			break;
		}
		case Fdican_Packet_ID_353 :
		{
			FDI_ECAN_MSG_ODO_VEL_t		FDI_ECAN_MSG_ODO_VEL;
			memcpy(&FDI_ECAN_MSG_ODO_VEL , Buffer , sizeof(FDI_ECAN_MSG_ODO_VEL_t));
			break;
		}
		case Fdican_Packet_ID_354 :
		{
			FDI_ECAN_MSG_AIR_DATA_INFO_t		FDI_ECAN_MSG_AIR_DATA_INFO;
			memcpy(&FDI_ECAN_MSG_AIR_DATA_INFO , Buffer , sizeof(FDI_ECAN_MSG_AIR_DATA_INFO_t));
			break;
		}
		case Fdican_Packet_ID_355 :
		{
			FDI_ECAN_MSG_AIR_DATA_ALTITUDE_t		FDI_ECAN_MSG_AIR_DATA_ALTITUDE;	
			memcpy(&FDI_ECAN_MSG_AIR_DATA_ALTITUDE , Buffer , sizeof(FDI_ECAN_MSG_AIR_DATA_ALTITUDE_t));
			break;
		}
		case Fdican_Packet_ID_356 :
		{
			FDI_ECAN_MSG_AIR_DATA_AIRSPEED_t		FDI_ECAN_MSG_AIR_DATA_AIRSPEED;
			memcpy(&FDI_ECAN_MSG_AIR_DATA_AIRSPEED , Buffer , sizeof(FDI_ECAN_MSG_AIR_DATA_AIRSPEED_t));
			break;
		}
		case Fdican_Packet_ID_358 :
		{
			FDI_ECAN_MSG_DEPTH_INFO_t		FDI_ECAN_MSG_DEPTH_INFO;
			memcpy(&FDI_ECAN_MSG_DEPTH_INFO , Buffer , sizeof(FDI_ECAN_MSG_DEPTH_INFO_t));
			break;
		}
		case Fdican_Packet_ID_368 :
		{
			FDI_ECAN_MSG_GPS1_VEL_INFO_t		FDI_ECAN_MSG_GPS1_VEL_INFO;
			memcpy(&FDI_ECAN_MSG_GPS1_VEL_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS1_VEL_INFO_t));
			break;
		}
		case Fdican_Packet_ID_369 :
		{
			FDI_ECAN_MSG_GPS1_VEL_t		FDI_ECAN_MSG_GPS1_VEL;
			memcpy(&FDI_ECAN_MSG_GPS1_VEL , Buffer , sizeof(FDI_ECAN_MSG_GPS1_VEL_t));
			break;
		}
		case Fdican_Packet_ID_370 :
		{
			FDI_ECAN_MSG_GPS1_VEL_ACC_t		FDI_ECAN_MSG_GPS1_VEL_ACC;
			memcpy(&FDI_ECAN_MSG_GPS1_VEL_ACC , Buffer , sizeof(FDI_ECAN_MSG_GPS1_VEL_ACC_t));
			break;
		}
		case Fdican_Packet_ID_371 :
		{
			FDI_ECAN_MSG_GPS1_VEL_COURSE_t		FDI_ECAN_MSG_GPS1_VEL_COURSE;
			memcpy(&FDI_ECAN_MSG_GPS1_VEL_COURSE , Buffer , sizeof(FDI_ECAN_MSG_GPS1_VEL_COURSE_t));
			break;
		}
		case Fdican_Packet_ID_372 :
		{
			FDI_ECAN_MSG_GPS1_POS_INFO_t		FDI_ECAN_MSG_GPS1_POS_INFO;
			memcpy(&FDI_ECAN_MSG_GPS1_POS_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS1_POS_INFO_t));
			break;
		}
		case Fdican_Packet_ID_373 :
		{
			FDI_ECAN_MSG_GPS1_POS_t		FDI_ECAN_MSG_GPS1_POS;
			memcpy(&FDI_ECAN_MSG_GPS1_POS , Buffer , sizeof(FDI_ECAN_MSG_GPS1_POS_t));
			break;
		}
		case Fdican_Packet_ID_374 :
		{
			FDI_ECAN_MSG_GPS1_POS_ALT_t		FDI_ECAN_MSG_GPS1_POS_ALT;
			memcpy(&FDI_ECAN_MSG_GPS1_POS_ALT , Buffer , sizeof(FDI_ECAN_MSG_GPS1_POS_ALT_t));
			break;
		}
		case Fdican_Packet_ID_375 :
		{
			FDI_ECAN_MSG_GPS1_POS_ACC_t		FDI_ECAN_MSG_GPS1_POS_ACC;
			memcpy(&FDI_ECAN_MSG_GPS1_POS_ACC , Buffer , sizeof(FDI_ECAN_MSG_GPS1_POS_ACC_t));
			break;
		}
		case Fdican_Packet_ID_376 :
		{
			FDI_ECAN_MSG_GPS1_HDT_INFO_t		FDI_ECAN_MSG_GPS1_HDT_INFO;
			memcpy(&FDI_ECAN_MSG_GPS1_HDT_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS1_HDT_INFO_t));
			break;
		}
		case Fdican_Packet_ID_377 :
		{
			FDI_ECAN_MSG_GPS1_HDT_t		FDI_ECAN_MSG_GPS1_HDT;
			memcpy(&FDI_ECAN_MSG_GPS1_HDT , Buffer , sizeof(FDI_ECAN_MSG_GPS1_HDT_t));
			break;
		}
		case Fdican_Packet_ID_384 :
		{
			FDI_ECAN_MSG_GPS2_VEL_INFO_t		FDI_ECAN_MSG_GPS2_VEL_INFO;
			memcpy(&FDI_ECAN_MSG_GPS2_VEL_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS2_VEL_INFO_t));
			break;
		}
		case Fdican_Packet_ID_385 :
		{
			FDI_ECAN_MSG_GPS2_VEL_t		FDI_ECAN_MSG_GPS2_VEL;
			memcpy(&FDI_ECAN_MSG_GPS2_VEL , Buffer , sizeof(FDI_ECAN_MSG_GPS2_VEL_t));
			break;
		}
		case Fdican_Packet_ID_386 :
		{
			FDI_ECAN_MSG_GPS2_VEL_COURSE_t		FDI_ECAN_MSG_GPS2_VEL_COURSE;
			memcpy(&FDI_ECAN_MSG_GPS2_VEL_COURSE , Buffer , sizeof(FDI_ECAN_MSG_GPS2_VEL_COURSE_t));
			break;
		}
		case Fdican_Packet_ID_387 :
		{
			FDI_ECAN_MSG_GPS2_VEL_ACC_t		FDI_ECAN_MSG_GPS2_VEL_ACC;
			memcpy(&FDI_ECAN_MSG_GPS2_VEL_ACC , Buffer , sizeof(FDI_ECAN_MSG_GPS2_VEL_ACC_t));
			break;
		}
		case Fdican_Packet_ID_388 :
		{
			FDI_ECAN_MSG_GPS2_POS_INFO_t		FDI_ECAN_MSG_GPS2_POS_INFO;
			memcpy(&FDI_ECAN_MSG_GPS2_POS_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS2_POS_INFO_t));
			break;
		}
		case Fdican_Packet_ID_389 :
		{
			FDI_ECAN_MSG_GPS2_POS_t		FDI_ECAN_MSG_GPS2_POS;
			memcpy(&FDI_ECAN_MSG_GPS2_POS , Buffer , sizeof(FDI_ECAN_MSG_GPS2_POS_t));
			break;
		}
		case Fdican_Packet_ID_390 :
		{
			FDI_ECAN_MSG_GPS2_POS_ALT_t		FDI_ECAN_MSG_GPS2_POS_ALT;
			memcpy(&FDI_ECAN_MSG_GPS2_POS_ALT , Buffer , sizeof(FDI_ECAN_MSG_GPS2_POS_ALT_t));
			break;
		}
		case Fdican_Packet_ID_391 :
		{
			FDI_ECAN_MSG_GPS2_POS_ACC_t		FDI_ECAN_MSG_GPS2_POS_ACC;
			memcpy(&FDI_ECAN_MSG_GPS2_POS_ACC , Buffer , sizeof(FDI_ECAN_MSG_GPS2_POS_ACC_t));
			break;
		}
		case Fdican_Packet_ID_392 :
		{
			FDI_ECAN_MSG_GPS2_HDT_INFO_t		FDI_ECAN_MSG_GPS2_HDT_INFO;
			memcpy(&FDI_ECAN_MSG_GPS2_HDT_INFO , Buffer , sizeof(FDI_ECAN_MSG_GPS2_HDT_INFO_t));
			break;
		}
		case Fdican_Packet_ID_393 :
		{
			FDI_ECAN_MSG_GPS2_HDT_t		FDI_ECAN_MSG_GPS2_HDT;
			memcpy(&FDI_ECAN_MSG_GPS2_HDT , Buffer , sizeof(FDI_ECAN_MSG_GPS2_HDT_t));
			break;
		}
		case Fdican_Packet_ID_518 :
		{
			FDI_ECAN_MSG_EVENT_INFO_D_t		FDI_ECAN_MSG_EVENT_INFO_D;
			memcpy(&FDI_ECAN_MSG_EVENT_INFO_D , Buffer , sizeof(FDI_ECAN_MSG_EVENT_INFO_D_t));
			break;
		}
		case Fdican_Packet_ID_519 :
		{
			memcpy(&FDI_ECAN_MSG_EVENT_TIME_D , Buffer , sizeof(FDI_ECAN_MSG_EVENT_TIME_D_t));
			break;
		}
		case Fdican_Packet_ID_520 :
		{
			memcpy(&FDI_ECAN_MSG_EVENT_INFO_E , Buffer , sizeof(FDI_ECAN_MSG_EVENT_INFO_E_t));
			break;
		}	
	  case Fdican_Packet_ID_521 :
		{
			memcpy(&FDI_ECAN_MSG_EVENT_TIME_E , Buffer , sizeof(FDI_ECAN_MSG_EVENT_TIME_E_t));
			break;
		}	
		case Fdican_Packet_ID_544 :
		{
			memcpy(&FDI_ECAN_MSG_TRACK_SLIP_CURVATURE , Buffer , sizeof(FDI_ECAN_MSG_TRACK_SLIP_CURVATURE_t));
			break;
		}
	}
}
