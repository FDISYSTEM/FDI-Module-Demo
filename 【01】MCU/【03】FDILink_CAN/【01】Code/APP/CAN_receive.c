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
	
#include "fdilink_decode.h"
#include "string.h"
#include "stm32f4xx_hal.h"

uint8_t rxcan_nine[32][8];
extern uint8_t CRC8_Table(uint8_t* p, uint8_t counter);

uint8_t RX_BUFFER[8];	

/*!
 *  CAN接收回调函数
 *	CAN interrupts the receive function.
 *	\param[out]	None
 *	\param[in]	hcan
 */
	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RX_Header;
																																		
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RX_Header,RX_BUFFER);
	
	static uint8_t type = 0;				//FDI数据帧类型
	static uint8_t length = 0;			//FDI数据帧载荷长度，CAN帧总载荷长度为FDI数据帧载荷长度加FDI数据帧头帧长度(8)
	static uint8_t error = 0;				//错误标志位，出错不进行处理，等待下一个FDI数据帧
	static uint8_t last_num = 0xFD;	//结束位，表示一帧的结尾
	static uint8_t total = 0;				//FDI数据帧总发送CAN帧数目，数据帧载荷长度/8 +  数据帧头帧长度/8
	static uint8_t left = 0;				//最后一个CAN帧的载荷长度
	static uint8_t counter = 0;			//计数器，用于统计当前是第几个帧
	
	if(RX_BUFFER[0]==0xFC && last_num==0xFD)
	{
		error = 0;
		type = RX_BUFFER[1];
		length = RX_BUFFER[2];
		uint8_t CRC_CAN = CRC8_Table(RX_BUFFER,4);
		if(CRC_CAN != RX_BUFFER[4])
		{
			error++;
			return;
		}
		left = length % 8;
		total = length / 8 + 1;
		if(left != 0)
			total += 1;
		counter = 0;
		last_num = RX_BUFFER[7];
	}
	if(error != 0)
		return;
	if(counter != total - 1)
	{
		for(int i = 0; i < 8; i++)
			rxcan_nine[counter][i] = RX_BUFFER[i];
		counter++;
	}else{
		if(left == 0)
		{
			for(int i = 0; i < 8; i++)
				rxcan_nine[counter][i] = RX_BUFFER[i];
			last_num = RX_BUFFER[7];
		}else{
			for(int i = 0; i < left; i++)
				rxcan_nine[counter][i] = RX_BUFFER[i];
			last_num = RX_BUFFER[left - 1];
		}
		if(last_num != 0xFD)
		{
			last_num = 0xFD;
		}
		error++;					//本FDI数据帧最后一个发送的CAN帧，如果下个CAN帧不是新的FDI数据帧可以认为是错误
		counter++;
	}
	if(counter == total)
	{
		switch(type)
		{
			case FDILink_VersionData_Packet_ID:			//VersionData 0x39 OK
			{
				FDILink_VersionData_Packet_t VersionData;
				memcpy(&VersionData, rxcan_nine + 7, sizeof(FDILink_VersionData_Packet_t));
				break;
			}
			case FDILink_IMUData_Packet_ID:			//MSG_IMU 0x40 OK
			{
				FDILink_IMUData_Packet_t	IMUData;
				memcpy(&IMUData, rxcan_nine + 7, sizeof(FDILink_IMUData_Packet_t));
				break;
			}
			case FDILink_AHRSData_Packet_ID:			//MSG_AHRS 0x41 OK
			{
				FDILink_AHRSData_Packet_t AHRSData;
				memcpy(&AHRSData, rxcan_nine + 7, sizeof(FDILink_AHRSData_Packet_t));
				break;
			}
			case FDILink_INSGPSData_Packet_ID:			//MSG_INS/GPS 0x42 OK
			{
				FDILink_INSGPSData_Packet_t INSGPSData;
				memcpy(&INSGPSData, rxcan_nine + 7, sizeof(FDILink_INSGPSData_Packet_t));
				break;
			}
			case System_State_Packet_ID:			//MSG_SYS_STATE 0x50 OK
			{
				System_State_Packet_t Statedata;
				memcpy(&Statedata, rxcan_nine + 7, sizeof(System_State_Packet_t));
				break;
			}
			case Unix_Time_Packet_ID:			//UNIX_TIME 0X51 OK
			{
				Unix_Time_Packet_t TimeData;
				memcpy(&TimeData, rxcan_nine + 7, sizeof(Unix_Time_Packet_t));
				break;
			}
			case Formatted_Time_Packet_ID:			//FORMATTED_TIME 0X52 OK
			{
				Formatted_Time_Packet_t FTimeData;
				memcpy(&FTimeData, rxcan_nine + 7, sizeof(Formatted_Time_Packet_t));
				break;
			}
			case Status_Packet_ID:			//Status 0X53 OK
			{
				Status_Packet_t StatusData;
				memcpy(&StatusData, rxcan_nine + 7, sizeof(Status_Packet_t));
				break;
			}	
			case Position_Standard_Deviation_Packet_ID:			//POSITION_STANDARD OX54 OK
			{
				Position_Standard_Deviation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Position_Standard_Deviation_Packet_t));
				break;
			}
			case Velocity_Standard_Deviation_Packet_ID:			//VELOCITY_STANDARD OX55 OK
			{
				Velocity_Standard_Deviation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Velocity_Standard_Deviation_Packet_t));
				break;
			}
			case Euler_Orientation_Standard_Deviation_Packet_ID:			//EULER_ORIENTATION_STANDARD OX56 OK
			{
				Euler_Orientation_Standard_Deviation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Euler_Orientation_Standard_Deviation_Packet_t));
				break;
			}
			case Quaternion_Orientation_Standard_Deviation_Packet_ID:			//QUATERNION_ORIENTATION_STANDARD OX57
			{
				Quaternion_Orientation_Standard_Deviation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Quaternion_Orientation_Standard_Deviation_Packet_t));
				break;
			}
			case Raw_Sensors_Packet_ID:			//RAW_SENSORS OX58
			{
				Raw_Sensors_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Raw_Sensors_Packet_t));
				break;
			}		
			case Raw_GNSS_Packet_ID:			//MSG_RAW_GNSS 0x59
			{
				Raw_GNSS_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Raw_GNSS_Packet_t));
				break;
			}
			case Satellites_Packet_ID:			//SATELLITES 0x5A
			{
				Satellites_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Satellites_Packet_t));
				break;
			}
			case Detailed_Satellites_Packet_ID:			//DETAILED_SATELLITES 0x5B
			{
				Detailed_Satellites_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Detailed_Satellites_Packet_t));
				break;
			}
			case Geodetic_Position_Packet_ID:    //GEODETIC_POSITION 0x5C
			{
				Geodetic_Position_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Geodetic_Position_Packet_t));
				break;
			}
			case ECEF_Position_Packet_ID:        //GEODETIC_POSITION 0x5D
			{
				ECEF_Position_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(ECEF_Position_Packet_t));
				break;
			}
			case UTM_Position_Packet_ID:         //UIM_POSITION 0x5E
			{
				UTM_Position_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(UTM_Position_Packet_t));
				break;
			}
			case NED_Velocity_Packet_ID:         //NED_VELOCITY 0x5F
			{
				NED_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(NED_Velocity_Packet_t));
				break;
			}
			case Body_Velocity_Packet_ID:        //BODY_VELOCITY 0x60
			{
				Body_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Body_Velocity_Packet_t));
				break;
			}
			case Acceleration_Packet_ID:         //ACCELERATION 0x61
			{
				Acceleration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Acceleration_Packet_t));
				break;
			}		
			case Body_Acceleration_Packet_ID:    //BODT_ACCELERATION 0x62
			{
				Body_Acceleration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Body_Acceleration_Packet_t));
				break;
			}		
			case Euler_Orientation_Packet_ID:    //EULER_ORIENTATION 0x63
			{
				Euler_Orientation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Euler_Orientation_Packet_t));
				break;
			}
			case Quaternion_Orientation_Packet_ID://QUATERNOION_ORIENTATION 0x64
			{
				Quaternion_Orientation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Quaternion_Orientation_Packet_t));
				break;
			}
			case DCM_Orientation_Packet_ID:      //DCM_ORIENTATION 0x65
			{
				DCM_Orientation_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(DCM_Orientation_Packet_t));
				break;
			}
			case Angular_Velocity_Packet_ID:     //ANGUAR_VELOCITY 0x66
			{
				Angular_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Angular_Velocity_Packet_t));
				break;
			}
			case Angular_Acceleration_Packet_ID: //ANGUAR_ACCELERATION 0x67
			{
				Angular_Acceleration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Angular_Acceleration_Packet_t));
				break;
			}
			case Running_Time_Packet_ID:         //RUNNING_TIME_PACKET 0x6D
			{
				Running_Time_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Running_Time_Packet_t));
				break;
			}
			case Local_Magnetic_Field_Packet_ID: //LOCAL 0x6E
			{
				Local_Magnetic_Field_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Local_Magnetic_Field_Packet_t));
				break;
			}
			case Odometer_State_Packet_ID:       //ODOMETER 0x6F
			{
				Odometer_State_Packet_t data;
				memcpy(&data, rxcan_nine + 7 ,sizeof(Odometer_State_Packet_t));
				break;
			}
			case Geoid_Height_Packet_ID:         //GEOID_HEIGHT 0x72
			{
				Geoid_Height_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Geoid_Height_Packet_t));
				break;
			}
			case RTCM_Corrections_Packet_ID:     //RTCM_CORRECTIONS 0x73
			{
				RTCM_Corrections_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(RTCM_Corrections_Packet_t));
				break;
			}
			case Wind_Packet_ID:                 //WIND 0x75
			{
				Wind_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Wind_Packet_t));
				break;
			}
			case Heave_Packet_ID:                //HEAVE 0x76
			{
				Heave_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Heave_Packet_t));
				break;
			}
			case Raw_Satellite_Data_Packet_ID:   //RAW_SATELLITE 0x77
			{
				Raw_Satellite_Data_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Raw_Satellite_Data_Packet_t));
				break;
			}
			case GNSS_DUAL_ANT_Data_Packet_ID:   //GNSS_DUAL 0x78
			{
				GNSS_DUAL_ANT_Data_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(GNSS_DUAL_ANT_Data_Packet_t));
				break;
			}
			case Gimbal_State_Packet_ID:         //GIMBAL_STATE 0x7A
			{
				Gimbal_State_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Gimbal_State_Packet_t));
				break;
			}		
			case Automotive_Packet_ID:           //AUTOMOTIVE 0x7B
			{
				Automotive_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Automotive_Packet_t));
				break;
			}
			case Packet_Timer_Period_Packet_ID:         //Packet_Timer_Period 0x7C
			{
				Packet_Timer_Period_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Packet_Timer_Period_Packet_t));
				break;
			}		
			case Packets_Period_Packet_ID:           //Packets_Period 0x7D
			{
				Packets_Period_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Packets_Period_Packet_t));
				break;
			}
			case Installation_Alignment_Packet_ID://INSTALLATION_ALIGNMENT 0x80
			{
				Installation_Alignment_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Installation_Alignment_Packet_t));
				break;
			}
			case Filter_Options_Packet_ID:       //FILTER_OPTIONS 0x81
			{
				Filter_Options_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Filter_Options_Packet_t));
				break;
			}
			case GPIO_Configuration_Packet_ID:   //GPIO_CONFIG Ox82
			{
				GPIO_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(GPIO_Configuration_Packet_t));
				break;
			}		
			case Magnetic_Calibration_Values_Packet_ID://MAGNETIC_CAIL 0x83
			{
				Magnetic_Calibration_Values_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Magnetic_Calibration_Values_Packet_t));
				break;
			}		
			case Magnetic_Calibration_Configuration_Packet_ID://MAGNETIC_CAIL_CONFIG 0x84
			{
				Magnetic_Calibration_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Magnetic_Calibration_Configuration_Packet_t));
				break;
			}			
			case Magnetic_Calibration_Status_Packet_ID://MAGNETIC_CAIL_STATUS 0x85
			{
				Magnetic_Calibration_Status_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Magnetic_Calibration_Status_Packet_t));
				break;
			}			
			case Odometer_Configuration_Packet_ID://ODOMETER_CONFIG 0x86
			{
				Odometer_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Odometer_Configuration_Packet_t));
				break;
			}	
			case Set_Zero_Orientation_Alignment_Packet_ID://SET_ZERO 0x87
			{
				Set_Zero_Orientation_Alignment_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Set_Zero_Orientation_Alignment_Packet_t));
				break;
			}		
			case Reference_Point_Offsets_Packet_ID://REFERENCE_POINT 0x88
			{
				Reference_Point_Offsets_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Reference_Point_Offsets_Packet_t));
				break;
			}			
			case User_Data_Packet_ID:             //USER_DATA 0x8A
			{
				User_Data_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(User_Data_Packet_t));
				break;
			}	
			case Baud_Rates_Packet_ID:            //BAUD_SATES 0xA0
			{
				Baud_Rates_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Baud_Rates_Packet_t));
				break;
			}	
			case Sensor_Ranges_Packet_ID:         //SENSOR_RANGES 0xA1
			{
				Sensor_Ranges_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Sensor_Ranges_Packet_t));
				break;
			}	
			case GPIO_Output_Configuration_Packet_ID://GPIO_OUTPUT_CONFIG 0xA2
			{
				GPIO_Output_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(GPIO_Output_Configuration_Packet_t));
				break;
			}		
			case GPIO_Input_Configuration_Packet_ID: //GPIO_INPUT_CONFIG 0xA3
			{
				GPIO_Input_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(GPIO_Input_Configuration_Packet_t));
				break;
			}			
			case Dual_Antenna_Configuration_Packet_ID://DUAL_ANTENNA_CONFIG 0xA4
			{
				Dual_Antenna_Configuration_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(Dual_Antenna_Configuration_Packet_t));
				break;
			}
			case External_Position_And_Velocity_Packet_ID://EXTERNAL_POSITION_AND_VELOCITY 0x68
			{
				External_Position_And_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Position_And_Velocity_Packet_t));
				break;
			}
			case External_Position_Packet_ID:    //EXTERNAL_POSITION 0x69
			{
				External_Position_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Position_Packet_t));
				break;
			}
			case External_Velocity_Packet_ID:    //EXTERNAL_VELOCITY 0x6A
			{
				External_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Velocity_Packet_t));
				break;
			}
			case External_Body_Velocity_Packet_ID://EXTERNAL_BODT_VELOCITY 0x6B
			{
				External_Body_Velocity_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Body_Velocity_Packet_t));
				break;
			}
			case External_Heading_Packet_ID:     //EXTERNAL_HEADING 0x6C
			{
				External_Heading_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Heading_Packet_t));
				break;
			}
			case External_Time_Packet_ID:        //EXTERNAL_TIME 0x70
			{
				External_Time_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Time_Packet_t));
				break;
			}
			case External_Depth_Packet_ID:       //EXTERNAL_DEPTH 0x71
			{
				External_Depth_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Depth_Packet_t));
				break;
			}
			case External_Pitot_Pressure_Packet_ID://External_Pitot_Pressure_Packet_ID 074
			{
				External_Pitot_Pressure_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Pitot_Pressure_Packet_t));
				break;
			}
			case External_Air_Data_Packet_ID:    //EXTERNAL_AIE_DATA 0x79
			{
				External_Air_Data_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Air_Data_Packet_t));
				break;
			}
			case External_Odom_Data_Packet_ID:   //EXTERNAL_ODOM_DATA 0x90
			{
				External_Odom_Data_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_Odom_Data_Packet_t));
				break;
			}
			case External_LIDAR_Packet_ID:       //EXTERNAL_LIDAR_DATA 0x91
			{
				External_LIDAR_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_LIDAR_Packet_t));
				break;
			}		
			case External_SLAM1_Packet_ID:       //EXTERNAL_SALAM1 0x92
			{
				External_SLAM1_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_SLAM1_Packet_t));
				break;
			}
			case External_SLAM2_Packet_ID:       //EXTERNAL_SALAM2 0x93
			{
				External_SLAM2_Packet_t data;
				memcpy(&data, rxcan_nine + 7, sizeof(External_SLAM2_Packet_t));
				break;
			}
		}
	}
}
