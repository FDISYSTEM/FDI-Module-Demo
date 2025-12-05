#include "FDILink.h"
#include "fdilink_decode.h"

//__packed typedef struct
//{
//	uint32_t      SubFrame;      // SubFrame=0
//	uint32_t      VariableType;	//变量数据类型
//	uint64_t      VariableValue;
//	char          VariableName[200];
//}FDILINK_COMPONENT_31_0;

//__packed typedef struct
//{
//	uint32_t      SubFrame;      // SubFrame=1
//	char          VariableName[200];
//}FDILINK_COMPONENT_31_1;

//typedef FDILINK_COMPONENT_31_0 FDILink_VarPushData;
//typedef FDILINK_COMPONENT_31_1 FDILink_VarPollData;

FDILink_VersionData_Packet_t Version_Receive;		//0x39
FDILink_IMUData_Packet_t IMU_Receive;		//0x40
FDILink_AHRSData_Packet_t AHRS_Receive;		//0x41
FDILink_INSGPSData_Packet_t INSGPS_Receive;		//0x42
System_State_Packet_t System_State_Receive;		//0x50
Unix_Time_Packet_t Unix_Time_Receive;		//0x51
Formatted_Time_Packet_t Formatted_Time_Receive;		//0x52
Status_Packet_t Status_Receive;		//0x53
Position_Standard_Deviation_Packet_t Position_Standard_Deviation_Receive;		//0x54
Velocity_Standard_Deviation_Packet_t Velocity_Standard_Deviation_Receive;		//0x55
Euler_Orientation_Standard_Deviation_Packet_t Euler_Orientation_Standard_Deviation_Receive;			//0x56
Quaternion_Orientation_Standard_Deviation_Packet_t Quaternion_Orientation_Standard_Deviation_Receive;		//0x57
Raw_Sensors_Packet_t Raw_Sensors_Receive;		//0x58
Raw_GNSS_Packet_t Raw_GNSS_Receive;		//0x59
Satellites_Packet_t Satellites_Receive;		//0x5A
Detailed_Satellites_Packet_t Detailed_Satellites_Receive;		//0x5B
Geodetic_Position_Packet_t Geodetic_Position_Receive;		//0x5C
ECEF_Position_Packet_t ECEF_Position_Receive;		//0x5D
UTM_Position_Packet_t UTM_Position_Receive;		//0x5E
NED_Velocity_Packet_t NED_Velocity_Receive;		//0x5F
Body_Velocity_Packet_t Body_Velocity_Receive;		//0x60
Acceleration_Packet_t Acceleration_Receive;		//0x61
Body_Acceleration_Packet_t Body_Acceleration_Receive;		//0x62
Euler_Orientation_Packet_t Euler_Orientation_Receive;		//0x63
Quaternion_Orientation_Packet_t Quaternion_Orientation_Receive;		//0x64
DCM_Orientation_Packet_t DCM_Orientation_Receive;		//0x65
Angular_Velocity_Packet_t Angular_Velocity_Receive;		//0x66
Angular_Acceleration_Packet_t Angular_Acceleration_Receive;		//0x67
//0x6D
Running_Time_Packet_t Running_Time_Receive;
//0x6E
Local_Magnetic_Field_Packet_t Local_Magnetic_Field_Receive;
//0x6F
Odometer_State_Packet_t Odometer_State_Receive;
//0x72
Geoid_Height_Packet_t Geoid_Height_Receive;
//0x73
RTCM_Corrections_Packet_t RTCM_Corrections_Receive;
//0x75
Wind_Packet_t Wind_Receive;
//0x76
Heave_Packet_t Heave_Receive;
//0x77
Raw_Satellite_Data_Packet_t Raw_Satellite_Data_Receive;
//0x78
GNSS_DUAL_ANT_Data_Packet_t GNSS_DUAL_ANT_Data_Receive;
//0x7A
Gimbal_State_Packet_t Gimbal_State_Receive;
//0x7B
Automotive_Packet_t Automotive_Receive;
//0x7C
Packet_Timer_Period_Packet_t Packet_Timer_Period_Receive;
//0x7D
Packets_Period_Packet_t Packets_Period_Receive;
//0x80
Installation_Alignment_Packet_t Installation_Alignment_Receive;
//0x81
Filter_Options_Packet_t Filter_Options_Receive;
//0x82	---
GPIO_Configuration_Packet_t GPIO_Configuration_Receive;
//0x83
Magnetic_Calibration_Values_Packet_t Magnetic_Calibration_Values_Receive;
//0x84
Magnetic_Calibration_Configuration_Packet_t Magnetic_Calibration_Configuration_Receive;
//0x85
Magnetic_Calibration_Status_Packet_t Magnetic_Calibration_Status_Receive;
//0x86
Odometer_Configuration_Packet_t Odometer_Configuration_Receive;
//0x87
Set_Zero_Orientation_Alignment_Packet_t Set_Zero_Orientation_Alignment_Receive;
//0x88	---
Reference_Point_Offsets_Packet_t Reference_Point_Offsets_Receive;
//0x8A
User_Data_Packet_t User_Data_Receive;
//0xA0	---
Baud_Rates_Packet_t	Baud_Rates_Receive;
//0xA1	---
Sensor_Ranges_Packet_t Sensor_Ranges_Receive;
//0xA2	---
GPIO_Output_Configuration_Packet_t GPIO_Output_Configuration_Receive;
//0xA3	---
GPIO_Input_Configuration_Packet_t GPIO_Input_Configuration_Receive;
//0xA4	---
Dual_Antenna_Configuration_Packet_t Dual_Antenna_Configuration_Receive;
/**	-------------外部输入-------------- */
//0x68
External_Position_And_Velocity_Packet_t External_Position_And_Velocity_Receive;
//0x69
External_Position_Packet_t External_Position_Receive;
//0x6A
External_Velocity_Packet_t External_Velocity_Receive;
//0x6B
External_Body_Velocity_Packet_t External_Body_Velocity_Receive;
//0x6C
External_Heading_Packet_t External_Heading_Receive;
//0x70
External_Time_Packet_t External_Time_Receive;
//0x71
External_Depth_Packet_t External_Depth_Receive;
//0x74
External_Pitot_Pressure_Packet_t External_Pitot_Pressure_Receive;
//0x79
External_Air_Data_Packet_t External_Air_Data_Receive;
//0x91	---
External_LIDAR_Packet_t	External_LIDAR_Receive;
//0x92	---
External_SLAM1_Packet_t	External_SLAM1_Receive;
//0x93	---
External_SLAM2_Packet_t	External_SLAM2_Receive;
//0x90	---
External_Odom_Data_Packet_t	External_Odom_Data_Receive;
//0xA0	---
Packet_Requst_Packet_t Packet_Requst_Receive;

void* const FDILink_Buffer_List[256] = 
{
	[FDILINK_VERSIONDATA_PACKET_ID]						  = &Version_Receive,
	[FDILINK_IMUDATA_PACKET_ID]                           = &IMU_Receive,
	[FDILINK_AHRSDATA_PACKET_ID]                          = &AHRS_Receive,
	[FDILINK_INSGPSDATA_PACKET_ID]                        = &INSGPS_Receive,
	[System_State_Packet_ID]                              = &System_State_Receive,
	[Unix_Time_Packet_ID]                                 = &Unix_Time_Receive,
	[Formatted_Time_Packet_ID]                            = &Formatted_Time_Receive,
	[Status_Packet_ID]                                    = &Status_Receive,
	[Position_Standard_Deviation_Packet_ID]               = &Position_Standard_Deviation_Receive,
	[Velocity_Standard_Deviation_Packet_ID]               = &Velocity_Standard_Deviation_Receive,
	[Euler_Orientation_Standard_Deviation_Packet_ID]      = &Euler_Orientation_Standard_Deviation_Receive,
	[Quaternion_Orientation_Standard_Deviation_Packet_ID] = &Quaternion_Orientation_Standard_Deviation_Receive,
	[Raw_Sensors_Packet_ID]                               = &Raw_Sensors_Receive,
	[Raw_GNSS_Packet_ID]                                  = &Raw_GNSS_Receive,
	[Satellites_Packet_ID]                                = &Satellites_Receive,
	[Detailed_Satellites_Packet_ID]                       = &Detailed_Satellites_Receive,
	[Geodetic_Position_Packet_ID]                         = &Geodetic_Position_Receive,
	[ECEF_Position_Packet_ID]                             = &ECEF_Position_Receive,
	[UTM_Position_Packet_ID]                              = &UTM_Position_Receive,
	[NED_Velocity_Packet_ID]                              = &NED_Velocity_Receive,
	[Body_Velocity_Packet_ID]                             = &Body_Velocity_Receive,
	[Acceleration_Packet_ID]                              = &Acceleration_Receive,
	[Body_Acceleration_Packet_ID]                         = &Body_Acceleration_Receive,
	[Euler_Orientation_Packet_ID]                         = &Euler_Orientation_Receive,
	[Quaternion_Orientation_Packet_ID]                    = &Quaternion_Orientation_Receive,
	[DCM_Orientation_Packet_ID]                           = &DCM_Orientation_Receive,
	[Angular_Velocity_Packet_ID]                          = &Angular_Velocity_Receive,
	[Angular_Acceleration_Packet_ID]                      = &Angular_Acceleration_Receive,
	[Running_Time_Packet_ID]                              = &Running_Time_Receive,
	[Local_Magnetic_Field_Packet_ID]                      = &Local_Magnetic_Field_Receive,
	[Odometer_State_Packet_ID]                            = &Odometer_State_Receive,
	[Geoid_Height_Packet_ID]                              = &Geoid_Height_Receive,
	[RTCM_Corrections_Packet_ID]                          = &RTCM_Corrections_Receive,
	[Wind_Packet_ID]                                      = &Wind_Receive,
	[Heave_Packet_ID]                                     = &Heave_Receive,
	[Raw_Satellite_Data_Packet_ID]                        = &Raw_Satellite_Data_Receive,
	[GNSS_DUAL_ANT_Data_Packet_ID]						  = &GNSS_DUAL_ANT_Data_Receive,
	[Gimbal_State_Packet_ID]                              = &Gimbal_State_Receive,
	[Automotive_Packet_ID]                                = &Automotive_Receive,
	[Packet_Timer_Period_Packet_ID]                       = &Packet_Timer_Period_Receive,
	[Packets_Period_Packet_ID]                            = &Packets_Period_Receive,
	[Installation_Alignment_Packet_ID]                    = &Installation_Alignment_Receive,
	[Filter_Options_Packet_ID]                            = &Filter_Options_Receive,
	[GPIO_Configuration_Packet_ID]               		  = &GPIO_Configuration_Receive,
	[Magnetic_Calibration_Values_Packet_ID]               = &Magnetic_Calibration_Values_Receive,
	[Magnetic_Calibration_Configuration_Packet_ID]        = &Magnetic_Calibration_Configuration_Receive,
	[Magnetic_Calibration_Status_Packet_ID]               = &Magnetic_Calibration_Status_Receive,
	[Odometer_Configuration_Packet_ID]                    = &Odometer_Configuration_Receive,
	[Set_Zero_Orientation_Alignment_Packet_ID]            = &Set_Zero_Orientation_Alignment_Receive,
	[Reference_Point_Offsets_Packet_ID]           		  = &Reference_Point_Offsets_Receive,
	[User_Data_Packet_ID]                                 = &User_Data_Receive,
	[Baud_Rates_Packet_ID]                                = &Baud_Rates_Receive,
	[Sensor_Ranges_Packet_ID]                             = &Sensor_Ranges_Receive,
	[GPIO_Output_Configuration_Packet_ID]                 = &GPIO_Output_Configuration_Receive,
	[GPIO_Input_Configuration_Packet_ID]                  = &GPIO_Input_Configuration_Receive,
	[Dual_Antenna_Configuration_Packet_ID]                = &Dual_Antenna_Configuration_Receive,

	[External_Position_And_Velocity_Packet_ID]            = &External_Position_And_Velocity_Receive,
	[External_Position_Packet_ID]                         = &External_Position_Receive,
	[External_Velocity_Packet_ID]                         = &External_Velocity_Receive,
	[External_Body_Velocity_Packet_ID]                    = &External_Body_Velocity_Receive,
	[External_Heading_Packet_ID]                          = &External_Heading_Receive,
	[External_Time_Packet_ID]                             = &External_Time_Receive,
	[External_Depth_Packet_ID]                            = &External_Depth_Receive,
	[External_Pitot_Pressure_Packet_ID]                   = &External_Pitot_Pressure_Receive,
	[External_Air_Data_Packet_ID]                         = &External_Air_Data_Receive,
	[External_LIDAR_Packet_ID]                        	  = &External_LIDAR_Receive,
	[External_SLAM1_Packet_ID]                            = &External_SLAM1_Receive,
	[External_SLAM2_Packet_ID]                            = &External_SLAM2_Receive,
	[External_Odom_Data_Packet_ID]                        = &External_Odom_Data_Receive,
	[Packet_Requst_Packet_ID]                        	  = &Packet_Requst_Receive,

};

FDILink_t FDILink_Handler;
uint8_t FDILink_Recive_Buffer[256];
int FDILink_Index = 0;
#include "string.h"

int InuptChar(uint8_t c)
{
	int update = 0;
	FDILink_Index = (FDILink_Index + 1) % 256;
	if(FDILink_Receive(&FDILink_Handler, c) > 0)
	{
		void* p = FDILink_Buffer_List[FDILink_Handler.RxType];
		if(p)
		{
			memcpy(p, FDILink_Handler.Buffer, FDILink_Handler.BufferIndex);
			update = FDILink_Handler.RxType;
		}
	}
	return update;
}

