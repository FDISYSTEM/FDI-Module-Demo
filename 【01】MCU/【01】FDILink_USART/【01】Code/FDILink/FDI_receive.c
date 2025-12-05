#include "FDI_receive.h"
#include "string.h"
#include "fdilink_decode.h"

FDILink_IMUData_Packet_t IMUData;

void fdiDecodeBuffer(int PACKET_ID,void* buf)
{
	switch(PACKET_ID)
	{
		case FDILink_VersionData_Packet_ID:			//VersionData 0x39 OK
		{
			FDILink_VersionData_Packet_t VersionData;
			memcpy(&VersionData,buf,sizeof(FDILink_VersionData_Packet_t));
			break;
		}
		case FDILink_IMUData_Packet_ID:			//MSG_IMU 0x40 OK
		{
//			FDILink_IMUData_Packet_t IMUData;
			memcpy(&IMUData,buf,sizeof(FDILink_IMUData_Packet_t));
			break;
		}
		case FDILink_AHRSData_Packet_ID:			//MSG_AHRS 0x41 OK
		{
			FDILink_AHRSData_Packet_t AHRSData;
			memcpy(&AHRSData,buf,sizeof(FDILink_AHRSData_Packet_t));
			break;
		}
		case FDILink_INSGPSData_Packet_ID:			//MSG_INS/GPS 0x42 OK
		{
			FDILink_INSGPSData_Packet_t INSGPSData;
			memcpy(&INSGPSData,buf,sizeof(FDILink_INSGPSData_Packet_t));
			break;
		}
		case System_State_Packet_ID:			//MSG_SYS_STATE 0x50 OK
		{
			System_State_Packet_t Statedata;
			memcpy(&Statedata,buf,sizeof(System_State_Packet_t));
			break;
		}
		case Unix_Time_Packet_ID:			//UNIX_TIME 0X51 OK
		{
			Unix_Time_Packet_t TimeData;
			memcpy(&TimeData,buf,sizeof(Unix_Time_Packet_t));
			break;
		}
		case Formatted_Time_Packet_ID:			//FORMATTED_TIME 0X52 OK
		{
			Formatted_Time_Packet_t FTimeData;
			memcpy(&FTimeData,buf,sizeof(Formatted_Time_Packet_t));
			break;
		}
		case Status_Packet_ID:			//Status 0X53 OK
		{
			Status_Packet_t StatusData;
			memcpy(&StatusData,buf,sizeof(Status_Packet_t));
			break;
		}	
		case Position_Standard_Deviation_Packet_ID:			//POSITION_STANDARD OX54 OK
		{
			Position_Standard_Deviation_Packet_t data;
			memcpy(&data,buf,sizeof(Position_Standard_Deviation_Packet_t));
			break;
		}
		case Velocity_Standard_Deviation_Packet_ID:			//VELOCITY_STANDARD OX55 OK
		{
			Velocity_Standard_Deviation_Packet_t data;
			memcpy(&data,buf,sizeof(Velocity_Standard_Deviation_Packet_t));
			break;
		}
		case Euler_Orientation_Standard_Deviation_Packet_ID:			//EULER_ORIENTATION_STANDARD OX56 OK
		{
			Euler_Orientation_Standard_Deviation_Packet_t data;
			memcpy(&data,buf,sizeof(Euler_Orientation_Standard_Deviation_Packet_t));
			break;
		}
		case Quaternion_Orientation_Standard_Deviation_Packet_ID:			//QUATERNION_ORIENTATION_STANDARD OX57
		{
			Quaternion_Orientation_Standard_Deviation_Packet_t data;
			memcpy(&data,buf,sizeof(Quaternion_Orientation_Standard_Deviation_Packet_t));
			break;
		}
		case Raw_Sensors_Packet_ID:			//RAW_SENSORS OX58
		{
			Raw_Sensors_Packet_t data;
			memcpy(&data,buf,sizeof(Raw_Sensors_Packet_t));
			break;
		}		
		case Raw_GNSS_Packet_ID:			//MSG_RAW_GNSS 0x59
		{
			Raw_GNSS_Packet_t data;
			memcpy(&data,buf,sizeof(Raw_GNSS_Packet_t));
			break;
		}
		case Satellites_Packet_ID:			//SATELLITES 0x5A
		{
			Satellites_Packet_t data;
			memcpy(&data,buf,sizeof(Satellites_Packet_t));
			break;
		}
		case Detailed_Satellites_Packet_ID:			//DETAILED_SATELLITES 0x5B
		{
			Detailed_Satellites_Packet_t data;
			memcpy(&data,buf,sizeof(Detailed_Satellites_Packet_t));
			break;
		}
		case Geodetic_Position_Packet_ID:    //GEODETIC_POSITION 0x5C
		{
			Geodetic_Position_Packet_t data;
			memcpy(&data,buf,sizeof(Geodetic_Position_Packet_t));
						break;
		}
		case ECEF_Position_Packet_ID:        //GEODETIC_POSITION 0x5D
		{
			ECEF_Position_Packet_t data;
			memcpy(&data,buf,sizeof(ECEF_Position_Packet_t));
						break;
		}
		case UTM_Position_Packet_ID:         //UIM_POSITION 0x5E
		{
			UTM_Position_Packet_t data;
			memcpy(&data,buf,sizeof(UTM_Position_Packet_t));
			break;
		}
		case NED_Velocity_Packet_ID:         //NED_VELOCITY 0x5F
		{
			NED_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(NED_Velocity_Packet_t));
			break;
		}
		case Body_Velocity_Packet_ID:        //BODY_VELOCITY 0x60
		{
			Body_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(Body_Velocity_Packet_t));
			break;
		}
		case Acceleration_Packet_ID:         //ACCELERATION 0x61
		{
			Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Acceleration_Packet_t));
			break;
		}		
		case Body_Acceleration_Packet_ID:    //BODT_ACCELERATION 0x62
		{
			Body_Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Body_Acceleration_Packet_t));
			break;
		}		
		case Euler_Orientation_Packet_ID:    //EULER_ORIENTATION 0x63
		{
			Euler_Orientation_Packet_t data;
			memcpy(&data,buf,sizeof(Euler_Orientation_Packet_t));
			break;
		}
		case Quaternion_Orientation_Packet_ID://QUATERNOION_ORIENTATION 0x64
		{
			Quaternion_Orientation_Packet_t data;
			memcpy(&data,buf,sizeof(Quaternion_Orientation_Packet_t));
			break;
		}
		case DCM_Orientation_Packet_ID:      //DCM_ORIENTATION 0x65
		{
			DCM_Orientation_Packet_t data;
			memcpy(&data,buf,sizeof(DCM_Orientation_Packet_t));
			break;
		}
		case Angular_Velocity_Packet_ID:     //ANGUAR_VELOCITY 0x66
		{
			Angular_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(Angular_Velocity_Packet_t));
			break;
		}
		case Angular_Acceleration_Packet_ID: //ANGUAR_ACCELERATION 0x67
		{
			Angular_Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Angular_Acceleration_Packet_t));
			break;
		}
		
		case Running_Time_Packet_ID:         //RUNNING_TIME_PACKET 0x6D
		{
			Running_Time_Packet_t data;
			memcpy(&data,buf,sizeof(Running_Time_Packet_t));
			break;
		}
		case Local_Magnetic_Field_Packet_ID: //LOCAL 0x6E
		{
			Local_Magnetic_Field_Packet_t data;
			memcpy(&data,buf,sizeof(Local_Magnetic_Field_Packet_t));
			break;
		}
		case Odometer_State_Packet_ID:       //ODOMETER 0x6F
		{
			Odometer_State_Packet_t data;
			memcpy(&data,buf,sizeof(Odometer_State_Packet_t));
			break;
		}
		case Geoid_Height_Packet_ID:         //GEOID_HEIGHT 0x72
		{
			Geoid_Height_Packet_t data;
			memcpy(&data,buf,sizeof(Geoid_Height_Packet_t));
			break;
		}
		case RTCM_Corrections_Packet_ID:     //RTCM_CORRECTIONS 0x73
		{
			RTCM_Corrections_Packet_t data;
			memcpy(&data,buf,sizeof(RTCM_Corrections_Packet_t));
			break;
		}
		case Wind_Packet_ID:                 //WIND 0x75
		{
			Wind_Packet_t data;
			memcpy(&data,buf,sizeof(Wind_Packet_t));
			break;
		}
		case Heave_Packet_ID:                //HEAVE 0x76
		{
			Heave_Packet_t data;
			memcpy(&data,buf,sizeof(Heave_Packet_t));
			break;
		}
		case Raw_Satellite_Data_Packet_ID:   //RAW_SATELLITE 0x77
		{
			Raw_Satellite_Data_Packet_t data;
			memcpy(&data,buf,sizeof(Raw_Satellite_Data_Packet_t));
			break;
		}
		case GNSS_DUAL_ANT_Data_Packet_ID:   //GNSS_DUAL 0x78
		{
			GNSS_DUAL_ANT_Data_Packet_t data;
			memcpy(&data,buf,sizeof(GNSS_DUAL_ANT_Data_Packet_t));
			break;
		}
		case Gimbal_State_Packet_ID:         //GIMBAL_STATE 0x7A
		{
			Gimbal_State_Packet_t data;
			memcpy(&data,buf,sizeof(Gimbal_State_Packet_t));
			break;
		}		
		case Automotive_Packet_ID:           //AUTOMOTIVE 0x7B
		{
			Automotive_Packet_t data;
			memcpy(&data,buf,sizeof(Automotive_Packet_t));
			break;
		}
		case Packet_Timer_Period_Packet_ID:         //Packet_Timer_Period 0x7C
		{
			Packet_Timer_Period_Packet_t data;
			memcpy(&data,buf,sizeof(Packet_Timer_Period_Packet_t));
			break;
		}		
		case Packets_Period_Packet_ID:           //Packets_Period 0x7D
		{
			Packets_Period_Packet_t data;
			memcpy(&data,buf,sizeof(Packets_Period_Packet_t));
			break;
		}
		case Installation_Alignment_Packet_ID://INSTALLATION_ALIGNMENT 0x80
		{
			Installation_Alignment_Packet_t data;
			memcpy(&data,buf,sizeof(Installation_Alignment_Packet_t));
			break;
		}
		case Filter_Options_Packet_ID:       //FILTER_OPTIONS 0x81
		{
			Filter_Options_Packet_t data;
			memcpy(&data,buf,sizeof(Filter_Options_Packet_t));
			break;
		}
		case GPIO_Configuration_Packet_ID:   //GPIO_CONFIG Ox82
		{
			GPIO_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Configuration_Packet_t));
			break;
		}		
		case Magnetic_Calibration_Values_Packet_ID://MAGNETIC_CAIL 0x83
		{
			Magnetic_Calibration_Values_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Values_Packet_t));
			break;
		}		
		case Magnetic_Calibration_Configuration_Packet_ID://MAGNETIC_CAIL_CONFIG 0x84
		{
			Magnetic_Calibration_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Configuration_Packet_t));
			break;
		}			
		case Magnetic_Calibration_Status_Packet_ID://MAGNETIC_CAIL_STATUS 0x85
		{
			Magnetic_Calibration_Status_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Status_Packet_t));
			break;
		}			
		case Odometer_Configuration_Packet_ID://ODOMETER_CONFIG 0x86
		{
			Odometer_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Odometer_Configuration_Packet_t));
			break;
		}	
		case Set_Zero_Orientation_Alignment_Packet_ID://SET_ZERO 0x87
		{
			Set_Zero_Orientation_Alignment_Packet_t data;
			memcpy(&data,buf,sizeof(Set_Zero_Orientation_Alignment_Packet_t));
			break;
		}		
		case Reference_Point_Offsets_Packet_ID://REFERENCE_POINT 0x88
		{
			Reference_Point_Offsets_Packet_t data;
			memcpy(&data,buf,sizeof(Reference_Point_Offsets_Packet_t));
			break;
		}			
		case User_Data_Packet_ID:             //USER_DATA 0x8A
		{
			User_Data_Packet_t data;
			memcpy(&data,buf,sizeof(User_Data_Packet_t));
			break;
		}	
		case Baud_Rates_Packet_ID:            //BAUD_SATES 0xA0
		{
			Baud_Rates_Packet_t data;
			memcpy(&data,buf,sizeof(Baud_Rates_Packet_t));
			break;
		}	
		case Sensor_Ranges_Packet_ID:         //SENSOR_RANGES 0xA1
		{
			Sensor_Ranges_Packet_t data;
			memcpy(&data,buf,sizeof(Sensor_Ranges_Packet_t));
			break;
		}	
		case GPIO_Output_Configuration_Packet_ID://GPIO_OUTPUT_CONFIG 0xA2
		{
			GPIO_Output_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Output_Configuration_Packet_t));
			break;
		}		
		case GPIO_Input_Configuration_Packet_ID: //GPIO_INPUT_CONFIG 0xA3
		{
			GPIO_Input_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Input_Configuration_Packet_t));
			break;
		}			
		case Dual_Antenna_Configuration_Packet_ID://DUAL_ANTENNA_CONFIG 0xA4
		{
			Dual_Antenna_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Dual_Antenna_Configuration_Packet_t));
			break;
		}
		case External_Position_And_Velocity_Packet_ID://EXTERNAL_POSITION_AND_VELOCITY 0x68
		{
			External_Position_And_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Position_And_Velocity_Packet_t));
			break;
		}
		case External_Position_Packet_ID:    //EXTERNAL_POSITION 0x69
		{
			External_Position_Packet_t data;
			memcpy(&data,buf,sizeof(External_Position_Packet_t));
			break;
		}
		case External_Velocity_Packet_ID:    //EXTERNAL_VELOCITY 0x6A
		{
			External_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Velocity_Packet_t));
			break;
		}
		case External_Body_Velocity_Packet_ID://EXTERNAL_BODT_VELOCITY 0x6B
		{
			External_Body_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Body_Velocity_Packet_t));
			break;
		}
		case External_Heading_Packet_ID:     //EXTERNAL_HEADING 0x6C
		{
			External_Heading_Packet_t data;
			memcpy(&data,buf,sizeof(External_Heading_Packet_t));
			break;
		}
		case External_Time_Packet_ID:        //EXTERNAL_TIME 0x70
		{
			External_Time_Packet_t data;
			memcpy(&data,buf,sizeof(External_Time_Packet_t));
			break;
		}
		case External_Depth_Packet_ID:       //EXTERNAL_DEPTH 0x71
		{
			External_Depth_Packet_t data;
			memcpy(&data,buf,sizeof(External_Depth_Packet_t));
			break;
		}
		case External_Pitot_Pressure_Packet_ID://External_Pitot_Pressure_Packet_ID 074
		{
			External_Pitot_Pressure_Packet_t data;
			memcpy(&data,buf,sizeof(External_Pitot_Pressure_Packet_t));
			break;
		}
		case External_Air_Data_Packet_ID:    //EXTERNAL_AIE_DATA 0x79
		{
			External_Air_Data_Packet_t data;
			memcpy(&data,buf,sizeof(External_Air_Data_Packet_t));
			break;
		}
		case External_Odom_Data_Packet_ID:   //EXTERNAL_ODOM_DATA 0x90
		{
			External_Odom_Data_Packet_t data;
			memcpy(&data,buf,sizeof(External_Odom_Data_Packet_t));
			break;
		}
		case External_LIDAR_Packet_ID:       //EXTERNAL_LIDAR_DATA 0x91
		{
			External_LIDAR_Packet_t data;
			memcpy(&data,buf,sizeof(External_LIDAR_Packet_t));
			break;
		}		
		case External_SLAM1_Packet_ID:       //EXTERNAL_SALAM1 0x92
		{
			External_SLAM1_Packet_t data;
			memcpy(&data,buf,sizeof(External_SLAM1_Packet_t));
			break;
		}
		case External_SLAM2_Packet_ID:       //EXTERNAL_SALAM2 0x93
		{
			External_SLAM2_Packet_t data;
			memcpy(&data,buf,sizeof(External_SLAM2_Packet_t));
			break;
		}
	}
}
