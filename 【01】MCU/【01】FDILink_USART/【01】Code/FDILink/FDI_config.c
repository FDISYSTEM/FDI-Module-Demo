#include "stm32f1xx_hal.h"
#include <string.h>
#include "FDI_config.h"

extern UART_HandleTypeDef huart1;

int fdiSetConfig(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"#fconfig\r\n", sizeof("#fconfig\r\n") - 1);
	HAL_Delay(1500);
}

int fdiSetDeconfig(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"#fdeconfig\r\n",sizeof("#fdeconfig\r\n") - 1);
	HAL_Delay(1500);
}

int fdiSetReboot(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#freboot\r\n", sizeof("#freboot\r\n") - 1);
	HAL_Delay(2000);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"y\r\n", sizeof("y\r\n") - 1);
	HAL_Delay(1500);
}

int fdiSetReset(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#freset\r\n",sizeof("#freset\r\n") - 1);
	HAL_Delay(1500);
}

int fdiSetSave(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fsave\r\n", sizeof("#fsave\r\n") - 1);
	HAL_Delay(1500);
}



int fdiComGetConfigAxis(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#faxis\r\n", sizeof("#faxis\r\n") - 1);
	HAL_Delay(1500);
}

int fdiComGetConfigAnte(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fante\r\n", sizeof("#fante\r\n") - 1);
	HAL_Delay(1500);
}

int fdiComGetConfigMsg(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fmsg\r\n", sizeof("#fmsg\r\n") - 1);
	HAL_Delay(1500);
}

int fdiGetParam(char* paramName)
{
	char send_buff[128];
	sprintf(send_buff, "#fparam get %s\r\n", paramName);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComGetConfigBaud(int COM)
{
	switch(COM)
	{
		case 1:
			fdiGetParam("COMM_BAUD1");break;
		case 2:
			fdiGetParam("COMM_BAUD2");break;
		case 3:
			fdiGetParam("COMM_BAUD3");break;
		case 4:
			fdiGetParam("COMM_BAUD4");break;
		case 5:
			fdiGetParam("COMM_BAUD5");break;
		case 6:
			fdiGetParam("COMM_BAUD6");break;
		case 7:
			fdiGetParam("COMM_BAUD7");break;
	}
}

int	fdiComGetAIDmag3DMagetic(void)
{
	fdiGetParam("AID_MAG_3D_MAGNETIC");
}

int	fdiComGetAIDmag2DMagetic(void)
{
	fdiGetParam("AID_MAG_2D_MAGNETIC");
}

int fdiGetDgnss(int DGNSS)
{
	char send_buff[128];
	switch(DGNSS)
	{
		case QXWZ_DSK_KEY:
			sprintf(send_buff, "#fdgnss get %s\r\n", "QXWZ_DSK_KEY");break;
		case QXWZ_DSK_SECRET:
			sprintf(send_buff, "#fdgnss get %s\r\n", "QXWZ_DSK_SECRET");break;
		case QXWZ_DEV_ID:
			sprintf(send_buff, "#fdgnss get %s\r\n", "QXWZ_DEV_ID");break;
		case QXWZ_DEV_TYPE:
			sprintf(send_buff, "#fdgnss get %s\r\n", "QXWZ_DEV_TYPE");break;
		case NTRIP_SVR_DOMAIN:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NTRIP_SVR_DOMAIN");break;
		case NTRIP_SVR_PORT:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NTRIP_SVR_PORT");break;
		case NTRIP_ACCOUNT:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NTRIP_ACCOUNT");break;
		case NTRIP_PASSWORD:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NTRIP_PASSWORD");break;
		case NTRIP_MOUNT:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NTRIP_MOUNT");break;
		case BASE_STATION_SOURCE:
			sprintf(send_buff, "#fdgnss get %s\r\n", "BASE_STATION_SOURCE");break;
		case USR_AUTHEMTICATION:
			sprintf(send_buff, "#fdgnss get %s\r\n", "USR_AUTHEMTICATION");break;
		case NET_INFO_IMEI:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NET_INFO_IMEI");break;
		case NET_INFO_CCID:
			sprintf(send_buff, "#fdgnss get %s\r\n", "NET_INFO_CCID");break;
	}
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
	return 0;
}


int fdiComSetConfigAxis(char* flip, float rot)
{
	if(rot < 0 || rot > 360)
		return -1;
	char send_buff[128];
	sprintf(send_buff, "#faxis %s %f\r\n", flip, rot);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigAnteHeadbias(float angle)
{
	if(angle < 0 || angle > 360)
		return -1;
	char send_buff[128];
	sprintf(send_buff, "#fanteheadbias %f\r\n", angle);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigAnteBaseline(float length)
{
	if(length < 0)
		return -1;
	char send_buff[128];
	sprintf(send_buff, "#fantebaseline %f\r\n", length);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigAnteArm(float x, float y, float z)
{
	char send_buff[128];
	sprintf(send_buff, "#fantearm %f %f %f\r\n", x, y, z);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigPacketSentMsg(char* msg, int freq)
{
	char send_buff[128];
	sprintf(send_buff, "#fmsg %s %d\r\n", msg, freq);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigPacketCloseMsg(char* msg)
{
	char send_buff[128];
	sprintf(send_buff, "#fmsg %s 0\r\n", msg);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigImucailedLevel(void)
{
	char send_buff[128];
	sprintf(send_buff, "#fimucal_level\r\n");
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigImucailedAcce(void)
{
	char send_buff[128];
	sprintf(send_buff, "#fimucal_acce\r\n");
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigImucailedGyro(void)
{
	char send_buff[128];
	sprintf(send_buff, "#fimucal_gyro\r\n");
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiSetParam(char* paramName, float paramValue)
{
	char send_buff[128];
	sprintf(send_buff, "#fparam set %s %f\r\n", paramName, paramValue);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}

int fdiComSetConfigBaud(int COM, int BAUD)
{
	switch(COM)
	{
		case 1:
			fdiSetParam("COMM_BAUD1", BAUD);break;
		case 2:
			fdiSetParam("COMM_BAUD2", BAUD);break;
		case 3:
			fdiSetParam("COMM_BAUD3", BAUD);break;
		case 4:
			fdiSetParam("COMM_BAUD4", BAUD);break;
		case 5:
			fdiSetParam("COMM_BAUD5", BAUD);break;
		case 6:
			fdiSetParam("COMM_BAUD6", BAUD);break;
		case 7:
			fdiSetParam("COMM_BAUD7", BAUD);break;
	}
}

int fdiComSetConfigType(int COM, int Type)
{
	switch(COM)
	{
		case 1:
			fdiSetParam("COMM_STREAM_TYP1", Type);break;
		case 2:
			fdiSetParam("COMM_STREAM_TYP2", Type);break;
		case 3:
			fdiSetParam("COMM_STREAM_TYP3", Type);break;
		case 4:
			fdiSetParam("COMM_STREAM_TYP4", Type);break;
		case 5:
			fdiSetParam("COMM_STREAM_TYP5", Type);break;
		case 6:
			fdiSetParam("COMM_STREAM_TYP6", Type);break;
		case 7:
			fdiSetParam("COMM_STREAM_TYP7", Type);break;
	}
}

int fdiComSetConfigGPIOs(int GPIO, int Fun)
{
	switch(GPIO)
	{
		case 1:
			fdiSetParam("GPIO_1_FUNCTION", Fun);break;
		case 2:
			fdiSetParam("GPIO_2_FUNCTION", Fun);break;
	}
}

int	fdiComSetAIDmag3DMagetic(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fparam set AID_MAG_3D_MAGNETIC 1\r\n", sizeof("#fparam set AID_MAG_3D_MAGNETIC 1\r\n") - 1);	
	HAL_Delay(1500);
}

int	fdiComResetAIDmag3DMagetic(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fparam set AID_MAG_3D_MAGNETIC 0\r\n", sizeof("#fparam set AID_MAG_3D_MAGNETIC 0\r\n") - 1);	
	HAL_Delay(1500);
}

int	fdiComSetAIDmag2DMagetic(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fparam set AID_MAG_2D_MAGNETIC 1\r\n", sizeof("#fparam set AID_MAG_2D_MAGNETIC 1\r\n") - 1);	
	HAL_Delay(1500);
}

int	fdiComResetAIDmag2DMagetic(void)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)"#fparam set AID_MAG_2D_MAGNETIC 0\r\n", sizeof("#fparam set AID_MAG_2D_MAGNETIC 0\r\n") - 1);	
	HAL_Delay(1500);
}

int fdiComSetConfigAID(int AID, int Status)
{
	switch(AID)
	{
		case AID_ACCEL_GRAVITY:
			fdiSetParam("AID_ACCEL_GRAVITY", Status);break;
		case AID_MAG_2D_MAGNETIC:
			fdiSetParam("AID_MAG_2D_MAGNETIC", Status);break;
		case AID_MAG_3D_MAGNETIC:
			fdiSetParam("AID_MAG_3D_MAGNETIC", Status);break;
		case AID_INIT_YAW_USE_MAG:
			fdiSetParam("AID_INIT_YAW_USE_MAG", Status);break;
		case AID_EXT_HEADING_UPDATE:
			fdiSetParam("AID_EXT_HEADING_UPDATE", Status);break;
		case AID_EXT_SLAM1_UPDATE:
			fdiSetParam("AID_EXT_SLAM1_UPDATE", Status);break;
		case AID_EXT_POS_VEL_UPDATE:
			fdiSetParam("AID_EXT_POS_VEL_UPDATE", Status);break;
		case AID_GNSS_VEL_UPDATE:
			fdiSetParam("AID_GNSS_VEL_UPDATE", Status);break;
		case AID_GNSS_POS_UPDATE:
			fdiSetParam("AID_GNSS_POS_UPDATE", Status);break;
		
		#if defined EPSILON_SERIES || defined DETA100_SERIES
		case AID_GNSS_DUAL_ANT_HEADING_UPDATE:
			fdiSetParam("AID_GNSS_DUAL_ANT_HEADING_UPDATE", Status);break;
		#if !defined DETA100_SERIES
		case AID_FFT_HEAVE_ENABLED:
			fdiSetParam("AID_FFT_HEAVE_ENABLED", Status);break;
		#endif
		#endif
		
		case AID_GNSS_TRACK_HEADING_UPDATE:
			fdiSetParam("AID_GNSS_TRACK_HEADING_UPDATE", Status);break;
		case AID_BRO_ALT_UPDATE:
			fdiSetParam("AID_BRO_ALT_UPDATE", Status);break;
		case AID_OPTICFLOW_UPDATE:
			fdiSetParam("AID_OPTICFLOW_UPDATE", Status);break;
		case AID_ZERO_RATE_UPDATE:
			fdiSetParam("AID_ZERO_RATE_UPDATE", Status);break;
		case AID_ZERO_VEL_UPDATE:
			fdiSetParam("AID_ZERO_VEL_UPDATE", Status);break;
		case AID_ZERO_POS_UPDATE:
			fdiSetParam("AID_ZERO_POS_UPDATE", Status);break;
		case AID_ODOMETER_VEL_UPDATE:
			fdiSetParam("AID_ODOMETER_VEL_UPDATE", Status);break;
		case AID_CAR_YZ_ZERO_VEL_NHC_ENABLED:
			fdiSetParam("AID_CAR_YZ_ZERO_VEL_NHC_ENABLED", Status);break;
		case AID_CAR_CENT_ACCEL_NHC_ENABLED:
			fdiSetParam("AID_CAR_CENT_ACCEL_NHC_ENABLED", Status);break;
		case AID_GYO_TRUN_ON_TARE_ENABLED:
			fdiSetParam("AID_GYO_TRUN_ON_TARE_ENABLED", Status);break;
	}
}

int fdiComSetConfigUserDefine(int type, float num)
{
	switch(type)
	{
		case USER_DEFINE_ROLL:
			fdiSetParam("USER_DEFINE_ROLL", num);break;
		case USER_DEFINE_PITCH:
			fdiSetParam("USER_DEFINE_PITCH", num);break;
		case USER_DEFINE_YAW:
			fdiSetParam("USER_DEFINE_YAW", num);break;
		case USER_DEFINE_VELN:
			fdiSetParam("USER_DEFINE_VELN", num);break;
		case USER_DEFINE_VELE:
			fdiSetParam("USER_DEFINE_VELE", num);break;
		case USER_DEFINE_VELD:
			fdiSetParam("USER_DEFINE_VELD", num);break;
		case USER_DEFINE_HOLDLAT_1:
			fdiSetParam("USER_DEFINE_HOLDLAT_1", num);break;
		case USER_DEFINE_HOLDLAT_2:
			fdiSetParam("USER_DEFINE_HOLDLAT_2", num);break;
		case USER_DEFINE_HOLDLON_1:
			fdiSetParam("USER_DEFINE_HOLDLON_1", num);break;
		case USER_DEFINE_HOLDLON_2:
			fdiSetParam("USER_DEFINE_HOLDLON_2", num);break;
		case USER_DEFINE_L_IMU_POINT_X:
			fdiSetParam("USER_DEFINE_L_IMU_POINT_X", num);break;
		case USER_DEFINE_L_IMU_POINT_Y:
			fdiSetParam("USER_DEFINE_L_IMU_POINT_Y", num);break;
		case USER_DEFINE_L_IMU_POINT_Z:
			fdiSetParam("USER_DEFINE_L_IMU_POINT_Z", num);break;
		case USER_DEFINE_CAN_ID:
			fdiSetParam("USER_DEFINE_CAN_ID", num);break;
		case USER_DEFINE_DRONECAN_ID:
			fdiSetParam("USER_DEFINE_DRONECAN_ID", num);break;
	}
}

int fdiComSetDgnss(int DGNSS, char* paramValue)
{
	char* paramName;
	memset(paramName, 0, strlen(paramName));
	switch(DGNSS)
	{
		case QXWZ_DSK_KEY:
			memcpy(paramName, "QXWZ_DSK_KEY", strlen("QXWZ_DSK_KEY"));break;
		case QXWZ_DSK_SECRET:
			memcpy(paramName, "QXWZ_DSK_SECRET", strlen("QXWZ_DSK_SECRET"));break;
		case QXWZ_DEV_ID:
			memcpy(paramName, "QXWZ_DEV_ID", strlen("QXWZ_DEV_ID"));break;
		case QXWZ_DEV_TYPE:
			memcpy(paramName, "QXWZ_DEV_TYPE", strlen("QXWZ_DEV_TYPE"));break;
		case NTRIP_SVR_DOMAIN:
			memcpy(paramName, "NTRIP_SVR_DOMAIN", strlen("NTRIP_SVR_DOMAIN"));break;
		case NTRIP_SVR_PORT:
			memcpy(paramName, "NTRIP_SVR_PORT", strlen("NTRIP_SVR_PORT"));break;
		case NTRIP_ACCOUNT:
			memcpy(paramName, "NTRIP_ACCOUNT", strlen("NTRIP_ACCOUNT"));break;
		case NTRIP_PASSWORD:
			memcpy(paramName, "NTRIP_PASSWORD", strlen("NTRIP_PASSWORD"));break;
		case NTRIP_MOUNT:
			memcpy(paramName, "NTRIP_MOUNT", strlen("NTRIP_MOUNT"));break;
		case BASE_STATION_SOURCE:
			memcpy(paramName, "BASE_STATION_SOURCE", strlen("BASE_STATION_SOURCE"));break;
		case USR_AUTHEMTICATION:
			memcpy(paramName, "USR_AUTHEMTICATION", strlen("USR_AUTHEMTICATION"));break;
		case NET_INFO_IMEI:
			memcpy(paramName, "NET_INFO_IMEI", strlen("NET_INFO_IMEI"));break;
		case NET_INFO_CCID:
			memcpy(paramName, "NET_INFO_CCID", strlen("NET_INFO_CCID"));break;
	}
	char send_buff[128];
	sprintf(send_buff, "#fdgnss set %s %s\r\n", paramName, paramValue);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
	HAL_Delay(1500);
}
