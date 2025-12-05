/*!
 *	\file		FDI_config.h
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
#ifndef __FDI_CONFIG_H
#define __FDI_CONFIG_H

/*
IMU参数部分，可以通过fdiGetParam(char* paramName)函数查看需要的数据，举例：fdiGetParam(IMU_ACC_AVG);
*/
enum IMU 
{
	IMU_TEMP0,
	IMU_ACC_AVG, //重力加速度
	IMU_TO_BODY_ALGN_ROLL,
	IMU_TO_BODY_ALGN_PITCH,
	IMU_TO_BODY_ALGN_YAW,

	IMU_LEVEL_ALGN_ROLL,
	IMU_LEVEL_ALGN_PITCH,

	//IMU_ROT;
	IMU_MAG_INCL,
	IMU_MAG_DECL,
	IMU_PRESS_SENSE,

	/*************** ACC *********************/
	IMU_RANGE_ACC,  
	IMU_ACC_BIAS_X,
	IMU_ACC_BIAS_Y,
	IMU_ACC_BIAS_Z,
	IMU_ACC_BIAS_TEMP1_X,
	IMU_ACC_BIAS_TEMP1_Y,
	IMU_ACC_BIAS_TEMP1_Z,
	IMU_ACC_BIAS_TEMP2_X,
	IMU_ACC_BIAS_TEMP2_Y,
	IMU_ACC_BIAS_TEMP2_Z,
	IMU_ACC_BIAS_TEMP3_X,
	IMU_ACC_BIAS_TEMP3_Y,
	IMU_ACC_BIAS_TEMP3_Z,
	IMU_ACC_BIAS_DTEMP_X,
	IMU_ACC_BIAS_DTEMP_Y,
	IMU_ACC_BIAS_DTEMP_Z,
	IMU_ACC_SCAL_CORR_X,
	IMU_ACC_SCAL_CORR_Y,
	IMU_ACC_SCAL_CORR_Z,
	IMU_ACC_SCAL_CORR_TEMP1_X,
	IMU_ACC_SCAL_CORR_TEMP1_Y,
	IMU_ACC_SCAL_CORR_TEMP1_Z,
	IMU_ACC_SCAL_CORR_TEMP2_X,
	IMU_ACC_SCAL_CORR_TEMP2_Y,
	IMU_ACC_SCAL_CORR_TEMP2_Z,
	IMU_ACC_SCAL_CORR_TEMP3_X,
	IMU_ACC_SCAL_CORR_TEMP3_Y,
	IMU_ACC_SCAL_CORR_TEMP3_Z,
	IMU_ACC_SCAL_CORR_DTEMP_X,
	IMU_ACC_SCAL_CORR_DTEMP_Y,
	IMU_ACC_SCAL_CORR_DTEMP_Z,
	IMU_ACC_ALGN_XY,
	IMU_ACC_ALGN_XZ,
	IMU_ACC_ALGN_YX,
	IMU_ACC_ALGN_YZ,
	IMU_ACC_ALGN_ZX,
	IMU_ACC_ALGN_ZY,
	IMU_RANGE_GYO, 
	IMU_GYO_BIAS_X,
	IMU_GYO_BIAS_Y,
	IMU_GYO_BIAS_Z,
	IMU_GYO_BIAS_TEMP1_X,
	IMU_GYO_BIAS_TEMP1_Y,
	IMU_GYO_BIAS_TEMP1_Z,
	IMU_GYO_BIAS_TEMP2_X,
	IMU_GYO_BIAS_TEMP2_Y,
	IMU_GYO_BIAS_TEMP2_Z,
	IMU_GYO_BIAS_TEMP3_X,
	IMU_GYO_BIAS_TEMP3_Y,
	IMU_GYO_BIAS_TEMP3_Z,
	IMU_GYO_BIAS_DTEMP_X,
	IMU_GYO_BIAS_DTEMP_Y,
	IMU_GYO_BIAS_DTEMP_Z,
	IMU_GYO_SCAL_CORR_X,
	IMU_GYO_SCAL_CORR_Y,
	IMU_GYO_SCAL_CORR_Z,
	IMU_GYO_SCAL_CORR_TEMP1_X,
	IMU_GYO_SCAL_CORR_TEMP1_Y,
	IMU_GYO_SCAL_CORR_TEMP1_Z,
	IMU_GYO_SCAL_CORR_TEMP2_X,
	IMU_GYO_SCAL_CORR_TEMP2_Y,
	IMU_GYO_SCAL_CORR_TEMP2_Z,
	IMU_GYO_SCAL_CORR_TEMP3_X,
	IMU_GYO_SCAL_CORR_TEMP3_Y,
	IMU_GYO_SCAL_CORR_TEMP3_Z,
	IMU_GYO_SCAL_CORR_DTEMP_X,
	IMU_GYO_SCAL_CORR_DTEMP_Y,
	IMU_GYO_SCAL_CORR_DTEMP_Z,
	IMU_GYO_ALGN_XY,
	IMU_GYO_ALGN_XZ,
	IMU_GYO_ALGN_YX,
	IMU_GYO_ALGN_YZ,
	IMU_GYO_ALGN_ZX,
	IMU_GYO_ALGN_ZY,

	IMU_GYO_TEMP_CALIB,
	IMU_GYO_TEMP_CALIB_MIN,
	IMU_GYO_TEMP_CALIB_MAX,

	IMU_ACOMPASS_A,
	IMU_ACOMPASS_B,
	IMU_ACOMPASS_C,
	IMU_ACOMPASS_D,
	IMU_ACOMPASS_E,
	IMU_MAG_BIAS_X,
	IMU_MAG_BIAS_Y,
	IMU_MAG_BIAS_Z,
	IMU_MAG_BIAS_TEMP1_X,
	IMU_MAG_BIAS_TEMP1_Y,
	IMU_MAG_BIAS_TEMP1_Z,
	IMU_MAG_BIAS_TEMP2_X,
	IMU_MAG_BIAS_TEMP2_Y,
	IMU_MAG_BIAS_TEMP2_Z,
	IMU_MAG_BIAS_TEMP3_X,
	IMU_MAG_BIAS_TEMP3_Y,
	IMU_MAG_BIAS_TEMP3_Z,
	IMU_MAG_BIAS_DTEMP_X,
	IMU_MAG_BIAS_DTEMP_Y,
	IMU_MAG_BIAS_DTEMP_Z,
	IMU_MAG_SCAL_CORR_X,
	IMU_MAG_SCAL_CORR_Y,
	IMU_MAG_SCAL_CORR_Z,
	IMU_MAG_SCAL_CORR_TEMP1_X,
	IMU_MAG_SCAL_CORR_TEMP1_Y,
	IMU_MAG_SCAL_CORR_TEMP1_Z,
	IMU_MAG_SCAL_CORR_TEMP2_X,
	IMU_MAG_SCAL_CORR_TEMP2_Y,
	IMU_MAG_SCAL_CORR_TEMP2_Z,
	IMU_MAG_SCAL_CORR_TEMP3_X,
	IMU_MAG_SCAL_CORR_TEMP3_Y,
	IMU_MAG_SCAL_CORR_TEMP3_Z,
	IMU_MAG_SCAL_CORR_DTEMP_X,
	IMU_MAG_SCAL_CORR_DTEMP_Y,
	IMU_MAG_SCAL_CORR_DTEMP_Z,
	IMU_MAG_ALGN_XY,
	IMU_MAG_ALGN_XZ,
	IMU_MAG_ALGN_YX,
	IMU_MAG_ALGN_YZ,
	IMU_MAG_ALGN_ZX,
	IMU_MAG_ALGN_ZY,
	IMU_MAG_TO_ACC_ROLL,
	IMU_MAG_TO_ACC_PITCH,
	IMU_MAG_TEMP_CALIB,
	IMU_ACC_ENABLE,
	IMU_GYO_ENABLE,
	IMU_MAG_ENABLE,
};

typedef enum comm
{
	COM_NONE,
	COM1,
	COM2,
	COM3,
	COM4,
	COM5,
	COM6,
	COM7,
}comm_t;

typedef enum commBaud
{
	COMM_BAUD_ERROR,
	//常用串口波特率
	COMM_BAUD_9600,
	COMM_BAUD_19200,
	COMM_BAUD_38400,
	COMM_BAUD_76800,
	COMM_BAUD_115200,
	COMM_BAUD_230400,
	COMM_BAUD_460800,
	COMM_BAUD_921600,
	//接近STM32F4极限的串口波特率,由于精度的问题无法靠近常用的波特率
	COMM_BAUD_2625000,
	COMM_BAUD_5250000,
	COMM_BAUD_10500000,
	//SBUS波特率
	COMM_BAUD_100000,
	//常用CAN波特率
	COMM_BAUD_250000,
	COMM_BAUD_500000,
	COMM_BAUD_1000000,
}commBaud_t;

typedef enum commStreamTypes
{
	COMM_STREAM_TYPE_NONE = 0,
	
	COMM_STREAM_TYPE_MAIN,
	COMM_STREAM_TYPE_NAV,
	COMM_STREAM_TYPE_RTCM,
#if defined EPSILON_SERIES || defined DETA100_SERIES
	COMM_STREAM_TYPE_RTCM_EC600,
#endif
	COMM_STREAM_TYPE_NMEA,
	COMM_STREAM_TYPE_NMEA_OUT,
	COMM_STREAM_TYPE_NMEA2000_OUT,
	COMM_STREAM_TYPE_FDI_CAN,
#ifdef DETA100_SERIES
	COMM_STREAM_TYPE_UBLOX,
	COMM_STREAM_TYPE_UM982,
	COMM_STREAM_TYPE_UBLOX_MOVING,
	COMM_STREAM_TYPE_UBLOX_ROVER,
#else
	COMM_STREAM_TYPE_UBLOX,
#endif	
	COMM_STREAM_TYPE_EXT_POS,        //外部位置
	COMM_STREAM_TYPE_EXT_VEL,        //外部速度
	COMM_STREAM_TYPE_EXT_PV,         //外部位置速度
	COMM_STREAM_TYPE_EXT_ATT,		 		 //外部姿态角度
	COMM_STREAM_TYPE_EXT_TIME,       //外部时间
	COMM_STREAM_TYPE_EXT_HEADING,    //外部航向
	COMM_STREAM_TYPE_EXT_DEPTH,      //外部航向
	COMM_STREAM_TYPE_EXT_SLAM1,      //外部SLAM1
	COMM_STREAM_TYPE_EXT_SLAM2,      //外部SLAM2
	COMM_STREAM_TYPE_EXT_PITOT_PRESS,//外部气压计
	COMM_STREAM_TYPE_EXT_AIR,        //外部空速管
	COMM_STREAM_TYPE_EXT_ODOM,       //外部里程计
	COMM_STREAM_TYPE_EXT_LIDAR,      //外部激光雷达
	COMM_STREAM_TYPE_OPTICAL_FLOW,	
	COMM_STREAM_TYPE_SCOUT_MINI,     //SCOUT MINI小车CAN总线编码器数据
	COMM_STREAM_TYPE_DRONECAN_CLIENT,

	//COMM_STREAM_TYPE_FDILINK      = COMM_STREAM_TYPE_MAIN,	// FDILink
	//COMM_STREAM_TYPE_QGC          = COMM_STREAM_TYPE_MAIN,	// MAVLink telemetry format
}commStreamTypes_t;

enum gpioFunctionType 
{
	GPIOS_INACTIVE                        = 0,
	GPIOS_1PPS_OUTPUT                     = 1, //1PPS Output, Digital Output                      //TIM Output 联动模式
	GPIOS_1PPS_INPUT                      = 2, //1PPS Input, Digital Input                        //TIM Input  同步模式
	GPIOS_GNSS_FIX_OUTPUT                 = 3, //GNSS Fix Output, Digital Output                  //GPO
	GPIOS_ODOMETER_INPUT                  = 4, //Odometer Input, Frequency Input                  //TIM Input  计数模式
	GPIOS_ZERO_VELOCITY_INPUT             = 5, //Zero Velocity Input, Digital Input               //GPI
	GPIOS_DISABLE_GNSS                    = 6, //Disable GNSS , Digital Input                     //GPI
	GPIOS_DISABLE_PRESSURE                = 7, //Disable Pressure , Digital Input                 //GPI
	GPIOS_SET_ZERO_ORIENTATION_ALIGNMENT  = 8, //Set Zero Orientation Alignment , Digital Output  //GPI
	GPIOS_RAW_SENSORS_PACKET_TRIGGER      = 9, //Raw Sensors Packet Trigger , Digital Output      //TIM Output 同步模式 PWM模式
	GPIOS_LEFT_WHEEL_SPEED_SENSOR         = 10,//Left Wheel Speed Sensor ,Frequency Input         //TIM Input  计数模式
	GPIOS_RIGHT_WHEEL_SPEED_SENSOR        = 11,//right Wheel Speed Sensor ,Frequency Input        //TIM Input  计数模式
	GPIOS_WHEEL_ENCODER_PHASE_A           = 12,//Wheel Encoder PhaseA ,Frequency Input            //TIM Input  AB相模式 TIM不同无法实现
	GPIOS_WHEEL_ENCODER_PHASE_B           = 13,//Wheel Encoder PhaseB ,Frequency Input            //TIM Input  AB相模式 TIM不同无法实现
	GPIOS_EVENT_1_INPUT                   = 14,//Event1 Input ,Digital Output                     //GPI
	GPIOS_EVENT_2_INPUT                   = 15,//Event2 Input ,Digital Output                     //GPI
	GPIOS_SET_RESTART                     = 16,//Set Restart , Digital Output                     //GPI
	GPIOS_UART_TX                         = 17,//Set Restart , Digital Output                     //UART Tx
	GPIOS_UART_RX                         = 18,//Set Restart , Digital Input                      //UART Rx
};

typedef enum AID
{
	AID_ACCEL_GRAVITY,
	AID_MAG_2D_MAGNETIC,
	AID_MAG_3D_MAGNETIC,
	AID_INIT_YAW_USE_MAG,
	AID_EXT_HEADING_UPDATE,
	AID_EXT_SLAM1_UPDATE,
	AID_EXT_POS_VEL_UPDATE,
	AID_GNSS_VEL_UPDATE,
	AID_GNSS_POS_UPDATE,
#if defined EPSILON_SERIES || defined DETA100_SERIES
	AID_GNSS_DUAL_ANT_HEADING_UPDATE,
#if !defined DETA100_SERIES
	AID_FFT_HEAVE_ENABLED,
#endif
#endif
	AID_GNSS_TRACK_HEADING_UPDATE,
	AID_BRO_ALT_UPDATE,
	AID_OPTICFLOW_UPDATE,
	AID_ZERO_RATE_UPDATE,
	AID_ZERO_VEL_UPDATE,
	AID_ZERO_POS_UPDATE,
	AID_ODOMETER_VEL_UPDATE,
	AID_CAR_YZ_ZERO_VEL_NHC_ENABLED,
	AID_CAR_CENT_ACCEL_NHC_ENABLED,
	AID_GYO_TRUN_ON_TARE_ENABLED,
}AID_t;

enum userDefine 
{
	USER_DEFINE_ROLL,
	USER_DEFINE_PITCH,
	USER_DEFINE_YAW,
	USER_DEFINE_VELN,
	USER_DEFINE_VELE,
	USER_DEFINE_VELD,
	USER_DEFINE_HOLDLAT_1,
	USER_DEFINE_HOLDLAT_2,
	USER_DEFINE_HOLDLON_1,
	USER_DEFINE_HOLDLON_2,
	USER_DEFINE_L_IMU_POINT_X,
	USER_DEFINE_L_IMU_POINT_Y,
	USER_DEFINE_L_IMU_POINT_Z,
	USER_DEFINE_CAN_ID,
	USER_DEFINE_DRONECAN_ID,
};

enum DGNSS
{
	QXWZ_DSK_KEY,
	QXWZ_DSK_SECRET,
	QXWZ_DEV_ID,
	QXWZ_DEV_TYPE,
	NTRIP_SVR_DOMAIN,
	NTRIP_SVR_PORT,
	NTRIP_ACCOUNT,
	NTRIP_PASSWORD,
	NTRIP_MOUNT,
	BASE_STATION_SOURCE,
	USR_AUTHEMTICATION,
	NET_INFO_IMEI,
	NET_INFO_CCID,
};

#define Freq_NONE							 0
#define Freq_1								 1
#define Freq_2								 2
#define Freq_5								 5
#define Freq_10								 10
#define Freq_20								 20
#define Freq_50								 50
#define Freq_100							 100
#define Freq_200							 200

#define GPIO1								1
#define GPIO2								2

/*
补充说明：
1.发送命令后必须加<CR><LF>，否则命令不会执行，在代码中呈现为每条语句后面有\r\n
2.模块接收命令后一般需要等待1s左右的执行时间才可以处理下一条命令，建议间隔1.5s以上发送下一条命令
3.命令执行成功后，会返回*#OK，可以通过检测接收到的数据中有没有这个返回值判断有没有接收成功
4.Main类型的COM端口最多只能有一个，用于和上位机通信。NAV和Main类型的端口可以用来通过COM端口配置命令
5.如果对配置的一些名词不太了解，可以查看模块相关手册，手册上有具体介绍
*/
int fdiSetConfig(void);																//进入配置模式，示例：#fconfig
int fdiSetDeconfig(void);															//退出配置模式，恢复到导航模式，示例：#fdeconfig
int fdiSetReboot(void);																//重新设备，需要确认，所有未保存的配置不会保存，也不会生效，示例：#freboot
int fdiSetReset(void);																//恢复出厂初始值，需要确认，示例：#freset
int fdiSetSave(void);																	//配置保存，示例：#fsave

int	fdiComGetConfigAxis(void);												//查询设备安装方向，示例：#faxis
int fdiComGetConfigAnte(void);												//查询双天线航向与载体的前后夹角，示例：#fante ---主天线到从天线为基线矢量正方向，顺时针为偏角的正方向
int fdiComGetConfigMsg(void);													//查询所有支持的数据包，数据包ID以及当前发送频率，示例：#fmsg
int fdiGetParam(char* paramName);											//查询所有参数，在此之后的配置函数都是这个函数封装而来，方便用户调用
int fdiComGetConfigBaud(int Com);											//查询COM波特率，示例：#fparam set COMM_BAUD2 ---输出COMM_BAUD2=5，表示波特率为115200bps
int	fdiComGetAIDmag3DMagetic(void);										//查询3D磁力计融合开关状态，示例：#fparam get AID_MAG_3D_MAGNETIC
int	fdiComGetAIDmag2DMagetic(void);										//查询2D磁力计融合开关状态，示例：#fparam get AID_MAG_2D_MAGNETIC
int fdiComGetConfigDgnss(int DGNSS);									//查询GNSS设置参数

int fdiComSetConfigAxis(char* flip, float rot);				//配置设备安装方向，示例：#faxis x 0.5 ---绕x轴顺时针旋转0.5度，rot范围0~360度
int fdiComSetConfigAnteHeadbias(float angle);					//配置双天线航向与载体的前后夹角，示例：#fanteheadbias angle ---angle范围0~360度
int fdiComSetConfigAnteBaseline(float length);				//配置双天线之间的基线长度，示例：#fantebaseline length ---length单位是米
int fdiComSetConfigAnteArm(float x, float y, float z);//配置主天线(也叫移动基站)到IMU的杆臂(模组系下)，示例：#fanterm 0.5 0.5 0.5 ---主天线在惯导系下坐标为(0.5m，0.5m，0.5m)
int fdiComSetConfigPacketSentMsg(char* msg, int freq);//配置指定数据包的发送频率，频率不能随意给定，示例：#fmsg 40 100 ---设置ID40包发送频率100Hz
int fdiComSetConfigPacketCloseMsg(char* msg);					//配置指定数据包不发送，本质上是配置发送频率为0，示例：#fmsg 40 0
int fdiComSetConfigImucailedLevel(void);							//坐标系调平，需要在水平静止状态下执行，示例：#fimucal_level
int fdiComSetConfigImucailedACCE(void);								//校准陀螺仪常值零偏，需要在静止状态下执行，示例：#fimucal_acce
int fdiComSetConfigImucailedGyro(void);								//校准加表常值零偏，需要在水平静止状态下执行，示例：#fimucal_gyro
int fdiSetParam(char* paramName, float paramValue);		//配置所有参数，在此之后的配置函数都是这个函数封装而来，方便用户调用
int fdiComSetConfigBaud(int COM, int BAUD);						//配置COM波特率，示例：#fparam set COMM_BAUD2 COMM_BAUD_921600 ---将端口2的波特率改为921600bps
int fdiComSetConfigType(int COM, int Type);						//配置COM类型，示例：#fparam set COMM_BAUD2 COMM_STREAM_TYPE_NAV ---将端口2的类型改为NAV
int fdiComSetConfigGPIOs(int GPIO, int Fun);					//配置GPIOs功能，第二个参数为GPIO口的模式，示例#fparam set GPIO_1_FUNCTION GPIOS_1PPS_OUTPUT --- GPIO1->OUTPUT
int	fdiComSetAIDmag3DMagetic(void);										//开启3D磁力计融合开关，示例：#fparam set AID_MAG_3D_MAGNETIC 1
int	fdiComResetAIDmag3DMagetic(void);									//关闭3D磁力计融合开关，示例：#fparam set AID_MAG_3D_MAGNETIC 0
int	fdiComSetAIDmag2DMagetic(void);										//开启2D磁力计融合开关，示例：#fparam set AID_MAG_2D_MAGNETIC 1
int	fdiComResetAIDmag2DMagetic(void);									//关闭2D磁力计融合开关，示例：#fparam set AID_MAG_2D_MAGNETIC 0
int fdiComSetConfigAID(int AID, int Status);					//配置融合开关，在模块手册里的SPKF融合开关章节有具体说明，示例：#fparam set AID_MAG_3D_MAGNETIC 1
int fdiComSetConfigUserDefine(int type, float num);		//配置人工输入参数，用户可以输入一些已知参数，使惯导提高准确性，示例：#fparam set USER_DEFINE_ROLL 0.5
int fdiComSetDgnss(int DGNSS, char* paramValue);			//配置DGNSS参数，建议看手册配置相关内容，示例；#fdgnss set RTCM_TYPE 3 ---切换为NTRIP配置

#endif