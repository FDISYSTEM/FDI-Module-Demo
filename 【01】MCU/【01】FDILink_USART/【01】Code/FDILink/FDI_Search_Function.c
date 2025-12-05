#include "main.h"
#include "fdilink_decode.h"
#include "FDILink.h"
#include "FDI_send.h"
#include "usb_device.h"
#include "FDI_config.h"
#include "FDI_Search_Function.h"
void	fdiComSearchFunction(void)
{
/*!
 *  //准备进入配置模式		
 *	Enter configuration mode.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiComSetConfig(void);																//准备进入配置模式		
	
/*!
 *  退出配置模式
 *	Exit configuration mode.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		FDI_NO_ERROR if we have exited configuration mode.
 */
int fdiSetDeconfig(void);																	//退出配置模式

/*!
 *  重启设备（重要数据的更新需要重启设备）
 *	Restart the device (the device needs to be restarted for the update of important data).
 *	\param[out]	None
 *	\param[in]	None
 *	\return		FDI_NO_ERROR if we have restarted the device.
 */
int fdiSetReboot(void);																		//重启设备

/*!
 *  恢复出厂设置
 *	Restore factory settings.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		FDI_NO_ERROR if we have restored factory settings.
 */
int fdiSetReset(void);																		//恢复出厂设置

/*!
 *  保存已修改的配置（重要的数据更新需要保存）
 *	Save modified configuration.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiSetSave(void);																			//保存已修改的配置
	
/*!
 *  设置设备的安装方向：flip可选为绕x、y、z轴旋转，rot（0-360度）为从设备顶部看去顺时针为正方向
 *	Set the installation direction of the equipment.
 *	\param[out]	None
 *	\param[in]	flip - choose one of value from x, y or z; rot - range from 0 to 360(degree)
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigAxis(char* flip, float rot);							//配置设备的安装方向					//(1.设置轴功能,2.设置角度)
	{
		fdiComSetConfigAxis("x", 0);													//(x轴0度)
		fdiComSetConfigAxis("y", 0);													//(y轴0度)
		fdiComSetConfigAxis("z", 0);													//(z轴0度)
	}
	
	
/*!
 *  配置双天线航向偏角，其中angle为角度值（0-360度）
 *	Configure dual antenna heading angle.
 *	\param[out]	None
 *	\param[in]	angle - dual antenna heading angle.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigAnteHeadbias(float angle);								//配置双天线航向偏角					//(设置航向偏角)
	{
		fdiComSetConfigAnteHeadbias(0);												//(航向偏角0)
	}
	
/*!
*  配置双天线之间的基线长度，length单位为米（m）
 *	Configure dual antenna baseline length.
 *	\param[out]	None
 *	\param[in]	length - dual antenna baseline length.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigAnteBaseline(float length);							//配置双天线之间的基线长度		//(设置基线长度)
	{
		fdiComSetConfigAnteBaseline(0);													//(基线长度0)
	}
	
/*!
 *  配置设备数据口的波特率，随着设备型号的改变可用数据口也会发生变化，具体参考各型号手册
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigBaud(int COM, int BAUD);								//配置设备数据口的波特率			//(1.设置COM口，2.设置波特率)
	{
		fdiComSetConfigBaud(COM1,COMM_BAUD_921600);						//(COM1波特率921600)
		fdiComSetConfigBaud(COM2,COMM_BAUD_921600);						//(COM2波特率921600)
		fdiComSetConfigBaud(COM2,COMM_BAUD_115200);						//(COM2波特率115200)
	}
	
/*!
 *  配置设备数据口的功能，随着设备型号的改变功能也会发生变化，具体参考各型号手册
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigType(int COM, int Type);								//配置设备数据口的功能				//(1.设置COM口，2.设置数据口功能)
	{
		fdiComSetConfigType(COM1,COMM_STREAM_TYPE_NAV);				//(COM1数据口功能COMM_STREAM_TYPE_NAV)
	}
	
/*!
 *  配置USER_DEFINE数据
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */	
int fdiComSetConfigUserDefine(int type, float num);				//配置USER_DEFINE数据				//(1.USE_DEFINE_ID,2.NUM)
	{
		fdiComSetConfigUserDefine(USER_DEFINE_CAN_ID,1);				//(CAN1)
	}
	
/*!
 *  配置GPIOs功能
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigGPIOs(int GPIO, int Fun);								//配置GPIOs功能							//(1.设置GPIO口,2.GPIO口的模式)
	{
		fdiComSetConfigGPIOs(GPIO2, GPIOS_1PPS_OUTPUT);					//(GPIO2->OUTPUT)
	}
	
/*!
 *  配置AID开关
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiComSetConfigAID(int AID, int Status);							//配置AID开关								//(1.设置AID功能,2.开关)
	{
		fdiComSetConfigAID(AID_ACCEL_GRAVITY, ENABLE);					//(AID_ACCEL_GRAVITY->开)
		fdiComSetConfigAID(AID_ACCEL_GRAVITY, DISABLE);					//(AID_ACCEL_GRAVITY->关)
	}
	
/*!
 *  配置GNSS主天线到IMU的杆臂命令
 *	Configure lever arm command from GNSS main antenna to IMU.
 *	\param[out]	None
 *	\param[in]	(x,y,z)
 *	\return		FDI_NO_ERROR if we have set the param.
 */	
int fdiComSetConfigAnteArm(float x, float y, float z);					//配置GNSS主天线到IMU杆臂的坐标//(1.x轴,2.y轴,3.z轴)
	{
		fdiComSetConfigAnteArm(0, 0, 0);											//(x轴->0,y轴->0,z轴->0)
		fdiComSetConfigAnteArm(30,30,30);											//(x轴->30,y轴->30,z轴->30)
	}
	
/*!
 *  配置参数，paramName为需要设置的参数名称，paramValue为设置参数的数值（10进制）
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */
int fdiSetParam(char* paramName, float paramValue);				//配置参数										//(1.参数名称，2.参数数值)
	
	
/*!
 *  配置GNSS设置参数，paramName为需要设置的参数名称，paramValue为设置参数的数值（10进制）
 *	configuration parameter.
 *	\param[out]	None
 *	\param[in]	paramName - Parameter name to be obtained; paramValue - Set the value of the parameter.
 *	\return		FDI_NO_ERROR if we have set the param.
 */	
int fdiComSetDgnss(int DGNSS, char* paramValue);					//配置GNSS设置参数		
	{
		fdiComSetDgnss(QXWZ_DSK_KEY, "0");										//(QXWZ_DSK_KEY->0)
		fdiComSetDgnss(QXWZ_DEV_ID, "0");											//(QXWZ_DEV_ID->0)
		fdiComSetDgnss(NTRIP_SVR_PORT, "0");									//(QXWZ_DEV_ID->0)		
	}	

/*!
 *  配置外部SLAM1数据包，所有外部数据包依照此格式配置（所有结构体在fdiDecodeBuffer中）
 *	Configure external SLAM1 packets. All external packets are configured in this format.
 *	\param[out]	None
 *	\param[in]	None
 *	\return		None
 */
void fdiSendSLAM1Datapacke(void);															//发送外部SLAM1数据包
	
/*!
 *  发送配置后的外部数据包给DETA10，type为数据包ID，buffer为配置后的数据包，length为buffer包含字节的长度
 *	Send configured external packets.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiSendExternalData(int type, void* buffer, int length);	//发送外部数据包

/**
  * @brief Initialize the FDILink status structure.
	* @return return 0 (always).
  */	
int fdiComProtocolInit(FDILink_Status_t* FDILink);						//fdi_COM口协议初始化

/**
  * @brief Send a FDI link frame.
  * @param[in] type Frame type.
  * @param[in] len Length of the data buffer.	(0-255)
	* @param[in] type Frame type.
	* @param[in] FDILink Pointer to the FDILink status structure.
	*	@param[in] buf Pointer to the data buffer.
	* @return Number of bytes sent.
  */
int fdiComProtocolSend(FDILink_Status_t* FDILink, uint8_t type, uint8_t * buf, int len);//fdi_COM口协议发送

/*!
 *  发送配置后的外部数据包给DETA10，type为数据包ID，buffer为配置后的数据包，length为buffer包含字节的长度
 *	Send configured external packets.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiComBufferTrans(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len);//fdi_Buffer发送

/*!
 *  发送配置后的外部数据包给DETA10，type为数据包ID，buffer为配置后的数据包，length为buffer包含字节的长度
 *	Send configured external packets.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
int fdiComProtocolReceive(FDILink_Status_t* FDILink, uint8_t * buf, int len);//fdi_COM口协议接收

/*!
 *  请求数据帧并解析，ID为要请求的数据帧ID编号。系统会返回当前时刻对应数据输出，如果该帧被设置为固定频率输出，则会持续输出。
 *  使用此指令会自动解析要获得的数据至构建好的结构体中，fdiDecodeBuffer中配置了部分数据包，用户可自行添加需要解析的数据包
 *	Request and analysis data frame.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */	
int fdiGetPacket(uint8_t ID);																	//查询数据帧并解析
int fdiComGetAxis(void);																			//查询设备安装的方向
int fdiComGetAnte(void);																			//查询双天线航向与载体前向夹角							
int fdiGetDgnss(int DGNSS);																		//读取GNSS设置参数
int fdiComGetParam(char* paramName);													//读取参数

}