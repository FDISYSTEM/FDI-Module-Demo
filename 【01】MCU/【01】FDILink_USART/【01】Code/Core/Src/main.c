/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdilink_decode.h"
#include "FDILink.h"
#include "FDI_send.h"
#include "FDI_receive.h"
#include "FDI_config.h"
#include "FDI_Search_Function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static FDILink_Status_t _FDILink;
static uint8_t 					RxBuffer[256];

/**
  * @brief 串口1接收数据空闲回调函数，用于调用FDILink解码函数解析数据
  * @param[in] huart
  */ 
void UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
	if(huart ->Instance == USART1)
	{
		int length = 256 - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		fdiComProtocolReceive(&_FDILink, RxBuffer, length);
		HAL_UART_Receive_DMA(huart, RxBuffer, 256);
	}
}

/* USER CODE END 0 */

/**
关于函数的一些简单介绍放置在对应的头文件里，用户可以不考虑具体实现，直接调用自己需要的函数

本函数演示一套通用的使用流程
用户想要部署到自己的工程时，只需将FDILink文件夹下的文件添加到工程中，调用相关函数即可
添加FDILink.c/h时，需要同时添加CRC_Table.c和fdilink_decode.h
添加FDI_config.c/h时，需要将"stm32f1xx_hal.h"改为当前芯片对应的头文件，huart1改为当前芯片与模块通信的串口
添加FDI_receive.c/h时，无需修改，可以通过将函数内的局部变量改为全局变量的方式，在witch里面查看帧结构体里面的数据
添加FDI_send.c/h时，需要将"stm32f1xx_hal.h"改为当前芯片对应的头文件，huart1改为当前芯片与模块通信的串口
  */
int main(void)
{
  /* MCU Configuration*/
  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
	
  /* USER CODE BEGIN 1 */
//使能UART1的IDLE中断，用于检测USART1接收空闲事件 
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//开启UART1的DMA接收，将数据接收到RxBuffer中，长度为256字节
  HAL_UART_Receive_DMA(&huart1, RxBuffer, 256);
//初始化FDILink结构体
	fdiComProtocolInit(&_FDILink);
//———————————————————————指令配置模式———————————————————————
////进入配置模式
//	fdiSetConfig();
////配置COM端口2的波特率为921600
//	fdiComSetConfigBaud(COM2,COMM_BAUD_921600);
////配置COM端口2的类型为NAV
//	fdiComSetConfigType(COM2,COMM_STREAM_TYPE_NAV);
////配置用户给定横滚为1.1度
//	fdiComSetConfigUserDefine(USER_DEFINE_ROLL,1.1);
////配置GPIO2的模式为输出
//	fdiComSetConfigGPIOs(GPIO2, GPIOS_1PPS_OUTPUT);
////开启2D磁力计融合开关	
//	fdiComSetConfigAID(AID_MAG_2D_MAGNETIC, ENABLE);
////配置安装方向沿Y轴顺时针旋转13.5度
//	fdiComSetConfigAxis("y", 13.5);
////查询双天线航向与载体的前后夹角
//	fdiComGetConfigAnte();
////配置双天线航向与载体的前后夹角为0.23度
//	fdiComSetConfigAnteHeadbias(0.23);
////配置双天线基线长度为1.1m
//	fdiComSetConfigAnteBaseline(1.1);
////配置主天线到IMU的杆臂
//	fdiComSetConfigAnteArm(0.11, 0.12, 0.13);
////校准加表常值零偏
//	fdiComSetConfigImucailedGyro();
////配置数据包发送频率
//	fdiComSetConfigPacketSentMsg(MSG_AHRS, Freq_10);
////关闭数据包发送
//	fdiComSetConfigPacketCloseMsg(MSG_IMU);
////查看MSG_IMU信息
//	fdiGetParam("MSG_IMU");
////查看COM3的波特率
//	fdiGetParam("COMM_BAUD3");
////配置COM3的波特率为921600
//	fdiSetParam("COMM_BAUD3", 8);
////保存配置
//	fdiSetSave();
////重启
//	fdiSetReboot();

//———————————————————————数据帧请求模式———————————————————————
//	fdiGetPacket(0x40);
//	fdiGetPacket(0x51);

//———————————————————————用户外部输入数据———————————————————————
//	fdiSendSLAM1Datapacke();
	

/* USER CODE END 1 */	 
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/**
  * @brief 串口发送
  * @param [1]ch
  * @param [2]FILE * fp
  * @retval None
  */
int fputc(int ch,FILE * fp)
{
	HAL_UART_Transmit(&huart1,(uint8_t * )&ch,1,0xffff);
	return ch;
}

/**
  * @brief 串口接收
  * @param [1]ch
  * @param [2]FILE * fp
  * @retval None
  */
int fgetc(FILE *f)
{
	int ch;
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
	HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return (ch);
}

 
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
