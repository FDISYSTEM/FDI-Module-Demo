/*!
 *	\file		FDI_receive.h
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

#ifndef __FDI_RECEIVE_H
#define __FDI_RECEIVE_H

#define VERSIONDATA         	 "39"
#define MSG_IMU                "40"
#define MSG_AHRS               "41"
#define MSG_INSGPS             "42"
#define MSG_SYS_STATE          "50"
#define MSG_UNIX_TIME          "51"
#define MSG_FORMAT_TIME        "52"
#define MSG_STATUS             "53"
#define MSG_POS_STD_DEV        "54"
#define MSG_VEL_STD_DEV        "55"
#define MSG_EULER_ORIEN_STD_DEV "56"
#define MSG_QUAT_ORIEN_STD_DEV "57"
#define MSG_RAW_SENSORS        "58"
#define MSG_RAW_GNSS           "59"
#define MSG_SATELLITE          "5a"
#define MSG_DETAILED_SATELLITE "5b"
#define MSG_GEODETIC_POS       "5c"
#define MSG_ECEF_POS           "5d"
#define MSG_UTM_POS            "5e"
#define MSG_NED_VEL            "5f"
#define MSG_BODY_VEL           "60"
#define MSG_ACCELERATION       "61"
#define MSG_BODY_ACCELERATION  "62"
#define MSG_EULER_ORIEN        "63"
#define MSG_QUAT_ORIEN         "64"
#define MSG_DCM_ORIEN          "65"
#define MSG_ANGULAR_VEL        "66"
#define MSG_ANGULAR_ACC        "67"
#define MSG_RUNNING_TIME       "6d"
#define MSG_LOCAL_MAG_FIELD    "6e"
#define MSG_ODOMETER_STATE     "6f"
#define MSG_GEOID_HEIGHT       "72"
#define MSG_RTCM_CORRECTIONS   "73"
#define MSG_WIND               "75"
#define MSG_HEAVE              "76"
#define MSG_RAW_SATELLITE      "77"
#define MSG_GNSS_DUAL_ANT      "78"
#define MSG_GIMBAL_STATE       "7a"
#define MSG_AUTOMOTIVE         "7b"
#define MSG_PACKET_TIMER_PERIOD "7c"
#define MSG_PACKETS_PERIOD     "7d"
#define MSG_INSTALL_ALIGN      "80"
#define MSG_FILTER_OPTIONS     "81"
#define MSG_GPIO_CONFIG        "82"
#define MSG_MAG_CALI_VALUES    "83"
#define MSG_MAG_CALI_CONFIG    "84"
#define MSG_MAG_CALI_STATUS    "85"
#define MSG_ODOMETER_CONFIG    "86"
#define MSG_SET_ZERO_ORIENT_ALIGN "87"
#define MSG_REF_POINT_OFFSET   "88"
#define MSG_USER_DATA          "8a"
#define MSG_BAUD_RATES         "a0"
#define MSG_SENSOR_RANGES      "a1"
#define MSG_GPIO_OUTPUT_CONFIG "a2"
#define MSG_GPIO_INPUT_CONFIG  "a3"
#define MSG_DUAL_ANT           "a4"

void fdiDecodeBuffer(int PACKET_ID,void* buf);

#endif
