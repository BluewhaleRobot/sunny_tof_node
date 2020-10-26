/*----------------------------------------------------------------------------------------------
*
* This file is Sunny Optical's property. It contains Sunny Optical's trade secret, proprietary and 		
* confidential information. 
* 
* The information and code contained in this file is only for authorized Sunny Optical employees 
* to design, create, modify, or review.
* 
* DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER AUTHORIZATION.
* 
* If you are not an intended recipient of this file, you must not copy, distribute, modify, 
* or take any action in reliance on it. 
* 
* If you have received this file in error, please immediately notify Sunny Optical and 
* permanently delete the original and any copy of any file and any printout thereof.
*
*-------------------------------------------------------------------------------------------------*/
/*
* 
* 
* 
* Author:
* Carlson Lee (fli@sunnyoptical.com)
* 
* History:
*		
*
*
*/

#ifndef __TOF_DEVICE_H__
#define __TOF_DEVICE_H__

#include "typedef.h"
#include "tof_error.h"

#ifdef WIN32
    #ifdef TOF_DEVICE_SDK_EXPORT
        #define TOFDDLL __declspec(dllexport)
    #else
        #define TOFDDLL __declspec(dllimport)
    #endif
#else
    #define TOFDDLL 
#endif

typedef enum tagTOF_DEV_TYPE
{
	TOF_DEV_CLEANER01A,//Cleaner01A
	TOF_DEV_CLEANER01A_NET,//Cleaner01A（网络版）
	TOF_DEV_CLEANER01B,//Cleaner01B
	TOF_DEV_CLEANER02A,//Cleaner02A
	TOF_DEV_CLEANER02A_NET,//Cleaner02A（网络版）
	TOF_DEV_MARS01A,//Mars01A
	TOF_DEV_MARS01B,//Mars01B
	TOF_DEV_MARS01C,//Mars01C
	TOF_DEV_MARS01D,//Mars01D
	TOF_DEV_MARS04,//Mars04
	TOF_DEV_MARS04A,//Mars04A
	TOF_DEV_MARS04B,//Mars04B
	TOF_DEV_MARS05,//Mars05
	TOF_DEV_MARS05A,//Mars05A
	TOF_DEV_MARS05B,//Mars05B
	TOF_DEV_USBTOF_HI,//UsbTof-Hi
	TOF_DEV_DREAM,//DREAM

}TOF_DEV_TYPE;



typedef struct tagRgbDData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbDData;


typedef struct tagTofFrameData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	PointData *pPointData;//点云数据

	GRAY_FORMAT grayFormat;//pGrayData内数据格式
	void   *pGrayData;//灰度数据

	RgbDData* pRgbD;//RgbD数据

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}TofFrameData;


typedef enum tagCOLOR_FORMAT
{
	//MJPG格式
	COLOR_FORMAT_MJPG = 0,

	//H264格式
	COLOR_FORMAT_H264,

	//YUV格式
	COLOR_FORMAT_YUV422,
	COLOR_FORMAT_YUV420,
	COLOR_FORMAT_YUV420SP,
	COLOR_FORMAT_YV12,
	COLOR_FORMAT_YUYV,

	//RGB格式
	COLOR_FORMAT_BGR, //RGB24（每个像素占3个字节，按照B、G、R的顺序存放）
	COLOR_FORMAT_RGB, //RGB24（每个像素占3个字节，按照R、G、B的顺序存放）
	COLOR_FORMAT_BGRA, //RGB32（每个像素占4个字节，按照B、G、R、A的顺序存放）
	COLOR_FORMAT_RGBA, //RGB32（每个像素占4个字节，按照R、G、B、A的顺序存放）

}COLOR_FORMAT;


typedef enum tagRgbVideoControlProperty
{
	RgbVideoControl_Exposure = 0x00000001,//RGB模组的曝光属性
	RgbVideoControl_Gain = 0x00000002,//RGB模组的增益属性

}RgbVideoControlProperty;

typedef enum tagRgbVideoControlFlags
{
	RgbVideoControlFlags_Auto = 0x00000001,//自动
	RgbVideoControlFlags_Manual = 0x00000002,//手动

}RgbVideoControlFlags;


typedef struct tagRgbVideoControl
{
	LONG  	lDefault;//默认值
	LONG  	lStep;//步进值
	LONG  	lMax;//最大值
	LONG  	lMin;//最小值
	LONG    lCapsFlags;//支持的值，是RgbVideoControlFlags的一种或多种组合

	LONG  	lCurrent;//当前值
	RgbVideoControlFlags    lFlags;//当前Flag值

}RgbVideoControl;


typedef struct tagRgbData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbData;


typedef struct tagRgbFrameData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	COLOR_FORMAT formatType;//指明pFrameData内数据帧的格式
	UINT32  nFrameLen;
	UINT8*  pFrameData;

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}RgbFrameData;

typedef struct tagImuFrameData
{
	UINT64 timeStamp;

	FLOAT32 accelData_x;
	FLOAT32 accelData_y;
	FLOAT32 accelData_z;

	FLOAT32 gyrData_x;
	FLOAT32 gyrData_y;
	FLOAT32 gyrData_z;

	FLOAT32 magData_x;
	FLOAT32 magData_y;
	FLOAT32 magData_z;

}ImuFrameData;


typedef struct tagTofDevInitParam
{
	SCHAR szDepthCalcCfgFileDir[200];//深度计算所需配置文件的目录，如home/user/temp
	UINT8 nLogLevel;//日志打印级别

}TofDevInitParam;



typedef struct tagTofDeviceDescriptor
{
	void*  	hDevice;
	void*  	hDriver;

}TofDeviceDescriptor;

typedef struct tagTofDeviceInfo
{
	//BASIC information
	TOF_DEV_TYPE devType;//用于区是哪款设备
	SCHAR szDevName[32];
	SCHAR szDevId[64];//设备/模块的序列号（标识设备唯一性）
	SCHAR szFirmwareVersion[32];//固件版本信息
	
	//TOF
	UINT32 supportedTOFMode;//TOF_MODE的组合
	UINT32 tofResWidth;
	UINT32 tofResHeight;
	GRAY_FORMAT grayFormat;//灰度数据格式
	
	//TOF Expouse
	UINT32 supportedTofExpMode;//EXP_MODE的组合

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER的组合

	//TOF HDRZ
	SBOOL bTofHDRZSupported;
	UINT8 byRes1[3];//字节对齐，预留

	//RGB
	SBOOL bRgbSupported;
	UINT8 byRes2[3];//字节对齐，预留
	COLOR_FORMAT rgbColorFormat;
	UINT32 rgbResWidth;
	UINT32 rgbResHeight;
	UINT32 supportedRgbProperty;// RgbVideoControlProperty的组合

	//RGBD
	SBOOL bRgbDSupported;
	UINT8 byRes3[3];//字节对齐，预留

	//IMU
	SBOOL bImuSupported;
	UINT8 byRes4[3];//字节对齐，预留

	//

}TofDeviceInfo;

typedef struct tagTofDeviceParam
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceParam;

typedef struct tagTofDeviceTemperature
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceTemperature;



typedef enum tagTOF_DEV_PARAM_TYPE
{
	TOF_DEV_PARAM_Temperature = 0,//温度信息
	TOF_DEV_PARAM_TofLensParameter,//TOF模组内参和畸变
	TOF_DEV_PARAM_TofCalibData,//TOF模组标定数据


}TOF_DEV_PARAM_TYPE;

typedef struct tagTofDeviceParamV20
{
	TOF_DEV_PARAM_TYPE type;//输入参数，只读
	
	union
	{
		TofDeviceTemperature struTemperature;//温度信息
		TofModuleLensParameter struTofLensParameter;//TOF模组内参和畸变
		TofCalibData struTofCalibData;//TOF模组标定数据

	}uParam;
}TofDeviceParamV20;



typedef enum tagTOFDEV_STATUS
{
	TOFDEV_STATUS_UNUSED = 0,//（该值未使用，有效的设备状态从1开始）

	TOFDEV_STATUS_DEV_BROKEN = 1,//设备异常断开
	//
	TOFDEV_STATUS_READ_CALIB_DATA_SUC,//读取标定数据成功
	TOFDEV_STATUS_READ_CALIB_DATA_FAILED,//读取标定数据失败
	//
	TOFDEV_STATUS_TOF_STREAM_FAILED,//取TOF流失败

}TOFDEV_STATUS;

typedef void* HTOFD;


typedef void (*FNTofStream)(TofFrameData *tofFrameData, void* pUserData);
typedef void (*FNTofDeviceStatus)(TOFDEV_STATUS tofDevStatus, void* pUserData);
typedef void (*FNRgbStream)(RgbFrameData *rgbFrameData, void* pUserData);
typedef void (*FNImuStream)(ImuFrameData *imuFrameData, void* pUserData);

#ifdef __cplusplus
extern "C" {
#endif

TOFDDLL TOFRET TOFD_Init(TofDevInitParam* pInitParam);
TOFDDLL TOFRET TOFD_Uninit(void);

TOFDDLL SCHAR* TOFD_GetSDKVersion(void);

TOFDDLL TOFRET TOFD_SearchDevice(TofDeviceDescriptor **ppDevsDesc, UINT32* pDevNum);

TOFDDLL HTOFD  TOFD_OpenDevice(TofDeviceDescriptor *pDevDesc, FNTofDeviceStatus fnTofDevStatus, void* pUserData);
TOFDDLL TOFRET TOFD_CloseDevice(HTOFD hTofDev);

TOFDDLL TOFRET TOFD_GetDeviceInfo(HTOFD hTofDev, TofDeviceInfo *pTofDeviceInfo);
TOFDDLL TOFRET TOFD_GetDeviceParam(HTOFD hTofDev, TofDeviceParam *pTofDeviceParam);
TOFDDLL TOFRET TOFD_GetDeviceParamV20(HTOFD hTofDev, TofDeviceParamV20 *pTofDeviceParam);

TOFDDLL TOFRET TOFD_SetTofAE(HTOFD hTofDev, const SBOOL bEnable);

TOFDDLL TOFRET TOFD_SetTofExpTime(HTOFD hTofDev, const UINT32 expTime);
TOFDDLL TOFRET TOFD_GetTofExpTime(HTOFD hTofDev, TofExpouse *pExp);

TOFDDLL TOFRET TOFD_SetTofFilter(HTOFD hTofDev, const TOF_FILTER type, const SBOOL bEnable);
TOFDDLL TOFRET TOFD_GetTofFilter(HTOFD hTofDev, const TOF_FILTER type, SBOOL* pbEnable);

TOFDDLL TOFRET TOFD_SetTofHDRZ(HTOFD hTofDev, const SBOOL bEnable);

TOFDDLL TOFRET TOFD_StartTofStream(HTOFD hTofDev, const TOF_MODE tofMode, FNTofStream fnTofStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopTofStream(HTOFD hTofDev);

TOFDDLL TOFRET TOFD_GetRgbProperty(HTOFD hTofDev, const RgbVideoControlProperty Property, RgbVideoControl *pValue);
TOFDDLL TOFRET TOFD_SetRgbProperty(HTOFD hTofDev, const RgbVideoControlProperty Property, const LONG lValue, const RgbVideoControlFlags lFlag);

TOFDDLL TOFRET TOFD_StartRgbStream(HTOFD hTofDev, FNRgbStream fnRgbStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopRgbStream(HTOFD hTofDev);

TOFDDLL TOFRET TOFD_StartImuStream(HTOFD hTofDev, FNImuStream fnImuStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopImuStream(HTOFD hTofDev);

#ifdef __cplusplus
}
#endif

#endif


