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
#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#ifdef WIN32
    #include <string.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <windows.h>
    typedef     unsigned int    UINT32;
    typedef     unsigned short  UINT16;
    typedef  	unsigned char	UINT8;
    typedef  	unsigned long long UINT64;

    typedef     signed int    	SINT32;
    typedef     signed short  	SINT16;
    typedef  	signed char		SINT8;
    typedef  	signed long long SINT64;

    typedef     float           FLOAT32;
    typedef     double 			FLOAT64;
    typedef     bool			SBOOL;
    typedef     char			SCHAR;	
#elif defined LINUX//linux
    typedef     unsigned int    UINT32;
    typedef     unsigned short  UINT16;
    typedef  	unsigned char	UINT8;
    typedef  	unsigned long long UINT64;

    typedef     signed int    	SINT32;
    typedef     signed short  	SINT16;
    typedef  	signed char		SINT8;
    typedef  	signed long long SINT64;
    typedef     long LONG;

    typedef     float           FLOAT32;
    typedef     double 			FLOAT64;
    typedef     bool			SBOOL;
    typedef     char			SCHAR;	
#else //other os
    typedef     unsigned int    UINT32;
    typedef     unsigned short  UINT16;
    typedef  	unsigned char	UINT8;
    typedef  	unsigned long long UINT64;

    typedef     signed int    	SINT32;
    typedef     signed short  	SINT16;
    typedef  	signed char		SINT8;
    typedef  	signed long long SINT64;
    typedef     long LONG;

    typedef     float           FLOAT32;
    typedef     double 			FLOAT64;
    typedef     bool			SBOOL;
    typedef     char			SCHAR;	
#endif

#ifndef    TRUE
    #define    TRUE	1
#endif

#ifndef    FALSE
    #define	   FALSE 0
#endif

#ifndef    NULL
    #define	   NULL 0
#endif




typedef enum tagTOF_MODE
{
	//˫Ƶ
	TOF_MODE_STERO_5FPS = 0x00000001,
	TOF_MODE_STERO_10FPS = 0x00000002,
	TOF_MODE_STERO_15FPS = 0x00000004,
	TOF_MODE_STERO_30FPS = 0x00000008,
	TOF_MODE_STERO_45FPS = 0x00000010,
	TOF_MODE_STERO_60FPS = 0x00000020,

	//��Ƶ
	TOF_MODE_MONO_5FPS = 0x00000040,
	TOF_MODE_MONO_10FPS = 0x00000080,
	TOF_MODE_MONO_15FPS = 0x00000100,
	TOF_MODE_MONO_30FPS = 0x00000200,
	TOF_MODE_MONO_45FPS = 0x00000400,
	TOF_MODE_MONO_60FPS = 0x00000800,

	//HDRZ
	TOF_MODE_HDRZ_5FPS = 0x00001000,
	TOF_MODE_HDRZ_10FPS = 0x00002000,
	TOF_MODE_HDRZ_15FPS = 0x00004000,
	TOF_MODE_HDRZ_30FPS = 0x00008000,
	TOF_MODE_HDRZ_45FPS = 0x00010000,
	TOF_MODE_HDRZ_60FPS = 0x00020000,

	//һ��Ƶ
	TOF_MODE_5FPS = 0x00040000,
	TOF_MODE_10FPS = 0x00080000,
	TOF_MODE_20FPS = 0x00100000,
	TOF_MODE_30FPS = 0x00200000,
	TOF_MODE_45FPS = 0x00400000,
	TOF_MODE_60FPS = 0x00800000,

	//���ƴ���
	TOF_MODE_ADI_1M5 = 0x01000000,
	TOF_MODE_ADI_5M = 0x02000000,


}TOF_MODE;


typedef enum tagTOF_FILTER
{
	TOF_FILTER_RemoveFlyingPixel = 0x00000001,
	TOF_FILTER_AdaptiveNoiseFilter = 0x00000002,
	TOF_FILTER_InterFrameFilter = 0x00000004,
	TOF_FILTER_PointCloudFilter = 0x00000008,
	TOF_FILTER_StraylightFilter = 0x00000010,
	TOF_FILTER_CalcIntensities = 0x00000020,
	TOF_FILTER_MPIFlagAverage = 0x00000040,
	TOF_FILTER_MPIFlagAmplitude = 0x00000080,
	TOF_FILTER_MPIFlagDistance = 0x00000100,
	TOF_FILTER_ValidateImage = 0x00000200,
	TOF_FILTER_SparsePointCloud = 0x00000400,
	TOF_FILTER_Average = 0x00000800,
	TOF_FILTER_Median = 0x00001000,
	TOF_FILTER_Confidence = 0x00002000,
	TOF_FILTER_MPIFilter = 0x00004000,

}TOF_FILTER;



typedef enum tagEXP_MODE
{
	EXP_MODE_MANUAL = 0x00000001,//�ֶ��ع�
	EXP_MODE_AUTO = 0x00000002,//�Զ��ع�(AE)
}EXP_MODE;



//�Ҷ����ݸ�ʽ
typedef enum tagGRAY_FORMAT
{
	GRAY_FORMAT_UINT8 = 0,//8λ����
	GRAY_FORMAT_UINT16,//�޷���16λ����
	GRAY_FORMAT_FLOAT,//����������
	GRAY_FORMAT_BGRD,//ÿ����32λ�� ��B/G/R/D˳����

}GRAY_FORMAT;


typedef struct tagPointData
{
	FLOAT32 x;
	FLOAT32 y;
	FLOAT32 z;
}PointData;


//TOF Expouse
typedef struct tagTofExpouse
{
	UINT32  	nCurrent;//��ǰֵ���ɶ�д
	UINT32  	nDefault;//Ĭ��ֵ��ֻ��
	UINT32  	nStep;//����ֵ��ֻ��
	UINT32  	nMax;//���ֵ��ֻ��
	UINT32  	nMin;//��Сֵ��ֻ��
}TofExpouse;

//TOFģ���ڲκͻ���
typedef struct tagTofModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}TofModuleLensParameter;

//TOFģ��궨����
typedef struct tagTofCalibData
{
	UINT8* pData;//ָ��궨����
	UINT32 nDataLen;//pData�ڱ궨���ݳ���
}TofCalibData;

typedef struct tagTofRawData
{
	//RAW����
	UINT8* pRaw;//һ֡RAW����
	UINT32 nRawLen;//RAW���ݳ��ȣ��ֽ�����

	//RAW�����������Բ���
	FLOAT32 fTemperature;//��RAW����ʱģ���¶ȣ�ע�⣺�����ͺ�ģ�鲻��Ҫ���ֶΡ�����ģ��RAW�����Դ������ݣ���ô��������0ֵ��

}TofRawData;





#endif //__TYPEDEF_H__


