#ifndef __RASPBOT_COMM_H__
#define __RASPBOT_COMM_H__

/****************************************************************************************
*                             Serial Protocol                                           *
*                                                                                       *
*  ,------+------+-------+- - - - - -+- - - - - -+- - - - -+                            *
*  | SOF  |  LEN | CRC16 |    DPKG   |    DPKG   |   ....  |                            *
*  |  2   |   1  |   2   |    ...    |    ...    |   ...   |                            *
*  '----- +------+-------+- - - - - -+- - - - - -+- - - - -+                            *
*  SOF  .........  start of frame, 2 bytes                                              *
*  LEN  .........  number of data package in the frame                                  *
*  CRC  .........  cyclic redundancy check                                              *
*  DPKG .........  data package,include DATA_TAG,DATA                                   *
*                                                                                       *
*  ,------+--------------+---------+---------+--------+                                 *
*  | Type |  elem        |  size   |  offset |  byte  |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      | frame  head1 | 1 BYTE  |    0    |    1   |                                 *
*  | SOF  ,--------------+---------+---------+--------+                                 *
*  |      | frame  head2 | 1 BYTE  |    1    |    2   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | LEN  |              | 1 BYTE  |    2    |    3   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | CRC  |              | 2 BYTES |    3    |    5   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |    5    |    6   |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | n BYTES |   5+n   |   6+n  |                                 *
*  ,---------------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |   6+n   |   7+n  |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | m BYTES |  6+m+n  |  7+m+n |                                 *
*  ----------------------+---------+---------+--------+                                 *
*                                                                                       *
*  Endian: Little-Endian                                                                *
*****************************************************************************************/

#include "stdint.h"
#include "raspbot_control.h"

//#define imu_mag

/**
 *  ??????????????????????
 */
#define MAX_RxBUFF_SIZE   0x01FE                   //510

/*
 *
 *
 */
#define FRAME_HEAD_CRC_BYTES        1     
#define FRAME_DPKG_CRC_BYTES        2
#define FRAME_DPKG_LEN_BYTES        1     
#define FRAME_INFO_SIZE             (2+FRAME_DPKG_LEN_BYTES+FRAME_HEAD_CRC_BYTES)
#define FRAME_CALCU_CRC_BYTES       (2 + FRAME_DPKG_LEN_BYTES)

/**
 *  ????????????????
 */
#define MAX_BUFF_SIZE      0xFF                             //255
#define MAX_DPKG_SIZE    (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //250


/**
 * @brief ??Byte??????
 * 
 */
#define FRAME_HEAD_OFFSET               2
#define FRAME_DPKG_LEN_OFFSET           (FRAME_HEAD_OFFSET + FRAME_DPKG_LEN_BYTES)
#define FRAME_HEAD_CRC_OFFSET           FRAME_INFO_SIZE


#define Header1                  (uint8_t)0xFE
#define Header2                  (uint8_t)0xEF


#define encoder_dpkg_len     (uint8_t)0x05
#ifdef   imu_mag
#define robot_dpkg_len       (uint8_t)54		 
#define imu_dpkg_len         (uint8_t)49    
#define imu_sensor_dpkg_len  (uint8_t)37  
#define imu_raw_dpkg_len     (uint8_t)25
#else
#define robot_dpkg_len       (uint8_t)42    
#define imu_dpkg_len         (uint8_t)37   
#define imu_sensor_dpkg_len  (uint8_t)25    
#define imu_raw_dpkg_len     (uint8_t)19
#endif

#define voltage_dpkg_len    (uint8_t)0x02


#define robot_tag                0xA0
#define speed_tag                0xB0
#define encoder_tag              0xC0
#define imu_tag                  0xD0
#define imu_sensor_tag           0xD1
#define imu_raw_tag              0xD2
#define voltage_tag              0xE0
#define pid_tag                  0x70

extern volatile int  receiveFlag;

extern short imu_ready_flag;

/* uinon ??????bytes?????????? */
typedef union 
{
  uint8_t bytes[4];
	float   number;
}Bytes_Float;
typedef union 
{
	uint8_t    bytes[2];
	uint16_t   number;
    
}Bytes_U16;
typedef union 
{
	uint8_t    bytes[2];
  int16_t    number;
}Bytes_INT16;
typedef union 
{
	uint8_t    bytes[4];
	uint32_t   number;
}Bytes_U32;
//-----------------------------------------

/* bytes??????????????  */
float Bytes2FloatConv(const uint8_t* buff);
int16_t Bytes2INT16Conv(const uint8_t* buff);
uint16_t Bytes2U16Conv(const uint8_t* buff);
//-----------------------------------------

/***********??????????????????***********/

/**
 * @brief ????+dpkg??????+crc
 */
__packed typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_header;
}Frame_Info;

/**
 * @brief ????
 */
__packed typedef struct
{
	uint8_t    data_tag;
  uint8_t    voltage;     //size = 2 
}Voltage_dpkg;
__packed typedef struct 
{
    uint8_t           header[2];
    uint8_t           len;
    uint8_t          	crc_header;
    Voltage_dpkg      voltage_dpkg;  //size = 5+2 
		uint16_t          crc_dpkg;
}Frame_Voltage_dpkg;

/**
 * @brief ??????
 */
__packed typedef struct Encoder_Pulse_msg
{
	uint8_t    data_tag;
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;     //size = 5+5 
}Encoder_dpkg;

__packed typedef struct dfdfd
{
    uint8_t           header[2];
    uint8_t           len;
    uint8_t           crc_header;
    Encoder_dpkg      encoder_dpkg;  //size = 5+5 
		uint16_t          crc_dpkg;
}Frame_Encoder_dpkg;

/**
 * @brief 
 */
__packed typedef struct
{
	uint8_t    data_tag;
    float      acc[3];
    float      gyr[3];
#ifdef     imu_mag
    float      mag[3];
#endif
}IMU_Sensor_dpkg;
__packed typedef struct 
{
    uint8_t            header[2];
    uint8_t            len;
    uint8_t            crc_header;
    IMU_Sensor_dpkg    imu_sensor_dpkg;  //size = 37+5 
		uint16_t           crc_dpkg;

}Frame_IMU_Sensor_dpkg;


/**
 * @brief IMU
 */
__packed typedef struct IMU_Acc_Gyr_Elu_msg
{
	uint8_t    data_tag;
    float      acc[3];
    float      gyr[3];
#ifdef     imu_mag
    float      mag[3];
#endif
    float      elu[3];
}IMU_dpkg;                  //size = 37 or 49     
__packed typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_header;
    IMU_dpkg   	 imu_dpkg;  //size = 37+5 or 49+5 
		uint16_t     crc_dpkg;

}Frame_IMU_dpkg;

/**
 * @brief IMU??????????????
 */
__packed typedef struct IMU_Acc_Gyr_Mag_Register_msg
{
	  uint8_t    data_tag;
    int16_t      accRaw[3];
    int16_t      gyrRaw[3];
#ifdef     imu_mag
    int16_t      magRaw[3];
#endif
    int16_t      eluRaw[3];
}IMU_Raw_dpkg;                  //size = 37 or 49     
__packed typedef struct 
{
    uint8_t      		header[2];
    uint8_t      		len;
    uint8_t      		crc_header;
    IMU_Raw_dpkg   	imu_raw_dpkg;  //size = 37+5 or 49+5 
		uint16_t     	  crc_dpkg;
}Frame_IMU_Raw_dpkg;

/**
 * @brief status of robot 
 */
__packed typedef struct 
{
    uint8_t    data_tag;
    uint8_t    voltage;                 //real voltage = voltage/10
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;  
    float      acc[3];
    float      gyr[3];
#ifdef     imu_mag
    float      mag[3];
#endif
    float      elu[3];
}Robot_dpkg;                    //size = 42 or 54
__packed typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_header;
    Robot_dpkg   robot_dpkg;  //size = 42+5 or 54+5 
		uint16_t          crc_dpkg;

}Frame_Robot_dpkg;
//-----------------------------------------


/***********??????????????????***********/

typedef struct
{

	uint8_t      len;
  uint16_t     crc;
	uint8_t      stream_buff[MAX_BUFF_SIZE];
}Stream_msgs;

extern Stream_msgs               stream_msgs;
//-----------------------------------------



/* ?????????????????????????? */

/**
 * @brief
 * @param[in] 
 * @note  ??????115200 ??????????????????????????=4.08ms(None mag)5.11ms(with mag)
 */
void sendFrame_Robot_dpkg(const Robot_msgs* robot_msgs);

/**
 * @brief
 * @param[in] 
 * @note   ??????115200 ??????????????????????????=3.6ms(None mag)4.6ms(with mag)
 */
void sendFrame_IMU_dpkg(const Robot_msgs* robot_msgs);

/**
 * @brief
 * @param[in] 
 * @note  ??????115200 ??????????????????????????=2.6ms(None mag)3.6ms(with mag)
 */
void sendFrame_IMU_Sensor_dpkg(const Robot_msgs* robot_msgs);

/**
 * @brief
 * @param[in] 
 * @note  
 */
void sendFrame_IMU_Raw_dpkg(const IMU_Raw_msg* imu_raw_msg);

/**
 * @brief
 * @param[in] 
 * @note   ??????115200 ??????????????????????????=0.88ms   
 */
void sendFrame_Encoder_dpkg(const Robot_msgs* robot_msgs);

/**
 * @brief
 * @param[in] 
 * @note   ??????115200 ??????????????????????????=0.61ms   
 */
void sendFrame_Voltage_dpkg(const Robot_msgs* robot_msgs);

/**
 * @brief
 * @param[in] 
 * @note  
 */
void sendFrame_Multi_dpkg(const Robot_msgs* robot_msgs);
//-----------------------------------------



/* ?????????? */

/**
 * @brief    ??????????????Buff,??????????????????????????????????
 * 
 * @param[out] stream_msgs 
 * @param[in]  buff 
 * @param[in]  size 
 * @return int 
 *         1   ????????
 *         -1  ????????
 *         0   ????????
 *         -2  ??????????????
 */
int parse_stream(Stream_msgs *stream_msgs,uint8_t buff);


/**
 * @brief ??????????????
 * 
 * @param[out] stream_msgs 
 * @param[out] buff
 * @return int
 *          1   ????????
 *          -1  ??????
 */
int decode_frame(Stream_msgs *stream_msgs);
//-----------------------------------------




#endif


