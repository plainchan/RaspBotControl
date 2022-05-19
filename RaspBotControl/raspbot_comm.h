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

// #define imu_mag

/**
 *  串口传输缓冲区最大大小
 */
#define MAX_RxBUFF_SIZE   0x01FE                   //510

/**
 *  帧缓冲区最大大小
 */
#define MAX_BUFF_SIZE      0xFF                             //255
#define FRAME_INFO_SIZE    0x05                             //5    非数据域
#define MAX_DPKG_SIZE    (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //250

#define Header1                  (uint8_t)0xFE
#define Header2                  (uint8_t)0xEF


#define encoder_dpkg_len    (uint8_t)0x05
#ifdef   imu_mag
#define robot_dpkg_len      (uint8_t)0x36		 //54
#define imu_dpkg_len        (uint8_t)0x31    //49
#else
#define robot_dpkg_len      (uint8_t)0x2A    //42
#define imu_dpkg_len        (uint8_t)0x25    //37
#endif
#define imu_6axis_dpkg_len  (uint8_t)0x19    //25
#define imu_9axis_dpkg_len  (uint8_t)0x25    //37
#define voltage_dpkg_len    (uint8_t)0x02

#define robot_tag                0xA0
#define speed_tag                0xB0
#define encoder_tag              0xC0
#define imu_tag                  0xD0
#define imu_6axis_tag            0xD6
#define imu_9axis_tag            0xD9
#define voltage_tag              0xE0

typedef union 
{
  uint8_t bytes[4];
	float   number;
}Bytes2Float;
typedef union 
{
	uint8_t    bytes[2];
	uint16_t   number;
    
}Bytes2U16;
typedef union 
{
	uint8_t    bytes[2];
  int16_t    number;
}Bytes2INT16;
typedef union 
{
	uint8_t    bytes[4];
	uint32_t   number;
}Bytes2U32;

float Bytes2FloatConv(const uint8_t* buff);
int16_t Bytes2INT16Conv(const uint8_t* buff);
uint16_t Bytes2U16Conv(const uint8_t* buff);


/***********数据封包结构体定义***********/

/**
 * @brief 帧头+dpkg总长度+crc
 */
__packed typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint16_t     crc;
}Frame_Info;

/**
 * @brief 电压
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
    uint16_t          crc;
    Voltage_dpkg      voltage;  //size = 5+2 
}Frame_Voltage_dpkg;

/**
 * @brief 编码器
 */
__packed typedef struct Encoder_Pulse_msg
{
	uint8_t    data_tag;
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;     //size = 5+5 
}Encoder_dpkg;
__packed typedef struct 
{
    uint8_t           header[2];
    uint8_t           len;
    uint16_t          crc;
    Encoder_dpkg      encoder_dpkg;  //size = 5+5 
}Frame_Encoder_dpkg;

/**
 * @brief 
 */
__packed typedef struct IMU_Acc_Gyr_msg
{
	uint8_t    data_tag;
    float      acc[3];
    float      gyr[3];
}IMU_6Axis_dpkg;                       //size = 25 
__packed typedef struct 
{
    uint8_t           header[2];
    uint8_t           len;
    uint16_t          crc;
    IMU_6Axis_dpkg    imu_6axis_dpkg;  //size = 25+5 

}Frame_IMU_6Axis_dpkg;

__packed typedef struct IMU_Acc_Gyr_Mag_msg
{
	uint8_t     data_tag;
    float       acc[3];
    float       gyr[3];
    float       mag[3];
}IMU_9Axis_dpkg;                       //size = 37 
__packed typedef struct 
{
    uint8_t           header[2];
    uint8_t           len;
    uint16_t          crc;
    IMU_9Axis_dpkg    imu_9axis_dpkg;  //size = 37+5 

}Frame_IMU_9Axis_dpkg;


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
    uint16_t     crc;
    IMU_dpkg   	 imu_dpkg;  //size = 37+5 or 49+5 

}Frame_IMU_dpkg;


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
    uint16_t     crc;
    Robot_dpkg   robot_dpkg;  //size = 54+5 or 54+5 

}Frame_Robot_dpkg;


/***** 数据封包结构体  *****/
extern Robot_dpkg          		        robot_dpkg;
extern IMU_dpkg            		        imu_dpkg;
extern IMU_9Axis_dpkg 				    imu_9Axis_dpkg;
extern IMU_6Axis_dpkg 				    imu_6Axis_dpkg;
extern Encoder_dpkg 				    encode_dpkg;
extern Voltage_dpkg 				    voltage_dpkg;

/***** 数据封包结构体  *****/
extern Frame_Robot_dpkg          		frame_robot_dpkg;
extern Frame_IMU_dpkg            		frame_imu_dpkg;
extern Frame_IMU_9Axis_dpkg 			frame_imu_9Axis_dpkg;
extern Frame_IMU_6Axis_dpkg 			frame_imu_6Axis_dpkg;
extern Frame_Encoder_dpkg 				frame_encode_dpkg;
extern Frame_Voltage_dpkg 				frame_voltage_dpkg;

/**
	* @brief  特定数据包封装帧结构体发送
 */
void sendFrame_Multi_dpkg(void);
void sendFrame_Robot_dpkg(Frame_Robot_dpkg *dpkg,uint16_t crc);
void sendFrame_IMU_dpkg(Frame_IMU_dpkg *dpkg,uint16_t crc);
void sendFrame_IMU_9Axis_dpkg(Frame_IMU_9Axis_dpkg *dpkg,uint16_t crc);
void sendFrame_IMU_6Axis_dpkg(Frame_IMU_6Axis_dpkg *dpkg,uint16_t crc);
void sendFrame_Encoder_dpkg(Frame_Encoder_dpkg *dpkg,uint16_t crc);
void sendFrame_Voltage_dpkg(Frame_Voltage_dpkg *dpkg,uint16_t crc);
//-----------------------------------------



/***********数据解包结构体定义***********/

typedef struct
{
  float        velocity;
  float        yaw;
}Speed_msgs;

typedef struct
{

	uint8_t      len;
    uint16_t     crc;
	uint8_t      stream_buff[MAX_BUFF_SIZE];
	Speed_msgs   speed_msgs;
}Stream_msgs;

//-----------------------------------------


/**
 * @brief    将数据流缓冲到Buff,数据流中可能只包含半帧或者多帧数据
 * 
 * @param[out] stream_msgs 
 * @param[in]  buff 
 * @param[in]  size 
 * @return int 
 *         1   解析成功
 *         -1  帧头错误
 *         0   校验错误
 *         -2  数据域长度出错
 */
int parse_stream(Stream_msgs *stream_msgs,uint8_t buff);


/**
 * @brief 对一帧数据解码
 * 
 * @param[out] stream_msgs 
 * @param[out] buff
 * @return int
 *          1   解析成功
 *          -1  帧错误
 */
int decode_frame(Stream_msgs *stream_msgs);
#endif
