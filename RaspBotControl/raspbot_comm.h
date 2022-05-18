#ifndef __RASPBOT_COMM_H__
#define __RASPBOT_COMM_H__

#include "stdint.h"

#define MAX_RxBUFF_SIZE   0x01FE                   //510

/**
 *  帧缓冲区最大大小
 * 
 */
#define MAX_BUFF_SIZE      0xFF                             //255
#define FRAME_INFO_SIZE    0x05                             //5    非数据域
#define MAX_DPKG_SIZE    (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //250

#define Header1                  (uint8_t)0xFE
#define Header2                  (uint8_t)0xEF

#define robot_tag                0xA0
#define speed_tag                0xB0

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

/**
	*  发送帧数据结构体定义
	*/
__packed typedef struct 
{
    int8_t     data_tag;
    uint8_t    voltage;                 //real voltage = voltage/10
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;  
    float      acc[3];
    float      gyr[3];
    float      mag[3];
    float      elu[3];
}Robot_msgs;    //size = 54

__packed typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint16_t     crc;
    Robot_msgs   robot_msgs;  //size = 54+5 

}Frame_Robot_msg;


void sendFrameData(Frame_Robot_msg *msg);
////////////////////////////////////////////////////////////////////////////////////////////////////

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
