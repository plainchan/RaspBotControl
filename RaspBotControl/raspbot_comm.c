#include "raspbot_comm.h"
#include "string.h"
#include "sys.h"


float Bytes2FloatConv(const uint8_t* buff)
{
	Bytes2Float conv;
	for(uint8_t i=0;i<4;++i)
		conv.bytes[i]=buff[i];
	return conv.number;
}
int16_t Bytes2INT16Conv(const uint8_t* buff)
{
	Bytes2INT16 conv;
	for(uint8_t i=0;i<2;++i)
		conv.bytes[i]=buff[i];
	return conv.number;
}
uint16_t Bytes2U16Conv(const uint8_t* buff)
{
	Bytes2U16 conv;
	for(uint8_t i=0;i<2;++i)
		conv.bytes[i]=buff[i];
	return conv.number;
}


int parse_stream(Stream_msgs *stream_msgs,uint8_t buff)
{
	static uint16_t bytesCount=0;
	stream_msgs->stream_buff[bytesCount++] = buff;
	if(bytesCount == 2) //检查帧头
	{
		
		if (stream_msgs->stream_buff[0] != Header1 || stream_msgs->stream_buff[1] != Header2)
		{
			bytesCount = 0;
			return -1; //错误帧
		}
	}
	else if (bytesCount == 3) // DPKG 长度
	{
		stream_msgs->len = stream_msgs->stream_buff[bytesCount - 1];
		if (stream_msgs->len > MAX_DPKG_SIZE)
		{
				bytesCount = 0;
				return -2;
		}
	}
	else if (bytesCount == 5) // crc
	{
		/*crc 校验  预留接口*/
		stream_msgs->crc = Bytes2U16Conv(&stream_msgs->stream_buff[bytesCount - 2]);
	}
	else if (bytesCount >= stream_msgs->len + FRAME_INFO_SIZE)
	{
		bytesCount = 0;
		return decode_frame(stream_msgs);
	}

  return 1; //帧未就绪
}


int decode_frame(Stream_msgs *stream_msgs)
{
	uint8_t offset = 0;
	uint8_t *buff = &stream_msgs->stream_buff[FRAME_INFO_SIZE];
	switch (buff[offset])
	{
	case speed_tag:
		stream_msgs->speed_msgs.velocity=Bytes2INT16Conv(&stream_msgs->stream_buff[offset+1]);offset+=2;
		stream_msgs->speed_msgs.yaw=Bytes2INT16Conv(&stream_msgs->stream_buff[offset+1]);offset+=2;
		break;
	default:++offset;
		break;
	}
	return 1;
}


uint8_t bytesBuff[MAX_BUFF_SIZE]={0};
void sendFrame_Robot_dpkg(Frame_Robot_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+robot_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_dpkg(Frame_IMU_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+imu_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_9Axis_dpkg(Frame_IMU_9Axis_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+imu_9axis_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_6Axis_dpkg(Frame_IMU_6Axis_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+imu_6axis_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_Encoder_dpkg(Frame_Encoder_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+encoder_dpkg_len;    //sizeof(*dpkg)
	memset(bytesBuff,0,size);
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_Voltage_dpkg(Frame_Voltage_dpkg *dpkg)
{
	int size = FRAME_INFO_SIZE+voltage_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
