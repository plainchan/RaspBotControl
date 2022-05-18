#include "raspbot_comm.h"
#include "string.h"
#include "sys.h"

Bytes2U16   B2U16;
Bytes2Float B2F;
Bytes2INT16 B2INT16;

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

		B2U16.bytes[0] = stream_msgs->stream_buff[bytesCount - 2];
		B2U16.bytes[1] = stream_msgs->stream_buff[bytesCount - 1];
		
		/*crc 校验  预留接口*/
		stream_msgs->crc = B2U16.number;
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
		B2INT16.bytes[0] = buff[++offset];
    B2INT16.bytes[1] = buff[++offset];
		stream_msgs->speed_msgs.velocity=B2INT16.number;
		B2INT16.bytes[0] = buff[++offset];
    B2INT16.bytes[1] = buff[++offset];
		stream_msgs->speed_msgs.yaw=B2INT16.number;
		break;
	default:++offset;
		break;
	}
	return 1;
}


uint8_t bytesBuff[MAX_BUFF_SIZE]={0};
void sendFrameData(Frame_Robot_msg *msg)
{
	int size = sizeof(*msg);
	memcpy(bytesBuff,msg,size);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

