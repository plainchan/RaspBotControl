#include "raspbot_comm.h"
#include "string.h"
#include "stm32f10x.h"

/***** ���ݷ���ṹ��  *****/
Robot_dpkg          		robot_dpkg={0};
IMU_dpkg            		imu_dpkg={0};
IMU_9Axis_dpkg 				imu_9Axis_dpkg={0};
IMU_6Axis_dpkg 				imu_6Axis_dpkg={0};
Encoder_dpkg 				encoder_dpkg={0};
Voltage_dpkg 				voltage_dpkg={0};

/***** ֡���ݷ���ṹ��  *****/
Frame_Robot_dpkg          		frame_robot_dpkg={0};
Frame_IMU_dpkg            		frame_imu_dpkg={0};
Frame_IMU_9Axis_dpkg 			frame_imu_9Axis_dpkg={0};
Frame_IMU_6Axis_dpkg 			frame_imu_6Axis_dpkg={0};
Frame_Encoder_dpkg 				frame_encode_dpkg={0};
Frame_Voltage_dpkg 				frame_voltage_dpkg={0};

/***** ���ݽ���ṹ��  *****/
Stream_msgs               stream_msgs={0};

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
	if(bytesCount == 2) //���֡ͷ
	{
		
		if (stream_msgs->stream_buff[0] != Header1 || stream_msgs->stream_buff[1] != Header2)
		{
			stream_msgs->stream_buff[0] = stream_msgs->stream_buff[1];
			bytesCount = 1;
			return -1; //����֡
		}
	}
	else if (bytesCount == 3) // DPKG ����
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
		/*crc У��  Ԥ���ӿ�*/
		stream_msgs->crc = Bytes2U16Conv(&stream_msgs->stream_buff[bytesCount - 2]);
	}
	else if (bytesCount >= FRAME_INFO_SIZE + stream_msgs->len)
	{
		bytesCount = 0;
		return decode_frame(stream_msgs);
	}

  return 0; //֡δ����
}


int decode_frame(Stream_msgs *stream_msgs)
{
	uint8_t offset = 0;
	uint8_t *buff = &stream_msgs->stream_buff[FRAME_INFO_SIZE];
	while(offset < stream_msgs->len)
	{
		switch (buff[offset])
		{
		case speed_tag:
//			robot_msgs.velocity=Bytes2INT16Conv(&stream_msgs->stream_buff[offset+1]);offset+=2;
//			robot_msgs.yaw=Bytes2INT16Conv(&stream_msgs->stream_buff[offset+1]);offset+=2;
			break;
		default:++offset;
			break;
		}
	}
	return 1;
}


uint8_t bytesBuff[MAX_BUFF_SIZE]={0};
void sendFrame_Robot_dpkg(Frame_Robot_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = robot_dpkg_len;
	dpkg->crc = crc;
	int size = FRAME_INFO_SIZE+robot_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_dpkg(Frame_IMU_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = imu_dpkg_len;
	dpkg->crc = crc;
	int size = FRAME_INFO_SIZE+imu_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_9Axis_dpkg(Frame_IMU_9Axis_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = imu_9axis_dpkg_len;
	dpkg->crc = crc;
	int size = FRAME_INFO_SIZE+imu_9axis_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_6Axis_dpkg(Frame_IMU_6Axis_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = imu_6axis_dpkg_len;
	dpkg->crc = crc;
	int size = FRAME_INFO_SIZE+imu_6axis_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_Encoder_dpkg(Frame_Encoder_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = encoder_dpkg_len;
	dpkg->crc = crc;
	dpkg->encoder_dpkg.data_tag = encoder_tag;
	dpkg->encoder_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
	dpkg->encoder_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
	int size = FRAME_INFO_SIZE+encoder_dpkg_len;    //sizeof(*dpkg)
	memset(bytesBuff,0,size);
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_Voltage_dpkg(Frame_Voltage_dpkg *dpkg,Robot_msgs* robot_msgs,uint16_t crc)
{
	dpkg->header[0]=Header1;
	dpkg->header[1]=Header2;
	dpkg->len = voltage_dpkg_len;
	dpkg->crc = crc;
	int size = FRAME_INFO_SIZE+voltage_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,dpkg,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

void sendFrame_Multi_dpkg(void)
{
	Frame_Info frame_info;
	frame_info.header[0]=Header1;
	frame_info.header[1]=Header2;
	frame_info.len = voltage_dpkg_len+encoder_dpkg_len;
	frame_info.crc = 0;

	int size = FRAME_INFO_SIZE+frame_info.len;

	if(FRAME_INFO_SIZE+frame_info.len>MAX_BUFF_SIZE) return;

	
	memcpy(bytesBuff,&frame_info,FRAME_INFO_SIZE);
	memcpy(&bytesBuff[FRAME_INFO_SIZE],&voltage_dpkg,voltage_dpkg_len);
	memcpy(&bytesBuff[FRAME_INFO_SIZE+voltage_dpkg_len],&encoder_dpkg,encoder_dpkg_len);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
	
}
