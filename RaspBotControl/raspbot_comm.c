#include "raspbot_comm.h"
#include "string.h"
#include "stm32f10x.h"



/***** 数据解包结构体  *****/
Stream_msgs               stream_msgs={0};




volatile int  receiveFlag =0;

float Bytes2FloatConv(const uint8_t* buff)
{
	Bytes_Float conv;
	for(uint8_t i=0;i<4;++i)
		conv.bytes[i]=buff[i];
	return conv.number;
}

int16_t Bytes2INT16Conv(const uint8_t* buff)
{
	Bytes_INT16 conv;
	for(uint8_t i=0;i<2;++i)
		conv.bytes[i]=buff[i];
	return conv.number;
}
uint16_t Bytes2U16Conv(const uint8_t* buff)
{
	Bytes_U16 conv;
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
			stream_msgs->stream_buff[0] = stream_msgs->stream_buff[1];
			bytesCount = 1;
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
	else if (bytesCount >= FRAME_INFO_SIZE + stream_msgs->len)
	{
		bytesCount = 0;
		return decode_frame(stream_msgs);
	}

  return 0; //帧未就绪
}

int decode_frame(Stream_msgs *stream_msgs)
{
	uint8_t offset = 0;
	uint8_t *buff = &stream_msgs->stream_buff[FRAME_INFO_SIZE];
	uint8_t bytesLen = stream_msgs->len;
	while(offset < bytesLen )
	{
		switch (buff[offset])
		{
		case speed_tag:
			motor_msgs.velocity=Bytes2INT16Conv(&buff[offset+1])/1000.0;offset+=2;
			motor_msgs.yaw=Bytes2INT16Conv(&buff[offset+1])/1000.0;offset+=2;
			break;
		default:++offset;
			break;
		}
		++offset;
	}
	return 1;
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
//	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET) // 检查 ORE 标志
//  {
//		USART_ReceiveData(USART1);
//		USART_ClearFlag(USART1,USART_FLAG_ORE);
//  }
//	if(USART_GetFlagStatus(USART1,USART_FLAG_FE) == SET) // 检查 ORE 标志
//  {
//		USART_ReceiveData(USART1);
//		USART_ClearFlag(USART1,USART_FLAG_FE);
//  }
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		receiveFlag = parse_stream(&stream_msgs,USART_ReceiveData(USART1));
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  } 
} 
uint8_t bytesBuff[MAX_BUFF_SIZE]={0};
void setCRC(uint8_t *buff,uint16_t value)
{
#define crc_p1 3
#define crc_p2 4
	Bytes_U16 conv;
	conv.number = value;
	buff[crc_p1] = conv.bytes[0];
	buff[crc_p2] = conv.bytes[1];
#undef crc_p1 
#undef crc_p2 
}

void sendFrame_Robot_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Robot_dpkg  frame;
	
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = robot_dpkg_len;
	frame.crc = 0;
	
	frame.robot_dpkg.data_tag = robot_tag;
	
	frame.robot_dpkg.voltage =(uint8_t)(robot_msgs->voltage*10);
	
	frame.robot_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
	frame.robot_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
   	
	frame.robot_dpkg.acc[0] = robot_msgs->acc[0];
	frame.robot_dpkg.acc[1] = robot_msgs->acc[1];
	frame.robot_dpkg.acc[2] = robot_msgs->acc[2];
	
	frame.robot_dpkg.gyr[0] = robot_msgs->gyr[0];
	frame.robot_dpkg.gyr[1] = robot_msgs->gyr[1];
	frame.robot_dpkg.gyr[2] = robot_msgs->gyr[2];
	
#ifdef imu_mag
	frame.robot_dpkg.mag[0] = robot_msgs->mag[0];
	frame.robot_dpkg.mag[1] = robot_msgs->mag[1];
	frame.robot_dpkg.mag[2] = robot_msgs->mag[2];
#endif
	frame.robot_dpkg.elu[0] = robot_msgs->elu[0];
	frame.robot_dpkg.elu[1] = robot_msgs->elu[1];
	frame.robot_dpkg.elu[2] = robot_msgs->elu[2];
	
	
	int size = FRAME_INFO_SIZE+robot_dpkg_len;
	
	memcpy(bytesBuff,&frame,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_IMU_dpkg  frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = imu_dpkg_len;
	frame.crc = 0;
	
	frame.imu_dpkg.data_tag = imu_tag;
	
	frame.imu_dpkg.acc[0] = robot_msgs->acc[0];
	frame.imu_dpkg.acc[1] = robot_msgs->acc[1];
	frame.imu_dpkg.acc[2] = robot_msgs->acc[2];
	
	frame.imu_dpkg.gyr[0] = robot_msgs->gyr[0];
	frame.imu_dpkg.gyr[1] = robot_msgs->gyr[1];
	frame.imu_dpkg.gyr[2] = robot_msgs->gyr[2];
	
#ifdef imu_mag
	frame.imu_dpkg.mag[0] = robot_msgs->mag[0];
	frame.imu_dpkg.mag[1] = robot_msgs->mag[1];
	frame.imu_dpkg.mag[2] = robot_msgs->mag[2];
#endif
	frame.imu_dpkg.elu[0] = robot_msgs->elu[0];
	frame.imu_dpkg.elu[1] = robot_msgs->elu[1];
	frame.imu_dpkg.elu[2] = robot_msgs->elu[2];
	
	int size = FRAME_INFO_SIZE+imu_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
void sendFrame_IMU_Sensor_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_IMU_Sensor_dpkg 			frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = imu_sensor_dpkg_len;
	frame.crc = 0;
	
	frame.imu_sensor_dpkg.data_tag = imu_sensor_tag;
	frame.imu_sensor_dpkg.acc[0] = robot_msgs->acc[0];
	frame.imu_sensor_dpkg.acc[1] = robot_msgs->acc[1];
	frame.imu_sensor_dpkg.acc[2] = robot_msgs->acc[2];
	
	frame.imu_sensor_dpkg.gyr[0] = robot_msgs->gyr[0];
	frame.imu_sensor_dpkg.gyr[1] = robot_msgs->gyr[1];
	frame.imu_sensor_dpkg.gyr[2] = robot_msgs->gyr[2];
	
#ifdef imu_mag
	frame.imu_sensor_dpkg.mag[0] = robot_msgs->mag[0];
	frame.imu_sensor_dpkg.mag[1] = robot_msgs->mag[1];
	frame.imu_sensor_dpkg.mag[2] = robot_msgs->mag[2];
#endif
	
	int size = FRAME_INFO_SIZE+imu_sensor_dpkg_len;   
	memcpy(bytesBuff,&frame,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

void sendFrame_Encoder_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Encoder_dpkg 				frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = encoder_dpkg_len;
	frame.crc = 0;
	
	frame.encoder_dpkg.data_tag = encoder_tag;
	frame.encoder_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
	frame.encoder_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
	
	int size = FRAME_INFO_SIZE+encoder_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

void sendFrame_Voltage_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Voltage_dpkg 				frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = voltage_dpkg_len;
	frame.crc = 0;
	
	frame.voltage_dpkg.data_tag = voltage_tag;
	
	frame.voltage_dpkg.voltage = (uint8_t)(robot_msgs->voltage*10);
	
	int size = FRAME_INFO_SIZE+voltage_dpkg_len;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	
	}
}
void sendFrame_Multi_dpkg(const Robot_msgs* robot_msgs)
{
#define sendRobot
#define sendIMU
#define sendIMUSensor
#define sendEncoder
#define sendVoltage
	
#if !defined (sendRobot) && !defined (sendIMU) && !defined (sendIMUSensor) && \
		!defined (sendEncoder) && !defined (sendVoltage)
	return;
#endif
	
	Frame_Info frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = 0;
	frame.crc = 0;
	
#ifdef sendRobot
	Robot_dpkg          robot_dpkg;
	frame.len+=robot_dpkg_len;
#endif /*  Robot_dpkg  */
#ifdef sendIMU
	IMU_dpkg            imu_dpkg;
	frame.len+=imu_dpkg_len;
#endif /*  IMU_dpkg  */
#ifdef sendIMUSensor
	IMU_Sensor_dpkg 		imu_sensor_dpkg;
	frame.len+=imu_sensor_dpkg_len;
#endif /*  IMU_Sensor_dpkg */
#ifdef sendEncoder
	Encoder_dpkg 				encoder_dpkg;
	frame.len+=encoder_dpkg_len;
#endif /*  Encoder_dpkg   */
#ifdef sendVoltage
	Voltage_dpkg 				voltage_dpkg;
	frame.len+=voltage_dpkg_len;
#endif /*  Voltage_dpkg   */

	int size = FRAME_INFO_SIZE+frame.len;
	if(size>MAX_BUFF_SIZE) return;
	
	memcpy(bytesBuff,&frame,FRAME_INFO_SIZE);
	
#ifdef sendRobot
	robot_dpkg.data_tag = robot_tag;
	
	robot_dpkg.voltage =(uint8_t)(robot_msgs->voltage*10);
	
	robot_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
	robot_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
   	
	robot_dpkg.acc[0] = robot_msgs->acc[0];
	robot_dpkg.acc[1] = robot_msgs->acc[1];
	robot_dpkg.acc[2] = robot_msgs->acc[2];
	
	robot_dpkg.gyr[0] = robot_msgs->gyr[0];
	robot_dpkg.gyr[1] = robot_msgs->gyr[1];
	robot_dpkg.gyr[2] = robot_msgs->gyr[2];

	
	#ifdef imu_mag
	robot_dpkg.mag[0] = robot_msgs->mag[0];
	robot_dpkg.mag[1] = robot_msgs->mag[1];
	robot_dpkg.mag[2] = robot_msgs->mag[2];
	#endif
	robot_dpkg.elu[0] = robot_msgs->elu[0];
	robot_dpkg.elu[1] = robot_msgs->elu[1];
	robot_dpkg.elu[2] = robot_msgs->elu[2];
	
	memcpy(bytesBuff,&robot_dpkg,FRAME_INFO_SIZE);
	
#endif /*  Robot_dpkg  */
#ifdef sendIMU


#endif /*  IMU_dpkg  */
#ifdef sendIMUSensor


#endif /*  IMU_Sensor_dpkg */
#ifdef sendEncoder


#endif /*  Encoder_dpkg   */
#ifdef sendVoltage

#endif /*  Voltage_dpkg   */
	
	

	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
	
}
