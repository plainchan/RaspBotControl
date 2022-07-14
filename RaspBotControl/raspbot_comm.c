#include "raspbot_comm.h"
#include "string.h"
#include "stm32f10x.h"
#include "crc8.h"
#include "crc16.h"

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
	if(bytesCount == FRAME_HEAD_OFFSET) //检查帧头
	{
		
		if (stream_msgs->stream_buff[0] != Header1 || stream_msgs->stream_buff[1] != Header2)
		{
			stream_msgs->stream_buff[0] = stream_msgs->stream_buff[1];
			bytesCount = 1;
			return -1; //错误帧
		}
	}
	else if (bytesCount == FRAME_DPKG_LEN_OFFSET) // DPKG 长度
	{
		stream_msgs->len = stream_msgs->stream_buff[bytesCount - 1];
		if (stream_msgs->len > MAX_DPKG_SIZE)
		{
				bytesCount = 0;
				return -3;
		}
	}
	else if (bytesCount == FRAME_HEAD_CRC_OFFSET) // crc
	{
		/*crc 校验  预留接口*/
		stream_msgs->crc = stream_msgs->stream_buff[bytesCount - 1];
		if(stream_msgs->crc!=crc_8(stream_msgs->stream_buff,FRAME_CALCU_CRC_BYTES))
		{
			bytesCount = 0;
			return -2;
		}
	}        //帧信息(帧头+帧数据域长度+帧CRC)+数据域(DATA_TAGE+DATA)+数据域CRC == 一帧接收完成
	else if (bytesCount >= FRAME_INFO_SIZE + stream_msgs->len+FRAME_DPKG_CRC_BYTES)
	{
//		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //关闭串口接收中断 由于在串口中端解码，可能解码时长大于串口中断时间，造成中断叠加
		stream_msgs->crc = Bytes2U16Conv(&stream_msgs->stream_buff[bytesCount-FRAME_DPKG_CRC_BYTES]);
		bytesCount = 0;
		if(stream_msgs->crc!=crc_16(&stream_msgs->stream_buff[FRAME_INFO_SIZE],stream_msgs->len))
		{
			return -1;
		}
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
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断

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

void setBuffHeaderCRC(uint8_t *buff,uint8_t value)
{
	*buff= value;
}

void setBuffDpkgCRC(uint8_t *buff,uint16_t value)
{
	Bytes_U16 conv;
	conv.number = value;
	*buff = conv.bytes[0];
	*(buff+1) = conv.bytes[1];
}

/*
 * @brief 发送机器状态信息(电压、编码器、IMU)
 */
void sendFrame_Robot_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Robot_dpkg  frame;
	
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = robot_dpkg_len;
	frame.crc_header = 0;
	
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
	
	frame.crc_dpkg = 0;
	
	
	int size = FRAME_INFO_SIZE+robot_dpkg_len+FRAME_DPKG_CRC_BYTES;
	
	memcpy(bytesBuff,&frame,size);
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],robot_dpkg_len);
	
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+robot_dpkg_len],crc16);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}
/*
 * @brief 发送IMU 加速度、角速度、磁力计[optional]和角度
 */
void sendFrame_IMU_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_IMU_dpkg  frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = imu_dpkg_len;
	frame.crc_header = 0;
	
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
	
	frame.crc_dpkg = 0;
	
	int size = FRAME_INFO_SIZE+imu_dpkg_len+FRAME_DPKG_CRC_BYTES;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],imu_dpkg_len);
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+imu_dpkg_len],crc16);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

/*
 * @brief 发送IMU 加速度、角速度和磁力计[optional]
 */
void sendFrame_IMU_Sensor_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_IMU_Sensor_dpkg 			frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = imu_sensor_dpkg_len;
	frame.crc_header = 0;
	
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
	
	frame.crc_dpkg = 0;
	
	int size = FRAME_INFO_SIZE+imu_sensor_dpkg_len+FRAME_DPKG_CRC_BYTES;   
	memcpy(bytesBuff,&frame,size);
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],imu_sensor_dpkg_len);
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+imu_sensor_dpkg_len],crc16);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

/*
 * @brief 发送IMU 加速度、角速度、磁力计[optional]和角度
 */
void sendFrame_IMU_Raw_dpkg(const IMU_Raw_msg* imu_raw_msg)
{
	Frame_IMU_Raw_dpkg  frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = imu_raw_dpkg_len;
	frame.crc_header = 0;
	
	frame.imu_raw_dpkg.data_tag = imu_raw_tag;
	
	frame.imu_raw_dpkg.accRaw[0] = imu_raw_msg->accRaw[0];
	frame.imu_raw_dpkg.accRaw[1] = imu_raw_msg->accRaw[1];
	frame.imu_raw_dpkg.accRaw[2] = imu_raw_msg->accRaw[2];
	
	frame.imu_raw_dpkg.gyrRaw[0] = imu_raw_msg->gyrRaw[0];
	frame.imu_raw_dpkg.gyrRaw[1] = imu_raw_msg->gyrRaw[1];
	frame.imu_raw_dpkg.gyrRaw[2] = imu_raw_msg->gyrRaw[2];
	
#ifdef imu_mag
	frame.imu_raw_dpkg.magRaw[0] = imu_raw_msg->magRaw[0];
	frame.imu_raw_dpkg.magRaw[1] = imu_raw_msg->magRaw[1];
	frame.imu_raw_dpkg.magRaw[2] = imu_raw_msg->magRaw[2];
#endif
	frame.imu_raw_dpkg.eluRaw[0] = imu_raw_msg->eluRaw[0];
	frame.imu_raw_dpkg.eluRaw[1] = imu_raw_msg->eluRaw[1];
	frame.imu_raw_dpkg.eluRaw[2] = imu_raw_msg->eluRaw[2];
	
	frame.crc_dpkg = 0;
	
	int size = FRAME_INFO_SIZE+imu_raw_dpkg_len+FRAME_DPKG_CRC_BYTES;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],imu_raw_dpkg_len);
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+imu_raw_dpkg_len],crc16);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

/*
 * @brief 发送电机编码器脉冲
 */
void sendFrame_Encoder_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Encoder_dpkg 				frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = encoder_dpkg_len;
	frame.crc_header = 0;
	
	frame.encoder_dpkg.data_tag = encoder_tag;
	frame.encoder_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
	frame.encoder_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
	
	frame.crc_dpkg = 0;
	
	int size = FRAME_INFO_SIZE+encoder_dpkg_len+FRAME_DPKG_CRC_BYTES;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],encoder_dpkg_len);
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+encoder_dpkg_len],crc16);
	
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

/*
 * @brief 发送电池电压
 */
void sendFrame_Voltage_dpkg(const Robot_msgs* robot_msgs)
{
	Frame_Voltage_dpkg 				frame;
	frame.header[0]=Header1;
	frame.header[1]=Header2;
	frame.len = voltage_dpkg_len;
	frame.crc_header = 0;
	
	frame.voltage_dpkg.data_tag = voltage_tag;
	
	frame.voltage_dpkg.voltage = (uint8_t)(robot_msgs->voltage*10);
	
	int size = FRAME_INFO_SIZE+voltage_dpkg_len+FRAME_DPKG_CRC_BYTES;    //sizeof(*dpkg)
	memcpy(bytesBuff,&frame,size);
	
	
	/* reset crc value */
	uint8_t crc8 = crc_8(bytesBuff,FRAME_CALCU_CRC_BYTES);
	uint16_t crc16 = crc_16(&bytesBuff[FRAME_HEAD_CRC_OFFSET],voltage_dpkg_len);
	setBuffHeaderCRC(&bytesBuff[FRAME_DPKG_LEN_OFFSET],crc8);
	setBuffDpkgCRC(&bytesBuff[FRAME_INFO_SIZE+voltage_dpkg_len],crc16);
	
	for(int i=0;i<size;++i)
	{
		USART_SendData(USART1,bytesBuff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	
	}
}

/*
 * @brief 发送TAG数据包
 */
void sendFrame_Multi_dpkg(const Robot_msgs* robot_msgs)
{
//#define sendRobot
//#define sendIMU
//#define sendIMUSensor
//#define sendEncoder
//#define sendVoltage
//	
//#if !defined (sendRobot) && !defined (sendIMU) && !defined (sendIMUSensor) && \
//		!defined (sendEncoder) && !defined (sendVoltage)
//	return;
//#endif
//	
//	Frame_Info frame;
//	frame.header[0]=Header1;
//	frame.header[1]=Header2;
//	frame.len = 0;
//	frame.crc = 0;
//	
//#ifdef sendRobot
//	Robot_dpkg          robot_dpkg;
//	frame.len+=robot_dpkg_len;
//#endif /*  Robot_dpkg  */
//#ifdef sendIMU
//	IMU_dpkg            imu_dpkg;
//	frame.len+=imu_dpkg_len;
//#endif /*  IMU_dpkg  */
//#ifdef sendIMUSensor
//	IMU_Sensor_dpkg 		imu_sensor_dpkg;
//	frame.len+=imu_sensor_dpkg_len;
//#endif /*  IMU_Sensor_dpkg */
//#ifdef sendEncoder
//	Encoder_dpkg 				encoder_dpkg;
//	frame.len+=encoder_dpkg_len;
//#endif /*  Encoder_dpkg   */
//#ifdef sendVoltage
//	Voltage_dpkg 				voltage_dpkg;
//	frame.len+=voltage_dpkg_len;
//#endif /*  Voltage_dpkg   */

//	int size = FRAME_INFO_SIZE+frame.len;
//	if(size>MAX_BUFF_SIZE) return;
//	
//	memcpy(bytesBuff,&frame,FRAME_INFO_SIZE);
//	
//#ifdef sendRobot
//	robot_dpkg.data_tag = robot_tag;
//	
//	robot_dpkg.voltage =(uint8_t)(robot_msgs->voltage*10);
//	
//	robot_dpkg.l_encoder_pulse = robot_msgs->l_encoder_pulse;
//	robot_dpkg.r_encoder_pulse = robot_msgs->r_encoder_pulse;
//   	
//	robot_dpkg.acc[0] = robot_msgs->acc[0];
//	robot_dpkg.acc[1] = robot_msgs->acc[1];
//	robot_dpkg.acc[2] = robot_msgs->acc[2];
//	
//	robot_dpkg.gyr[0] = robot_msgs->gyr[0];
//	robot_dpkg.gyr[1] = robot_msgs->gyr[1];
//	robot_dpkg.gyr[2] = robot_msgs->gyr[2];

//	
//	#ifdef imu_mag
//	robot_dpkg.mag[0] = robot_msgs->mag[0];
//	robot_dpkg.mag[1] = robot_msgs->mag[1];
//	robot_dpkg.mag[2] = robot_msgs->mag[2];
//	#endif
//	robot_dpkg.elu[0] = robot_msgs->elu[0];
//	robot_dpkg.elu[1] = robot_msgs->elu[1];
//	robot_dpkg.elu[2] = robot_msgs->elu[2];
//	
//	memcpy(bytesBuff,&robot_dpkg,FRAME_INFO_SIZE);
//	
//#endif /*  Robot_dpkg  */
//#ifdef sendIMU


//#endif /*  IMU_dpkg  */
//#ifdef sendIMUSensor


//#endif /*  IMU_Sensor_dpkg */
//#ifdef sendEncoder


//#endif /*  Encoder_dpkg   */
//#ifdef sendVoltage

//#endif /*  Voltage_dpkg   */
//	
//	

//	
//	for(int i=0;i<size;++i)
//	{
//		USART_SendData(USART1,bytesBuff[i]);
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//	}
	
}


/**
 * @brief 串口读取IMU数据
 * 
 */
short imu_ready_flag=0x000f;
void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	static uint8_t buff[11]={0};
	static uint8_t count=0;
	static uint16_t checksum=0x55;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		buff[count++] = USART_ReceiveData(USART2);
		if(count==1 && buff[0]!=0x55 )  //检查帧头
		{
			count = 0;
			return;
		}
		else if(count<=10)
		{
			checksum+=buff[count-1];  //header+acc+gyr
			if(count == 10)
			{
				checksum = !(checksum>>8|(checksum&0x00FF));
			}
		}
		else if(count>10)
		{
			count=0;

//			if(checksum!=buff[10]) 
//			{
//				checksum = 0x55;
//				return;
//			}/* 校验错误 */
			checksum = 0x55;

			switch (buff[1])            //标签
			{
			case 0x51:  
				imu_raw_msg.accRaw[0]=Bytes2INT16Conv(&buff[2]);
				imu_raw_msg.accRaw[1]=Bytes2INT16Conv(&buff[4]);
				imu_raw_msg.accRaw[2]=Bytes2INT16Conv(&buff[6]);
				imu_ready_flag|=0xf000;
				break;  /* 加速度 */
			case 0x52:  
				imu_raw_msg.gyrRaw[0]=Bytes2INT16Conv(&buff[2]);
				imu_raw_msg.gyrRaw[1]=Bytes2INT16Conv(&buff[4]);
				imu_raw_msg.gyrRaw[2]=Bytes2INT16Conv(&buff[6]);
				imu_ready_flag|=0x0f00;
				break;	/* 角速度 */
			case 0x53:  
				imu_raw_msg.eluRaw[0]=Bytes2INT16Conv(&buff[2]);
				imu_raw_msg.eluRaw[1]=Bytes2INT16Conv(&buff[4]);
				imu_raw_msg.eluRaw[2]=Bytes2INT16Conv(&buff[6]);
				imu_ready_flag|=0x00f0;
				break;	/* 角度 */	
			default:
				break;
			}  /* switch */
			
		}
	} 
} 

/**
 * @brief 串口3引出端口
 */
void USART3_IRQHandler(void)                	//串口1中断服务程序
{
//	static uint8_t buff[11]={0};
//	static uint8_t count=0;
//	static uint16_t checksum=0x55;
//	++a;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		buff[count++] = USART_ReceiveData(USART3);
//		a = count;
//		if(count==1 && buff[0]!=0x55 )  //检查帧头
//		{
//			count = 0;
//			return;
//		}
//		else if(count<=10)
//		{
//			checksum+=buff[count-1];  //header+acc+gyr
//			if(count == 10)
//			{
//				checksum = !(checksum>>8|(checksum&0x00FF));
//			}
//		}
//		else if(count>10)
//		{
//			count=0;

////			if(checksum!=buff[10]) 
////			{
////				checksum = 0x55;
////				return;
////			}/* 校验错误 */
//			checksum = 0x55;

//			switch (buff[1])            //标签
//			{
//			case 0x51:  
//				imu_raw_msg.accRaw[0]=Bytes2INT16Conv(&buff[2]);
//				imu_raw_msg.accRaw[1]=Bytes2INT16Conv(&buff[4]);
//				imu_raw_msg.accRaw[2]=Bytes2INT16Conv(&buff[6]);
//				break;  /* 加速度 */
//			case 0x52:  
//				imu_raw_msg.gyrRaw[0]=Bytes2INT16Conv(&buff[2]);
//				imu_raw_msg.gyrRaw[1]=Bytes2INT16Conv(&buff[4]);
//				imu_raw_msg.gyrRaw[2]=Bytes2INT16Conv(&buff[6]);
//				break;	/* 角速度 */
//			case 0x53:  
//				imu_raw_msg.eluRaw[0]=Bytes2INT16Conv(&buff[2]);
//				imu_raw_msg.eluRaw[1]=Bytes2INT16Conv(&buff[4]);
//				imu_raw_msg.eluRaw[2]=Bytes2INT16Conv(&buff[6]);
//				break;	/* 角度 */	
//			default:
//				break;
//			}  /* switch */
//			
//		}
//	} 
} 
