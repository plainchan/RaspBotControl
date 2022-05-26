#include "raspbot_config.h"

/**
  * @brief  定时器1(高级定时器)中断,设置10ms中断，作为程序的控制时间逻辑
  * @param       
  */
void TIM1_IT_Init(u8 ms)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	NVIC_InitTypeDef NVIC_InitTypeStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //使能APB2 TIM1
	
	
	TIM_TimeBaseInitTypeStruct.TIM_Period = ms*10-1;          // arr
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 7199;      // Tout=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision =TIM_CKD_DIV1; //做输入捕获时滤波用的并不是定时器的分频器
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0;   //高级定时器才有，重复几次更新中断
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitTypeStruct);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);   //允许更新中断
	
	NVIC_InitTypeStruct.NVIC_IRQChannel =TIM1_UP_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority =0;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority =1;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitTypeStruct);
	
	TIM_ClearFlag(TIM1,TIM_IT_Update);   
	TIM_Cmd(TIM1,ENABLE);
}

/**
  * @brief  定时器2(通用定时器),用于控制无源蜂鸣器
  * @param
  */
void TIM2_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    //使能APB1 TIM2,PA1
	
	TIM_DeInit(TIM2);

	//引脚复用  PA1
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_1;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitTypeStruct);
	
	//定时器
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;  //无关PWM配置，ClockDivision是对于输入的分频，在输入捕获的时候要用到，相当于滤波
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period = 60000/1000-1;  
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =1199;          //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =0;           
	

	TIM_OC2Init(TIM2,&TIM_OCInitTypeStruct);  //CH2

	
	
	//高级定时器必须要配置，否则无法输出P波
//	TIM_CtrlPWMOutputs(TIMx,ENABLE);   //高级定时器  使能刹车和死区寄存器 MOE使能

	//预装置使能，修改比较值则立即写入寄存器，否则等待下一个周期更改比较值
	//对P波几乎没有影响
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable); //使能预装载寄存器  CH2
	
	TIM_Cmd(TIM2,ENABLE);	
}
/**
  * @brief  定时器3(通用定时器)，1KHz,输出PWM，控制电机
  * @param       
  */
void TIM3_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   //使能APB1 TIM3,PB0


	//引脚复用 PB0
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_0;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	//定时器
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period =999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =71;          //1KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =500;           //占空比
	

	TIM_OC3Init(TIM3,&TIM_OCInitTypeStruct);  //CH3

	
	
	//高级定时器必须要配置，否则无法输出P波
//	TIM_CtrlPWMOutputs(TIM3,ENABLE);   //高级定时器  使能刹车和死区寄存器 MOE使能

	//预装置使能，修改比较值则立即写入寄存器，否则等待下一个周期更改比较值
	//对P波几乎没有影响
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable); //使能预装载寄存器  CH3

	
	TIM_Cmd(TIM3,ENABLE);	
}

/**
  * @brief  定时器4(通用定时器)，1KHz,输出PWM，控制电机
  * @param       
  */
void TIM4_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);   //使能APB1 TIM4


	//引脚复用
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin =GPIO_Pin_8;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	//定时器
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period =999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler =71;          //10KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitTypeStruct);
	
	//PWM
	TIM_OCInitTypeStruct.TIM_OCMode =TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse =500;           //占空比
	
	TIM_OC3Init(TIM4,&TIM_OCInitTypeStruct);  //CH3

	//高级定时器必须要配置，否则无法输出P波
  //	TIM_CtrlPWMOutputs(TIMx,ENABLE);   //高级定时器  使能刹车和死区寄存器 MOE使能

	//预装置使能，修改比较值则立即写入寄存器，否则等待下一个周期更改比较值
	//对P波几乎没有影响
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能预装载寄存器  CH3
	
	//Reset counter
  TIM_SetCounter(TIM4,0);
	
	TIM_Cmd(TIM4,ENABLE);	
}
/**
  * @brief  buzzer 频率,用于控制无源蜂鸣器
  * @param  hz  1=<hz<=3000
  */
void buzzer_hz(u16 hz,FunctionalState enable)
{
	if(hz<1)
		hz=1;
	else if(hz>3000)
		hz=3000;
	u16 arr = 60000/hz -1;
	u16 pulse = 30000/hz -1;   //50%
	TIM_SetAutoreload(TIM2,arr);
	TIM_SetCompare2(TIM2,pulse);
	
	TIM_Cmd(TIM2,enable);
}


/**
  * @brief  电机IO初始化  
  */
void motor_init(void)
{
	//电机方向控制端口
	//左电机 A4 A5
	//右电机 B9 C13
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2;		 		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOA,MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2);			
//	GPIO_PinLockConfig(GPIOA,MOTOR_L_DIR_PIN1|MOTOR_L_DIR_PIN2);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_DIR_PIN1;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOB,MOTOR_R_DIR_PIN1);			
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_DIR_PIN2;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOC,MOTOR_R_DIR_PIN2);		

	//电机PWM 端口
	//左 PB0 TIM3_CH3
	//右 PB8 TIM4_CH3
//	TIM3_PWM_Init();
//	TIM4_PWM_Init();
}

/////////////////////////电机正反转///////////////////////
void motor_L_stop(void)
{
	MOTOR_L_DIR1=0;
	MOTOR_L_DIR2=0;
}
void motor_L_forward(void)
{
	MOTOR_L_DIR1=1;
	MOTOR_L_DIR2=0;
}
void motor_L_reverse(void)
{
	MOTOR_L_DIR1=0;
	MOTOR_L_DIR2=1;
}

void motor_R_stop(void)
{
	MOTOR_R_DIR1=0;
	MOTOR_R_DIR2=0;

}
void motor_R_forward(void)
{
	MOTOR_R_DIR1=1;
	MOTOR_R_DIR2=0;
}
void motor_R_reverse(void)
{
	MOTOR_R_DIR1=0;
	MOTOR_R_DIR2=1;
}

void motor_forward(void)
{
	motor_L_forward();
	motor_R_forward();
}
void motor_reverse(void)
{
	motor_L_reverse();
	motor_R_reverse();
}
void motor_stop(void)
{
	motor_L_stop();
	motor_R_stop();
}

/**
  * @brief  电机PWM控制
  * @param  duty_L：左电机PWM <=1000   duty_R：右电机PWM <=1000      
  */
void motor_pwm(uint16_t duty_L,uint16_t duty_R)
{
	TIM_SetCompare3(TIM3,duty_L);
	TIM_SetCompare3(TIM4,duty_R);
}


/**
  * @brief  编码器IO初始化
  */
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);  //使能定时器4的时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		

  
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;                    // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                   //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  ////TIM向上计数  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	//TIM_ICPolarity_Rising表示TIx极性不反相     TIM_ICPolarity_Rising TIx极性反相(正转计数器下降)
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);   //使用编码器模式3
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
	TIM_ICStructInit(&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
  //Reset counter
  TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);
	
  TIM_Cmd(TIM3, ENABLE); 
	TIM_Cmd(TIM4, ENABLE); 
}
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM2 -> CNT=0 ;break;
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT; /* TIM3 -> CNT=0 */;break;	
	 }
	 return Encoder_TIM;
}
/**************************************************************************
函数功能：TIM3中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}

/**
  * @brief  电压检测初始化
  */
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	ADC_InitTypeDef ADC_InitTypeStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  //使能ADC时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    //ADC时钟 72/6=12MHZ,超过14MHZ精度会变低
	ADC_DeInit(ADC1);                                    //复位时钟
	
	//引脚 PB1
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AIN;                          //引脚模拟输入
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitTypeStruct);
	
	ADC_InitTypeStruct.ADC_Mode = ADC_Mode_Independent;                     //独立模式
	ADC_InitTypeStruct.ADC_ScanConvMode = DISABLE;                          //单通道模式
	ADC_InitTypeStruct.ADC_ContinuousConvMode = DISABLE;                    //单次转换
	ADC_InitTypeStruct.ADC_DataAlign = ADC_DataAlign_Right;                 //数据 
	ADC_InitTypeStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;    //软件触发 
	ADC_InitTypeStruct.ADC_NbrOfChannel = 1;                                //规则转换的ADC通道的数目
	ADC_Init(ADC1,&ADC_InitTypeStruct);
		
	ADC_Cmd(ADC1,ENABLE);                          //使能ADC
	ADC_ResetCalibration(ADC1);                    //重置校准
	while(ADC_GetResetCalibrationStatus(ADC1));   //等待重置完成
	ADC_StartCalibration(ADC1);                    //开始校准ADC1;
	while(ADC_GetCalibrationStatus(ADC1));        //等待校准完成
	
}

u16 getAnalogValue()
{
	 //设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		       //使能指定的ADC1的软件转换启动功能	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));  //等待转换结束
	return ADC_GetConversionValue(ADC1);
}

/**
	*		@brief  串口通信，
	*
	*
	*/

void UART1_Init(u32 baud)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = baud;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断
  USART_Cmd(USART1, ENABLE);                    
}

/**
*
*
*/
void UART2_Init(u32 baud)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART2_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART2_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = baud;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接收中断
  USART_Cmd(USART2, ENABLE);                    
}



/**
  * @brief  board IO initial
  */
void board_configInit(void)
{
	//使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
<<<<<<< HEAD
=======
	
	
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee
	
	
	
//	SystemInit();
	delay_init();
	
	oled_init();
	oled_picture(0,0,128,64,start_bmp);
	oled_update();
	delay_ms(1000);
	OLED_Clear();
	
	motor_init();
<<<<<<< HEAD
	UART1_Init(115200);
	TIM1_IT_Init(10);         /* must initialize after usart,because send data in interrupts */
=======
	TIM1_IT_Init(10);
>>>>>>> 248ba16e6f35584dac582c2177468445d51485ee
	TIM2_PWM_Init();
	adc_init();
	
//	motor_stop();
//	motor_pwm(500,500);
//	encoder_init();
	
	
	
	
}
