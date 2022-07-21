#include "raspbot_config.h"
#include "ps2lib.h"
/**
 * @brief  定时器1(高级定时器)中断,设置10ms中断，作为程序的控制时间逻辑
 * @param
 */
void TIM1_IT_Init(u8 ms)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitTypeStruct;
	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能APB2 TIM1

	TIM_TimeBaseInitTypeStruct.TIM_Period = ms * 10 - 1;		 // arr
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 7199;			 // Tout=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1; //做输入捕获时滤波用的并不是定时器的分频器
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0; //高级定时器才有，重复几次更新中断
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitTypeStruct);

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //允许更新中断

	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitTypeStruct);

	TIM_ClearFlag(TIM1, TIM_IT_Update);
	TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  定时器2(通用定时器),编码器模式，
 * @param
 */
void TIM2_Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
//	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); //使能定时器4的时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;				// 预分频器
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;					//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////TIM向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// TIM_ICPolarity_Rising表示TIx极性不反相     TIM_ICPolarity_Rising TIx极性反相(正转计数器下降)
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3

	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update); //清除TIM的更新标志位
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

//	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitTypeStruct);

	// Reset counter
	TIM_SetCounter(TIM2, 0);

	TIM_Cmd(TIM2, ENABLE);
}
/**
 * @brief  定时器4(通用定时器)，编码器模式
 * @param
 */
void TIM4_Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
//	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE); //使能定时器4的时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;				// 预分频器
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;					//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// TIM_ICPolarity_Rising表示TIx极性不反相     TIM_ICPolarity_Rising TIx极性反相(正转计数器下降)
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3


	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update); //清除TIM的更新标志位
//	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

//	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM4_IRQn;
//	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitTypeStruct);

	// Reset counter
	TIM_SetCounter(TIM4, 0);

	TIM_Cmd(TIM4, ENABLE);
}
/**
 * @brief  编码器初始化
 */
void encoder_init(void)
{
	TIM2_Encoder_Init();
	TIM4_Encoder_Init();
}

/**
 * @brief  编码器IO初始化
 */
short Read_Encoder(u8 TIMX)
{
	short pluse;
	switch (TIMX)
	{
	case 2:
		pluse = (short)TIM2->CNT;
		TIM2->CNT = 0;
		break;
	case 4:
		pluse = (short)TIM4->CNT;
		TIM4->CNT = 0;
		break;
	}
	return pluse;
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能APB1 TIM3,PB0
	
	//引脚复用 CH1 PA6  CH2 PA7
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;   
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeStruct);
	//引脚复用 CH3 PB0  CH4 PB1
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;   
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);
	
	
	//定时器
	TIM_TimeBaseInitTypeStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitTypeStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitTypeStruct.TIM_Period = 999;
	TIM_TimeBaseInitTypeStruct.TIM_Prescaler = 71; // 1KHz   //Hz=(arr+1)*(prc+1)/Tclk
	TIM_TimeBaseInitTypeStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitTypeStruct);

	// PWM
	TIM_OCInitTypeStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitTypeStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitTypeStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitTypeStruct.TIM_Pulse = 0; //占空比
	
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStruct); // CH1
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStruct); // CH2
	TIM_OC3Init(TIM3, &TIM_OCInitTypeStruct); // CH3
	TIM_OC4Init(TIM3, &TIM_OCInitTypeStruct); // CH4

	//高级定时器必须要配置，否则无法输出P波
	//	TIM_CtrlPWMOutputs(TIM3,ENABLE);   //高级定时器  使能刹车和死区寄存器 MOE使能

	//预装置使能，修改比较值则立即写入寄存器，否则等待下一个周期更改比较值
	//对P波几乎没有影响
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能预装载寄存器  CH3

	TIM_Cmd(TIM3, ENABLE);
}



////////////////////////
void MOTOR_L_ENABLE(FunctionalState state)
{
	if(state)
		MOTOR_EN_L=1;
	else
		MOTOR_EN_L=0;
}
void MOTOR_R_ENABLE(FunctionalState state)
{
	if(state)
		MOTOR_EN_R=1;
	else
		MOTOR_EN_R=0;
}
void MOTOR_ENABLE(FunctionalState state)
{
	MOTOR_L_ENABLE(state);
	MOTOR_R_ENABLE(state);
}
/**
 * @brief  电机IO初始化
 */
void motor_init(void)
{
	//电机使能控制端口
	//左电机 A4
	//右电机 B2
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = MOTOR_EN_L_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA, MOTOR_EN_L_PIN);
	
	GPIO_InitStructure.GPIO_Pin =MOTOR_EN_R_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOB,MOTOR_EN_R_PIN);
	
	//	GPIO_PinLockConfig(GPIOA,MOTOR_EN_L_PIN);
	TIM3_PWM_Init();
	MOTOR_ENABLE(ENABLE);

}



/**
 * @brief  电机PWM控制
 * @param  duty_L：左电机PWM <=1000   duty_R：右电机PWM <=1000  
 */
void motor_pwm(int16_t duty_L, int16_t duty_R)
{
	if(duty_L>=0)
	{	
		TIM_SetCompare1(TIM3, duty_L);
		TIM_SetCompare2(TIM3, 0);
	}
	else
	{
		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, -duty_L);
	}
	
	if(duty_R>=0)
	{	
		TIM_SetCompare3(TIM3, 0);
		TIM_SetCompare4(TIM3, duty_R);
	}
	else
	{
		TIM_SetCompare3(TIM3, -duty_R);
		TIM_SetCompare4(TIM3, 0);
	}
}


/**
 * @brief  TIM2 中断服务函数，编码器溢出中断处理
 */
void TIM2_IRQHandler(void)
{

	if (TIM2->SR & 0X0001) //溢出中断
	{
	}
	TIM2->SR &= ~(1 << 0); //清除中断标志位
}
/**
 * @brief  TIM4 中断服务函数，编码器溢出中断处理
 */
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & 0X0001) //溢出中断
	{
	}
	TIM4->SR &= ~(1 << 0); //清除中断标志位
}

/**
 * @brief  电压检测初始化
 */
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	ADC_InitTypeDef ADC_InitTypeStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADC时钟 72/6=12MHZ,超过14MHZ精度会变低
	ADC_DeInit(ADC1);				  //复位时钟

	//引脚 PA5
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AIN; //引脚模拟输入
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_5;
	//	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);

	ADC_InitTypeStruct.ADC_Mode = ADC_Mode_Independent;					 //独立模式
	ADC_InitTypeStruct.ADC_ScanConvMode = DISABLE;						 //单通道模式
	ADC_InitTypeStruct.ADC_ContinuousConvMode = DISABLE;				 //单次转换
	ADC_InitTypeStruct.ADC_DataAlign = ADC_DataAlign_Right;				 //数据
	ADC_InitTypeStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //软件触发
	ADC_InitTypeStruct.ADC_NbrOfChannel = 1;							 //规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitTypeStruct);

	ADC_Cmd(ADC1, ENABLE);		//使能ADC
	ADC_ResetCalibration(ADC1); //重置校准
	while (ADC_GetResetCalibrationStatus(ADC1))
		;						//等待重置完成
	ADC_StartCalibration(ADC1); //开始校准ADC1;
	while (ADC_GetCalibrationStatus(ADC1))
		; //等待校准完成
}

u16 getAnalogValue()
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); // ADC1,ADC通道,采样时间为239.5周期

	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能指定的ADC1的软件转换启动功能
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
		; //等待转换结束
	return ADC_GetConversionValue(ADC1);
}

/**
 *		@brief  串口通信，
 *
 *
 */

void UART1_Init(u32 baud)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能USART1，GPIOA时钟

	// USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化GPIOA.9

	// USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  // PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //初始化GPIOA.10

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器

	// USART 初始化设置

	USART_InitStructure.USART_BaudRate = baud;										//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART1, &USART_InitStructure);	   //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口接收中断
	USART_Cmd(USART1, ENABLE);
}


void UART2_Init(u32 baud)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   //初始化GPIOA.2

	// USART2_RX	  GPIOA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;			  // PA.3
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPD;   //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //初始化GPIOA.3

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器

	// USART 初始化设置

	USART_InitStructure.USART_BaudRate = baud;										//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART2, &USART_InitStructure);	   //初始化串口1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //开启串口接收中断
	USART_Cmd(USART2, ENABLE);
}

/**
 *
 *
 */
void UART3_Init(u32 baud)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);   //使能USART1，GPIOA时钟

	// USAR3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化GPIOB.10

	// UART3_RX	  GPIOB.11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			// PB.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);				  //初始化GPIOB.11

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器

	// USART 初始化设置

	USART_InitStructure.USART_BaudRate = baud;										//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART3, &USART_InitStructure);	   //初始化串口1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //开启串口接收中断
	USART_Cmd(USART3, ENABLE);
}

/**
 * @brief  状态指示灯
 */
void stateLED_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//  GPIOC.13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         // PC.13
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);			      //初始化GPIOC.13
}

/**
 * @brief  安全模式，长按案件上锁/解锁
 */
volatile char safe_mode = 0;
void security_mode(void)
{
	while(safe_mode)
	{
	}
}


volatile char uart_lock = 0;

/**
 * @brief  board IO initial
 */
void board_configInit(void)
{
	SystemInit();
	//使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/* Disable JLink, enable SW */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);

	//中断分组
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	
	/* ps2 port use JTAG */
	delay_init();
	ps2_init();
	stateLED_init();
	oled_init();
	oled_picture(0, 0, 128, 64, start_bmp);
	oled_update();
	
	delay_ms(1000);
	OLED_Clear();
	
	
	motor_init();
	UART1_Init(115200);    // 通信串口
//	UART2_Init(115200);    // IMU串口
//	UART3_Init(115200);    // 串口3引出端口

	TIM1_IT_Init(10); /* must initialize after usart,because send data in interrupts */
	
	motor_init();
	adc_init();
	encoder_init();
	
	//Due to the occupation of JTAG's ports,must configure the remapping of IO.
	//It can't work if the position of the function "ps2_init()" ahead any sentens int the function "board_configInit";
	// So the function "ps2_init()" must be in the end.
	//WHY???????
	ps2_init();
	
}
