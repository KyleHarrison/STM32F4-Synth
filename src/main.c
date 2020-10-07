/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **                STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 **  Distribution: The file is distributed “as is,” without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. Distribution of this file (unmodified or modified) is not
 **  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 **  rights to distribute the assembled, compiled & linked contents of this
 **  file as part of an application binary file, provided that it is built
 **  using the Atollic TrueSTUDIO(R) toolchain.
 **
 **
 *****************************************************************************
 */

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

/* Private macro */
#define DACBUFFERSIZE 		200
#define WAVEFREQ			880 /* 880Hz --> A5 */
#define TIMER6_PRESCALER	2 	/* produces a 42MHz tick */
#define TIMER_CLOCK			84E6 /* TIM6 runs at 84MHz */
/* Private variables */
uint16_t DACBuffer[DACBUFFERSIZE]; 	/* Array for the waveform */

/* Private function prototypes */
/* Private functions */
void RCC_Configuration(void);
void DMA_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void UART_Configuration(void);
void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler);
void DAC_Configuration(void);
uint32_t ConvertedValue =0; //Converted value readed from ADC
uint8_t encoder =0;
uint8_t freqChange = 1;
/*******************************************************************************
 * Function Name  : TIM2_IRQHandler
 * Description    : This function handles TIM2 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//Do stuff here
		//STM_EVAL_LEDToggle(LED6);

		uint16_t timP;

		/* Calculate frequency of timer */
		uint32_t fTimer = 27.5*DACBUFFERSIZE;

		uint32_t lol = ConvertedValue;

		/* Calculate Tick Rate */
		uint32_t timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

		/* Calculate period of Timer */
		timP = (uint16_t)( (timerFreq) / fTimer );
		timP = (uint16_t)(timP/pow(1.104089514,freqChange-1));
		uint16_t n;

		TIM6->ARR = timP;
	}
}


void adc_configure(){
	ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
	//Analog pin configuration
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(GPIOC,&GPIO_initStructre);//Affecting the port with the initialization structure configuration
	//ADC structure configuration
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
	ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 1;//I think this one is clear :p
	ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration
	//Enable ADC conversion
	ADC_Cmd(ADC1,ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_144Cycles);
}


uint32_t adc_convert(){
	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	return ADC_GetConversionValue(ADC1); //Return the converted data
}

/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Do your stuff when PD0 is changed */
		STM_EVAL_LEDToggle(LED6);
		delay_ms(20);
		encoder = encoder + 1;
		freqChange = encoder;
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		/* Do your stuff when PB12 is changed */
		//STM_EVAL_LEDToggle(LED6);

		STM_EVAL_LEDToggle(LED6);

		delay_ms(20);
		encoder+=2;
		freqChange = encoder;

		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}
/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void)
{
	uint32_t conv = ConvertedValue;
	uint32_t fTimer;
	uint32_t timerFreq;
	uint16_t timerPeriod;
	uint16_t n;
	uint16_t m;


	adc_configure();//Start configuration

	/* Configure PD0 as interrupt */
	Configure_PD0();
	/* Configure PB12 as interrupt */
	Configure_PB12();


	// SystemInit();

	/* Calculate the gradient of the Sawtooth */
	m = (uint16_t) ( 4095 / DACBUFFERSIZE);

	/* Create wave table for sawtooth */

	/*for (n = 0; n<DACBUFFERSIZE; n++)
	{
		DACBuffer[n] = (uint16_t)(m*n);
	}*/


	/* Create wave table for sin */


	for (n = 0; n<DACBUFFERSIZE; n++)
	{
		DACBuffer[n] = (uint16_t)((0xFFF+1)/2)*(sin((2*3.142*n)/DACBUFFERSIZE) + 1);
	}



	/* Calculate frequency of timer */
	fTimer = WAVEFREQ * DACBUFFERSIZE;

	/* Calculate Tick Rate */
	timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

	/* Calculate period of Timer */
	timerPeriod = (uint16_t)( timerFreq / fTimer );


	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();

	/* Timer Configuration */
	Timer_Configuration( timerPeriod, TIMER6_PRESCALER );

	/* DAC Configuration */
	DAC_Configuration();

	/* DMA Config */
	DMA_Configuration ();

	STM_EVAL_LEDInit(LED6);


	/* Infinite loop */
	while (1)
	{
conv = encoder;
//TIM6->ARR = timP;
encoder = 0;

		/* Do nothing... */
	}
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval : None
 */
void RCC_Configuration(void)
{
	/* Enable DMA and GPIOA Clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable DAC1 and TIM6 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM2, ENABLE);

}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval : None
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval : None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Pack the struct */
	GPIO_InitStruct.GPIO_Speed = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Call Init function */
	GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
 * @brief  Configures the DMA.
 * @param  None
 * @retval : None
 */
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	//Initialize the structure to default values
	DMA_StructInit(&DMA_InitStructure);

	/* PACK YOUR INIT STRUCT HERE */
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_BASE+0x08;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DACBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = DACBUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* Call Init function */
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	/* Enable DMA */
	DMA_Cmd(DMA1_Stream5, ENABLE);

}

/**
 * @brief  Configures the Timers.
 * @param  wavePeriod (period of timer), preScaler (prescaler for timer)
 * @retval : None
 */
void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;

	/* pack Timer struct */
	TIM_TimeBaseStruct.TIM_Period = wavPeriod-1;
	TIM_TimeBaseStruct.TIM_Prescaler = preScaler-1;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0x0000;

	/* Call init function */
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

	/* Select Timer to trigger DAC */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	TIM_TimeBaseInitStruct.TIM_Prescaler = 8400-1;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct );
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}

/**
 * @brief  Configures the DAC
 * @param  None
 * @retval : None
 */

/* Configure pins to be interrupts */
void Configure_PD0(void) {
	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
}

void Configure_PB12(void) {
	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOB */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Tell system that you will use PB12 for EXTI_Line12 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

	/* PB12 is connected to EXTI_Line12 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
}




void DAC_Configuration(void)
{
	DAC_InitTypeDef DAC_InitStruct;

	/* Initialize the DAC_Trigger member */
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;
	/* Initialize the DAC_WaveGeneration member */
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	/* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	/* Initialize the DAC_OutputBuffer member */
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* Init DAC */
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);

	/* Enable DMA request */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
	/* TODO, implement your code here */
	return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
	/* TODO, implement your code here */
	return -1;
}

/**
 * @brief  Delay in ms
 * @param  integer amount of milliseconds
 * @retval None
 */
void delay_ms(uint32_t milli)
{
	uint32_t delay = milli * 17612;              // approximate loops per ms at 168 MHz, Debug config
	for(; delay != 0; delay--);
}
