/**
 *****************************************************************************
 **
 ** File : main.c
 **
 ** Abstract : main function.
 **
 ** Functions : main
 **
 ** Environment : Atollic TrueSTUDIO(R)
 ** STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 ** Distribution: The file is distributed “as is,” without any warranty
 ** of any kind.
 **
 ** (c)Copyright Atollic AB.
 ** You may use this file as-is or modify it according to the needs of your
 ** project. Distribution of this file (unmodified or modified) is not
 ** permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 ** rights to distribute the assembled, compiled & linked contents of this
 ** file as part of an application binary file, provided that it is built
 ** using the Atollic TrueSTUDIO(R) toolchain.
 **
 **
 *****************************************************************************
 */

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

/* Private macro */
#define DACBUFFERSIZE 250
#define WAVEFREQ 880 /* 880Hz --> A5 */
#define TIMER6_PRESCALER 2 /* produces a 42MHz tick */
#define TIMER_CLOCK 84E6 /* TIM6 runs at 84MHz */
/* Private variables */
uint16_t DACBuffer[DACBUFFERSIZE]; /* Array for the waveform */

/* Private function prototypes */
/* Private functions */
void RCC_Configuration(void);
void DMA_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void UART_Configuration(void);
void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler);
void DAC_Configuration(void);

/*******************************************************************************
 * Function Name : TIM2_IRQHandler
 * Description : This function handles TIM2 global interrupt request.
 * Input : None
 * Output : None
 * Return : None
 *******************************************************************************/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//Do stuff here
		static int toggle = 1;
		uint16_t timP;
		if(toggle)
		{
			/* Calculate frequency of timer */
			uint32_t fTimer = 880 * DACBUFFERSIZE;

			/* Calculate Tick Rate */
			uint32_t timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

			/* Calculate period of Timer */
			timP = (uint16_t)( timerFreq / fTimer );
			toggle = 0;
		}
		else
		{
			/* Calculate frequency of timer */
			uint32_t fTimer = 1046.5 * DACBUFFERSIZE;

			/* Calculate Tick Rate */
			uint32_t timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

			/* Calculate period of Timer */
			timP = (uint16_t)( timerFreq / fTimer );
			toggle = 1;
		}
		TIM6->ARR = timP;
	}
}


/**
 **===========================================================================
 **
 ** Abstract: main program
 **
 **===========================================================================
 */
int main(void)
{
	uint32_t fTimer;
	uint32_t timerFreq;
	uint16_t timerPeriod;
	uint16_t n;
	uint16_t m;

	/* Calculate the gradient of the Sawtooth */
	m = (uint16_t) ( 4095 / DACBUFFERSIZE);

	/* Create wave table for sawtooth */
	for (n = 0; n<DACBUFFERSIZE; n++)
	{
		DACBuffer[n] = (uint16_t)((0xFFF+1)/2)*(sin((2*3.142*n)/DACBUFFERSIZE) + 1); /* Remember to cast! */
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


	/* Infinite loop */
	while (1)
	{
		/* Do nothing... */
	}
}

/**
 * @brief Configures the different system clocks.
 * @param None
 * @retval : None
 */
void RCC_Configuration(void)
{
	/* Enable DMA and GPIOA Clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);


	/* Enable DAC1 and TIM6 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM2 , ENABLE);
}

/**
 * @brief Configures the nested vectored interrupt controller.
 * @param None
 * @retval : None
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


/**
 * @brief Configures the different GPIO ports.
 * @param None
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
 * @brief Configures the DMA.
 * @param None
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
 * @brief Configures the Timers.
 * @param wavePeriod (period of timer), preScaler (prescaler for timer)
 * @retval : None
 */
void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

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
	TIM_TimeBaseInitStruct.TIM_Period = 9999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct );
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}

/**
 * @brief Configures the DAC
 * @param None
 * @retval : None
 */
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
