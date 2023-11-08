#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

#include "misc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 333;  //Значение регистра сравнения CCR1
uint16_t CCR2_Val = 249; //Значение регистра сравнения CCR2
uint16_t CCR3_Val = 166; //Значение регистра сравнения CCR3
uint16_t CCR4_Val = 83;     //Значение регистра сравнения CCR4
uint16_t PrescalerValue = 0; //Значение предделителя

__IO uint16_t ADCConvertedValue[4];


void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}
/*  Обработка событий таймера  TIM4  */
void TIM4_IRQHandler(void)
{
  /* Событие канала сравнения 1*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    /* Очищаем TIM4 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

    /* Здесь можно выполнить какую-нибудь команду */
 //   GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  }
  /* Событие канала сравнения 2*/
  else if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  {
    /* Очищаем TIM4 Capture Compare2 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

    /* Здесь можно выполнить какую-нибудь команду */
 //   GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  }
  /* Событие канала сравнения 3*/
  else if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
  {
    /* Очищаем TIM4 Capture Compare3 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);

    /* Здесь можно выполнить какую-нибудь команду */
//    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  }
  /* Событие канала сравнения 4*/
  else if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
  {
    /* Очищаем TIM4 Capture Compare4 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);

    /* Здесь можно выполнить какую-нибудь команду */
//    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  } /* Событие обновления счетчика   */
  else
  {
    /* Очищаем TIM4 update interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    /* Здесь можно перезагрузить значения регистров сравнения */
    TIM4->CCR1=ADCConvertedValue[1];
    TIM4->CCR2=ADCConvertedValue[2];

  }
}

/**
  * @brief   TIM_PWMOutput, конфигурирование  Channel1-Channel4 TIM4 для генерации  4 PWM сигналов с четырьмя разными коэффициентами заполнения.
  */

void TIM_PWMOutput(void)
{


    NVIC_Configuration();

    PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

    TIM_TimeBaseStructure.TIM_Period = 4096; //Верхнее значение счетчика

    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

}


void GPIO_Configuration_Tim(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Структуры инициализации */
GPIO_InitTypeDef GPIO_InitStructure;
//Глобальная переменная, содержащая результат преобразования
__IO uint16_t ADCConvertedValue[4]; ////
DMA_InitTypeDef DMA_InitStructure;
ADC_InitTypeDef ADC_InitStructure;

int main(void)
{
		SystemInit();
		/*  конфигурирование ADC как в листинге 3.1
		    ADC1 конфигурация -------------------------------*/

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		/* Устанавливаем делитель тактовой частоты преобразователя ADC    */
		RCC_ADCCLKConfig(RCC_PCLK2_Div4);


		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		/* Подаем тактовую частоту к DMA1 */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
	                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		 /* Конфигурирование контроллера прямого доступа к памяти DMA1 channel1------*/
		DMA_DeInit(DMA1_Channel1);//Сбрасываем предыдущую конфигурацию
		DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x4001244C);//Адрес источника
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;//Адрес назначения
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//Направление передачи периферия-> памяти
		DMA_InitStructure.DMA_BufferSize = 4; //Размер приемного буфера
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//Инкремент адреса источника выключен
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//Инкремент адреса приемника выключен!!!!
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//размер данных 16 бит
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//размер данных 16 бит
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //Непрерывный режим работы
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; //Приоритет высокий
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //режим память-память отключен
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);//Инициализауия контроллера
		  /* Включение первого канала (DMA1 channel1) */
		DMA_Cmd(DMA1_Channel1, ENABLE);
		/* ADC1 конфигурирование канала 1 */
		ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
		ADC_DMACmd(ADC1, ENABLE);
		/* Включение ADC1 */
		ADC_Cmd(ADC1, ENABLE);
		/* Прсограммный старт преобразования ADC1 */
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		ADC_InitTypeDef ADC_InitStructure;
		/* Конфигурируем PA01 (ADC Channel1) как analog input --------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* ADC1 конфигурация ---------------------------------------------------*/
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //Независимыйрежим
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;       //Сканировниеканаловвключено
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //Режимнепрерывногопреобразованиявключен
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Стартповнешнемусобытиюотключен
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //Сдвигданныхвправо
		ADC_InitStructure.ADC_NbrOfChannel = 4;            //Число задействованных каналов 4
		ADC_Init(ADC1, &ADC_InitStructure);//Инициализация ADC

		ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 4, ADC_SampleTime_55Cycles5);
		  /* Включение ADC1 */	i
		/* Подключение  DMA к ADC1 */
		ADC_Cmd(ADC1, ENABLE);
		/* Сброс регистров калибровки  ADC1  */
		ADC_ResetCalibration(ADC1);
		/* ожидание завершения сброса регистров ADC1  */
		while(ADC_GetResetCalibrationStatus(ADC1));
		/* Старт калибровки ADC1  */
		ADC_StartCalibration(ADC1);
		  /* Ожидание калибровки ADC1  */
		while(ADC_GetCalibrationStatus(ADC1));

		//////
		/* Подаем тактовую частоту к DMA1 */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* Устанавливаем делитель тактовой частоты преобразователя ADC    */
		RCC_ADCCLKConfig(RCC_PCLK2_Div4);
		/* Подаем тактовую частоту к DMA1 */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_Configuration_Tim();
		TIM_PWMOutput();
		while(1)
		{

		}

}

