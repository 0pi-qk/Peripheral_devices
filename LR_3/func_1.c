#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

__IO uint16_t ADCConvertedValue; //Для одного канала

void change_light(uint16_t pin, int stay) {
	GPIO_WriteBit(GPIOB, pin, stay);
}


void Func_1(void)
{
    int i;
    GPIO_InitTypeDef  GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div4); // Делитель тактовой частоты
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    /* Подаем тактовую частоту к DMA1 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Конфигурируем PA01 (ADC Channel1) как analog input --------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ADC1 конфигурация ---------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //Независимый режим
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; //Сканирование каналов включено
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //Режим непрерывного преобразования включен
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Старт по внешнему событию отключен
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //Сдвиг данных вправо
    ADC_InitStructure.ADC_NbrOfChannel = 1; //Число задействованных каналов 4
    ADC_Init(ADC1, &ADC_InitStructure); //Инициализация ADC

     /* Конфигурирование контроллера прямого доступа к памяти DMA1 channel1------*/
     DMA_DeInit(DMA1_Channel1); //Сбрасываем предыдущую конфигурацию
     DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x4001244C); //Адрес источника
     DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue; //Адрес назначения
     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //Направление передачи периферия->память
     DMA_InitStructure.DMA_BufferSize = 1; //Размер приемного буфера
     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //Инкремент адреса источника выключен
     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; //Инкремент адреса приемника выключен
     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //размер данных 16 бит
     DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //размер данных 16 бит
     DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Непрерывный режим работы
     DMA_InitStructure.DMA_Priority = DMA_Priority_High;//Приоритет высокий
     DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //режим память-память отключен
     DMA_Init(DMA1_Channel1, &DMA_InitStructure); //Инициализация контроллера

     /* Включение первого канала (DMA1 channel1) */
     DMA_Cmd(DMA1_Channel1, ENABLE);


     /ADC1 конфигурация -------------------------------*/

     ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);


     /* Подключение DMA к ADC1 */
     ADC_DMACmd(ADC1, ENABLE);

     /* Включение ADC1 */
     ADC_Cmd(ADC1, ENABLE);

     /* Программный старт преобразования ADC1 */
     ADC_SoftwareStartConvCmd(ADC1, ENABLE);

       while (1) {
    	   // Наклон вверх/вниз
    	   if(ADCConvertedValue<1000 ) { // up
    		   change_light(GPIO_Pin_6, Bit_SET);
    	   }
    	   if(ADCConvertedValue>3900 & ADCConvertedValue<4100) { // down
    		   change_light(GPIO_Pin_8, Bit_SET);
    	   }
    	   if(ADCConvertedValue>1200 & ADCConvertedValue<2400) {
    		   change_light(GPIO_Pin_6, Bit_RESET);
    		   change_light(GPIO_Pin_8, Bit_RESET);
    	   }
       }
}
