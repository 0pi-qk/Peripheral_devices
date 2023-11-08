#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"

#include "misc.h"

static __IO uint32_t TimingDelay;
uint8_t FlagBTN = 0;

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) //Проверяем наличие запроса на прерывание
    {
/* Устанавливаем флаг */
        FlagBTN = 1;
    }
    /* Снимаем бит запроса прерывания по линии EXTI_Line0 (pending bit) */
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line15) != RESET) //Проверяем наличие запроса на прерывание
    {
/* Устанавливаем флаг */x
                FlagBTN = 0;
    }
    /* Снимаем бит запроса прерывания по линии EXTI_Line0 (pending bit) */
    EXTI_ClearITPendingBit(EXTI_Line15);
}

void SysTick_Handler(void)
{
    /* Декремент переменной, содержащей временной интервал */
    if (TimingDelay != 0x00) TimingDelay--;
}

void Delay(__IO uint32_t nCount)
{
    TimingDelay = nCount;
    while(TimingDelay != 0);
}

void Button_Configuration()
{

    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Подключаем WAKEUP_BUTTON GPIO к линии EXTI */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    /* Конфигурируем внешнюю линию_0 EXTI для WAKEUP_BUTTON прерываний*/
    EXTI_InitStructure.EXTI_Line = EXTI_Line0; // Вывод 0 подключен к нулевой линии
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Режим-прерывание
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //Детектирование фронта
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; // Включить
    EXTI_Init(&EXTI_InitStructure); //Инициализация посредством заполненной структуры
    /* Разрешаем прерывания WAKEUP_BUTTON_IRQn с низшим приоритетом */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Подключаем WAKEUP_BUTTON GPIO к линии EXTI */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
    /* Конфигурируем внешнюю линию_0 EXTI для WAKEUP_BUTTON прерываний*/
    EXTI_InitStructure.EXTI_Line = EXTI_Line15; // Вывод 0 подключен к нулевой линии
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Режим-прерывание
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //Детектирование фронта
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; // Включить
    EXTI_Init(&EXTI_InitStructure); //Инициализация посредством заполненной структуры
    /* Разрешаем прерывания WAKEUP_BUTTON_IRQn с низшим приоритетом */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void Func(void)
{
    SysTick_Config(SystemCoreClock / 1000);


    int i;
    /* Initialize Leds mounted on STM32 board */
    GPIO_InitTypeDef  GPIO_InitStructure;
    /* Initialize LED which connected to PC6,9, Enable the Clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_9|GPIO_Pin_7|GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    Button_Configuration();
    while (1)
    {
        if(FlagBTN != 0)
        {
            GPIOB->ODR ^= GPIO_Pin_  6;
            Delay(  200);
            GPIOB->ODR ^= GPIO_Pin_7;
            Delay(200);
            GPIOB->ODR ^= GPIO_Pin_8;
            Delay(200);
            GPIOB->ODR ^= GPIO_Pin_9;
            Delay(200);
        }
        else {
            GPIOB->ODR ^= GPIO_Pin_6;
            Delay(50);
            GPIOB->ODR ^= GPIO_Pin_7;
            Delay(50);
            GPIOB->ODR ^= GPIO_Pin_8;
            Delay(50);
            GPIOB->ODR ^= GPIO_Pin_9;
            Delay(50);
        }

    }
}
