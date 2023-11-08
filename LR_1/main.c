#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
void GPIO_Blink(void);

int main(void)
{
    int i;
    SystemInit();
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(0x00000008 | 0x00000004, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    unsigned char Flag=0;
    unsigned char Flag1=0;
    unsigned char Flag2=0;
    unsigned char Flag12=0;

    unsigned char Flag=0;
    unsigned char Flag1=0;

    while (1)
    {
        Flag2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
        Flag12 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
        if (Flag2 && !FHold)
            FHold= !FHold;
        if (Flag12 && !FHold)
            FHold= !FHold;

        if (!Flag && Flag2) FHold !=FHold;
        if (!Flag2 && Flag12) FHold1 !=FHold;

        GPIO_WriteBit(GPIOB, GPIO_Pin_9, FHold);
        GPIO_WriteBit(GPIOB, GPIO_Pin_8, FHold);

        Flag = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
        Flag1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);

        For (int i = 0; I < 1000000; i++);
    }
}
