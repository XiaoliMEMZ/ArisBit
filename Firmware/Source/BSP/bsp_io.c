#include "bsp_io.h"

void IOInit(void)
{
    // Initialize IO_1
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(IO_1_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = IO_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(IO_1_PORT, &GPIO_InitStructure);
    
    // Initialize IO_2
    RCC_APB2PeriphClockCmd(IO_2_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = IO_2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(IO_2_PORT, &GPIO_InitStructure);

    // Initialize IO_3
    RCC_APB2PeriphClockCmd(IO_3_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = IO_3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IO_3_PORT, &GPIO_InitStructure);
    GPIO_SetBits(IO_3_PORT, IO_3_PIN);
    
    // Initialize IO_4
    RCC_APB2PeriphClockCmd(IO_4_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = IO_4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IO_4_PORT, &GPIO_InitStructure);
    GPIO_SetBits(IO_4_PORT, IO_4_PIN);

    IO_3_LOW();
    IO_4_LOW();
}

void SetIO(uint8_t IOPort, uint8_t State)
{
    if (IOPort == 3)
    {
        if (State)
            IO_3_HIGH();
        else
            IO_3_LOW();
    }
    else if (IOPort == 4)
    {
        if (State)
            IO_4_HIGH();
        else
            IO_4_LOW();
    }
    
}

uint8_t GetIO(uint8_t IOPort)
{
    uint8_t State = 0;
    if (IOPort == 1)
    {
        State = GPIO_ReadInputDataBit(IO_1_PORT, IO_1_PIN);
    }
    else if (IOPort == 2)
    {
        State = GPIO_ReadInputDataBit(IO_2_PORT, IO_2_PIN);
    }
    
    
    return State;

}
