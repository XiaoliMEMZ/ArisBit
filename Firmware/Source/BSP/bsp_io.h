#ifndef __BSP_IO_H__
#define __BSP_IO_H__

#include "bsp_common.h"

#define IO_1_PORT	GPIOC
#define IO_2_PORT	GPIOC
#define IO_3_PORT	GPIOC
#define IO_4_PORT	GPIOC

#define IO_1_PIN	GPIO_Pin_3
#define IO_2_PIN	GPIO_Pin_2
#define IO_3_PIN	GPIO_Pin_1
#define IO_4_PIN	GPIO_Pin_0

#define IO_1_CLK	RCC_APB2Periph_GPIOC
#define IO_2_CLK	RCC_APB2Periph_GPIOC
#define IO_3_CLK    RCC_APB2Periph_GPIOC
#define IO_4_CLK	RCC_APB2Periph_GPIOC

#define IO_3_HIGH()  PCout(1) = 1
#define IO_3_LOW()   PCout(1) = 0

#define IO_4_HIGH()  PCout(0) = 1
#define IO_4_LOW()   PCout(0) = 0

void IOInit(void);
void SetIO(uint8_t IOPort, uint8_t State);
uint8_t GetIO(uint8_t IOPort);

#endif
