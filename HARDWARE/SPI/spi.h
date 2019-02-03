#ifndef _SPI_H_
#define _SPI_H_

#include "sys.h"

#define SPI_PERIPH SPI1
#define SPI_PERIPH_RCC RCC_APB2Periph_SPI1
#define SPI_GPIO_PORT GPIOA
#define SPI_GPIO_RCC RCC_APB2Periph_GPIOA
#define SPI_SCK_PIN GPIO_Pin_5
#define SPI_MISO_PIN GPIO_Pin_6
#define SPI_MOSI_PIN GPIO_Pin_7

#define SPI_SS_PORT GPIOA
#define SPI_SS_RCC RCC_APB2Periph_GPIOA
#define SPI_SS_PIN GPIO_Pin_3


void 	HardSPI_Init(void);
uint8_t SPI_RW(uint8_t dat);

static inline void SPI_SS_H(void)
{
	GPIO_SetBits(SPI_SS_PORT, SPI_SS_PIN);
}

static inline void SPI_SS_L(void)
{
	GPIO_ResetBits(SPI_SS_PORT, SPI_SS_PIN);
}
		 
#endif //_SPI_H_

