#ifndef _SPI_H_
#define _SPI_H_

#include "sys.h"

#define SPI_PERIPH SPI2
#define SPI_GPIO_PORT GPIOB
#define SPI_GPIO_RCC RCC_APB2Periph_GPIOB
#define SPI_SCK_PIN GPIO_Pin_13
#define SPI_MISO_PIN GPIO_Pin_14
#define SPI_MOSI_PIN GPIO_Pin_15


void 	HardSPI_Init(void);
uint8_t SPI_RW(uint8_t dat);

	 
#endif //_SPI_H_

