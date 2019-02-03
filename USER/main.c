/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include <stdio.h>
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "mfrc522.h"





int main(void)
{
    uint8_t result[64] = {0};
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    USART1_Init(115200);
    LED_Init();

    PCD_ReadMultiRegister(FIFODataReg, 64, result, 0);

    for(i = 0; i < 64; i++)
    {
        printf("result[%d] = %X\r\n", i, result[i]);
    }
    

    while (1)
    {
        LED_Blink(1);
        printf("verison : %d\r\n", PCD_ReadRegister(VersionReg));
        delay_ms(500);
    }
    return 0;
}

void assert_failed(uint8_t *file, uint32_t line)
{
    while (1)
    {
        printf("File: %s, line: %d\r\n", file, line);
        delay_ms(500);
    }
}
