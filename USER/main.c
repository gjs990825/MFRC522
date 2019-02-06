/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include <stdio.h>
#include <stdlib.h>
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "mfrc522.h"

int main(void)
{
    MIFARE_Key key;
    // uint8_t block, len;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    USART1_Init(115200);
    LED_Init();

    PCD_Init();

    printf("Read personal data on a MIFARE PICC:\r\n");

    while (1)
    {
        for (uint8_t i = 0; i < 6; i++)
            key.keyByte[i] = 0xFF;

        if (!PICC_IsNewCardPresent())
        {
            continue;
        }

        if (!PICC_ReadCardSerial())
        {
            continue;
        }

        printf("**Card Detected:**\r\n");

        PICC_DumpDetailsToSerial(&uid);

        PICC_HaltA();
        PCD_StopCrypto1();


        LED_Blink(1);
        // printf("verison : %d\r\n", PCD_ReadRegister(VersionReg));
        delay_ms(50);
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
