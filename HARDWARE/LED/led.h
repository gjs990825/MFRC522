#ifndef _LED_H_
#define _LED_H_

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED0 PBout(5)
#define LED1 PEout(5)
//#define LED2 PBout(13)

void LED_Init(void);
void LED_Blink(uint8_t LEDn);

#ifdef __cplusplus
}
#endif

#endif

