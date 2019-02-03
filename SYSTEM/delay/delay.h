#ifndef _DELAY_H_
#define _DELAY_H_

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#ifdef __cplusplus
}
#endif

#endif

