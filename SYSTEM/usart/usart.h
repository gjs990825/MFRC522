#ifndef _USART_H_
#define _USART_H_


#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define USART1_ENABLE_INTERRUPT 0  //�ж�ʹ��(ʹ��scanf��������ʹ�ܣ����ܻᶪʧ����)
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���

extern char UART_read(void);
extern void UART_write(char);

void USART1_Init(uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif //_USART_H_


