#include "AX12.h"
#include "mfrc522.h"

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t finish=1;	
	
uint8_t msgReceiveComplete = 0;
uint8_t getInstructionFlag = 0;
uint8_t instruction = 0;

extern uint8_t dataWrite[5];
extern uint8_t dataRead[5];

void ax12ReceivedMsgProcess(uint8_t *msg)
{
	instruction = rx_buffer[4];
	dataWrite[0] = rx_buffer[5];
	dataWrite[1] = rx_buffer[6];
	dataWrite[2] = rx_buffer[7];
	dataWrite[3] = rx_buffer[8];
	dataWrite[4] = rx_buffer[9];
}

