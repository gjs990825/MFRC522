#include "mfrc522.h"

uint8_t FIFO_SIZE = 64;
uint8_t UNUSED_PIN = UINT8_MAX; 

// Member variables
Uid uid;								// Used by PICC_ReadCardSerial().
	

uint8_t PCD_ReadRegister(PCD_Register reg)
{
    uint8_t value;
    SPI_SS_L();         // Select slave
    SPI_RW(0x80 | reg); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = SPI_RW(0);  // Read the value back. Send 0 to stop reading.
    SPI_SS_H();         // Release slave again
    return value;
} // End PCD_ReadRegister()























