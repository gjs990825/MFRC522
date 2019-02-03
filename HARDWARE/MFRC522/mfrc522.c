#include "mfrc522.h"
#include "stdio.h"

uint8_t FIFO_SIZE = 64;
uint8_t UNUSED_PIN = UINT8_MAX;

// Member variables
Uid uid; // Used by PICC_ReadCardSerial().

void MFRC522_PinConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MFRC522_SS_RST_RCC, ENABLE);

    //≈‰÷√ SS°¢RST “˝Ω≈
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = MFRC522_SS_PIN | MFRC522_RST_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(MFRC522_SS_RST_PORT, &GPIO_InitStructure);

    MFRC522_RST_H();
    MFRC522_SS_H();
}

// Init MFRC522 hardware
void MFRC522_Init(void)
{
    HardSPI_Init();
    MFRC522_PinConfig();
}

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                       uint8_t value     ///< The value to write.
)
{
    MFRC522_SS_L(); // Select slave
    SPI_RW(reg);    // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    SPI_RW(value);
    MFRC522_SS_H(); // Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteMultiRegister(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                            uint8_t count,    ///< The number of bytes to write to the register
                            uint8_t *values   ///< The values to write. Byte array.
)
{
    uint8_t index;
    MFRC522_SS_L(); // Select slave
    SPI_RW(reg);    // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    for (index = 0; index < count; index++)
    {
        SPI_RW(values[index]);
    }
    MFRC522_SS_H(); // Release slave again
} // End PCD_WriteMultiRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegister(PCD_Register reg ///reg < The register to read from. One of the PCD_Register enums.
)
{
    uint8_t value;
    MFRC522_SS_L();     // Select slave
    SPI_RW(0x80 | reg); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = SPI_RW(0);  // Read the value back. Send 0 to stop reading.
    MFRC522_SS_H();     // Release slave again
    return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadMultiRegister(PCD_Register reg, ///< The register to read from. One of the PCD_Register enums.
                           uint8_t count,    ///< The number of bytes to read
                           uint8_t *values,  ///< Byte array to store the values in.
                           uint8_t rxAlign   ///< Only bit positions rxAlign..7 in values[0] are updated.
)
{
    uint8_t mask, value, address, index;

    if (count == 0)
    {
        return;
    }
    // printf("reading %d bytes from register\r\n", count);
    address = 0x80 | reg; // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    index = 0;            // Index in values array.
    MFRC522_SS_L();       // Select slave
    count--;              // One read is performed outside of the loop
    SPI_RW(address);      // Tell MFRC522 which address we want to read
    if (rxAlign)          // Only update bit positions rxAlign..7 in values[0]
    {
        // Create bit mask for bit positions rxAlign..7
        mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        value = SPI_RW(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }
    while (index < count)
    {
        values[index] = SPI_RW(address); // Read value and tell that we want to read the same address again.
        index++;
    }
    values[index] = SPI_RW(0); // Read the final byte. Send 0 to stop reading.
    MFRC522_SS_H();            // Release slave again
} // End PCD_ReadMultiRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(PCD_Register reg, ///< The register to update. One of the PCD_Register enums.
                            uint8_t mask      ///< The bits to set.
)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp | mask); // set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(PCD_Register reg, ///< The register to update. One of the PCD_Register enums.
                              uint8_t mask      ///< The bits to clear.
)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask)); // clear bit mask
} // End PCD_ClearRegisterBitMask()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(uint8_t *data,  ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                            uint8_t length, ///< In: The number of bytes to transfer.
                            uint8_t *result ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
)
{
    PCD_WriteRegister(CommandReg, PCD_Idle);           // Stop any active command.
    PCD_WriteRegister(DivIrqReg, 0x04);                // Clear the CRCIRq interrupt request bit
    PCD_WriteRegister(FIFOLevelReg, 0x80);             // FlushBuffer = 1, FIFO initialization
    PCD_WriteMultiRegister(FIFODataReg, length, data); // Write data to the FIFO
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);        // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73Œºs.
    // TODO check/modify for other architectures than Arduino Uno 16bit

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    for (uint16_t i = 5000; i > 0; i--)
    {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = PCD_ReadRegister(DivIrqReg);
        if (n & 0x04)
        {                                            // CRCIRq bit set - calculation done
            PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = PCD_ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        }
    }
    // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()
