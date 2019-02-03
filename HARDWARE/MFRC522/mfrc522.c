#include "mfrc522.h"
#include "stdio.h"
#include "delay.h"

uint8_t FIFO_SIZE = 64;
uint8_t UNUSED_PIN = UINT8_MAX;

// Member variables
Uid uid; // Used by PICC_ReadCardSerial().


// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const uint8_t MFRC522_firmware_referenceV0_0[] = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV1_0[] = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV2_0[] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const uint8_t FM17522_firmware_reference[] = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};


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

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init(void)
{
    _Bool hardReset = 0;

    HardSPI_Init();
    MFRC522_PinConfig();

    //do not select the slave yet
    MFRC522_SS_H();

    // NOT USED YET
    // // If a valid pin number has been set, pull device out of power down / reset state.
    // if (_resetPowerDownPin != UNUSED_PIN) {
    // 	// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
    // 	pinMode(_resetPowerDownPin, INPUT);

    // 	if (digitalRead(_resetPowerDownPin) == LOW) {	// The MFRC522 chip is in power down mode.
    // 		pinMode(_resetPowerDownPin, OUTPUT);		// Now set the resetPowerDownPin as digital output.
    // 		digitalWrite(_resetPowerDownPin, LOW);		// Make shure we have a clean LOW state.
    // 		delayMicroseconds(2);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2Œºsl
    // 		digitalWrite(_resetPowerDownPin, HIGH);		// Exit power down mode. This triggers a hard reset.
    // 		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74Œºs. Let us be generous: 50ms.
    // 		delay(50);
    // 		hardReset = true;
    // 	}
    // }

    if (!hardReset)
    { // Perform a soft reset if we haven't triggered a hard reset above.
        PCD_Reset();
    }

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25Œºs.
    PCD_WriteRegister(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);

    PCD_WriteRegister(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn();                   // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset(void)
{
    uint8_t count = 0;
    PCD_WriteRegister(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74Œºs. Let us be generous: 50ms.
    do
    {
        // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
        delay_ms(50);
    } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn(void)
{
    uint8_t value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03)
    {
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff(void)
{
    PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t PCD_GetAntennaGain(void)
{
    return PCD_ReadRegister(RFCfgReg) & (0x07 << 4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain(uint8_t mask)
{
    if (PCD_GetAntennaGain() != mask)
    {                                                         // only bother if there is a change
        PCD_ClearRegisterBitMask(RFCfgReg, (0x07 << 4));      // clear needed to allow 000 pattern
        PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07 << 4)); // only set RxGain[2:0] bits
    }
} // End PCD_SetAntennaGain()

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
_Bool PCD_PerformSelfTest(void)
{
    const uint8_t *reference;
    uint8_t ZEROES[25] = {0x00};
    uint8_t result[64];
    uint8_t version, n, i;

    // This follows directly the steps outlined in 16.1.1
    // 1. Perform a soft reset.
    PCD_Reset();

    // 2. Clear the internal buffer by writing 25 bytes of 00h
    PCD_WriteRegister(FIFOLevelReg, 0x80);      // flush the FIFO buffer
    PCD_WriteMultiRegister(FIFODataReg, 25, ZEROES); // write 25 bytes of 00h to FIFO
    PCD_WriteRegister(CommandReg, PCD_Mem);     // transfer to internal buffer

    // 3. Enable self-test
    PCD_WriteRegister(AutoTestReg, 0x09);

    // 4. Write 00h to FIFO buffer
    PCD_WriteRegister(FIFODataReg, 0x00);

    // 5. Start self-test by issuing the CalcCRC command
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);

    // 6. Wait for self-test to complete
    for (i = 0; i < 0xFF; i++)
    {
        // The datasheet does not specify exact completion condition except
        // that FIFO buffer should contain 64 bytes.
        // While selftest is initiated by CalcCRC command
        // it behaves differently from normal CRC computation,
        // so one can't reliably use DivIrqReg to check for completion.
        // It is reported that some devices does not trigger CRCIRq flag
        // during selftest.
        n = PCD_ReadRegister(FIFOLevelReg);
        if (n >= 64)
        {
            break;
        }
    }
    PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.

    // 7. Read out resulting 64 bytes from the FIFO buffer.
    PCD_ReadMultiRegister(FIFODataReg, 64, result, 0);

    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    PCD_WriteRegister(AutoTestReg, 0x00);

    // Determine firmware version (see section 9.3.4.8 in spec)
    version = PCD_ReadRegister(VersionReg);

    // printf("Firmvare version :0x%X\r\n", version);

    // Pick the appropriate reference values
    switch (version)
    {
    case 0x88: // Fudan Semiconductor FM17522 clone
        reference = FM17522_firmware_reference;
        break;
    case 0x90: // Version 0.0
        reference = MFRC522_firmware_referenceV0_0;
        break;
    case 0x91: // Version 1.0
        reference = MFRC522_firmware_referenceV1_0;
        break;
    case 0x92: // Version 2.0
        reference = MFRC522_firmware_referenceV2_0;
        break;
    default:      // Unknown version
        return 0; // abort test
    }

    // Verify that the results match up to our expectations
    for (uint8_t i = 0; i < 64; i++)
    {
        if (result[i] != reference[i])
        {
            return 0;
        }
    }

    // Test passed; all is good.
    return 1;
} // End PCD_PerformSelfTest()
