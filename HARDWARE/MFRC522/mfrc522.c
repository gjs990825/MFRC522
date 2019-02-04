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
    0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D};
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
    0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};
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
    0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};
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
    0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62};

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
    // 		hardReset = 1;
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
    PCD_WriteRegister(FIFOLevelReg, 0x80);           // flush the FIFO buffer
    PCD_WriteMultiRegister(FIFODataReg, 25, ZEROES); // write 25 bytes of 00h to FIFO
    PCD_WriteRegister(CommandReg, PCD_Mem);          // transfer to internal buffer

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

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!
//For more details about power control, refer to the datasheet - page 33 (8.6)

void PCD_SoftPowerDown(void)
{                                               //Note : Only soft power down mode is available throught software
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register
    val |= (1 << 4);                            // set PowerDown bit ( bit 4 ) to 1
    PCD_WriteRegister(CommandReg, val);         //write new value to the command register
}

void PCD_SoftPowerUp(void)
{
    uint16_t timeout = 500;                     // set timeout to 500 ms
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register
    val &= ~(1 << 4);                           // set PowerDown bit ( bit 4 ) to 0
    PCD_WriteRegister(CommandReg, val);         //write new value to the command register

    // wait until PowerDown bit is cleared (this indicates end of wake up procedure)
    while (timeout--)
    {
        delay_ms(1);
        val = PCD_ReadRegister(CommandReg); // Read state of the command register
        if (!(val & (1 << 4)))
        {          // if powerdown bit is 0
            break; // wake up procedure is finished
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(uint8_t *sendData,  ///< Pointer to the data to transfer to the FIFO.
                              uint8_t sendLen,    ///< Number of bytes to transfer to the FIFO.
                              uint8_t *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
                              uint8_t *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                              uint8_t *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
                              uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                              _Bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
)
{
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(uint8_t command,    ///< The command to execute. One of the PCD_Command enums.
                                   uint8_t waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                   uint8_t *sendData,  ///< Pointer to the data to transfer to the FIFO.
                                   uint8_t sendLen,    ///< Number of bytes to transfer to the FIFO.
                                   uint8_t *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
                                   uint8_t *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                   uint8_t *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                   uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                   _Bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
)
{
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    PCD_WriteRegister(CommandReg, PCD_Idle);           // Stop any active command.
    PCD_WriteRegister(ComIrqReg, 0x7F);                // Clear all seven interrupt request bits
    PCD_WriteRegister(FIFOLevelReg, 0x80);             // FlushBuffer = 1, FIFO initialization
    PCD_WriteMultiRegister(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
    PCD_WriteRegister(BitFramingReg, bitFraming);      // Bit adjustments
    PCD_WriteRegister(CommandReg, command);            // Execute the command
    if (command == PCD_Transceive)
    {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86Œºs.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 2000; i > 0; i--)
    {
        uint8_t n = PCD_ReadRegister(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq)
        { // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01)
        { // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0)
    {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13)
    { // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    uint8_t _validBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen)
    {
        uint8_t n = PCD_ReadRegister(FIFOLevelReg); // Number of bytes in the FIFO
        if (n > *backLen)
        {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                        // Number of bytes returned
        PCD_ReadMultiRegister(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07;    // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits)
        {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08)
    { // CollErr
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC)
    {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4)
        {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*backLen < 2 || _validBits != 0)
        {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK)
        {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
        {
            return STATUS_CRC_WRONG;
        }
    }

    return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_RequestA(uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                         uint8_t *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
)
{
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_WakeupA(uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                        uint8_t *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
)
{
    return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_REQA_or_WUPA(uint8_t command,     ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
                             uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                             uint8_t *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
)
{
    uint8_t validBits;
    StatusCode status;

    if (bufferATQA == NULL || *bufferSize < 2)
    { // The ATQA response is 2 bytes long.
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                           // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
    if (status != STATUS_OK)
    {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0)
    { // ATQA must be exactly 16 bits.
        return STATUS_ERROR;
    }
    return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_Select(Uid *uid,      ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
                       uint8_t validBits ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
)
{
    _Bool uidComplete;
    _Bool selectDone;
    _Bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    StatusCode result;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;                // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];               // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t bufferUsed;              // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign;                 // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;              // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
    //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
    //		Byte 3: UID-data
    //		Byte 4: UID-data
    //		Byte 5: UID-data
    //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
    //		Byte 7: CRC_A
    //		Byte 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
    //		========	=============	=====	=====	=====	=====
    //		 4 bytes		1			uid0	uid1	uid2	uid3
    //		 7 bytes		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 bytes		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Sanity checks
    if (validBits > 80)
    {
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = 0;
    while (!uidComplete)
    {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel)
        {
        case 1:
            buffer[0] = PICC_CMD_SEL_CL1;
            uidIndex = 0;
            useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
            break;

        case 2:
            buffer[0] = PICC_CMD_SEL_CL2;
            uidIndex = 3;
            useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
            break;

        case 3:
            buffer[0] = PICC_CMD_SEL_CL3;
            uidIndex = 6;
            useCascadeTag = 0; // Never used in CL3.
            break;

        default:
            return STATUS_INTERNAL_ERROR;
            break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0)
        {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag)
        {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy)
        {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes)
            {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++)
            {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag)
        {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = 0;
        while (!selectDone)
        {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32)
            { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            }
            else
            { // This is an ANTICOLLISION.
                //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
                index = 2 + count;                     // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                          // Having a separate variable is overkill. But it makes the next line easier to read.
            PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
            if (result == STATUS_COLLISION)
            {                                                    // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20)
                {                            // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0)
                {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits)
                { // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = currentLevelKnownBits % 8; // The bit to modify
                checkBit = (currentLevelKnownBits - 1) % 8;
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << checkBit);
            }
            else if (result != STATUS_OK)
            {
                return result;
            }
            else
            { // STATUS_OK
                if (currentLevelKnownBits >= 32)
                {                      // This was a SELECT.
                    selectDone = 1; // No more anticollision
                                       // We continue below outside the while.
                }
                else
                { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++)
        {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0)
        { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK)
        {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
        {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04)
        { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else
        {
            uidComplete = 1;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_HaltA(void)
{
    StatusCode result;
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK)
    {
        return result;
    }

    // Send the command.
    // The standard says:
    //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //		HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, 0);
    if (result == STATUS_TIMEOUT)
    {
        return STATUS_OK;
    }
    if (result == STATUS_OK)
    { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
} // End PICC_HaltA()
