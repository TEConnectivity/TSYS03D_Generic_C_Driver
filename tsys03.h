/**
* Copyright (c) 2021 TE Connectivity. ALL RIGHTS RESERVED.
* Use of the Software is governed by the terms of the license agreement, if any,
* which accompanies or is included with the Software ("License Agreement").
***********************************************************************************************************************
* @file   tsys03.h
* @brief  tsys03 Temperature sensor driver header file
*/

#ifndef TSYS03_H
#define TSYS03_H

/**
* @defgroup GROUP_TSYS03 Tsys03 driver subcomponent
* This subcomponent manage TSYS03 sensor
*/

/****************************************************
* INCLUDES
****************************************************/

/****************************************************
* PUBLIC CONFIGURATION DEFINES
****************************************************/

/** TSYS03 I2C address selection */
#define TSYS03_ADDRESS  ((uint16_t)0x0040U)

/****************************************************
* PUBLIC DEFINES
*****************************************************/

/****************************************************
* PUBLIC TYPEDEFS
*****************************************************/

/** Enumeration type definition for command execution status */
//@{
typedef enum
{
	eTSYS03_COMMSTATUS_OK,					/**< No communication error  */
	eTSYS03_COMMSTATUS_NO_RESPONSE,			/**< Tsys03 not responded    */
	eTSYS03_COMMSTATUS_COMMAND_REJECTED,	/**< Tsys03 command rejected */
	eTSYS03_COMMSTATUS_READ_ERROR,			/**< Tsys03 read error       */
	eTSYS03_COMMSTATUS_DATA_CRC_ERROR,		/**< Tsys03 data crc error   */
	eTSYS03_COMMSTATUS_NVM_ERROR            /**< Tsys03 NVM error   */
}eTsys03_CommStatus_t;
//@}

/** Type definition for Tsys03 Serial number */
typedef uint32_t Tsys03_SerialNumber_t;

/****************************************************
* PUBLIC FUNCTION PROTOTYPES
*****************************************************/
void vTsys03_init(void);
eTsys03_CommStatus_t eTsys03_Reset(void);
eTsys03_CommStatus_t eTsys03_LaunchMeasurements(void);
eTsys03_CommStatus_t eTsys03_ReadSerialNumber(Tsys03_SerialNumber_t * const ptSerialNumber);
eTsys03_CommStatus_t eTsys03_ReadTemperature(float32_t * const pf32Temperature);
eTsys03_CommStatus_t eTsys03_UpdateI2CAddress(const uint8_t u8I2cAddress);

/****************************************************
* Unit Test Wrappers
****************************************************/
#ifdef UNIT_TEST
bool_t UT_bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize);
eTsys03_CommStatus_t UT_eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand);
eTsys03_CommStatus_t UT_eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size);
float32_t UT_f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc);
eTsys03_CommStatus_t UT_eUnlockNVM(void);
eTsys03_CommStatus_t UT_eWriteNVMI2C(const uint8_t u8Address);
eTsys03_CommStatus_t UT_eReadNVMI2C(uint8_t * const pu8ReadAddress);
#endif

#endif /* TSYS03_H */
