/**
* Copyright (c) 2021 TE Connectivity. ALL RIGHTS RESERVED.
* Use of the Software is governed by the terms of the license agreement, if any,
* which accompanies or is included with the Software ("License Agreement").
***********************************************************************************************************************
* @file   tsys03.c
* @brief  tsys03 Temperature sensor driver file
* For details on programming, refer to tsys03 datasheet:
* www.te.com/usa-en/product-CAT-DTS0001.html
*/


/** @addtogroup GROUP_TSYS03
* @{
*/

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************
* INCLUDES
****************************************************/

/* 
* The header "basic_types.h" contain basic types definitions.
* According to project integration, this header can be used or replaced.
* typedef int               bool_t;      Boolean
* typedef signed char       int8_t;      8-Bit Signed
* typedef unsigned char     uint8_t;     8-Bit Unsigned
* typedef signed short      int16_t;    16-Bit Signed
* typedef unsigned short    uint16_t;   16-Bit Unsigned
* typedef signed long       int32_t;    32-Bit Signed
* typedef unsigned long     uint32_t;   32-Bit Unsigned
* typedef float             float32_t;  32-Bit Float
* #define B_TRUE            (1U) Boolean true definition
* #define B_FALSE           (0U) Boolean false definition
*/
#include "basic_types.h"

/*
* The header "i2c.h" has to be implemented for your own platform to
* conform the following protocol :
*
* Enumeration type definition for i2c returned status
* typedef enum
* {
*     eI2C_STATUS_OK              = 0x00, Exchange complete without error
*     eI2C_STATUS_ERR_OVERFLOW    = 0x01, i2c driver full
*     eI2C_STATUS_ERR_ADDRESS     = 0x02, device not respond on address
*     eI2C_STATUS_ERR_TRANSFER    = 0x03  transfer error. Unexpected NACK
* }eI2c_StatusCode_t;
*
* typedef struct
* {
*     uint16_t u16Address;    Address to slave device
*     uint16_t u16DataLength; Length of data array
*     uint8_t *pu8Data;       Data array containing all data to be transferred
* }xI2c_MasterPacket_t;
*
* Initialise Driver and sub layer
* void i2c_master_init(void);
* 
* Read data from i2c device
* eI2c_StatusCode_t tI2c_MasterReadPacket(xI2c_MasterPacket_t *const ptPacket);
* 
* write data to i2c device
* eI2c_StatusCode_t tI2c_MasterWritePacket(xI2c_MasterPacket_t *const ptPacket);
* 
* Write data to i2C device without closing link with device. Mainly used as first step of a i2c read process
* eI2c_StatusCode_t tI2c_MasterWritePacketNoStop(xI2c_MasterPacket_t *const ptPacket);
*/
#include "i2c.h"

/** tsys03 driver header */
#include "tsys03.h"

/****************************************************
* DEFINES
****************************************************/
/** TSYS03 Command: Perform a reset of Tsys03 */
#define U8_COMMAND_RESET                ((uint8_t) 0x1EU)
/** TSYS03 Command: Read TSYS03 Serial Number */
#define U8_COMMAND_READ_SN              ((uint8_t) 0x0AU)
/** TSYS03 Command: Request Temperature */
#define U8_COMMAND_START_CONVERSION     ((uint8_t) 0x46U)
/** TSYS03 Command: Read Temperature measurement result */
#define U8_COMMAND_READ_ADC_T        	((uint8_t) 0x00U)
/** TSYS03 Command: Unlock NVM */
#define U8_COMMAND_UNLOCK_NVM           ((uint8_t) 0x12U)
/** TSYS03 Command: Write NVM I2C ADD */
#define U8_COMMAND_WRITE_NVM_I2C_ADD    ((uint8_t) 0xAAU)
/** TSYS03 Command: Read NVM I2C ADD */
#define U8_COMMAND_READ_NVM_I2C_ADD     ((uint8_t) 0xCAU)

/** I2C protocol data size: Read Serial Number request 3 bytes + 1 crc byte */
#define U8_READ_SN_I2C_SIZE				((uint8_t) 0x04U)
/** I2C protocol data size: Read Temperature request 2 bytes + 1 crc byte */
#define U8_READ_T_I2C_SIZE				((uint8_t) 0x03U)
/** I2C protocol data size: Write Command 1 byte no crc byte */
#define U16_CMD_I2C_SIZE                ((uint16_t)0x0001U)
/** I2C protocol data size: Unlock NVM 1 byte no crc byte */
#define U16_UNLOCK_NVM_I2C_SIZE         ((uint16_t)0x0001U)
/** I2C protocol data size: Write NVM 7 byte no crc byte */
#define U16_WRITE_NVM_I2C_SIZE          ((uint16_t)0x0007U)
/** I2C protocol data size: Write NVM 6 byte no crc byte */
#define U16_READ_NVM_I2C_SIZE          ((uint16_t)0x0006U)
/** I2C protocol READ NVM I2C address data index */
#define U8_READ_NVM_I2C_ADD_INDEX      ((uint8_t)0x04U)


/** Temperature computation coefficients */
//@{
#define F32_TEMPERATURE_COEFF_MUL		((float32_t)165.00f)
#define F32_TEMPERATURE_COEFF_ADD		((float32_t)-40.00f)
#define F32_TEMPERATURE_COEFF_DIV		((float32_t)((1UL << 16UL) - 1UL))
//@}

/** Crc polynomial form: x^8 + x^5 + x^4 + 1 */
#define U32_CRC_3_BYTES_POLYNOM         ((uint32_t)0x98800000UL)
/** Crc computation MSB pattern */
#define U32_CRC_3_BYTES_MSB             ((uint32_t)0x80000000UL)
/** Crc computation MSB threshold value */
#define U32_CRC_MSB_THRESHOLD           ((uint32_t)0x00000080UL)
/** Crc computation data mask */
#define U32_CRC_3_BYTES_MASK            ((uint32_t)0xFF800000UL)
/** Mask to be applied on updated result to extract computed CRC */
#define U32_CRC_RESULT_MASK             ((uint32_t)0x000000FFUL)

/** Bit manipulation symbols: 8 Bit Shifting value for storage handling */
#define U8_BYTE_SHIFT                   ((uint8_t)  8U)
/** Bit manipulation symbols: 16 Bit Shifting value for storage handling */
#define U8_SHORT_SHIFT                  ((uint8_t) 16U)

/** I2C address Lower Limit */
#define U8_I2C_ADDRESS_LLIMIT           ((uint8_t)0x40U)
/** I2C address Upper Limit */
#define U8_I2C_ADDRESS_ULIMIT           ((uint8_t)0x7EU)
/** I2C address mask */
#define U8_I2C_ADDRESS_MASK             ((uint8_t)0x3FU)

/****************************************************
* TYPEDEFS
****************************************************/

/****************************************************
* VARIABLES
****************************************************/
/** Variable use to hold TSYS03 I2C address */
static uint16_t u16TSYS03Address = TSYS03_ADDRESS;

/****************************************************
* CONSTANTS
****************************************************/
 
/****************************************************
* PROTOTYPES
****************************************************/
static bool_t				bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize);
static eTsys03_CommStatus_t eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand);
static eTsys03_CommStatus_t eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size);
static float32_t			f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc);
static eTsys03_CommStatus_t eUnlockNVM(void);
static eTsys03_CommStatus_t eWriteNVMI2C(const uint8_t u8Address);
static eTsys03_CommStatus_t eReadNVMI2C(uint8_t * const pu8ReadAddress);
/****************************************************
* PRIVATE FUNCTIONS
****************************************************/

/**
*@brief         Perform a crc check on a buffer containing [data + crc]
*@param[in]     au8Buffer[]: Buffer to check
*@param[in,out] u8BufferSize: size of the data + crc
*@warning       function available for One, Two and Tree byte data size.
*/
static bool_t bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize)
{
    bool_t   bStatus       = B_FALSE;
    uint8_t  u8ExpectedCrc = (uint8_t)0x00U;
    uint32_t u32Result     = (uint32_t)0x00000000UL;
    uint32_t u32Polynom    = U32_CRC_3_BYTES_POLYNOM; 
    uint32_t u32Msb        = U32_CRC_3_BYTES_MSB;
    uint32_t u32Mask       = U32_CRC_3_BYTES_MASK;
    
    if (u8BufferSize == U8_READ_SN_I2C_SIZE)
    {
        u8ExpectedCrc = au8Buffer[U8_READ_SN_I2C_SIZE - 1U];
        u32Result  = (uint32_t)au8Buffer[0];
        u32Result <<= U8_BYTE_SHIFT;
        u32Result |= (uint32_t)au8Buffer[1];
        u32Result <<= U8_BYTE_SHIFT;
        u32Result |= (uint32_t)au8Buffer[2];
        u32Result <<= U8_BYTE_SHIFT;
        //au8Buffer[3] (U8_READ_SN_I2C_SIZE - 1U) contain expected crc. u32Result LSB (byte) shall cleared
    }
    else if (u8BufferSize == U8_READ_T_I2C_SIZE)
    {
        u8ExpectedCrc = au8Buffer[U8_READ_T_I2C_SIZE - 1U];
        u32Result  = (uint32_t)au8Buffer[0] << U8_SHORT_SHIFT;
        u32Result |= (uint32_t)au8Buffer[1] <<  U8_BYTE_SHIFT;
        //au8Buffer[2] (U8_READ_T_I2C_SIZE - 1U) contains expected crc. u32Result LSB (byte) shall cleared
    }
    else
    {
    	// unexpected size: generate crc error
        u8ExpectedCrc = 0U;
        u32Result = U32_CRC_RESULT_MASK;
        u32Msb = U32_CRC_MSB_THRESHOLD;
    }
    
    while(u32Msb != U32_CRC_MSB_THRESHOLD)
    {
        // Check if msb of current value is 1 and apply XOR mask
        if((u32Result & u32Msb) != 0UL)
        {
            u32Result = ((u32Result ^ u32Polynom) & u32Mask) | (u32Result & ~u32Mask);
        }
        
        // Shift by one
        u32Msb     >>= 1UL;
        u32Mask    >>= 1UL;
        u32Polynom >>= 1UL;
    }
    
    if(u8ExpectedCrc == (uint8_t)(U32_CRC_RESULT_MASK & u32Result))
    {
        bStatus = B_TRUE;
    }
    else
    {
        bStatus = B_FALSE;
    }
    
return (bStatus);
}

/**
*@brief			Perform a Write command to Tsys03
*@param[in]		u8Command: Command to be transmitted
*@param[in]		bIsReadCommand: Is the Command is a read command
*@return		tCommStatus: execution status of the command
*				- eTSYS03_COMMSTATUS_OK: command executed as expected
*				- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*				- eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
static eTsys03_CommStatus_t eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand)
{
	eTsys03_CommStatus_t	tCommStatus;
	eI2c_StatusCode_t		eI2cStatus;
	uint8_t					u8Data;
	xI2c_MasterPacket_t     tPacket;

	// prepare packet
	tPacket.u16Address		= u16TSYS03Address;
	tPacket.u16DataLength	= U16_CMD_I2C_SIZE;
	tPacket.pu8Data		    = &u8Data;
	
    // Get the command
    u8Data = u8Command;

	// send the command
	if(B_FALSE == bIsReadCommand)
	{
		eI2cStatus = tI2c_MasterWritePacket(&tPacket);
	}
	else
	{
		eI2cStatus = tI2c_MasterWritePacketNoStop(&tPacket);
	}

	if (eI2C_STATUS_ERR_OVERFLOW == eI2cStatus)
	{
		tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_ADDRESS == eI2cStatus)
	{
		tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_TRANSFER == eI2cStatus)
	{
		tCommStatus = eTSYS03_COMMSTATUS_COMMAND_REJECTED;
	}
	else
	{
		tCommStatus = eTSYS03_COMMSTATUS_OK;
	}

	return (tCommStatus);
}

/**
*@brief			Perform a Read command to Tsys03
*@param[in]		au8Buffer[]: Buffer to be filled
*@param[in]		u8Size: Size of the buffer
*@return		tCommStatus: execution status of the command
*				- eTSYS03_COMMSTATUS_OK: command executed as expected
*				- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*				- eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*/
static eTsys03_CommStatus_t eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size)
{
	eTsys03_CommStatus_t	tCommStatus;
	eI2c_StatusCode_t		eI2cStatus;
	xI2c_MasterPacket_t     tPacket;
	
	// prepare packet
	tPacket.u16Address		= u16TSYS03Address;
	tPacket.u16DataLength	= (uint16_t) u8Size;
	tPacket.pu8Data		    = au8Buffer;

	// read data
	eI2cStatus = tI2c_MasterReadPacket(&tPacket);

	if (eI2C_STATUS_ERR_OVERFLOW == eI2cStatus)
	{
		tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_ADDRESS == eI2cStatus)
	{
		tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_TRANSFER == eI2cStatus)
	{
		tCommStatus = eTSYS03_COMMSTATUS_READ_ERROR;
	}
	else
	{
		tCommStatus = eTSYS03_COMMSTATUS_OK;
	}

	return (tCommStatus);
}

/**
*@brief			Convert 16-bits ADC temperature from Tsys03 to floating point degree celsius
*@param[in]		u16TemperatureAdc: Raw ADC temperature from Tsys03
*@return		f32Temperature: Temperature in degree celsius
*/
static float32_t f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc)
{
	float32_t f32Temperature;

	//Temperature = Offset + Coef * AdcValue / AdcResolution
	f32Temperature  = (float32_t)u16TemperatureAdc;
	f32Temperature *= F32_TEMPERATURE_COEFF_MUL;
	f32Temperature /= F32_TEMPERATURE_COEFF_DIV;
	f32Temperature += F32_TEMPERATURE_COEFF_ADD;

	return (f32Temperature);
}

/**
*@brief         Perform an Unlock NVM command to Tsys03
*@return        tCommStatus: execution status of the command
*                  - eTSYS03_COMMSTATUS_OK: command executed as expected
*                  - eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*                  - eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*                  - eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*                  - eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*/
static eTsys03_CommStatus_t eUnlockNVM(void)
{
    eTsys03_CommStatus_t    tCommStatus;
    
    // send unlock command
    tCommStatus = eWriteCommand(U8_COMMAND_UNLOCK_NVM, B_FALSE);
    
    return (tCommStatus);
}

/**
*@brief         Write I2C Address in TSYS03 nvm
*@param[in]     u8Address: I2C address to be written
*@return        tCommStatus: execution status of the command
*                   - eTSYS03_COMMSTATUS_OK: command executed as expected
*                   - eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*                   - eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*                   - eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*                   - eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*/
static eTsys03_CommStatus_t eWriteNVMI2C(const uint8_t u8Address)
{
    eTsys03_CommStatus_t    tCommStatus;
    eI2c_StatusCode_t       eI2cStatus;
    uint8_t                 au8Data[U16_WRITE_NVM_I2C_SIZE];
    xI2c_MasterPacket_t     tPacket;

    // prepare data
    au8Data[0] = U8_COMMAND_WRITE_NVM_I2C_ADD;
    au8Data[1] = u8Address;
    au8Data[2] = 0x00U;
    au8Data[3] = 0x00U;
    au8Data[4] = 0x00U;
    au8Data[5] = 0x00U;
    au8Data[6] = 0x00U;
    
    // prepare packet
    tPacket.u16Address      = u16TSYS03Address;
    tPacket.u16DataLength   = U16_WRITE_NVM_I2C_SIZE;
    tPacket.pu8Data         = au8Data;
    
    // send the packet
    eI2cStatus = tI2c_MasterWritePacket(&tPacket);
    
    if (eI2C_STATUS_ERR_OVERFLOW == eI2cStatus)
    {
        tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
    }
    else if (eI2C_STATUS_ERR_ADDRESS == eI2cStatus)
    {
        tCommStatus =  eTSYS03_COMMSTATUS_NO_RESPONSE;
    }
    else if (eI2C_STATUS_ERR_TRANSFER == eI2cStatus)
    {
        tCommStatus = eTSYS03_COMMSTATUS_COMMAND_REJECTED;
    }
    else
    {
        tCommStatus = eTSYS03_COMMSTATUS_OK;
    }

    return (tCommStatus);
}

/**
*@brief         Write I2C Address in TSYS03 nvm
*@param[in]     u8Address: I2C address to be written
*@return        tCommStatus: execution status of the command
*                   - eTSYS03_COMMSTATUS_OK: command executed as expected
*                   - eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*                   - eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*                   - eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*                   - eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*/
static eTsys03_CommStatus_t eReadNVMI2C(uint8_t * const pu8ReadAddress)
{
    eTsys03_CommStatus_t    tCommStatus;
    uint8_t                 au8Data[U16_READ_NVM_I2C_SIZE];
    uint8_t                 u8NvmAddress;
    
    // send read NVM I2C ADD command
    tCommStatus = eWriteCommand(U8_COMMAND_READ_NVM_I2C_ADD, B_TRUE);

    if (eTSYS03_COMMSTATUS_OK == tCommStatus)
    {
        // read NVM I2C ADD data
        tCommStatus = eReadCommand(au8Data, U16_READ_NVM_I2C_SIZE);
    }
    
    if (eTSYS03_COMMSTATUS_OK == tCommStatus)
    {
        // extract I2C address
        u8NvmAddress = au8Data[U8_READ_NVM_I2C_ADD_INDEX];
        u8NvmAddress &= U8_I2C_ADDRESS_MASK;
        *pu8ReadAddress = u8NvmAddress;
    }
    
    return (tCommStatus);
}
/****************************************************
* PUBLIC FUNCTIONS
****************************************************/

/**
*@brief  Initialise TSYS03 driver
*@brief  In the current implementation, there is nothing to do.
*@brief  At PowerUp, TSYS03 needs at least 2ms to handle internal nvm check.
*@brief  User can implement here a procedure to handle power lines and timings.
*/
void vTsys03_init(void)
{
    //Power line can be managed here
    
    //Set TSYS03 default address
    u16TSYS03Address = TSYS03_ADDRESS;
}

/**
*@brief    Request a reset sequence of Tsys03
*@return   tCommStatus: execution status of the command
*				- eTSYS03_COMMSTATUS_OK: command executed as expected
*				- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*				- eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*@warning  When requested, all commands will be rejected for at least 15ms
*/
eTsys03_CommStatus_t eTsys03_Reset(void)
{
	eTsys03_CommStatus_t	tCommStatus;
	
	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_RESET, B_FALSE);

	return (tCommStatus);
}

/**
*@brief      Launch measurement of temperature according to configuration parameter
*@param[in]  tMeasureConfiguration: Measurement configuration
*@return     tCommStatus: execution status of the command
*				- eTSYS03_COMMSTATUS_OK: command executed as expected
*				- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*				- eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
eTsys03_CommStatus_t eTsys03_LaunchMeasurements(void)
{
	eTsys03_CommStatus_t	tCommStatus;
	
	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_START_CONVERSION, B_FALSE);
    
	return (tCommStatus);
}

/**
*@brief          Read Tsys03 serial number
*@param[in,out]  ptSerialNumber: Serial Number container
*@return         tCommStatus: execution status of the command
*					- eTSYS03_COMMSTATUS_OK: command executed as expected
*					- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*					- eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*					- eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Serial number crc check is not correct
*/
eTsys03_CommStatus_t eTsys03_ReadSerialNumber(Tsys03_SerialNumber_t * const ptSerialNumber)
{
	eTsys03_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint8_t					au8Data[U8_READ_SN_I2C_SIZE] = {0U};

	// clear Serial Number container
	*ptSerialNumber = 0U;

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_SN, B_TRUE);

	if (eTSYS03_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Data, U8_READ_SN_I2C_SIZE);
	}

	if (eTSYS03_COMMSTATUS_OK == tCommStatus)
	{
		//Check data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Data, U8_READ_SN_I2C_SIZE);

		if (B_TRUE == bIsDataCrcOk)
		{
			// copy Serial Number data
			ptSerialNumber[1] = au8Data[0];
			ptSerialNumber[2] = au8Data[1];
			ptSerialNumber[3] = au8Data[2];
		}
		else
		{
			//set error status and keep Serial Number cleared
			tCommStatus = eTSYS03_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief          Read measured Temperature
*@param[in,out]  pf32Temperature: measured Temperature in degree celsius
*@return         tCommStatus: execution status of the command
*					- eTSYS03_COMMSTATUS_OK: command executed as expected
*					- eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*					- eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*					- eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*/
eTsys03_CommStatus_t eTsys03_ReadTemperature(float32_t * const pf32Temperature)
{
	eTsys03_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint16_t				u16AdcValue;
	uint8_t					au8Data[U8_READ_T_I2C_SIZE] = {0U};

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_ADC_T, B_TRUE);

	if (eTSYS03_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Data, U8_READ_T_I2C_SIZE);
	}

	if (eTSYS03_COMMSTATUS_OK == tCommStatus)
	{
		//Check temperature data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Data, U8_READ_T_I2C_SIZE);

		if (B_TRUE == bIsDataCrcOk)
		{
			// construct 16bits data
			u16AdcValue = au8Data[0];
			u16AdcValue <<= U8_BYTE_SHIFT;
			u16AdcValue |= au8Data[1];
			//Convert adc value to degree
			*pf32Temperature = f32ConvertTemperatureAdcToDegree(u16AdcValue);
		}
		else
		{
			//set error status
			tCommStatus = eTSYS03_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief          Update TSYS03 I2C address
*@param[in]      u8I2cAddress: new address
*@return         tCommStatus: execution status of the command
*                   - eTSYS03_COMMSTATUS_OK: command executed as expected
*                   - eTSYS03_COMMSTATUS_NO_RESPONSE: No response from Tsys03
*                   - eTSYS03_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*                   - eTSYS03_COMMSTATUS_READ_ERROR: Read Error
*                   - eTSYS03_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*@warning        Due to mcu speed regarding mcu peripheral, ensure sufficient delay between I2C commands to prevent collision.
*/
eTsys03_CommStatus_t eTsys03_UpdateI2CAddress(const uint8_t u8I2cAddress)
{
    eTsys03_CommStatus_t    tCommStatus;
    uint8_t                 u8NewAddress;
    uint8_t                 u8NVMAddress;
    
    // new address range check and odd check
    if ((U8_I2C_ADDRESS_LLIMIT > u8I2cAddress) ||	//check Low range address
        (U8_I2C_ADDRESS_ULIMIT < u8I2cAddress) ||	//check High range address
		((u8I2cAddress & 0x01U) != 0U)			)	//check Even value address
    {
        tCommStatus = eTSYS03_COMMSTATUS_COMMAND_REJECTED;
    }
    else
    {
        //Prepare new address
        u8NewAddress = u8I2cAddress;
        u8NewAddress >>= 1U;
        u8NewAddress &= U8_I2C_ADDRESS_MASK;
        //set tCommStatus to allow next step
        tCommStatus = eTSYS03_COMMSTATUS_OK;
    }
    
    if (tCommStatus == eTSYS03_COMMSTATUS_OK)
    {
        //Unlock NVM
        tCommStatus = eUnlockNVM();
    }
    
    if (tCommStatus == eTSYS03_COMMSTATUS_OK)
    {
        //Write I2C Address in nvm
        tCommStatus = eWriteNVMI2C(u8NewAddress);
    }
    
    if (tCommStatus == eTSYS03_COMMSTATUS_OK)
    {
        //Read back I2C Address from nvm
        tCommStatus = eReadNVMI2C(&u8NVMAddress);
    }
    
    if (tCommStatus == eTSYS03_COMMSTATUS_OK)
    {
        // check if read back give the expected address
        if (u8NewAddress != u8NVMAddress)
        {
            tCommStatus = eTSYS03_COMMSTATUS_NVM_ERROR;
        }
    }
    
    if (tCommStatus == eTSYS03_COMMSTATUS_OK)
    {
        //Reset TSYS03
        (void)eTsys03_Reset();
        //Update Used I2C address
        u16TSYS03Address = (uint16_t)u8I2cAddress;
    }

    return (tCommStatus);
}

/****************************************************
* Unit Test Wrappers
****************************************************/
#ifdef UNIT_TEST
/**
*@brief      Unit test wrapper of the function bIsDataCrcCorrect()
*/
bool_t UT_bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize)
{
    return (bIsDataCrcCorrect(au8Buffer, u8BufferSize));
}

/**
*@brief      Unit test wrapper of the function eWriteCommand()
*/
eTsys03_CommStatus_t UT_eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand)
{
    return (eWriteCommand(u8Command, bIsReadCommand));
}

/**
*@brief      Unit test wrapper of the function eReadCommand()
*/
eTsys03_CommStatus_t UT_eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size)
{
    return (eReadCommand(au8Buffer, u8Size));
}

/**
*@brief      Unit test wrapper of the function f32ConvertTemperatureAdcToDegree()
*/
float32_t UT_f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc)
{
    return (f32ConvertTemperatureAdcToDegree(u16TemperatureAdc));
}

/**
*@brief      Unit test wrapper of the function eUnlockNVM()
*/
eTsys03_CommStatus_t UT_eUnlockNVM(void)
{
    return (eUnlockNVM());
}

/**
*@brief      Unit test wrapper of the function eWriteNVMI2C()
*/
eTsys03_CommStatus_t UT_eWriteNVMI2C(const uint8_t u8Address)
{
    return (eWriteNVMI2C(u8Address));
}

/**
*@brief      Unit test wrapper of the function eReadNVMI2C()
*/
eTsys03_CommStatus_t UT_eReadNVMI2C(uint8_t * const pu8ReadAddress)
{
    return (eReadNVMI2C(pu8ReadAddress));
}

#endif

#ifdef __cplusplus
}
#endif
/**@}*/
