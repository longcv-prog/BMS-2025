/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "BQ769x2Header.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEV_ADDR        0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode        1     // 0: disabled, 1: enabled (default use CRC)
#define MAX_BUFFER_SIZE 64

#define R   0 								// Read; Used in DirectCommands and Subcommands functions
#define W   1 								// Write; Used in DirectCommands and Subcommands functions
#define W2  2 								// Write data with two bytes; Used in Subcommands function

HAL_StatusTypeDef res;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RX_data [2] 			= {0x00, 0x00}; // used in several functions to store data read from BQ769x2
uint8_t RX_2Byte [2] 			= {0x00, 0x00};
uint8_t RX_32Byte [32] 		= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //used in Subcommands read function
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float Temperature [7] 		= {0, 0, 0, 0, 0, 0, 0};
float FET_Temperature 		= 0;
uint16_t CellVoltage[16];
float CellVoltage_V[16];
uint16_t Stack_mVoltage 	= 0x00;
float Stack_Voltage;
uint16_t Pack_Voltage 		= 0x00;
uint16_t LD_Voltage 			= 0x00;
int16_t Pack_Current 		  = 0x00;
float I_pack							= 0;
float sum_voltage 				= 0;
int index_check 					= 0;
uint16_t Fetstatus				= 0;
uint16_t current 					= 0x00;

uint16_t AlarmBits 				= 0x00;
uint16_t value_SafetyAlertA;  			// Safety Status Register A
uint16_t value_SafetyAlertB;  			// Safety Status Register B
uint16_t value_SafetyAlertC;  			// Safety Status Register C
uint16_t value_SafetyStatusA;  		  // Safety Status Register A
uint16_t value_SafetyStatusB;  		  // Safety Status Register B
uint16_t value_SafetyStatusC;  		  // Safety Status Register C
uint16_t value_PFStatusA;   				// Permanent Fail Status Register A
uint16_t value_PFStatusB;   				// Permanent Fail Status Register B
uint16_t value_PFStatusC;   				// Permanent Fail Status Register C
uint16_t FET_Status;  							// FET Status register contents  - Shows states of FETs
uint8_t value_FETOptions; 					// PreDischarge status
uint16_t CB_ActiveCells;  					// Cell Balancing Active Cells

uint16_t control_status     = 0;  	// 0x00 Control Status
uint16_t battery_status     = 0;  	// 0x12 Battery Status
uint16_t alarm_status_reg   = 0;  	// 0x62 Alarm Status (latched)
uint16_t alarm_raw_status   = 0;  	// 0x64 Alarm Raw Status (unlatched)
uint16_t alarm_enable_mask  = 0;  	// 0x66 Alarm Enable mask
uint16_t manufacturing_status = 0;

uint64_t OTP;
uint16_t OTP_Status;
uint8_t CFGUPDATE 	  = 0;
uint8_t PCHG_MODE 	  = 0;
uint8_t SLEEP_EN 		  = 0;
uint8_t POR 				  = 0;
uint8_t WD 				 	  = 0;
uint8_t COW_CHK 	    = 0;
uint8_t OTPW 				  = 0;
uint8_t OTPB 			 	  = 0;
uint8_t SEC0 				  = 0;
uint8_t SEC1 			 	  = 0;
uint8_t FUSE 				  = 0;
uint8_t SS 					  = 0;
uint8_t PF 					  = 0;
uint8_t SDM 				  = 0;
uint8_t SLEEP 			  = 0;   					// OTP write pending state

uint8_t	UV_Fault 		  = 0;   					// under-voltage fault state
uint8_t	OV_Fault 		  = 0;   					// over-voltage fault state
uint8_t	SCD_Fault	 	  = 0;  					// short-circuit fault state
uint8_t	OCD_Fault 	  = 0;  					// over-current fault state
uint8_t	OCD_Fault1 	  = 0;
uint8_t	OCC_Fault 	  = 0;
 
uint8_t	OTF_Fault 	  = 0;   					// under-voltage fault state
uint8_t	OTINT_Fault   = 0;   					// over-voltage fault state
uint8_t	OTD_Fault 	  = 0;  					// short-circuit fault state
uint8_t	OTC_Fault 	  = 0;  					// over-current fault state
uint8_t	UTINT_Fault   = 0;
uint8_t	UTD_Fault 	  = 0;
uint8_t	UTC_Fault 	  = 0;
 
uint8_t ProtectionsTriggered = 0; 		// Set to 1 if any protection triggers

uint8_t LD_ON 			  = 0;						// Load Detect status bit
uint8_t DSG 			 	  = 0;   					// discharge FET state
uint8_t CHG 				  = 0;   					// charge FET state
uint8_t PCHG 				  = 0;  					// pre-charge FET state
uint8_t PDSG 				  = 0;  					// pre-discharge FET state
uint8_t DCHG_pin 		  = 0;
uint8_t DDSG_pin 		  = 0;
uint8_t ALRT_pin 		  = 0;
uint8_t RSVD_pin 		  = 0;

uint8_t RSVD_00 			= 0;
uint8_t RSVD_01 			= 0;
uint8_t FET_INIT_OFF 	= 0;
uint8_t PDSG_EN 			= 0;
uint8_t FET_CTRL_EN 	= 0;
uint8_t HOST_FET_EN 	= 0;
uint8_t SLEEPCHG 			= 0;
uint8_t SFET 					= 0;

uint32_t CC1; 												// in BQ769x2_READPASSQ func
uint32_t CC2;													// in BQ769x2_READPASSQ func
uint32_t CC3;													// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Int; 			// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac;			// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time;			// in BQ769x2_READPASSQ func
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void delayUS(uint32_t us) {   										// Sets the delay in microseconds.
	__HAL_TIM_SET_COUNTER(&htim1,0);  							// set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  		// wait for the counter to reach the us input in the parameter
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.	
{
	unsigned char i;
	unsigned char checksum = 0;

	for(i=0; i<len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return(checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions 
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= 0x107;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return(crc);
}

void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
	uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if CRC_Mode
	{
		uint8_t crc_count = 0;
		crc_count = count * 2;
		uint8_t crc1stByteBuffer [3] = {0x10, reg_addr, reg_data[0]};
		unsigned int j;
		unsigned int i;
		uint8_t temp_crc_buffer [3];

		TX_Buffer[0] = reg_data[0];
		TX_Buffer[1] = CRC8(crc1stByteBuffer,3);

		j = 2;
		for(i=1; i<count; i++)
		{
			TX_Buffer[j] = reg_data[i];
			j = j + 1;
			temp_crc_buffer[0] = reg_data[i];
			TX_Buffer[j] = CRC8(temp_crc_buffer,1);
			j = j + 1;
		}
		HAL_I2C_Mem_Write(&hi2c2, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count, 1000);
	}
#else 
	HAL_I2C_Mem_Write(&hi2c2, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
}

int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
	unsigned int RX_CRC_Fail = 0;  // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
	uint8_t RX_Buffer [MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if CRC_Mode
	{
		uint8_t crc_count = 0;
		uint8_t ReceiveBuffer [10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		crc_count = count * 2;
		unsigned int j;
		unsigned int i;
		unsigned char CRCc = 0;
		uint8_t temp_crc_buffer [3];

		HAL_I2C_Mem_Read(&hi2c2, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 1000);
		uint8_t crc1stByteBuffer [4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
		CRCc = CRC8(crc1stByteBuffer,4);
		if (CRCc != ReceiveBuffer[1])
		{
			RX_CRC_Fail += 1;
		}
		RX_Buffer[0] = ReceiveBuffer[0];

		j = 2;
		for (i=1; i<count; i++)
		{
			RX_Buffer[i] = ReceiveBuffer[j];
			temp_crc_buffer[0] = ReceiveBuffer[j];
			j = j + 1;
			CRCc = CRC8(temp_crc_buffer,1);
			if (CRCc != ReceiveBuffer[j])
				RX_CRC_Fail += 1;
			j = j + 1;
		}
		CopyArray(RX_Buffer, reg_data, crc_count);
	}
#else
	HAL_I2C_Mem_Read(&hi2c2, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
	return 0;
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
	uint8_t TX_Buffer[2] = {0x00, 0x00};
	uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	//TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff; 
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch(datalen)
    {
		case 1: //1 byte datalength
      I2C_WriteReg(0x3E, TX_RegData, 3);
			delayUS(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 3); 
			TX_Buffer[1] = 0x05; //combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayUS(2000);
			break;
		case 2: //2 byte datalength
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			I2C_WriteReg(0x3E, TX_RegData, 4);
			delayUS(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 4); 
			TX_Buffer[1] = 0x06; //combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayUS(2000);
			break;
		case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			TX_RegData[4] = (reg_data >> 16) & 0xff;
			TX_RegData[5] = (reg_data >> 24) & 0xff;
			I2C_WriteReg(0x3E, TX_RegData, 6);
			delayUS(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 6); 
			TX_Buffer[1] = 0x08; //combined length of register address and data
      I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayUS(2000);
			break;
    }
}

static uint8_t DM_Read8(uint16_t addr) {
  uint8_t tx[2] = { addr & 0xFF, (uint8_t)(addr >> 8) };
  I2C_WriteReg(0x3E, tx, 2);
  delayUS(2000);
  uint8_t v = 0;
  I2C_ReadReg(0x40, &v, 1);   // read 1 byte from Data Memory
  return v;
}

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{	//For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively
	
	uint8_t TX_Reg[2] = {0x00, 0x00};

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	I2C_WriteReg(0x3E,TX_Reg,2); 
	delayUS(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
	uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t TX_Buffer[2] = {0x00, 0x00};

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff; 

	if (type == R) {//read
		I2C_WriteReg(0x3E,TX_Reg,2);
		delayUS(2000);
		I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
	}
	else if (type == W) {
		//FET_Control, REG12_Control
		TX_Reg[2] = data & 0xff; 
		I2C_WriteReg(0x3E,TX_Reg,3);
		delayUS(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 3);
		TX_Buffer[1] = 0x05; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
		delayUS(1000); 
	}
	else if (type == W2){ //write data with 2 bytes
		//CB_Active_Cells, CB_SET_LVL
		TX_Reg[2] = data & 0xff; 
		TX_Reg[3] = (data >> 8) & 0xff;
		I2C_WriteReg(0x3E,TX_Reg,4);
		delayUS(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 4); 
		TX_Buffer[1] = 0x06; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
		delayUS(1000); 
	}
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = {0x00, 0x00};

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {//Read
		I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
		delayUS(2000);
	}
	if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
		I2C_WriteReg(command,TX_data,2);
		delayUS(2000);
	}
}

void BQ769x2_Init() {
	CommandSubcommands(SET_CFGUPDATE);

	// Calibration
	BQ769x2_SetRegister(Cell1Gain, 12120, 2);
	BQ769x2_SetRegister(Cell2Gain, 12120, 2);
	BQ769x2_SetRegister(Cell3Gain, 12120, 2);
	BQ769x2_SetRegister(Cell4Gain, 12120, 2);
	BQ769x2_SetRegister(Cell5Gain, 12119, 2);
	BQ769x2_SetRegister(Cell6Gain, 12119, 2);
	BQ769x2_SetRegister(Cell7Gain, 12119, 2);
	BQ769x2_SetRegister(Cell8Gain, 12119, 2);
	BQ769x2_SetRegister(Cell9Gain, 12122, 2);
	BQ769x2_SetRegister(Cell10Gain, 12122, 2);
	BQ769x2_SetRegister(Cell11Gain, 12122, 2);
	BQ769x2_SetRegister(Cell12Gain, 12122, 2);
	BQ769x2_SetRegister(Cell13Gain, 12121, 2);
	BQ769x2_SetRegister(Cell14Gain, 12121, 2);
	BQ769x2_SetRegister(Cell15Gain, 12121, 2);
	BQ769x2_SetRegister(Cell16Gain, 12121, 2);
	
	BQ769x2_SetRegister(PackGain, 34067, 2);
	BQ769x2_SetRegister(TOSGain, 33961, 2);
	BQ769x2_SetRegister(LDGain, 34268, 2);
	BQ769x2_SetRegister(ADCGain, 4039, 2);
	BQ769x2_SetRegister(CCGain, 1.000, 4);      			
	BQ769x2_SetRegister(CapacityGain, 1.000, 4);			
	BQ769x2_SetRegister(VcellOffset, 0, 2);        		
	BQ769x2_SetRegister(VdivOffset, 0, 2);     				
	BQ769x2_SetRegister(CoulombCounterOffsetSamples, 64, 2);
	BQ769x2_SetRegister(BoardOffset, 0, 2);	
	
	BQ769x2_SetRegister(InternalTempOffset, 0.0, 1);
	BQ769x2_SetRegister(CFETOFFTempOffset, 0.0, 1);
	BQ769x2_SetRegister(DFETOFFTempOffset, 0.0, 1);
	BQ769x2_SetRegister(ALERTTempOffset, 0.0, 1);
	BQ769x2_SetRegister(TS1TempOffset, 0.0, 1);
	BQ769x2_SetRegister(TS2TempOffset, 0.0, 1);
	BQ769x2_SetRegister(TS3TempOffset, 0.0, 1);
	BQ769x2_SetRegister(HDQTempOffset, 0.0, 1);
	BQ769x2_SetRegister(DCHGTempOffset, 0.0, 1);
	BQ769x2_SetRegister(DDSGTempOffset, 0.0, 1);
	BQ769x2_SetRegister(IntGain, 25390, 2);
	BQ769x2_SetRegister(Intbaseoffset, 3032, 2);
	BQ769x2_SetRegister(IntMaximumAD, 16383, 2);
	BQ769x2_SetRegister(IntMaximumTemp, 6379, 2);
	
	BQ769x2_SetRegister(T18kCoeffa1, (uint16_t)-15524, 2);
	BQ769x2_SetRegister(T18kCoeffa2, 26423, 2);
	BQ769x2_SetRegister(T18kCoeffa3, (uint16_t)-22664, 2);
	BQ769x2_SetRegister(T18kCoeffa4, 28834, 2);
	BQ769x2_SetRegister(T18kCoeffa5, 672, 2);
	BQ769x2_SetRegister(T18kCoeffb1, (uint16_t)-371, 2);
	BQ769x2_SetRegister(T18kCoeffb2, 708, 2);
	BQ769x2_SetRegister(T18kCoeffb3, (uint16_t)-3498, 2);
	BQ769x2_SetRegister(T18kCoeffb4, 5051, 2);
	BQ769x2_SetRegister(T18kAdc0, 11703, 2);
	
	BQ769x2_SetRegister(T180kCoeffa1, (uint16_t)-17513, 2);
	BQ769x2_SetRegister(T180kCoeffa2, 25759, 2);
	BQ769x2_SetRegister(T180kCoeffa3, (uint16_t)-23593, 2);
	BQ769x2_SetRegister(T180kCoeffa4, 32175, 2);
	BQ769x2_SetRegister(T180kCoeffa5, 2090, 2);
	BQ769x2_SetRegister(T180kCoeffb1, (uint16_t)-2055, 2);
	BQ769x2_SetRegister(T180kCoeffb2, 2955, 2);
	BQ769x2_SetRegister(T180kCoeffb3, (uint16_t)-3427, 2);
	BQ769x2_SetRegister(T180kCoeffb4, 4385, 2);
	BQ769x2_SetRegister(T180kAdc0, 17246, 2);

	BQ769x2_SetRegister(CustomCoeffa1, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa2, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa3, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa4, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa5, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb1, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb2, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb3, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb4, 0x00, 2);
	BQ769x2_SetRegister(CustomRc0, 0x00, 2);
	BQ769x2_SetRegister(CustomAdc0, 0x00, 2);
	
	BQ769x2_SetRegister(CoulombCounterDeadband, 9, 1);
	BQ769x2_SetRegister(CUVThresholdOverride, 0xFFFF, 2);
	BQ769x2_SetRegister(COVThresholdOverride, 0xFFFF, 2);
	BQ769x2_SetRegister(MinBlowFuseVoltage, 500, 2);
	BQ769x2_SetRegister(FuseBlowTimeout, 30, 1);
	
	BQ769x2_SetRegister(PowerConfig, 0x2C82, 2);
	BQ769x2_SetRegister(REG12Config, 0x0D, 1);
	BQ769x2_SetRegister(REG0Config, 0x01, 1);
	BQ769x2_SetRegister(HWDRegulatorOptions, 0x00, 1);
	BQ769x2_SetRegister(CommType, 0, 1);
	BQ769x2_SetRegister(I2CAddress, 0, 1);
	BQ769x2_SetRegister(SPIConfiguration, 0x20, 1);
	BQ769x2_SetRegister(CommIdleTime, 0, 1);
	BQ769x2_SetRegister(CFETOFFPinConfig, 0xAF, 1);
	BQ769x2_SetRegister(DFETOFFPinConfig, 0xEE, 1);
	BQ769x2_SetRegister(ALERTPinConfig, 0x3A, 1);
	BQ769x2_SetRegister(TS1Config, 0x07, 1);
	BQ769x2_SetRegister(TS2Config, 0x0F, 1);
	BQ769x2_SetRegister(TS3Config, 0x0F, 1);	
	BQ769x2_SetRegister(HDQPinConfig, 0x2B, 1);  
	BQ769x2_SetRegister(DCHGPinConfig, 0x2A, 1); 
	BQ769x2_SetRegister(DDSGPinConfig, 0x2A, 1); 
	BQ769x2_SetRegister(DAConfiguration, 0x05, 1); 	
	BQ769x2_SetRegister(VCellMode, 0x0000, 2);
	BQ769x2_SetRegister(CC3Samples, 80, 1);
	BQ769x2_SetRegister(ProtectionConfiguration, 0x0000, 2); //0x0002
	BQ769x2_SetRegister(EnabledProtectionsA, 0x00, 1);			 //0xFC
	BQ769x2_SetRegister(EnabledProtectionsB, 0x00, 1);
	BQ769x2_SetRegister(EnabledProtectionsC, 0x00, 1);
	BQ769x2_SetRegister(CHGFETProtectionsA, 0x98, 1);
	BQ769x2_SetRegister(CHGFETProtectionsB, 0xD5, 1);
	BQ769x2_SetRegister(CHGFETProtectionsC, 0x56, 1);
	BQ769x2_SetRegister(DSGFETProtectionsA, 0xE4, 1);	
	BQ769x2_SetRegister(DSGFETProtectionsB, 0xE6, 1);
	BQ769x2_SetRegister(DSGFETProtectionsC, 0xE2, 1);
	BQ769x2_SetRegister(BodyDiodeThreshold, 50, 2);

	BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);
	BQ769x2_SetRegister(SFAlertMaskA, 0xFC, 1);
	BQ769x2_SetRegister(SFAlertMaskB, 0xF7, 1);
	BQ769x2_SetRegister(SFAlertMaskC, 0xF4, 1);
	BQ769x2_SetRegister(PFAlertMaskA, 0x5F, 1);
	BQ769x2_SetRegister(PFAlertMaskB, 0x9F, 1);
	BQ769x2_SetRegister(PFAlertMaskC, 0x00, 1);
	BQ769x2_SetRegister(PFAlertMaskD, 0x00, 1);
	BQ769x2_SetRegister(EnabledPFA, 0x03, 1);
	BQ769x2_SetRegister(EnabledPFB, 0x00, 1);
	BQ769x2_SetRegister(EnabledPFC, 0x07, 1);
	BQ769x2_SetRegister(EnabledPFD, 0x00, 1);

	BQ769x2_SetRegister(FETOptions, 0x0F, 1);
	BQ769x2_SetRegister(ChgPumpControl, 0x01, 1);
	BQ769x2_SetRegister(PrechargeStartVoltage, 0, 2);
	BQ769x2_SetRegister(PrechargeStopVoltage, 0, 2);
	BQ769x2_SetRegister(PredischargeTimeout, 5, 1);	
	BQ769x2_SetRegister(PredischargeStopDelta, 50, 1);
	BQ769x2_SetRegister(DsgCurrentThreshold, 100, 2);
	BQ769x2_SetRegister(ChgCurrentThreshold, 50, 2);
	BQ769x2_SetRegister(CheckTime, 5, 1);

	BQ769x2_SetRegister(Cell1Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell2Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell3Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell4Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell5Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell6Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell7Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell8Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell9Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell10Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell11Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell12Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell13Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell14Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell15Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell16Interconnect, 0, 2);
	
	BQ769x2_SetRegister(MfgStatusInit, 0x0050, 2);
	BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);
	BQ769x2_SetRegister(MinCellTemp, (uint8_t)-20, 1);   		// -20°C
	BQ769x2_SetRegister(MaxCellTemp, 60, 1);   		// 60°C
	BQ769x2_SetRegister(MaxInternalTemp, 70, 1); 	// 70°C
	BQ769x2_SetRegister(CellBalanceInterval, 20, 1);
	BQ769x2_SetRegister(CellBalanceMaxCells, 1, 1);
	BQ769x2_SetRegister(CellBalanceMinCellVCharge, 3900, 2);
	BQ769x2_SetRegister(CellBalanceMinDeltaCharge, 40, 1);
	BQ769x2_SetRegister(CellBalanceStopDeltaCharge, 20, 1);
	BQ769x2_SetRegister(CellBalanceMinCellVRelax, 3900, 2);
	BQ769x2_SetRegister(CellBalanceMinDeltaRelax, 40, 1);
	BQ769x2_SetRegister(CellBalanceStopDeltaRelax, 20, 1);

	BQ769x2_SetRegister(ShutdownCellVoltage, 0, 2);
	BQ769x2_SetRegister(ShutdownStackVoltage, 600, 2);
	BQ769x2_SetRegister(LowVShutdownDelay, 1, 1);
	BQ769x2_SetRegister(ShutdownTemperature, 85, 1);
	BQ769x2_SetRegister(ShutdownTemperatureDelay, 5, 1);
	BQ769x2_SetRegister(FETOffDelay, 0, 1);
	BQ769x2_SetRegister(ShutdownCommandDelay, 0, 1);
	BQ769x2_SetRegister(AutoShutdownTime, 0, 1);
	BQ769x2_SetRegister(RAMFailShutdownTime, 5, 1);
	BQ769x2_SetRegister(SleepCurrent, 20, 2);
	BQ769x2_SetRegister(VoltageTime, 5, 1);
	BQ769x2_SetRegister(WakeComparatorCurrent, 500, 2);
	BQ769x2_SetRegister(SleepHysteresisTime, 10, 1);
	BQ769x2_SetRegister(SleepChargerVoltageThreshold, 2000, 2);
	BQ769x2_SetRegister(SleepChargerPACKTOSDelta, 200, 2);
	BQ769x2_SetRegister(ConfigRAMSignature, 0, 2);
	
	BQ769x2_SetRegister(CUVThreshold, 50, 1);
 	BQ769x2_SetRegister(CUVDelay, 74, 2);
 	BQ769x2_SetRegister(CUVRecoveryHysteresis, 2, 1);

	BQ769x2_SetRegister(COVThreshold, 86, 1);
	BQ769x2_SetRegister(COVDelay, 74, 2);
	BQ769x2_SetRegister(COVRecoveryHysteresis, 2, 1);
	
	BQ769x2_SetRegister(COVLLatchLimit, 0, 1);
	BQ769x2_SetRegister(COVLCounterDecDelay, 10, 1);
	BQ769x2_SetRegister(COVLRecoveryTime, 15, 1);
	
	BQ769x2_SetRegister(OCCThreshold, 2, 1);
	BQ769x2_SetRegister(OCCDelay, 4, 1);
	BQ769x2_SetRegister(OCCRecoveryThreshold, (uint16_t)-200, 2);
	BQ769x2_SetRegister(OCCPACKTOSDelta, 200, 2);	
	
	BQ769x2_SetRegister(OCD1Threshold, 4, 1);
	BQ769x2_SetRegister(OCD1Delay, 1, 1);
	BQ769x2_SetRegister(OCD2Threshold, 3, 1);
	BQ769x2_SetRegister(OCD2Delay, 7, 1);
	
	BQ769x2_SetRegister(SCDThreshold, 0, 1);  
	BQ769x2_SetRegister(SCDDelay, 2, 1);	
	BQ769x2_SetRegister(SCDRecoveryTime, 5, 1);		
	
	BQ769x2_SetRegister(OCD3Threshold, (uint16_t)-4000, 2);
	BQ769x2_SetRegister(OCD3Delay, 2, 1);	
	BQ769x2_SetRegister(OCDRecoveryThreshold, 200, 2);
	BQ769x2_SetRegister(OCDLLatchLimit, 0, 1);
	BQ769x2_SetRegister(OCDLCounterDecDelay, 10, 1);
	BQ769x2_SetRegister(OCDLRecoveryTime, 15, 1);
	BQ769x2_SetRegister(OCDLRecoveryThreshold, 200, 2);
	
	BQ769x2_SetRegister(SCDLLatchLimit, 0, 1);
	BQ769x2_SetRegister(SCDLCounterDecDelay, 10, 1);	
	BQ769x2_SetRegister(SCDLRecoveryTime, 15, 1);	
	BQ769x2_SetRegister(SCDLRecoveryThreshold, 200, 2);	
	
	BQ769x2_SetRegister(OTCThreshold, 55, 1);
	BQ769x2_SetRegister(OTCDelay, 2, 1);
	BQ769x2_SetRegister(OTCRecovery, 50, 1);

	BQ769x2_SetRegister(OTDThreshold, 60, 1);
	BQ769x2_SetRegister(OTDDelay, 2, 1);
	BQ769x2_SetRegister(OTDRecovery, 55, 1);

	BQ769x2_SetRegister(OTFThreshold, 80, 1);
	BQ769x2_SetRegister(OTFDelay, 2, 1);
	BQ769x2_SetRegister(OTFRecovery, 65, 1);

	BQ769x2_SetRegister(OTINTThreshold, 85, 1);
	BQ769x2_SetRegister(OTINTDelay, 2, 1);
	BQ769x2_SetRegister(OTINTRecovery, 80, 1);

	BQ769x2_SetRegister(UTCThreshold, 0, 1);
	BQ769x2_SetRegister(UTCDelay, 2, 1);
	BQ769x2_SetRegister(UTCRecovery, 5, 1);

	BQ769x2_SetRegister(UTDThreshold, 0, 1);
	BQ769x2_SetRegister(UTDDelay, 2, 1);
	BQ769x2_SetRegister(UTDRecovery, 5, 1);
	
	BQ769x2_SetRegister(UTINTThreshold, (uint8_t)-20, 1);
	BQ769x2_SetRegister(UTINTDelay, 2, 1);
	BQ769x2_SetRegister(UTINTRecovery, (uint8_t)-15, 1);
	
	BQ769x2_SetRegister(ProtectionsRecoveryTime, 3, 1);	
	BQ769x2_SetRegister(HWDDelay, 60, 2);

	BQ769x2_SetRegister(LoadDetectActiveTime, 0, 1);
	BQ769x2_SetRegister(LoadDetectRetryDelay, 50, 1);	
	BQ769x2_SetRegister(LoadDetectTimeout, 1, 2);

	BQ769x2_SetRegister(PTOChargeThreshold, 250, 2);
	BQ769x2_SetRegister(PTODelay, 1800, 2);	
	BQ769x2_SetRegister(PTOReset, 2, 2);	
	
	BQ769x2_SetRegister(CUDEPThreshold, 1500, 2);
	BQ769x2_SetRegister(CUDEPDelay, 2, 1);	
	BQ769x2_SetRegister(SUVThreshold, 2200, 2);	
	BQ769x2_SetRegister(SUVDelay, 5, 1);
	BQ769x2_SetRegister(SOVThreshold, 4500, 2);	
	BQ769x2_SetRegister(SOVDelay, 5, 1);		
	BQ769x2_SetRegister(TOSSThreshold, 500, 2);
	BQ769x2_SetRegister(TOSSDelay, 5, 1);	
	BQ769x2_SetRegister(SOCCThreshold, 10000, 2);	
	BQ769x2_SetRegister(SOCCDelay, 5, 1);
	BQ769x2_SetRegister(SOCDThreshold, (uint16_t)-32000, 2);	
	BQ769x2_SetRegister(SOCDDelay, 5, 1);	
	BQ769x2_SetRegister(SOTThreshold, 65, 1);
	BQ769x2_SetRegister(SOTDelay, 5, 1);	
	BQ769x2_SetRegister(SOTFThreshold, 85, 1);	
	BQ769x2_SetRegister(SOTFDelay, 5, 1);		
	BQ769x2_SetRegister(VIMRCheckVoltage, 3500, 2);
	BQ769x2_SetRegister(VIMRMaxRelaxCurrent, 10, 2);	
	BQ769x2_SetRegister(VIMRThreshold, 500, 2);	
	BQ769x2_SetRegister(VIMRDelay, 5, 1);
	BQ769x2_SetRegister(VIMRRelaxMinDuration, 100, 2);	
	BQ769x2_SetRegister(VIMACheckVoltage, 3700, 2);	
	BQ769x2_SetRegister(VIMAMinActiveCurrent, 50, 2);
	BQ769x2_SetRegister(VIMAThreshold, 200, 2);	
	BQ769x2_SetRegister(VIMADelay, 5, 1);
	BQ769x2_SetRegister(CFETFOFFThreshold, 20, 2);
	BQ769x2_SetRegister(CFETFOFFDelay, 5, 1);	
	BQ769x2_SetRegister(DFETFOFFThreshold, (uint16_t)-20, 2);	
	BQ769x2_SetRegister(DFETFOFFDelay, 5, 1);
	BQ769x2_SetRegister(VSSFFailThreshold, 100, 2);	
	BQ769x2_SetRegister(VSSFDelay, 5, 1);	
	BQ769x2_SetRegister(PF2LVLDelay, 5, 1);
	BQ769x2_SetRegister(LFOFDelay, 5, 1);	
	BQ769x2_SetRegister(HWMXDelay, 5, 1);

	BQ769x2_SetRegister(SecuritySettings, 0x00, 1);
	BQ769x2_SetRegister(UnsealKeyStep1, 0x0414, 2);
	BQ769x2_SetRegister(UnsealKeyStep2, 0x3672, 2);
	BQ769x2_SetRegister(FullAccessKeyStep1, 0xFFFF, 2);
	BQ769x2_SetRegister(FullAccessKeyStep2, 0xFFFF, 2);
	
	CommandSubcommands(EXIT_CFGUPDATE);
}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_BOTHOFF () {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);  // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF () {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_ReadFETStatus() {
  DirectCommands(FETStatus, 0x00, R);
  FET_Status = (RX_data[1]*256 + RX_data[0]);
  CHG   		 = (RX_data[0] & 0x01) >> 0;
  PCHG  		 = (RX_data[0] & 0x02) >> 1;
  DSG   		 = (RX_data[0] & 0x04) >> 2;
  PDSG  		 = (RX_data[0] & 0x08) >> 3;
  DCHG_pin 	 = (RX_data[0] & 0x10) >> 4; 
  DDSG_pin 	 = (RX_data[0] & 0x20) >> 5; 
  ALRT_pin 	 = (RX_data[0] & 0x40) >> 6; 
  RSVD_pin 	 = (RX_data[0] & 0x80) >> 7; 
}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_ShutdownPin() {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	HAL_GPIO_WritePin(GPIOE, RST_SHUT_Pin, GPIO_PIN_SET);  // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	HAL_GPIO_WritePin(GPIOE, RST_SHUT_Pin, GPIO_PIN_RESET);  // Resets RST_SHUT pin
}

// ********************************* End of BQ769x2 Power Commands   *****************************************


// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus() { 
	// Read this register to find out why the ALERT pin was asserted
	DirectCommands(AlarmStatus, 0x00, R);
	return (RX_data[1]*256 + RX_data[0]);
}

void BQ769x2_ReadSafetyStatus() { //good example functions
	// Read Safety Status A/B/C and find which bits are set
	// This shows which primary protections have been triggered
	DirectCommands(SafetyAlertA, 0x00, R);
	value_SafetyAlertA = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(SafetyAlertB, 0x00, R);
	value_SafetyAlertB = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(SafetyAlertC, 0x00, R);
	value_SafetyAlertC = (RX_data[1]*256 + RX_data[0]);
	
	DirectCommands(SafetyStatusA, 0x00, R);
	value_SafetyStatusA = (RX_data[1]*256 + RX_data[0]);
	//Example Fault Flags	
	UV_Fault 		= ((0x04 & RX_data[0])>>2);
	OV_Fault 		= ((0x08 & RX_data[0])>>3);
	OCC_Fault 	= ((0x10 & RX_data[0])>>4);
	OCD_Fault1 	= ((0x20 & RX_data[0])>>5);
	OCD_Fault 	= ((0x40 & RX_data[0])>>6);
	SCD_Fault 	= ((0x80 & RX_data[0])>>7);
	
	DirectCommands(SafetyStatusB, 0x00, R);
	value_SafetyStatusB = (RX_data[1]*256 + RX_data[0]);
//	OTC_Fault 	= ((0x10 & RX_data[0])>>4);
//	OTD_Fault 	= ((0x20 & RX_data[0])>>5);
//	OTINT_Fault = ((0x40 & RX_data[0])>>6);
//	OTF_Fault 	= ((0x80 & RX_data[0])>>7);

	DirectCommands(SafetyStatusC, 0x00, R);
	value_SafetyStatusC = (RX_data[1]*256 + RX_data[0]);
	if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
		ProtectionsTriggered = 1; }
	else {
		ProtectionsTriggered = 0; }
}

void statusread() {
  value_FETOptions = DM_Read8(FETOptions); 

  SFET        =  (value_FETOptions & 0x01) >> 0;
  SLEEPCHG    =  (value_FETOptions & 0x02) >> 1;
  HOST_FET_EN =  (value_FETOptions & 0x04) >> 2;
  FET_CTRL_EN =  (value_FETOptions & 0x08) >> 3;
  PDSG_EN     =  (value_FETOptions & 0x10) >> 4;  
  FET_INIT_OFF=  (value_FETOptions & 0x20) >> 5;  
  RSVD_01     =  (value_FETOptions & 0x40) >> 6;  
  RSVD_00     =  (value_FETOptions & 0x80) >> 7;  
}

void BQ769x2_ReadPFStatus() {
	// Read Permanent Fail Status A/B/C and find which bits are set
	// This shows which permanent failures have been triggered
	DirectCommands(PFStatusA, 0x00, R);
	value_PFStatusA = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(PFStatusB, 0x00, R);
	value_PFStatusB = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(PFStatusC, 0x00, R);
	value_PFStatusC = (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadControlStatus(void)
{
    DirectCommands(ControlStatus, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadBatteryStatus(void)
{
    DirectCommands(BatteryStatus, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadAlarmStatusReg(void)
{
    DirectCommands(AlarmStatus, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadAlarmRawStatus(void)
{
    DirectCommands(AlarmRawStatus, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadAlarmEnable(void)
{
    DirectCommands(AlarmEnable, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

uint16_t BQ769x2_ReadManufacturingStatus(void)
{
    Subcommands(MANUFACTURINGSTATUS, 0x00, R);
    return (uint16_t)((RX_32Byte[1] << 8) | RX_32Byte[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************


// ********************************* BQ769x2 Measurement Commands   *****************************************


uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	//RX_data is global var
	DirectCommands(command, 0x00, R);
	if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
		return (RX_data[1]*256 + RX_data[0]); //voltage is reported in mV
	}
	else {//stack, Pack, LD
		return 10 * (RX_data[1]*256 + RX_data[0]); //voltage is reported in 0.01V units
	}

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
	int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
	for (int i = 0; i < 16; i++) {//Reads all cell voltages
	CellVoltage[i] = BQ769x2_ReadVoltage(cellvoltageholder);
	cellvoltageholder = cellvoltageholder + 2;
	index_check = i;
	}
	sum_voltage = 0;
	for (int i = 0; i <16; i++) {
	sum_voltage +=CellVoltage[i];
	}
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}
 
uint16_t BQ769x2_ReadCurrent() 
// Reads PACK current 
{
	DirectCommands(CC2Current, 0x00, R);
	return (RX_data[1]*256 + RX_data[0]);  // current is reported in mA
}

uint16_t AFE_ReadCurrent() {
	I2C_ReadReg(0x3A, RX_2Byte, 2);
	return (RX_2Byte[1]*256 + RX_2Byte[0]);  // current is reported in mA
}

float BQ769x2_ReadTemperature(uint8_t command) 
{
	DirectCommands(command, 0x00, R);
	//RX_data is a global var
	return (0.1 * (float)(RX_data[1]*256 + RX_data[0])) - 273.15;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadPassQ(){ // Read Accumulated Charge and Time from DASTATUS6 
	Subcommands(DASTATUS6, 0x00, R);
	AccumulatedCharge_Int = ((RX_32Byte[3]<<24) + (RX_32Byte[2]<<16) + (RX_32Byte[1]<<8) + RX_32Byte[0]); //Bytes 0-3
	AccumulatedCharge_Frac = ((RX_32Byte[7]<<24) + (RX_32Byte[6]<<16) + (RX_32Byte[5]<<8) + RX_32Byte[4]); //Bytes 4-7
	AccumulatedCharge_Time = ((RX_32Byte[11]<<24) + (RX_32Byte[10]<<16) + (RX_32Byte[9]<<8) + RX_32Byte[8]); //Bytes 8-11
}

void BQ769x2_OTP_STATUS(){
    Subcommands(OTP_WR_CHECK, 0x00, R);
    OTP = ((uint64_t)RX_32Byte[7] << 56) | ((uint64_t)RX_32Byte[6] << 48) |
          ((uint64_t)RX_32Byte[5] << 40) | ((uint64_t)RX_32Byte[4] << 32) |
          ((uint64_t)RX_32Byte[3] << 24) | ((uint64_t)RX_32Byte[2] << 16) |
          ((uint64_t)RX_32Byte[1] << 8)  | RX_32Byte[0];
}

void BQ769x2_OTP_SCAN(){
	DirectCommands(BatteryStatus, 0x00, R);
	OTP_Status = (RX_data[1]*256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Measurement Commands   *****************************************

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    volatile int i = 0;
//		char uart_buf[50];
//		int uart_buf_len;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// Start timer
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(GPIOE, RST_SHUT_Pin, GPIO_PIN_RESET);  // RST_SHUT pin set low
	HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);   // DFETOFF pin (BOTHOFF) set low
  delayUS(10000);
	
	CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
	delayUS(60000);
	BQ769x2_Init();  // Configure all of the BQ769x2 register settings
	delayUS(10000);
	CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
	delayUS(10000);
	CommandSubcommands(SLEEP_DISABLE); // Sleep mode is enabled by default. For this example, Sleep is disabled to 
																		 // demonstrate full-speed measurements in Normal mode.
	delayUS(60000); 
	delayUS(60000); 
	delayUS(60000); 
	delayUS(60000);  //wait to start measurements after FETs close

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		// Nháy LED LD2 để báo nạp code thành công
		for (int i = 0; i < 1; i++) {
				HAL_GPIO_WritePin(GPIOE, LD2_Pin, GPIO_PIN_SET);   // Bật LED
				HAL_Delay(500);                                    // Chờ 500ms
				HAL_GPIO_WritePin(GPIOE, LD2_Pin, GPIO_PIN_RESET); // Tắt LED
				HAL_Delay(500);                                    // Chờ 500ms
		}
    /* USER CODE END WHILE */
		BQ769x2_ReadAllVoltages();
	  Pack_Current = (int16_t)BQ769x2_ReadCurrent();
	  BQ769x2_ReadFETStatus();
	  statusread();
	  BQ769x2_ReadSafetyStatus();
	  BQ769x2_ReadPFStatus();
	  BQ769x2_OTP_STATUS();
	  BQ769x2_OTP_SCAN();

		control_status    = BQ769x2_ReadControlStatus();
    battery_status    = BQ769x2_ReadBatteryStatus();
    alarm_status_reg  = BQ769x2_ReadAlarmStatusReg();
    alarm_raw_status  = BQ769x2_ReadAlarmRawStatus();
    alarm_enable_mask = BQ769x2_ReadAlarmEnable();
		manufacturing_status = BQ769x2_ReadManufacturingStatus();
		
	  Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
	  Temperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
	  Temperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
	  Temperature[3] = BQ769x2_ReadTemperature(HDQTemperature);
//	  Temperature[4] = BQ769x2_ReadTemperature(DCHGTemperature);
	  Temperature[5] = BQ769x2_ReadTemperature(CFETOFFTemperature);
	  Temperature[6] = BQ769x2_ReadTemperature(IntTemperature);
	  HAL_Delay(200);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RST_SHUT_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DFETOFF_GPIO_Port, DFETOFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_SHUT_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RST_SHUT_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DDSG_Pin DCHG_Pin */
  GPIO_InitStruct.Pin = DDSG_Pin|DCHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DFETOFF_Pin */
  GPIO_InitStruct.Pin = DFETOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DFETOFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALERT_Pin */
  GPIO_InitStruct.Pin = ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



