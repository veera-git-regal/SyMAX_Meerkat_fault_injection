/**
  ***************************************************************************************************
  * @file    module_modbus.h
  * @author  Regal Myron Mychal/Doug Oda
  * @version V1.0
  * @date    13-Oct-2021
  * @brief   Modbus message parsing and handling definitions
  * @note    
  ***************************************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_H_
#define _MODULE_MODBUS_H_

/* Includes ------------------------------------------------------------------*/
#include "driver_usart2.h"
#include "scheduler.h"
#include "structured_memory.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/**********************Public constant definitions*****************************/
//#define USE_COIL_REGISTERS  
  
#define DEFAULT_MODBUS_ADDRESS        247 
#define DEFAULT_MODBUS_ADMIN_ADDRESS  252
#define DEFAULT_GLOBAL_MODBUS_ADDRESS 0  

#define MIN_VALID_MODBUS_ADDRESS 0
#define MAX_VALID_MODBUS_ADDRESS 247   


  
#define MODBUS_MIN_RX_MESSAGE_LEN  8
#define MODBUS_MIN_TX_MESSAGE_LEN  5
#define MODBUS_MESSAGE_MAX_LEN     USART1_TX_RX_BUF_SIZE
#define MODBUS_COMMAND_QUE_SIZE    2

// Temporary definitions for testing the module - can be thron away eventually MRM
#define MODBUS_STOP  0
#define MODBUS_START 1

#define BYTE_TO_WORD_MSB 256 // shift of 2^8 = 256 to put into upper byte in a word
#define BYTE_TO_WORD_LSB 1   // shift of 2^0 = 1   to put into lower byte in a word

#define BYTE_0  0 // Byte 0 in a Modbus packet
#define BYTE_1  1
#define BYTE_2  2
#define BYTE_3  3
#define BYTE_4  4
#define BYTE_5  5
#define BYTE_6  6
#define BYTE_7  7
#define BYTE_8  8
#define BYTE_9  9
#define BYTE_10 10
#define BYTE_11 11

#define EXCEPTION_FUNCTION_CODE_OFFSET        128

#define CRC_LENGTH                            2 // crc_hi and crc_lo
#define MINIMUM_RESPONSE_LENGTH               5
#define MINIMUM_WRITE_HEADER_LENGTH           7

#define WRITE_SINGLE_COIL_HEADER_LENGTH       4
#define WRITE_MULTIPLE_COIL_HEADER_LENGTH     7
#define WRITE_SINGLE_REGISTER_HEADER_LENGTH   4
#define WRITE_MULTIPLE_REGISTER_HEADER_LENGTH 7

#define WRITE_RESPONSE_HEADER_LENGTH          6 // slave address, function code, starting address MSB, LSB, Number of total bytes written MSB, LSB
#define READ_HEADER_LENGTH                    6 // slave address, function code, starting address MSB, LSB, Total number of registers read MSB, LSB
#define READ_RESPONSE_HEADER_LENGTH           3 // slave address, function code, number of bytes in response
#define MINIMUM_READ_RESPONSE_LENGTH          7 // slave address, function code, number of registers read, starting address MSB/LSB + CRC low and high
#define MINIMUM_WRITE_RESPONSE_LENGTH         8 // slave address, function code, number of registers written, strting address MSB/LSB + CRC low and high
#define EXCEPTION_RESPONSE_LENGTH             5 // slave address, function code, exception code + CRC LSB and MSB

#define READ_RECORDS_HEADER_LENGTH                                                                                               \
    10 // slave address, function code, # of bytes in message, reference type, file # hi and lo, data address hi and lo, number of data bytes(registers?) hi and low
#define READ_RECORDS_RESPONSE_HEADER_LENGTH   10
#define WRITE_RECORDS_HEADER_LENGTH           1
#define WRITE_RECORDS_RESPONSE_HEADER_LENGTH  1
#define MODBUS_REFERENCE_TYPE                 6 // used for read/write records functions and is always a 6

#define MODBUS_COILS_DISCRETES_FUNCTION_CODE_ENABLED FALSE // TRUE or FALSE

// macro definitions for MODBUS Protocol
#define MODBUS_FUNCTION_CODE_UNKNOWN                       0x00
#define MODBUS_FUNCTION_CODE_READ_COILS                    0x01
#define MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS          0x02
#define MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS        0x03
#define MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS          0x04
#define MODBUS_FUNCTION_CODE_WRITE_COIL                    0x05
#define MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER 0x06
#define MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS          0x0F
#define MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS       0x10
#define MODBUS_FUNCTION_CODE_READ_RECORDS                  0x14
#define MODBUS_FUNCTION_CODE_WRITE_RECORDS                 0x15
#define MODBUS_FUNCTION_CODE_UTILITY                       0xFB
      
// file numbers for read/write records
#define DATA_LOGGER_FILE_NUMBER                            0x00
#define COMPONENT_DATA_FILE_NUMBER                         0x01
#define	APPLICATION_CONFIGURATION_FILE_NUMBER	           0x02
#define	APPLICATION_PROGRAM_FILE_NUMBER			   0x03
#define	DRIVE_CONFIGURATION_FILE_NUMBER			   0x04
#define DRIVE_PROGRAM_FILE_NUMBER                          0x05  
  
// file address minimums
#define DATA_LOGGER_ADDRESS_MIN                            0x0000 
#define DATA_LOGGER_ADDRESS_MAX	                           0x00FF //0x0052 //for GMI EEPROM data logging data is only 82 bytes long
#define	COMPONENT_DATA_ADDRESS_MIN                         0x0100 //0x0053 //make the rest of the EEPROM avaliable and call it component data
#define COMPONENT_DATA_ADDRESS_MAX                         0x0400 //0x4000 
#define DRIVE_CONFIGURATION_ADDRESS_MIN	                   0x0000 //this has an offset of 0x0800E000
#define DRIVE_CONFIGURATION_ADDRESS_MAX	                   0x07FF //with the offset of  0x0800E000 this is  0x0800E7FF      
      

// file numbers for flash files for function code 20/21 messages
#define APP_FLASH_SETTINGS_PAGE          0x01 // file numbers for flash files for function code 20/21 messages
#define APP_FLASH_SETTINGS_MIRROR_PAGE   0x02
#define DRIVE_FLASH_SETTINGS_PAGE        0x03
#define DRIVE_FLASH_SETTINGS_MIRROR_PAGE 0x04

#define MODUBS_FUNCTION_CODE_UTILITY_MIN_MESSAGE_LEN 7 // Address, FunctionCode, D1, D2, D3, cksum, cksum

#define MODBUS_EXCEPTION_01              0x01 // Illegal Function Code
#define MODBUS_EXCEPTION_02              0x02 // Ilegal Data Address
#define MODBUS_EXCEPTION_03              0x03 // Illegal Data Value
#define MODBUS_EXCEPTION_04              0x04 // Slave Device Failure
#define MODBUS_EXCEPTION_05              0x05 // Acknowledge
#define MODBUS_EXCEPTION_06              0x06 // Slave Device Busy

#define MODBUS_COIL_ADDRESS              0x0001
#define MODBUS_COIL_VALUE_ON             0xFF00 // value for coil that is in ON state
#define MODBUS_COIL_VALUE_OFF            0x0000 // value for coil that is in OFF state



// TODO: Move all these extern function declarations to a separate file once we finalize the motor system.
//
//extern void Modbus_UpdateStoredStatus(uint8_t status);
//extern void Modbus_UpdateStoredFaults(uint16_t faults);
//extern void Modbus_UpdateStoredBusVoltage(uint16_t bus_voltage);
////
//extern void Modbus_UpdateStoredDirection(uint8_t direction);
//extern void Modbus_UpdateStoredMeasuredSpeed(int16_t measured_speed);
//extern void Modbus_UpdateStoredTorque(int16_t torque);
////
//extern void Modbus_UpdateStoredPower(int16_t power);
//extern void Modbus_UpdateStoredTemperature(int16_t temperature);
//extern void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter,
//                                     uint8_t function_subparameter); // TODO: Move to own File
//
//extern void Modbus_UpdateStoredRegalMCStatus(uint16_t regal_mc_status_data_u16);
//extern void Modbus_UpdateStoredPhaseCurrentIa(int16_t current_ia_s16);
//extern void Modbus_UpdateStoredPhaseCurrentIb(int16_t current_ib_s16);
//
//extern uint16_t Modbus_GetStoredStatus(void);
//extern uint16_t Modbus_GetStoredMeasuredSpeed(void);
/******************************************************************************/
/*********************Public Typedef definitions*******************************/
// file numbers for flash files for function code 20/21 messages
enum
{
    DATA_LOGGER_FILE_TYPE = 0, // EEPROM
    COMPONENT_DATA_FILE_TYPE,  // EEPROM
    APPLICATION_CONFIGURATION_FILE_TYPE,// FLASH DATA			
    APPLICATION_PROGRAM_FILE_TYPE, //FLASH PROGRAM
    DRIVE_CONFIGURATION_FILE_TYPE, // FLASH DATA
    DRIVE_PROGRAM_FILE_TYPE,  //FLASH PROGRAM
    FILE_TYPE_MAX_VALUE, //invalid file type, all file types must have a value less than this
    FILE_TYPE_MIN_VALUE = 0
};


typedef enum {
    MB_READ_REGISTER, // Read holding register values and pass
    MB_WRITE_REGISTER // Write to holding register values
} MBRegisterMode;

  typedef enum {
    MB_NO_ERROR,         //no error
    MB_ILLEGAL_FUNCTION, //unused, message is not parsed without valid function
    MB_ILLEGAL_ADDRESS,  //illegal register address
    MB_ILLEGAL_ARGUMENT, //illegal register argument
    MB_DEVICE_FAILURE,   //unused
    MB_GATEWAY_PATH_UNAVAILABLE = 0x0A, //unused drive will be connected to application
    MB_TIMED_OUT = 0x0B,        //
  } MBErrorCode;

////////// Start of Modbus RTU Control ////////////////////////////////////////
typedef struct 
{
  // ADMIN Access
  uint16_t is_driveModbusEnable:1;   
  uint16_t unusedFlags02:1; 
  uint16_t unusedFlags03:1; 
  uint16_t unusedFlags04:1;   
  uint16_t unusedFlags05:1; 
  uint16_t unusedFlags06:1; 
  uint16_t unusedFlags07:1; 
  uint16_t unusedFlags08:1; 
  uint16_t unusedFlags09:1;   
  uint16_t unusedFlags10:1; 
  uint16_t unusedFlags11:1;
  uint16_t unusedFlags12:1; 
  uint16_t unusedFlags13:1;
  uint16_t unusedFlags14:1;
  uint16_t unusedFlags15:1;   
  uint16_t unusedFlags16:1;    
} ModbusRtuAdmin_Flags01;

typedef struct
{  
  // USER
  uint16_t unusedDiscretes01:1; //
  uint16_t unusedDiscretes02:1; //
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
  uint16_t unusedDiscretes05:1; 
  uint16_t unusedDiscretes06:1;
  uint16_t unusedDiscretes07:1;
  uint16_t unusedDiscretes08:1;
  uint16_t unusedDiscretes09:1;   
  uint16_t unusedDiscretes10:1;  
  uint16_t unusedDiscretes11:1;   
  uint16_t unusedDiscretes12:1;    
  uint16_t unusedDiscretes13:1; 
  uint16_t unusedDiscretes14:1;
  uint16_t unusedDiscretes15:1;
  uint16_t unusedDiscretes16:1;
} ModbusRtuUser_Discretes01;

// Live Modbus Data
struct ModbusRtu_Data 
{
  // USER Access
  ModbusRtuUser_Discretes01 userDiscretes01_u16;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t messagesReceived_u16; // number of modbus messages received
  uint16_t unusedData13;
  uint16_t unusedData14;
};

// Modbus RTU settings
struct ModbusRtu_Settings 
{
  // USER Access
  uint16_t unusedUserDiscretes01;
  uint16_t unusedSettings02; // RESERVED for Baud Rate if no App micro
  uint16_t unusedSettings03; // RESERVED for data bits if no App micro
  uint16_t unusedSettings04; // RESERVED for stop bits if no App micro
  uint16_t unusedSettings05; // RESERVED for parity if no App micro
  uint16_t unusedSettings06; // RESERVED Modbus global address if no App micro
  uint16_t unusedSettings07; // RESERVED Modbus address if no App micro
  uint16_t unusedSettings08; // 
  uint16_t unusedSettings09; 
  uint16_t unusedSettings10;
  uint16_t unusedSettings11;
  uint16_t unusedSettings12;
  uint16_t unusedSettings13;
  uint16_t unusedSettings14;
  
  // ADMIN Access
  ModbusRtuAdmin_Flags01 adminFlags01_u16;
  uint16_t baudRate_u16;      // baud rate
  uint16_t dataBits_u16;      // data bits
  uint16_t stopBits_u16;      // stop bits
  uint16_t parity_u16;        // parity
  uint16_t driveModbusGlobalAddress_u16; // Universal address drive board will respond to
  uint16_t driveModbusAddress_u16;       // Drive Modbus address to which drive board will respond to for all user registers only
  uint16_t driveCurrentModbusAddress_u16; // Stores modbus address on power up, to prevent updating Modbus address live
  uint16_t driveModbusAdminAddress;      // Drive Modbus address to which drive board will respond to for all registers admin/uere
  uint16_t unusedSettings23;
  uint16_t unusedSettings24;       // Added to prevent padding
};

typedef struct {
    struct ModbusRtu_Settings modbusRtu_Settings;
    struct ModbusRtu_Data modbusRtu_Data;
} ModbusRtu_Control;

////////// END of Modbus RTU Control ////////////////////////////////////////

typedef void (*P_Task)(void);

typedef struct {
    uint8_t command_u8;
    uint8_t command_length_u8;
    P_Task p_task;
} Modbus_Commands;

typedef struct ModbusDataRangeStruct {
    uint16_t minimum_value_u16;
    uint16_t maximum_value_u16;
} ModbusDataRange;

typedef struct ModbusMapBlockStruct {
    uint16_t *start_of_data_pu16;       // a pointer to where the sequential data is stored
    uint16_t start_address_u16;         // the modbus start address
    uint8_t number_of_registers_u8;   // number registers from the start address in this struct
    uint8_t user_access_data_count_u8; // Number of registes that are accessable by user
    //ModbusDataRange data_range_pu16[];  // store the minimum values for the data in this block (sequentially)
    //uint16_t maximum_values_pu16[];	// store the maximum values for the data in this block (sequentially)
} ModbusMapBlock;

#if MODBUS_COILS_DISCRETES_FUNCTION_CODE_ENABLED
typedef struct ModbusCoilMapBlockStruct {
    uint16_t *start_of_coil_data_pu16;     // a pointer to where the sequential data is stored
    uint16_t start_coil_address_u16;       // the modbus start address
    uint8_t number_of_coil_registers_u8; // number registers from the start address in this struct
    uint8_t number_of_coils_u8;          // number registers from the start address in this struct
    uint8_t user_access_coil_count_u8;    // Number of registes that are accessable by user
} ModbusCoilMapBlock;
#endif

typedef struct tag_mbmsg_ctrl {
    uint8_t send_ack : 1; //Send ack response
    uint8_t send_err : 1; //Send err response  -- use bmc_send_err
    uint8_t rx00 : 1;     //Broadcast addr used
    uint8_t noecho : 1;   //Echo is suppressed
    uint8_t spare : 4;    //spare
    uint8_t msg_cmd;      //message command sent by master
    uint8_t err_code;     //code to return
} mb_msg_ctrl_def;


#pragma pack(1)          // .odd.word needs to be offset by one byte.
#define MB_MAXREGS 20    //*Max number of registers to R/W
union mb_resp_parm_def { //Define a structure for response parameters
    struct {
        uint8_t byte;
        uint16_t word[(MB_MAXREGS) -1];
    } odd;
    uint8_t byte[MB_MAXREGS * 2]; //
    uint16_t word[MB_MAXREGS];
};
#pragma pack()

/************Start of Modbus Control (inside shared memory)*********/
//struct Modbus_Settings {
//    uint16_t baudRate_u16;            //4800, 9600, 14400, 19200, 38400, 57600, 115200
//    uint8_t data_Bits_u8;             // 8 or 9
//    uint8_t parity_u8;                //1= Odd; 2 = Even; 0 = None;
//    uint8_t stop_Bits_u8;             //1 or 2
//    uint8_t flow_Control;             //0 = None; 1= Xon/Xoff; 2 = Hardware
//    uint8_t modbus_Slave_Address_u8;  //modbus address
//    uint8_t modbus_Master_Address_u8; //modbus master address
//};
//
//struct Modbus_Data {
//    bool is_first_Valid_Msg; //Flag indicating first valid msg received.
//    bool is_modbus_Activity; //Flag indicating recent serial activity.
//    bool is_admin_Mode;      //Flag indicating enter Administrator Mode
//};
//
//typedef struct {
//    struct Modbus_Settings modbus_Settings;
//    struct Modbus_Data modbus_Data;
//} Modbus_Control;

/***********End of Modbus Control (inside shared memory)***************/

/******************************************************************************/
/*****************Public function prototypes***********************************/

/**
********************************************************************************
* @brief   Periodic function used to receive, process and send modbus communications
* @details 
* @param   drv_id_u8 The var used by the sceduler to identify this module
*          prev_state_u8 Unused by this module
*          next_state_u8 State to be executed MEMORY_INIT_MODULE,INIT_MODULE, RUN_MODULE or KILL_MODULE
*          irq_id_u8 Unused by this module
* @return  Current State INIT_MODULE, RUN_MODULE or KILL_MODULE
* @note Unused parameters are maintained because this is called by a pointer whose
*       definition has four uint8_t parameters.
********************************************************************************
*/
uint8_t moduleModbus(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8);

/**
********************************************************************************
* @brief   Fill out and send the previously started reply for flash or 
*          EEPROM data request using the most recent Universal Protorcol message
* @details 
* @param   uint8_t *data_address_pu 
* @return 
********************************************************************************
*/
void Modbus_PassDriveData(uint8_t *data_address_pu8);
#endif /* _MODULE_MODBUS_H_ */
