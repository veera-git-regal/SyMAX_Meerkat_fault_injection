/**
  ***************************************************************************************************
  * @file    module_modbus.c
  * @author  Regal Myron Mychal/Doug Oda
  * @version V1.0
  * @date    13-April-2022
  * @brief   Modbus message parsing and handling
  * @note    
  ***************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart2.h"
#include "module_modbus.h"
#include "module_modbus_drive_map.h"
#include "main.h"
#include "zz_module_flash.h"
#include "module_dynamic.h"


/******************************************************************************/
/**********************Private function protoypes******************************/

void AssignModuleMemModbus(void);
void Modbus_ParseReceivedMessages(void);
void Modbus_ProcessCommands(void);
void Modbus_SendException(MBErrorCode exception, uint8_t function);
int8_t findMessageLenght(void);
uint8_t validChecksumHasBeenFound(void);
void ReadHoldingAndInputRegisters(void);
void WriteSingleHoldingRegister(void);
void WriteMultipleHoldingRegisters(void);
void InitSettingsModbus(void);
void Modbus_LoadDriveIdData(void);

MBErrorCode ProcessMBHoldingRegister(uint8_t *data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16,MBRegisterMode eMode);
ModbusMapBlock *find_modbus_block(uint16_t desired_address_u16, uint8_t number_of_registers_u8);

extern uint16_t meerkatCore_MajorVersion_u16;
extern uint16_t meerkatCore_MedianVersion_u16;
extern uint16_t meerkatCore_MinorVersion_u16;

uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16);
//extern uint16_t meerkatCore_MajorVersion_u16;
//extern uint16_t meerkatCore_MedianVersion_u16;
//extern uint16_t meerkatCore_MinorVersion_u16;

/******************************************************************************/
/*********************Private constant definitions*****************************/
#define NUMBER_OF_VALID_MODBUS_COMMANDS 4

#define VARIABLE_LENGTH_DESIGNATOR 13

#define NUM_MODBUS_REGISTERS_100s 17
#define NUM_MODBUS_REGISTERS_200s 19
#define NUM_MODBUS_REGISTERS_300s 4
#define NUM_MODBUS_REGISTERS_600s 3
#define NUM_MODBUS_REGISTERS_1000s 3
#define NUM_MODBUS_REGISTERS_5000s 1 // 49

static const Modbus_Commands ModbusCommands[NUMBER_OF_VALID_MODBUS_COMMANDS] =	
{
  {MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS,         8,                              (P_Task)ReadHoldingAndInputRegisters},	
  {MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS,           8,                              (P_Task)ReadHoldingAndInputRegisters},	  	
  {MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER,  8,                              (P_Task)WriteSingleHoldingRegister},
  {MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS,        VARIABLE_LENGTH_DESIGNATOR,     (P_Task)WriteMultipleHoldingRegisters},	
};

enum 
{
  MEM_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

/******************************************************************************/
/**********************Private variable definitions****************************/


ModbusMapBlock BlockDriveData_Inputs =
{
  (uint16_t*)(&(dynamic_data.driveData_Data)), //0,                                           // first data value in block
  MODULE_DRIVE_DATA_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_DRIVE_DATA_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_DRIVE_DATA_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockDriveData_Holdings =
{
  (uint16_t*)(&(dynamic_data.driveData_Settings)), //0,				               // first data value in block
  MODULE_DRIVE_DATA_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_DATA_NUMBER_OF_HOLDINGS,	       // number of elements in block					
  MODULE_DRIVE_DATA_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockModbusRtu_Inputs =
{
  (uint16_t*)(&(modbusRtu_Control.modbusRtu_Data)), //0,                                           // first data value in block
  MODULE_DRIVE_MODBUS_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_MODBUS_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_MODBUS_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockModbusRtu_Holdings =
{
  (uint16_t*)(&(modbusRtu_Control.modbusRtu_Settings)), //0,				                 // first data value in block  
  MODULE_DRIVE_MODBUS_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_MODBUS_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_DRIVE_MODBUS_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockId_Inputs =
{
  (uint16_t*) &(motorId_Control.motorId_Data), //0,                                         // first data value in block
  MODULE_DRIVE_ID_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_DRIVE_ID_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_DRIVE_ID_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockId_Holdings =
{
  (uint16_t*) &(motorId_Control.motorId_Settings), //0,				             // first data value in block
  MODULE_DRIVE_ID_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_ID_NUMBER_OF_HOLDINGS,	     // number of elements in block					
  MODULE_DRIVE_ID_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockStartUp_Inputs =
{
  (uint16_t*) &(startupParameters_Control.startupParameters_Data), //0,                                         // first data value in block
  MODULE_START_UP_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_START_UP_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_START_UP_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockStartUp_Holdings =
{
  (uint16_t*) &(startupParameters_Control.startupParameters_Settings), //0,				             // first data value in block
  MODULE_START_UP_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_START_UP_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_START_UP_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockMotorParam_Inputs =
{
  (uint16_t*) &(motorParameters_Control.motorParameters_Data), //0,                                            // first data value in block
  MODULE_MOTOR_PARAM_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_MOTOR_PARAM_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_MOTOR_PARAM_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockMotorParam_Holdings =
{
  (uint16_t*) &(motorParameters_Control.motorParameters_Settings), //0,				                // first data value in block
  MODULE_MOTOR_PARAM_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_MOTOR_PARAM_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_MOTOR_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockTuning01Param_Inputs =
{
  (uint16_t*) &(motorTunning01_Control.motorTunning01_Data), //0,                                             // first data value in block
  MODULE_TUNING01_PARAM_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_TUNING01_PARAM_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_TUNING01_PARAM_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockTuning01Param_Holdings =
{
  (uint16_t*) &(motorTunning01_Control.motorTunning01_Settings), //0,				                 // first data value in block
  MODULE_TUNING01_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_TUNING01_PARAM_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_TUNING01_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockTuning02Param_Inputs =
{
  (uint16_t*) &(motorTunning02_Control.motorTunning02_Data), //0,                                             // first data value in block
  MODULE_TUNING02_PARAM_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_TUNING02_PARAM_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_TUNING02_PARAM_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockTuning02Param_Holdings =
{
  (uint16_t*) &(motorTunning02_Control.motorTunning02_Settings), //0,				                 // first data value in block
  MODULE_TUNING02_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_TUNING02_PARAM_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_TUNING02_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockLimits_Inputs =
{
  (uint16_t*) &(motorLimits01_Control.motorLimits01_Data), //0,                                           // first data value in block
  MODULE_DRIVE_LIMITS01_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_LIMITS01_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_LIMITS01_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockLimits_Holdings =
{
  (uint16_t*) &(motorLimits01_Control.motorLimits01_Settings), //0,				                 // first data value in block
  MODULE_DRIVE_LIMITS01_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_LIMITS01_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_DRIVE_LIMITS01_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockProtections01_Inputs =
{
  (uint16_t*) &(motorProtections01_Control.motorProtections01_Data), //0,                                                  // first data value in block
  MODULE_DRIVE_PROTECTIONS01_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_DRIVE_PROTECTIONS01_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_DRIVE_PROTECTIONS01_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockProtections01_Holdings =
{
  (uint16_t*) &(motorProtections01_Control.motorProtections01_Settings), //0,				                      // first data value in block
  MODULE_DRIVE_PROTECTIONS01_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_PROTECTIONS01_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_DRIVE_PROTECTIONS01_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockOtfParam_Inputs =
{
  (uint16_t*) &(otfParameters_Control.otfParameters_Data), //0,                                          // first data value in block
  MODULE_OTF_PARAM_START_OF_INPUTS_ADDRESS,   // starting address for block
  MODULE_OTF_PARAM_NUMBER_OF_INPUTS,          // number of elements in block				
  MODULE_OTF_PARAM_NUMBER_OF_USER_INPUTS,     // Number of user accessible Inputs
};

ModbusMapBlock BlockOtfParam_Holdings =
{
  (uint16_t*) &(otfParameters_Control.otfParameters_Settings), //0,				              // first data value in block
  MODULE_OTF_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_OTF_PARAM_NUMBER_OF_HOLDINGS,	      // number of elements in block					
  MODULE_OTF_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockBrakingParam_Inputs =
{
  (uint16_t*) &(brakingParameters_Control.brakingParameters_Data), //0,                                            // first data value in block
  MODULE_BRAKING_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_BRAKING_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_BRAKING_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockBrakingParam_Holdings =
{
  (uint16_t*) &(brakingParameters_Control.brakingParameters_Settings), //0,				                  // first data value in block
  MODULE_BRAKING_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_BRAKING_PARAM_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_BRAKING_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


//ModbusMapBlock BlockWindmillParam_Inputs =
//{
//  (uint16_t*) &(windMillingParameters_Control.windMillingParameters_Data), //0,                                             // first data value in block
//  MODULE_WINDMILL_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
//  MODULE_WINDMILL_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
//  MODULE_WINDMILL_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
//};
//
//ModbusMapBlock BlockWindmillParam_Holdings =
//{
//  (uint16_t*) &(windMillingParameters_Control.windMillingParameters_Settings), //0,				                   // first data value in block
//  MODULE_WINDMILL_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
//  MODULE_WINDMILL_PARAM_NUMBER_OF_HOLDINGS,	   // number of elements in block					
//  MODULE_WINDMILL_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
//};


ModbusMapBlock BlockHwSpecific_Inputs =
{
  (uint16_t*) &(hardwareSpecificParameters_Control.hardwareSpecificParameters_Data), //0,                                                // first data value in block
  MODULE_DRIVE_HW_SPECIFIC_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_HW_SPECIFIC_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_HW_SPECIFIC_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockHwSpecific_Holdings =
{
  (uint16_t*) &(hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings), //0,				                      // first data value in block
  MODULE_DRIVE_HW_SPECIFIC_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_HW_SPECIFIC_NUMBER_OF_HOLDINGS,	      // number of elements in block					
  MODULE_DRIVE_HW_SPECIFIC_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockAppSpecific_Inputs =
{
  (uint16_t*) &(applicationSpecificParameters_Control.applicationSpecificParameters_Data), //0,                                                 // first data value in block
  MODULE_DRIVE_APP_SPECIFIC_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_APP_SPECIFIC_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_APP_SPECIFIC_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockAppSpecific_Holdings =
{
  (uint16_t*) &(applicationSpecificParameters_Control.applicationSpecificParameters_Settings), //0,				                       // first data value in block
  MODULE_DRIVE_APP_SPECIFIC_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_APP_SPECIFIC_NUMBER_OF_HOLDINGS,	       // number of elements in block					
  MODULE_DRIVE_APP_SPECIFIC_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockHc01Param_Inputs =
{
  (uint16_t*) &(harmonicCompensation01_Control.harmonicCompensation01_Data), //0,                                        // first data value in block
  MODULE_HC01_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_HC01_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_HC01_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockHc01Param_Holdings =
{
  (uint16_t*) &(harmonicCompensation01_Control.harmonicCompensation01_Settings), //0,				              // first data value in block
  MODULE_HC01_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_HC01_PARAM_NUMBER_OF_HOLDINGS,	      // number of elements in block					
  MODULE_HC01_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockHc02Param_Inputs =
{
  (uint16_t*) &(harmonicCompensation02_Control.harmonicCompensation02_Data), //0, // first data value in block
  MODULE_HC02_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_HC02_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_HC02_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockHc02Param_Holdings =
{
  (uint16_t*) &(harmonicCompensation02_Control.harmonicCompensation02_Settings), //0, // first data value in block
  MODULE_HC02_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_HC02_PARAM_NUMBER_OF_HOLDINGS,	      // number of elements in block					
  MODULE_HC02_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockHc03Param_Inputs =
{
  (uint16_t*) &(harmonicCompensation03_Control.harmonicCompensation03_Data), //0, // first data value in block
  MODULE_HC03_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_HC03_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_HC03_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockHc03Param_Holdings =
{
  (uint16_t*) &(harmonicCompensation03_Control.harmonicCompensation03_Settings), //0, // first data value in block
  MODULE_HC03_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_HC03_PARAM_NUMBER_OF_HOLDINGS,	      // number of elements in block					
  MODULE_HC03_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

ModbusMapBlock BlockFlash_Inputs =
{
  (uint16_t*) (&(moduleFlash_Control.moduleFlash_Data)), //0,  // first data value in block
  MODULE_DRIVE_FLASH_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_FLASH_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_FLASH_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockFlash_Holdings =
{
  (uint16_t*) (&(moduleFlash_Control.moduleFlash_Settings)), //0,  // first data value in block
  MODULE_DRIVE_FLASH_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DRIVE_FLASH_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_DRIVE_FLASH_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockEEPROM_Inputs =
{
  (uint16_t*) (&(moduleEEPROM_Control.moduleEEPROM_Data)), //0, // first data value in block
  MODULE_EEPROM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_EEPROM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_EEPROM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockEEPROM_Holdings =
{
  (uint16_t*) (&(moduleEEPROM_Control.moduleEEPROM_Settings)), //0, // first data value in block
  MODULE_EEPROM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_EEPROM_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_EEPROM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockTest_Inputs =
{
  (uint16_t*) &(moduleTest_Control.moduleTest_Data), //0, // first data value in block
  MODULE_DRIVE_TEST_PARAM_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_DRIVE_TEST_PARAM_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_DRIVE_TEST_PARAM_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockTest_Holdings =
{
  (uint16_t*) &(moduleTest_Control.moduleTest_Settings), //0, // first data value in block
  MODULE_DRIVE_TEST_PARAM_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_DRIVE_TEST_PARAM_NUMBER_OF_HOLDINGS,	     // number of elements in block					
  MODULE_DRIVE_TEST_PARAM_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};


ModbusMapBlock BlockStParam01_Inputs =
{
  (uint16_t*) &(stParameters01_Control.stParameters01_Data), //0, // first data value in block
  MODULE_ST_MC_PARAM01_START_OF_INPUTS_ADDRESS, // starting address for block
  MODULE_ST_MC_PARAM01_NUMBER_OF_INPUTS,        // number of elements in block				
  MODULE_ST_MC_PARAM01_NUMBER_OF_USER_INPUTS,   // Number of user accessible Inputs
};

ModbusMapBlock BlockStParam01_Holdings =
{
  (uint16_t*) &(stParameters01_Control.stParameters01_Settings), //0, // first data value in block
  MODULE_ST_MC_PARAM01_START_OF_HOLDINGS_ADDRESS, // starting address for block
  MODULE_ST_MC_PARAM01_NUMBER_OF_HOLDINGS,        // number of elements in block					
  MODULE_ST_MC_PARAM01_NUMBER_OF_USER_HOLDINGS,   // number of user accessible Inputs
};

// Master modbus block map
#if MODBUS_COILS_DISCRETES_FUNCTION_CODE_ENABLED == TRUE
ModbusCoilMapBlock *masterCoilBlocks[] = 
{
  &BlockDriveData_Coils,         &BlockDriveData_Discretes,
  &BlockModbusRtu_Coils,         &BlockModbusRtu_Discretes,
  &BlockId_Coils,                &BlockId_Discretes,
  &BlockStartUp_Coils,           &BlockStartUp_Discretes,
  &BlockMotorParam_Coils,        &BlockMotorParam_Discretes,
  &BlockTuningParam_Coils,       &BlockTuningParam_Discretes,
  &BlockLimits_Coils,            &BlockLimits_Discretes,
  &BlockProtections_Coils,       &BlockProtections_Discretes,
  //&BlockFanParam_Coils,          &BlockFanParam_Discretes,
  &BlockOtfParam_Coils,          &BlockOtfParam_Discretes,
  &BlockBrakingParam_Coils,      &BlockBrakingParam_Discretes,
  //&BlockWindmillParam_Coils,     &BlockWindmillParam_Discretes,
  &BlockHwSpecific_Coils,        &BlockHwSpecific_Discretes,
  &BlockAppSpecific_Coils,       &BlockAppSpecific_Discretes,
  &BlockTest_Coils,              &BlockTest_Discretes, 
  &BlockHc01Param_Coils,         &BlockHc01Param_Discretes,
  &BlockHc02Param_Coils,         &BlockHc02Param_Discretes,
  &BlockHc03Param_Coils,         &BlockHc03Param_Discretes,
  &BlockEEPROM_Coils,            &BlockEEPROM_Discretes,
  &BlockFlash_Coils,             &BlockFlash_Discretes,
  &BlockStParam01_Coils,         &BlockStParam01_Discretes,
};
#endif

#define NUMBER_OF_MASTER_BLOCKS 42
ModbusMapBlock *masterBlocks[NUMBER_OF_MASTER_BLOCKS] = 
{    
  &BlockDriveData_Inputs,        &BlockDriveData_Holdings,      //index 0 msb0
  &BlockModbusRtu_Inputs,        &BlockModbusRtu_Holdings,      //index 1 msb1
  &BlockId_Inputs,               &BlockId_Holdings,             //index 2 msb2
  &BlockStartUp_Inputs,          &BlockStartUp_Holdings,        //index 3 msb3
  &BlockMotorParam_Inputs,       &BlockMotorParam_Holdings,     //index 4 msb4
  &BlockTuning01Param_Inputs,    &BlockTuning01Param_Holdings,  //index 5 msb5
  &BlockTuning02Param_Inputs,    &BlockTuning02Param_Holdings,  //index 6 msb6
  &BlockLimits_Inputs,           &BlockLimits_Holdings,         //index 7 msb7
  &BlockProtections01_Inputs,    &BlockProtections01_Holdings,  //index 8 msb9
  &BlockOtfParam_Inputs,         &BlockOtfParam_Holdings,       //index 9 msb13
  &BlockBrakingParam_Inputs,     &BlockBrakingParam_Holdings,   //index 10 msb12
  &BlockHwSpecific_Inputs,       &BlockHwSpecific_Holdings,     //index 11 msb14
  &BlockAppSpecific_Inputs,      &BlockAppSpecific_Holdings,    //index 12 msb15
  &BlockTest_Inputs,             &BlockTest_Holdings,           //index 13 msb21
  &BlockHc01Param_Inputs,        &BlockHc01Param_Holdings,      //index 14 msb16
  &BlockHc02Param_Inputs,        &BlockHc02Param_Holdings,      //index 15 msb17
  &BlockHc03Param_Inputs,        &BlockHc03Param_Holdings,      //index 16 msb18
  &BlockEEPROM_Inputs,           &BlockEEPROM_Holdings,         //index 17 msb20
  &BlockFlash_Inputs,            &BlockFlash_Holdings,          //index 18 msb19
  &BlockStParam01_Inputs,        &BlockStParam01_Holdings,      //index 19 msb24
};

//the index into this array is the msb of the modbus address
//the numbe contained in this array is the index/2 for Inputs and (index/2)+1 for holding
const uint8_t masterBlockIndex[] = 
{
  0,  //0 MODULE_DRIVE_DATA_GROUP/
  1,  //1 MODULE_DRIVE_MODBUS_GROUP/
  2,  //2 MODULE_DRIVE_ID_GROUP/
  3,  //3 MODULE_START_UP_GROUP/
  4,  //4 MODULE_MOTOR_PARAM_GROUP/
  5,  //5 MODULE_TUNING01_PARAM_GROUP/
  6,  //6 MODULE_TUNING02_PARAM_GROUP/ 
  7,  //7 MODULE_DRIVE_LIMITS01_GROUP/
  255,//8 no module with modbus address msb of 8
  8,  //9 MODULE_DRIVE_PROTECTIONS01_GROUP
  255,//10 no module with modbus address msb of 10
  255,//11 no module with modbus address msb of 11
  10, //12 MODULE_BRAKING_PARAM_GROUP
  9,  //13 MODULE_OTF_PARAM_GROUP
  11, //14 MODULE_DRIVE_HW_SPECIFIC_GROUP
  12, //15 MODULE_DRIVE_APP_SPECIFIC_GROUP
  14, //16 MODULE_HC01_PARAM_GROUP
  15, //17 MODULE_HC02_PARAM_GROUP
  16, //18 MODULE_HC03_PARAM_GROUP
  18, //19 MODULE_DRIVE_FLASH_GROUP
  17, //20 MODULE_EEPROM_GROUP
  13, //21 MODULE_DRIVE_TEST_PARAM_GROUP
  255,//22 no module with modbus address msb of 22
  255,//23 no module with modbus address msb of 23
  19  //24 MODULE_ST_MC_PARAM01_GROUP
};
  
 
  
  
uint8_t modbus_command_index_u8 = 0;
uint32_t modbus_message_length_u32 = 0;
uint16_t write_single_holding_register_previous_checksum = 0;
uint16_t write_multiple_holding_register_previous_checksum = 0;
uint16_t current_incoming_message_checksum = 0;
uint16_t number_of_modbus_blocks_u16 = sizeof(masterBlocks) / sizeof(masterBlocks[0]);
uint8_t current_modbus_address = DEFAULT_MODBUS_ADDRESS;

extern ProcessInfo processInfoTable[];
extern ModuleTest_Control moduleTest_Control;
extern MotorId_Control motorId_Control;
Usart2_Control *Control_Modbus;
ModbusRtu_Control modbusRtu_Control;
ModuleFlash_Control* flash_modbusControl; 

// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH USART2_TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
uint8_t fixedModbus_ProtocolBufRX_Length = 0;
uint8_t fixedModbus_ProtocolBufRX[FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufRX = fixedModbus_ProtocolBufRX;

// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH USART2_TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
uint32_t modbus_response_message_length_u32 = 0;
uint8_t modbus_response_message_u8[FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufTX = modbus_response_message_u8;


/******************************************************************************/
/**********************Public Method Implementation****************************/


/**
********************************************************************************
* @brief   Periodic function used to receive, process and send modbus communications
* @details 
* @param   drv_id_u8 The var used by the sceduler to identify this module
*          prev_state_u8 Unused by this module
*          next_state_u8 State to be executed INIT_MODULE, RUN_MODULE or KILL_MODULE
*          irq_id_u8 Unused by this module
* @return  Current State INIT_MODULE, RUN_MODULE or KILL_MODULE
* @note Unused parameters are maintained because this is called by a pointer whose
*       definition has four uint8_t parameters.
********************************************************************************
*/
uint8_t moduleModbus(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEM_INIT_MODULE;
  switch (next_state_u8) 
  {
  case MEM_INIT_MODULE:
    {
      uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2);              //return Process index from processInfo array with the Uart2 driver
      Control_Modbus = (Usart2_Control*) ((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
     
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      uint8_t module_flash_Index = getProcessInfoIndex(MODULE_FLASH);
      flash_modbusControl = (ModuleFlash_Control*)((*(processInfoTable[module_flash_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      InitSettingsModbus();    
      Modbus_LoadDriveIdData();
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case RUN_MODULE: 
    {
      if ( RingBuf_GetUsedNumOfElements((*Control_Modbus).seqMemModbus) > 1 ) {
        //extract message if there is data
        Modbus_ParseReceivedMessages();
      }
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case KILL_MODULE: 
    {
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_MODULE;
      break;
    }
  default: 
    {
      return_state_u8 = KILL_MODULE;
      break;
    }
  }
  return return_state_u8;
}


/******************************************************************************/
/**********************Private Method Implementation***************************/
/**
********************************************************************************
* @brief   Init Modbus Settings
* @details 
* @param   None 
* @return  None
********************************************************************************
*/
void InitSettingsModbus(void) 
{ 
  if( ((*flash_modbusControl).moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess == FALSE) &&
      ((*flash_modbusControl).moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess == FALSE) )
  { // If setting from FLASH are not copied to RAM, use the below default value
    modbusRtu_Control.modbusRtu_Settings.baudRate_u16 = (uint16_t)(DEFAULT_BAUD_RATE/((float)BAUD_RATE_FACTOR)); // baud rate = 1152*BAUD_RATE_FACTOR = 115200
    modbusRtu_Control.modbusRtu_Settings.dataBits_u16 = 8;
    modbusRtu_Control.modbusRtu_Settings.stopBits_u16 = 1;  
    modbusRtu_Control.modbusRtu_Settings.parity_u16 = 0;      // parity 0 = NONE, 1 = ODD; 2 = EVEN
    modbusRtu_Control.modbusRtu_Settings.driveModbusGlobalAddress_u16 = DEFAULT_GLOBAL_MODBUS_ADDRESS; // motor's MDBUS follower address
    modbusRtu_Control.modbusRtu_Settings.driveModbusAddress_u16 = DEFAULT_MODBUS_ADDRESS;
    modbusRtu_Control.modbusRtu_Settings.driveCurrentModbusAddress_u16 = DEFAULT_MODBUS_ADDRESS;
    modbusRtu_Control.modbusRtu_Settings.driveModbusAdminAddress = DEFAULT_MODBUS_ADMIN_ADDRESS;
  }
  current_modbus_address = modbusRtu_Control.modbusRtu_Settings.driveModbusAddress_u16;
  modbusRtu_Control.modbusRtu_Settings.driveCurrentModbusAddress_u16 = current_modbus_address;
  if(current_modbus_address > MAX_VALID_MODBUS_ADDRESS)
  {
    current_modbus_address = MAX_VALID_MODBUS_ADDRESS;
    modbusRtu_Control.modbusRtu_Settings.driveModbusAddress_u16 = current_modbus_address;
    modbusRtu_Control.modbusRtu_Settings.driveCurrentModbusAddress_u16 = current_modbus_address;    
  }
}


/**10/18/21
********************************************************************************
* @brief   Parse incoming buffer and process if message found
* @param   None 
* @return  None
* @note !NOTE! message must be verified by calling function. When this was written
*        it is called as part of a UP message that has a checksum.
********************************************************************************
*/

void Modbus_ParseReceivedMessages(void)
{
  int8_t evaluation_s8 = 0;
  modbus_message_length_u32 = MODBUS_MIN_RX_MESSAGE_LEN;
  while (RingBuf_GetUsedNumOfElements((*Control_Modbus).seqMemModbus) >= modbus_message_length_u32){
    //look for slave address followed by a valid command and enough of the message to extract a lenght
    evaluation_s8 = findMessageLenght(); 
    switch ( evaluation_s8 ){
    case 0: {
      //need more characters so leave the while loop
      modbus_message_length_u32 = USART2_TX_RX_BUF_SIZE + 1;
      break;
    }
    case 1: {
      //start of message found(?), now look for a valid checksum
      if ( validChecksumHasBeenFound() ) {
        //valid checksum so process the message if the message
        RingBuf_ClearContents((*Control_Modbus).seqMemTX);
        RingBuf_ReadBlock((*Control_Modbus).seqMemModbus,fixedModbus_ProtocolBufRX, &modbus_message_length_u32); 
        ModbusCommands[modbus_command_index_u8].p_task(); 
        RingBuf_ClearContents((*Control_Modbus).seqMemModbus);
        RingBuf_ClearContents((*Control_Modbus).seqMem_RawRx);
        break;
      } 
    }
    //fall through if invalid checksum
    case -1: {
      //BAD checksum or incorrect message lenght so remove the first character which is the erroneous slave address and evaluate again
      uint32_t number_of_characters_to_discard_u32 = 1;
      uint8_t temporary_buffer_u8[1];
      RingBuf_ReadBlock((*Control_Modbus).seqMemModbus, temporary_buffer_u8, &number_of_characters_to_discard_u32 ); 
      modbus_message_length_u32 = MODBUS_MIN_RX_MESSAGE_LEN;
      break;
    }
    case -2:
    default:
      RingBuf_ClearContents((*Control_Modbus).seqMemModbus);
      modbus_message_length_u32 = MODBUS_MIN_RX_MESSAGE_LEN;

      break;
    }
  }
}

/**10/15/21
********************************************************************************
* @brief   Build and store an exception response
* @param   None 
* @return  None
********************************************************************************
*/
void Modbus_SendException(MBErrorCode exception, uint8_t function_u8){
  //build the message and save it in the correct place
  modbus_response_message_u8[BYTE_0] = current_modbus_address; // REVIEW //slave_address_index_u32;
  modbus_response_message_u8[BYTE_1] = function_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
  modbus_response_message_u8[BYTE_2] = (uint8_t) exception;
  uint16_t crc_u16 = calculateModbusCRC(&modbus_response_message_u8[0],3);  
  modbus_response_message_u8[BYTE_3] = (uint8_t)(crc_u16);
  modbus_response_message_u8[BYTE_4] = (uint8_t)(crc_u16 / BYTE_TO_WORD_MSB); 
  modbus_response_message_length_u32 = EXCEPTION_RESPONSE_LENGTH;
  RingBuf_WriteBlock((*Control_Modbus).seqMemTX, modbus_response_message_u8 ,&modbus_response_message_length_u32 );
}

/**10/15/21
********************************************************************************
* @brief   Sort through buffer and find the length of the message
* @details Function the entire buffer for the slave address and discards any
*          data before the slave address. It then checks for a valid command
*          code in the next byte if an address is not followed by valid command
*          then return -1.
*          Next save the message lenght for message types with fixed lenghts or
*          read enough of the message to determine the message lenght for 
*          variable lenght messages. 
*          Get more of the message until the entire message has arrived  by 
*          returning a 0. Return a 1 when entire is received.
* @param   None 
* @return  0 no errors found but need more characters
*          1 address,command and lenght are viable
*          -1 bad sequence of characters found
*          -2 all data bad
********************************************************************************
*/
int8_t findMessageLenght(void)
{
  uint32_t number_of_characters_u32 = 0;
  uint8_t temporary_buffer_u8[USART2_TX_RX_BUF_SIZE];
  uint32_t number_of_used_elements_u32 = RingBuf_GetUsedNumOfElements((*Control_Modbus).seqMemModbus);
  
  //get enough of the message to get address and function code and identify MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS
  number_of_characters_u32 = MODBUS_MIN_RX_MESSAGE_LEN;
  RingBuf_Observe((*Control_Modbus).seqMemModbus, temporary_buffer_u8,0,&number_of_characters_u32);
  if ( number_of_characters_u32 < MODBUS_MIN_RX_MESSAGE_LEN){
    //nneed more characters
    return 0;
  }
  
  //save the modbus address (if this is a correct message)
  current_modbus_address = temporary_buffer_u8[0];
  
  //check if byte is a valid function code
  for ( uint8_t index_u8 = 0; index_u8 < NUMBER_OF_VALID_MODBUS_COMMANDS; index_u8++ ){
    if (ModbusCommands[index_u8].command_u8 == temporary_buffer_u8[1]){
      modbus_command_index_u8 = index_u8;
      if ( temporary_buffer_u8[1] == MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS ){
        
        //ok the byte count must be an even number or we are barking up the wrong tree
        if (( temporary_buffer_u8[6] % 2 ) == 1 ){
          //this is  not  correct, kill first character and try again.
          return -1;
        }          
        //The magic number "9" below is address(1)+functioncode(1)+firstAddress(2)+numberToWrite(2)+dataByteCounter(1)+checksum(2)
        //The magic number "11" below is the minimum lenght of write holding registers 
        modbus_message_length_u32 = (temporary_buffer_u8[6] + 9);
        if ( modbus_message_length_u32 < 11 ){
          return -1;
        }
      } else {  
        //not MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS so just use the fixed lenght in the table
        modbus_message_length_u32 = ModbusCommands[modbus_command_index_u8].command_length_u8;
      }
      if (number_of_used_elements_u32 >= modbus_message_length_u32){
        //process
        return 1;
      } else {
        //get more characters
        return 0;
      }           
    }
  }
  return -1;
}

/**10/18/21
********************************************************************************
* @brief   Verify checksum if the full message is present
* @details Function checks if full message is present, it then checks the
*          checksum. If the checksum fails then it removes the first character
*          which is what was believed to be the slave address so that
*          the entire process can start again
* @param   None 
* @return  True if valid checksum, false if full message not present or bad CRC
********************************************************************************
*/
uint8_t validChecksumHasBeenFound(void)
{
  uint8_t return_value_u8 = FALSE;
  uint8_t temporary_buffer_u8[USART2_TX_RX_BUF_SIZE];
  uint32_t number_of_characters_to_read_u32 = 0;
  number_of_characters_to_read_u32 = modbus_message_length_u32;
  RingBuf_Observe((*Control_Modbus).seqMemModbus, temporary_buffer_u8,0,&number_of_characters_to_read_u32);
  if ( number_of_characters_to_read_u32 == modbus_message_length_u32 ){
    //calculate checksum
    current_incoming_message_checksum = calculateModbusCRC(temporary_buffer_u8, (modbus_message_length_u32-2));
    if ( (uint8_t)(current_incoming_message_checksum) == temporary_buffer_u8[modbus_message_length_u32 - 2] &&
        (uint8_t)(current_incoming_message_checksum / BYTE_TO_WORD_MSB) == temporary_buffer_u8[modbus_message_length_u32 - 1] ){
          return_value_u8 = TRUE;
        }
  }  
  return return_value_u8;  
}

 /**10/18/21
********************************************************************************
* @brief   Read holding and input registers 
* @param   None 
* @return  None
********************************************************************************
*/                                                                                                                                     
void ReadHoldingAndInputRegisters(void){ 
  uint16_t register_address_u16 = fixedModbus_ProtocolBufRX[BYTE_2]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_3]*BYTE_TO_WORD_LSB;
  uint16_t number_of_registers_u16 = fixedModbus_ProtocolBufRX[BYTE_4]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_5]*BYTE_TO_WORD_LSB;  
  
  MBErrorCode exception = ProcessMBHoldingRegister(&modbus_response_message_u8[READ_RESPONSE_HEADER_LENGTH], register_address_u16, number_of_registers_u16, MB_READ_REGISTER);
  if (exception != MB_NO_ERROR) {
  // send exception code for bad address
    Modbus_SendException(exception,fixedModbus_ProtocolBufRX[BYTE_1]);				
  } else {	
    // get the length
    modbus_response_message_length_u32 = READ_RESPONSE_HEADER_LENGTH + 2*number_of_registers_u16 + CRC_LENGTH;
    if ( modbus_response_message_length_u32 <= FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH){
      // build header
      modbus_response_message_u8[BYTE_0] = current_modbus_address;
      modbus_response_message_u8[BYTE_1] = fixedModbus_ProtocolBufRX[BYTE_1];
      modbus_response_message_u8[BYTE_2] = (uint8_t) (2*number_of_registers_u16);  // 2 bytes per registers				
      // build CRC
      uint16_t crc_calculated_u16 = calculateModbusCRC(modbus_response_message_u8, modbus_response_message_length_u32 - 2);
      modbus_response_message_u8[modbus_response_message_length_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);  // crc LSB
      modbus_response_message_u8[modbus_response_message_length_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);  // crc MSB
      RingBuf_WriteBlock((*Control_Modbus).seqMemTX, modbus_response_message_u8 ,&modbus_response_message_length_u32 );  
    } else {
      Modbus_SendException(MB_ILLEGAL_ARGUMENT,fixedModbus_ProtocolBufRX[BYTE_1]);
    }
  }  
}
                        
 /**10/18/21
********************************************************************************
* @brief   Write single holding register and update motor demand if changed
* @param   None 
* @return  None
********************************************************************************
*/    
void WriteSingleHoldingRegister(void){
  uint16_t register_address_u16 = fixedModbus_ProtocolBufRX[BYTE_2]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_3]*BYTE_TO_WORD_LSB;
  uint16_t register_value_u16 = fixedModbus_ProtocolBufRX[BYTE_4]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_5]*BYTE_TO_WORD_LSB;  
  
  MBErrorCode exception = ProcessMBHoldingRegister(&fixedModbus_ProtocolBufRX[WRITE_SINGLE_REGISTER_HEADER_LENGTH], register_address_u16, 1, MB_WRITE_REGISTER);
  if (exception != MB_NO_ERROR) {
  // send exception code for bad address
    Modbus_SendException(exception,MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER);				
  } else {				
    // get the length
    modbus_response_message_length_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
    //build message
    modbus_response_message_u8[BYTE_0] = current_modbus_address;
    modbus_response_message_u8[BYTE_1] = MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER;
    modbus_response_message_u8[BYTE_2] = (uint8_t) (register_address_u16 >> 8);  // starting register MSB/LSB
    modbus_response_message_u8[BYTE_3] = (uint8_t) (register_address_u16 >> 0);	
    modbus_response_message_u8[BYTE_4] = (uint8_t) (register_value_u16 >> 8);			
    modbus_response_message_u8[BYTE_5] = (uint8_t) (register_value_u16 >> 0);				
    // build CRC
    uint16_t crc_calculated_u16 = calculateModbusCRC(modbus_response_message_u8, modbus_response_message_length_u32 - 2);
    modbus_response_message_u8[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);  // crc LSB
    modbus_response_message_u8[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);  // crc MSB
    RingBuf_WriteBlock((*Control_Modbus).seqMemTX, modbus_response_message_u8 ,&modbus_response_message_length_u32 );  
    
    if ( write_multiple_holding_register_previous_checksum != current_incoming_message_checksum && 
                       current_modbus_address == DEFAULT_MODBUS_ADMIN_ADDRESS ){
       write_single_holding_register_previous_checksum = current_incoming_message_checksum;
       if( (register_address_u16 >= MODULE_DRIVE_DATA_START_OF_HOLDINGS_ADDRESS) && (register_address_u16 < MODULE_DRIVE_DATA_END_OF_GROUP) )
       { // Check if the received message address is Drive Dynamic Data address
         UpdateMotorDemand();
       } 
    }
  }
}


/**10/18/21
********************************************************************************
* @brief   Write multiple holding registers
* @param   None 
* @return  None
********************************************************************************
*/    
void WriteMultipleHoldingRegisters(void){      
  uint16_t register_address_u16 = fixedModbus_ProtocolBufRX[BYTE_2]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_3]*BYTE_TO_WORD_LSB;
  uint16_t number_of_registers_u16 = fixedModbus_ProtocolBufRX[BYTE_4]*BYTE_TO_WORD_MSB + fixedModbus_ProtocolBufRX[BYTE_5]*BYTE_TO_WORD_LSB;  
  
  MBErrorCode exception = ProcessMBHoldingRegister(&fixedModbus_ProtocolBufRX[WRITE_MULTIPLE_REGISTER_HEADER_LENGTH], register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER);
  if (exception != MB_NO_ERROR) {
  // send exception code for bad address
    Modbus_SendException(exception,MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS);				
  } else {				
    //get length
    modbus_response_message_length_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
    // build header
    modbus_response_message_u8[BYTE_0] = current_modbus_address;
    modbus_response_message_u8[BYTE_1] = MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS;
    modbus_response_message_u8[BYTE_2] = (uint8_t) (register_address_u16 >> 8);  // starting register MSB/LSB
    modbus_response_message_u8[BYTE_3] = (uint8_t) (register_address_u16 >> 0);	
    modbus_response_message_u8[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);  // number of registers written MSB/LSB
    modbus_response_message_u8[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
    // build CRC
    uint16_t crc_calculated_u16 = calculateModbusCRC(modbus_response_message_u8, modbus_response_message_length_u32 - 2);
    modbus_response_message_u8[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);  // crc LSB
    modbus_response_message_u8[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);  // crc MSB
    RingBuf_WriteBlock((*Control_Modbus).seqMemTX, modbus_response_message_u8 ,&modbus_response_message_length_u32 ); 

    if ( write_multiple_holding_register_previous_checksum != current_incoming_message_checksum && 
                      current_modbus_address == DEFAULT_MODBUS_ADMIN_ADDRESS ){
      write_multiple_holding_register_previous_checksum = current_incoming_message_checksum;
     if( (register_address_u16 >= MODULE_DRIVE_DATA_START_OF_HOLDINGS_ADDRESS) && (register_address_u16 < MODULE_DRIVE_DATA_END_OF_GROUP) )
      { // Check if the received message address is Drive Dynamic Data address
        UpdateMotorDemand();
      }
    }
  } 
}


/**
********************************************************************************
* @brief   Process holding register command
* @details 
* @param   uint8_t *data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16,
                                     MBRegisterMode eMode 
* @return  MBErrorCode
********************************************************************************
*/
MBErrorCode ProcessMBHoldingRegister(uint8_t *data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16,
                                     MBRegisterMode eMode) {
                    
    MBErrorCode eStatus = MB_ILLEGAL_ADDRESS;
    ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_registers_u16);

    if (block) {
        //block_1 =(uint16_t) ( block->start_address_u16);
        //offset_1 = starting_address_u16;
        uint16_t index_u16 = starting_address_u16 - block->start_address_u16;

        while (number_of_registers_u16 > 0) {
            if (eMode == MB_WRITE_REGISTER) {
                block->start_of_data_pu16[index_u16] = ((uint16_t) *data_buffer_pu8++) << 8;
                block->start_of_data_pu16[index_u16] |= (uint16_t) *data_buffer_pu8++;
            } else {
                *data_buffer_pu8++ = (uint8_t) (block->start_of_data_pu16[index_u16] >> 8);
                *data_buffer_pu8++ = (uint8_t) (block->start_of_data_pu16[index_u16] & 0xff);
            }
            ++index_u16;
            --number_of_registers_u16;
        }
        eStatus = MB_NO_ERROR;
    }
    return eStatus;
}

//
// Given a modbus start address and number of registers,
// return the mapping block that contains those values, or 0 (null) if no match
//
/**
********************************************************************************
* @brief   Get modbus block address 
* @details 
* @param   uint16_t desired_address_u16, uint16_t number_of_registers_u8, uint8_t block number
* @return  block: The mapping block that contains the values
*              0: Null block addresses are not valid
********************************************************************************
*/
ModbusMapBlock *find_modbus_block(uint16_t desired_address_u16, uint8_t number_of_registers_u8) {
  
  uint8_t addressMsByte = (uint8_t)(desired_address_u16 >> 8);
  uint8_t addressLsByte = (uint8_t)(desired_address_u16 & 0x00FF);
  uint8_t blockIndex_u8 = masterBlockIndex[addressMsByte] * 2;

  if ( addressLsByte  >= (uint8_t)104 ){
    blockIndex_u8 = (masterBlockIndex[addressMsByte] * 2) + 1;
  }
  
  if ( blockIndex_u8 < NUMBER_OF_MASTER_BLOCKS){
    ModbusMapBlock *block = masterBlocks[blockIndex_u8];
    if (moduleTest_Control.moduleTest_Data.userDiscretes01.is_adminMode  || 
                              current_modbus_address == DEFAULT_MODBUS_ADMIN_ADDRESS){
       if ((block->start_address_u16 <= desired_address_u16) &&
          (desired_address_u16 + number_of_registers_u8 <= block->start_address_u16 + block->number_of_registers_u8))
          return block;       
    }else{
      if ((block->start_address_u16 <= desired_address_u16) &&
          (desired_address_u16 + number_of_registers_u8 <= block->start_address_u16 + block->user_access_data_count_u8))
          return block;
    }
  }
  return 0;
}


// Compute the MODBUS RTU CRC
/**
********************************************************************************
* @brief   Calculate CRC
* @details 
* @param   uint8_t *buf_pu8, uint16_t length_u16 
* @return  crc_u16: Calculated CRC
********************************************************************************
*/
uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16) 
{
  uint16_t crc_u16 = 0xFFFF;
  
  for (uint16_t pos_u16 = 0; pos_u16 < length_u16; pos_u16++) {
    crc_u16 ^= (uint16_t) buf_pu8[pos_u16]; // XOR byte into least sig. byte of crc
    
    for (uint8_t index_u8 = 8; index_u8 != 0; index_u8--) { // Loop over each bit
      if ((crc_u16 & 0x0001) != 0) {                      // If the LSB is set
        crc_u16 >>= 1;                                  // Shift right and XOR 0xA001
        crc_u16 ^= 0xA001;
      } else             // Else LSB is not set
        crc_u16 >>= 1; // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc_u16;
}

void Modbus_LoadDriveIdData(void)
{
  motorId_Control.motorId_Data.minorDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MINOR;  // in ASCII YY (VV.XX.YY)
  motorId_Control.motorId_Data.medianDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MEDIAN; // In ASCII XX (VV.XX.YY)
  motorId_Control.motorId_Data.majorDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MAJOR;  // In ASCII VV (VV.XX.YY)
  motorId_Control.motorId_Data.minorDriveFirmwareCrc_u16 = *((uint16_t *)(FIRMWARE_CRC_ADDR));  // Motor code CRC
  motorId_Control.motorId_Data.majorDriveFirmwareCrc_u16 = *((uint16_t *)(FIRMWARE_CRC_ADDR + 2));  // Motor code CRC
  motorId_Control.motorId_Data.minorDriveFlashRev_u16 = *((uint16_t *)FLASH_SETTINGS_VERSION_ADDR); // In ASCII, but each nible only 0-9 values are allowed
  motorId_Control.motorId_Data.majorDriveFlashRev_u16 = *((uint16_t *)(FLASH_SETTINGS_VERSION_ADDR + 2)); // In ASCII, but each nible only 0-9 values are allowed 00VV, XXYY
  motorId_Control.motorId_Data.minorDriveFlashCrc_u16 = *((uint16_t *)FLASH_SETTINGS_CRC_ADDR); // Motor flash settings CRC
  motorId_Control.motorId_Data.majorDriveFlashCrc_u16 = *((uint16_t *)(FLASH_SETTINGS_CRC_ADDR + 2)); // Motor flash settings CRC
  
  motorId_Control.motorId_Data.minorSafetyFirmwareRev_u16 = meerkatCore_MinorVersion_u16;  // in ASCII YY (VV.XX.YY)
  motorId_Control.motorId_Data.medianSafetyFirmwareRev_u16 = meerkatCore_MedianVersion_u16; // In ASCII XX (VV.XX.YY)
  motorId_Control.motorId_Data.majorSafetyFirmwareRev_u16 = meerkatCore_MajorVersion_u16;  // In ASCII VV (VV.XX.YY)
  motorId_Control.motorId_Data.minorSafetyFirmwareCrc_u16 = *((uint16_t *)(SAFETY_FIRMWARE_CRC_ADDR)) ;  // Motor code CRC
  motorId_Control.motorId_Data.majorSafetyFirmwareCrc_u16 = *((uint16_t *)(SAFETY_FIRMWARE_CRC_ADDR + 2));  // Motor code CRC
  //motorId_Control.motorId_Data.minorSafetyFlashRev_u16 = *((uint16_t *)FLASH_SAFETY_SETTINGS_VERSION_ADDR); // In ASCII, but each nible only 0-9 values are allowed
  //motorId_Control.motorId_Data.majorSafetyFlashRev_u16 = *((uint16_t *)(FLASH_SAFETY_SETTINGS_VERSION_ADDR + 2)); // In ASCII, but each nible only 0-9 values are allowed 00VV, XXYY
  motorId_Control.motorId_Data.minorSafetyFlashCrc_u16 = *((uint16_t *)FLASH_SAFETY_SETTINGS_CRC_ADDR); // Motor flash settings CRC
  motorId_Control.motorId_Data.majorSafetyFlashCrc_u16 = *((uint16_t *)(FLASH_SAFETY_SETTINGS_CRC_ADDR + 2)); // Motor flash settings CRC
  
}



