/**
  *****************************************************************************
  * @file    module_test.h 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    05-Oct-2021
  * @brief   Header for "Module Test"
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_TEST_H_
#define _MODULE_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "structured_memory.h"
#include "scheduler.h"
#include "zz_module_flash.h"
  
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *module_Test_Settings_StructMem_u32;
extern Ram_Buf *module_Test_Data_StructMem_u32;

#define MAX_TEST_CMD_DATA_LENGTH_IN_WORDS 4  // Words (uint16_t)
#define MAX_PASSWORD_LENGTH_IN_WORDS 4  // Words (uint16_t)

// Operating modes
typedef enum 
{
  ENTER_STD_MODE = 0,     // Non-test mode
  ENTER_ADMIN_MODE,       // Adminstrator mode
  ENTER_TEST_MODE,        // Generic test mode. Includes admin access
  ENTER_HW_FCT_TEST_MODE, // Set to FCT test mode. Includes admin access
  ENTER_HW_ICT_TEST_MODE, // Set to ICT test mode. Includes admin access
  ENTER_HW_BI_TEST_MODE,  // Set to BI test mode. Includes admin access
  ENTER_EOL_TEST_MODE,    // Set to EOL test mode. Includes admin access
  ENTER_SW_TEST_MODE,     // Set to ths for SW debugging test. Includes admin access
  UNKNOWN_OPERATING_MODE = 0xFFFF
}Operating_Modes;

// Operating Status
typedef enum
{
  STD_MODE = 0, 
  OPERATION_MODE_OK, // Normal operating mode 
  PASSWORD_FAIL,    // Entered password wrong
  PASSWORD_PASS,    // Password is accepted
  ADMIN_MODE,       // Adminstrator mode
  TEST_MODE,        // Test mode
  HW_FCT_TEST_MODE, // FCT test mode. Includes admin access
  HW_ICT_TEST_MODE, // ICT test mode. Includes admin access
  HW_BI_TEST_MODE,  // BI test mode. Includes admin access
  EOL_TEST_MODE,    // EOL test mode.
  SW_TEST_MODE,     // Software test mode
  
  ENTER_ADMIN_MODE_FAIL,       // Enter admin mode failed
  ENTER_TEST_MODE_FAIL,        // Faile to enter the test mode
  ENTER_HW_FCT_TEST_MODE_FAIL, // Enter hardware FCT test mode fail
  ENTER_HW_ICT_TEST_MODE_FAIL, // Enter hardware ICT test mode fail
  ENTER_HW_BI_TEST_MODE_FAIL,  // Enter hardwre burn-in test mode fail
  ENTER_SW_TEST_MODE_FAIL,     // Enter software tese mode fail
  
  UNKNOWN_OPERATING_STATUS = 0xFFFF // Use 0xFFFF insted of 0xFF To ensure data type is u16
}Operating_Status;

// Test commands
enum
{
  //WAITING_TO_ENTER_TEST_MODE = 0,
  NO_TEST_CMD = 0,
  ANALOG_VOLTS_INPUT_CALIB_CMD,
  ANALOG_AMPS_INPUT_CALIB_CMD,
  SET_USER_SETTINGS_CRC_CMD,
  SET_DEFAULT_SETTINGS_CRC_CMD,
  SET_APP_VERSION_CMD,
  SET_FLASH_VERSION_CMD,
  SET_DIGITAL_OUTPUT_CMD,
  SET_BURN_IN_CMD,
  SET_INRUSH_RELAY_CMD,
  
  UNKNOWN_TEST_CMD = 0xFFFF  // Use 0xFFFF insted of 0xFF To ensure data type is u16
};

// Test command status
typedef enum
{
  WAITING_TO_ENTER_TEST_MODE = 0,
  WAITING_FOR_TEST_COMMAND,
  TEST_STATUS_OK,
  TEST_BUSY,
  ANALOG_VOLTS_INPUT_CALIB,
  ANALOG_AMPS_INPUT_CALIB,
  SET_USER_SETTINGS_CRC,
  SET_DEFAULT_SETTINGS_CRC,
  SET_APP_VERSION,
  SET_FLASH_VERSION,
  ANALOG_VOLTS_INPUT_CALIB_PASS,
  ANALOG_AMPS_INPUT_CALIB_PASS,
  SET_USER_SETTINGS_CRC_PASS,
  SET_DEFAULT_SETTINGS_CRC_PASS,
  SET_APP_VERSION_PASS,
  SET_FLASH_VERSION_PASS,
  ANALOG_VOLTS_INPUT_CALIB_FAIL,
  ANALOG_AMPS_INPUT_CALIB_FAIL,
  SET_USER_SETTINGS_CRC_FAIL,
  SET_DEFAULT_SETTINGS_CRC_FAIL,
  SET_APP_VERSION_FAIL,
  SET_FLASH_VERSION_FAIL,
  
  UNKNOWN_TEST_STATUS  = 0xFFFF  // Use 0xFFFF insted of 0xFF To ensure data type is u16 
}Test_Command_Status;

//******************* Module Test Control (inside shared memory) ************ 
//////////// Start of "Test Specific" Parameters /////////////////

typedef struct
{ 
  // USER Access
  uint16_t unusedFlags01:1;   
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
} ModuleTestUser_Flags01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_testModuleEnable:1; // Set to "1" to enable test module  
  uint16_t is_burnInEnable:1;     // Set to "1" to enable burn-in mode  
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
} ModuleTestAdmin_Flags01;

typedef struct
{ 
  // USER Access
  uint16_t is_adminMode:1;      // TRUE if drive is in Admin mode
  uint16_t unusedDiscretes02:1; 
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
} ModuleTestUser_Discretes01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_testMode:1;       // TRUE if drive is in test mode
  uint16_t is_burninMode:1;     // TRUE if drive is in burn-in mode   
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
} ModuleTestAdmin_Discretes01;

struct ModuleTest_Data
{
  // USER Access
  ModuleTestUser_Discretes01 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  Operating_Status operatingStatus_u16;
  uint16_t testCommandStatus_u16;
  uint16_t testCommandCondition_u16;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  ModuleTestAdmin_Discretes01 adminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData13;
  uint16_t unusedData14; 
};

struct ModuleTest_Settings
{
  // USER Access
  ModuleTestUser_Flags01 userFlags01;
  uint16_t unusedUserFlags02;
  Operating_Modes setOperatingMode_u16;   // Operating mode set by Modbus or UP
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  uint16_t testCommandData_u16[MAX_TEST_CMD_DATA_LENGTH_IN_WORDS]; // Test command data corresponding to test command or setOperatingMode to be processed;
  uint16_t unusedSettings12;
  uint16_t unusedSettings13;
  uint16_t unusedSettings14;
  
  // ADMIN Access
  ModuleTestAdmin_Flags01 adminFlags01;
  uint16_t unusedAdminFlags02;
  uint16_t testCommand_u16;        // Test command to be processed
  uint16_t testCheckPeriod_u16;    // Poll time for test module
  uint16_t burnInFrequency_u16;    // Burn-in frequency in Hz
  uint16_t burnInCurrent_u16;      // Burn-in current in mA
  uint16_t unusedSettings21;
  uint16_t unusedSettings22;       // Added to prevent padding for flags
};


typedef struct
{
  struct ModuleTest_Settings moduleTest_Settings;
  struct ModuleTest_Data moduleTest_Data;  
}ModuleTest_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct ModuleTest_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"ModuleTest_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//******* end of Module Test Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_TEST_H_ */

