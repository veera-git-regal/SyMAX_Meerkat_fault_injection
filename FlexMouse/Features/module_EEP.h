/**
  ***************************************************************************************************
  * @file    module_EEP.h 
  * @author  Doug Oda
  * @version V1.0
  * @date    11-May-2022
  * @brief   Header of c++ function/s for APP example0 (simple template test APP header)
  * @note    
  ***************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_EPPCMD_H
#define __MODULE_EPPCMD_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#ifdef __cplusplus
extern "C" {
#endif
#define EEPROM_ERASE_KEY 0x03A1
#define EEPROM_KEY (EEPROM_ERASE_KEY ^ DRIVE_FW_VERSION_MINOR) 
  
// EEPROM Module settings
typedef struct
{ 
  // Admin Access
  uint16_t is_ready:1;   
  uint16_t unusedFlags02:1; 
  uint16_t unusedFlags03:1; 
  uint16_t unusedFlags04:1;  
  uint16_t unusedFlags05:1;   
  uint16_t unusedFlags06:1; 
  uint16_t unusedFlags08:1;       
  uint16_t unusedFlags09:1; 
  uint16_t unusedFlags10:1; 
  uint16_t unusedFlags11:1;
  uint16_t unusedFlags12:1; 
  uint16_t unusedFlags13:1; 
  uint16_t unusedFlags14:1; 
  uint16_t unusedFlags15:1;
  uint16_t unusedFlags16:1;
} ModuleEEPROMAdmin_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
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
} ModuleEEPROMUser_Discretes01;

struct ModuleEEPROM_Data
{  
  // USER Access
  ModuleEEPROMUser_Discretes01 userDiscretes01_u16;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01_u16;
  uint16_t RegistersReadCount_u16;  
  uint16_t AddressRead_u16;   
  uint16_t Data[8];
  uint16_t EEPROMEraseKey_u16; 
  uint16_t unusedData21;
  uint16_t unusedData22;
};

struct ModuleEEPROM_Settings
{ 
  // USER Access
  uint16_t unusedUserFlags01_u16;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  uint16_t unusedSettings08;
  uint16_t unusedSettings09;
  uint16_t unusedSettings10;
  uint16_t unusedSettings11;
  uint16_t unusedSettings12;  
  
  // ADMIN Access
  ModuleEEPROMAdmin_Flags01 adminFlags01_u16;
  uint16_t WriteRegisterCount_u16;      
  uint16_t WriteAddress_u16;
  uint16_t WriteData[8];
  uint16_t ReadRegisterCount_u16;   
  uint16_t ReadAddress_u16;   
  uint16_t EEPROMErase_u16;
  uint16_t unusedSettings27;
  uint16_t unusedSettings28;  // Added to prevent padding
};

typedef struct{
 struct ModuleEEPROM_Settings moduleEEPROM_Settings ;
 struct ModuleEEPROM_Data moduleEEPROM_Data;
}ModuleEEPROM_Control; 

/**
********************************************************************************
* @brief   Periodic function used to handle EEPROM read/write
* @details 
* @param   drv_id_u8 The var used by the sceduler to identify this module
*          prev_state_u8 Unused by this module
*          next_state_u8 State to be executed INIT_APP, RUN_APP_WRITE_EEPROM, RUN_APP_READ_EEPROM or KILL_MODULE
*          irq_id_u8 Unused by this module
* @return  Current State INIT_APP, RUN_APP_WRITE_EEPROM, RUN_APP_READ_EEPROM or KILL_MODULE
* @note Unused parameters are maintained because this is called by a pointer whose
*       definition has four uint8_t parameters.
********************************************************************************
*/
uint8_t moduleEEP(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8);



#ifdef __cplusplus
}
#endif

#endif /* __MODULE_EEPCMD_H */

