/**
  ***************************************************************************************************
  * @file    module_dynamic.h 
  * @author  Doug Oda
  * @version V1.0
  * @date    11-May-2022
  * @brief   Header of c++ function/s for APP example0 (simple template test APP header)
  * @note    
  ***************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_DYNAMIC_H
#define __MODULE_DYNAMIC_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"
#include "zz_module_flash.h"

#ifdef __cplusplus
extern "C" {
#endif
  
#define DELAY_BETWEEN_DRIVE_DATA_UPDATES 50
 

//////////// Start of "Drive Data" Parameters /////////////////

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
} DriveDataUser_Flags01;

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
} DriveDataUser_Discretes01;

struct DriveData_Data
{
  // USER Access
  DriveDataUser_Discretes01 userDiscretes01_u16;
  uint16_t mcState_u16;
  uint16_t mcFaults01_u16;
  uint16_t mcFaults02_u16;
  uint16_t mcAppState_u16;
  uint16_t motorActualDirection_u16;
  uint16_t dcBusVoltage_u16;
  int16_t  measuredSpeed_s16;
  int16_t  measuredTorque_s16;
  int16_t  measuredShaftPower_s16;
  int16_t  ipmTemperature_u16;
  int16_t  phaseCurrentIa_s16;
  int16_t  phaseCurrentIb_s16;
  uint16_t unusedData14; // Reserved for PFC temperature   
  uint16_t unusedData15; 
  uint16_t unusedData16;
  uint16_t unusedData17;
  uint16_t unusedData18;
  uint16_t unusedData19;
  uint16_t unusedData20;
  uint16_t unusedData21;
  uint16_t unusedData22;
  uint16_t unusedData23;
  uint16_t unusedData24;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01_u16;
  uint16_t unusedData26;
  uint16_t unusedData27;
  uint16_t unusedData28;
  uint16_t unusedData29;
  uint16_t unusedData30;
  uint16_t unusedData31;
  uint16_t unusedData32; 
};

struct DriveData_Settings 
{
  // USER Access
  DriveDataUser_Flags01 userFlags01_u16;   
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
  uint16_t unusedSettings13;
  uint16_t unusedSettings14;

  // ADMIN Access
  uint16_t unusedAdminFlags01_u16;
  uint16_t commanded_speed_u16;
  uint16_t direction_u16;
  uint16_t commanded_demand_u16;
  uint16_t start_command_u16; 
  uint16_t unusedSettings20; // Added to prevent padding  
};

typedef struct
{
  struct DriveData_Settings driveData_Settings;
  struct DriveData_Data driveData_Data;  
}DriveData_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct DriveData_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"DriveData_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Drive Data" Parameters /////////////////


/**
********************************************************************************
* @brief   Periodic function used to handle real time control via modbus
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
uint8_t module_dynamic(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8);

/**
********************************************************************************
* @brief   Called when new drive dynamic data is received, updates speed direct etc
* @details 
* @param  
* @return   
********************************************************************************
*/  
void UpdateMotorDemand(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_DYNAMIC_H */

