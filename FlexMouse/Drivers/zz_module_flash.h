/**
  ********************************************************************************************************************************
  * @file    zz_module_flash.h 
  * @author  Pamela Lee
  * @brief   Main driver module for flash storgae management.
  * @details This module initializes the flash 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _ZZ_MODULE_FLASH_H_
#define _ZZ_MODULE_FLASH_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"
/* Includes for default settings -----------------------------------------------------------------------------------------------*/
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "mc_stm_types.h"
#include "mc_type.h"
#include "hall_speed_pos_fdbk.h"
#include "regal_mc_lib.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Base address of the Flash sectors */
/* Base address of the Flash pages */
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) // Base address of Page 0, 2 Kbytes Protected area//
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) // Base address of Page 1, 2 Kbytes Protected area//
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) // Base address of Page 2, 2 Kbytes Interrupt Vector Wrappers and Start of code space//
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) // Base address of Page 3, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) // Base address of Page 4, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) // Base address of Page 5, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) // Base address of Page 6, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) // Base address of Page 7, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) // Base address of Page 8, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) // Base address of Page 9, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) // Base address of Page 10, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) // Base address of Page 11, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) // Base address of Page 12, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) // Base address of Page 13, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) // Base address of Page 14, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) // Base address of Page 15, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000) // Base address of Page 16, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08008800) // Base address of Page 17, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08009000) // Base address of Page 18, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08009800) // Base address of Page 19, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x0800A000) // Base address of Page 20, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x0800A800) // Base address of Page 21, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x0800B000) // Base address of Page 22, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x0800B800) // Base address of Page 23, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x0800C000) // Base address of Page 24, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x0800C800) // Base address of Page 25, 2 Kbytes code space//
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x0800D000) // Base address of Page 26, 2 Kbytes Flash Settings/Shared ROM main//
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x0800D800) // Base address of Page 27, 2 Kbytes Flash Settings/Shared ROM copy//
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x0800E000) // Base address of Page 28, 2 Kbytes Safety core//
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x0800E800) // Base address of Page 29, 2 Kbytes Safety core//
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x0800F000) // Base address of Page 30, 2 Kbytes Safety core//
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x0800F800) // Base address of Page 31, 2 Kbytes Safety core//
  
#define MAX_NUMBER_OF_FLASH_PAGES 31
  
//#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_28 //ADDR_FLASH_PAGE_30   // Start @ of user Flash area //
//#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_29 //ADDR_FLASH_PAGE_31   // End @ of user Flash area //
#define NumOfPage 1                                  // page size of setting // 

#define FIRMWARE_CRC_ADDR  ((uint32_t)0x0800CFFC)        // END of ADDR_FLASH_PAGE_25
#define FLASH_START_ADDR             ADDR_FLASH_PAGE_2   // Start of flash address
#define FLASH_DEFAULT_SETTINGS_ADDR  ADDR_FLASH_PAGE_26  // Page where all default settings for modules are stored
#define FLASH_SETTINGS_START_ADDR    ADDR_FLASH_PAGE_27  // Page where all module settings are stored

#define FLASH_SAFETYCORE_1      ADDR_FLASH_PAGE_28
#define FLASH_SAFETYCORE_2      ADDR_FLASH_PAGE_29
#define FLASH_SAFETYCORE_3      ADDR_FLASH_PAGE_30
#define FLASH_SAFETYCORE_4      ADDR_FLASH_PAGE_31
#define SAFETY_FIRMWARE_REV_ADDR FLASH_SAFETYCORE_1
#define SAFETY_FIRMWARE_CRC_ADDR  ((uint32_t)FLASH_SAFETYCORE_4 + PAGE_SIZE - 4) 
  
#define FLASH_BOOTLOADER_1      ADDR_FLASH_PAGE_0
#define FLASH_BOOTLOADER_2      ADDR_FLASH_PAGE_1
  
#define NUMBER_OF_FLASH_PAGES 1  //NumOfPage 1           // page size of setting // 
#define PAGE_SIZE 2048                                   // Bytes per page
  
#define MAX_MODULES_IN_FLASH 32 // # modules that can be shored in FLASH
#define MAX_MODULE_STRUCTURE_SIZE_BYTES 60 // Max size per module in FLASH

// Settings page consists of User Settings, version, CRC followed by
// Safety core Settings, Version and CRC
// This is same for default page
  
#define FLASH_SETTINGS_VERSION      (0x01000200)  // 0x00VVXXYY = VV.XX.YY version. Only 0-9 numbers allower per nibble
#define FLASH_SETTINGS_VERSION_ADDR ( (FLASH_SETTINGS_START_ADDR + (MAX_MODULES_IN_FLASH * MAX_MODULE_STRUCTURE_SIZE_BYTES)) )
#define FLASH_SETTINGS_CRC_ADDR     ( (FLASH_SETTINGS_START_ADDR + (MAX_MODULES_IN_FLASH * MAX_MODULE_STRUCTURE_SIZE_BYTES) + 4) )

#define FLASH_SETTINGS_VERSION_ADDR_OFFSET ((MAX_MODULES_IN_FLASH * MAX_MODULE_STRUCTURE_SIZE_BYTES))
#define FLASH_SETTINGS_CRC_ADDR_OFFSET     ((MAX_MODULES_IN_FLASH * MAX_MODULE_STRUCTURE_SIZE_BYTES) + 4)

#define FLASH_SAFETY_SETTINGS_VERSION (0x01000200)
#define FLASH_VERSION_SIZE 4
#define FLASH_CRC_SIZE 4
#define FLASH_SAFETY_SETTINGS_START_ADDRESS (FLASH_SETTINGS_CRC_ADDR + FLASH_CRC_SIZE)
#define FLASH_SAFETY_SETTINGS_VERSION_ADDR ((FLASH_SETTINGS_START_ADDR + PAGE_SIZE) - 8)
#define FLASH_SAFETY_SETTINGS_CRC_ADDR ((FLASH_SETTINGS_START_ADDR + PAGE_SIZE) - FLASH_VERSION_SIZE)
#define FLASH_SAFETY_SETTINGS_SIZE (uint16_t)(FLASH_SAFETY_SETTINGS_VERSION_ADDR - FLASH_SAFETY_SETTINGS_START_ADDRESS + 8)

#define FLASH_SAFETY_SETTINGS_START_ADDR_OFFSET (FLASH_SETTINGS_CRC_ADDR_OFFSET + FLASH_CRC_SIZE)
#define FLASH_SAFETY_SETTINGS_VERSION_ADDR_OFFSET (uint16_t)(FLASH_SAFETY_SETTINGS_START_ADDR_OFFSET + FLASH_SAFETY_SETTINGS_SIZE)
#define FLASH_SAFETY_SETTINGS_CRC_ADDR_OFFSET (uint16_t)(FLASH_SAFETY_SETTINGS_VERSION_ADDR_OFFSET + FLASH_VERSION_SIZE)
   
  
#ifndef FLASH_SETTINGS_VERSION
#error MOTOR FLASH_SETTINGS_VERSION not defined
#endif
  
#ifndef FLASH_SAFETY_SETTINGS_VERSION
#error MOTOR FLASH_SAFETY_SETTINGS_VERSION not defined
#endif
  
#if (PAGE_SIZE - FLASH_SAFETY_SETTINGS_START_ADDR_OFFSET) < 90
#error Flash size for Meerkat does not meet MIN Meerkat settings size
#endif

#define FLASH_COPY_BYTE_SIZE 8 // Number of bytes copied at a time by HAL
  
// Limited by min bytes in structures (FLAGS 2byte, Settings 2 bytes min) 
// Limited by min bytes (8 bytes) HAL can write
#if FLASH_COPY_BYTE_SIZE < 8
#define MIN_MODULE_SIZE 8 
#else
  #define MIN_MODULE_SIZE FLASH_COPY_BYTE_SIZE 
#endif

#if (MAX_MODULE_STRUCTURE_SIZE_BYTES * MAX_MODULES_IN_FLASH) > (PAGE_SIZE - 8) 
#error "DRIVE FLASH SETTING SIZE > (PAGE_SIZE - 8) BYTES" // Flash settings should be blow 2040 bytes// Page size is 2048
#endif
  
#define FLASH_BUFFER_SIZE 10 // Size of buffer used to copy data from RAM to flash for each module

#if MAX_MODULE_STRUCTURE_SIZE_BYTES > (FLASH_BUFFER_SIZE * 8) // flash_buffer can only hold (10 * 8) = 80 bytes
#error "DRIVE flash_buffer SIZE CAN ONLY HOLD (FLASH_BUFFER_SIZE * 8) BYTES"
#endif

  
enum
{
  MOTOR_ID_SETTINGS = MIN_MODULE_ID, 
  //DRIVE_MODBUS,
  STARTUP_SETTINGS,
  MOTOR_SETTINGS,
  TUNING_SETTINGS01,
  TUNING_SETTINGS02,
  LIMITS_SETTINGS01,
  LIMITS_SETTINGS02,
  PROTECTION_SETTINGS01,
  PROTECTION_SETTINGS02,
  BRAKING_SETTINGS,
  OTF_SETTINGS,
  //WINDMILL_SETTINGS,  
  HW_SPECIFIC_SETTINGS,
  APP_SPECIFIC_SETTINGS,
  HARMONICS_COMPENSATION_1_SETTINGS,
  HARMONICS_COMPENSATION_2_SETTINGS,
  HARMONICS_COMPENSATION_3_SETTINGS,
  //DRIVE_FLASH,
  //MODULE_EEPROM,
  //TEST_SETTINGS,
  // UNUSED22,
  // UNUSED23,
  ST_MC_SETTINGS01,
  ST_MC_SETTINGS02,
  MAX_SETTINGS_INDEX, // Always the last Index
};

#define NON_PROCESS_TABLE_STRUCTURES_COUNT MAX_SETTINGS_INDEX
#define PROCESS_TABLE_STRUCT_IN_FLASH 3

#if (NON_PROCESS_TABLE_STRUCTURES_COUNT + PROCESS_TABLE_STRUCT_IN_FLASH) > (MAX_MODULES_IN_FLASH) 
#error "DRIVE FLASH MODULES THAT NEED TO BE STORED IN FLASH ARE GREATER THAN MAX_MODULES_IN_FLASH" // # modules that can be shored in FLASH are MAX_MODULES_IN_FLASH
#endif



typedef enum
{
  // USER Commands
  FLASH_WAITING_FOR_CMD = 0,                  // Waiting for flash commands
  FLASH_WRITE_RAM2FLASH_USER_SETTINGS_CMD,    // Write settings from module structures to user settings FLASH page
  FLASH_READ_FLASH2RAM_USER_SETTINGS_CMD,     // Read settings from user settings FLASH page to module structures
  FLASH_READ_FLASH2RAM_DEFAULT_SETTINGS_CMD,  // Read settings from default settings FLASH page to module structures
  
  // ADMIN Commands
  FLASH_WRITE_RAM2FLASH_DEFAULT_SETTINGS_CMD, // Write settings from module structures to default settings FLASH page
  FLASH_COPY_USER_SETTINGS_TO_DEFAULTS_CMD,   // Copy user settings into default settings (replace factor settings)
  FLASH_CRC_UPDATE,                           // Update CRC in the flash 
  FLASH_ERASE_CMD,                            // Erase flash page
  FLASH_WRITE_CMD, // Dont implement this. Only save data from RAM to FLASH. No direct access to flash
  
  FLASH_WRITE_RAM2FLASH_CMD,                  // UNUSED
  FLASH_READ_FALSH2RAM_CMD,                   // UNUSED  
  FLASH_SETTINGS_INIT,                        // UNUSED // Init RAM with user settings from flash 
  FLASH_DEFAULT_SETTINGS_INIT,                // UNUSED // Init RAM with default settings from flash 
  FLASH_SETTINGS_UPDATE,                      // UNUSED
  
  FLASH_UNKNOWN_CMD = 0x00FF  
}Flash_Commands;

typedef enum
{
  FLASH_OK = 0,
  FLASH_ERROR,
  FLASH_BUSY,
  FLASH_TIMEOUT,
  FLASH_EMPTY,
  FLASH_NOT_EMPTY,
  FLASH_SETTINGS_INIT_COMPLETE,
  FLASH_SETTINGS_WRITE_COMPLETE,
  FLASH_DEFAULT_SETTINGS_INIT_COMPLETE,
  FLASH_READ_IN_PROCESS,
  FLASH_COPY_IN_PROCESS,
  FLASH_ERASE_IN_PROCESS,
  FLASH_ERASE_ADD_ERROR,
  FLASH_WRITE_IN_PROCESS,
  FLASH_READ_COMPLETE,
  FLASH_ERASE_COMPLETE,
  FLASH_WRITE_COMPLETE, 
  FLASH_WRITE_CRC_COMPLETE,
  FLASH_CRC_VALID,
  FLASH_CRC_EMPTY,
  FLASH_CRC_ERROR,
  FLASH_READ_ERROR,
  FLASH_OUT_OF_RANGE_ERROR,
  FLASH_VERSION_ERROR,
  FLASH_WRITE_DATA_LENGTH_ERROR,
  FLASH_BUF_FULL,
  FLASH_COPY_TO_BUF_COMPLETE,
  FLASH_DATA_MISMATCH,
  COMPARING_RAM_TO_FLASH,      // Compare RAM settings to Flash setting
  COMPARE_RAM_TO_FLASH_PASS,
  FLASH_UNKNOWN_STATE = 0x00FF
}Flash_Status;

//******************* Flash Module Control (inside shared memory) ************  
// Flash Module settings
typedef struct
{  
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
} ModuleFlashUser_Flags01;
typedef struct
{  
  uint16_t is_driveFlashEnable:1; 
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
} ModuleFlashAdmin_Flags01;

typedef struct
{  
  // USER Access
  uint16_t is_copyUserFlash2RamSuccess:1; 
  uint16_t is_copyDefaultFlash2RamSuccess:1; 
  uint16_t is_copyRam2UserFlashSuccess:1;   
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
} ModuleFlashUser_Discretes01;

typedef struct
{  
  // ADMIN Access
  uint16_t is_copyRam2DefaultFlashSuccess:1;
  uint16_t is_userFlashEmpty:1;   
  uint16_t is_defaultFlashEmpty:1; 
  uint16_t is_copyFlash2UserDeaultFlashSuccess:1; 
  uint16_t is_userFlashCrcValid:1; 
  uint16_t is_defaultFlashCrcValid:1;
  uint16_t is_userFlashRecoveredFromDefaults:1;
  uint16_t is_defaultFlashRecoveredFromUser:1; 
  uint16_t is_flashPageEraseError:1;   
  uint16_t unusedDiscretes10:1; 
  uint16_t unusedDiscretes11:1;
  uint16_t unusedDiscretes12:1; 
  uint16_t unusedDiscretes13:1;
  uint16_t unusedDiscretes14:1;
  uint16_t unusedDiscretes15:1; 
  uint16_t unusedDiscretes16:1;  
} ModuleFlashAdmin_Discretes01;

// Live Flash Module Data
struct ModuleFlash_Data
{ 
  // USER Access
  ModuleFlashUser_Discretes01 userDiscretes01_u16;
  Flash_Status moduleFlashStatus_u16;     // Current status of flash  
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // Admin Access
  ModuleFlashAdmin_Discretes01 adminDiscretes01_u16;
  uint16_t unusedAdminDiscretes01_u16; 
  uint16_t unusedData11;
  uint16_t unusedData12;
};

struct ModuleFlash_Settings
{ 
  // USER Access
  ModuleFlashUser_Flags01 userFlags01_u16;
  Flash_Commands flashCommands_u16;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  
  // ADMIN Access
  ModuleFlashAdmin_Flags01 adminFlags01_u16;
  uint16_t unusedAdminFlags02_u16;
  uint32_t pageAddress_u32;
  uint16_t unusedSettings11;
  uint16_t unusedSettings12;       // Added to prevent padding
};

typedef struct{
 struct ModuleFlash_Settings moduleFlash_Settings ;
 struct ModuleFlash_Data moduleFlash_Data;
}ModuleFlash_Control;  

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct ModuleFlash_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"ModuleFlash_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

  
uint16_t FlashRead16Bits(unsigned char* aDataBuf, uint16_t offsetByte);
uint64_t FlashRead64Bits(unsigned char* aDataBuf, uint16_t offsetByte);
//uint8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress); // Not used
uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress);  // Not used
 

Flash_Status flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress);
Flash_Status flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length);
Flash_Status flashBlockCopy(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length);
Flash_Status flashBlockProgram2Bytes(uint8_t module_id_u8, uint32_t _TopageAddress, uint32_t _length);
//Flash_Status isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t _NumOfPage);
Flash_Status isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t size_u16);
//uint16_t FlashBufDeRegistered(uint16_t _offset, uint64_t* _returnBuf);

uint32_t flashGetSettingAddress(uint8_t drv_id_u8, uint8_t *module_address_ptr, uint8_t *setting_address_ptr);

Flash_Status copy_Setting_Flash_To_RAM(uint8_t module_id_u8, uint8_t isCopyDefaultSettings);
//Flash_Status flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress);
//Flash_Status flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress, uint16_t size_u16);
Flash_Status flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress, uint16_t size_u16);
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
/** @Caution :All the settings offset/register number will be assigned in serquential by this enum 
    Please make sure MODBUS Universal-protocol and this flash assignment are lineup together **/
//typedef enum                                     
//{ 
//  //ST_motor libraries flash parameters in flash table
//  Index_A_POLE_PAIR_NUM                ,     // reg0  _1
//  Index_A_RS                           ,     // reg1   _ 
//  Index_A_RS_FACTOR                    ,     // reg2
//  Index_A_LS                           ,     // reg3   _ 
//  Index_A_LS_FACTOR                    ,     // reg4 
//  Index_A_NOMINAL_CURRENT              ,     // reg5   _1
//  Index_A_MOTOR_VOLTAGE_CONSTANT       ,     // reg6 
//  Index_A_BEMF_CONSTANT_FACTOR         ,     // reg7   
//  Index_A_MAX_APPLICATION_SPEED_RPM    ,     // reg8   _ 
//  Index_A_MIN_APPLICATION_SPEED_RPM    ,     // reg9   _ 
//  Index_A_PLL_KP_GAIN                  ,     // reg10   _1	MC_PROTOCOL_REG_PLL_KP   
//  Index_A_PLL_KI_GAIN                  ,     // reg11   _1	MC_PROTOCOL_REG_PLL_KI
//  Index_A_PWM_FREQUENCY                ,     // reg12   _ 
//  Index_A_PID_TORQUE_KP_DEFAULT        ,     // reg13  _1	MC_PROTOCOL_REG_TORQUE_KP
//  Index_A_PID_TORQUE_KI_DEFAULT        ,     // reg14  _1	MC_PROTOCOL_REG_TORQUE_KI
//  Index_A_PID_FLUX_KP_DEFAULT          ,     // reg15  _1	MC_PROTOCOL_REG_FLUX_KP
//  Index_A_PID_FLUX_KI_DEFAULT          ,     // reg16  _1	MC_PROTOCOL_REG_FLUX_KI
//  Index_A_PID_SPEED_KP_DEFAULT         ,     // reg17  _1	MC_PROTOCOL_REG_SPEED_KP
//  Index_A_PID_SPEED_KI_DEFAULT         ,     // reg18  _1	MC_PROTOCOL_REG_SPEED_KI
//  Index_A_IQMAX                        ,     // reg19  _1
//  Index_A_DEFAULT_CONTROL_MODE         ,     // reg20  _1       MC_PROTOCOL_REG_CONTROL_MODE
//  Index_A_OV_VOLTAGE_THRESHOLD_V       ,     // reg21  _ 
//  Index_A_UD_VOLTAGE_THRESHOLD_V       ,     // reg22  _ 
//  Index_A_OV_TEMPERATURE_THRESHOLD_C   ,     // reg23  _ 
//  Index_A_OV_TEMPERATURE_HYSTERESIS_C  ,     // reg24  _ 
//  Index_A_PHASE1_DURATION              ,     // reg25  _1       MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE1_FINAL_SPEED_UNIT      ,     // reg26  _1       MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE1_FINAL_CURRENT         ,     // reg27  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE2_DURATION              ,     // reg28  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE2_FINAL_SPEED_UNIT      ,     // reg29  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE2_FINAL_CURRENT         ,     // reg30  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE3_DURATION              ,     // reg31  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE3_FINAL_SPEED_UNIT      ,     // reg32  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE3_FINAL_CURRENT         ,     // reg33  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE4_DURATION              ,     // reg34  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE4_FINAL_SPEED_UNIT      ,     // reg35  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE4_FINAL_CURRENT         ,     // reg36  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE5_DURATION              ,     // reg37  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE5_FINAL_SPEED_UNIT      ,     // reg38  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_PHASE5_FINAL_CURRENT         ,     // reg39  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  Index_A_OBS_MINIMUM_SPEED_RPM        ,     // reg40
//  Index_A_TRANSITION_DURATION          ,     // reg41  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
//  //Braking and On-the-fly definitions          
//  Index_FAULT_AUTOSTART                ,     // reg42 _1    1u /* enable/disable auto-start when fault occurs */
//  Index_CONTROLLED_BRAKING             ,     // reg43 _     1u
//  Index_BK_VBUS_CLIP                    ,     // reg44 _     20 
//  Index_BK_RAMP_a                      ,     // reg45 _     (int32_t) 13
//  Index_BK_RAMP_b                      ,     // reg46 _     (int32_t) -6
//  Index_BK_RAMP_c                      ,     // reg47 _     (int32_t) 1220
//  
//  Index_REGAL_OTF                      ,     // reg48  _     1u
//  Index_OTF_DBEMFG                     ,     // reg49  _             256
//  Index_OTF_MAX_BEMFG                  ,     // reg50  _     320
//  Index_OTF_MIN_BEMFG                  ,     // reg51  _     250
//  Index_OTF_MAX_SYNC_SPEED             ,     // reg52  _     120
//  Index_OTF_MIN_SYNC_SPEED             ,     // reg53  _     30
//  
//  Index_DECEL_CONTROL                  ,     // reg54  _        1u
//  Index_BK_LOWSIDE_DURATION            ,     // reg55  _        5000
//  Index_BK_ENDSPEED                    ,     // reg56  _        10
//  Index_BK_CURRENTSEEDING              ,
//  
//  //parameters necessary for auto-tune
//  Index_MOTOR_INERTIA                  ,     // reg57 _      1u
//  Index_FAN_INERTIA                    ,     // reg58 _      1u  
//  Index_MOTOR_VISCOUS_DAMPING          ,     // reg59 _      1u  
//  Index_RAMPEND_CURRENT                ,     // reg60 _      10  
//  Index_RAMP_STEP                      ,     // reg61 _      2  
//  Index_SPEED_TRANSITION               ,     // reg62 _      66  
//  Index_LOWERIQLIMIT                   ,     // reg63 _      300  
//  
//  //parameters necessary for windmilling
//  Index_WM_MAX_REVERSE_SPEED              ,     // reg64 _      105  
//  Index_WM_CLAMP_DURATION                 ,     // reg65 _      18000 
//  Index_WM_CLAMP_CURRENT                  ,     // reg66 _      14000  
//  Index_WM_REVERSE_ENDSPEED               ,     // reg67 _      5 
//  Index_WM_CLAMP_RAMP                     ,     // reg68 _      100  
//  Index_WM_SHORTCOIL_DURATION             ,     // reg69 _      5000 
//  
////////////////////////////////////////////////////////////
//  // below are future configurable data      
//  Index_D_HALL_SENSORS_PLACEMENT       ,     // reg70  _1
//  Index_D_HALL_PHASE_SHIFT             ,     // reg71  _1
//  Index_D_M1_ENCODER_PPR               ,     // reg72  _ 
//
///////////////////////////////////////////////////////////
//  // additional drive parameter configurable data
//  Index_TF_KPDIV                        ,       // reg73
//  Index_TF_KIDIV                        ,       // reg74
//  Index_TF_KDDIV                        ,       // reg75
//  Index_SP_KPDIV                        ,       // reg76
//  Index_SP_KIDIV                        ,       // reg77
//  Index_SP_KDDIV                        ,       // reg78
//  Index_F1                              ,       // reg79
//  Index_F2                              ,       // reg80
//  Index_PLL_KPDIV                       ,       // reg81
//  Index_PLL_KIDIV                       ,       // reg82
//  
//#if GAIN1 != 0   //pll or cord
//  Index_A_GAIN1                        ,     // reg83  _ 
//  Index_A_GAIN2                        ,     // reg84  _ 
//#else
//  Index_D_CORD_GAIN1                   ,     // reg85 _ 
//  Index_D_CORD_GAIN2                   ,     // reg86  _ 
//  Index_D_CORD_MAX_ACCEL_DPPP          ,     // reg87  _1
//#endif //GAIN1   //pll or cord
//  Index_DRIVE_FW_VERSION,
//  Index_ST_MOT_LIB_PARAMETERS_END      ,      
//  //Module setting data should append below here and group as each module
//
//#ifdef _AB_MODULE_MC_STATEMACHINE_H_
//  Index_MIN_COMMANDABLE_SPEED	  =   Index_ST_MOT_LIB_PARAMETERS_END,                  //<---- please use the index above this line for the start of index for this module
//  Index_MAX_COMMANDABLE_SPEED           ,                                               /** @caution1: if you change the sequence of module/s please make sure you follow the same logic)**/
//  Index_SPEED_UP_RAMP_RATE              ,                                               /** @caution2: All parameter directly write into flash and active instantly !!!!! **/
//  Index_SPEED_DOWN_RAMP_RATE            ,
//  Index_SPEED_CONSIDERED_STOPPED        , 
//  Index_MotSpinTimeOut                  , 
//  Index_SpinPollPeriod                  , 
//  Index_numOfStartRetry                 , 
//  Index_StartRetryPeriod                , 
//  Index_StartPeriodInc                  , 
//  Index_over_current_threshold          , 
//  Index_over_current_lower_threshold    , 
//  Index_over_current_rpm_Reduce         , 
//  Index_OvCurrent_derate_period         , 
//  Index_over_power_threshold            , 
//  Index_over_power_lower_threshold      , 
//  Index_over_power_rpm_Reduce           , 
//  Index_OvPower_derate_period           , 
//  Index_over_temperature_threshold      , 
//  Index_over_temperature_lower_threshold , 
//  Index_over_temperature_rpm_Reduce     , 
//  Index_OvTemp_derate_period            , 
//  Index_MODULE_MC_STATEMACHINE_END      ,
//#else
//  Index_MODULE_MC_STATEMACHINE_END =  Index_ST_MOT_LIB_PARAMETERS_END,                   //<----- If this module is not installed in this system, Flash index system will resume the last index of former module
//#endif //_MODULE_MC_STATEMACHINE_H_
//
//  index_Blk_Flash_CRC   = (FLASH_PAGE_SIZE/2)                                           //always reserve the last word for CRC         
//      
//}FlashOffsetIndex; 

//// These defines has to match up with App-Side for correct reception
//#define LS_FACTOR       100000u
//#define RS_FACTOR      1000u
//#define BEMF_FACTOR     10u
//  
//__weak const uint16_t A_POLE_PAIR_NUM               @(FLASH_USER_START_ADDR + (2 * Index_A_POLE_PAIR_NUM               )  ) = POLE_PAIR_NUM            		; 
//__weak const uint16_t A_RS                          @(FLASH_USER_START_ADDR + (2 * Index_A_RS                          )  ) = (uint16_t)(RS * RS_FACTOR)              		; //in mOhm
//__weak const uint16_t A_RS_FACTOR                   @(FLASH_USER_START_ADDR + (2 * Index_A_RS_FACTOR                   )  ) =  (uint16_t) RS_FACTOR              		;
//__weak const uint16_t A_LS                          @(FLASH_USER_START_ADDR + (2 * Index_A_LS                          )  ) = (uint16_t)(LS * LS_FACTOR)           ; //in mH
//__weak const uint16_t A_LS_FACTOR                   @(FLASH_USER_START_ADDR + (2 * Index_A_LS_FACTOR                   )  ) =  (uint16_t) LS_FACTOR              		;
//__weak const uint16_t A_NOMINAL_CURRENT             @(FLASH_USER_START_ADDR + (2 * Index_A_NOMINAL_CURRENT             )  ) = NOMINAL_CURRENT          		; 
//__weak const int16_t A_MOTOR_VOLTAGE_CONSTANT       @(FLASH_USER_START_ADDR + (2 * Index_A_MOTOR_VOLTAGE_CONSTANT      )  ) = (MOTOR_VOLTAGE_CONSTANT * BEMF_FACTOR)     ;
//__weak const uint16_t A_BEMF_CONSTANT_FACTOR        @(FLASH_USER_START_ADDR + (2 * Index_A_BEMF_CONSTANT_FACTOR        )  ) =  (uint16_t) BEMF_FACTOR              		;
//__weak const uint16_t A_MAX_APPLICATION_SPEED_RPM   @(FLASH_USER_START_ADDR + (2 * Index_A_MAX_APPLICATION_SPEED_RPM   )  ) = MAX_APPLICATION_SPEED_RPM         ; //this parameter have to takecare of its dependance //MAX_BEMF_VOLTAGE parameter dependance => C3
//                                                                                                                                                                                                                      //MAX_APPLICATION_SPEED_RPM parameter dependance => MAX_APPLICATION_SPEED_UNIT
//__weak const uint16_t A_MIN_APPLICATION_SPEED_RPM   @(FLASH_USER_START_ADDR + (2 * Index_A_MIN_APPLICATION_SPEED_RPM   )  ) = MIN_APPLICATION_SPEED_RPM         ; //this parameter have to takecare of its dependance //MIN_APPLICATION_SPEED_RPM parameter dependance => MIN_APPLICATION_SPEED_UNIT
//__weak const uint16_t A_PLL_KP_GAIN                 @(FLASH_USER_START_ADDR + (2 * Index_A_PLL_KP_GAIN                 )  ) = PLL_KP_GAIN                	; 
//__weak const uint16_t A_PLL_KI_GAIN                 @(FLASH_USER_START_ADDR + (2 * Index_A_PLL_KI_GAIN                 )  ) = PLL_KI_GAIN                	; 
//__weak const uint16_t A_PWM_FREQUENCY               @(FLASH_USER_START_ADDR + (2 * Index_A_PWM_FREQUENCY               )  ) = PWM_FREQUENCY              	; //this parameter have to takecare of its dependance //PWM_FREQUENCY parameter dependance => TF_REGULATION_RATE 
//__weak const uint16_t A_PID_TORQUE_KP_DEFAULT       @(FLASH_USER_START_ADDR + (2 * Index_A_PID_TORQUE_KP_DEFAULT       )  ) = PID_TORQUE_KP_DEFAULT      	; 
//__weak const uint16_t A_PID_TORQUE_KI_DEFAULT       @(FLASH_USER_START_ADDR + (2 * Index_A_PID_TORQUE_KI_DEFAULT       )  ) = PID_TORQUE_KI_DEFAULT      	; 
//__weak const uint16_t A_PID_FLUX_KP_DEFAULT         @(FLASH_USER_START_ADDR + (2 * Index_A_PID_FLUX_KP_DEFAULT         )  ) = PID_FLUX_KP_DEFAULT        	; 
//__weak const uint16_t A_PID_FLUX_KI_DEFAULT         @(FLASH_USER_START_ADDR + (2 * Index_A_PID_FLUX_KI_DEFAULT         )  ) = PID_FLUX_KI_DEFAULT        	; 
//__weak const uint16_t A_PID_SPEED_KP_DEFAULT        @(FLASH_USER_START_ADDR + (2 * Index_A_PID_SPEED_KP_DEFAULT        )  ) = PID_SPEED_KP_DEFAULT       	; 
//__weak const uint16_t A_PID_SPEED_KI_DEFAULT        @(FLASH_USER_START_ADDR + (2 * Index_A_PID_SPEED_KI_DEFAULT        )  ) = PID_SPEED_KI_DEFAULT       	; 
//__weak const uint16_t A_IQMAX                       @(FLASH_USER_START_ADDR + (2 * Index_A_IQMAX                       )  ) = IQMAX                      	; 
//__weak const uint16_t A_DEFAULT_CONTROL_MODE        @(FLASH_USER_START_ADDR + (2 * Index_A_DEFAULT_CONTROL_MODE        )  ) = DEFAULT_CONTROL_MODE       	;
//__weak const uint16_t A_OV_VOLTAGE_THRESHOLD_V      @(FLASH_USER_START_ADDR + (2 * Index_A_OV_VOLTAGE_THRESHOLD_V      )  ) = OV_VOLTAGE_THRESHOLD_V     	;
//__weak const uint16_t A_UD_VOLTAGE_THRESHOLD_V      @(FLASH_USER_START_ADDR + (2 * Index_A_UD_VOLTAGE_THRESHOLD_V      )  ) = UD_VOLTAGE_THRESHOLD_V     	; 
//__weak const uint16_t A_OV_TEMPERATURE_THRESHOLD_C  @(FLASH_USER_START_ADDR + (2 * Index_A_OV_TEMPERATURE_THRESHOLD_C  )  ) = OV_TEMPERATURE_THRESHOLD_C 	; 
//__weak const uint16_t A_OV_TEMPERATURE_HYSTERESIS_C @(FLASH_USER_START_ADDR + (2 * Index_A_OV_TEMPERATURE_HYSTERESIS_C )  ) = OV_TEMPERATURE_HYSTERESIS_C	; 
//__weak const uint16_t A_PHASE1_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_DURATION             )  ) = PHASE1_DURATION            	; 
//__weak const uint16_t A_PHASE1_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_FINAL_SPEED_UNIT     )  ) = PHASE1_FINAL_SPEED_UNIT    	; 
//__weak const uint16_t A_PHASE1_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_FINAL_CURRENT        )  ) = PHASE1_FINAL_CURRENT       	; 
//__weak const uint16_t A_PHASE2_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_DURATION             )  ) = PHASE2_DURATION            	; 
//__weak const uint16_t A_PHASE2_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_FINAL_SPEED_UNIT     )  ) = PHASE2_FINAL_SPEED_UNIT    	; 
//__weak const uint16_t A_PHASE2_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_FINAL_CURRENT        )  ) = PHASE2_FINAL_CURRENT       	; 
//__weak const uint16_t A_PHASE3_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_DURATION             )  ) = PHASE3_DURATION            	; 
//__weak const uint16_t A_PHASE3_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_FINAL_SPEED_UNIT     )  ) = PHASE3_FINAL_SPEED_UNIT    	; 
//__weak const uint16_t A_PHASE3_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_FINAL_CURRENT        )  ) = PHASE3_FINAL_CURRENT       	; 
//__weak const uint16_t A_PHASE4_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_DURATION             )  ) = PHASE4_DURATION            	; 
//__weak const uint16_t A_PHASE4_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_FINAL_SPEED_UNIT     )  ) = PHASE4_FINAL_SPEED_UNIT    	; 
//__weak const uint16_t A_PHASE4_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_FINAL_CURRENT        )  ) = PHASE4_FINAL_CURRENT       	; 
//__weak const uint16_t A_PHASE5_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_DURATION             )  ) = PHASE5_DURATION            	; 
//__weak const uint16_t A_PHASE5_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_FINAL_SPEED_UNIT     )  ) = PHASE5_FINAL_SPEED_UNIT    	; 
//__weak const uint16_t A_PHASE5_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_FINAL_CURRENT        )  ) = PHASE5_FINAL_CURRENT       	; 
//__weak const uint16_t A_OBS_MINIMUM_SPEED_RPM       @(FLASH_USER_START_ADDR + (2 * Index_A_OBS_MINIMUM_SPEED_RPM       )  ) = OBS_MINIMUM_SPEED_RPM             ;
//
//
//
//__weak const uint16_t A_TRANSITION_DURATION         @(FLASH_USER_START_ADDR + (2 * Index_A_TRANSITION_DURATION         )  ) = TRANSITION_DURATION        	; 
//
//__weak const uint16_t A_FAULT_AUTOSTART             @(FLASH_USER_START_ADDR + (2 *   Index_FAULT_AUTOSTART             )  ) = default_FAULT_AUTOSTART   	; 
//__weak const uint16_t A_CONTROLLED_BRAKING          @(FLASH_USER_START_ADDR + (2 *   Index_CONTROLLED_BRAKING          )  ) = default_CONTROLLED_BRAKING   	; 
//__weak const uint16_t A_BK_VBUS_CLIP                @(FLASH_USER_START_ADDR + (2 *   Index_BK_VBUS_CLIP                )  ) = default_BK_VBUS_CLIP          	; 
//__weak const int16_t A_BK_RAMP_a                    @(FLASH_USER_START_ADDR + (2 *   Index_BK_RAMP_a                   )  ) = default_BK_RAMP_a            	; 
//__weak const int16_t A_BK_RAMP_b                    @(FLASH_USER_START_ADDR + (2 *   Index_BK_RAMP_b                   )  ) = default_BK_RAMP_b            	; 
//__weak const int16_t A_BK_RAMP_c                    @(FLASH_USER_START_ADDR + (2 *   Index_BK_RAMP_c                   )  ) = default_BK_RAMP_c            	; 
//__weak const int16_t A_REGAL_OTF                    @(FLASH_USER_START_ADDR + (2 *   Index_REGAL_OTF                   )  ) = default_REGAL_OTF          	; 
//__weak const uint16_t A_OTF_DBEMFG                  @(FLASH_USER_START_ADDR + (2 *   Index_OTF_DBEMFG                  )  ) = default_OTF_DBEMFG           	; 
//__weak const uint16_t A_OTF_MAX_BEMFG               @(FLASH_USER_START_ADDR + (2 *   Index_OTF_MAX_BEMFG               )  ) = default_OTF_MAX_BEMFG        	; 
//__weak const uint16_t A_OTF_MIN_BEMFG               @(FLASH_USER_START_ADDR + (2 *   Index_OTF_MIN_BEMFG               )  ) = default_OTF_MIN_BEMFG        	; 
//__weak const uint16_t A_OTF_MAX_SYNC_SPEED          @(FLASH_USER_START_ADDR + (2 *   Index_OTF_MAX_SYNC_SPEED          )  ) = default_OTF_MAX_SYNC_SPEED   	; 
//__weak const uint16_t A_OTF_MIN_SYNC_SPEED          @(FLASH_USER_START_ADDR + (2 *   Index_OTF_MIN_SYNC_SPEED          )  ) = default_OTF_MIN_SYNC_SPEED   	; 
//
//__weak const uint16_t A_DECEL_CONTROL               @(FLASH_USER_START_ADDR + (2 *   Index_DECEL_CONTROL               )  ) = default_DECEL_CONTROL   	        ;
//__weak const uint16_t A_BK_LOWSIDE_DURATION         @(FLASH_USER_START_ADDR + (2 *   Index_BK_LOWSIDE_DURATION         )  ) = default_BK_LOWSIDE_DURATION   	;
//__weak const uint16_t A_BK_ENDSPEED                 @(FLASH_USER_START_ADDR + (2 *   Index_BK_ENDSPEED                 )  ) = default_BK_ENDSPEED    	        ;
//__weak const uint16_t A_BK_CURRENTSEEDING           @(FLASH_USER_START_ADDR + (2 *   Index_BK_CURRENTSEEDING           )  ) = default_BK_CURRENTSEEDING         ;    	        ;
//
// //RPa: parameters necessary for auto-tune
//__weak const uint16_t A_MOTOR_INERTIA               @(FLASH_USER_START_ADDR + (2 *   Index_MOTOR_INERTIA               )  ) = (uint16_t)default_MOTOR_INERTIA   	        ;
//__weak const uint16_t A_FAN_INERTIA                 @(FLASH_USER_START_ADDR + (2 *   Index_FAN_INERTIA                 )  ) = (uint16_t)FAN_INERTIA   	                ;
//__weak const uint16_t A_MOTOR_VISCOUS_DAMPING       @(FLASH_USER_START_ADDR + (2 *   Index_MOTOR_VISCOUS_DAMPING       )  ) = (uint16_t)default_MOTOR_VISCOUS_DAMPING 	;
//__weak const uint16_t A_RAMPEND_CURRENT             @(FLASH_USER_START_ADDR + (2 *   Index_RAMPEND_CURRENT             )  ) = default_RAMPEND_CURRENT   	;
//__weak const uint16_t A_RAMP_STEP                   @(FLASH_USER_START_ADDR + (2 *   Index_RAMP_STEP                   )  ) = default_RAMP_STEP   	        ;
//__weak const uint16_t A_SPEED_TRANSITION            @(FLASH_USER_START_ADDR + (2 *   Index_SPEED_TRANSITION            )  ) = default_SPEED_TRANSITION   	;
//__weak const uint16_t A_LOWERIQLIMIT                @(FLASH_USER_START_ADDR + (2 *   Index_LOWERIQLIMIT                )  ) = default_LOWERIQLIMIT       	;
//
//  //parameters necessary for windmilling
//__weak const uint16_t A_WM_MAX_REVERSE_SPEED        @(FLASH_USER_START_ADDR + (2 *   Index_WM_MAX_REVERSE_SPEED        )  ) = default_WM_MAX_REVERSE_SPEED      ;
//__weak const uint16_t A_WM_CLAMP_DURATION           @(FLASH_USER_START_ADDR + (2 *   Index_WM_CLAMP_DURATION           )  ) = default_WM_CLAMP_DURATION   	;
//__weak const uint16_t A_WM_CLAMP_CURRENT            @(FLASH_USER_START_ADDR + (2 *   Index_WM_CLAMP_CURRENT            )  ) = default_WM_CLAMP_CURRENT          ;
//__weak const uint16_t A_WM_REVERSE_ENDSPEED         @(FLASH_USER_START_ADDR + (2 *   Index_WM_REVERSE_ENDSPEED         )  ) = default_WM_BK_REVERSE_ENDSPEED   	;
//__weak const uint16_t A_WM_CLAMP_RAMP               @(FLASH_USER_START_ADDR + (2 *   Index_WM_CLAMP_RAMP               )  ) = default_WM_BK_CLAMP_RAMP   	;
//__weak const uint16_t A_WM_SHORTCOIL_DURATION       @(FLASH_USER_START_ADDR + (2 *   Index_WM_SHORTCOIL_DURATION       )  ) = default_WM_SHORTCOIL_DURATION     ;
//
//__weak const uint16_t D_HALL_SENSORS_PLACEMENT      @(FLASH_USER_START_ADDR + (2 * Index_D_HALL_SENSORS_PLACEMENT      )  ) = HALL_SENSORS_PLACEMENT     	; 
//__weak const uint16_t D_HALL_PHASE_SHIFT            @(FLASH_USER_START_ADDR + (2 * Index_D_HALL_PHASE_SHIFT            )  ) = HALL_PHASE_SHIFT           	; 
//__weak const uint16_t D_M1_ENCODER_PPR              @(FLASH_USER_START_ADDR + (2 * Index_D_M1_ENCODER_PPR              )  ) = M1_ENCODER_PPR             	; 
//
//  // additional drive parameters
//__weak const uint16_t A_TF_KPDIV                    @(FLASH_USER_START_ADDR + (2 * Index_TF_KPDIV                      )  ) = TF_KPDIV             	; 
//__weak const uint16_t A_TF_KIDIV                    @(FLASH_USER_START_ADDR + (2 * Index_TF_KIDIV                      )  ) = TF_KIDIV             	; 
//__weak const uint16_t A_TF_KDDIV                    @(FLASH_USER_START_ADDR + (2 * Index_TF_KDDIV                      )  ) = TF_KDDIV             	; 
//__weak const uint16_t A_SP_KPDIV                    @(FLASH_USER_START_ADDR + (2 * Index_SP_KPDIV                      )  ) = SP_KPDIV             	; 
//__weak const uint16_t A_SP_KIDIV                    @(FLASH_USER_START_ADDR + (2 * Index_SP_KIDIV                      )  ) = SP_KIDIV             	; 
//__weak const uint16_t A_SP_KDDIV                    @(FLASH_USER_START_ADDR + (2 * Index_SP_KDDIV                      )  ) = SP_KDDIV             	; 
//__weak const uint16_t A_F1                          @(FLASH_USER_START_ADDR + (2 * Index_F1                            )  ) = F1             	; 
//__weak const uint16_t A_F2                          @(FLASH_USER_START_ADDR + (2 * Index_F2                            )  ) = F2             	; 
//__weak const uint16_t A_PLL_KPDIV                   @(FLASH_USER_START_ADDR + (2 * Index_PLL_KPDIV                     )  ) = PLL_KPDIV             	; 
//__weak const uint16_t A_PLL_KIDIV                   @(FLASH_USER_START_ADDR + (2 * Index_PLL_KIDIV                     )  ) = PLL_KIDIV             	; 
//
//#ifdef GAIN1 
//  __weak const int16_t  A_GAIN1                       @(FLASH_USER_START_ADDR + (2 * Index_A_GAIN1                     )  ) = GAIN1				; //!!!!!!!!!!!!!!!!!!
//  __weak const int16_t  A_GAIN2                       @(FLASH_USER_START_ADDR + (2 * Index_A_GAIN2                     )  ) = GAIN2                      	; //!!!!!!!!!!!!!!!!!!
//#else
//__weak const uint16_t D_CORD_GAIN1                  @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_GAIN1                  )  ) = CORD_GAIN1         		; //!!!!!!!!!!!!!!!!
//__weak const uint16_t D_CORD_GAIN2                  @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_GAIN2                  )  ) = CORD_GAIN2         		; //!!!!!!!!!!!!!!!!
//__weak const uint16_t D_CORD_MAX_ACCEL_DPPP         @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_MAX_ACCEL_DPPP         )  ) = CORD_MAX_ACCEL_DPPP	        ;  //!!!!!!!!!!!!!!!!!!
//#endif //GAIN1   //pll or cord
//
////__weak const uint16_t Z_DRIVE_FIRMWARE_VERSION      @(FLASH_USER_START_ADDR + (2 * Index_DRIVE_FW_VERSION              )  ) = DRIVE_FW_VERSION	                ;

 
//uint8_t flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress);
//int8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress);
//uint8_t flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length);
//uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress);
//uint8_t isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t _NumOfPage);
//uint16_t FlashRead(unsigned char* aDataBuf, uint16_t offsetByte);
//uint16_t FlashBufDeRegistered(uint16_t _offset, uint16_t* _returnBuf);
 

//extern __weak const uint16_t A_RS;
//extern __weak const uint16_t A_LS;
 /*
0x800f000  0x0f,  0x00,  0xff,  0xff,  0xff,  0xff,  0x64,  0x15,  0xd0,  0x07,  0x00,  0x00,  0x3c,  0x06,  0x71,  0x00,  
0x800f010  0x10,  0x27,  0x61,  0x08,  0xa9,  0x07,  0x61,  0x08,  0xa9,  0x07,  0x4c,  0x1d,  0xf4,  0x01,  0x64,  0x15, 
 
0x800f020  0x00,  0x00,  0x26,  0x02,  0x23,  0x00,  0x5a,  0x00,  0xff,  0xff,  0xe8,  0x03,  0x00,  0x00,  0x8e,  0x08,  
0x800f030  0x88,  0x13,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  

0x800f040  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x19,  0x00,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  
0x800f050  0x18,  0xa1,  0xb3,  0x6a,  0x2c,  0x01,  0xc4,  0x09,  0x4b,  0x00,  0xff,  0xff,  0xff,  0xff,  0x04,  0x00, 
 
0x800f060  0xe8,  0x03,  0x06,  0x00,  0xd0,  0x07,  0x10,  0x27,  0xe8,  0x03,  0x0a,  0x00,  0xc8,  0x00,  0xb8,  0x0b,  
0x800f070  0x0a,  0x00,  0xc8,  0x00,  0x21,  0x00,  0x0a,  0x00,  0x30,  0x75,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,

0x800f7ff  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 
0x800f7f0  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0x90,  0x08, 
*/ 
  

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ZZ_MODULE_FLASH_H_ */