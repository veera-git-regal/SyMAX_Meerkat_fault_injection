/**
********************************************************************************************************************************
* @file    module_flash.c 
* @author  Pamela Lee
* @brief   Main driver module for flash.
* @details This module initializes the flash
*          The ST motor libraries parameters will be mapped from the top of FLASH_USER_START_ADDR in 16bit data,
*          the data can be updated by using the function of uint8_t FlashDatSet(uint16_t _offset, uint16_t _flashDat), then 
*          data and offset will store in internal buffer as temporary data, user can either store all the temporary data into flash 
*          by flashPageUpdate(), or if the internal buffer is full will also update the temporary into flash.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "zz_module_flash.h"

#include "scheduler.h"
#include "regal_mc_settings.h"
#include "module_test.h"
#include "module_EEP.h"
#include "module_modbus.h"
#include "module_dynamic.h"

uint8_t flash_buffer_head_u8 = 0; // Starting Index of buffer that stored flash data that need to be copied to flash
uint8_t flash_buffer_tail_u8 = 0; // End index of buffer that stores flash data that need to be copied to flash

extern MotorId_Control motorId_Control;
extern ApplicationSpecificParameters_Control applicationSpecificParameters_Control;
extern HardwareSpecificParameters_Control hardwareSpecificParameters_Control;
//extern WindMillingParameters_Control windMillingParameters_Control;
extern BrakingParameters_Control brakingParameters_Control;
extern OtfParameters_Control otfParameters_Control;
//extern FanParameters_Control fanParameters_Control;
extern MotorProtections01_Control motorProtections01_Control;
extern MotorProtections02_Control motorProtections02_Control;
extern MotorLimits01_Control motorLimits01_Control;
extern HarmonicCompensation01_Control harmonicCompensation01_Control;
extern HarmonicCompensation02_Control harmonicCompensation02_Control;
extern HarmonicCompensation03_Control harmonicCompensation03_Control;
extern MotorTunning01_Control motorTunning01_Control;
extern MotorTunning02_Control motorTunning02_Control;
extern MotorParameters_Control motorParameters_Control;
extern StartupParameters_Control startupParameters_Control;
extern ModuleTest_Control moduleTest_Control;
extern StParameters01_Control stParameters01_Control;

extern ModbusRtu_Control modbusRtu_Control;
extern ModuleEEPROM_Control moduleEEPROM_Control;
extern ModuleTest_Control moduleTest_Control;
extern DriveData_Control  dynamic_data;
//
//extern  DriveDynamic_Control drive_dynamic_control;

// Buffer that stores data that need to flashed
typedef  struct 
{
  uint32_t address_u32;
  uint64_t flashData_u64;
}FlashBuf;

// Buffer that stores address offsets and pointers for both flash and RAM 
typedef struct
{
  uint16_t module_settings_begin_offset_u16;
  uint16_t module_settings_end_offset_u16;
  uint16_t module_settings_size_u16;
  uint8_t *module_settings_address_ptr;
  uint32_t module_flash_settings_address_ptr;
}FlashSettingsIndex;

FlashBuf flash_buffer[FLASH_BUFFER_SIZE]; // Buffer that stores data that need to flashed // Max stored bytes = FLASH_BUFFER_SIZE * 8 = 10*8 = 80 bytes
FlashSettingsIndex flash_settings_index[TOTAL_NUM_OF_PROCESSES]; // Buffer that stores address offsets and pointers for both flash and RAM
FlashSettingsIndex flash_settings_non_process_table[NON_PROCESS_TABLE_STRUCTURES_COUNT]; // Buffer that stores address offsets and pointed for both flash and RAM for structures that are not in processInfoTable

/* Content ---------------------------------------------------------------------------------------------------------------------*/

//extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
uint32_t Address = 0;                                     //, PageError = 0;
__IO uint8_t readData = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

uint8_t is_flash_Default_Init_Complete = TRUE; // Flag to trigger that default setting need to be init in flash
uint8_t is_flashInitComplete = FALSE; // Flag to trigger flash index init
uint8_t is_flashNeedsUpdate = FALSE; // Flag to trigger updating falsh with latest settings
uint8_t is_flashGetSettingsFromFlash = FALSE; // Flag to trigger getting setting from flash to RAM

uint8_t flash_status_u8 = 0;
uint32_t address_value_ptr = 0;
ModuleFlash_Control moduleFlash_Control;
static Ram_Buf_Handle module_Flash_StructMem_u32;
//********************************************************************************************************************************************************

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

void AssignModuleMemFlash(void);
void InitModuleFlashData(void);
void InitModuleFlashSettings(void);
void Get_Flash_Index(void);
void Flash_Buf_Init(void);

Flash_Status copy_Setting_RAM_To_Flash(uint32_t _TopageAddress);
Flash_Status copy_Data_To_Buffer(uint32_t index_u32, uint8_t* buff, uint8_t length_u8);
Flash_Status flashBlockProgram8Bytes(uint8_t drv_id_u8, uint32_t _TopageAddress, uint32_t _length);
Flash_Status check_flash_empty(uint32_t page_start_addres_u32, uint16_t page_size_u16);
uint8_t isFlash_Buf_Full(void);
Flash_Status compareRamToFlash(uint32_t);
void CopyRam2flash(uint8_t drv_id_u8, uint32_t pageAddress);
void CopyFlash2Ram(uint8_t drv_id_u8, uint32_t pageAddress);
void RecoverFlashSettings(uint8_t drv_id_u8);
void copy_meerkat_settings_to_ram(void);
Flash_Status  write_Meerkat_Settings(uint8_t drv_id_u8, uint32_t pageAddress);

//void Update_Flash(uint8_t module_id_u8, uint8_t flash_state_u8);
//uint32_t flashGetSettingAddress(uint8_t drv_id_u8, uint8_t *module_address_ptr, uint8_t *setting_address_ptr);
//Flash_Status update_Flash_Settings(uint8_t module_id_u8);
//Flash_Status writeDataToFlash(uint8_t module_id_u8, uint32_t _TopageAddress, uint8_t* ptr_u8, uint8_t dataLen_u8 );

//internal buffer management function
//void FlashBufInit(void); 
//void FlashFlushBuf(void);
//uint8_t FlashDataSet(uint16_t _offset, uint64_t _flashDat);
//uint8_t IsFlashBufFull(void);
//uint8_t Reg2Ram(uint32_t _RegNum, uint16_t _Value);

Flash_Status status_u16;

uint8_t moduleFlash_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) 
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemFlash(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      if(is_flashInitComplete == FALSE) // Flash index is not initilized
      {
        Get_Flash_Index(); 
        is_flashInitComplete = TRUE;
      }
      InitModuleFlashData();     // Init flash daat
      InitModuleFlashSettings(); // Init flash settings
      
      if(check_flash_empty(FLASH_DEFAULT_SETTINGS_ADDR, PAGE_SIZE) == FLASH_NOT_EMPTY)
      { // Check if flash page is empty
        if(isFlashCRCValid(FLASH_DEFAULT_SETTINGS_ADDR, FLASH_SETTINGS_VERSION_ADDR_OFFSET) == FLASH_CRC_VALID)
        { // Validate CRC
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashEmpty = FALSE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashCrcValid = TRUE;
        }
      }
      if(check_flash_empty(FLASH_SETTINGS_START_ADDR, PAGE_SIZE) == FLASH_NOT_EMPTY)
      { // Check if flash page is empty
        if(isFlashCRCValid(FLASH_SETTINGS_START_ADDR, FLASH_SETTINGS_VERSION_ADDR_OFFSET) == FLASH_CRC_VALID)
        { // Validate CRC          
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty = FALSE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashCrcValid = TRUE;
        }
      }     
      
      // Procedure to recover default settings from user settings or vice versa
      RecoverFlashSettings(drv_id_u8); 
      
      if( (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty == FALSE) &&
          (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashCrcValid == TRUE) )
      {
        CopyFlash2Ram(drv_id_u8, FLASH_SETTINGS_START_ADDR); //CopyRam2flash(drv_id_u8, FLASH_SETTINGS_START_ADDR); 
      }
      
      init_flash_parmeters();
      
      return_state_u8 = RUN_MODULE;
      break;
    }
  case RUN_MODULE: 
    {
      // Not needed since we are going to use flash commands to initilized flash
//      if(is_flash_Default_Init_Complete == FALSE) // Default settings are not initilized
//      {
//        if(check_flash_empty(FLASH_DEFAULT_SETTINGS_ADDR, PAGE_SIZE) != FLASH_NOT_EMPTY) // Default settings are not populated into flash
//        {
//          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = copy_Setting_RAM_To_Flash(FLASH_DEFAULT_SETTINGS_ADDR);          
//        }
//        is_flash_Default_Init_Complete = TRUE;
//      }
//      if(is_flashNeedsUpdate == TRUE) // Flash data is not initilized
//      {
//        if(check_flash_empty(FLASH_SETTINGS_START_ADDR, PAGE_SIZE) != FLASH_NOT_EMPTY) // Settings are not populated into flash
//        {
//          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = copy_Setting_RAM_To_Flash(FLASH_SETTINGS_START_ADDR);
//        }
//        is_flashNeedsUpdate = FALSE;
//      }
      if(moduleTest_Control.moduleTest_Data.userDiscretes01.is_adminMode == FALSE)
      { // Check if admin mode
        if(moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 > FLASH_READ_FLASH2RAM_DEFAULT_SETTINGS_CMD)
        { // Prevent user from performing admin operations
          moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD;
          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR; // Incorrect command
        }
      }  
      switch(moduleFlash_Control.moduleFlash_Settings.flashCommands_u16) 
      {
      case FLASH_WAITING_FOR_CMD:
        {
          break; // Waiting for flash commands
        }        
      case FLASH_WRITE_RAM2FLASH_USER_SETTINGS_CMD:
        { // Copies all module settings from RAM to flash page FLASH_SETTINGS_START_ADDR
          CopyRam2flash(drv_id_u8, FLASH_SETTINGS_START_ADDR);
          break; 
        }
      case FLASH_READ_FLASH2RAM_USER_SETTINGS_CMD:
        { // Copies all user settings from flash to module structures in RAM
          CopyFlash2Ram(drv_id_u8, FLASH_SETTINGS_START_ADDR);
          break;
        } 
      case FLASH_READ_FLASH2RAM_DEFAULT_SETTINGS_CMD:
        { // Copies all default settings from flash to module structures in RAM
          CopyFlash2Ram(drv_id_u8, FLASH_DEFAULT_SETTINGS_ADDR);
          break;
        }
      
      // ADMIN only commands
      case FLASH_WRITE_RAM2FLASH_DEFAULT_SETTINGS_CMD:
        { // Copies all module settings from RAM to flash page FLASH_DEFAULT_SETTINGS_ADDR
          CopyRam2flash(drv_id_u8, FLASH_DEFAULT_SETTINGS_ADDR);
          break; 
        }     
      case FLASH_COPY_USER_SETTINGS_TO_DEFAULTS_CMD:
        { // Copy user settings into default settings (replace factor settings)
          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_COPY_IN_PROCESS;
          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashPageErase(drv_id_u8, FLASH_DEFAULT_SETTINGS_ADDR); // Erase default settings
            if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 == FLASH_ERASE_COMPLETE)
            {
              if(moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_flashPageEraseError == TRUE)
              {
                if (flashPageCopy(drv_id_u8, FLASH_SETTINGS_START_ADDR, FLASH_DEFAULT_SETTINGS_ADDR) == FALSE)
                {
                  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_copyFlash2UserDeaultFlashSuccess = TRUE;
                  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_WRITE_COMPLETE;
                }
                moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD;
              }
            }
          break;
        }
      case FLASH_SETTINGS_INIT:
        {
          break;
        }
      case FLASH_DEFAULT_SETTINGS_INIT:
        {
          break;
        }
      case FLASH_SETTINGS_UPDATE:
        {
          break;
        }
      case FLASH_CRC_UPDATE:
        {
          break;
        }
        
      case FLASH_ERASE_CMD:
          { // Erase a pirticular flash page whose address is pageAddress_u32
             moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERASE_IN_PROCESS; // Identifies that erase command recieved  
             if ( (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_DEFAULT_SETTINGS_ADDR) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_SETTINGS_START_ADDR) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_SAFETYCORE_1) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_SAFETYCORE_2) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_SAFETYCORE_3) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_SAFETYCORE_3) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_BOOTLOADER_1) &&
                  (moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 != FLASH_BOOTLOADER_2) )
             {
            moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
          }else
          {
            moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashPageErase(drv_id_u8, moduleFlash_Control.moduleFlash_Settings.pageAddress_u32);
            moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 = 0;
            moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD; 
          }
          break;
        }
        
      case FLASH_WRITE_CMD: // Dont implement this. Only save data from RAM to FLASH. No direct access to falsh
        {
          break; 
        }
        
      default: //FLASH_UNKNOWN_CMD = 0xFFFF
        {
          break;
        }
      }
      
      return_state_u8 = RUN_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
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

/**
********************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for digital outputs module
* @param   None 
* @return  None
********************************************************************************
*/
void AssignModuleMemFlash(void) 
{
  module_Flash_StructMem_u32 =
    StructMem_CreateInstance(MODULE_FLASH, sizeof(ModuleFlash_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*module_Flash_StructMem_u32).p_ramBuf_u8 =
    (uint8_t *) &moduleFlash_Control; // Map the ADC1 memory into the structured memory
   uint8_t module_modbus_index_u8 = getProcessInfoIndex(MODULE_FLASH);
   processInfoTable[module_modbus_index_u8].Sched_ModuleData.p_masterSharedMem_u32 =
     (Ram_Buf_Handle) module_Flash_StructMem_u32;
}

/**
********************************************************************************
* @brief   Init Modbus Settings
* @details 
* @param   None 
* @return  None
********************************************************************************
*/
void InitModuleFlashData(void) {
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_OK;     // Current status of flash  
  
  moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess = FALSE;     // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyRam2UserFlashSuccess = FALSE;     // discrete bit indicators
  
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_copyRam2DefaultFlashSuccess = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty = TRUE;     // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashEmpty = TRUE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_copyFlash2UserDeaultFlashSuccess = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashCrcValid = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashCrcValid = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashRecoveredFromDefaults = FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashRecoveredFromUser =FALSE;  // discrete bit indicators
  moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_flashPageEraseError = FALSE; 
}

/**
********************************************************************************
* @brief   Init Modbus Data
* @details 
* @param   None 
* @return  None
********************************************************************************
*/  
void InitModuleFlashSettings(void) {
  moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD;     // Current status of flash  
  moduleFlash_Control.moduleFlash_Settings.adminFlags01_u16.is_driveFlashEnable = 1;  // flag bit indicators
}

/** =========================== private functions =========================================== **/
/**
* @brief  Erase page/s of flash data
* @param  drv_id_u8   The function caller module ID in case error occur within this function
*         pageAddress The starting address of the flash page
* @retval successful
*/
Flash_Status flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress)                 //pam tested
{
  Flash_Status returnValue = FLASH_ERASE_COMPLETE;
  uint32_t PageError = 0;
  //volatile uint32_t page_number_u32 = 0;
  
  /** Unlock the Flash to enable the flash control register access **/
  HAL_FLASH_Unlock();
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  //page_number_u32 = (uint32_t)( (pageAddress - FLASH_START_ADDR) / (float)PAGE_SIZE) ;
  EraseInitStruct.PageAddress = pageAddress;
  EraseInitStruct.NbPages     = (uint32_t)NUMBER_OF_FLASH_PAGES;
  //// Clear Flash error flags which was set in previous programming sequence
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
  /**                                                Flash erase                                             **/
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)   //22ms
  {  // Error occurred while page erase.                                                                           
    setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, sizeof(uint32_t), (uint8_t*)(&PageError));     //report the error address 
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
    moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_flashPageEraseError = TRUE;
    returnValue = FLASH_ERROR;
  }
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();    
  return returnValue;
}

/**
* @brief  Copy data from RAM into Flash for all moduels
* @param  drv_id_u8   The function caller module ID in case error occur within this function
*         pageAddress_u32 The starting address of the flash page
* @retval NONE
*/
void CopyRam2flash(uint8_t drv_id_u8, uint32_t pageAddress_u32)
{
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERASE_IN_PROCESS; // Identifies that erase command recieved             
  copy_meerkat_settings_to_ram();
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashPageErase(drv_id_u8, pageAddress_u32);
  if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  != FLASH_ERROR)
  { // Write to flash
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_WRITE_IN_PROCESS; 
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = copy_Setting_RAM_To_Flash(pageAddress_u32);
  }
  if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  != FLASH_ERROR)
  { // Write CRC and version
    //moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashWriteCRC32Version(drv_id_u8, pageAddress_u32);
    //moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashWriteCRC32Version(drv_id_u8, pageAddress_u32,  FLASH_SETTINGS_VERSION_ADDR_OFFSET);
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashWriteCRC32Version(drv_id_u8, pageAddress_u32, FLASH_SETTINGS_VERSION_ADDR_OFFSET);
  }
  uint32_t safety_settings_address = pageAddress_u32 + FLASH_SAFETY_SETTINGS_START_ADDR_OFFSET;
  if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  != FLASH_ERROR)
  { // Write Safety core settings
    
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = write_Meerkat_Settings(drv_id_u8, safety_settings_address);
  }
  // do not write version and crc. This is written by Meerkat
//  if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  != FLASH_ERROR)
//  { // Write CRC and version
//    //moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashWriteCRC32Version(drv_id_u8, FLASH_SAFETY_SETTINGS_START_ADDRESS, FLASH_SAFETY_SETTINGS_SIZE);
//    //uint32_t address_safety_settings_start_u32 = pageAddress_u32 + FLASH_SETTINGS_VERSION_ADDR_OFFSET + 8;
//    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = flashWriteCRC32Version(drv_id_u8, safety_settings_address, FLASH_SAFETY_SETTINGS_SIZE);
//  }
  moduleFlash_Control.moduleFlash_Settings.pageAddress_u32 = 0;
  if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  != FLASH_ERROR)
  { // Identify that write is complete
    // Compare settings in RAM vs user settting in FLASH
    //moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = compareRamToFlash(pageAddress_u32); // FLASH_OK == PASS else FAIL
    //if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  == COMPARE_RAM_TO_FLASH_PASS)
    if( compareRamToFlash(pageAddress_u32)  == COMPARE_RAM_TO_FLASH_PASS )
    { // RAM and FLASH matches
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_WRITE_COMPLETE;
      if(pageAddress_u32 == FLASH_DEFAULT_SETTINGS_ADDR)
      {
        moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_copyRam2DefaultFlashSuccess = TRUE;
        moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyRam2UserFlashSuccess = FALSE;
      }
      if(pageAddress_u32 == FLASH_SETTINGS_START_ADDR)
      {
        moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyRam2UserFlashSuccess = TRUE;
        moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_copyRam2DefaultFlashSuccess = FALSE;
      }
    }else
    {
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
    }
  }
  moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD;  
}

/**
* @brief  Preserve Meerkat setting in Flash by copying them to RAM
* @param  NONE
* @retval NONE
*/
uint8_t meerkat_settings_backup_u8[FLASH_SAFETY_SETTINGS_SIZE];
void copy_meerkat_settings_to_ram()
{
  for(uint16_t index_u16=0; index_u16 < FLASH_SAFETY_SETTINGS_SIZE; index_u16++)
  { // Copy data from flash address to backup array
    meerkat_settings_backup_u8[index_u16] =  *((uint8_t *)(FLASH_SAFETY_SETTINGS_START_ADDRESS + index_u16));
    //meerkat_settings_backup_u8[index_u16] = index_u16; // TODO, statement is for testing only // Populates sequential data into falsh for testing
  }  
} 

/**
* @brief  Copy the preserved Meerkat setting back to Flash
* @param  drv_id_u8   The function caller module ID in case error occur within this function
*         pageAddress The starting address of the Meerkat settings
* @retval NONE
*/
Flash_Status  write_Meerkat_Settings(uint8_t drv_id_u8, uint32_t pageAddress)
{
  return(flashBlockCopy(drv_id_u8, pageAddress, meerkat_settings_backup_u8, FLASH_SAFETY_SETTINGS_SIZE));
}

/**
* @brief  Copy data from FLASH into RAM for all moduels
* @param  drv_id_u8   The function caller module ID in case error occur within this function
*         pageAddress The starting address of the flash page
* @retval NONE
*/
void CopyFlash2Ram(uint8_t drv_id_u8, uint32_t pageAddress)
{
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_READ_IN_PROCESS;
  if(check_flash_empty(pageAddress, PAGE_SIZE) == FLASH_NOT_EMPTY)
  { // Check if flag page is empty 
    if(isFlashCRCValid(pageAddress, FLASH_SETTINGS_VERSION_ADDR_OFFSET) == FLASH_CRC_VALID)
    { // Validate CRC
      uint8_t is_defaultSettings = FALSE;
      if(pageAddress == FLASH_DEFAULT_SETTINGS_ADDR)
      {
        is_defaultSettings = TRUE;
      }
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = copy_Setting_Flash_To_RAM(drv_id_u8, is_defaultSettings);
      if(moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 != FLASH_READ_ERROR)
      {
        if(compareRamToFlash(pageAddress) == COMPARE_RAM_TO_FLASH_PASS)
        { 
          moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_READ_COMPLETE;
        }
      }
      // Compare settings in RAM vs user settting in FLASH
      //moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = compareRamToFlash(pageAddress); // FLASH_OK == PASS else FAIL
      if( moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 == FLASH_READ_COMPLETE )
      {
        if(pageAddress == FLASH_SETTINGS_START_ADDR)
        {
          moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess = TRUE;
          moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess = FALSE;
        }
        if(pageAddress == FLASH_DEFAULT_SETTINGS_ADDR)
        {       
          moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess = TRUE;
          moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess = FALSE;
        }
      }else
      {
        moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess = FALSE; 
        moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess = FALSE;
      }
    }else
    {
      moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess = FALSE; 
      moduleFlash_Control.moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess = FALSE;
    }
    
    moduleFlash_Control.moduleFlash_Settings.flashCommands_u16 = FLASH_WAITING_FOR_CMD;
  }else
  {
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_READ_ERROR;
  }
}            

/**
* @brief  Copy data from default page to User page and vice versa if one of the pages is empty
* @param  drv_id_u8   The function caller module ID in case error occur within this function
* @retval NONE
*/
void RecoverFlashSettings(uint8_t drv_id_u8)
{
  // Procedure to recover default settings from user settings
  if(moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashEmpty == TRUE)
  { // No setting is defaults page
    if( (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty == 1) && (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashCrcValid == FALSE) )
    { // Settings not populated in both default page and user settings page, or no setting in default page and user page CRC is not valid
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = FLASH_ERROR;
    }else
    { // Settings not populated in user page but no present in default settings page
      // Copy settings from user setting page to default page
      if (flashPageErase(drv_id_u8, FLASH_DEFAULT_SETTINGS_ADDR) != FLASH_ERROR)  // Erase flash to prevent copying error
      {
        if (flashPageCopy(drv_id_u8, FLASH_SETTINGS_START_ADDR, FLASH_DEFAULT_SETTINGS_ADDR) == FALSE)
        { // Flash page copy success and not empty anymore
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashEmpty = FALSE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashRecoveredFromUser = TRUE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashCrcValid = TRUE;
        }
      }else
      {
        moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = FLASH_ERROR;
      }          
    }
  }
  // Procedure to recover user settings from defaults
  if(moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty  == TRUE)
  { // No setting is user page
    if( (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashEmpty == TRUE) && (moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_defaultFlashCrcValid == FALSE) )
    { // Settings not populated in both default page and user settings page or  no setting in user page and default page CRC is not valid
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = FLASH_ERROR;
    }else
    { // Settings not populated in defaults page but not presetnt in user page
      // Copy settings from default to user setting page
      if (flashPageErase(drv_id_u8, FLASH_SETTINGS_START_ADDR) != FLASH_ERROR) // Erase flash to prevent copying error
      {
        if (flashPageCopy(drv_id_u8, FLASH_DEFAULT_SETTINGS_ADDR ,FLASH_SETTINGS_START_ADDR) == FALSE)
        { // Flash page copy success and not empty anymore
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashEmpty = FALSE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashRecoveredFromDefaults = TRUE;
          moduleFlash_Control.moduleFlash_Data.adminDiscretes01_u16.is_userFlashCrcValid = TRUE;
        }
      }else
      {
        moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = FLASH_ERROR;
      }          
    }
  }
}

/**
* @brief  Program a block of data into flash with input buffer
* @param  drv_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*         _buf            Buffer of data to burn into flash
*          _length        The starting address of the source flash page/s
* @retval successful
*/
Flash_Status flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length)                  //Pam Tested
{
  Flash_Status returnValue = FLASH_WRITE_COMPLETE;
  uint64_t data_u64 = 0;
  HAL_FLASH_Unlock();
  
  for(uint16_t index = 0  ;  index < _length ; index += FLASH_COPY_BYTE_SIZE)
  {
    if(FLASH_COPY_BYTE_SIZE == 8)
    {
      data_u64 = (((uint64_t)_buf[index]) << 56)     + (((uint64_t)_buf[index + 1]) << 48) + 
        (((uint64_t)_buf[index + 2]) << 40) + (((uint64_t)_buf[index + 3]) << 32) + 
          (((uint64_t)_buf[index + 4]) << 24) + (((uint64_t)_buf[index + 5]) << 16) + 
            (((uint64_t)_buf[index + 6]) << 8)  +  _buf[index +7];
    }
    else if(FLASH_COPY_BYTE_SIZE == 2)
    {
      data_u64 = (uint64_t) ((uint64_t)(_buf[index + 1] << 8 ) + (uint64_t)(_buf[index] << 8 ) );
    }
	
    //// Clear Flash error flags which was set in previous programming sequence
    //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);

    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + index), data_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
      returnValue = FLASH_ERROR;
    }  
  }
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return returnValue;
}

/**
* @brief  Program a block of data into flash with input buffer
* @param  drv_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*         _buf            Buffer of data to burn into flash
*          _length        The starting address of the source flash page/s
* @retval successful
*/
Flash_Status flashBlockCopy(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length)                  //Pam Tested
{
  Flash_Status returnValue = FLASH_WRITE_COMPLETE;
  uint64_t data_u64 = 0;
  HAL_FLASH_Unlock();
  
  for(uint16_t index = 0  ;  index < _length ; index += FLASH_COPY_BYTE_SIZE)
  {
    if(FLASH_COPY_BYTE_SIZE == 8)
    {
      data_u64 = (((uint64_t)_buf[index + 7]) << 56)     + (((uint64_t)_buf[index + 6]) << 48) + 
        (((uint64_t)_buf[index + 5]) << 40) + (((uint64_t)_buf[index + 4]) << 32) + 
          (((uint64_t)_buf[index + 3]) << 24) + (((uint64_t)_buf[index + 2]) << 16) + 
            (((uint64_t)_buf[index + 1]) << 8)  +  _buf[index];
    }
    else if(FLASH_COPY_BYTE_SIZE == 2)
    {
      data_u64 = (uint64_t) ((uint64_t)(_buf[index] << 8 ) + (uint64_t)(_buf[index + 1] << 8 ) );
    }
	
    //// Clear Flash error flags which was set in previous programming sequence
    //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);

    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + index), data_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
      returnValue = FLASH_ERROR;
    }  
  }
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return returnValue;
}

/**
* @brief  Check last word of flash page to verify if CRC is valid
* @param  _FLASH_START_ADDR      FLASH page starting address
*         NUMBER_OF_FLASH_PAGES  Number of sector for CRC check ( @Caution each page will contains two bytes of CRC and is accumulated                                             
*                                                                 for example> page0 CRC= page0, if also has page1 CRC in the last two bytes of page1 will be from page0 to page1 )
* @retval successful
*/
Flash_Status isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t size_u16)
{ 
  uint32_t uwCRCValue_u32 = 0;
  uint32_t FlashCRCValue_u32 = 0;
  // TODO: Current CRC32 only returns u16
  uwCRCValue_u32 = Calculate_CRC32(size_u16 , (unsigned char*)_FLASH_START_ADDR);   // Skip flash version and CRC bytes //TODO: Current CRC32 does not inlcude flash version 
  
  uint8_t* crc_address_u32 = (uint8_t *)(_FLASH_START_ADDR + size_u16 + 4); // Last 4 bytes is CRC, +4 to skip version
  FlashCRCValue_u32 = (((uint32_t)*(crc_address_u32))) + (((uint32_t)*(crc_address_u32 + 1)) << 8) +
    (((uint32_t)*(crc_address_u32 + 2)) << 16) + (((uint32_t)*(crc_address_u32 + 3))<<24);
  
  if(uwCRCValue_u32 == FlashCRCValue_u32) 
    return(FLASH_CRC_VALID);
  else
  {
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_CRC_ERROR;
    return (FLASH_CRC_ERROR);
  }
}

/**
* @brief  Read 16bit data from flash
* @param  aDataBuf buffer where the data need to be copied to
*         offsetByte  offset of the physical flash address (as the offset in byte [caution !!! this is absolute offset address in flash !!!])
* @retval data (16bit)
*/
uint16_t FlashRead16Bits(unsigned char* aDataBuf, uint16_t offsetByte)       //this read a word(16bitS)
{
  return((((uint16_t)aDataBuf[offsetByte]) << 8) +  aDataBuf[offsetByte +1]);
}

/**
* @brief  Read 64bits from flash
* @param  pageAddress flash page starting address
*         offsetByte  offset of the physical flash address (as the offset in byte [caution !!! this is absolute offset address in flash !!!])
* @retval data (64bit)
*/
uint64_t FlashRead64Bits(unsigned char* aDataBuf, uint16_t offsetByte) //this reads 64 bits
{
  return( (((uint64_t)aDataBuf[offsetByte + 7]) << 56) + (((uint64_t)aDataBuf[offsetByte + 6]) << 48) + 
         (((uint64_t)aDataBuf[offsetByte + 5]) << 40) + (((uint64_t)aDataBuf[offsetByte + 4]) << 32) + 
           (((uint64_t)aDataBuf[offsetByte + 3]) << 24) + (((uint64_t)aDataBuf[offsetByte + 2]) << 16) + 
             (((uint64_t)aDataBuf[offsetByte + 1]) << 8)  +  aDataBuf[offsetByte] );
}


/**
* @brief  Init flash_buffer to clear any junk values
* @param  None
*         
* @retval None
*/
void Flash_Buf_Init(void)
{
  for(uint8_t index = 0; index < FLASH_BUFFER_SIZE ; index++)
  {
    flash_buffer[index].address_u32 = 0xffff;
    flash_buffer[index].flashData_u64 = 0;
  }  
  flash_buffer_head_u8 = 0;
  flash_buffer_tail_u8 = 0;
}


/**
* @brief  Get all the addresses, begin/end address offset for settings structures of modules
* @param  None
*         
* @retval None
*/
uint16_t current_Index_u16 = 0;
void Get_Flash_Index(void)
{
  for(uint16_t current_Id_u16= MIN_MODULE_ID; current_Id_u16< TOTAL_NUM_OF_PROCESSES; current_Id_u16++)
  {
    // Module settings begin offset (previous module settings end index)
    flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
    //uint8_t module_Index_u8 = 0;
    uint16_t settings_size_u16 = 0;
    //uint16_t module_address_offset_u8 = 0;
    switch(current_Id_u16)
    {
    case MODULE_FLASH:	
      { 
        // NOTE: Dont save FLASH module to FLASH
        
        //flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
        //settings_size_u16 = sizeof(moduleFlash_Control.moduleFlash_Settings);      
        break;
      }
    case MODULE_MC_STATEMACHINE:	
      { 
        // NOTE: Dont save MC Statemachine module to FLASH
              
        break;
      }
    case MODULE_DYNAMIC:
      { 
        // NOTE: DriveData_Settings struct should not be saved to FLASH
        
        //flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        //flash_settings_index[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(dynamic_data.driveData_Settings));
        //settings_size_u16 = sizeof(dynamic_data.driveData_Settings);
        break;
      }
    case MODULE_MODBUS:
      {
        flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_index[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(modbusRtu_Control.modbusRtu_Settings));
        settings_size_u16 = sizeof(modbusRtu_Control.modbusRtu_Settings);
        break;
      }
    case MODULE_TEST:
      {
        // NOTE: MODULE TEST struct should not be saved to FALSH
        
        //flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        //flash_settings_index[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(moduleTest_Control.moduleTest_Settings));
        //settings_size_u16 = sizeof(moduleTest_Control.moduleTest_Settings);
        break;
      }
    case MODULE_EEP_CMD:
      {
        // MODULE EEPROM struct should not be saved to FALSH
        
        //flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        //flash_settings_index[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(moduleEEPROM_Control.moduleEEPROM_Settings));
        //settings_size_u16 = sizeof(moduleEEPROM_Control.moduleEEPROM_Settings); 
        break;
      }
    default:
      {
        break;
      }  
    }
    if(settings_size_u16 != 0)
    {
      // Module settings end offset (# of bytes + begin offset)
      current_Index_u16 = current_Index_u16 + MAX_MODULE_STRUCTURE_SIZE_BYTES;
      flash_settings_index[current_Id_u16].module_settings_size_u16 = settings_size_u16;       // Size of settings structure. This does not have the 8 byte padding
      flash_settings_index[current_Id_u16].module_settings_end_offset_u16= current_Index_u16;  // Number of Bytes // Module Memory Index_End // Last address of the analog settings
    }
  }
  for(uint16_t current_Id_u16= 0; current_Id_u16< NON_PROCESS_TABLE_STRUCTURES_COUNT; current_Id_u16++)
  {
    uint16_t settings_size_u16 = 0;
    flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16;
    switch(current_Id_u16)
    {
    case MOTOR_ID_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorId_Control.motorId_Settings));
        settings_size_u16 = sizeof(motorId_Control.motorId_Settings);
        break;
      }
    case STARTUP_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(startupParameters_Control.startupParameters_Settings));
        settings_size_u16 = sizeof(startupParameters_Control.startupParameters_Settings);
        break;
      }
    case MOTOR_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorParameters_Control.motorParameters_Settings));
        settings_size_u16 = sizeof(motorParameters_Control.motorParameters_Settings);
        break;
      }
    case TUNING_SETTINGS01:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorTunning01_Control.motorTunning01_Settings));
        settings_size_u16 = sizeof(motorTunning01_Control.motorTunning01_Settings);
        break;
      }      
    case TUNING_SETTINGS02:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorTunning02_Control.motorTunning02_Settings));
        settings_size_u16 = sizeof(motorTunning02_Control.motorTunning02_Settings);
        break;
      }
    case LIMITS_SETTINGS01:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorLimits01_Control.motorLimits01_Settings));
        settings_size_u16 = sizeof(motorLimits01_Control.motorLimits01_Settings);
        break;
      }
    case PROTECTION_SETTINGS01:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorProtections01_Control.motorProtections01_Settings));
        settings_size_u16 = sizeof(motorProtections01_Control.motorProtections01_Settings);
        break;
      }
    case PROTECTION_SETTINGS02:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(motorProtections02_Control.motorProtections02_Settings));
        settings_size_u16 = sizeof(motorProtections01_Control.motorProtections01_Settings);
        break;
      }    
    case BRAKING_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(brakingParameters_Control.brakingParameters_Settings));
        settings_size_u16 = sizeof(brakingParameters_Control.brakingParameters_Settings);
        break;
      }
    case OTF_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(otfParameters_Control.otfParameters_Settings));
        settings_size_u16 = sizeof(otfParameters_Control.otfParameters_Settings);
        break;
      }
    //     case WINDMILL_SETTINGS:
      //      {
      //        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
      //        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(windMillingParameters_Control.windMillingParameters_Settings));
      //        settings_size_u16 = sizeof(windMillingParameters_Control.windMillingParameters_Settings);
      //        break;
      //      }
    case HW_SPECIFIC_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings));
        settings_size_u16 = sizeof(hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings);
        break;
      }
    case APP_SPECIFIC_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(applicationSpecificParameters_Control.applicationSpecificParameters_Settings));
        settings_size_u16 = sizeof(applicationSpecificParameters_Control.applicationSpecificParameters_Settings);
        break;
      }    
    case HARMONICS_COMPENSATION_1_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(harmonicCompensation01_Control.harmonicCompensation01_Settings));
        settings_size_u16 = sizeof(harmonicCompensation01_Control.harmonicCompensation01_Settings);
        break;
      }
    case HARMONICS_COMPENSATION_2_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(harmonicCompensation02_Control.harmonicCompensation02_Settings));
        settings_size_u16 = sizeof(harmonicCompensation02_Control.harmonicCompensation02_Settings);
        break;
      }
      
    case HARMONICS_COMPENSATION_3_SETTINGS:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(harmonicCompensation03_Control.harmonicCompensation03_Settings));
        settings_size_u16 = sizeof(harmonicCompensation03_Control.harmonicCompensation03_Settings);
        break;
      }    
    case ST_MC_SETTINGS01:
      {
        flash_settings_non_process_table[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
        flash_settings_non_process_table[current_Id_u16].module_settings_address_ptr = (uint8_t*)(&(stParameters01_Control.stParameters01_Settings));
        settings_size_u16 = sizeof(stParameters01_Control.stParameters01_Settings);
        break;        
      }
    default:
      {
        break;
      }
    }
    if(settings_size_u16 != 0)
    {
      // Module settings end offset (# of bytes + begin offset)
      current_Index_u16 = current_Index_u16 + MAX_MODULE_STRUCTURE_SIZE_BYTES;
      // Module settings end offset (# of bytes + begin offset)
      flash_settings_non_process_table[current_Id_u16].module_settings_size_u16 = settings_size_u16;       // Size of settings structure. This does not have the 8 byte padding
      flash_settings_non_process_table[current_Id_u16].module_settings_end_offset_u16= current_Index_u16;  // Number of Bytes // Module Memory Index_End // Last address of the analog settings
    }
  }    
}

/**
* @brief  Copies the data from each module(RAM) into flash
* @param  _TopageAddress  Flash Page address where the data have to be copied to
*       
* @retval Flash_Status FLASH_SETTINGS_WRITE_COMPLETE
FLASH_OUT_OF_RANGE_ERROR
*/
Flash_Status copy_Setting_RAM_To_Flash(uint32_t _TopageAddress)
{
  uint16_t current_flash_index_u16 = 0;
  Flash_Status error_u16 = FLASH_SETTINGS_WRITE_COMPLETE;
  uint8_t current_Id = 0;
  for(current_Id= MIN_MODULE_ID; current_Id< TOTAL_NUM_OF_PROCESSES; current_Id++)
  {
    if( (current_Id != MODULE_FLASH) ) // && (current_Id != MODULE_TEST) ) // Skip saving these modules setting to FLASH
    {
      if( (flash_settings_index[current_Id].module_settings_size_u16 != 0) && (flash_settings_index[current_Id].module_settings_size_u16 != 0xFFFF))
      { 
        Flash_Buf_Init(); // Clear any junk values in buffer
        uint16_t module_settings_index_u8= 0;
        // Copy FLASH_COPY_BYTE_SIZE bytes at a time into flash_buffer
        //uint8_t module_byte_count_u8 = 0;
        
        module_settings_index_u8 = flash_settings_index[current_Id].module_settings_size_u16 ;
        
        if( (_TopageAddress + flash_settings_index[current_Id].module_settings_end_offset_u16) > (_TopageAddress + FLASH_PAGE_SIZE) )
        {
          error_u16 = FLASH_OUT_OF_RANGE_ERROR;
          return(error_u16);
        }else
        {
          uint8_t settings_offset_u8 = 0;
          current_flash_index_u16 = flash_settings_index[current_Id].module_settings_begin_offset_u16;
          flash_settings_index[current_Id].module_flash_settings_address_ptr = flash_settings_index[current_Id].module_settings_begin_offset_u16 + _TopageAddress;
          while(module_settings_index_u8!= 0)
          {
            uint8_t length_bytes_u8 = 0;
            
            uint32_t address_u32 = _TopageAddress + current_flash_index_u16;
            
            if(module_settings_index_u8 < FLASH_COPY_BYTE_SIZE)
            {
              length_bytes_u8 = module_settings_index_u8;
            }else
            {
              length_bytes_u8 = FLASH_COPY_BYTE_SIZE;
              
            }
            module_settings_index_u8 = module_settings_index_u8 - length_bytes_u8;
            if(length_bytes_u8 != 0)
            { // Copy data to buffer
              copy_Data_To_Buffer( address_u32, (flash_settings_index[current_Id].module_settings_address_ptr + settings_offset_u8), length_bytes_u8 ); // 
              current_flash_index_u16 = current_flash_index_u16 + FLASH_COPY_BYTE_SIZE;
              settings_offset_u8 = settings_offset_u8 + length_bytes_u8;
            } 
          }
        }
        if(flash_buffer_tail_u8 != flash_buffer_head_u8)
        { // Write data to FLASH
          error_u16 = flashBlockProgram8Bytes( current_Id,(flash_settings_index[current_Id].module_settings_begin_offset_u16+_TopageAddress), flash_buffer_tail_u8);
        }
      }      
    }
  }
  
  // Copy non processInfoTable structures into FLASH from RAM
  for(uint16_t current_id_u16= 0; current_id_u16< NON_PROCESS_TABLE_STRUCTURES_COUNT; current_id_u16++)
  {
    //if( (current_id_u16 != MODULE_FLASH) || (current_id_u16 != MODULE_TEST) ) // Skip saving flash module setting to FLASH
    //{
    if( (flash_settings_non_process_table[current_id_u16].module_settings_size_u16 != 0) && (flash_settings_non_process_table[current_id_u16].module_settings_size_u16 != 0xFFFF) )
    { 
      Flash_Buf_Init(); // Clear any junk values in buffer
      uint16_t module_settings_index_u8= 0;
      // Copy FLASH_COPY_BYTE_SIZE bytes at a time into flash_buffer
      module_settings_index_u8 = flash_settings_non_process_table[current_id_u16].module_settings_size_u16 ;
      if( (_TopageAddress + flash_settings_non_process_table[current_id_u16].module_settings_end_offset_u16) > (_TopageAddress + FLASH_PAGE_SIZE) )
      {
        error_u16 = FLASH_OUT_OF_RANGE_ERROR;
        return(error_u16);
      }else
      {
        uint8_t settings_offset_u8 = 0;
        current_flash_index_u16 = flash_settings_non_process_table[current_id_u16].module_settings_begin_offset_u16;
        flash_settings_non_process_table[current_id_u16].module_flash_settings_address_ptr = flash_settings_non_process_table[current_id_u16].module_settings_begin_offset_u16 + _TopageAddress;
        while(module_settings_index_u8!= 0)
        {
          uint8_t length_bytes_u8 = 0;
          
          uint32_t address_u32 = _TopageAddress + current_flash_index_u16;
          
          if(module_settings_index_u8 < FLASH_COPY_BYTE_SIZE)
          {
            length_bytes_u8 = module_settings_index_u8;
          }else
          {
            length_bytes_u8 = FLASH_COPY_BYTE_SIZE;
            
          }
          module_settings_index_u8 = module_settings_index_u8 - length_bytes_u8;
          if(length_bytes_u8 != 0)
          { // Copy data to buffer
            copy_Data_To_Buffer( address_u32, (flash_settings_non_process_table[current_id_u16].module_settings_address_ptr + settings_offset_u8), length_bytes_u8 ); // 
            current_flash_index_u16 = current_flash_index_u16 + FLASH_COPY_BYTE_SIZE;
            settings_offset_u8 = settings_offset_u8 + length_bytes_u8;
          } 
        }
      }
      if(flash_buffer_tail_u8 != flash_buffer_head_u8)
      { // Write to FLASH
        error_u16 = flashBlockProgram8Bytes( current_id_u16,(flash_settings_non_process_table[current_id_u16].module_settings_begin_offset_u16+_TopageAddress), flash_buffer_tail_u8);
      }
    }      
    //}
  }
  return(error_u16);  
}


/**
* @brief  Copy 8 bytes of data to buffer that needs to be flashed into flash
* @param  index_u32  Index of where the data need to be flashed
*         ptr_u8     Address from where we have to copy data
* @retval None
*/
Flash_Status copy_Data_To_Buffer(uint32_t index_u32, uint8_t* ptr_u8, uint8_t length_u8)
{
  flash_buffer[flash_buffer_tail_u8].address_u32 = index_u32;  
  uint64_t data_u64 = 0; //0xFFFFFFFFFFFFFFFF;
  if(length_u8 < FLASH_COPY_BYTE_SIZE)
  {
    uint64_t temp_u64 = 0xFFFFFFFFFFFFFFFF;
    temp_u64 = temp_u64 << (length_u8*8);
    data_u64 = temp_u64;
  }
  if(!isFlash_Buf_Full())
  {
    for(uint8_t byte_u8 = 0; byte_u8 < length_u8; byte_u8++)
    {
      data_u64 = (uint64_t)((uint64_t)data_u64 + (uint64_t)((uint64_t)(*(ptr_u8+(byte_u8))) << (byte_u8*8)) ); 
    }
    flash_buffer[flash_buffer_tail_u8].flashData_u64 = data_u64;
    flash_buffer_tail_u8++;
    
    return(FLASH_COPY_TO_BUF_COMPLETE);
  }else
  {
    return(FLASH_BUF_FULL); 
  }  
}


/**
* @brief  Check if flash buffer is full
* @param  None
* @retval ture/false
*/
uint8_t isFlash_Buf_Full(void)
{
  int16_t result = (int16_t)flash_buffer_head_u8 - (int16_t)flash_buffer_tail_u8;
  if( (result == -(FLASH_BUFFER_SIZE-1)) || (result == 1)) 
  { 
    return (true);
  }  
  return (false);
}




/**
* @brief  Program a data in flash_buffer into flash
* @param  module_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*          _length        The starting address of the source flash page/s
* @retval successful
*/
Flash_Status flashBlockProgram8Bytes(uint8_t module_id_u8, uint32_t _TopageAddress, uint32_t _length)
{
  Flash_Status returnValue = FLASH_WRITE_COMPLETE;
  HAL_FLASH_Unlock();
  for(uint16_t index_u16 = flash_buffer_head_u8  ;  index_u16 < _length ; index_u16 ++)
  {
    //// Clear Flash error flags which was set in previous programming sequence
    //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
     
    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_buffer[index_u16].address_u32, flash_buffer[index_u16].flashData_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
      returnValue = FLASH_ERROR;
    } else
    {
      flash_buffer_head_u8++;
    }
  }
  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) 
  HAL_FLASH_Lock();
  return returnValue;
}

/**
* @brief  Program a data in flash_buffer into flash
* @param  module_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*          _length        The starting address of the source flash page/s
* @retval successful
*/
Flash_Status flashBlockProgram2Bytes(uint8_t module_id_u8, uint32_t _TopageAddress, uint32_t _length)
{
  Flash_Status returnValue = FLASH_WRITE_COMPLETE;
  HAL_FLASH_Unlock();
  for(uint16_t index_u16 = flash_buffer_head_u8  ;  index_u16 < _length ; index_u16 ++)
  {
    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_buffer[index_u16].address_u32, flash_buffer[index_u16].flashData_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
      returnValue = FLASH_ERROR;
    } else
    {
      flash_buffer_head_u8++;
    }
  }
  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) 
  HAL_FLASH_Lock();
  return returnValue;
}


/**
* @brief  Check if flash is empty by looking at the CRC
* @param  module_id_u8                 The function caller module ID in case error occur within this function
*         *module_address_ptr       Module settings strucuture address
*         *setting_address_ptr      Address of setting in the structure
* @retval setting_address_flash_ptr Address of setting in flash memory
*/

Flash_Status check_flash_empty(uint32_t page_start_address_u32, uint16_t page_size_u16)
{
  uint64_t crc_flash_data_u64= 0;
  uint8_t *crc_address_u32;
  crc_address_u32 = (uint8_t *)(page_start_address_u32 + page_size_u16 - 8); // Last 4 bytes is CRC
  crc_flash_data_u64 = ((uint64_t)*crc_address_u32) << 56 ;
  crc_flash_data_u64 = crc_flash_data_u64 + (((uint64_t)*(crc_address_u32 + 1)) << 48) +
    (((uint64_t)*(crc_address_u32 + 2)) << 40)  + (((uint64_t)*(crc_address_u32 + 3)) << 32) +
      (((uint64_t)*(crc_address_u32 + 4)) << 24) + (((uint64_t)*(crc_address_u32 + 5)) << 16) +
        (((uint64_t)*(crc_address_u32 + 6)) << 8) + ((uint64_t)*(crc_address_u32 + 7))  ;
  if( crc_flash_data_u64 == 0xFFFFFFFFFFFFFFFF)
  {
    return(FLASH_EMPTY); 
  }
  else
  {
    return(FLASH_NOT_EMPTY);  
  }  
}

/**
* @brief  Write CRC32 and flash version to flash
* @param  module_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
* @retval successful
*/
Flash_Status flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress, uint16_t size_u16)
{
  Flash_Status returnValue = FLASH_WRITE_CRC_COMPLETE;
  HAL_FLASH_Unlock();
  //TODO: Current CRC does not inlcude flash version
  //uint32_t uwCRCValue_u32 = Calculate_CRC32((uint32_t)((FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) - 8) , (unsigned char*)_TopageAddress);   // TODO: Get rid of compiler warning on ToPageAddress   // Skip flash version and CRC bytes
  uint32_t uwCRCValue_u32 = Calculate_CRC32(size_u16 , (unsigned char*)_TopageAddress);   // TODO: Get rid of compiler warning on ToPageAddress   // Skip flash version and CRC bytes
  //put calculated CRC back to the last word of the page
  uint64_t data_u64 = ((uint64_t)uwCRCValue_u32 << 32) + FLASH_SETTINGS_VERSION; // Flash writes MSB LSB in senond and first address resp
  //uint16_t offset_u16 = _TopageAddress + PAGE_SIZE - 8; // last 8 bytes are FLASH_VERSION (4 bytes) + CRC (4 bytes)
  uint32_t offset_address_u32 = _TopageAddress + size_u16; // last 8 bytes are FLASH_VERSION (4 bytes) + CRC (4 bytes)
  
  // // Clear Flash error flags which was set in previous programming sequence
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
     
  if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, offset_address_u32, data_u64) == HAL_OK) //write 64bit data
  {
    // Error occurred while writing data in Flash memory.                                                                 
    setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address
    moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = FLASH_ERROR;
    returnValue = FLASH_ERROR;
  }
  
  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) 
  HAL_FLASH_Lock();
  return returnValue;
}

Flash_Status copy_Setting_Flash_To_RAM(uint8_t module_id_u8, uint8_t isCopyDefaultSettings)
{ // Get setting from flash and update the RAM
  Flash_Status returnValue_u16 = FLASH_READ_ERROR;
  uint16_t length_u16 = 0; // Length of module data in falsh 
  uint32_t module_falsh_address_ptr;  // Address of structure in FALSH      
  uint8_t *module_settings_struct_address_ptr; // Address of structure in RAM
  //Flash_Status error_u16 = FLASH_SETTINGS_WRITE_COMPLETE;
  uint8_t current_Id = 0;
  int32_t settings_start_add_u32 = 0;
  
  if(isCopyDefaultSettings == TRUE)
  { // Caculate offset to add to the address point of user settings  
    settings_start_add_u32 = FLASH_DEFAULT_SETTINGS_ADDR;
  }else
  {
    settings_start_add_u32 = FLASH_SETTINGS_START_ADDR;
  }
  for(current_Id= MIN_MODULE_ID; current_Id< TOTAL_NUM_OF_PROCESSES; current_Id++)
  { 
    if( (current_Id != MODULE_FLASH) )// && (current_Id != MODULE_TEST) ) // Skip loading data from flash to RAM for these modules
    {
      module_settings_struct_address_ptr = flash_settings_index[current_Id].module_settings_address_ptr;
      length_u16 = flash_settings_index[current_Id].module_settings_size_u16;
      // Copy each byte info RAM from Flash
      if( (length_u16 > 0) && (length_u16 != 0xFFFF) ) // Data 
      {
        // Calculate the starting address in Flash for the module
        module_falsh_address_ptr = (uint32_t) (flash_settings_index[current_Id].module_settings_begin_offset_u16 + settings_start_add_u32);
        for (uint16_t index_u16 = 0; index_u16 < length_u16; index_u16++)
        {
          *(module_settings_struct_address_ptr + index_u16) = *((uint8_t *)module_falsh_address_ptr + index_u16); // Copy data from flash into RAM
        }
      }
    }
  }
  // Copy non processInfoTable structures
  for(uint16_t current_id_u16= 0; current_id_u16< NON_PROCESS_TABLE_STRUCTURES_COUNT; current_id_u16++)
  {
    module_settings_struct_address_ptr = flash_settings_non_process_table[current_id_u16].module_settings_address_ptr;
    length_u16 = flash_settings_non_process_table[current_id_u16].module_settings_size_u16;
    // Copy each byte info RAM from Flash
    if((length_u16 > 0) && (length_u16 != 0xFFFF)) // Data 
    {
      // Calculate the starting address in Flash for the module
      module_falsh_address_ptr = (uint32_t) (flash_settings_non_process_table[current_id_u16].module_settings_begin_offset_u16 + settings_start_add_u32);
      for (uint16_t index_u16 = 0; index_u16 < length_u16; index_u16++)
      {
        *(module_settings_struct_address_ptr + index_u16) = *((uint8_t *)module_falsh_address_ptr + index_u16); // Copy data from flash into RAM
      }
    }
  }
  returnValue_u16 = FLASH_READ_COMPLETE;
  return(returnValue_u16); 
}


/**
* @brief  Compare data in RAM to FLASH
* @param  module_id_u8    The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
ptr_u8          Pointer to where the data is stores
dataLen_u8      Length of data to write to flash
* @retval successful
*/
uint8_t ram_data_u8 = 0;
uint8_t flash_data_u8 = 0;
uint16_t current_id_u16 = 0;
uint8_t byte_count_u8 = 0;
uint16_t current_flash_index_u16 = 0;
uint32_t module_address_in_flash_u32 = 0;
Flash_Status compareRamToFlash(uint32_t page_address_p )
{
  Flash_Status error_u16 = COMPARE_RAM_TO_FLASH_PASS;
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16 = COMPARING_RAM_TO_FLASH;
  for(uint16_t current_Id= MIN_MODULE_ID; current_Id< TOTAL_NUM_OF_PROCESSES; current_Id++)
  {
    if( (current_Id != MODULE_FLASH) && (current_Id != MODULE_TEST) ) // Skip saving flash module setting to FLASH
    {
      uint8_t byte_count_u8 = 0;
      // Get current module index in FLASH
      uint16_t current_flash_index_u16 = flash_settings_index[current_Id].module_settings_begin_offset_u16;
      if(flash_settings_index[current_Id].module_settings_size_u16 != 0xFFFF)
      { 
        while(byte_count_u8 < flash_settings_index[current_Id].module_settings_size_u16)
        {  
          uint32_t address_u32 = page_address_p + current_flash_index_u16;
          if( address_u32 > (page_address_p + FLASH_PAGE_SIZE - 8) ) // Last 8 bytes for Flash Version (4 bytes) + CRC (4 bytes)
          {
            error_u16 = FLASH_OUT_OF_RANGE_ERROR;
            return(error_u16);
          }
          else
          {
            if(byte_count_u8 < flash_settings_index[current_Id].module_settings_size_u16)
            {
              ram_data_u8 = (*(flash_settings_index[current_Id].module_settings_address_ptr + byte_count_u8));
              flash_data_u8 = *((uint8_t *)address_u32);          
              if( ram_data_u8 != flash_data_u8 ) // Check if FLASH data matches RAM data
              {
                error_u16 = FLASH_DATA_MISMATCH;
                return(error_u16);
              }
            }
            byte_count_u8++;
            current_flash_index_u16++;
          }        
        }
      }
    }
  }
  
  // Compare non-processInfoTable structures in RAM to FLASH
  for(current_id_u16= 0; current_id_u16< NON_PROCESS_TABLE_STRUCTURES_COUNT; current_id_u16++)
  {    
    byte_count_u8 = 0;
    // Get current module index in FLASH
    current_flash_index_u16 = flash_settings_non_process_table[current_id_u16].module_settings_begin_offset_u16;
    
    if(flash_settings_non_process_table[current_id_u16].module_settings_size_u16 != 0xFFFF)
    { 
      while(byte_count_u8 < flash_settings_non_process_table[current_id_u16].module_settings_size_u16) // Compare byte at a time
      {  
        module_address_in_flash_u32 = page_address_p + (uint32_t)current_flash_index_u16;
        if( module_address_in_flash_u32 > (page_address_p + FLASH_PAGE_SIZE - 8) ) // Last 8 bytes for Flash Version (4 bytes) + CRC (4 bytes)
        {
          error_u16 = FLASH_OUT_OF_RANGE_ERROR;
          return(error_u16);
        }else
        {
          if(byte_count_u8 < flash_settings_non_process_table[current_id_u16].module_settings_size_u16)
          {
            ram_data_u8 = (*(flash_settings_non_process_table[current_id_u16].module_settings_address_ptr + byte_count_u8));
            flash_data_u8 = *((uint8_t *)module_address_in_flash_u32);          
            if( ram_data_u8 != flash_data_u8 ) // Check if FLASH data matches RAM data
            {
              error_u16 = FLASH_DATA_MISMATCH;
              return(error_u16);
            }
          }
          byte_count_u8++;
          current_flash_index_u16++;
        }      
      }
    }
  }  
  moduleFlash_Control.moduleFlash_Data.moduleFlashStatus_u16  = error_u16;
  return(error_u16);
}

/**
* @brief  Copy page of flash memory to another sector
* @param  drv_id_u8          The function caller module ID in case error occur within this function
*         _FrompageAddress   The starting address of the source flash page/s
*         _TopageAddress     The starting address of the sink flash page/s
* @retval successful
*/
uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
{
  /** Unlock the Flash to enable the flash control register access **/
  uint8_t error_value_u8 = 0;
  HAL_FLASH_Unlock();
  uint16_t indx = 0;
  uint64_t currentDat_u64;
  for(  ;  indx < (FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) ; indx += FLASH_COPY_BYTE_SIZE)
  {
    currentDat_u64 = FlashRead64Bits((unsigned char*)_FrompageAddress, indx); 
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + indx), (uint64_t)currentDat_u64) == HAL_OK) //write 16bit data
    {
      Address += FLASH_COPY_BYTE_SIZE;
    }
    else
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address
      error_value_u8 = 1;
    }  
  }     
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return error_value_u8;
}
