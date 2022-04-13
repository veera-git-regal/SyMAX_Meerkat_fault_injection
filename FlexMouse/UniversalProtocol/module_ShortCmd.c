/**
  ***************************************************************************************************
  * @file    module_ShortCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    1-Jul-2020
  * @brief   Decode and perform group 1 and 2 CMD
  * @note    This App decode Group1 and 2 CMD in Universal protocol
  ***************************************************************************************************
  */
#include "pmsm_motor_parameters.h"
#include "module_ShortCmd.h"
#include "mc_api.h"
#include "driver_usart1.h"
#include "ab_module_Mc_StateMachine.h"
#include "harmonic_injection.h"
#include "module_meerkat_interface_config.h"

extern uint64_t getSysCount(void);
void MotorBoard_PrepareToJumpToBootloader(void);

/* SysCon handle declaration */
extern ProcessInfo processInfoTable[];
extern uint8_t p_harmonic_injection_parameters_rx_u8[HARMONIC_COMP_CONFIG_LENGTH_BYTES];

Usart1_Control* usart1Control_ShortCmd;
Module_StateMachineControl*  module_StateMachineControl_ShortCmd;

enum AppStates {
    INIT_APP,
    RUN_APP,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

//uint16_t adcFilterVal = 0;

#define ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN 1
#if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define RX_SHORTCMD_LENGTH 32 // This buffer only catches the one 'short' message. 20210301: max message length is 15 bytes.
  #define FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH RX_SHORTCMD_LENGTH // Inclusive (this value is accepted) 
  unsigned char fixedProtocolBuf_ShortCmd_Length = 0;
  unsigned char fixedProtocolBuf_ShortCmd[FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH];
  unsigned char* protocolBuf_ShortCmd = fixedProtocolBuf_ShortCmd;
#else // if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0
  unsigned char* protocolBuf_ShortCmd;
#endif // if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0

volatile uint16_t meerkat_fault_occured;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t moduleShortCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8)                 
{ 
  uint8_t     returnStage = 0;  
  
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_ShortCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          uint8_t Mc_StateMachineindex  = getProcessInfoIndex(MODULE_MC_STATEMACHINE);              //return Process index from processInfo array with the MC_statemachine module
          module_StateMachineControl_ShortCmd = (Module_StateMachineControl*) ((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_ShortCmd).seqMemRXG1_2_u32) >= DataLen2 )
          { 
            // Fixed Length: No Need to Length Check here, buffer must be bigger than UniHeaderLen (constant)       
            #if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0
              if((protocolBuf_ShortCmd = (unsigned char*) realloc(protocolBuf_ShortCmd,DataLen2)) == NULL) reallocError++;     
            #endif // if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0
            RingBuf_Observe((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, 0, &DataLen2);  
            
            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_ShortCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;

            #if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN >= 1
              // Check for Buffer Space. Discard Data, if would cause overflow.
              // - Return if invalid.
              if (DataLen2 > FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH) { // Normal Case: Buffer Overflow
                // Read All Data (Clear the Buffer)
                while (DataLen2 > 0) {
                  if (DataLen2 > FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH) {
                    // REVIEW: Replace with RingBuf_ClearContents? Much less processing
                    unsigned int read_length = FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH;
                    RingBuf_ReadBlock((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, &read_length); //extract the whole frame
                    // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
                    DataLen2 -= FIXED_PROTOCOLBUF_SHORTCMD_MAX_LENGTH;
                  } else {
                    RingBuf_ReadBlock((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, &DataLen2); //extract the whole frame
                    // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &DataLen2);             //copy the complete frame into buffer
                    DataLen2 = 0;
                  }
                }
                // Exit Gracefully
                returnStage = RUN_APP; 
                return returnStage;
              }            
            #else // if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0
              if((protocolBuf_ShortCmd = (unsigned char*) realloc(protocolBuf_ShortCmd,DataLen2)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
            #endif // if ENABLE_PROTOCOLBUF_SHORTCMD_FIXED_LEN <= 0
            RingBuf_ReadBlock((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, &DataLen2); //extract the whole frame
            //decode and perform the CMD function
            switch(protocolBuf_ShortCmd[2])
            {
              case 0x00:
                {
                  MC_StartMotor1();
                  break;
                }
              case 0x01:
                {
                  MC_StopMotor1();                  
                  break;
                }
              case 0x02:
                {
                  MC_StopRampMotor1();
                  break;
                }
              case 0x03:
                {
                  MC_AcknowledgeFaultMotor1();
                  break;
                }
              case 0x21:
                {
                  int32_t speed_target = protocolBuf_ShortCmd[6];
                  speed_target += (int16_t) protocolBuf_ShortCmd[5] << 8;
                  (*module_StateMachineControl_ShortCmd).command_Speed = speed_target;
                  
                  uint8_t dir_target = (uint8_t) protocolBuf_ShortCmd[7];
                  
                  if (dir_target == 6)(*module_StateMachineControl_ShortCmd).motorDir = 1;
                  if (dir_target == 9) (*module_StateMachineControl_ShortCmd).motorDir = -1;
                  
                  break;
                }
			  case 0x30:  // Harmonic Injection Enable - is_harmonic_injection_allowed_u8 
			    {
				  p_harmonic_injection_parameters_rx_u8[0] = protocolBuf_ShortCmd[5];
				  break;
			    }
			  case 0x1f:  // Harmonic Injection Amplitudes - amplitude_s16
			    {
                              bool bNoError = FALSE;
                              switch(protocolBuf_ShortCmd[5])
                              {
                             case 0x00: // ADCCHECK
                              {
                                // Write Maximum Value for One Current Phase. This will trigger a 'Phase Coherence Fault'
                                uint16_t *ram_ptr = (uint16_t*)MEERKAT_FAULT_INJECT_ADC_ADDRESS;
                                *ram_ptr = 0x7FFF; 
                                bNoError = true;
                                MC_StopMotor1(); 
                               meerkat_fault_occured = 1;
                            
                              }
                              break;
                            case 0x01: // REGISTERCHECK
                              {
                                // Write Error Code Directly to  Register Check Test
                                // - REVIEW: Are there other ways to genuinely trigger failure of this test.
                                uint8_t *ram_ptr = (uint8_t*)MEERKAT_FAULT_INJECT_REGISTER_ADDRESS;
                                *ram_ptr = 0x01; 
                                bNoError = true;
                                 MC_StopMotor1(); 
                                  meerkat_fault_occured = 2;
                              
                              }
                              break;
                            case 0x02: // CLOCKCHECK
                              {
                                uint32_t *ram_ptr = (uint32_t*)MEERKAT_FAULT_INJECT_CLOCK_ADDRESS;
                                *ram_ptr++ = 0x00; 
                                *ram_ptr++ = 0x00;
                                *ram_ptr++ = 0x00;
                                *ram_ptr++ = 0x00;
                                bNoError = true;
                                 MC_StopMotor1(); 
                                  meerkat_fault_occured = 3;
                              
                              }
                              break;
                            case 0x03: // RAMCHECK
                              {
                                // Write Invalid Value to a Meerkat Shadow RAM address that isn't written to frequently (App Identifier).
                                uint8_t *ram_ptr = (uint8_t*)MEERKAT_FAULT_INJECT_RAM_ADDRESS;
                                *ram_ptr = 0x00;
                                bNoError = true;
                                 MC_StopMotor1(); 
                                  meerkat_fault_occured = 4;
                               
                              }
                              break;
                            case 0x04: // ROMCHECK
                              {
                                // Set ROM Check to be 'in progress'
                                uint8_t *ram_ptr_stage = (uint8_t*)MEERKAT_FAULT_INJECT_ROM_STAGE_ADDRESS;
                                *ram_ptr_stage = 0x02; // 0x02 = ROM_CHECK_STAGE
                                // - adjust shadow ram so as not to fail the RAM Check 
                                // -- note: RAM Check could still fail if lighting strikes and ram check is looking at this address)
                                uint8_t *ram_ptr_stage_n = (uint8_t*)MEERKAT_FAULT_INJECT_ROM_STAGE_N_ADDRESS; // shadow ram
                                *ram_ptr_stage_n = ~0x01; // 0x01 = ROM_CHECK_STAGE


                                // Set the ROM CRC Start Value to a Random number to throw off the calculation
                                uint8_t *ram_ptr_crc = (uint8_t*)MEERKAT_FAULT_INJECT_ROM_CRC_ADDRESS;
                                *ram_ptr_crc = 0xAA; // 0x01 = ROM_CHECK_STAGE
                                // - adjust shadow ram so as not to fail the RAM Check 
                                // -- note: RAM Check could still fail if lighting strikes and ram check is looking at this address)
                                uint8_t *ram_ptr_crc_n = (uint8_t*)MEERKAT_FAULT_INJECT_ROM_CRC_N_ADDRESS;
                                *ram_ptr_crc_n = (uint8_t)(~0xAA); // 0x01 = ROM_CHECK_STAGE
                                bNoError = true;
                                 MC_StopMotor1();
                                 
                                  meerkat_fault_occured = 5;
                              
                              }
                              break;
                               //  bNoError = true;
          
				/*  p_harmonic_injection_parameters_rx_u8[1] = protocolBuf_ShortCmd[5];	// Harmonic 1, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[2] = protocolBuf_ShortCmd[6];	// Harmonic 1, Amplitude LSB
				  p_harmonic_injection_parameters_rx_u8[3] = protocolBuf_ShortCmd[7];	// Harmonic 2, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[4] = protocolBuf_ShortCmd[8];	// Harmonic 2, Amplitude LSB
				  p_harmonic_injection_parameters_rx_u8[5] = protocolBuf_ShortCmd[9];	// Harmonic 3, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[6] = protocolBuf_ShortCmd[10];	// Harmonic 3, Amplitude LSB
				  p_harmonic_injection_parameters_rx_u8[7] = protocolBuf_ShortCmd[11];	// Harmonic 4, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[8] = protocolBuf_ShortCmd[12];	// Harmonic 4, Amplitude LSB				
				  p_harmonic_injection_parameters_rx_u8[9] = protocolBuf_ShortCmd[13];	// Harmonic 5, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[10] = protocolBuf_ShortCmd[14];	// Harmonic 5, Amplitude LSB				
				  p_harmonic_injection_parameters_rx_u8[11] = protocolBuf_ShortCmd[15];	// Harmonic 6, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[12] = protocolBuf_ShortCmd[16];	// Harmonic 6, Amplitude LSB								  
				  p_harmonic_injection_parameters_rx_u8[13] = protocolBuf_ShortCmd[17];	// Harmonic 7, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[14] = protocolBuf_ShortCmd[18];	// Harmonic 7, Amplitude LSB								  
				  p_harmonic_injection_parameters_rx_u8[15] = protocolBuf_ShortCmd[19];	// Harmonic 8, Amplitude MSB
				  p_harmonic_injection_parameters_rx_u8[16] = protocolBuf_ShortCmd[20];	// Harmonic 8, Amplitude LSB								  
				  */
                              }
                                   break;
			    }
			  case 0x32:  // Harmonic Injection Angle Multiplier - angle_multiplier_u8
			    {
				  p_harmonic_injection_parameters_rx_u8[17] = protocolBuf_ShortCmd[5];	// Harmonic 1, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[18] = protocolBuf_ShortCmd[6];	// Harmonic 2, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[19] = protocolBuf_ShortCmd[7];	// Harmonic 3, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[20] = protocolBuf_ShortCmd[8];	// Harmonic 4, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[21] = protocolBuf_ShortCmd[9];	// Harmonic 5, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[22] = protocolBuf_ShortCmd[10];	// Harmonic 6, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[23] = protocolBuf_ShortCmd[11];	// Harmonic 7, Angle Multiplier
				  p_harmonic_injection_parameters_rx_u8[24] = protocolBuf_ShortCmd[12];	// Harmonic 8, Angle Multiplier												  
				  break;
			    }				
			  case 0x33:  // Harmonic Injection Angle Offset - angle_offset_u16
			    {
				  p_harmonic_injection_parameters_rx_u8[25] = protocolBuf_ShortCmd[5];	// Harmonic 1, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[26] = protocolBuf_ShortCmd[6];	// Harmonic 1, Angle Offset LSB
				  p_harmonic_injection_parameters_rx_u8[27] = protocolBuf_ShortCmd[7];	// Harmonic 2, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[28] = protocolBuf_ShortCmd[8];	// Harmonic 2, Angle Offset LSB
				  p_harmonic_injection_parameters_rx_u8[29] = protocolBuf_ShortCmd[9];	// Harmonic 3, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[30] = protocolBuf_ShortCmd[10];	// Harmonic 3, Angle Offset LSB
				  p_harmonic_injection_parameters_rx_u8[31] = protocolBuf_ShortCmd[11];	// Harmonic 4, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[32] = protocolBuf_ShortCmd[12];	// Harmonic 4, Angle Offset LSB				
				  p_harmonic_injection_parameters_rx_u8[33] = protocolBuf_ShortCmd[13];	// Harmonic 5, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[34] = protocolBuf_ShortCmd[14];	// Harmonic 5, Angle Offset LSB				
				  p_harmonic_injection_parameters_rx_u8[35] = protocolBuf_ShortCmd[15];	// Harmonic 6, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[36] = protocolBuf_ShortCmd[16];	// Harmonic 6, Angle Offset LSB								  
				  p_harmonic_injection_parameters_rx_u8[37] = protocolBuf_ShortCmd[17];	// Harmonic 7, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[38] = protocolBuf_ShortCmd[18];	// Harmonic 7, Angle Offset LSB								  
				  p_harmonic_injection_parameters_rx_u8[39] = protocolBuf_ShortCmd[19];	// Harmonic 8, Angle Offset MSB
				  p_harmonic_injection_parameters_rx_u8[40] = protocolBuf_ShortCmd[20];	// Harmonic 8, Angle Offset LSB								  				  
				  break;
			    }					
			  case 0x34:  // Harmonic Injection Inversion yes/no - is_phase_inverted_u8
			    {
				  p_harmonic_injection_parameters_rx_u8[41] = protocolBuf_ShortCmd[5];	// Harmonic 1, Inversion
				  p_harmonic_injection_parameters_rx_u8[42] = protocolBuf_ShortCmd[6];	// Harmonic 2, Inversion
				  p_harmonic_injection_parameters_rx_u8[43] = protocolBuf_ShortCmd[7];	// Harmonic 3, Inversion
				  p_harmonic_injection_parameters_rx_u8[44] = protocolBuf_ShortCmd[8];	// Harmonic 4, Inversion
				  p_harmonic_injection_parameters_rx_u8[45] = protocolBuf_ShortCmd[9];	// Harmonic 5, Inversion
				  p_harmonic_injection_parameters_rx_u8[46] = protocolBuf_ShortCmd[10];	// Harmonic 6, Inversion
				  p_harmonic_injection_parameters_rx_u8[47] = protocolBuf_ShortCmd[11];	// Harmonic 7, Inversion
				  p_harmonic_injection_parameters_rx_u8[48] = protocolBuf_ShortCmd[12];	// Harmonic 8, Inversion											  				  
				  break;
			    }
			  case 0x35:  // Harmonic Injection Minimum Speed - min_speed_u16
			    {
				  p_harmonic_injection_parameters_rx_u8[49] = protocolBuf_ShortCmd[5];	// Harmonic 1, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[50] = protocolBuf_ShortCmd[6];	// Harmonic 1, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[51] = protocolBuf_ShortCmd[7];	// Harmonic 2, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[52] = protocolBuf_ShortCmd[8];	// Harmonic 2, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[53] = protocolBuf_ShortCmd[9];	// Harmonic 3, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[54] = protocolBuf_ShortCmd[10];	// Harmonic 3, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[55] = protocolBuf_ShortCmd[11];	// Harmonic 4, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[56] = protocolBuf_ShortCmd[12];	// Harmonic 4, Min Speed LSB				
				  p_harmonic_injection_parameters_rx_u8[57] = protocolBuf_ShortCmd[13];	// Harmonic 5, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[58] = protocolBuf_ShortCmd[14];	// Harmonic 5, Min Speed LSB				
				  p_harmonic_injection_parameters_rx_u8[59] = protocolBuf_ShortCmd[15];	// Harmonic 6, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[60] = protocolBuf_ShortCmd[16];	// Harmonic 6, Min Speed LSB								  
				  p_harmonic_injection_parameters_rx_u8[61] = protocolBuf_ShortCmd[17];	// Harmonic 7, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[62] = protocolBuf_ShortCmd[18];	// Harmonic 7, Min Speed LSB								  
				  p_harmonic_injection_parameters_rx_u8[63] = protocolBuf_ShortCmd[19];	// Harmonic 8, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[64] = protocolBuf_ShortCmd[20];	// Harmonic 8, Min Speed LSB								  				  
				  break;
			    }					
			  case 0x36:  // Harmonic Injection Maximum Speed - max_speed_u16
			    {
				  p_harmonic_injection_parameters_rx_u8[65] = protocolBuf_ShortCmd[5];	// Harmonic 1, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[66] = protocolBuf_ShortCmd[6];	// Harmonic 1, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[67] = protocolBuf_ShortCmd[7];	// Harmonic 2, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[68] = protocolBuf_ShortCmd[8];	// Harmonic 2, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[69] = protocolBuf_ShortCmd[9];	// Harmonic 3, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[70] = protocolBuf_ShortCmd[10];	// Harmonic 3, Min Speed LSB
				  p_harmonic_injection_parameters_rx_u8[71] = protocolBuf_ShortCmd[11];	// Harmonic 4, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[72] = protocolBuf_ShortCmd[12];	// Harmonic 4, Min Speed LSB				
				  p_harmonic_injection_parameters_rx_u8[73] = protocolBuf_ShortCmd[13];	// Harmonic 5, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[74] = protocolBuf_ShortCmd[14];	// Harmonic 5, Min Speed LSB				
				  p_harmonic_injection_parameters_rx_u8[75] = protocolBuf_ShortCmd[15];	// Harmonic 6, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[76] = protocolBuf_ShortCmd[16];	// Harmonic 6, Min Speed LSB								  
				  p_harmonic_injection_parameters_rx_u8[77] = protocolBuf_ShortCmd[17];	// Harmonic 7, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[78] = protocolBuf_ShortCmd[18];	// Harmonic 7, Min Speed LSB								  
				  p_harmonic_injection_parameters_rx_u8[79] = protocolBuf_ShortCmd[19];	// Harmonic 8, Min Speed MSB
				  p_harmonic_injection_parameters_rx_u8[80] = protocolBuf_ShortCmd[20];	// Harmonic 8, Min Speed LSB								  				  				  
				  break;
			    }
                            
           
                            
               case UP_SHORT_CMD_ID_UTILITY:
              {
                Utility_ExecuteOperation(protocolBuf_ShortCmd[5], protocolBuf_ShortCmd[6], protocolBuf_ShortCmd[7]);
                //  MotorBoard_PrepareToJumpToBootloader();
              }
              default:
                break;
            }

          }
          
          returnStage = RUN_APP;
          break;
        }
      case IRQ_APP:
        {
          //if more than 1 driver interrupt attached to this APP
//           uint8_t index = getProcessInfoIndex(interruptIdentfer);         //return Process index from processInfo array of the driver interrupt call, APP can response respectively
          returnStage = RUN_APP;
          break;
        }               
      case STOP_APP:
        {
          returnStage = INIT_APP;
          break;
        }
      default:
        returnStage = STOP_APP;   
    }
  return returnStage;
}

// TODO: Below this line, move to their own file

#define UTILITY_FUNCTION_ID_JUMP_TO_PARTITION 1 
#define UTILITY_SECURITY_CODE 99
#define PARTITION_ID_BOOTLOADER 1
void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter, uint8_t function_subparameter) {
  switch (function_id)
  {
    case UTILITY_FUNCTION_ID_JUMP_TO_PARTITION:
      { // function_parameter = partition id, function_subparameter = sercurity code
        if (function_subparameter ==  UTILITY_SECURITY_CODE) { // only execute if security code was entered
          if (function_parameter == PARTITION_ID_BOOTLOADER) {
            MotorBoard_PrepareToJumpToBootloader();
          }
        }
        break;
      }
    default:
      {
        break;
      }

  }
}

// typedef void (*pFunction)(void);
volatile uint32_t BootloaderAddress = 0x1FFFD800;  // for custom bootloader
void MotorBoard_PrepareToJumpToBootloader(void) {
    // typedef void (*pFunction)(void);
    void (*SysMemBootJump)(void);
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(BootloaderAddress + 4)));

    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /** * Step: Disable all interrupts */
    __disable_irq();
    /* ARM Cortex-M Programming Guide to Memory Barrier Instructions.*/
    __DSB();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    /* Remap is bot visible at once. Execute some unrelated command! */
    __DSB();
    __ISB();

   /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) BootloaderAddress);
    SysMemBootJump();
}