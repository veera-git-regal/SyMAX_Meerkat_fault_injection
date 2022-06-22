/**
********************************************************************************************************************************
* @file    module_Mc_StateMachine.c 
* @author  Pamela Lee
* @brief   The main motor interface with ST motor libraries.
* @details This module will keep track with the motor start/stop and running status, and perform startup fail retry, active breaking 
*          , on the fly startup and derating control for the motor
* @note    when de-rating was happening, the target speed will reduce to below the threshold value, will not go back to the orginial 
*          value until user override the speed with command again!
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "ab_module_Mc_StateMachine.h"
#include "mc_api.h"
#include "state_machine.h"
#include "mc_config.h"
#include "math.h"
#include "zz_module_flash.h"
#include "regal_mc_lib.h"
#include "driver_I2c.h"
#include "module_i2c.h"
#include "module_dynamic.h"

//#include "flash_parameters.h"
#include "regal_mc_settings.h"
//extern MotorId_Control motorId_Control;

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern MCT_Handle_t MCT[NBR_OF_MOTORS];
static MCT_Handle_t *pMCT = &MCT[M1];                          /* pointer on motor control tuning handler */
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];

extern ProcessInfo processInfoTable[];
Module_StateMachineControl  module_StateMachineControl;
static Ram_Buf_Handle Mc_StateMachineStructMem_u32;

uint16_t NTC_P_GetAvTemp_C(NTC_Handle_t * pHandle );

extern I2c_Control *i2cControl;

//extern WindMillingParameters_Control windMillingParameters_Control;
extern BrakingParameters_Control brakingParameters_Control;
extern OtfParameters_Control otfParameters_Control;
extern MotorProtections01_Control motorProtections01_Control;
extern MotorProtections02_Control motorProtections02_Control;
extern MotorLimits01_Control motorLimits01_Control;
extern ApplicationSpecificParameters_Control applicationSpecificParameters_Control;
extern StartupParameters_Control startupParameters_Control;
extern DriveData_Control  dynamic_data;

void InitStateMachineSettings();

#ifdef eepromInstall  
static void I2C_SendFault(uint16_t errorCode);
#endif
/************************************ All the setting should be mapped into FLash *********************************************/
////__weak __no_init const uint16_t   MIN_COMMANDABLE_SPEED	       @(FLASH_USER_START_ADDR + (2 *  Index_MIN_COMMANDABLE_SPEED	   ) );
//__weak const uint16_t   MIN_COMMANDABLE_SPEED	     @(FLASH_USER_START_ADDR + (2 *  Index_MIN_COMMANDABLE_SPEED	) ) = default_MIN_COMMANDABLE_SPEED       ;             
//__weak const uint16_t   MAX_COMMANDABLE_SPEED        @(FLASH_USER_START_ADDR + (2 *  Index_MAX_COMMANDABLE_SPEED        ) ) = default_MAX_COMMANDABLE_SPEED      ;             
//__weak const uint16_t   SPEED_UP_RAMP_RATE           @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_UP_RAMP_RATE           ) ) = default_SPEED_UP_RAMP_RATE       ;             
//__weak const uint16_t   SPEED_DOWN_RAMP_RATE         @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_DOWN_RAMP_RATE         ) ) = default_SPEED_DOWN_RAMP_RATE       ;             
//__weak const uint16_t   SPEED_CONSIDERED_STOPPED     @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_CONSIDERED_STOPPED     ) ) = default_CONSIDERED_STOPPED       ;             
//__weak const uint16_t   MotSpinTimeOut               @(FLASH_USER_START_ADDR + (2 *  Index_MotSpinTimeOut               ) ) = default_MOTSPINTIMEOUT          ;             
//__weak const uint16_t   SpinPollPeriod               @(FLASH_USER_START_ADDR + (2 *  Index_SpinPollPeriod               ) ) = default_SPINPOLLPERIOD       ;             
//__weak const uint16_t   numOfStartRetry              @(FLASH_USER_START_ADDR + (2 *  Index_numOfStartRetry              ) ) = default_NUMSTARTRETRY          ;             
//__weak const uint16_t   StartRetryPeriod             @(FLASH_USER_START_ADDR + (2 *  Index_StartRetryPeriod             ) ) = default_START_RETRY_PERIOD      ;             
//__weak const uint16_t   StartPeriodInc               @(FLASH_USER_START_ADDR + (2 *  Index_StartPeriodInc               ) ) = default_START_PERIOD_INC     ;             
//__weak const uint16_t   over_current_threshold       @(FLASH_USER_START_ADDR + (2 *  Index_over_current_threshold       ) ) = default_OCP_THRESHOLD      ;             
//__weak const uint16_t   over_current_lower_threshold @(FLASH_USER_START_ADDR + (2 *  Index_over_current_lower_threshold ) ) = default_OCP_LOWER_THRESHOLD      ; 
//
//__weak const uint16_t   over_current_rpm_Reduce      @(FLASH_USER_START_ADDR + (2 *  Index_over_current_rpm_Reduce      ) ) = default_OCP_RPM_REDUCE       ;             
//__weak const uint16_t   OvCurrent_derate_period      @(FLASH_USER_START_ADDR + (2 *  Index_OvCurrent_derate_period      ) ) = default_OCP_DERATE_PERIOD       ;             
//__weak const uint16_t   over_power_threshold         @(FLASH_USER_START_ADDR + (2 *  Index_over_power_threshold         ) ) = default_OVP_THRESHOLD     ;             
//__weak const uint16_t   over_power_lower_threshold   @(FLASH_USER_START_ADDR + (2 *  Index_over_power_lower_threshold   ) ) = default_OVP_LOWER_THRESHOLD      ;   
//
//__weak const uint16_t   over_power_rpm_Reduce        @(FLASH_USER_START_ADDR + (2 *  Index_over_power_rpm_Reduce        ) ) = default_OVP_RPM_REDUCE         ;             
//__weak const uint16_t   OvPower_derate_period        @(FLASH_USER_START_ADDR + (2 *  Index_OvPower_derate_period        ) ) = default_OVP_DERATE_PERIOD      ;             
//__weak const uint16_t   over_temperature_threshold   @(FLASH_USER_START_ADDR + (2 *  Index_over_temperature_threshold   ) ) = default_OTP_THRESHOLD         ;             
//__weak const uint16_t   over_temperature_lower_threshold @(FLASH_USER_START_ADDR + (2 *  Index_over_temperature_lower_threshold   ) ) = default_OTP_LOWER_THRESHOLD         ;    
//
//__weak const uint16_t   over_temperature_rpm_Reduce  @(FLASH_USER_START_ADDR + (2 *  Index_over_temperature_rpm_Reduce  ) ) = default_OTP_RPM_REDUCE        ;             
//__weak const uint16_t   OvTemp_derate_period         @(FLASH_USER_START_ADDR + (2 *  Index_OvTemp_derate_period         ) ) = default_OTP_DERATE_PERIOD     ;     

int32_t target_speed = 0;                               //Actual speed to submit into ST motor libaries                        
int32_t lastDeratingSpeed = 0;                          //Speed value break from the last Derating loop
bool currentDerating_lowThreshold_flag = FALSE;
bool powerDerating_lowThreshold_flag = FALSE;
int16_t act_dir = 1;                                    //actual direction while running
int16_t prev_dir = 1;                                   //previous direction when motor was running
bool autorestart = TRUE;                                //auto-restart, and can be used for debugging when disabled
/************************ Motor start spinning timing parameter  ********************************************************/
static uint16_t MotSpinPollCount = 0;
uint64_t tt_SpinPollTime;
/**********************************   Start-up retry  *******************************************/                           
/***  total re-start waiting period = StartRetryPeriod +(StartPeriodInc * numOfStartRetry) ms  **/
uint16_t StartRetryCounter = 0;
uint64_t tt_StartUpRetryTime;
/************************************* de_rating parameters  *********************************************/
uint64_t tt_derateCurrentPollTime;
uint64_t tt_deratePowerPollTime;

typedef union
{
  struct
  {
    uint8_t currentDrateTrg:1;          /*!< bit:      0  current triggered*/
    uint8_t powerDrateTrg:1;            /*!< bit:      1  power triggered*/
    uint8_t temptureDrateTrg:1;         /*!< bit:      2  temperature triggered */
    uint8_t _reserved3:1;               /*!< bit:      3  Reserved */
    uint8_t currentDerateDone:1;        /*!< bit:      4  current has just finished derating process */
    uint8_t powerDerateDone:1;          /*!< bit:      5  power has just finished derating process*/
    uint8_t temptureDerateDone:1;       /*!< bit:      6  temperature has just finished derating process */
    uint8_t _reserved7:1;               /*!< bit:      7  Reserved */
  };                                    /*!< Structure used for bit  access */
  uint8_t deratingTriggeredFlag;        /*!< Type      used for byte access */
}DERATING_STATUS;

DERATING_STATUS DeratingStatus;

/******************* Temperature derating parameters ******************************/
uint64_t tt_derateTempPollTime;
uint16_t midTemperatureThreshold_u16 = 0;
uint16_t marginOfMidTemperatureH_u16 = 0;
uint16_t marginOfMidTemperatureL_u16 = 0;
#define temperatureDefaultTimeOutV 10  //Timeout value is the multiple of "OvTemp_derate_period" (if OvTemp_derate_period = 1 second then it will timeout in 10 seconds)
uint16_t TemperatureTimeOut = temperatureDefaultTimeOutV; //counter for wait if no temperature change
//#define midTemperatureThreshold ((over_temperature_threshold - over_temperature_lower_threshold)/2)+ over_temperature_lower_threshold              //mid point of the upper and lower temperature threshold
//#define marginOfMidTemperatureH ((uint16_t)(midTemperatureThreshold + (over_temperature_threshold - over_temperature_lower_threshold) * 0.1))      //Temperature maintain upper region 
//#define marginOfMidTemperatureL ((uint16_t)(midTemperatureThreshold - (over_temperature_threshold - over_temperature_lower_threshold) * 0.4))      //Temperature maintain lower region
//uint16_t midTemperatureThreshold_V = 0;
//uint16_t marginOfMidTemperatureH_V = 0;
//uint16_t marginOfMidTemperatureL_V = 0;
uint16_t H_deltaTemperature_percent = 0;
int16_t L_deltaTemperature_percent = 0;

uint16_t H_dividerSpeed =0;
uint16_t L_dividerSpeed =0;

uint16_t lastTemperature = 0 ; //remeber the temperature before execute the last speed step
uint16_t CurrentIPM_Temperature = 0;

int32_t lastCommandSpeed = 0; //store the command speed before entering temperature-derating
uint8_t temperatureChanged = 0; //temperature had changed from last sample


/************************************* counters for delays  *********************************************/
uint64_t tt_StopWaitTime;                        //Stop and Brake counter for delay
uint64_t tt_StopTime;                           // Time while stopping (braking, coasting)
uint64_t tt_FaultOTFWaitTime_u64 = 0;            //Fault counter for delay
uint64_t tt_PowerupWaitTime_u64 = 0;             //Powerup counter for delay
/****************** local fault status ************************/
/** GMI_FaultStatus => 0x01 = start-up retry error            */
/**                    0x02 = ST_Motor fault error persisting */
uint8_t GMI_FaultStatus = 0;                            //GMI statemachine Error message status for report to module_err_logHandle      

int16_t tmpryTempature = 0;

/**Writing to EEPROM procedure#1: Build the State character array based on the Data Logger EEPROM module management specification  **/
unsigned char StateData[] = {0x02, 0x03, 0x10, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x01, 0x03};
/**Writing to EEPROM procedure#2: Acquire the length of the State character array **/
unsigned StateDataLen = sizeof(StateData);
uint8_t Mc_StateMachineStructMem_u32_buf[sizeof(Module_StateMachineControl)];
uint8_t module_Mc_StateMachine_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                                   uint8_t irq_id_u8) 
{
  uint8_t return_state_u8 = MEM_INIT_MODULE;
  /** pre-process the Motor stop speed command or fault status, both get higher priority in the state machine**/
  //updateAvrCurrent();
  
  if((next_State_u8 == MEM_INIT_MODULE) )
  {
    module_StateMachineControl.command_Speed = 0; //disable the motor
    return_state_u8 = INIT_MODULE;    
  }else if( (!module_StateMachineControl.motorEnable) && (next_State_u8 != INIT_MODULE) ){
    module_StateMachineControl.command_Speed = 0; //disable the motor
    return_state_u8 = IDLE_MODULE;  
  }
  if(next_State_u8 == MEM_INIT_MODULE)
  {
    return_state_u8 = INIT_MODULE;
  }
  else if((next_State_u8 != INIT_MODULE)&&( MC_GetSTMStateMotor1() == FAULT_NOW)) {//
    GMI_FaultStatus = 0x02;                                           //ST_Motor fault error persisting
    next_State_u8 = FAULT_REPORT_MODULE;                              //report fault and return
  }
  else if((next_State_u8 != INIT_MODULE)&&(MC_GetSTMStateMotor1() == FAULT_OVER)) next_State_u8 = FAULT_PROCESS_MODULE;                         //after fault over then can process the fault and restart             //
  else if((next_State_u8 != INIT_MODULE)&& (next_State_u8 != IRQ_MODULE))
  {        
    if(module_StateMachineControl.command_Speed == 0) {   // any situation see stop command will change to stop state, unless IRQ and stopping states
      switch (next_State_u8){
      case MOTOR_RUNNING_MODULE:
      case CURRENT_DERATING_MODULE:
      case POWER_DERATING_MODULE:
      case TEMPERATURE_DERATING_MODULE:
      case TEMPERATURE_DERATING_LOW_MODULE:
      case TEMPERATURE_DERATING_TrimDown_MODULE:
      case TEMPERATURE_DERATING_MaintainTemp_MODULE:
        next_State_u8 = STOP_MOTOR_MODULE;
      default:
        break;
      } 
    }
    else {  
      //RPa: direction commands 
      //if(module_StateMachineControl.command_Speed < MIN_COMMANDABLE_SPEED) module_StateMachineControl.command_Speed = MIN_COMMANDABLE_SPEED; 
      //if(module_StateMachineControl.command_Speed > MAX_COMMANDABLE_SPEED) module_StateMachineControl.command_Speed = MAX_COMMANDABLE_SPEED;  
      if(module_StateMachineControl.command_Speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16) module_StateMachineControl.command_Speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16; 
      if(module_StateMachineControl.command_Speed > applicationSpecificParameters_Control.applicationSpecificParameters_Settings.maxApplicationRpm_u16) module_StateMachineControl.command_Speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.maxApplicationRpm_u16;  
    }
  }
  module_StateMachineControl.current_State = (ModuleMotorStates)next_State_u8;
  /** pre-process the Motor stop speed command or fault status, both get higher priority in the state machine end**/
  
  switch (next_State_u8) 
  {
  case MEM_INIT_MODULE:
    {
      //AssignModuleMemFlash(); // Assign structured memory
      //////////////////////////////////////////////////////////////////////////////////
      //midTemperatureThreshold_V = midTemperatureThreshold;  //for debug only
      //marginOfMidTemperatureH_V = marginOfMidTemperatureH;  //for debug only
      //marginOfMidTemperatureL_V = marginOfMidTemperatureL;  //for debug only
      //////////////////////////////////////////////////////////////////
      
      Mc_StateMachineStructMem_u32 =  StructMem_CreateInstance(MODULE_MC_STATEMACHINE, sizeof(Module_StateMachineControl), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
      //Mc_StateMachineStructMem_u32->p_ramBuf_u8 = Mc_StateMachineStructMem_u32_buf;
      
      (*Mc_StateMachineStructMem_u32).p_ramBuf_u8 = (uint8_t *)&module_StateMachineControl ;    //map the generated module's control memory into the structured memory
      uint8_t module_Mc_StateMachine_Index = getProcessInfoIndex(MODULE_MC_STATEMACHINE);
      processInfoTable[module_Mc_StateMachine_Index].Sched_ModuleData.p_masterSharedMem_u32 =(Ram_Buf_Handle) Mc_StateMachineStructMem_u32; //also map it back to module_paramters under kernel 
      
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {          
      //init this module  
      /*** check the motor setting and init all motor setting ***/            
      module_StateMachineControl.command_Speed = 0;            
      module_StateMachineControl.errorCode_u16 = 0;
      module_StateMachineControl.motorDir = 1; //CW
      module_StateMachineControl.motorEnable = TRUE;            
      
      RegalSetting_Init(); //SPA Update motor/drive setting from FLASH to over write defaults
      InitStateMachineSettings();
      
#if VBUS_BASED_POWER_LIMIT == TRUE
            
       module_StateMachineControl.currentPowerLimit_u16 = Regal_CalcPowerLimitPerVbus();
#else
       module_StateMachineControl.currentPowerLimit_u16 = motorLimits01_Control.motorLimits01_Settings.outputPowerLimit_u16;
#endif
      //tt_PowerupWaitTime_u64 = getSysCount() + default_POWERUP_DURATION;
      tt_PowerupWaitTime_u64 = getSysCount() + startupParameters_Control.startupParameters_Settings.powerUpDelay_u16;
      //init motor flag
      return_state_u8 = IDLE_MODULE;
      break;
    }
  case IDLE_MODULE: 
    {
      if((module_StateMachineControl.command_Speed != 0)&&(autorestart==TRUE)&&(MC_GetSTMStateMotor1() == IDLE)&&(getSysCount() >= tt_PowerupWaitTime_u64))
      {
        module_StateMachineControl.errorCode_u16 =0;
        return_state_u8 = PRE_START_MODULE;
        break;
      }
      else if ((module_StateMachineControl.command_Speed == 0)&&(autorestart==FALSE))
      {
        module_StateMachineControl.errorCode_u16 =0;
        autorestart = TRUE;
      }
      StartRetryCounter = 0;                                                    //this variable combine with StartRetryPeriod to form the timeout of start retry
      MotSpinPollCount = 0;                                                     //this value combine the startRetryTime delay to form the timeout 
      return_state_u8 = IDLE_MODULE;
      break;
    }
  case PRE_START_MODULE: 
    {        //calculate the actual execution time according to the current motor speed and the target speed in RPM
      if ( 0 == module_StateMachineControl.command_Speed)
      {
        return_state_u8 = IDLE_MODULE;
        break;
      }
      else
      {
        target_speed = module_StateMachineControl.command_Speed;
        setSpeed(module_StateMachineControl.command_Speed * (int32_t) module_StateMachineControl.motorDir);                                                                 //Command ST motor libraries for running the speed of module_StateMachineControl.targetSpeed
        MC_StartMotor1();
        act_dir = MC_GetImposedDirectionMotor1();
        MotSpinPollCount = 0;                                                                                               //Reset motor spinning loop count as timeout counter
        //tt_SpinPollTime = getSysCount() + SpinPollPeriod; //prepare next time tick value for OTF_STARTUP_MODULE
        tt_SpinPollTime = getSysCount() + applicationSpecificParameters_Control.applicationSpecificParameters_Settings.motorSpinPollPeriod_u16; //prepare next time tick value for OTF_STARTUP_MODULE
        currentDerating_lowThreshold_flag = FALSE;
        powerDerating_lowThreshold_flag = FALSE;
#ifdef eepromInstall             
        /**Writing to EEPROM procedure#3: Assign the fixed register number to the proper location on the state character array (based on specification)  **/
        StateData[12] = (unsigned char) Idfy_logDatAddr_MotorStart_CMDByUser; //Start command to be executed
        /**Writing to EEPROM procedure#4: Write State Character Array to the Ring Buffer. This will then be written to EEPROM by the I2C module **/
//DATA_LOG_REMOVED        RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif
        return_state_u8 = OTF_STARTUP_MODULE;
        break;
      }
    }
  case OTF_STARTUP_MODULE: 
    {      //monitor the startup process and check motor is running successfully
      // startup successfully proof 
      //ST motor state in either START_RUN or RUN, no fault state, measured speed normal, 
      if(MC_GetOccurredFaultsMotor1()){          //if any fault happen
        MC_StopMotor1();                                        //stop the motor first
        MC_AcknowledgeFaultMotor1();                            //clear the fault
        MotSpinPollCount = 0;                                   //this value combine the startRetryTime delay to form the timeout 
        //tt_StartUpRetryTime = getSysCount() + StartRetryPeriod +(StartPeriodInc * StartRetryCounter);
        tt_StartUpRetryTime = getSysCount() + startupParameters_Control.startupParameters_Settings.startRetyPeriod_u16 +(startupParameters_Control.startupParameters_Settings.startRetryIncPeriod_u16 * StartRetryCounter);
        return_state_u8 = MOTOR_START_RETRY_MODULE;
        break;
      }    
      State_t tmpryMC_State = MC_GetSTMStateMotor1();
      if ((getSysCount() >= tt_SpinPollTime)&&(Burnin_M1.en_ctrl==DISABLE))
      {  //wait for motor spin up to speed   
        //if(MotSpinPollCount++ >= MotSpinTimeOut)
        if(MotSpinPollCount++ >= applicationSpecificParameters_Control.applicationSpecificParameters_Settings.motorSpinTimeout_u16)
        {                 //motor spin-up time-out, timeout period = SpinPollPeriod * MotSpinTimeOut
          MC_StopMotor1();                                        //stop the motor first
          MC_AcknowledgeFaultMotor1();                            //clear the fault
          MotSpinPollCount = 0;                                   //this value combine the startRetryTime delay to form the timeout 
#ifdef eepromInstall               
          /* Send a count to EEPROM for data logging*/
          StateData[12] = (unsigned char) Idfy_logDatAddr_SpinMotor2TargetSpeedFail; 
 //DATA_LOG_REMOVED          RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif                
          //tt_StartUpRetryTime = getSysCount() + StartRetryPeriod +(StartPeriodInc * StartRetryCounter);
          tt_StartUpRetryTime = getSysCount() + startupParameters_Control.startupParameters_Settings.startRetyPeriod_u16 +(startupParameters_Control.startupParameters_Settings.startRetryIncPeriod_u16 * StartRetryCounter);
          return_state_u8 = MOTOR_START_RETRY_MODULE;   
          break;
        } 
        //tt_SpinPollTime = getSysCount() + SpinPollPeriod;  //update next time tick value
        tt_SpinPollTime = getSysCount() + applicationSpecificParameters_Control.applicationSpecificParameters_Settings.motorSpinPollPeriod_u16;  //update next time tick value 
      }
      
      if ( 0 == module_StateMachineControl.command_Speed)
      {
        //if (A_REGAL_OTF==1)
        if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable == TRUE)
        {
          BrakeHandle_M1.TransitionPhase = START_TO_STOP;
        }
        return_state_u8 = STOP_MOTOR_MODULE;
        break;
      }
      if ((OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      {                                               
        act_dir = MC_GetImposedDirectionMotor1();
        return_state_u8 = MOTOR_STOPPING_MODULE;
        break;
      }            
      if((tmpryMC_State == START_RUN) || ( tmpryMC_State == RUN)){                                              //ST motor libries in correct state?
        
        if(abs((MC_GetMecSpeedAverageMotor1() * 6 * act_dir) - target_speed) <= ((abs(target_speed) *8)/10)) {                 //check motor actual running at more than +-20% of target speed
          MotSpinPollCount = 0;    
#ifdef eepromInstall
          if (StartRetryCounter>0)
          {
            /* Send a count to EEPROM for data logging*/
            StateData[12] = (unsigned char) Idfy_logDatAddr_MotorStart_RetryNSuccess; 
 //DATA_LOG_REMOVED            RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
          }
          
          if (OTFHandle_M1.hdir==1)
          {
            /* Send a count to EEPROM for data logging*/
            StateData[12] = (unsigned char) Idfy_logDatAddr_OTFCount; 
//DATA_LOG_REMOVED             RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
          }
          
          if (prev_dir != act_dir)
          {
            prev_dir = act_dir;
            StateData[12] = (unsigned char) Idfy_logDatAddr_ReverseSpinCount; //Start command to be executed
//DATA_LOG_REMOVED             RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
          }
#endif
          
          StartRetryCounter = 0;   
          BrakeHandle_M1.TransitionPhase = NO_TRANSITION;
          return_state_u8 = MOTOR_RUNNING_MODULE;
          break;   
        }   
      }
      return_state_u8 = OTF_STARTUP_MODULE;
      break;
    }
    
  case MOTOR_RUNNING_MODULE: 
    {                   //keep tracking over_temperature, over_current, over_power
      deratingCheck();  //update the status of any type of derating is/are above the upper threshold
      return_state_u8 = MOTOR_RUNNING_MODULE;
      if (( DeratingStatus.currentDrateTrg && !DeratingStatus.currentDerateDone )&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      {
        tt_derateCurrentPollTime = getSysCount();                               //prepare next time tick value for CURRENT_DERATING_MODULE
#ifdef eepromInstall               
        /* Send a count to EEPROM for data logging*/
        StateData[12] = (unsigned char) Idfy_logDatAddr_DeratingOverCurrent; 
//DATA_LOG_REMOVED         RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif
        return_state_u8 = CURRENT_DERATING_MODULE;
      }
      if ((DeratingStatus.powerDrateTrg && !DeratingStatus.powerDerateDone) &&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      {
        tt_deratePowerPollTime = getSysCount();                                   //prepare next time tick value for POWER_DERATING_MODULE
#ifdef eepromInstall 
        /* Send a count to EEPROM for data logging*/
        StateData[12] = (unsigned char) Idfy_logDatAddr_DeratingOverPower; 
//DATA_LOG_REMOVED         RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif
        return_state_u8 = POWER_DERATING_MODULE;
      }
      // temperature is the lower priority
      if ((DeratingStatus.temptureDrateTrg && !DeratingStatus.temptureDerateDone)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD)){
        tt_derateTempPollTime = getSysCount();                                     //prepare next time tick value for TEMPATURE_DERATING_MODULE  
#ifdef eepromInstall  
        /* Send a count to EEPROM for data logging*/
        StateData[12] = (unsigned char) Idfy_logDatAddr_DeratingTemperature; 
 //DATA_LOG_REMOVED        RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif
        lastTemperature = NTC_P_GetAvTemp_C(pMCT->pTemperatureSensor); //put current temperature to the last temperature valuable
        TemperatureTimeOut = 2; //counter for wait if no temperature change
        temperatureChanged = 0;
        lastCommandSpeed = module_StateMachineControl.command_Speed;
        tt_derateTempPollTime = getSysCount(); // put zero delay for the first polling time for temperature derating.              
        return_state_u8 = TEMPERATURE_DERATING_TrimDown_MODULE;
      } 
      if(return_state_u8 == MOTOR_RUNNING_MODULE) DeratingStatus.deratingTriggeredFlag &= 0x0f; //clear all derating done status
      
      // May need to check motor running status where  running properly 
      /*        if(abs((MC_GetMecSpeedAverageMotor1() * 6) - target_speed) > ((target_speed *5)/10)) {                 //check motor actual running lower than within +-50% of target speed
      module_StateMachineControl.command_Speed = 0;
      next_State_u8 = STOP_MOTOR_MODULE;                                                                    //assume this is an abnormal operation, only stop motor at this stage.
      break;   
    }   */
      //if can run to this line means no de-rating happen, then will update the new speed 
      if (module_StateMachineControl.motorDir != act_dir)
      {
        module_StateMachineControl.command_Speed = 0;
        return_state_u8 = STOP_MOTOR_MODULE;           
      }
      else
      {
        if(target_speed != module_StateMachineControl.command_Speed)  
        {   
          target_speed = module_StateMachineControl.command_Speed;
          setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);   
          lastDeratingSpeed = 0;                            //once new user speed value executed the last derating speed will reset to zero
        }
      }
      
      if ((OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      {
        module_StateMachineControl.command_Speed = 0;
        return_state_u8 = STOP_MOTOR_MODULE;  
      }
      
      if ( Regal_GetAvrgMotorPowerW(&EEPowerCalcHandle_M1) > motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMinPower_u16 &&
          motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_lossOfInputPhaseEnable ){
            if ( Regal_CheckForPhaseLoss( Get_RDivider_Handle() )) {
              //shut down for high wattage
              STM_FaultProcessing(&STM[M1], MC_INPUT_PHASE_LOSS, 0); 
            }
          }
      break; 
    }
    
    
    //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
  case CURRENT_DERATING_MODULE: 
    { 
      //uint32_t tmpryDrate = -(uint32_t)over_current_rpm_Reduce;
      uint32_t tmpryDrate = -(uint32_t)motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitRpmReduce_u16;
      return_state_u8 = CURRENT_DERATING_MODULE;
      if (target_speed > module_StateMachineControl.command_Speed)
      {//running speed is higher than the user speed check for the direction
        if (module_StateMachineControl.motorDir != act_dir)//collinear
        { //in different direction 
          module_StateMachineControl.command_Speed = 0;                         //stop the motor
          return_state_u8 = STOP_MOTOR_MODULE;
        }   
      }
      // note: only one status will exist for every deratingCheck(), and only CURRENT_DERATING_MODULE (high threshold) will enter this loop
      //if(( avrCurrentRdOP >= over_current_lower_threshold) && (currentDerating_lowThreshold_flag == TRUE))
      if(( avrCurrentRdOP >= motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16) && (currentDerating_lowThreshold_flag == TRUE))
      { //stop the derating process (just only let the current between the upper and lower threshold, otherwise should keep derate until reach in the middle of these two limit) 
        //if( avrCurrentRdOP <= over_current_lower_threshold +((over_current_threshold - over_current_lower_threshold)/2))
        if( avrCurrentRdOP <= motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16 +((motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16 - motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16)/2))
        {
          if(target_speed >= module_StateMachineControl.command_Speed)
          {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
            lastDeratingSpeed = target_speed;
            currentDerating_lowThreshold_flag = FALSE;
            DeratingStatus.currentDerateDone = 1;
            return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
          }else
          {  
            //tmpryDrate = (uint32_t)over_current_rpm_Reduce;               //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
            tmpryDrate = (uint32_t)motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitRpmReduce_u16; //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
          }
        }
        //else if (( avrCurrentRdOP > over_current_lower_threshold +((over_current_threshold - over_current_lower_threshold)/2)) &&(avrCurrentRdOP <= over_current_threshold))
        else if (( avrCurrentRdOP > motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16 +((motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16 - motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16)/2)) &&(avrCurrentRdOP <= motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16))
        {
          if(target_speed >= module_StateMachineControl.command_Speed)
          {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
            lastDeratingSpeed = target_speed;
            currentDerating_lowThreshold_flag = FALSE;
            DeratingStatus.currentDerateDone = 1;
            return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
          }
          else
            tmpryDrate = 0;
        }
        //else if (avrCurrentRdOP > over_current_threshold)
        else if (avrCurrentRdOP > motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16)
          currentDerating_lowThreshold_flag = FALSE;          
      }
      //else if ( avrCurrentRdOP <= over_current_lower_threshold +((over_current_threshold - over_current_lower_threshold)/2))//( avrCurrentRdOP < over_current_lower_threshold)
      else if ( avrCurrentRdOP <= motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16 +((motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16 - motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16)/2))
      {
        //if (deratingCheck() != CURRENT_DERATING_MODULE) tmpryDrate = (uint32_t)over_current_rpm_Reduce;               //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
        currentDerating_lowThreshold_flag = TRUE;
        //tmpryDrate = (uint32_t)over_current_rpm_Reduce;               //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
        tmpryDrate = (uint32_t)motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitRpmReduce_u16; //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
        if(target_speed >= module_StateMachineControl.command_Speed)
        {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
          lastDeratingSpeed = target_speed;
          currentDerating_lowThreshold_flag = FALSE;
          DeratingStatus.currentDerateDone = 1;
          return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
        }
      }
      if (getSysCount() >= tt_derateCurrentPollTime) {                                                    //wait for motor spin down to reach the lower threshold
        target_speed += tmpryDrate;                                                                         //reduce the speed 
        //if (target_speed < MIN_COMMANDABLE_SPEED)
        if (target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16)
        {
          module_StateMachineControl.command_Speed = 0;
          return_state_u8 = STOP_MOTOR_MODULE;// Stop the motor and go back to IDLE
        }            
        setSpeed(target_speed * (int32_t) act_dir);                                                                           //Command ST motor libraries for running the speed of target_speed
        //tt_derateCurrentPollTime = getSysCount() + OvCurrent_derate_period; //prepare next time tick value for CURRENT_DERATING_MODULE 
        tt_derateCurrentPollTime = getSysCount() + motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitPeriod_u16; //prepare next time tick value for CURRENT_DERATING_MODULE 
      }
      break;
    }
    //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
  case POWER_DERATING_MODULE: 
    { 
      //uint32_t tmpryDrate = -(uint32_t)over_power_rpm_Reduce;
      uint32_t tmpryDrate = -(uint32_t)motorLimits01_Control.motorLimits01_Settings.outputPowerLimitRpmReduce_u16;
      return_state_u8 = POWER_DERATING_MODULE;
      if (target_speed > module_StateMachineControl.command_Speed)
      {//running speed is higher than the user speed check for the motor running direction
        if (module_StateMachineControl.motorDir != act_dir)//collinear
        { //in different direction 
          module_StateMachineControl.command_Speed = 0;                         //stop the motor
          return_state_u8 = STOP_MOTOR_MODULE;
        }   
      }
      //if(( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) >= over_power_lower_threshold) && (powerDerating_lowThreshold_flag == TRUE))
      if(( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) >= motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16) && (powerDerating_lowThreshold_flag == TRUE))
      { //stop the derating process (just only let the current between the upper and lower threshold, otherwise should keep derate until reach in the middle of these two limit) 
        //if( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= over_power_lower_threshold +((over_power_threshold - over_power_lower_threshold)/2))
        if( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16 +((module_StateMachineControl.currentPowerLimit_u16 - motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16)/2))
        {
          if(target_speed >= module_StateMachineControl.command_Speed)
          {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
            lastDeratingSpeed = target_speed;
            powerDerating_lowThreshold_flag = FALSE;
            DeratingStatus.powerDerateDone = 1;
            return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
          }
          else
          {
            //tmpryDrate = (uint32_t)over_power_rpm_Reduce; //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
            tmpryDrate = (uint32_t)motorLimits01_Control.motorLimits01_Settings.outputPowerLimitRpmReduce_u16; //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
          }
        }
        //else if (( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) > over_power_lower_threshold +((over_power_threshold - over_power_lower_threshold)/2)) &&(Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= over_power_threshold))
        else if (( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) > motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16 +((module_StateMachineControl.currentPowerLimit_u16 - motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16)/2)) &&(Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= module_StateMachineControl.currentPowerLimit_u16))
        {
          if(target_speed >= module_StateMachineControl.command_Speed)
          {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
            lastDeratingSpeed = target_speed;
            powerDerating_lowThreshold_flag = FALSE;
            DeratingStatus.powerDerateDone = 1;
            return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
          }
          else
            tmpryDrate = 0;
        }
        //else if (Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) > over_power_threshold)
        else if (Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) > module_StateMachineControl.currentPowerLimit_u16)
          powerDerating_lowThreshold_flag = FALSE;          
      }
      //else if ( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= over_power_lower_threshold +((over_power_threshold - over_power_lower_threshold)/2))//( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) < over_power_lower_threshold)
      else if ( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) <= motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16 +((module_StateMachineControl.currentPowerLimit_u16- motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16)/2))
      {
        powerDerating_lowThreshold_flag = TRUE;
        //tmpryDrate = (uint32_t)over_power_rpm_Reduce;  //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
        tmpryDrate = (uint32_t)motorLimits01_Control.motorLimits01_Settings.outputPowerLimitRpmReduce_u16; //increase speed after de_rate condition when current messurement are both lower than the upper and lower threshold
        if(target_speed >= module_StateMachineControl.command_Speed)
        {//make sure derating will finish at between the mid-range of the two thresholds and the lower threshold
          lastDeratingSpeed = target_speed;
          powerDerating_lowThreshold_flag = FALSE;
          DeratingStatus.powerDerateDone = 1;
          return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out 
        }
      }          
      //execute ramp up or down depend on "tmpryDrate" value
      if (getSysCount() >= tt_deratePowerPollTime) {                                                        //wait for motor spin down to reach the lower threshold  
        target_speed += tmpryDrate;                                                                         //reduce the speed 
        //if (target_speed < MIN_COMMANDABLE_SPEED)
        if (target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16)
        {
          module_StateMachineControl.command_Speed = 0;
          return_state_u8 = STOP_MOTOR_MODULE;// Stop the motor and go back to IDLE
        }
        setSpeed(target_speed * (int32_t) act_dir);    
        //tt_deratePowerPollTime = getSysCount() + OvPower_derate_period; //prepare next time tick value for POWER_DERATING_MODULE
        tt_deratePowerPollTime = getSysCount() + motorLimits01_Control.motorLimits01_Settings.outputPowerLimitPeriod_u16; //prepare next time tick value for POWER_DERATING_MODULE
      }
      break;
    }
    //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
  case TEMPERATURE_DERATING_TrimDown_MODULE: 
    {  
      return_state_u8 = TEMPERATURE_DERATING_TrimDown_MODULE;
      if (getSysCount() >= tt_derateTempPollTime)                            //wait for motor spin up/down to reach the lower threshold  
      {
        //tt_derateTempPollTime = getSysCount() +  OvTemp_derate_period;          //prepare next time tick value for TEMPATURE_DERATING_MODULE       
        tt_derateTempPollTime = getSysCount() +  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitPeriod_u16;          //prepare next time tick value for TEMPATURE_DERATING_MODULE       
        CurrentIPM_Temperature = NTC_P_GetAvTemp_C(pMCT->pTemperatureSensor);
        
        //if( CurrentIPM_Temperature <= midTemperatureThreshold)
        if( CurrentIPM_Temperature <= midTemperatureThreshold_u16)
        { //proceed to TEMPERATURE_DERATING_TrimUp_MODULE
          TemperatureTimeOut = temperatureDefaultTimeOutV; //counter for wait if no temperature change
          lastTemperature = CurrentIPM_Temperature;
          temperatureChanged = 0;
          return_state_u8 = TEMPERATURE_DERATING_MaintainTemp_MODULE;                                                                              //TEMPERATURE_DERATING_TrimUp_MODULE;
        }
        else
        {
          if(TemperatureTimeOut == 0)
          {
            if(temperatureChanged)
            { /** pamela Debug test **/
              // target_speed += -(uint32_t)over_temperature_rpm_Reduce;                              
            }
            else
            {
              //target_speed += -(uint32_t)over_temperature_rpm_Reduce *2 ;
              target_speed += -(uint32_t)motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16 *2 ;
            }
            temperatureChanged = 0;
            TemperatureTimeOut = temperatureDefaultTimeOutV; //counter for wait if no temperature change
            lastTemperature = CurrentIPM_Temperature;
            //if(target_speed < MIN_COMMANDABLE_SPEED) target_speed = MIN_COMMANDABLE_SPEED;
            if(target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16)
            {
              target_speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16;
            }
            setSpeed(target_speed * (int32_t) act_dir);                       
          }
          else
          {
            if((CurrentIPM_Temperature < lastTemperature) && (temperatureChanged == 0))
            {//if current temperature is lower and last sample is not just trim down already
              temperatureChanged = 1;                                     //flag for remember this sample going to drim down the temperature
              //target_speed += -(uint32_t)over_temperature_rpm_Reduce;
              target_speed += -(uint32_t)motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16;
              TemperatureTimeOut = temperatureDefaultTimeOutV; //counter for wait if no temperature change
              lastTemperature = CurrentIPM_Temperature;
              //if(target_speed < MIN_COMMANDABLE_SPEED) target_speed = MIN_COMMANDABLE_SPEED;
              if(target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16) 
              {
                target_speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16;
              }
              setSpeed(target_speed * (int32_t) act_dir);    
            }
          }
          TemperatureTimeOut--;
        } 
        if(target_speed > module_StateMachineControl.command_Speed) 
        {//If user input a lower speed than what the currently speed, will execute the new user speed
          target_speed = module_StateMachineControl.command_Speed;
          setSpeed(target_speed * (int32_t) act_dir); 
        }
      } 
      break;        
    }
  case TEMPERATURE_DERATING_MaintainTemp_MODULE:
    {
      return_state_u8 = TEMPERATURE_DERATING_MaintainTemp_MODULE;
      if (getSysCount() >= tt_derateTempPollTime)                            //wait for motor spin up/down to reach the lower threshold  
      {
        //tt_derateTempPollTime = getSysCount() +  OvTemp_derate_period; //prepare next time tick value for TEMPATURE_DERATING_MODULE
        tt_derateTempPollTime = getSysCount() +  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitPeriod_u16; //prepare next time tick value for TEMPATURE_DERATING_MODULE
        CurrentIPM_Temperature = NTC_P_GetAvTemp_C(pMCT->pTemperatureSensor); //get current temperature (for example if [CurrentIPM_Temperature==2500] * 10^-2 == 25 degree)
        //if (( CurrentIPM_Temperature <  over_temperature_lower_threshold) && (target_speed >= module_StateMachineControl.command_Speed))
        if (( CurrentIPM_Temperature <  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16) && (target_speed >= module_StateMachineControl.command_Speed))
        { //exit temperature derating
          /** @where the current speed can maintain within the mid derating temperature **/
          DeratingStatus.temptureDerateDone = 1;
          return_state_u8 = MOTOR_RUNNING_MODULE;   // after speed get back to command then jump out  
        }
        else
        {
          //if(CurrentIPM_Temperature > marginOfMidTemperatureH) 
          if(CurrentIPM_Temperature > marginOfMidTemperatureH_u16) 
          {// need to trim down
            //if(CurrentIPM_Temperature > over_temperature_threshold)
            if(CurrentIPM_Temperature > motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16)
            { //if the current temperature is greater than the upper threshold goto back to heavy trim down again
              //target_speed += -(uint32_t)over_temperature_rpm_Reduce;
              target_speed += -(uint32_t)motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16;
              //if(target_speed < MIN_COMMANDABLE_SPEED) target_speed = MIN_COMMANDABLE_SPEED;
              if(target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16) 
              {
                target_speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16;
              }
              setSpeed(target_speed * (int32_t) act_dir);    
              TemperatureTimeOut = temperatureDefaultTimeOutV; //counter for wait if no temperature change
              lastTemperature = CurrentIPM_Temperature;
              return_state_u8 = TEMPERATURE_DERATING_TrimDown_MODULE;
            }
            else
            {
              //H_deltaTemperature_percent = over_temperature_rpm_Reduce - ((CurrentIPM_Temperature - marginOfMidTemperatureH) * 10)/ (over_temperature_threshold - marginOfMidTemperatureH);
              //H_dividerSpeed = (uint32_t)(over_temperature_rpm_Reduce/H_deltaTemperature_percent);
              //target_speed += -(uint32_t)(over_temperature_rpm_Reduce/H_deltaTemperature_percent);
              //if(target_speed < MIN_COMMANDABLE_SPEED) target_speed = MIN_COMMANDABLE_SPEED;
              
              H_deltaTemperature_percent = motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16 - ((CurrentIPM_Temperature - marginOfMidTemperatureH_u16) * 10)/ (motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16 - marginOfMidTemperatureH_u16);
              H_dividerSpeed = (uint32_t)(motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16/H_deltaTemperature_percent);
              target_speed += -(uint32_t)(motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16/H_deltaTemperature_percent);
              if(target_speed < applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16) 
              {
                target_speed = applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16;
              }
              setSpeed(target_speed * (int32_t) act_dir);    
            }
          }
          else
          {
            //if(CurrentIPM_Temperature <= marginOfMidTemperatureL)
            if(CurrentIPM_Temperature <= marginOfMidTemperatureL_u16) 
            {//need to trim up
              //if (CurrentIPM_Temperature <= over_temperature_lower_threshold)
              if (CurrentIPM_Temperature <= motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16)
              {
                //target_speed += (uint32_t)(over_temperature_rpm_Reduce);
                target_speed += (uint32_t)(motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16);
              }
              else
              {
                //L_deltaTemperature_percent = over_temperature_rpm_Reduce - ((marginOfMidTemperatureL - CurrentIPM_Temperature) * 10)/ (marginOfMidTemperatureL - over_temperature_lower_threshold);
                //if(L_deltaTemperature_percent <= 0)  L_deltaTemperature_percent = 1;
                //L_dividerSpeed = (uint32_t)(over_temperature_rpm_Reduce/L_deltaTemperature_percent);
                //target_speed += (uint32_t)(over_temperature_rpm_Reduce/L_deltaTemperature_percent
                                           
                L_deltaTemperature_percent = motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16 - ((marginOfMidTemperatureL_u16 - CurrentIPM_Temperature) * 10)/ (marginOfMidTemperatureL_u16 - motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16);
                if(L_deltaTemperature_percent <= 0)  L_deltaTemperature_percent = 1;
                L_dividerSpeed = (uint32_t)(motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16/L_deltaTemperature_percent);
                target_speed += (uint32_t)(motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16/L_deltaTemperature_percent);
              }
            }
          }
          //If user input a lower speed than what the currently speed, will execute the new user speed
          if(target_speed > module_StateMachineControl.command_Speed) target_speed = module_StateMachineControl.command_Speed;
          setSpeed(target_speed * (int32_t) act_dir);    
        }
      }          
      break;
    }
    
    //Error retry handling
  case MOTOR_START_RETRY_MODULE: 
    {
      if (getSysCount() >= tt_StartUpRetryTime) {                     //wait for motor spin up to speed
        //if( StartRetryCounter++ < numOfStartRetry -1)
        if( StartRetryCounter++ < startupParameters_Control.startupParameters_Settings.startNumOfRetries_u16 -1)         
        {
          return_state_u8 = PRE_START_MODULE;
          break;
        }  
        else
        {
          StartRetryCounter = 0;
          module_StateMachineControl.command_Speed = 0;
          GMI_FaultStatus = 0x01;                                           //start-up retry error
#ifdef eepromInstall               
          /* Send a count to EEPROM for data logging*/
          StateData[12] = (unsigned char) Idfy_logDatAddr_MotorStart_RetryNFail; 
//DATA_LOG_REMOVED           RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
          
          if (prev_dir != act_dir)
          {
            /* Send a count to EEPROM for data logging*/
            StateData[12] = (unsigned char) Idfy_logDatAddr_ReverseSpinCountNFail; 
//DATA_LOG_REMOVED             RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
          }
#endif              
          return_state_u8 = FAULT_PROCESS_MODULE;
          break;     
        }
      }      
      return_state_u8 = MOTOR_START_RETRY_MODULE;
      break;
    }
    
    //Error report 
  case FAULT_PROCESS_MODULE: 
    {
      //     if( setting status ){                                                                                             //Fault command will be issued according to user setting!!
      //State_Check = MC_GetOccurredFaultsMotor1();
      dynamic_data.driveData_Data.mcFaults01_u16 = MC_GetOccurredFaultsMotor1();
      module_StateMachineControl.errorCode_u16 = MC_GetOccurredFaultsMotor1();   
      RegalSetting_Init();
      if (Burnin_M1.en_ctrl == ENABLE)
        Burnin_InitProfile(&Burnin_M1, &RevUpControlM1);
      OTFHandle_M1.hdir = WM_RESET;
      BrakeHandle_M1.TransitionPhase = NO_TRANSITION;
      BrakeHandle_M1.BrakingPhase = CURRENT_STARTRAMP;
      MC_AcknowledgeFaultMotor1();                                                                                      //CLear ST motor libraries fault status
      //     }
      MotSpinPollCount = 0;                                                                                               //Reset motor spinning loop count as timeout counter
      return_state_u8 = FAULT_REPORT_MODULE;
      break;
    }
    
  case FAULT_REPORT_MODULE: 
    {
      setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xEF, GMI_FaultStatus, 0x00, NULL);  //Note: in FAULT_NOW situation will continue to issue fault to error-module 
      tt_FaultOTFWaitTime_u64 = getSysCount() + FAULT_WAIT_TIME; // An arbitrary wait time for stability after fault
      return_state_u8 = FAULT_WAIT_MODULE;
      break;
    }
    
  case FAULT_WAIT_MODULE: { 
    if(getSysCount() >= tt_FaultOTFWaitTime_u64)
    {
      //if (A_FAULT_AUTOSTART==0) 
      if(motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_faultAutoStartEnable == FALSE)
        autorestart = FALSE;
#ifdef eepromInstall 
      //DATA_LOG_REMOVED I2C_SendFault(module_StateMachineControl.errorCode_u16);                                                     //Send Fault to EEPROM    
#endif
      return_state_u8 = IDLE_MODULE;
      break;
    }
    else
    {
      
    }
    return_state_u8 = FAULT_WAIT_MODULE;
    break;
  }
  
  //Motor stop
  case STOP_MOTOR_MODULE: 
    {
      //check motor current situation to perform motor stop sequency
      MC_StopMotor1();     
#ifdef eepromInstall 
      /* Send a count to EEPROM for data logging*/
      StateData[12] = (unsigned char) Idfy_logDatAddr_ShutdownCount; //Stop command to be executed
//DATA_LOG_REMOVED       RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, StateData, &StateDataLen);
#endif
      tt_StopTime = getSysCount() + default_STOP_TIMEOUT; //set timeout for the whole stopping process
      return_state_u8 = MOTOR_STOPPING_MODULE;
      break;
    }
  case MOTOR_STOPPING_MODULE: 
    {
      //if ((A_CONTROLLED_BRAKING==1)||(OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      if ((brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_brakingControlEnable==TRUE)||
          (OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      {
        if ((BrakeHandle_M1.BrakingPhase == TURNONLOWSIDE)|| ((BrakeHandle_M1.TransitionPhase == START_TO_STOP)&&(BrakeHandle_M1.BrakingPhase != CURRENT_STARTRAMP)))
        {
          return_state_u8 = MOTOR_BRAKING_MODULE;
          break;
        }
        //if (A_REGAL_OTF==1)
        if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable == TRUE)
        {
          if((0 != module_StateMachineControl.command_Speed)&&(module_StateMachineControl.motorDir == act_dir)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))//State-Machine commanded speed is in RPM
          {
            uint16_t StoppingSpeed = abs(MC_GetMecSpeedAverageMotor1() * act_dir);// this is 0_01Hz and based on ST-Micro parameter conversion
            if ((StoppingSpeed >= OTFHandle_M1.MinSyncSpeed)&&(StoppingSpeed <=  OTFHandle_M1.MaxSyncSpeed)) //greater than the Minimum synchronization speed and less than twice the maximum synchronization speed
            {
              BrakeHandle_M1.TransitionPhase = STOP_TO_START;
              return_state_u8 = MOTOR_BRAKING_MODULE;
              break;
            }
          }
          
          if ((0 == module_StateMachineControl.command_Speed)&&((OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD)))
          {
            BrakeHandle_M1.TransitionPhase = START_TO_STOP;
            return_state_u8 = MOTOR_BRAKING_MODULE;
            break;            
          }
        }
      }
      
      //if ((A_CONTROLLED_BRAKING==0)&&(MC_GetSTMStateMotor1() == STOP_IDLE)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      if ((brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_brakingControlEnable==FALSE)&&(MC_GetSTMStateMotor1() == STOP_IDLE)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      {
        //if (A_REGAL_OTF==1)
        if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable == TRUE)
        {
          //tt_StopWaitTime = getSysCount() + STOP_WAIT_TIME;// RPa: small delay when OTF is enabled
          tt_StopWaitTime = getSysCount() + otfParameters_Control.otfParameters_Settings.stopWaitTime_u16;// RPa: small delay when OTF is enabled
        }else
        {
          //tt_StopWaitTime = getSysCount() + default_POWERUP_DURATION;// RPa: make sure to add delay based on motor parameters when braking and OTF are disabled
          tt_StopWaitTime = getSysCount() + startupParameters_Control.startupParameters_Settings.powerUpDelay_u16;// RPa: make sure to add delay based on motor parameters when braking and OTF are disabled
        }
        return_state_u8 = MOTOR_BRAKING_MODULE;
        break;
      }  
      
      if ((getSysCount()>= tt_StopTime)&&(MC_GetSTMStateMotor1() == IDLE))
      {
        return_state_u8 = MOTOR_BRAKING_MODULE;
        break;       
      }
      return_state_u8 = MOTOR_STOPPING_MODULE;
      break;
      
    }
  case MOTOR_BRAKING_MODULE: 
    {
      //if ((A_CONTROLLED_BRAKING==1)||(OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      if ((brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_brakingControlEnable==TRUE)||
          (OTFHandle_M1.hdir==WM_HIGH_REVERSE)||(OTFHandle_M1.hdir == WM_LOW_FORWARD))
      {
        if ((BrakeHandle_M1.BrakingPhase == CURRENT_STARTRAMP)||(BrakeHandle_M1.TransitionPhase == STOP_TO_START))
        {
          RegalSetting_Init();
          if (Burnin_M1.en_ctrl == ENABLE)
          {
            Burnin_InitProfile(&Burnin_M1, &RevUpControlM1);
          }
          return_state_u8 = IDLE_MODULE;
          break;
        }
        else if ((BrakeHandle_M1.TransitionPhase == START_TO_STOP)&&((MC_GetSTMStateMotor1() == STOP_IDLE)||(MC_GetSTMStateMotor1() == IDLE)))
        {
          RegalSetting_Init();
          if (Burnin_M1.en_ctrl == ENABLE)
            Burnin_InitProfile(&Burnin_M1, &RevUpControlM1);
          OTFHandle_M1.hdir = WM_RESET;
          BrakeHandle_M1.TransitionPhase = NO_TRANSITION;
          return_state_u8 = IDLE_MODULE;
          break;          
        }
      }
      //if ((A_CONTROLLED_BRAKING==0)&&(getSysCount()>= tt_StopWaitTime)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      if ((brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_brakingControlEnable==FALSE)&&(getSysCount()>= tt_StopWaitTime)&&(OTFHandle_M1.hdir!=WM_HIGH_REVERSE)&&(OTFHandle_M1.hdir != WM_LOW_FORWARD))
      {
        if (BrakeHandle_M1.TransitionPhase == START_TO_STOP)
        {
          OTFHandle_M1.hdir = WM_RESET;
        }
        RegalSetting_Init();
        if (Burnin_M1.en_ctrl == ENABLE)
          Burnin_InitProfile(&Burnin_M1, &RevUpControlM1);
        return_state_u8 = IDLE_MODULE;
        break;
      }
      
      return_state_u8 = MOTOR_BRAKING_MODULE;
      break;
    }             
    
  case IRQ_MODULE: 
    {
      return_state_u8 = IDLE_MODULE;
      break;
    }
  case STOP_MODULE: 
    {
      return_state_u8 = IDLE_MODULE;
      break;
    }
  default: 
    {
      return_state_u8 = IDLE_MODULE;
      break;
    }
  }
  return return_state_u8;
}

void setSpeed(int32_t target_speed){
  int32_t current_speed = MC_GetMecSpeedAverageMotor1() * 6;   //convert to rpm
  //uint32_t speed_ramp_up_duration = (abs(current_speed - target_speed ) * 1000) / SPEED_UP_RAMP_RATE;
  uint32_t speed_ramp_up_duration = (abs(current_speed - target_speed ) * 1000) / (applicationSpecificParameters_Control.applicationSpecificParameters_Settings.rampUpRate_u16);
  MC_ProgramSpeedRampMotor1( target_speed / 6, speed_ramp_up_duration );
}


void deratingCheck(void){ //check any of the derating is/are above the upper threshold
  DeratingStatus.deratingTriggeredFlag &= 0xf0; //clear all derating threshold flag
  //if( avrCurrentRdOP >= over_current_threshold) DeratingStatus.currentDrateTrg = 1; //check for over current de-rating
  //if( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) >= over_power_threshold) DeratingStatus.powerDrateTrg=1; //check for over power de-rating
  //if( (tmpryTempature = (NTC_GetAvTemp_C(pMCT->pTemperatureSensor) * 10)) >= over_temperature_threshold) DeratingStatus.temptureDrateTrg = 1; //check for over temperature de-rating
  if( avrCurrentRdOP >= motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16) DeratingStatus.currentDrateTrg = 1; //check for over current de-rating
  if( Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 ) >= module_StateMachineControl.currentPowerLimit_u16) DeratingStatus.powerDrateTrg=1; //check for over power de-rating
  if( (tmpryTempature = (NTC_GetAvTemp_C(pMCT->pTemperatureSensor) * 10)) >= motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16) DeratingStatus.temptureDrateTrg = 1; //check for over temperature de-rating
}



/*
RPa: Please see Data Logger EEPROM module managament document specification
FaultData[1];//Data Size
FaultData[2][3] = 0x0010; //Register Update
FaultData[12][13]; //Command
FaultData[14]; //Count
*/
#ifdef eepromInstall  
unsigned char FaultData[] = {0x02, 0x03, 0x10, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x01, 0x03};
unsigned DataLen2 = sizeof(FaultData);
static void I2C_SendFault(uint16_t errorCode)
{  
  //RPa: revisit solution later when we need to optimise code through for-loop, bit-shifting, address-shifting
  if ((errorCode & MC_FOC_DURATION)==MC_FOC_DURATION)// FOC Duration Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibFOCDuration; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_OVER_VOLT)==MC_OVER_VOLT) // Overvoltage Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibOverVoltage; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_UNDER_VOLT)==MC_UNDER_VOLT) // Undervoltage Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibUnderVoltage; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_OVER_TEMP)==MC_OVER_TEMP) // Over Temperature Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibOverHeat; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_START_UP)==MC_START_UP) //Speed Feedback Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibStartUpFailure; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_SPEED_FDBK)==MC_SPEED_FDBK) //Speed Feedback Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibSpeedFeedback; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_BREAK_IN)==MC_BREAK_IN) //Speed Feedback Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibOverCurrent; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
  
  if ((errorCode & MC_SW_ERROR)==MC_SW_ERROR) //Software Error Fault
  {
    FaultData[12] = (unsigned char) Idfy_logDatAddr_STLibSoftwareError; //Fault command to be executed         
    RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, FaultData, &DataLen2);
  }
}
#endif

uint16_t NTC_P_GetAvTemp_C(NTC_Handle_t * pHandle )
{
  int32_t wTemp;
  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    wTemp = (uint16_t)(((((float)(pHandle->hAvTemp_d) - pHandle->wV0)* pHandle->hSensitivity)/65536 + pHandle->hT0) * 10);
  }
  else
  {
    wTemp = pHandle->hExpectedTemp_C;
  }
  return ( wTemp );
}

void InitStateMachineSettings()
{
  midTemperatureThreshold_u16 = (uint16_t)(((motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16 - motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16)/2)+ motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16); //mid point of the upper and lower temperature threshold
  marginOfMidTemperatureH_u16 = ((uint16_t)(midTemperatureThreshold_u16 + (motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16 - motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16) * 0.1)); //Temperature maintain upper region 
  marginOfMidTemperatureL_u16 = ((uint16_t)(midTemperatureThreshold_u16 - (motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16 - motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16) * 0.4)); //Temperature maintain lower region
}
