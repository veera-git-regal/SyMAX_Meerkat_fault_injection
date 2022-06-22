/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */ 
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "hardware_config.h"
#include "zz_module_flash.h"
#include "regal_mc_settings.h"
#include "module_test.h"

//SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
//PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];

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
extern ApplicationSpecificParameters_Control applicationSpecificParameters_Control;

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0     
#include "pqd_motor_power_measurement.h"
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */ 

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1; 

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  * @details comments removed with variable information
  *     .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  *     .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT, 
  *     .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  *     .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  *     .hUpperOutputLimit       = (int16_t)IQMAX, 
  *     .hLowerOutputLimit       = -(int16_t)IQMAX,
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
//  .hKpDivisor          = (uint16_t)SP_KPDIV,
//  .hKiDivisor          = (uint16_t)SP_KIDIV,
//  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
//  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  * @details comments removed with variable information
  *             .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  *             .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  */
PID_Handle_t PIDIqHandle_M1 =
{
//  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
//  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,   
  .hUpperOutputLimit       = INT16_MAX,     
  .hLowerOutputLimit       = -INT16_MAX,           
//  .hKpDivisor          = (uint16_t)TF_KPDIV,       
//  .hKiDivisor          = (uint16_t)TF_KIDIV,       
//  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
//  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,        
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  * @details comments removed with variable information
  *        .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,      
  *        .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT, 
  */
PID_Handle_t PIDIdHandle_M1 =
{
//  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV, 
//  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,                 
  .hLowerOutputLimit       = -INT16_MAX,                
//  .hKpDivisor          = (uint16_t)TF_KPDIV,          
//  .hKiDivisor          = (uint16_t)TF_KIDIV,          
//  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
//  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,       
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  * @details comments removed with variable information
  *         .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT), 
  *         .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT), 
  *         .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT), 
  *         .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  *         .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		 
  *         .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,       
  *         .ModeDefault =					DEFAULT_CONTROL_MODE,       
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE, 	 
  .MecSpeedRefUnitDefault = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault = (int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault = (int16_t)DEFAULT_FLUX_COMPONENT,                                                                     
};

/**
  * @brief  RevUp Controller parameters Motor 1
  * @details comments removed with variable information
  *             .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT, 
  *             .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED_UNIT/2),    
  *               .ParamsData             = {{(uint16_t)A_PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
  *                          {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
  *                          {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
  *                          {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
  *                          {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
  *                          },   
  */
RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,   
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE-1u),   
  .OTFStartupEnabled       = true,  
  .OTFPhaseParams         = {(uint16_t)500,                 
                                         0,                 
                             (int16_t)PHASE5_FINAL_CURRENT,
                             (void*)MC_NULL},
};

/**
  * @brief PWM Controller parameters Motor 1
  * @details comments removed with variable information
  *             .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,   
  *             .PWMperiod          = PWM_PERIOD_CYCLES,
  *             .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u, 
  */
PWMC_R3_1_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetPhaseCurrents              = &R3_1_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3_1_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3_1_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3_1_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &R3_1_TurnOnLowSides,         
    .pFctIsOverCurrentOccurred         = &R3_1_IsOverCurrentOccurred,    
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3_1_RLDetectionModeEnable,    
    .pFctRLDetectionModeDisable        = &R3_1_RLDetectionModeDisable,   
    .pFctRLDetectionModeSetDuty        = &R3_1_RLDetectionModeSetDuty,    
    .Sector = 0,    
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false, 
    .OffCalibrWaitTimeCounter = 0, 
    .Motor = M1,     
    .RLDetectionMode = false, 
    .Ia = 0, 
    .Ib = 0, 
    .Ic = 0, 
    .DTTest = 0,   
    .DTCompCnt = DTCOMPCNT, 
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .Ton                 = TON,                              
    .Toff                = TOFF
  },
  .PhaseAOffset = 0,   
  .PhaseBOffset = 0,   
  .PhaseCOffset = 0,   
  .OverCurrentFlag = false,    
  .OverVoltageFlag = false,    
  .BrakeActionLock = false,                               
  .pParams_str = &R3_1_ParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  * @details comments removed with variable information
  *             .bElToMecRatio                     =	POLE_PAIR_NUM, 
  *             .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
  *             .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  *             .hMeasurementFrequency             =	(uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING)),
  *             .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{
  
  ._Super = {
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,      
    .hMaxReliableMecAccelUnitP         =	65535,                             
    .DPPConvFactor                     =  DPP_CONV_FACTOR,       
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE, 
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  * @details comments removed with variable information
  *             .bElToMecRatio                     =	POLE_PAIR_NUM,
  *             .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
  *             .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  *             .hMeasurementFrequency             =	(uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING)),
  *             .hC2                         =	C2,                             
  *             .hC3                         =	C3,                             
  *             .hC4                         =	C4,                             
  *             .hDefKpGain = PLL_KP_GAIN, 
  *             .hDefKiGain = PLL_KI_GAIN, 
  *             .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,             
  *             .MaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT*1.15), 
  */
STO_PLL_Handle_t STO_PLL_M1 =
{
  ._Super = {
    .SpeedUnit                         = SPEED_UNIT,
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,           
    .hMaxReliableMecAccelUnitP         =	65535,                               
    .DPPConvFactor                     =  DPP_CONV_FACTOR,    
  },
// .hC1                         =	C1,                             
// .hC5                         =	C5,                             
// .hF1                         =	F1,                             
// .hF2                         =	F2,                             
 .PIRegulator = {
	 .hDefKdGain = 0x0000U,     
//     .hKpDivisor = PLL_KPDIV,   
//     .hKiDivisor = PLL_KIDIV,   
	 .hKdDivisor = 0x0000U,			 
     .wUpperIntegralLimit = INT32_MAX, 
     .wLowerIntegralLimit = -INT32_MAX,
     .hUpperOutputLimit = INT16_MAX, 
     .hLowerOutputLimit = -INT16_MAX, 
//     .hKpDivisorPOW2 = PLL_KPDIV_LOG,  
//     .hKiDivisorPOW2 = PLL_KIDIV_LOG, 
     .hKdDivisorPOW2       = 0x0000U, 
   },      			
 .SpeedBufferSizeUnit                =	STO_FIFO_DEPTH_UNIT,           
 .SpeedBufferSizeDpp                 =	STO_FIFO_DEPTH_DPP,            
 .VariancePercentage                 =	PERCENTAGE_FACTOR,             
 .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,        
 .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,        
 .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,  	       
 .Reliability_hysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS, 
 .BemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,          
 .BemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,         
// .F1LOG                              =	F1_LOG,                            
// .F2LOG                              =	F2_LOG,                            
 .SpeedBufferSizeDppLOG              =	STO_FIFO_DEPTH_DPP_LOG,
 .hForcedDirection                   =  0x0000U             
};
STO_PLL_Handle_t *pSTO_PLL_M1 = &STO_PLL_M1; 

STO_Handle_t STO_M1 = 
{
  ._Super                        = (SpeednPosFdbk_Handle_t*)&STO_PLL_M1,
  .pFctForceConvergency1         = &STO_PLL_ForceConvergency1,
  .pFctForceConvergency2         = &STO_PLL_ForceConvergency2,
  .pFctStoOtfResetPLL            = &STO_OTF_ResetPLL,
  .pFctSTO_SpeedReliabilityCheck = &STO_PLL_IsVarianceTight                              
};

ICL_Handle_t ICL_M1 =
{
  .ICLstate			=	ICL_INACTIVE,						
  .hICLTicksCounter	=	0u,    								
  .hICLTotalTicks	=	UINT16_MAX,							
  .hICLFrequencyHz 	=	SPEED_LOOP_FREQUENCY_HZ,			
  .hICLDurationms	=	INRUSH_CURRLIMIT_CHANGE_AFTER_MS,	
};

/**
  *@brief temperature sensor parameters Motor 1
  * @details comments removed with variable information
  *             .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  *             .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC1,
#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_8KW))   
    .channel = MC_ADC_CHANNEL_12,
#elif ((HARDWARE_VERSION == HARDWARE_VERSION_1p3KW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW_EXT_CRYSTAL)|| (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_MV))
    .channel = MC_ADC_CHANNEL_14,   
#endif
    .samplingTime = M1_TEMP_SAMPLING_TIME,   
  },  
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,											 
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * @brief Bus voltage sensor parameters Motor 1
  * @details comments removed with variable information
  *             .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,   
  *             .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,  
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,                 
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),                                                   
  },
  
  .VbusRegConv =
  {
    .regADC = ADC1,
    .channel = MC_ADC_CHANNEL_11,
    .samplingTime = M1_VBUS_SAMPLING_TIME,   
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,  
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

UI_Handle_t UI_Params =
{
  .bDriveNum = 0,
  .pFct_DACInit = &DAC_Init,               
  .pFct_DACExec = &DAC_Exec,
  .pFctDACSetChannelConfig    = &DAC_SetChannelConfig,
  .pFctDACGetChannelConfig    = &DAC_GetChannelConfig,
  .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
  .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,
 
};

DAC_UI_Handle_t DAC_UI_Params = 
{
  .hDAC_CH1_ENABLED = ENABLE,  
  .hDAC_CH2_ENABLED = DISABLE
};

/** 
  * @brief      RAMP for Motor1.
  * @details comments removed with variable information
  *            .FrequencyHz = TF_REGULATION_RATE    
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1;

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,
  .MaxVd          	  = (uint16_t)(MAX_MODULE * 950 / 1000),
  .Circle_limit_table = MMITABLE,        	
  .Start_index        = START_INDEX, 		
};
DOUT_handle_t ICLDOUTParamsM1 =
{
  .OutputState       = INACTIVE,
#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_8KW))
  .hDOutputPort      = M1_ICL_SHUT_OUT_GPIO_Port,
  .hDOutputPin       = M1_ICL_SHUT_OUT_Pin,	
#endif
  .bDOutputPolarity  = DOUT_ACTIVE_LOW		
};

/**
  * @brief  Regal flash setting redirection for all ST motor libraries user updatable parameters
  * @details comments removed with variable information
  *               MAX_APPLICATION_SPEED_RPM parameter dependance => MAX_BEMF_VOLTAGE parameter dependance => C3
  *               MAX_APPLICATION_SPEED_RPM parameter dependance => MAX_APPLICATION_SPEED_UNIT
  *               MIN_APPLICATION_SPEED_RPM parameter dependance => MIN_APPLICATION_SPEED_UNIT
  *               PWM_FREQUENCY parameter dependance => TF_REGULATION_RATE 
  */
void RegalSetting_Init(void){
  
  //uint32_t TF_RegRate = (uint32_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE)); // read pwm freq from flash
  uint32_t TF_RegRate = (uint32_t) ((uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)/(motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16)); // read pwm freq from flash
  //VirtualSpeedSensorM1._Super.bElToMecRatio = A_POLE_PAIR_NUM;
  VirtualSpeedSensorM1._Super.bElToMecRatio = motorParameters_Control.motorParameters_Settings.polePairs_u16;
  
  //STO_PLL_M1._Super.bElToMecRatio =  A_POLE_PAIR_NUM;
  //STO_PLL_M1.MinStartUpValidSpeed =  (uint16_t) ((A_OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/_RPM);
  
  STO_PLL_M1._Super.bElToMecRatio = motorParameters_Control.motorParameters_Settings.polePairs_u16;
  STO_PLL_M1.MinStartUpValidSpeed =  (uint16_t) (( (motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16)*SPEED_UNIT)/_RPM);
  
  //SpeednTorqCtrlM1.MaxPositiveTorque = (int16_t)A_NOMINAL_CURRENT;		 
  //SpeednTorqCtrlM1.MinNegativeTorque =	-(int16_t)A_NOMINAL_CURRENT;   
  //SpeednTorqCtrlM1.ModeDefault = (STC_Modality_t)A_DEFAULT_CONTROL_MODE,

  SpeednTorqCtrlM1.MaxPositiveTorque = motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16;//(int16_t)Regal_ConvertmAToCounts(motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16);		 
  SpeednTorqCtrlM1.MinNegativeTorque =	-motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16;//(int16_t)Regal_ConvertmAToCounts(motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16);   
  SpeednTorqCtrlM1.ModeDefault = (STC_Modality_t)applicationSpecificParameters_Control.applicationSpecificParameters_Settings.controlMode_u16;
  
  //PIDIqHandle_M1.hDefKpGain = (int16_t)A_PID_TORQUE_KP_DEFAULT;
  //PIDIqHandle_M1.hDefKiGain = (int16_t)A_PID_TORQUE_KI_DEFAULT;
  //PIDIqHandle_M1.hKpDivisor = (uint16_t) A_TF_KPDIV;
  //PIDIqHandle_M1.hKiDivisor = (uint16_t) A_TF_KIDIV;
  //PIDIqHandle_M1.hKdDivisor = (uint16_t) A_TF_KDDIV;
  //PIDIqHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(A_TF_KPDIV);
  //PIDIqHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(A_TF_KIDIV);
  //PIDIqHandle_M1.hKdDivisorPOW2 = (uint16_t) LOG2(A_TF_KDDIV);
  
  PIDIqHandle_M1.hDefKpGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.torqueKp_s16;
  PIDIqHandle_M1.hDefKiGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.torqueKi_s16;
  PIDIqHandle_M1.hKpDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKpFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKpDiv_u16;
  PIDIqHandle_M1.hKiDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; // stParameters01_Control.stParameters01_Settings.tfKiDiv_u16;
  PIDIqHandle_M1.hKdDivisor = 0;//(uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKdFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKdDiv_u16;
  PIDIqHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKpFactor_u16);
  PIDIqHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16);
  PIDIqHandle_M1.hKdDivisorPOW2 = 0;//(uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKdFactor_u16); //stParameters01_Control.stParameters01_Settings.tfKdDiv_u16);
  
  PID_HandleInit(&PIDIqHandle_M1);
//  PIDIdHandle_M1.hDefKpGain = (int16_t)A_PID_FLUX_KP_DEFAULT;
//  PIDIdHandle_M1.hDefKiGain = (int16_t)A_PID_FLUX_KI_DEFAULT;
//  PIDIdHandle_M1.hKpDivisor = (uint16_t) A_TF_KPDIV;
//  PIDIdHandle_M1.hKiDivisor = (uint16_t) A_TF_KIDIV;
//  PIDIdHandle_M1.hKdDivisor = (uint16_t) A_TF_KDDIV;
//  PIDIdHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(A_TF_KPDIV);
//  PIDIdHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(A_TF_KIDIV);
//  PIDIdHandle_M1.hKdDivisorPOW2 = (uint16_t) LOG2(A_TF_KDDIV);  
  
  PIDIdHandle_M1.hDefKpGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.fluxKp_s16;
  PIDIdHandle_M1.hDefKiGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.fluxKi_s16;
  PIDIdHandle_M1.hKpDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKpFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKpDiv_u16;
  PIDIdHandle_M1.hKiDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKiDiv_u16;
  PIDIdHandle_M1.hKdDivisor = 0;//(uint16_t) motorTunning01_Control.motorTunning01_Settings.torqueKdFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKdDiv_u16;
  PIDIdHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKpFactor_u16);
  PIDIdHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16);
  PIDIdHandle_M1.hKdDivisorPOW2 = 0;//(uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.torqueKdFactor_u16);

  
  PID_HandleInit(&PIDIdHandle_M1);
//  PIDSpeedHandle_M1.hDefKpGain = (int16_t)A_PID_SPEED_KP_DEFAULT;
//  PIDSpeedHandle_M1.hDefKiGain = (int16_t)A_PID_SPEED_KI_DEFAULT;
//  PIDSpeedHandle_M1.hKpDivisor = (uint16_t) A_SP_KPDIV;
//  PIDSpeedHandle_M1.hKiDivisor = (uint16_t) A_SP_KIDIV;
//  PIDSpeedHandle_M1.hKdDivisor = (uint16_t) A_SP_KDDIV;
//  PIDSpeedHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(A_SP_KPDIV);
//  PIDSpeedHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(A_SP_KIDIV);
//  PIDSpeedHandle_M1.hKdDivisorPOW2 = (uint16_t) LOG2(A_SP_KDDIV); 

  PIDSpeedHandle_M1.hDefKpGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.speedKp_s16;
  PIDSpeedHandle_M1.hDefKiGain = (int16_t)motorTunning01_Control.motorTunning01_Settings.speedKi_s16;
  PIDSpeedHandle_M1.hKpDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.speedKpFactor_u16;
  PIDSpeedHandle_M1.hKiDivisor = (uint16_t) motorTunning01_Control.motorTunning01_Settings.speedKiFactor_u16;
  PIDSpeedHandle_M1.hKdDivisor = 0;//(uint16_t) motorTunning01_Control.motorTunning01_Settings.speedKdFactor_u16;
  PIDSpeedHandle_M1.hKpDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.speedKpFactor_u16);
  PIDSpeedHandle_M1.hKiDivisorPOW2 = (uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.speedKiFactor_u16);
  PIDSpeedHandle_M1.hKdDivisorPOW2 = 0;//(uint16_t) LOG2(motorTunning01_Control.motorTunning01_Settings.speedKdFactor_u16);
  
  PID_HandleInit(&PIDSpeedHandle_M1);
  
//  PIDIqHandle_M1.wUpperIntegralLimit = (int32_t)INT16_MAX * (int32_t) A_TF_KIDIV;
//  PIDIqHandle_M1.wLowerIntegralLimit = (int32_t)-INT16_MAX * (int32_t) A_TF_KIDIV; 
//  PIDIdHandle_M1.wUpperIntegralLimit = (int32_t)INT16_MAX * (int32_t) A_TF_KIDIV;
//  PIDIdHandle_M1.wLowerIntegralLimit = (int32_t)-INT16_MAX * (int32_t) A_TF_KIDIV; 
//  PIDSpeedHandle_M1.wUpperIntegralLimit = (int32_t)A_IQMAX * (int32_t)A_SP_KIDIV;
//  PIDSpeedHandle_M1.wLowerIntegralLimit = -(int32_t)A_IQMAX * (int32_t)A_SP_KIDIV;
//  PIDSpeedHandle_M1.hUpperOutputLimit = (int16_t)A_IQMAX;
//  PIDSpeedHandle_M1.hLowerOutputLimit = -(int16_t)A_IQMAX;
  
  uint16_t current_count_u16 = 0;
  current_count_u16 = motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16;//Regal_ConvertmAToCounts(motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16);
  
  PIDIqHandle_M1.wUpperIntegralLimit = (int32_t)INT16_MAX * (int32_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKiDiv_u16;
  PIDIqHandle_M1.wLowerIntegralLimit = (int32_t)-INT16_MAX * (int32_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKiDiv_u16; 
  PIDIdHandle_M1.wUpperIntegralLimit = (int32_t)INT16_MAX * (int32_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKiDiv_u16;
  PIDIdHandle_M1.wLowerIntegralLimit = (int32_t)-INT16_MAX * (int32_t) motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16; //stParameters01_Control.stParameters01_Settings.tfKiDiv_u16; 
  PIDSpeedHandle_M1.wUpperIntegralLimit = (int32_t)(current_count_u16) * (int32_t)motorTunning01_Control.motorTunning01_Settings.speedKiFactor_u16 ; //rameters01_Control.stParameters01_Settings.spKiDiv_u16;
  PIDSpeedHandle_M1.wLowerIntegralLimit = -(int32_t)(current_count_u16) * (int32_t)motorTunning01_Control.motorTunning01_Settings.speedKiFactor_u16; //stParameters01_Control.stParameters01_Settings.spKiDiv_u16;
  PIDSpeedHandle_M1.hUpperOutputLimit = (int16_t)(current_count_u16);
  PIDSpeedHandle_M1.hLowerOutputLimit = -(int16_t)(current_count_u16);

  //if (A_REGAL_OTF==1)
  if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable == TRUE)
  {
//    RevUpControlM1.ParamsData[0].hDurationms = (uint16_t)A_PHASE1_DURATION;
//    RevUpControlM1.ParamsData[0].hFinalMecSpeedUnit = (int16_t)(A_PHASE1_FINAL_SPEED_UNIT);
//    RevUpControlM1.ParamsData[0].hFinalTorque = (int16_t)A_PHASE1_FINAL_CURRENT;
//    RevUpControlM1.ParamsData[0].pNext = &RevUpControlM1.ParamsData[1];
    
    RevUpControlM1.ParamsData[0].hDurationms = (uint16_t)otfParameters_Control.otfParameters_Settings.detectDuration_u16; //stParameters01_Control.stParameters01_Settings.phase1Duration_u16;
    RevUpControlM1.ParamsData[0].hFinalMecSpeedUnit = (int16_t)((stParameters01_Control.stParameters01_Settings.phase1Speed_u16)*SPEED_UNIT/(float)_RPM); //(A_PHASE1_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[0].hFinalTorque = (int16_t)(stParameters01_Control.stParameters01_Settings.phase1FinalCurrent_u16); //A_PHASE1_FINAL_CURRENT;
    RevUpControlM1.ParamsData[0].pNext = &RevUpControlM1.ParamsData[1];
    
//    RevUpControlM1.ParamsData[1].hDurationms = (uint16_t)A_PHASE2_DURATION;
//    RevUpControlM1.ParamsData[1].hFinalMecSpeedUnit = (int16_t)(A_PHASE2_FINAL_SPEED_UNIT);
//    RevUpControlM1.ParamsData[1].hFinalTorque = (int16_t)A_PHASE2_FINAL_CURRENT;
//    RevUpControlM1.ParamsData[1].pNext = &RevUpControlM1.ParamsData[2];
    
    RevUpControlM1.ParamsData[1].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.startupBrakingDuration_u16; //.stParameters01_Settings.phase2Duration_u16;
    RevUpControlM1.ParamsData[1].hFinalMecSpeedUnit = (int16_t)((stParameters01_Control.stParameters01_Settings.phase2Speed_u16)*SPEED_UNIT/(float)_RPM); //(A_PHASE2_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[1].hFinalTorque = (int16_t)(stParameters01_Control.stParameters01_Settings.phase2FinalCurrent_u16); //A_PHASE2_FINAL_CURRENT;
    RevUpControlM1.ParamsData[1].pNext = &RevUpControlM1.ParamsData[2];
    
//    RevUpControlM1.ParamsData[2].hDurationms = (uint16_t)A_PHASE3_DURATION;
//    RevUpControlM1.ParamsData[2].hFinalMecSpeedUnit = (int16_t)(A_PHASE3_FINAL_SPEED_UNIT);
//    RevUpControlM1.ParamsData[2].hFinalTorque = (int16_t)A_PHASE3_FINAL_CURRENT;
//    RevUpControlM1.ParamsData[2].pNext = &RevUpControlM1.ParamsData[3];
    
    RevUpControlM1.ParamsData[2].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime01_u16; //A_PHASE3_DURATION;
    RevUpControlM1.ParamsData[2].hFinalMecSpeedUnit = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentSpeed01_u16);// * SPEED_UNIT/(float)_RPM); //(A_PHASE3_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[2].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.alignmentCurrent01_u16; //A_PHASE3_FINAL_CURRENT;
    RevUpControlM1.ParamsData[2].pNext = &RevUpControlM1.ParamsData[3];
    
//    RevUpControlM1.ParamsData[3].hDurationms = (uint16_t)A_PHASE4_DURATION;
//    RevUpControlM1.ParamsData[3].hFinalMecSpeedUnit = (int16_t)(A_PHASE4_FINAL_SPEED_UNIT);
//    RevUpControlM1.ParamsData[3].hFinalTorque = (int16_t)A_PHASE4_FINAL_CURRENT;
//    RevUpControlM1.ParamsData[3].pNext = &RevUpControlM1.ParamsData[4];
    
    RevUpControlM1.ParamsData[3].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime02_u16; //A_PHASE4_DURATION;
    RevUpControlM1.ParamsData[3].hFinalMecSpeedUnit = (int16_t)(((startupParameters_Control.startupParameters_Settings.alignmentSpeed02_u16)));//*SPEED_UNIT/(float)_RPM));
    RevUpControlM1.ParamsData[3].hFinalTorque = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16); //A_PHASE4_FINAL_CURRENT
    RevUpControlM1.ParamsData[3].pNext = &RevUpControlM1.ParamsData[4];
    
//    RevUpControlM1.ParamsData[4].hDurationms = (uint16_t)A_PHASE5_DURATION;
//    RevUpControlM1.ParamsData[4].hFinalMecSpeedUnit = (int16_t)(A_PHASE5_FINAL_SPEED_UNIT);
//    RevUpControlM1.ParamsData[4].hFinalTorque = (int16_t)A_PHASE5_FINAL_CURRENT;
//    RevUpControlM1.ParamsData[4].pNext = (void*)MC_NULL;
    
    RevUpControlM1.ParamsData[4].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.startRampTime_u16; //A_PHASE5_DURATION;
    RevUpControlM1.ParamsData[4].hFinalMecSpeedUnit = (int16_t)((startupParameters_Control.startupParameters_Settings.startRampToSpeed_u16));//*(SPEED_UNIT/(float)_RPM)); //A_PHASE5_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[4].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.startRampCurrent_u16; //A_PHASE5_FINAL_CURRENT;
    RevUpControlM1.ParamsData[4].pNext = (void*)MC_NULL;
  } 
  else
  {
    RevUpControlM1.ParamsData[0].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime01_u16; //A_PHASE3_DURATION;
    RevUpControlM1.ParamsData[0].hFinalMecSpeedUnit = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentSpeed01_u16);// * SPEED_UNIT/(float)_RPM); //A_PHASE3_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[0].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.alignmentCurrent01_u16; //A_PHASE3_FINAL_CURRENT;
    RevUpControlM1.ParamsData[0].pNext = &RevUpControlM1.ParamsData[1];
    
    RevUpControlM1.ParamsData[1].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime02_u16; //A_PHASE4_DURATION;
    RevUpControlM1.ParamsData[1].hFinalMecSpeedUnit = (int16_t)(((startupParameters_Control.startupParameters_Settings.alignmentSpeed02_u16)));//*SPEED_UNIT/(float)_RPM)); //(A_PHASE4_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[1].hFinalTorque = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16); //A_PHASE4_FINAL_CURRENT;
    RevUpControlM1.ParamsData[1].pNext = &RevUpControlM1.ParamsData[2];
    
    RevUpControlM1.ParamsData[2].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.startRampTime_u16; //A_PHASE5_DURATION;;
    RevUpControlM1.ParamsData[2].hFinalMecSpeedUnit = (int16_t)((startupParameters_Control.startupParameters_Settings.startRampToSpeed_u16));//*(SPEED_UNIT/(float)_RPM)); //(A_PHASE5_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[2].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.startRampCurrent_u16; //A_PHASE5_FINAL_CURRENT;
    RevUpControlM1.ParamsData[2].pNext = &RevUpControlM1.ParamsData[3];
    
    RevUpControlM1.ParamsData[3].hDurationms = 0;
    RevUpControlM1.ParamsData[3].hFinalMecSpeedUnit = (int16_t)(((startupParameters_Control.startupParameters_Settings.alignmentSpeed02_u16)));//*SPEED_UNIT/(float)_RPM)); //(A_PHASE4_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[3].hFinalTorque = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16); //A_PHASE4_FINAL_CURRENT;
    RevUpControlM1.ParamsData[3].pNext = &RevUpControlM1.ParamsData[4];
    
    RevUpControlM1.ParamsData[4].hDurationms = 0;
    RevUpControlM1.ParamsData[4].hFinalMecSpeedUnit = (int16_t)((startupParameters_Control.startupParameters_Settings.startRampToSpeed_u16));//*(SPEED_UNIT/(float)_RPM)); //(A_PHASE5_FINAL_SPEED_UNIT);
    RevUpControlM1.ParamsData[4].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.startRampCurrent_u16; //A_PHASE5_FINAL_CURRENT;
    RevUpControlM1.ParamsData[4].pNext = (void*)MC_NULL;
  }
  //RevUpControlM1.hMinStartUpValidSpeed = (uint16_t) ((A_OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/_RPM);
  //RevUpControlM1.hMinStartUpFlySpeed = A_OTF_MIN_SYNC_SPEED;//(int16_t)(((uint16_t) ((A_OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/_RPM))/2);
 
  RevUpControlM1.hMinStartUpValidSpeed = (uint16_t) ((motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16*SPEED_UNIT)/_RPM);
  RevUpControlM1.hMinStartUpFlySpeed = otfParameters_Control.otfParameters_Settings.minSyncSpeed_u16;
  
  //STO_PLL_M1.hC1 = (int32_t)((((int32_t) A_F1* (int32_t) A_RS) * (A_LS_FACTOR/A_RS_FACTOR))/((int32_t) A_LS * TF_RegRate));
  //STO_PLL_M1.hC3 = (int32_t)((((int16_t)A_F1)* (uint16_t)((A_MAX_APPLICATION_SPEED_RPM * 1.2 * (A_MOTOR_VOLTAGE_CONSTANT/A_BEMF_CONSTANT_FACTOR)*SQRT_2)/(1000u*SQRT_3)))/(((int32_t)A_LS * MAX_CURRENT * TF_RegRate)/A_LS_FACTOR));    
  //STO_PLL_M1.hC5 = (int32_t)((((int16_t)A_F1)*MAX_VOLTAGE)/(((int32_t)A_LS * MAX_CURRENT * TF_RegRate) /A_LS_FACTOR));
  
  STO_PLL_M1.hC1 = (int32_t)((((int32_t) (motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16)* (int32_t) (motorParameters_Control.motorParameters_Settings.statorPhaseResistance_u16)) * 
                              (motorParameters_Control.motorParameters_Settings.inductanceFactor_u16/motorParameters_Control.motorParameters_Settings.resistanceFactor_u16))/((int32_t) (motorParameters_Control.motorParameters_Settings.phaseInductanceQ_u16) * TF_RegRate));
  STO_PLL_M1.hC3 = (int32_t)((((int16_t)(motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16))* (uint16_t)((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16  
                                                           * 1.2 * (motorParameters_Control.motorParameters_Settings.bemfConstant_u16/motorParameters_Control.motorParameters_Settings.bemfFactor_u16)*SQRT_2)/
                                                          (1000u*SQRT_3)))/(((int32_t)(motorParameters_Control.motorParameters_Settings.phaseInductanceQ_u16) * 
                                                          MAX_CURRENT * TF_RegRate)/motorParameters_Control.motorParameters_Settings.inductanceFactor_u16));    
  STO_PLL_M1.hC5 = (int32_t)((((int16_t)(motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16))*MAX_VOLTAGE)/(((int32_t)(motorParameters_Control.motorParameters_Settings.phaseInductanceQ_u16) * MAX_CURRENT * TF_RegRate) /motorParameters_Control.motorParameters_Settings.inductanceFactor_u16));
  
  //SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit =	(int16_t)((A_MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM);
  //SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit =	(int16_t)(-((A_MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));
  //VirtualSpeedSensorM1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*((A_MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));
  //STO_PLL_M1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*((A_MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));
  //STO_PLL_M1.MaxAppPositiveMecSpeedUnit = (uint16_t)(((A_MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)*1.15);
  
  SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit =	(int16_t)((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16*SPEED_UNIT)/_RPM);
  SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit =	(int16_t)(-((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16*SPEED_UNIT)/_RPM));
  VirtualSpeedSensorM1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16*SPEED_UNIT)/_RPM));
  STO_PLL_M1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16*SPEED_UNIT)/_RPM));
  STO_PLL_M1.MaxAppPositiveMecSpeedUnit = (uint16_t)(((motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16*SPEED_UNIT)/_RPM)*1.15);
    
  //SpeednTorqCtrlM1.MinAppPositiveMecSpeedUnit =	(uint16_t)((A_MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM); 
  //SpeednTorqCtrlM1.MaxAppNegativeMecSpeedUnit =	(int16_t)(-((A_MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));
  //VirtualSpeedSensorM1._Super.hMinReliableMecSpeedUnit = (uint16_t)(((A_MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));
  //STO_PLL_M1._Super.hMinReliableMecSpeedUnit = (uint16_t)(((A_MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM));

  SpeednTorqCtrlM1.MinAppPositiveMecSpeedUnit =	(uint16_t)((motorParameters_Control.motorParameters_Settings.motorMinRpm_u16*SPEED_UNIT)/_RPM); 
  SpeednTorqCtrlM1.MaxAppNegativeMecSpeedUnit =	(int16_t)(-((motorParameters_Control.motorParameters_Settings.motorMinRpm_u16*SPEED_UNIT)/_RPM));
  VirtualSpeedSensorM1._Super.hMinReliableMecSpeedUnit = (uint16_t)(((motorParameters_Control.motorParameters_Settings.motorMinRpm_u16*SPEED_UNIT)/_RPM));
  STO_PLL_M1._Super.hMinReliableMecSpeedUnit = (uint16_t)(((motorParameters_Control.motorParameters_Settings.motorMinRpm_u16*SPEED_UNIT)/_RPM));
  
//  RampExtMngrHFParamsM1.FrequencyHz = (uint32_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE));  // TF_REGULATION_RATE  
//  if (A_REGAL_OTF==1) 
//    VirtualSpeedSensorM1.hTransitionSteps = 0;
//  else
//    VirtualSpeedSensorM1.hTransitionSteps = (int16_t)((uint32_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE)) * A_TRANSITION_DURATION/ 1000.0); // TF_REGULATION_RATE
//  VirtualSpeedSensorM1._Super.hMeasurementFrequency  =	(uint16_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING)); // TF_REGULATION_RATE
//  STO_PLL_M1._Super.hMeasurementFrequency            =	(uint16_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING)); // TF_REGULATION_RATE
//  PWM_Handle_M1._Super.hT_Sqrt3 = ((uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(A_PWM_FREQUENCY)))*SQRT3FACTOR)/16384u; 
//  PWM_Handle_M1._Super.PWMperiod = (uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(A_PWM_FREQUENCY)));
//  PWM_Handle_M1.Half_PWMPeriod = (uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(A_PWM_FREQUENCY)))/2u;
  
  RampExtMngrHFParamsM1.FrequencyHz = (uint32_t) ((uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)/(motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16));  // TF_REGULATION_RATE  
  if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable  == TRUE) 
    VirtualSpeedSensorM1.hTransitionSteps = 0;
  else
    VirtualSpeedSensorM1.hTransitionSteps = (int16_t)((uint32_t) ((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)/(motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16)) * (startupParameters_Control.startupParameters_Settings.transitionTime_u16)/ 1000.0); // TF_REGULATION_RATE
  VirtualSpeedSensorM1._Super.hMeasurementFrequency  =	(uint16_t) ((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)/( (motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16)* (stParameters01_Control.stParameters01_Settings.pwmFreqScaling_u16) )); // TF_REGULATION_RATE
  STO_PLL_M1._Super.hMeasurementFrequency            =	(uint16_t) ((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)/( (motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16) *(stParameters01_Control.stParameters01_Settings.pwmFreqScaling_u16) )); // TF_REGULATION_RATE
  PWM_Handle_M1._Super.hT_Sqrt3 = ((uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)))*SQRT3FACTOR)/16384u; 
  PWM_Handle_M1._Super.PWMperiod = (uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)));
  PWM_Handle_M1.Half_PWMPeriod = (uint16_t)(ADV_TIM_CLK_MHz* (uint32_t)1000000u/((uint32_t)(uint32_t)(motorParameters_Control.motorParameters_Settings.pwmFrequency_u16)))/2u;
  
  //RealBusVoltageSensorParamsM1.OverVoltageThreshold  = (uint16_t)(A_OV_VOLTAGE_THRESHOLD_V*65535/(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR));
  //RealBusVoltageSensorParamsM1.UnderVoltageThreshold = (uint16_t)((A_UD_VOLTAGE_THRESHOLD_V*65535)/((uint16_t)(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR)));
  uint16_t voltage_threhold_u16 = 0;
  voltage_threhold_u16 = (uint16_t) ((uint32_t)NOMINAL_BUS_VOLTAGE_V * (uint32_t)motorProtections01_Control.motorProtections01_Settings.overVoltageThreshold_u16 * ONE_HUNDEREDTH_FACTOR);
  RealBusVoltageSensorParamsM1.OverVoltageThreshold  = (uint16_t)( (voltage_threhold_u16) *65535/(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR));
  
  voltage_threhold_u16 = (uint16_t) ((uint32_t)NOMINAL_BUS_VOLTAGE_V * (uint32_t)motorProtections01_Control.motorProtections01_Settings.underVoltageThreshold_u16 * ONE_HUNDEREDTH_FACTOR);
  RealBusVoltageSensorParamsM1.UnderVoltageThreshold = (uint16_t)(((voltage_threhold_u16)*65535)/((uint16_t)(ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR)));
  
  //TempSensorParamsM1.hOverTempThreshold      = (uint16_t)(((V0_V + (dV_dT * (A_OV_TEMPERATURE_THRESHOLD_C- T0_C)))*INT_SUPPLY_VOLTAGE));
  //TempSensorParamsM1.hOverTempDeactThreshold = (uint16_t)(((V0_V + (dV_dT * (A_OV_TEMPERATURE_THRESHOLD_C- T0_C)))*INT_SUPPLY_VOLTAGE) - ((dV_dT * OV_TEMPERATURE_HYSTERESIS_C)*INT_SUPPLY_VOLTAGE));
  
  TempSensorParamsM1.hOverTempThreshold      = (uint16_t)(((V0_V + (dV_dT * ((uint16_t)(motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureThreshold_u16 * ONE_TENTH_FACTOR)- T0_C)))*INT_SUPPLY_VOLTAGE));
  TempSensorParamsM1.hOverTempDeactThreshold = (uint16_t)(((V0_V + (dV_dT * ((uint16_t)(motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureThreshold_u16 * ONE_TENTH_FACTOR)- T0_C)))*INT_SUPPLY_VOLTAGE) - ((dV_dT * (uint16_t)(motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureHysteresis_u16 * ONE_TENTH_FACTOR))*INT_SUPPLY_VOLTAGE));
  
  
#ifdef GAIN1 //Config as PLL observer STO_PLL_M1 //
//  STO_PLL_M1.PIRegulator.hDefKpGain = A_PLL_KP_GAIN;
//  STO_PLL_M1.PIRegulator.hDefKiGain = A_PLL_KI_GAIN;
//  STO_PLL_M1.PIRegulator.hKpDivisor = A_PLL_KPDIV;
//  STO_PLL_M1.PIRegulator.hKiDivisor = A_PLL_KIDIV;
//  STO_PLL_M1.PIRegulator.hKpDivisorPOW2 = LOG2(A_PLL_KPDIV);
//  STO_PLL_M1.PIRegulator.hKiDivisorPOW2 = LOG2(A_PLL_KIDIV);
//  STO_PLL_M1.hC2 = (int32_t) A_GAIN1;    
//  STO_PLL_M1.hC4 = (int32_t) A_GAIN2; 
//  STO_PLL_M1.hF1 = A_F1;
//  STO_PLL_M1.hF2 = A_F2;
//  STO_PLL_M1.F1LOG = LOG2(A_F1);
//  STO_PLL_M1.F2LOG = LOG2(A_F2);
  
  STO_PLL_M1.PIRegulator.hDefKpGain = motorTunning02_Control.motorTunning02_Settings.angleEst01Gain_u16;
  STO_PLL_M1.PIRegulator.hDefKiGain = motorTunning02_Control.motorTunning02_Settings.angleEst02Gain_u16;
  STO_PLL_M1.PIRegulator.hKpDivisor = motorTunning02_Control.motorTunning02_Settings.angleEst01GainFactor_u16;
  STO_PLL_M1.PIRegulator.hKiDivisor = motorTunning02_Control.motorTunning02_Settings.angleEst02GainFactor_u16;
  STO_PLL_M1.PIRegulator.hKpDivisorPOW2 = LOG2(motorTunning02_Control.motorTunning02_Settings.angleEst01GainFactor_u16);
  STO_PLL_M1.PIRegulator.hKiDivisorPOW2 = LOG2(motorTunning02_Control.motorTunning02_Settings.angleEst02GainFactor_u16);
  STO_PLL_M1.hC2 = (int32_t) (motorTunning02_Control.motorTunning02_Settings.bemfObservedGain01_s16);    
  STO_PLL_M1.hC4 = (int32_t) (motorTunning02_Control.motorTunning02_Settings.bemfObservedGain02_s16); 
  STO_PLL_M1.hF1 = motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16;
  STO_PLL_M1.hF2 = motorTunning02_Control.motorTunning02_Settings.bemfObserverGain02Factor_u16;
  STO_PLL_M1.F1LOG = LOG2(motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16); // 
  STO_PLL_M1.F2LOG = LOG2(motorTunning02_Control.motorTunning02_Settings.bemfObserverGain02Factor_u16);  
  
  STO_PLL_M1.PIRegulator.hKpGain = STO_PLL_M1.PIRegulator.hDefKpGain;
  STO_PLL_M1.PIRegulator.hKiGain = STO_PLL_M1.PIRegulator.hDefKiGain;
  

  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
  //pPIDSpeed[M1] = &PIDSpeedHandle_M1;
  
  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  //pSTC[M1] = &SpeednTorqCtrlM1;
  STO_PLL_Init (&STO_PLL_M1);


  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  //STC_Init(pSTC[M1],pPIDSpeed[M1], &STO_PLL_M1._Super);
  STC_Init(&SpeednTorqCtrlM1,&PIDSpeedHandle_M1, &STO_PLL_M1._Super);
  
#endif
  
#ifdef CORD_GAIN1 //config as CORDIC observer STO_CR_M1 //
  //STO_CR_M1.hC2 =(int32_t) D_CORD_GAIN1;
  //STO_CR_M1.hC4 =(int32_t) D_CORD_GAIN2;
  
  STO_CR_M1.hC2 =(int32_t) (stParameters01_Control.stParameters01_Settings.dCordGain1_s32);
  STO_CR_M1.hC4 =(int32_t) (stParameters01_Control.stParameters01_Settings.dCordGain2_s32);
#endif 

  

}
		

/* USER CODE BEGIN Additional configuration */
RDivider_Handle_t * Get_RDivider_Handle( void ){
  return &RealBusVoltageSensorParamsM1;
}
/* USER CODE END Additional configuration */ 

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

