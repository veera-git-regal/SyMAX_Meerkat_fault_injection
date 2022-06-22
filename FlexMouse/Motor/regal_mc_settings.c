/**
********************************************************************************************************************************
* @file   regal_mc_settings.c 
* @author  Satya Akkina
* @brief   Main driver module for flash.
* @details This module initializes the flash
*          The ST motor libraries parameters will be mapped from the top of FLASH_USER_START_ADDR in 16bit data,
*          the data can be updated by using the function of uint8_t FlashDatSet(._offset, ._flashDat), then 
*          data and offset will store in internal buffer as temporary data, user can either store all the temporary data into flash 
*          by flashPageUpdate(), or if the internal buffer is full will also update the temporary into flash.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "regal_mc_settings.h"
#include "zz_module_flash.h"
#include "module_test.h"

#define RESISTANCE_FACTOR 1000   // covert ohms to mOhms
#define INDUCTANCE_FACTOR 10000 // convert H to mH*100
#define KE_FACTOR 10

extern ProcessInfo processInfoTable[];


MotorId_Control motorId_Control; // = {{12,13,450,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},{1,2,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}};
ApplicationSpecificParameters_Control applicationSpecificParameters_Control;
HardwareSpecificParameters_Control hardwareSpecificParameters_Control;
//WindMillingParameters_Control windMillingParameters_Control;
BrakingParameters_Control brakingParameters_Control;
OtfParameters_Control otfParameters_Control;
//FanParameters_Control fanParameters_Control;
MotorProtections01_Control motorProtections01_Control;
MotorProtections02_Control motorProtections02_Control;
MotorLimits01_Control motorLimits01_Control;
HarmonicCompensation01_Control harmonicCompensation01_Control;
HarmonicCompensation02_Control harmonicCompensation02_Control;
HarmonicCompensation03_Control harmonicCompensation03_Control;
MotorTunning01_Control motorTunning01_Control;
MotorTunning02_Control motorTunning02_Control;
MotorParameters_Control motorParameters_Control;
StartupParameters_Control startupParameters_Control;
extern ModuleTest_Control moduleTest_Control;
StParameters01_Control stParameters01_Control;

void Init_Motor_Id_Settings(void);
void Init_App_Specific_Settings(void);
void Init_Hw_Specific_Settings(void);
//void Init_Windmill_Settings(void);
void Init_Braking_Settings(void);
void Init_Otf_Settings(void);
void Init_Fan_Settings(void);
void Init_Protection01_Settings(void);
void Init_Protection02_Settings(void);
void Init_Limits_Settings(void);
void Init_HC01_Settings(void);
void Init_HC02_Settings(void);
void Init_HC03_Settings(void);
void Init_Tuning01_Settings(void);
void Init_Tuning02_Settings(void);
void Init_Motor_Settings(void);
void Init_Startup_Settings(void);
void Init_Test_Settings(void);
void Init_St_Mc_Param(void);

void Init_Motor_Id_Data(void);

ModuleFlash_Control* flash_RegalMcSettingsControl;
void init_flash_parmeters()
{
  //motorId_Control.motorId_Settings.hardwareVariant_u16 = 12;
  // motorId_Control.motorId_Settings = {{12,13,450}};
  uint8_t module_flash_Index = getProcessInfoIndex(MODULE_FLASH);
  flash_RegalMcSettingsControl = (ModuleFlash_Control*)((*(processInfoTable[module_flash_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
  if( ((*flash_RegalMcSettingsControl).moduleFlash_Data.userDiscretes01_u16.is_copyUserFlash2RamSuccess == FALSE) &&
      ((*flash_RegalMcSettingsControl).moduleFlash_Data.userDiscretes01_u16.is_copyDefaultFlash2RamSuccess == FALSE) )
  { // If setting from FLASH are not copied to RAM, use the below default value
    Init_Default_Settings();    
  }
  //Init_Motor_Id_Data();
}

void Init_Default_Settings(void)
{
  Init_Motor_Id_Settings();
  Init_App_Specific_Settings();
  Init_Hw_Specific_Settings();
  //Init_Windmill_Settings();
  Init_Braking_Settings();
  Init_Otf_Settings();
  Init_Fan_Settings();
  Init_Protection01_Settings();
  Init_Protection02_Settings();
  Init_Limits_Settings();
  Init_HC01_Settings();
  Init_HC02_Settings();
  Init_HC03_Settings();
  Init_Tuning01_Settings();
  Init_Tuning02_Settings();
  Init_Motor_Settings();
  Init_Startup_Settings();
  Init_Test_Settings();
  Init_St_Mc_Param();
}

void Init_Motor_Id_Settings(void)
{
  //motorId_Control.motorId_Settings.hardwareVariant_u16 = 13; // Hardware Version/Variant
  //motorId_Control.motorId_Settings.hardwareVersion_u16 = 11; // Hardware Version/Variant
  motorId_Control.motorId_Settings.setLoadVariant_u16 = 315;     // Fan/pump or any load coupled Version/Variant

}

void Init_Motor_Id_Data()
{
  uint16_t * pCRC_u16 = (uint16_t*)0x0800FFFC;
  motorId_Control.motorId_Data.minorDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MAJOR;  // in ASCII YY (VV.XX.YY)
  motorId_Control.motorId_Data.medianDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MEDIAN; // In ASCII XX (VV.XX.YY)
  motorId_Control.motorId_Data.majorDriveFirmwareRev_u16 = DRIVE_FW_VERSION_MINOR;  // In ASCII VV (VV.XX.YY)
  motorId_Control.motorId_Data.minorDriveFirmwareCrc_u16 = *pCRC_u16++;     // Motor code CRC
  motorId_Control.motorId_Data.majorDriveFirmwareCrc_u16 = *pCRC_u16;       // Motor code CRC
  motorId_Control.motorId_Data.minorDriveFlashRev_u16 = *((uint16_t *)FLASH_SETTINGS_VERSION_ADDR); // In ASCII, but each nible only 0-9 values are allowed
  motorId_Control.motorId_Data.majorDriveFlashRev_u16 = *((uint16_t *)(FLASH_SETTINGS_VERSION_ADDR+2)); // In ASCII, but each nible only 0-9 values are allowed 00VV, XXYY
  motorId_Control.motorId_Data.minorDriveFlashCrc_u16 = *((uint16_t *)FLASH_SETTINGS_CRC_ADDR);     // Motor flash settings CRC
  motorId_Control.motorId_Data.majorDriveFlashCrc_u16 = *((uint16_t *)(FLASH_SETTINGS_CRC_ADDR+2)); // Motor flash settings CRC
  motorId_Control.motorId_Settings.setLoadVariant_u16 = 315;  // 315 Fan
  motorId_Control.motorId_Settings.setProductType_u16 = 1322; // 1.3kW 230V, MS
}

void Init_Startup_Settings(void)
{
  startupParameters_Control.startupParameters_Settings.detectionDuration_u16 = PHASE1_DURATION;  // Detection duration in mSec // PHASE1_DURATION
  startupParameters_Control.startupParameters_Settings.startupBrakingDuration_u16 = PHASE2_DURATION; // Braking duration while start-up. In mSec // PHASE2_DURATION
  startupParameters_Control.startupParameters_Settings.alignmentTime01_u16 = PHASE3_DURATION;    // Alignment Time in mSec // PHASE3_DURATION
  startupParameters_Control.startupParameters_Settings.alignmentSpeed01_u16 = 0;
  startupParameters_Control.startupParameters_Settings.alignmentCurrent01_u16 = 15638;//0;  // Alignment Current in mAmps
  startupParameters_Control.startupParameters_Settings.alignmentTime02_u16 = PHASE4_DURATION;     // PHASE4_DURATION
  startupParameters_Control.startupParameters_Settings.alignmentSpeed02_u16 = PHASE4_FINAL_SPEED_UNIT;
  startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16 = PHASE4_FINAL_CURRENT;  
  startupParameters_Control.startupParameters_Settings.startRampTime_u16 = PHASE5_DURATION;//8000;    // Start-up Ramp Time in mSec // PHASE5_DURATION
  startupParameters_Control.startupParameters_Settings.startRampToSpeed_u16 = PHASE5_FINAL_SPEED_UNIT; // Start-up Ramp to Speed in RPM
  startupParameters_Control.startupParameters_Settings.startRampCurrent_u16 = PHASE5_FINAL_CURRENT; // Start Ramp Current in mAmps
  startupParameters_Control.startupParameters_Settings.transitionTime_u16 = TRANSITION_DURATION;   // Transition time (open to closed loop) in mSec
  startupParameters_Control.startupParameters_Settings.nbConsecutiveTest_u16 = NB_CONSECUTIVE_TESTS; // Number of consicutive tests before speed feedback fault
  startupParameters_Control.startupParameters_Settings.powerUpDelay_u16 = 3000;
  startupParameters_Control.startupParameters_Settings.startRetyPeriod_u16 = 2000;
  startupParameters_Control.startupParameters_Settings.startRetryIncPeriod_u16 = 10000;
  startupParameters_Control.startupParameters_Settings.startNumOfRetries_u16 = 6;
}

void Init_Motor_Settings(void)
{
  
  motorParameters_Control.motorParameters_Settings.polePairs_u16 = POLE_PAIR_NUM;             // Pole Pairs
  motorParameters_Control.motorParameters_Settings.statorPhaseResistance_u16 = (RS * 1000); // Stator Resistance in mOhms(Rs)
  motorParameters_Control.motorParameters_Settings.resistanceFactor_u16 = (uint16_t)(1000);
  motorParameters_Control.motorParameters_Settings.phaseInductanceD_u16 = (uint16_t)(LS * 10000) ; // Ld in micro H
  motorParameters_Control.motorParameters_Settings.phaseInductanceQ_u16 = (uint16_t)(LS * 10000) ; // Lq in micro H
  motorParameters_Control.motorParameters_Settings.inductanceFactor_u16 = (uint16_t)(10000);
  motorParameters_Control.motorParameters_Settings.bemfConstant_u16 = (MOTOR_VOLTAGE_CONSTANT * 10);    // BEMF @ 1000RPM (Ke Voltage Constant) * 10
  motorParameters_Control.motorParameters_Settings.bemfFactor_u16 = (uint16_t)(10);
  motorParameters_Control.motorParameters_Settings.motorInertia_u16 = (default_MOTOR_INERTIA * 10 );     // Motor Inertia
  motorParameters_Control.motorParameters_Settings.motorInertiaFactor_u16 = 10;
  motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16 = IQMAX;  // Max Motor Current in mAmps
  motorParameters_Control.motorParameters_Settings.motorViscousDampingFactor_u16 = (uint16_t)(default_MOTOR_VISCOUS_DAMPING); // Motor Viscous Damping Factor
  motorParameters_Control.motorParameters_Settings.torqueConstant_u16 =(uint16_t) (MOTOR_VOLTAGE_CONSTANT/(60.459979* 1000)); // Torque Constant (NM/A) Ke/60.459979* 1000
  motorParameters_Control.motorParameters_Settings.motorMaxRpm_u16 = MAX_APPLICATION_SPEED_RPM;    // Motor Max RPM
  motorParameters_Control.motorParameters_Settings.motorMinRpm_u16 = MIN_APPLICATION_SPEED_RPM;
  motorParameters_Control.motorParameters_Settings.pwmFrequency_u16 = PWM_FREQUENCY;
}

void Init_Tuning01_Settings(void)
{  
  motorTunning01_Control.motorTunning01_Settings.fluxKp_s16 = PID_FLUX_KP_DEFAULT;        // Flux Kp
  motorTunning01_Control.motorTunning01_Settings.fluxKpFactor_u16 = TF_KPDIV;  // Flux Kp factor
  motorTunning01_Control.motorTunning01_Settings.fluxKi_s16 = PID_FLUX_KI_DEFAULT;         // Flux Ki
  motorTunning01_Control.motorTunning01_Settings.fluxKiFactor_u16 = TF_KIDIV; // Flux Ki factor
  motorTunning01_Control.motorTunning01_Settings.fluxKd_s16 = PID_FLUX_KD_DEFAULT;         // Flux Kd
  motorTunning01_Control.motorTunning01_Settings.fluxKdFactor_u16 = TF_KDDIV;  // Flux Kd factor
  motorTunning01_Control.motorTunning01_Settings.torqueKp_s16 = PID_TORQUE_KP_DEFAULT;        // Torque Kp
  motorTunning01_Control.motorTunning01_Settings.torqueKpFactor_u16 = TF_KPDIV;  // Torque Kp factor
  motorTunning01_Control.motorTunning01_Settings.torqueKi_s16 = PID_TORQUE_KI_DEFAULT;         // Torque Ki
  motorTunning01_Control.motorTunning01_Settings.torqueKiFactor_u16 = TF_KIDIV; // Torque Ki factor  
  motorTunning01_Control.motorTunning01_Settings.torqueKd_s16 = PID_TORQUE_KD_DEFAULT;         // Torque Kd
  motorTunning01_Control.motorTunning01_Settings.torqueKdFactor_u16 = TF_KDDIV;  // Torque Kd factor
  motorTunning01_Control.motorTunning01_Settings.speedKp_s16 = PID_SPEED_KP_DEFAULT;         // Speed Kp
  motorTunning01_Control.motorTunning01_Settings.speedKpFactor_u16 = SP_KPDIV;     // Speed Kp factor
  motorTunning01_Control.motorTunning01_Settings.speedKi_s16 = PID_SPEED_KI_DEFAULT;         // Speed Ki
  motorTunning01_Control.motorTunning01_Settings.speedKiFactor_u16 = SP_KIDIV;  // Speed Ki factor
  motorTunning01_Control.motorTunning01_Settings.speedKd_s16 = PID_SPEED_KD_DEFAULT;            // Speed Kd  
  motorTunning01_Control.motorTunning01_Settings.speedKdFactor_u16 = SP_KDDIV;        // Speed Kd factor
 
  motorTunning01_Control.motorTunning01_Settings.varianceThreshold_u16 = 10; // Multiply by ONE_TENTH_FACTOR //% variance of Est. and Mes. speed 
}

void Init_Tuning02_Settings(void)
{
  
  motorTunning02_Control.motorTunning02_Settings.bemfObservedGain01_s16 = GAIN1;
  motorTunning02_Control.motorTunning02_Settings.bemfObserverGain01Factor_u16 = F1;
  motorTunning02_Control.motorTunning02_Settings.bemfObservedGain02_s16 = GAIN2;
  motorTunning02_Control.motorTunning02_Settings.bemfObserverGain02Factor_u16 = F2;
  motorTunning02_Control.motorTunning02_Settings.bemfObservedGain03_s16 = 1;
  motorTunning02_Control.motorTunning02_Settings.bemfObserverGain03Factor_u16 = 1;
  motorTunning02_Control.motorTunning02_Settings.bemfObservedGain04_s16 = 1;
  motorTunning02_Control.motorTunning02_Settings.bemfObserverGain04Factor_u16 = 1;
  motorTunning02_Control.motorTunning02_Settings.bemfObservedGain05_s16 = 1;
  motorTunning02_Control.motorTunning02_Settings.bemfObserverGain05Factor_u16 = 1;
  motorTunning02_Control.motorTunning02_Settings.angleEst01Gain_u16 = PLL_KP_GAIN;  // Angle estimator gain 01 // PLL_KP_GAIN
  motorTunning02_Control.motorTunning02_Settings.angleEst01GainFactor_u16 = PLL_KPDIV; // Angle Estimator Gain 01 Factor
  motorTunning02_Control.motorTunning02_Settings.angleEst02Gain_u16 = PLL_KI_GAIN;   // Angle estimator gain 02 // PLL_KI_GAIN
  motorTunning02_Control.motorTunning02_Settings.angleEst02GainFactor_u16 = PLL_KIDIV; // Angle Estimator Gain 02 Factor
  motorTunning02_Control.motorTunning02_Settings.angleEst03Gain_u16 = 0;   // Angle estimator gain 03
  motorTunning02_Control.motorTunning02_Settings.angleEst03GainFactor_u16 = 1; // Angle Estimator Gain 03 Factor
  motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16 = OBS_MINIMUM_SPEED_RPM;   // Observer Min Speed  
  motorTunning02_Control.motorTunning02_Settings.adcSamplingCycles_u16 = ADC_SAMPLING_CYCLES ; // ADC sampling rate. 
  motorTunning02_Control.motorTunning02_Settings.regSamplingRate_u16 = REGULATION_EXECUTION_RATE;     // Number of PWM cycles before FOC calc is updated
  motorTunning02_Control.motorTunning02_Settings.gain1_s16 = -24470;   // Gain1
  motorTunning02_Control.motorTunning02_Settings.gain2_s16 = 16649;    // Gain2 
  
}

void Init_Limits_Settings(void)
{
  motorLimits01_Control.motorLimits01_Settings.adminFlags01.is_powerLimitEnable = TRUE;
  motorLimits01_Control.motorLimits01_Settings.adminFlags01.is_currentLimitEnable = TRUE;
  motorLimits01_Control.motorLimits01_Settings.adminFlags01.is_ipmTemperatureLimit = TRUE;
  motorLimits01_Control.motorLimits01_Settings.adminFlags01.is_pfcTemperatureLimit = FALSE;
  motorLimits01_Control.motorLimits01_Settings.adminFlags01.is_iclEnable = TRUE;
  
  motorLimits01_Control.motorLimits01_Settings.maxOutputPower_u16 = 1700;     // Absolute Max. Power in Watts 
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimit_u16 = 1617;   // Power Limit Threshold for derate in Watts
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimitHys_u16 = 1550;  
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimitDelay_u16 = 10000;  // Delay before power limit is activated   
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimitRpmReduce_u16 = 10; // Reduce rpm by this about for derate
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimitRmpIncrease_u16 = 10; // Increase RPM by this amount
  motorLimits01_Control.motorLimits01_Settings.outputPowerLimitPeriod_u16 = 200;     // Apply derate after this time period in Sec  
  
  motorLimits01_Control.motorLimits01_Settings.maxOutputCurrent_u16 = Regal_ConvertCountsTomA(21938) ;    // Absolute Max. current IPM can support in mA peak
  motorLimits01_Control.motorLimits01_Settings.outputCurrentLimit_u16 = Regal_ConvertCountsTomA(19744);    // Current Limit Threshold for derate
  motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitHys_u16 = Regal_ConvertCountsTomA(18647); // Current Limit Threshold for derate
  motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitRpmReduce_u16 = 10; // Reduce rpm by this about for derate
  motorLimits01_Control.motorLimits01_Settings.outputCurrentLimitPeriod_u16 = 200;   // Apply derate after this time period in Sec  
  
  motorLimits01_Control.motorLimits01_Settings.maxIpmTemperature_u16 = 125;   // Absolute Max. IPM Temperature in deg C
  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimit_u16 = 1020; // IPM Temperature Limit Threshold for derate
  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitHys_u16 = 970;
  motorLimits01_Control.motorLimits01_Settings.ipmTempratureLimitRpmReduce_u16 = 10;  // Reduce rpm by this about for derate
  motorLimits01_Control.motorLimits01_Settings.ipmTemperatureLimitPeriod_u16 = 30000; // Apply derate after this time period in Sec  
  
  motorLimits01_Control.motorLimits01_Settings.maxPfcTemperature_u16 = 125;   // Absolute Max. PFC Temperature
  motorLimits01_Control.motorLimits01_Settings.pftTemperatureLimit_u16 = 102; // PFC Temperature Limit Threshold for derate
  motorLimits01_Control.motorLimits01_Settings.pfcTemperatureLimitHys_u16 = 97;
  motorLimits01_Control.motorLimits01_Settings.pfcTempratureLimitRpmReduce_u16 = 10;  // Reduce rpm by this about for derate
  motorLimits01_Control.motorLimits01_Settings.pfcTemperatureLimitPeriod_u16 = 30000; // Apply derate after this time period 
  
}

void Init_Protection01_Settings(void)
{
  motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_overVoltageEnable = TRUE;
  motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_underVoltageEnable = TRUE;
  motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_overCurrentEnable = TRUE;
  motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_lossOfInputPhaseEnable = TRUE;
  motorProtections01_Control.motorProtections01_Settings.adminFlags01.is_faultAutoStartEnable = TRUE;
  
  motorProtections01_Control.motorProtections01_Settings.overVoltageThreshold_u16 = 136 ;    //Over Voltage Threshold, % of nominal Vbus * 100
  motorProtections01_Control.motorProtections01_Settings.overVoltageHysteresis_u16 = 5 ;     //Over Voltage Hysteresis, % of nominal Vbus * 100
  motorProtections01_Control.motorProtections01_Settings.overVoltageMaxRetries_u16 = 0xFF;   // Max retries before latching fault, 0xFF is unlimited retries
  motorProtections01_Control.motorProtections01_Settings.overVoltageMinRetryDelay_u16 = 10;  // Delay between each retry in Sec.
  motorProtections01_Control.motorProtections01_Settings.overVoltageMaxRetryDelay_u16 = 120;  // Max delay between each retry in Sec.
  motorProtections01_Control.motorProtections01_Settings.overVoltageRetryDelayFactor_u16 = 1; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  motorProtections01_Control.motorProtections01_Settings.underVoltageThreshold_u16 = 61;    //Under Voltage Threshold, % of nominal Vbus * 100
  motorProtections01_Control.motorProtections01_Settings.underVoltageHysteresis_u16 = 5;    //Under Voltage Hysteresis, % of nominal Vbus * 100
  motorProtections01_Control.motorProtections01_Settings.underVoltageMaxRetries_u16 = 0xFF; // Max retries before latching fault, 0xFF is unlimited retries
  motorProtections01_Control.motorProtections01_Settings.underVoltageMinRetryDelay_u16= 10; // Delay between each retry in Sec.
  motorProtections01_Control.motorProtections01_Settings.underVoltageMaxRetryDelay_u16 = 120;   // Max delay between each retry in Sec.
  motorProtections01_Control.motorProtections01_Settings.underVoltageRetryDelayFactor_u16 = 1; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  motorProtections01_Control.motorProtections01_Settings.overCurrentThreshold_u16 = 100;    //Over Current threshold, % of nominal current * 100
  motorProtections01_Control.motorProtections01_Settings.overCurrentHysteresis_u16 = 2;     // Over Current Hysteresis
  motorProtections01_Control.motorProtections01_Settings.overCurrentMaxRetries_u16 = 0xFF;  // Max retries before latching fault
  motorProtections01_Control.motorProtections01_Settings.overCurrentMinRetryDelay_u16 = 30; // Delay between each retry
  motorProtections01_Control.motorProtections01_Settings.overCurrentMaxRetryDelay_u16 = 120;   // Max delay between each retry in Sec.
  motorProtections01_Control.motorProtections01_Settings.overCurrentRetryDelayFactor_u16 = 10; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMaxRetries_u16 = 0xFF;
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMinRetryDelay_u16 = 10;
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMaxRetryDelay_u16 = 120;
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseRetryDelayFactor_u16 = 1;
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMinPower_u16 = 500; 
  motorProtections01_Control.motorProtections01_Settings.lossOfInputPhaseMaxPower_u16 = 500;
}

void Init_Protection02_Settings(void)
{
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_ipmOverTemperatureEnable = TRUE;
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_speedFeedBackFaultEnable = TRUE;
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_startUpFaultEnable = TRUE;
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_softwareFaultEnable = TRUE;
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_focFaultEnable = TRUE;
  motorProtections02_Control.motorProtections02_Settings.adminFlags01.is_pfcOverTemperatureEnable = TRUE;
  
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureThreshold_u16 = 1200; //120 deg C //Over Temperature Threshold, % of MAX IPM temp (maxIpmTemperature_u16)
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureHysteresis_u16 = 50;  // 5deg C //Over Temperature Hysteresis %*100
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureMaxRetries_u16 = 0xFF;    // Max retries before latching fault
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureMinRetryDelay_u16 = 30;   // Delay between each retry in Sec
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureMaxRetryDelay_u16 = 120;  // Max delay between each retry in Sec.
  motorProtections02_Control.motorProtections02_Settings.ipmOverTemperatureRetryDelayFactor_u16 = 1; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureThreshold_u16 = 1100; //Over Temperature Threshold
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureHysteresis_u16 = 50;  // Over Temperature Hysteresis
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureMaxRetries_u16 = 0xFF;     // Max retries before latching fault
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureMinRetryDelay_u16 = 30;   // Delay between each retry
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureMaxRetryDelay_u16 = 120;   // Max delay between each retry in Sec.
  motorProtections02_Control.motorProtections02_Settings.pfcOverTemperatureRetryDelayFactor_u16 = 1; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1) 
}

void Init_Otf_Settings(void)
{
  otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable = TRUE;//FALSE;
  otfParameters_Control.otfParameters_Settings.detectDuration_u16 = PHASE1_DURATION;//250; // PHASE_1_DURATION
  otfParameters_Control.otfParameters_Settings.detectBemfGain_u16 = 256; // DBEMFG
  otfParameters_Control.otfParameters_Settings.maxBemfGain_u16 = 307; // Max BEMFG, % of detectBemfGain_u16 * 100
  otfParameters_Control.otfParameters_Settings.minBemfGain_u16 = 205;  // Min BEMFG, % of detectBemfGain_u16 * 100
  otfParameters_Control.otfParameters_Settings.maxSyncSpeed_u16 = 133 ;   // Max Sync Speed, in RPM
  otfParameters_Control.otfParameters_Settings.minSyncSpeed_u16 = 41;   // Min Sync Speed, in RPM
  otfParameters_Control.otfParameters_Settings.loadIntertia_u16 = 20;
  otfParameters_Control.otfParameters_Settings.stopWaitTime_u16 = 1000;
  
  otfParameters_Control.otfParameters_Settings.adminFlags01.is_windmillControlEnable = FALSE;
  otfParameters_Control.otfParameters_Settings.wmMaxReverseSpeed_u16 = 125; // Max Reverse Speed in RPM
  otfParameters_Control.otfParameters_Settings.wmClampDuration_u16 = 9000;   // Clamp Duration in mSec
  otfParameters_Control.otfParameters_Settings.wmClampCurrent_u16 = Regal_ConvertCountsTomA(15356); //2500;    // Clamp Current in mAmps
  otfParameters_Control.otfParameters_Settings.wmMaxClampCurrent_u16 = Regal_ConvertCountsTomA(21938);
  otfParameters_Control.otfParameters_Settings.wmReverseEndSpeed_u16 = 30;  // Reverse End Speed in RPM
  otfParameters_Control.otfParameters_Settings.wmClampRamp_u16 = 100;         // Clamp Ramp in mSec
  otfParameters_Control.otfParameters_Settings.wmCoilShortDuration_u16 = 5000; // Coil short Duration in mSec

}

void Init_Braking_Settings(void)
{
  brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_decelControlEnable = TRUE;
  brakingParameters_Control.brakingParameters_Settings.adminFlags01.is_brakingControlEnable = TRUE;
  brakingParameters_Control.brakingParameters_Settings.lowSideOnDuration_u16 = 5000;    // Low side on Duration in mSec
  brakingParameters_Control.brakingParameters_Settings.brakeEndSpeed_u16 = 40;        // Brake End Speed
  brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16 = 439;  // Brake Current seeding in mAmps
  brakingParameters_Control.brakingParameters_Settings.brakingMaxVbus_u16 = 129; // 256V above nominal Vbus is clipped to this % of Max Vbus
  brakingParameters_Control.brakingParameters_Settings.brakingRampA_s16 = 7;     // Ramp a, gain
  brakingParameters_Control.brakingParameters_Settings.brakingRampB_s16 = -3;    // Ramp b, gain
  brakingParameters_Control.brakingParameters_Settings.brakingRampC_s16 = 725;   // Ramp c, gain
  brakingParameters_Control.brakingParameters_Settings.brakingRampEndCurrent_u16 = 10; // Ramp end current in mAmps
  brakingParameters_Control.brakingParameters_Settings.brakeSpeedTranstionToBrake_u16 = 67; // Speed Transition to Brake
  brakingParameters_Control.brakingParameters_Settings.brakeCurrentRampStep_u16 = 10; // Current ramp step in mAmps
  brakingParameters_Control.brakingParameters_Settings.brakeLowCurrentLimit_u16 = 2194; // Low current limit in mAmps
//  brakingParameters_Control.brakingParameters_Settings.brakeKp_u16 = 45;
//  brakingParameters_Control.brakingParameters_Settings.brakeKpFactor_u16 = 16;
//  brakingParameters_Control.brakingParameters_Settings.brakeKi_u16 = 10;
//  brakingParameters_Control.brakingParameters_Settings.brakeKiFactor_u16 = 256;
//  brakingParameters_Control.brakingParameters_Settings.brakeKd_u16 = 0;
//  brakingParameters_Control.brakingParameters_Settings.brakeKdFactor_u16 = 16;
}

//void Init_Windmill_Settings(void)
//{
//  windMillingParameters_Control.windMillingParameters_Settings.adminFlags01.is_windmillControlEnable = TRUE;
//  windMillingParameters_Control.windMillingParameters_Settings.wmMaxReverseSpeed_u16 = 125; // Max Reverse Speed in RPM
//  windMillingParameters_Control.windMillingParameters_Settings.wmClampDuration_u16 = 9000;   // Clamp Duration in mSec
//  windMillingParameters_Control.windMillingParameters_Settings.wmClampCurrent_u16 = Regal_ConvertCountsTomA(15356); //2500;    // Clamp Current in mAmps
//  windMillingParameters_Control.windMillingParameters_Settings.wmMaxClampCurrent_u16 = Regal_ConvertCountsTomA(21938);
//  windMillingParameters_Control.windMillingParameters_Settings.wmReverseEndSpeed_u16 = 30;  // Reverse End Speed in RPM
//  windMillingParameters_Control.windMillingParameters_Settings.wmClampRamp_u16 = 100;         // Clamp Ramp in mSec
//  windMillingParameters_Control.windMillingParameters_Settings.wmCoilShortDuration_u16 = 5000; // Coil short Duration in mSec
//}

void Init_Hw_Specific_Settings(void)
{
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.nominalVbusVolts_u16 = 330; // DC bus volts
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.iaCalSlope_u16 = 1;    // Ia Cal Slope
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.iaCalOffset_u16 = 0;   // Ia Cal Offset
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.ibCalSlope_u16 = 1;    // Ib Cal Slope
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.ibCalOffset_u16 = 0;   // Ib Cal Offset
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.vBusCalSlope_u16 = 1;  // Vbus Cal Slope
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.vBusCalOffset_u16 = 0; // Vbus Cal Offset
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.ipmCalSlope_u16 = 1;   // IPM Temp Cal Slope
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.ipmCalOffset_u16 = 0;  // IPM Temp Cal Offset
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.pfcCalSlope_u16 = 1;   // PFC Temp Cal Slope
  hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.pfcCalOffset_u16 = 0;  // PFC temp Cal Offset
}

void Init_App_Specific_Settings(void)
{
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.controlMode_u16 = SPEED_CONTROL_MODE;
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationRpm_u16 = 400;   // Min. Application RPM
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.maxApplicationRpm_u16 = 3150;  // Max. Application RPM
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.rampUpRate_u16 = 200;   // Acceleration RPM/Sec
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.rampDownRate_u16 = 100; // Deceleration RPM/Sec
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.stopSpeed_u16 = 100;    // Speed at which motor is considered stopped
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.directionOfRotation_u16 = STD_ROTATION;   // REVERSE_ROTATION = 6, STD_ROTATION = 9 //Direction of Rotation
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationTorque_u16 = 0;  // Min Torque in ft-lb*10
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.maxApplictionTorque_u16 = 139;   // Max Torque in ft-lb*10
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.minApplicationAirflow_u16 = 0; // Min Airflow in cfm
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.maxApplicationAirflow_u16 = 5000; // Max Airflow in cfm
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.motorSpinPollPeriod_u16 = 18250;
  applicationSpecificParameters_Control.applicationSpecificParameters_Settings.motorSpinTimeout_u16 = 4;
}

void Init_HC01_Settings(void)
{
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleOffsets1_u16 = 1; // Angle Offsets
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleMultlipliers1_u16 = 2; // Angle Multipliers
  harmonicCompensation01_Control.harmonicCompensation01_Settings.amplitudes1_u16 = 3;  // Amplitudes
  harmonicCompensation01_Control.harmonicCompensation01_Settings.minSpeeds1_u16 = 5;   // Min Speeds
  harmonicCompensation01_Control.harmonicCompensation01_Settings.maxSpeeds1_u16 = 6;   // Max Speeds
  
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleOffsets2_u16 = 7; // Angle Offsets
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleMultlipliers2_u16 = 8; // Angle Multipliers
  harmonicCompensation01_Control.harmonicCompensation01_Settings.amplitudes2_u16 = 9;   // Amplitudes
  harmonicCompensation01_Control.harmonicCompensation01_Settings.minSpeeds2_u16 = 10;   // Min Speeds
  harmonicCompensation01_Control.harmonicCompensation01_Settings.maxSpeeds2_u16 = 11;   // Max Speeds
  
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleOffsets3_u16 = 12; // Angle Offsets
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleMultlipliers3_u16 = 13; // Angle Multipliers
  harmonicCompensation01_Control.harmonicCompensation01_Settings.amplitudes3_u16 = 14;  // Amplitudes
  harmonicCompensation01_Control.harmonicCompensation01_Settings.minSpeeds3_u16 = 15;   // Min Speeds
  harmonicCompensation01_Control.harmonicCompensation01_Settings.maxSpeeds3_u16 = 16;   // Max Speeds
  
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleOffsets4_u16 = 17; // Angle Offsets
  harmonicCompensation01_Control.harmonicCompensation01_Settings.angleMultlipliers4_u16 = 18; // Angle Multipliers
  harmonicCompensation01_Control.harmonicCompensation01_Settings.amplitudes4_u16 = 19;  // Amplitudes
  harmonicCompensation01_Control.harmonicCompensation01_Settings.minSpeeds4_u16 = 20;   // Min Speeds
  harmonicCompensation01_Control.harmonicCompensation01_Settings.maxSpeeds4_u16 = 21;   // Max Speeds
  
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_harmonicCompensationModule1Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_harmonicCompensation1Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_phaseInvert1Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_harmonicCompensation2Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_phaseInvert2Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_harmonicCompensation3Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_phaseInvert3Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_harmonicCompensation4Enable = TRUE;
  harmonicCompensation01_Control.harmonicCompensation01_Settings.adminFlags01.is_phaseInvert4Enable = TRUE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty10 = TRUE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty11 = FALSE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty12 = TRUE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty13 = FALSE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty14 = TRUE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty15 = FALSE;
//  harmonicCompensation1_Control.harmonicCompensation1_Settings.flags_u16.empty16 = TRUE;
}

void Init_HC02_Settings(void)
{
  //harmonicCompensation_Control.harmonicCompensation_Settings.harmonicCompensationEnable_u16; // Harmonic Compensation Enable
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleOffsets5_u16 = 22; // Angle Offsets
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleMultlipliers5_u16 = 23; // Angle Multipliers
  harmonicCompensation02_Control.harmonicCompensation02_Settings.amplitudes5_u16 = 24;  // Amplitudes
  harmonicCompensation02_Control.harmonicCompensation02_Settings.minSpeeds5_u16 = 25;   // Min Speeds
  harmonicCompensation02_Control.harmonicCompensation02_Settings.maxSpeeds5_u16 = 26;   // Max Speeds
  
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleOffsets6_u16 = 27; // Angle Offsets
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleMultlipliers6_u16 = 28; // Angle Multipliers
  harmonicCompensation02_Control.harmonicCompensation02_Settings.amplitudes6_u16 = 29;  // Amplitudes
  harmonicCompensation02_Control.harmonicCompensation02_Settings.minSpeeds6_u16 = 30;   // Min Speeds
  harmonicCompensation02_Control.harmonicCompensation02_Settings.maxSpeeds6_u16 = 31;   // Max Speeds
  
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleOffsets7_u16 = 32; // Angle Offsets
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleMultlipliers7_u16 = 33; // Angle Multipliers
  harmonicCompensation02_Control.harmonicCompensation02_Settings.amplitudes7_u16 = 34;  // Amplitudes
  harmonicCompensation02_Control.harmonicCompensation02_Settings.minSpeeds7_u16 = 35;   // Min Speeds
  harmonicCompensation02_Control.harmonicCompensation02_Settings.maxSpeeds7_u16 = 36;   // Max Speeds
  
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleOffsets8_u16 = 37; // Angle Offsets
  harmonicCompensation02_Control.harmonicCompensation02_Settings.angleMultlipliers8_u16 = 38; // Angle Multipliers
  harmonicCompensation02_Control.harmonicCompensation02_Settings.amplitudes8_u16 = 39;  // Amplitudes
  harmonicCompensation02_Control.harmonicCompensation02_Settings.minSpeeds8_u16 = 40;   // Min Speeds
  harmonicCompensation02_Control.harmonicCompensation02_Settings.maxSpeeds8_u16 = 41;   // Max Speeds
  
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_harmonicCompensationModule2Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_harmonicCompensation5Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_phaseInvert5Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_harmonicCompensation6Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_phaseInvert6Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_harmonicCompensation7Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_phaseInvert7Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_harmonicCompensation8Enable = TRUE;
  harmonicCompensation02_Control.harmonicCompensation02_Settings.adminFlags01.is_phaseInvert8Enable = TRUE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty10 = TRUE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty11 = FALSE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty12 = TRUE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty13 = FALSE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty14 = TRUE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty15 = FALSE;
//  harmonicCompensation2_Control.harmonicCompensation2_Settings.flags_u16.empty16 = TRUE;
}

void Init_HC03_Settings(void)
{
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleOffsets9_u16 = 42; // Angle Offsets
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleMultlipliers9_u16 = 43; // Angle Multipliers
  harmonicCompensation03_Control.harmonicCompensation03_Settings.amplitudes9_u16 = 44;   // Amplitudes
  harmonicCompensation03_Control.harmonicCompensation03_Settings.minSpeeds9_u16 = 45;    // Min Speeds
  harmonicCompensation03_Control.harmonicCompensation03_Settings.maxSpeeds9_u16 = 46;    // Max Speeds
  
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleOffsets10_u16 = 47; // Angle Offsets
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleMultlipliers10_u16 = 48; // Angle Multipliers
  harmonicCompensation03_Control.harmonicCompensation03_Settings.amplitudes10_u16 = 49;   // Amplitudes
  harmonicCompensation03_Control.harmonicCompensation03_Settings.minSpeeds10_u16 = 50;    // Min Speeds
  harmonicCompensation03_Control.harmonicCompensation03_Settings.maxSpeeds10_u16 = 51;    // Max Speeds
  
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleOffsets11_u16 = 52; // Angle Offsets
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleMultlipliers11_u16 = 53; // Angle Multipliers
  harmonicCompensation03_Control.harmonicCompensation03_Settings.amplitudes11_u16 = 54;   // Amplitudes
  harmonicCompensation03_Control.harmonicCompensation03_Settings.minSpeeds11_u16 = 55;    // Min Speeds
  harmonicCompensation03_Control.harmonicCompensation03_Settings.maxSpeeds11_u16 = 56;    // Max Speeds
  
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleOffsets12_u16 = 57; // Angle Offsets
  harmonicCompensation03_Control.harmonicCompensation03_Settings.angleMultlipliers12_u16 = 58; // Angle Multipliers
  harmonicCompensation03_Control.harmonicCompensation03_Settings.amplitudes12_u16 = 59;   // Amplitudes
  harmonicCompensation03_Control.harmonicCompensation03_Settings.minSpeeds12_u16 = 60;    // Min Speeds
  harmonicCompensation03_Control.harmonicCompensation03_Settings.maxSpeeds12_u16 = 61;    // Max Speeds
  
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_harmonicCompensationModule3Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_harmonicCompensation9Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_phaseInvert9Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_harmonicCompensation10Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_phaseInvert10Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_harmonicCompensation11Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_phaseInvert11Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_harmonicCompensation12Enable = TRUE;
  harmonicCompensation03_Control.harmonicCompensation03_Settings.adminFlags01.is_phaseInvert12Enable = TRUE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty10 = TRUE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty11 = FALSE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty12 = TRUE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty13 = FALSE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty14 = TRUE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty15 = FALSE;
//  harmonicCompensation3_Control.harmonicCompensation3_Settings.flags_u16.empty16 = TRUE;
}

void Init_St_Mc_Param()
{
  stParameters01_Control.stParameters01_Settings.phase1Speed_u16 = PHASE1_FINAL_SPEED_UNIT;
  stParameters01_Control.stParameters01_Settings.phase2Speed_u16 = PHASE2_FINAL_SPEED_UNIT;
  stParameters01_Control.stParameters01_Settings.phase1FinalCurrent_u16 = PHASE1_FINAL_CURRENT;
  stParameters01_Control.stParameters01_Settings.phase2FinalCurrent_u16 = PHASE2_FINAL_CURRENT;
  stParameters01_Control.stParameters01_Settings.nbConsecutiveTest_u16 = 2;     // NB_CONSECUTIVE_TESTS
  stParameters01_Control.stParameters01_Settings.pwmFreqScaling_u16 = 1;
  
}



void Init_Fan_Settings(void)
{
  
}

void Init_Test_Settings(void)
{

}

