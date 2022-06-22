/**
********************************************************************************************************************************
* @file    regal_mc_lib.c
* @author  Roel Pantonial
* @brief   This source file contains the regal motor control library
* @details This file has declarations for motor control algorithms such as on-the-fly and non-regenerative braking
********************************************************************************************************************************
*/

#include "drive_parameters.h"
#include "regal_mc_lib.h"
#include "bus_voltage_sensor.h"
#include "parameters_conversion.h"
#include "mc_math.h"
#include "mc_api.h"
#include "mc_config.h"
#include "revup_ctrl.h"
#include "zz_module_flash.h"
#include "regal_mc_settings.h"
#include "module_test.h"

/*Private Variables */
static uint8_t Imax_count = 0;     /* sampling count (ms) for maximum current reference update for copper loss controller */
static uint16_t LowSide_count = 0; /* count (ms) for shorting coils through low-side switching */
static uint16_t OTF_Ph1_count = 0; /* Synchronisation phase count */
static uint8_t ImCtrl_en = 1;      /* copper loss control enable bit */
static int16_t Spd_Ph0_End;        /* speed at the end of detection phase */

#define CURRENT_DERATING_BUFFER_SIZE 10
int32_t avrCurrentRd[]={0,0,0,0,0,0,0,0,0,0};       // RPa: ensure that it has the same number of element as defined in CURRENT_DERATING_BUFFER_SIZE
int32_t avrCurrentRdOP = 0;
/* Phase Current averaging */
#define PHASE_CURRENT_BUFFER_SIZE       40u
uint16_t avrPhACurrentRd[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t avrPhBCurrentRd[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t avrPhACurrentRdOP = 0;
uint32_t avrPhBCurrentRdOP = 0;

extern MotorTunning01_Control motorTunning01_Control;
extern MotorTunning02_Control motorTunning02_Control;
extern MotorParameters_Control motorParameters_Control;
extern StartupParameters_Control startupParameters_Control;
extern StParameters01_Control stParameters01_Control;
//extern WindMillingParameters_Control windMillingParameters_Control;
extern BrakingParameters_Control brakingParameters_Control;
extern OtfParameters_Control otfParameters_Control;
extern ModuleTest_Control moduleTest_Control;
extern HardwareSpecificParameters_Control hardwareSpecificParameters_Control;
extern MotorLimits01_Control motorLimits01_Control;
/**
* @brief  Burnin Parameters
* @details frequency and current  
*/
Burnin_Handle_t Burnin_M1 =
{
  .en_ctrl = DISABLE,
  .frequency = 50,
  .current = 150,
};

/**
* @brief  Power Calculation parameters Motor 1
**/
Power_Handle_t EEPowerCalcHandle_M1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR};

/**
* @brief  PI / PID Bus Voltage parameters Motor 1
* @details default values are assigned from a pre-compiled definitions   
*/
PID_Handle_t PIDBkHandle_M1 =
{
  .hDefKpGain = (int16_t)PID_BRAKE_KP_DEFAULT,                               // Proportional Gain for the Bus Voltage Controller
  .hDefKiGain = (int16_t)PID_BRAKE_KI_DEFAULT,                               // Integral Gain for the the Bus Voltage Controller
  // .wUpperIntegralLimit = (int32_t)default_LOWERIQLIMIT * (int32_t)BK_KIDIV,  //This will be dynamically updated based on motor direction
  // .wLowerIntegralLimit = -(int32_t)default_LOWERIQLIMIT * (int32_t)BK_KIDIV, //This will be dynamically updated based on motor direction
  .hUpperOutputLimit = 0,                                                    //A factor of the OVP, this will be dynamically updated based on motor direction
  //.hLowerOutputLimit = -default_LOWERIQLIMIT,                                //A factor of the UVP, this will be dynamically updated based on motor direction
  .hKpDivisor = (uint16_t)BK_KPDIV,                                          // Proportional Gain Divisor for the Bus Voltage Controller
  .hKiDivisor = (uint16_t)BK_KIDIV,                                          // Integral Gain Divisor for the Bus Voltage Controller
  .hKpDivisorPOW2 = (uint16_t)BK_KPDIV_LOG,                                  //Proportional Gain divider (in linear scale) for Bus Voltage Controller
  .hKiDivisorPOW2 = (uint16_t)BK_KIDIV_LOG,                                  //Integral Gain divider (in linear scale) for Bus Voltage Controller
  .hDefKdGain = 0x0000U,
  .hKdDivisor = 0x0000U,
  .hKdDivisorPOW2 = 0x0000U,
  //        
};

/**
* @brief  PI / PID Imax Controller (Copper Loss Controller) parameters Motor 1
* @details default values are assigned from a pre-compiled definitions  
*/
PID_Handle_t PIDImHandle_M1 =
{
  .hDefKpGain = (int16_t)PID_IMAX_KP_DEFAULT,                                       // Proportional Gain for the Copper Loss Controller
  .hDefKiGain = (int16_t)PID_IMAX_KI_DEFAULT,                                       // Integral Gain for the Copper Loss Controller
  //.wUpperIntegralLimit = (int32_t)default_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV,  //Upper Integral Limit for the Copper Loss Controller
  //.wLowerIntegralLimit = -(int32_t)default_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV, //Lower Integral Limit for the Copper Loss Controller
  //.hUpperOutputLimit = default_BK_CURRENTSEEDING,                                   //INT16_MAX,
  //.hLowerOutputLimit = -default_BK_CURRENTSEEDING,                                  //RPa: make sure that this is more than -Flux/Ld,
  .hKpDivisor = (uint16_t)IMAX_KPDIV,                                               //Proportional Gain divider for Copper Loss Controller
  .hKiDivisor = (uint16_t)IMAX_KIDIV,                                               //Integral Gain divider for Copper Loss Controller
  .hKpDivisorPOW2 = (uint16_t)IMAX_KPDIV_LOG,                                       //Proportional Gain divider (in linear scale) for Copper Loss Controller
  .hKiDivisorPOW2 = (uint16_t)IMAX_KIDIV_LOG,                                       //Integral Gain divider (in linear scale) for Copper Loss Controller
  .hDefKdGain = 0x0000U,
  .hKdDivisor = 0x0000U,
  .hKdDivisorPOW2 = 0x0000U,
};

/**
* @brief  Brake parametrs Motor 1
* @details comments removed with variable information
*     .Vbus_Clip = BK_VBUS_CLIP             **Vbus upper limit to give motor braking power        
*/
Braking_Handle_t BrakeHandle_M1 =
{
  .Nbar = default_NBar,              /* Feed-forward gain */
  .BrakingPhase = CURRENT_STARTRAMP, /* Braking state used for Id injection */
  .IMax_Ref = default_IMRef,         /* default IMax reference for copper loss control */
  .FeedForward_term = 0,             /* Feedforward Voltage that would improve transient response without affecting the stability of the system */
  .TransitionPhase = NO_TRANSITION,  /* Check state transitions when start and stopping motor */
};

/**
* @brief  OTF Parameters Motor 1
* @details comments removed with variable information
*             .MaxSyncSpeed = OTF_MAX_SYNC_SPEED, ** Maximum speed for motor to synchronise 
*             .MinSyncSpeed = OTF_MIN_SYNC_SPEED, ** Minimum speed for motor to synchronise 
*             .detect_bemfg = OTF_DBEMFG, ** bemf gain for good detection
*             .max_bemfg = OTF_MAX_BEMFG, ** maximum gain to clip vbus without causing instability to the observer or OCP tripping
*             .min_bemfg = OTF_MIN_BEMFG ** minimum gain for vbus clipping without causing OVP tripping, at the nominal voltage, can be set to 256 to flatten out dc bus    
*/
OTF_Handle_t OTFHandle_M1 =
{
  .hdir = WM_RESET,                      /* check for collinearity during detection phase */
  //.hSyncTRef = TREF_SYNC,                /* Reference Iq during transition to speed control, can be set to the final ramp current in the drive parameters (or 70% of it) */
  //.CoilShortSpeed = default_BK_ENDSPEED, /* speed in the syncrhonisation phase when turning the low-side is safe and be done to put motor to stationary position */
  .seamless_transfer = 0,                /* tracking the different phase of revup and used for integrator seeding */
  .bemfg_alpha = 256,                    /* alpha gain of the bemf */
  .bemfg_beta = 256,                     /* beta gain of the bemf */
};

/**
* @brief Execution of DC Bus Voltage Control
*/
int32_t FOC_BusVoltageControlM1(Braking_Handle_t *pHandle, PID_Handle_t *pPIDHandle, RDivider_Handle_t *pBSHandle)
{
  int32_t hVBusError;
  uint16_t BusVoltCheck_16bit;
  int32_t BrakeTorque_q;
  
  // Compute Error given BusVoltageRef and measured bus voltage in 16bit representation
  BusVoltCheck_16bit = VBS_GetAvBusVoltage_d(&(pBSHandle->_Super)); /* do bus voltage measurements */
  hVBusError = (int32_t)BusVoltCheck_16bit - (int32_t)(((uint32_t)pHandle->Adapt_BusVoltageRef * FP16) / ((uint32_t)(pBSHandle->_Super.ConversionFactor)));
  
  // Check Direction
  int16_t dir = MC_GetImposedDirectionMotor1(); //direction information will be used for the sign of braking torque
  // PI Control
  BrakeTorque_q = (int32_t)PI_Controller(pPIDHandle, hVBusError) * dir; // By default, it utilises ST-Micro PI structure
  
  return (BrakeTorque_q);
}

/**
* @brief Execution of maximum current control
*/
int32_t FOC_ImaxCurrentControllerM1(Braking_Handle_t *pHandle, PID_Handle_t *pPIDHandle, FOCVars_t *pFOCHandle_t)
{
  int32_t CurrentLoss;
  int32_t hCurrentError;
  uint32_t current_meas;
  
  // Compute Error given with current reference and the magnitude of the current vector
  current_meas = (uint32_t)((int32_t)pFOCHandle_t->Iqd.d * (int32_t)pFOCHandle_t->Iqd.d) + (uint32_t)((int32_t)pFOCHandle_t->Iqd.q * (int32_t)pFOCHandle_t->Iqd.q);
  current_meas = MCM_Sqrt((int32_t)current_meas); /* scalar current measurement based on its d and q components */
  hCurrentError = (int32_t)current_meas - (int32_t)pHandle->IMax_Ref;
  // PI Control
  CurrentLoss = PI_Controller(pPIDHandle, hCurrentError);
  
  //added feed-forward term to improve response of the controller without affecting stability
  pHandle->FeedForward_term = ((int32_t)pHandle->Nbar * (int32_t)pHandle->IMax_Ref) >> BYTE_SHIFT;
  
  //Output_calculation, maximum current reference for d-current injection
  CurrentLoss = CurrentLoss - pHandle->FeedForward_term;
  
  //RPa: an insurance that d-current will never get to the IV quadrant
  if (CurrentLoss > 0)
    CurrentLoss = 0;
  
  return (CurrentLoss);
}

/**
* @brief Initialisation of Braking structure
*/
void BrakingStruct_Init(Braking_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTCHandle)
{
  pHandle->rMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pSTCHandle->SPD); //take the absolute value of speed measure
  //  pHandle->Adapt_IMax = (int32_t)((A_BK_RAMP_a * (int32_t)pHandle->rMeasuredSpeed * (int32_t)pHandle->rMeasuredSpeed) >> BYTE_SHIFT) +
  //                        (int32_t)(A_BK_RAMP_b * (int32_t)pHandle->rMeasuredSpeed) + A_BK_RAMP_c; // Initialised the adapted maximum current reference for copper loss controller
  //  
  //   PIDBkHandle_M1.wUpperIntegralLimit = (int32_t)A_LOWERIQLIMIT * (int32_t)BK_KIDIV;
  //   PIDBkHandle_M1.wLowerIntegralLimit = - (int32_t)A_LOWERIQLIMIT * (int32_t)BK_KIDIV;
  //   PIDBkHandle_M1.hLowerOutputLimit = - A_LOWERIQLIMIT;
  //   
  //   PIDImHandle_M1.wUpperIntegralLimit = (int32_t)A_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV;
  //   PIDImHandle_M1.wLowerIntegralLimit = - (int32_t)A_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV;
  //   PIDImHandle_M1.hUpperOutputLimit = A_BK_CURRENTSEEDING;
  //   PIDImHandle_M1.hLowerOutputLimit = - A_BK_CURRENTSEEDING;
  //
  //   OTFHandle_M1.hSyncTRef = (int16_t) (70 * (int32_t) A_PHASE4_FINAL_CURRENT / 100);
  //   OTFHandle_M1.CoilShortSpeed = A_BK_ENDSPEED;
  
  pHandle->Adapt_IMax = (int32_t)((brakingParameters_Control.brakingParameters_Settings.brakingRampA_s16 * (int32_t)pHandle->rMeasuredSpeed * (int32_t)pHandle->rMeasuredSpeed) >> BYTE_SHIFT) +
                        (int32_t)(brakingParameters_Control.brakingParameters_Settings.brakingRampB_s16 * (int32_t)pHandle->rMeasuredSpeed) + brakingParameters_Control.brakingParameters_Settings.brakingRampC_s16; // Initialised the adapted maximum current reference for copper loss controller
  
  PIDBkHandle_M1.wUpperIntegralLimit = (int32_t)(brakingParameters_Control.brakingParameters_Settings.brakeLowCurrentLimit_u16) * (int32_t)BK_KIDIV; //(int32_t)brakingParameters_Control.brakingParameters_Settings.brakeKiFactor_u16;
  PIDBkHandle_M1.wLowerIntegralLimit = -(int32_t)(brakingParameters_Control.brakingParameters_Settings.brakeLowCurrentLimit_u16) * (int32_t)BK_KIDIV; //(int32_t)brakingParameters_Control.brakingParameters_Settings.brakeKiFactor_u16;
  PIDBkHandle_M1.hLowerOutputLimit = -(brakingParameters_Control.brakingParameters_Settings.brakeLowCurrentLimit_u16);
  
//  PIDBkHandle_M1.hDefKpGain = (int16_t)brakingParameters_Control.brakingParameters_Settings.brakeKp_u16; // Proportional Gain for the Bus Voltage Controller
//  PIDBkHandle_M1.hDefKiGain = (int16_t)brakingParameters_Control.brakingParameters_Settings.brakeKi_u16; // Integral Gain for the the Bus Voltage Controller
//  PIDBkHandle_M1.hKpDivisor = (uint16_t)brakingParameters_Control.brakingParameters_Settings.brakeKpFactor_u16; // Proportional Gain Divisor for the Bus Voltage Controller
//  PIDBkHandle_M1.hKiDivisor = (uint16_t)brakingParameters_Control.brakingParameters_Settings.brakeKiFactor_u16; // Integral Gain Divisor for the Bus Voltage Controller
//  PIDBkHandle_M1.hKpDivisorPOW2 = (uint16_t)LOG2(brakingParameters_Control.brakingParameters_Settings.brakeKpFactor_u16); //Proportional Gain divider (in linear scale) for Bus Voltage Controller
//  PIDBkHandle_M1.hKiDivisorPOW2 = (uint16_t)LOG2(brakingParameters_Control.brakingParameters_Settings.brakeKiFactor_u16); //Integral Gain divider (in linear scale) for Bus Voltage Controller
//  
  
  PIDImHandle_M1.wUpperIntegralLimit = (int32_t)brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16 * (int32_t)IMAX_KIDIV;
  PIDImHandle_M1.wLowerIntegralLimit = - (int32_t)brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16 * (int32_t)IMAX_KIDIV;
  PIDImHandle_M1.hUpperOutputLimit = brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16;
  PIDImHandle_M1.hLowerOutputLimit = - brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16;
  
  OTFHandle_M1.hSyncTRef = (int16_t) (70 * (int32_t) startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16 / 100);
  OTFHandle_M1.CoilShortSpeed = brakingParameters_Control.brakingParameters_Settings.brakeEndSpeed_u16;
}

/**
* @brief Motor Braking State Machine
*/
void MotorBraking_StateMachine(Braking_Handle_t *pBkHandle, PID_Handle_t *pPIDBusHandle, PID_Handle_t *pPIDImHandle, SpeednTorqCtrl_Handle_t *pSTCHandle, FOCVars_t *pFOCHandle, RDivider_Handle_t *pBSHandle)
{
  //RPa controlled way to set the next Iq and Id
  pBkHandle->rMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pSTCHandle->SPD);
  //RPa: take the absolute value of speed measure
  int16_t MeasuredSpeedAbsValue = (pBkHandle->rMeasuredSpeed < 0 ? (-pBkHandle->rMeasuredSpeed) : (pBkHandle->rMeasuredSpeed));
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //RPa: IMax trajectory is controlled to always be within the motor loss ellipse (copper+iron losses)
  if (Imax_count >= 2) //RPa: does an IMax trajectory sampling every 2msec but sampling can be increased
  {
    //pBkHandle->Adapt_IMax = (int32_t)((A_BK_RAMP_a * (int32_t)MeasuredSpeedAbsValue * (int32_t)MeasuredSpeedAbsValue) >> BYTE_SHIFT) +
    //                        (int32_t)(A_BK_RAMP_b * (int32_t)MeasuredSpeedAbsValue) + A_BK_RAMP_c;
    pBkHandle->Adapt_IMax = (int32_t)((brakingParameters_Control.brakingParameters_Settings.brakingRampA_s16 * (int32_t)MeasuredSpeedAbsValue * (int32_t)MeasuredSpeedAbsValue) >> BYTE_SHIFT) +
      (int32_t)(brakingParameters_Control.brakingParameters_Settings.brakingRampB_s16 * (int32_t)MeasuredSpeedAbsValue) + brakingParameters_Control.brakingParameters_Settings.brakingRampC_s16;
    Imax_count = 0;
  }
  Imax_count++;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // State Machine for the d current injection based on motor loss ellipse
  switch (pBkHandle->BrakingPhase)
  {
  case CURRENT_STARTRAMP: // first state when controlled braking is invoked; either to ramp current up or down
    //if (MeasuredSpeedAbsValue >= A_SPEED_TRANSITION)
    if (MeasuredSpeedAbsValue >= brakingParameters_Control.brakingParameters_Settings.brakeSpeedTranstionToBrake_u16)
      pBkHandle->BrakingPhase = CURRENT_RAMPUP;
    else
      pBkHandle->BrakingPhase = CURRENT_RAMPDOWN;
    Imax_count = 0;
    //pPIDImHandle->hLowerOutputLimit = -A_BK_CURRENTSEEDING;
    //pPIDImHandle->hUpperOutputLimit = A_BK_CURRENTSEEDING;
    //pPIDImHandle->wUpperIntegralLimit = (int32_t)A_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV;
    //pPIDImHandle->wLowerIntegralLimit = -(int32_t)A_BK_CURRENTSEEDING * (int32_t)IMAX_KIDIV;
    
    pPIDImHandle->hLowerOutputLimit = -brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16;
    pPIDImHandle->hUpperOutputLimit = brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16;
    pPIDImHandle->wUpperIntegralLimit = (int32_t)brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16 * (int32_t)IMAX_KIDIV;
    pPIDImHandle->wLowerIntegralLimit = -(int32_t)brakingParameters_Control.brakingParameters_Settings.brakeCurrentSeeding_u16 * (int32_t)IMAX_KIDIV;
    
    
    FOCStop_CalcCurrRef(pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);
    break;
    
  case CURRENT_RAMPUP: //ramp current up to a point when current >= the computed maximum current based on the copper loss ellipse
    if (pBkHandle->IMax_Ref >= pBkHandle->Adapt_IMax)
    {
      // Always compute the I-maximum reference to be within the copper loss ellipse
      pBkHandle->IMax_Ref = pBkHandle->Adapt_IMax;
      pBkHandle->BrakingPhase = CURRENT_STEADYSTATE;
    }
    else
    {
      //pBkHandle->IMax_Ref += A_RAMP_STEP;
      pBkHandle->IMax_Ref += brakingParameters_Control.brakingParameters_Settings.brakeCurrentRampStep_u16;
    }
    //RPa: can be placed the next lines into a function as this is common with CURRENT_STEADYSTATE
    //if (MeasuredSpeedAbsValue < A_SPEED_TRANSITION)
    if (MeasuredSpeedAbsValue < brakingParameters_Control.brakingParameters_Settings.brakeSpeedTranstionToBrake_u16)
    {
      pBkHandle->BrakingPhase = CURRENT_RAMPDOWN;
      pPIDImHandle->hLowerOutputLimit = -5;
      pPIDImHandle->hUpperOutputLimit = 5;
    }
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef(pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);
    break;
    
  case CURRENT_STEADYSTATE: //I-maximum reference is based on the computed adapted maximum current through motor loss ellipse
    pBkHandle->IMax_Ref = pBkHandle->Adapt_IMax;
    
    //RPa: can be placed the next lines into a function as this is common with CURRENT_RAMPUP
    //if (MeasuredSpeedAbsValue < A_SPEED_TRANSITION)
    if (MeasuredSpeedAbsValue < brakingParameters_Control.brakingParameters_Settings.brakeSpeedTranstionToBrake_u16)
    {
      pBkHandle->BrakingPhase = CURRENT_RAMPDOWN;
      // Limit the boundaries of the Id injection when ramping down
      pPIDImHandle->hLowerOutputLimit = -5;
      pPIDImHandle->hUpperOutputLimit = 5;
    }
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef(pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);
    break;
    
  case CURRENT_RAMPDOWN:                          //ramp current down state when speed is very low
    //if (pBkHandle->IMax_Ref <= A_RAMPEND_CURRENT) // go to Iq Hold state when Id injection is at the end of a specified minimum
    if (pBkHandle->IMax_Ref <= brakingParameters_Control.brakingParameters_Settings.brakingRampEndCurrent_u16) // go to Iq Hold state when Id injection is at the end of a specified minimum
    {
      pBkHandle->FilteredSpeed = MeasuredSpeedAbsValue;
      
      //pBkHandle->IMax_Ref = A_RAMPEND_CURRENT;
      pBkHandle->IMax_Ref = brakingParameters_Control.brakingParameters_Settings.brakingRampEndCurrent_u16;
      pBkHandle->BrakingPhase = LOWSPEED_IQHOLD;
    }
    else
    {
      //pBkHandle->IMax_Ref -= A_RAMP_STEP;
      pBkHandle->IMax_Ref -= brakingParameters_Control.brakingParameters_Settings.brakeCurrentRampStep_u16;
    }
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef(pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);
    break;
    
  case LOWSPEED_IQHOLD: // short coils by turning on the low-sides
    //RPa: Low-Pass Filtering of Speed measurement
    pBkHandle->FilteredSpeed = (int16_t)((((256 - (int32_t)alpha_br) * (int32_t)MeasuredSpeedAbsValue) + ((int32_t)alpha_br * (int32_t)pBkHandle->FilteredSpeed) + 128) >> BYTE_SHIFT);
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef(pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);
    
    //if ((pBkHandle->FilteredSpeed < A_BK_ENDSPEED) && (OTFHandle_M1.hdir != WM_HIGH_REVERSE) && (OTFHandle_M1.hdir != WM_LOW_FORWARD)) //When the filtered speed is lower than a given very low speed, short the coils
    if ((pBkHandle->FilteredSpeed < brakingParameters_Control.brakingParameters_Settings.brakeEndSpeed_u16) && 
        (OTFHandle_M1.hdir != WM_HIGH_REVERSE) && (OTFHandle_M1.hdir != WM_LOW_FORWARD)) //When the filtered speed is lower than a given very low speed, short the coils
    {
      LowSide_count = 0;
      
      R3_1_SwitchOffPWM(&PWM_Handle_M1._Super);
      R3_1_TurnOnLowSides(&PWM_Handle_M1._Super); // Turning the low-side on when speed is lower than a specified threshold
      
      pBkHandle->BrakingPhase = TURNONLOWSIDE;
    }
    //else if ((pBkHandle->FilteredSpeed < (A_WM_REVERSE_ENDSPEED)) && ((OTFHandle_M1.hdir == WM_HIGH_REVERSE) || (OTFHandle_M1.hdir == WM_LOW_FORWARD)))
    else if ((pBkHandle->FilteredSpeed < (otfParameters_Control.otfParameters_Settings.wmReverseEndSpeed_u16)) && 
             ((OTFHandle_M1.hdir == WM_HIGH_REVERSE) || (OTFHandle_M1.hdir == WM_LOW_FORWARD)))
    {
      LowSide_count = 0;
      R3_1_SwitchOffPWM(&PWM_Handle_M1._Super);
      R3_1_TurnOnLowSides(&PWM_Handle_M1._Super); // Turning the low-side on when speed is lower than a specified threshold
      pBkHandle->BrakingPhase = LOWSPEED_OL_IQRAMPDOWN;
    }
    break;
    
  case LOWSPEED_OL_IQRAMPDOWN: // at the very low speed for high inertia system and low Kt motor, bemf signal is severely affected by noise, and so IQ is ramped down in open-loop fashion before turning low-sides ON
    
    //if (LowSide_count > A_WM_SHORTCOIL_DURATION)
    if (LowSide_count > otfParameters_Control.otfParameters_Settings.wmCoilShortDuration_u16)
    {
      LowSide_count = 0;
      pBkHandle->BrakingPhase = TURNONLOWSIDE;
    }
    LowSide_count++;
    break;
    
  case TURNONLOWSIDE: // low-side is turned on at the end of controlled braking to put motor to stationary
    
    //if ((LowSide_count > A_BK_LOWSIDE_DURATION) || (A_BK_LOWSIDE_DURATION == INDEFINITE_LOWSIDE) || (OTFHandle_M1.hdir == WM_HIGH_REVERSE) || (OTFHandle_M1.hdir == WM_LOW_FORWARD)) //Low-side turned on for a specified amount of time; this can be tuned later on for the application (or not even turned off at all)
    if ((LowSide_count > brakingParameters_Control.brakingParameters_Settings.lowSideOnDuration_u16) || 
        (brakingParameters_Control.brakingParameters_Settings.lowSideOnDuration_u16 == INDEFINITE_LOWSIDE) || 
          (OTFHandle_M1.hdir == WM_HIGH_REVERSE) || (OTFHandle_M1.hdir == WM_LOW_FORWARD)) //Low-side turned on for a specified amount of time; this can be tuned later on for the application (or not even turned off at all)
    {
      pBkHandle->BrakingPhase = CURRENT_STARTRAMP; // controlled braking is complete
      ImCtrl_en = 1;                               // enable the copper loss control after the first low-sides are turned ON
    }
    LowSide_count++;
    break;
    
  default:
    
    break;
  }
}

/**
* @brief Current Reference calculation for non-regenerative braking
*/
void FOCStop_CalcCurrRef(Braking_Handle_t *pBrakeHandle, PID_Handle_t *pPIDBusHandle, PID_Handle_t *pPIDImHandle, FOCVars_t *pFOCHandle_t, RDivider_Handle_t *pBSHandle)
{
  if (pFOCHandle_t->bDriveInput == INTERNAL)
  {
    pFOCHandle_t->Iqdref.q = FOC_BusVoltageControlM1(pBrakeHandle, pPIDBusHandle, pBSHandle); //calculate q-current reference
    if (ImCtrl_en > 0)                                                                        // calculate d-current reference through copper loss controller when flag is above zero
      pFOCHandle_t->Iqdref.d = FOC_ImaxCurrentControllerM1(pBrakeHandle, pPIDImHandle, pFOCHandle_t);
    else
      pFOCHandle_t->Iqdref.d = 0;
  }
}

/**
* @brief non-regenerative braking when motor is at running state
*/
void RegenControlM1(Braking_Handle_t *pBkHandle, PID_Handle_t *pPIDBusHandle, PID_Handle_t *pPIDSpeedHandle, SpeednTorqCtrl_Handle_t *pSTCHandle, RDivider_Handle_t *pBSHandle)
{
  //Only do the regen control when there is an error in speed, otherwise skip computation
  if ((MC_GetImposedDirectionMotor1() == 1) && ((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) > 1))
  {                                                                                                         //FWD direction
    pPIDSpeedHandle->hLowerOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle);      //update lower limit when direction is forward
    pPIDSpeedHandle->wLowerIntegralLimit = (int32_t)pPIDSpeedHandle->hLowerOutputLimit * (int32_t)SP_KIDIV; //update lower integral limit when direction is forward
  }
  else if ((MC_GetImposedDirectionMotor1() == -1) && ((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) < -1))
  {                                                                                                         //REV direction
    pPIDSpeedHandle->hUpperOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle);      // update upper limit when direction is reverse
    pPIDSpeedHandle->wUpperIntegralLimit = (int32_t)pPIDSpeedHandle->hUpperOutputLimit * (int32_t)SP_KIDIV; //update  upper integral limit when direction is reverse
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief  Main revup controller procedure executing overall programmed phases and
*         on-the-fly startup handling.
*/
__weak bool Regal_OTF_Exec(RevUpCtrl_Handle_t *pHandle, OTF_Handle_t *pOTFHandle)
{
  bool IsSpeedReliable;
  bool retVal = true;
  bool condition = false;
  
  if (pHandle->hPhaseRemainingTicks > 0u) // do OTF when remaining ticks is greater than zero
  {
    /* Decrease the hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks--;
    
    /* OTF start-up */
    if (pHandle->bStageCnt == 0u) // First Phase: DETECTION
    {
      if (pHandle->EnteredZone1 == false)
      {
        if (pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL)
        {
          pHandle->bResetPLLCnt++;
          if (pHandle->bResetPLLCnt > pHandle->bResetPLLTh)
          {
            pHandle->pSNSL->pFctStoOtfResetPLL(pHandle->pSNSL);
            pHandle->bOTFRelCounter = 0u;
            pHandle->bResetPLLCnt = 0u;
          }
        }
        
        IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck(pHandle->pSNSL);
        
        if (IsSpeedReliable) // Check if speed readings are within variance
        {
          if (pHandle->bOTFRelCounter < 127u)
            pHandle->bOTFRelCounter++;
        }
        else
        {
          pHandle->bOTFRelCounter = 0u;
        }
        
        if (pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL)
        {
          if (pHandle->bOTFRelCounter == (pHandle->bResetPLLTh >> 1))
            condition = true;
        }
        else
        {
          if (pHandle->bOTFRelCounter == 127)
            condition = true;
        }
        
        if (pOTFHandle->hdir == WM_CURRENT_CLAMP)
        {
          pHandle->hPhaseRemainingTicks = 0u;
          pOTFHandle->seamless_transfer = 1;
        }
        
        if (condition == true) // if speed is reliable, then check for OTF condition and synchronization
        {
          bool bCollinearSpeed = false;
          int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit(pHandle->pSNSL->_Super);
          int16_t hObsSpeedUnitAbsValue =
            (hObsSpeedUnit < 0 ? (-hObsSpeedUnit) : (hObsSpeedUnit)); /* hObsSpeedUnit absolute value */
          
          if (pHandle->hDirection > 0) //FWD direction command
          {
            if (hObsSpeedUnit > 0)
              bCollinearSpeed = true; /* actual and reference speed are collinear*/
          }
          else //REV direction
          {
            if (hObsSpeedUnit < 0)
              bCollinearSpeed = true; /* actual and reference speed are collinear*/
          }
          
          if (bCollinearSpeed == false) //speed direction and direction command are not the same
          {
            /*reverse speed management*/
            pHandle->bOTFRelCounter = 0u;
            //if (((uint16_t)(hObsSpeedUnitAbsValue) > A_WM_REVERSE_ENDSPEED) && ((uint16_t)(hObsSpeedUnitAbsValue) < A_WM_MAX_REVERSE_SPEED)) // if speed is higher than a pre-defined value
            if (((uint16_t)(hObsSpeedUnitAbsValue) > otfParameters_Control.otfParameters_Settings.wmReverseEndSpeed_u16) && 
                ((uint16_t)(hObsSpeedUnitAbsValue) < otfParameters_Control.otfParameters_Settings.wmMaxReverseSpeed_u16)) // if speed is higher than a pre-defined value
            {                                                                                                                                //go to synchronisation phase
              pOTFHandle->hdir = WM_HIGH_REVERSE;
              /* startup end, go to run */
              pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
              pHandle->EnteredZone1 = true;
              pOTFHandle->seamless_transfer = 1;
              pHandle->hDirection *= (int8_t)pOTFHandle->hdir;
            }
            //else if ((uint16_t)(hObsSpeedUnitAbsValue) >= A_WM_MAX_REVERSE_SPEED)
            else if ((uint16_t)(hObsSpeedUnitAbsValue) >= otfParameters_Control.otfParameters_Settings.wmMaxReverseSpeed_u16)
            {
              retVal = false;
            }
            else
            {
              pOTFHandle->hdir = WM_VERYLOWSPEED; /* this case when speed is lower than the hMinStartUpFlySpeed */
            }
            Spd_Ph0_End = (int16_t)(((int32_t)OTF_PHASE1_FACTOR * (int32_t)hObsSpeedUnitAbsValue) >> BYTE_SHIFT);
            OTF_Ph1_count = 0;
          }
          else /*speeds are collinear*/
          {
            if ((uint16_t)(hObsSpeedUnitAbsValue) > pHandle->hMinStartUpFlySpeed) // if speed is higher than a pre-defined value
            {                                                                     //go to synchronisation phase
              pHandle->hPhaseRemainingTicks = 0u;
              pOTFHandle->seamless_transfer = 1;
              pHandle->bStageCnt = 1u;
              pOTFHandle->hdir = WM_HIGH_FORWARD;
            }                                                                                                                                                 /* speed > MinStartupFly */
            else if (((uint16_t)(hObsSpeedUnitAbsValue) > pOTFHandle->CoilShortSpeed) && ((uint16_t)(hObsSpeedUnitAbsValue) <= pHandle->hMinStartUpFlySpeed)) // if speed is higher than a pre-defined value
            {                                                                                                                                                 //go to synchronisation phase
              pOTFHandle->hdir = WM_LOW_FORWARD;
              /* startup end, go to run */
              pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
              pHandle->EnteredZone1 = true;
              pOTFHandle->seamless_transfer = 1;
            }
            else
            {
              pOTFHandle->hdir = WM_VERYLOWSPEED; /* this case when speed is lower than the hMinStartUpFlySpeed */
            }
            Spd_Ph0_End = (int16_t)(((int32_t)OTF_PHASE1_FACTOR * (int32_t)hObsSpeedUnitAbsValue) >> BYTE_SHIFT);
            OTF_Ph1_count = 0;
          } /* speeds are collinear */
        }   /* speed is reliable */
      }     /*EnteredZone1 1 is false */
      else
      {
        pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
      }
    } /*stage 0: detection stage*/
    
    if (pHandle->bStageCnt == 1u) //SYNCHRONIZATION PHASE
    {
      int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit(pHandle->pSNSL->_Super);
      int16_t hObsSpeedUnitAbsValue =
        (hObsSpeedUnit < 0 ? (-hObsSpeedUnit) : (hObsSpeedUnit)); /* hObsSpeedUnit absolute value */
      
      if ((pOTFHandle->hdir == WM_HIGH_REVERSE) || (pOTFHandle->hdir == WM_HIGH_FORWARD) || (pOTFHandle->hdir == WM_LOW_FORWARD)) /* handling of forward and reverse direction*/
      {
        if (hObsSpeedUnitAbsValue > pOTFHandle->MaxSyncSpeed)
        {
          if (hObsSpeedUnitAbsValue >= Spd_Ph0_End) //check if speed is getting higher due to estimate instability
          {
            OTF_Ph1_count++;
          }
          if (OTF_Ph1_count >= OTF_PHASE1_TIMEOUT)
          {
            retVal = false;
          } /* throw a start-up failure fault */
          pHandle->hPhaseRemainingTicks++;
        }
        else if ((hObsSpeedUnitAbsValue <= pOTFHandle->MaxSyncSpeed) && (hObsSpeedUnitAbsValue >= pOTFHandle->MinSyncSpeed))
        { //When speed is within range, go straight to closed-loop speed control
          VSS_SetCopyObserver(pHandle->pVSS);
          pHandle->pSNSL->pFctForceConvergency2(pHandle->pSNSL);
          
          STC_ExecRamp(pHandle->pSTC, pHandle->hDirection * pOTFHandle->hSyncTRef, 0u);
          
          pHandle->hPhaseRemainingTicks = 1u;
          
          pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;
          // TODO: add flag for OTF count on data logging
          pHandle->bStageCnt = 6u;
        }
        else if (hObsSpeedUnitAbsValue < pOTFHandle->CoilShortSpeed)
        { //Turn ON the low-sides if speed is really low (below 60RPM), but this has to be re-visited
          if (pHandle->OTFSCLowside == false)
          {
            PWMC_SwitchOffPWM(pHandle->pPWM);
            pHandle->OTFSCLowside = true;
            PWMC_TurnOnLowSides(pHandle->pPWM);
          }                                                                                     /* check of coil shorting */
        }                                                                                       /* check of absolute speed */
      }                                                                                         /*hdir check for forward and reversal*/
      else if ((pOTFHandle->hdir == WM_VERYLOWSPEED) || (pOTFHandle->hdir == WM_CURRENT_CLAMP)) /* when speed is too low */
      {
      }
      else
      {
        if ((pHandle->hPhaseRemainingTicks < 10) && (hObsSpeedUnitAbsValue > pOTFHandle->MaxSyncSpeed))
        {
          retVal = false;
        } /* throw a start-up failure fault */
      }   /* hdir check for unstable speed reading*/
    }     /*stage 1: synchronization phase*/
    
  } /* hPhaseRemainingTicks > 0 */
  
  // if tick count is zero, do the necessary initialisation for the next phase
  if (pHandle->hPhaseRemainingTicks == 0u)
  {
    if (pHandle->pCurrentPhaseParams != MC_NULL)
    {
      if (pHandle->bStageCnt == 0u)
      {
        /*end of OTF*/
        pHandle->bOTFRelCounter = 0u;
        if (pOTFHandle->hdir == WM_CURRENT_CLAMP)
        {
          PWMC_SwitchOnPWM(pHandle->pPWM);
          pHandle->OTFSCLowside = false;
          //pHandle->pSTC->MaxPositiveTorque = (int16_t)WM_MAX_CLAMP_CURRENT;
          //pHandle->pSTC->MinNegativeTorque = -(int16_t)WM_MAX_CLAMP_CURRENT;
          //pHandle->ParamsData[1].hDurationms = A_WM_CLAMP_RAMP;
          //pHandle->ParamsData[1].hFinalMecSpeedUnit = 0;
          //pHandle->ParamsData[1].hFinalTorque = (int16_t)A_WM_CLAMP_CURRENT;
          
          pHandle->pSTC->MaxPositiveTorque = (int16_t)Regal_ConvertmAToCounts(otfParameters_Control.otfParameters_Settings.wmMaxClampCurrent_u16);
          pHandle->pSTC->MinNegativeTorque = -(int16_t)Regal_ConvertmAToCounts(otfParameters_Control.otfParameters_Settings.wmMaxClampCurrent_u16);
          pHandle->ParamsData[1].hDurationms = otfParameters_Control.otfParameters_Settings.wmClampRamp_u16;
          pHandle->ParamsData[1].hFinalMecSpeedUnit = 0;
          pHandle->ParamsData[1].hFinalTorque = (int16_t)Regal_ConvertmAToCounts(otfParameters_Control.otfParameters_Settings.wmClampCurrent_u16);
        }
        else if ((pOTFHandle->hdir == WM_VERYLOWSPEED) || (pOTFHandle->hdir == WM_RESET))
        {
          if (pHandle->OTFSCLowside == false)
          {
            PWMC_SwitchOffPWM(pHandle->pPWM);
            pHandle->OTFSCLowside = true;
            PWMC_TurnOnLowSides(pHandle->pPWM);
          } /* check of coil shorting */
        }
      }
      else if ((pHandle->bStageCnt == 1u))
      {
        PWMC_SwitchOnPWM(pHandle->pPWM);
        pHandle->OTFSCLowside = false;
        if (pOTFHandle->hdir == WM_CURRENT_CLAMP)
        {
          //pHandle->ParamsData[1].hDurationms = (uint16_t)A_PHASE2_DURATION;
          //pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(A_PHASE2_FINAL_SPEED_UNIT);
          // pHandle->ParamsData[1].hFinalTorque = (int16_t)A_PHASE2_FINAL_CURRENT;
          //pHandle->ParamsData[2].hDurationms = (uint16_t)A_WM_CLAMP_DURATION;
          //pHandle->ParamsData[2].hFinalTorque = (int16_t)A_WM_CLAMP_CURRENT;
          
          pHandle->ParamsData[1].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.startupBrakingDuration_u16;
          pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(stParameters01_Control.stParameters01_Settings.phase2Speed_u16 * SPEED_UNIT / _RPM);
          pHandle->ParamsData[1].hFinalTorque = (int16_t)stParameters01_Control.stParameters01_Settings.phase2FinalCurrent_u16;
          pHandle->ParamsData[2].hDurationms = (uint16_t)otfParameters_Control.otfParameters_Settings.wmClampDuration_u16;
          pHandle->ParamsData[2].hFinalTorque = (int16_t)otfParameters_Control.otfParameters_Settings.wmClampCurrent_u16;
        }
      }
      else if ((pHandle->bStageCnt == 2u) && (pOTFHandle->hdir == WM_CURRENT_CLAMP))
      {
        //pHandle->ParamsData[2].hDurationms = (uint16_t)A_PHASE3_DURATION;
        //pHandle->ParamsData[2].hFinalTorque = (int16_t)A_PHASE3_FINAL_CURRENT;
        //pHandle->ParamsData[3].hFinalTorque = (int16_t)A_WM_CLAMP_CURRENT;
        
        pHandle->ParamsData[2].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime01_u16;
        pHandle->ParamsData[2].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.alignmentCurrent01_u16;
        pHandle->ParamsData[3].hFinalTorque = (int16_t)otfParameters_Control.otfParameters_Settings.wmClampCurrent_u16;
      }
      else if ((pHandle->bStageCnt == 3u) && (pOTFHandle->hdir == WM_CURRENT_CLAMP))
      {
        //pHandle->ParamsData[3].hFinalTorque = (int16_t)A_PHASE4_FINAL_CURRENT;
        //pHandle->ParamsData[4].hFinalTorque = (int16_t)A_PHASE5_FINAL_CURRENT;
        
        pHandle->ParamsData[3].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16;
        pHandle->ParamsData[4].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.startRampCurrent_u16;
      }
      else if ((pHandle->bStageCnt == 4u) && (pOTFHandle->hdir == WM_CURRENT_CLAMP))
      {
        //pHandle->ParamsData[4].hFinalTorque = (int16_t)A_PHASE5_FINAL_CURRENT;
        //pHandle->pSTC->MaxPositiveTorque = (int16_t)A_NOMINAL_CURRENT;
        //pHandle->pSTC->MinNegativeTorque = -(int16_t)A_NOMINAL_CURRENT;
        
        pHandle->ParamsData[4].hFinalTorque = (int16_t)startupParameters_Control.startupParameters_Settings.startRampCurrent_u16;
        pHandle->pSTC->MaxPositiveTorque = (int16_t)Regal_ConvertmAToCounts(motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16);
        pHandle->pSTC->MinNegativeTorque = -(int16_t)Regal_ConvertmAToCounts(motorParameters_Control.motorParameters_Settings.maxMotorCurrent_u16);
      }
      else
      {
      }
      
      /* If it becomes zero the current phase has been completed.*/
      /* Gives the next command to STC and VSS.*/
      STC_ExecRamp(pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                   (uint32_t)(pHandle->pCurrentPhaseParams->hDurationms));
      
      VSS_SetMecAcceleration(pHandle->pVSS,
                             pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                             pHandle->pCurrentPhaseParams->hDurationms);
      
      /* Compute hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks =
        (uint16_t)(((uint32_t)pHandle->pCurrentPhaseParams->hDurationms *
                    (uint32_t)pHandle->hRUCFrequencyHz) /
                   1000u);
      pHandle->hPhaseRemainingTicks++;
      
      /*Set the next phases parameter pointer.*/
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;
      
      /*Increases the rev up stages counter.*/
      pHandle->bStageCnt++;
    }
    else
    {
      if (pHandle->bStageCnt == pHandle->bPhaseNbr - 1) /* End of user programmed revup */
      {
        retVal = false;
      }
      else if (pHandle->bStageCnt == 7u) /* End of first OTF runs */
      {
        pHandle->bStageCnt = 0u; /* Breaking state */
        pHandle->hPhaseRemainingTicks = 0u;
      }
      else
      {
      }
    }
  }
  return retVal;
}

/////////////////////////////
void Burnin_InitProfile(Burnin_Handle_t *pBurninHandle, RevUpCtrl_Handle_t *pHandle)
{
  //if (pBurninHandle->en_ctrl == ENABLE)
  if((moduleTest_Control.moduleTest_Data.adminDiscretes01.is_burninMode == TRUE) && (pBurninHandle->en_ctrl == TRUE) )
  {
    pHandle->ParamsData[0].hDurationms = 1000;
    pHandle->ParamsData[1].hDurationms = 1000;
    pHandle->ParamsData[1].hFinalMecSpeedUnit = pBurninHandle->frequency;
    pHandle->ParamsData[1].hFinalTorque = pBurninHandle->current;
    pHandle->ParamsData[0].hFinalTorque = pBurninHandle->current;
   
    pHandle->hMinStartUpValidSpeed = MAX_BURNIN_FREQUENCY;
    pHandle->hMinStartUpFlySpeed = MAX_BURNIN_FREQUENCY;
    STO_PLL_M1.MinStartUpValidSpeed = MAX_BURNIN_FREQUENCY;
  }
  else
  {
    //if (A_REGAL_OTF == 1)
    if (otfParameters_Control.otfParameters_Settings.adminFlags01.is_otfControlEnable == TRUE)
    {
      //pHandle->ParamsData[0].hDurationms = (uint16_t)A_PHASE1_DURATION;
      //pHandle->ParamsData[1].hDurationms = (uint16_t)A_PHASE2_DURATION;
      //pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(A_PHASE2_FINAL_SPEED_UNIT);
      //pHandle->ParamsData[1].hFinalTorque = A_PHASE2_FINAL_CURRENT;
      //pHandle->ParamsData[0].hFinalTorque = A_PHASE1_FINAL_CURRENT;
      
      pHandle->ParamsData[0].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.detectionDuration_u16;
      pHandle->ParamsData[1].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.startupBrakingDuration_u16;
      pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(stParameters01_Control.stParameters01_Settings.phase2Speed_u16 * SPEED_UNIT / _RPM); // 0
      pHandle->ParamsData[1].hFinalTorque = stParameters01_Control.stParameters01_Settings.phase1FinalCurrent_u16; // 0
      pHandle->ParamsData[0].hFinalTorque = stParameters01_Control.stParameters01_Settings.phase2FinalCurrent_u16; // 0
    }
    else
    {
      //pHandle->ParamsData[0].hDurationms = (uint16_t)A_PHASE3_DURATION;
      //pHandle->ParamsData[1].hDurationms = (uint16_t)A_PHASE4_DURATION;
      //pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(A_PHASE4_FINAL_SPEED_UNIT);
      //pHandle->ParamsData[1].hFinalTorque = A_PHASE4_FINAL_CURRENT;
      //pHandle->ParamsData[0].hFinalTorque = A_PHASE3_FINAL_CURRENT;
      
      pHandle->ParamsData[0].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime01_u16;
      pHandle->ParamsData[1].hDurationms = (uint16_t)startupParameters_Control.startupParameters_Settings.alignmentTime02_u16;
      pHandle->ParamsData[1].hFinalMecSpeedUnit = (int16_t)(startupParameters_Control.startupParameters_Settings.alignmentSpeed02_u16 * SPEED_UNIT / _RPM);
      pHandle->ParamsData[1].hFinalTorque = startupParameters_Control.startupParameters_Settings.alignmentCurrent02_u16;
      pHandle->ParamsData[0].hFinalTorque = startupParameters_Control.startupParameters_Settings.alignmentCurrent01_u16;
    }
    //pHandle->hMinStartUpValidSpeed = (uint16_t)((A_OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / _RPM);
    //pHandle->hMinStartUpFlySpeed = (int16_t)(((uint16_t)((A_OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / _RPM)) / 2);
    //STO_PLL_M1.MinStartUpValidSpeed = (uint16_t)((A_OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / _RPM);
    
    pHandle->hMinStartUpValidSpeed = (uint16_t)((motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16 * SPEED_UNIT) / _RPM);
    pHandle->hMinStartUpFlySpeed = (int16_t)(((uint16_t)((motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16 * SPEED_UNIT) / _RPM)) / 2);
    STO_PLL_M1.MinStartUpValidSpeed = (uint16_t)((motorTunning02_Control.motorTunning02_Settings.observerMinRpm_u16 * SPEED_UNIT) / _RPM);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Power_Clear(Power_Handle_t *pHandle)
{
  uint16_t i;
  for (i = 0u; i < POWER_BUFFER_SIZE; i++)
  {
    pHandle->hMeasBuffer[i] = 0;
  }
  pHandle->hNextMeasBufferIndex = 0u;
  pHandle->hLastMeasBufferIndex = 0u;
  pHandle->hAvrgMotorPowerW = 0u;
}

void Regal_CalcElecMotorPower(Power_Handle_t *pHandle, RDivider_Handle_t *pBSHandle, FOCVars_t *pFOCHandle)
{
  uint16_t i;
  int32_t wAux;
  int64_t wAux2;
  int32_t wAux2_v;
  int32_t CurrentMotorPower;
  int32_t wAux3 = 0;
  qd_t Iqd = pFOCHandle->Iqd;
  qd_t Vqd = pFOCHandle->Vqd;
  
  wAux = ((int32_t)Iqd.q * (int32_t)Vqd.q) +
    ((int32_t)Iqd.d * (int32_t)Vqd.d);
  wAux /= 65536;
  
  wAux2 = (int64_t)pHandle->wConvFact * (int64_t)VBS_GetAvBusVoltage_V(&(pBSHandle->_Super));
  wAux2_v = (int32_t)(wAux2 / 1000); /* 1000 is max bus voltage expressed in volt.*/
  
  CurrentMotorPower = (int32_t)((int64_t)wAux * (int64_t)wAux2_v);
  //  CurrentMotorPower *= 9; /* 9 is max bus voltage expressed in thousend of volt.*/
  //  CurrentMotorPower /= 10;
  CurrentMotorPower /= 65536;
  
  /* Store the measured values in the buffer.*/
  pHandle->hMeasBuffer[pHandle->hNextMeasBufferIndex] = (int16_t)CurrentMotorPower;
  pHandle->hLastMeasBufferIndex = pHandle->hNextMeasBufferIndex;
  pHandle->hNextMeasBufferIndex++;
  if (pHandle->hNextMeasBufferIndex >= POWER_BUFFER_SIZE)
  {
    pHandle->hNextMeasBufferIndex = 0u;
  }
  /* Compute the average measured motor power */
  for (i = 0u; i < POWER_BUFFER_SIZE; i++)
  {
    wAux3 += (int32_t)(pHandle->hMeasBuffer[i]);
  }
  wAux3 /= (int32_t)POWER_BUFFER_SIZE;
  pHandle->hAvrgMotorPowerW = (int16_t)(wAux3);
  /* Return the last measured motor power */
}

int16_t Regal_GetAvrgMotorPowerW(Power_Handle_t *pHandle)
{
  return (pHandle->hAvrgMotorPowerW);
}

uint16_t Regal_ConvertmAToCounts(uint16_t milli_amps_u16)
{
  // Convert current in mA peak to counts
  uint16_t current_count_u16 = 0;
  current_count_u16 = (uint16_t)((MA_TO_COUNTS_CONVERSION * (uint32_t)milli_amps_u16) >> MA_TO_COUNTS_PRECISION);
  return (current_count_u16);
}

//int16_t Regal_ConvertCountsTomA(int16_t counts_s16)
//{
//  int16_t current_mA_s16 = 0;
//  current_mA_s16 = (int16_t)(((int64_t)COUNTS_TO_MA_CONVERSION * (int64_t)counts_s16) >> COUNTS_TO_MA_PRECISION);
//  return (current_mA_s16);
//}

uint16_t Regal_ConvertCountsTomA(uint16_t counts_u16)
{
  uint16_t current_mA_u16 = 0;
  current_mA_u16 = (uint16_t)((COUNTS_TO_MA_CONVERSION * (uint32_t)counts_u16)>> COUNTS_TO_MA_PRECISION );
  return (current_mA_u16);
}

/* Phase Curent averaging used for the front end */
uint8_t AvrPhCurrentIndx = 0;
void updateAvrPhCurrent(FOCVars_t * pFOCHandle_t){            //average current for fault monitor 
  avrPhACurrentRd[AvrPhCurrentIndx] = (uint16_t) (pFOCHandle_t->Iab.a < 0 ? (-pFOCHandle_t->Iab.a) : (pFOCHandle_t->Iab.a));
  avrPhBCurrentRd[AvrPhCurrentIndx] = (uint16_t) (pFOCHandle_t->Iab.b < 0 ? (-pFOCHandle_t->Iab.b) : (pFOCHandle_t->Iab.b));
  AvrPhCurrentIndx++;
  if(AvrPhCurrentIndx >= PHASE_CURRENT_BUFFER_SIZE) {
    avrPhACurrentRdOP = 0;
    avrPhBCurrentRdOP = 0;
    for (uint8_t j = 0; j < PHASE_CURRENT_BUFFER_SIZE; j++)
    {
      avrPhACurrentRdOP += avrPhACurrentRd[j];
      avrPhBCurrentRdOP += avrPhBCurrentRd[j];
    }
    avrPhACurrentRdOP = avrPhACurrentRdOP/PHASE_CURRENT_BUFFER_SIZE;
    avrPhBCurrentRdOP = avrPhBCurrentRdOP/PHASE_CURRENT_BUFFER_SIZE;
    AvrPhCurrentIndx = 0;
  }  
}
/* Wrappers for the phase currents*/
uint16_t Regal_GetAvrgMotorCurrentPhaseA(void) {
  return (avrPhACurrentRdOP);
}
uint16_t Regal_GetAvrgMotorCurrentPhaseB(void) {
  return (avrPhBCurrentRdOP);
}

void Regal_ClearPhaseCurrents(void)
{
  avrPhACurrentRdOP = 0;
  avrPhBCurrentRdOP = 0;
}

/* Current Averaging used for derating */
uint8_t AvrCurrentIndx = 0;
void updateAvrCurrent(FOCVars_t * pFOCHandle_t)
{ //average current for fault monitor 
  avrCurrentRd[AvrCurrentIndx] = (pFOCHandle_t->Iqd.q < 0 ? (-pFOCHandle_t->Iqd.q) : (pFOCHandle_t->Iqd.q));
  AvrCurrentIndx++;
  if(AvrCurrentIndx >= CURRENT_DERATING_BUFFER_SIZE) {
    avrCurrentRdOP = 0;
    for (uint8_t j = 0; j < CURRENT_DERATING_BUFFER_SIZE; j++)
    {
      avrCurrentRdOP += avrCurrentRd[j];
    }
    avrCurrentRdOP = avrCurrentRdOP/CURRENT_DERATING_BUFFER_SIZE;
    AvrCurrentIndx = 0;
  }
}

/**
  * @brief  Checks for lost phase by evaluating data set and counting transitions
  *         between -std deviation and +std deviation of the data set.
  * @param  pHandle related RDivider_Handle_t
  * @retval 1 if lost phase confirmed, 0 if 3 phase detected or not checked 
  *         becuase of low power or not enabled
  */
#define PHASE_CHECK_DEBOUNCE 40
uint8_t Regal_CheckForPhaseLoss( RDivider_Handle_t * pHandle ){     
  //find the upper/lower control limit
  static int16_t failure_count_s16 = 0;
  uint8_t phase_loss_confirmed_u8 = 0;
  uint32_t vbus_average_u32 = 0;
  uint32_t sum_of_the_squares_u32 = 0;
  for ( uint8_t index_u8 = 0; index_u8 < M1_VBUS_SW_FILTER_BW_FACTOR; index_u8 ++ ){
    vbus_average_u32 += pHandle->aBuffer[index_u8];
  }
  vbus_average_u32 /= M1_VBUS_SW_FILTER_BW_FACTOR;
  for ( uint8_t index_u8 = 0; index_u8 < M1_VBUS_SW_FILTER_BW_FACTOR; index_u8 ++ ){
    sum_of_the_squares_u32 += (pHandle->aBuffer[index_u8] - vbus_average_u32) * (pHandle->aBuffer[index_u8] - vbus_average_u32);
  }    
  int32_t radicand_s32 = sum_of_the_squares_u32 / M1_VBUS_SW_FILTER_BW_FACTOR;
  uint16_t three_fourths_std_deviation_u16 = (uint16_t)MCM_Sqrt(radicand_s32) >>1 + (uint16_t)MCM_Sqrt(radicand_s32) >>2;
  uint16_t upper_control_limit_u16 = (uint16_t)vbus_average_u32 + three_fourths_std_deviation_u16; 
  uint16_t lower_control_limit_u16 = (uint16_t)vbus_average_u32 - three_fourths_std_deviation_u16; 
 //determine how many times the data has transitioned between control limits
  int8_t transition_track_s8 = 0;
  uint16_t ripple_count_u16 = 0;
  for ( uint8_t index_u8 = 0; index_u8 < M1_VBUS_SW_FILTER_BW_FACTOR; index_u8 ++ ){
    if ( pHandle->aBuffer[index_u8] > upper_control_limit_u16 && transition_track_s8 <= 0 ){
      transition_track_s8 = 1;
      ripple_count_u16 ++;
    }
    if ( pHandle->aBuffer[index_u8] < lower_control_limit_u16 && transition_track_s8 >= 0 ){
      transition_track_s8 = -1;
      ripple_count_u16 ++;
    }
  }  

  //will be 3 peaks for no phase loss and 1 for phase loss, only evaluate data if 
  //the final numbers of transitions 6 and 2 repectivly
  if ( ripple_count_u16 < 7 && ripple_count_u16 > 0){
    //good ripple count so check for phase loss     
    if ( ripple_count_u16 < 4 ) {
      if (++failure_count_s16 > PHASE_CHECK_DEBOUNCE){
       failure_count_s16 = 0;
       phase_loss_confirmed_u8 = 1; 
      }
    } else {
      if ( ripple_count_u16 > 4 ) {
       if (--failure_count_s16 < -PHASE_CHECK_DEBOUNCE){
         failure_count_s16 = -PHASE_CHECK_DEBOUNCE;
       }
      }
    }
  }  
  return phase_loss_confirmed_u8;
}
/**
  * @brief  Check Vbus and update power limit value based on Vbus. This is created for customer Trane to limit input amps based on input AC volts.
  * @param  pHandle related RDivider_Handle_t
  * @retval 1 if lost phase confirmed, 0 if 3 phase detected or not checked 
  *         becuase of low power or not enabled
  */
uint16_t Regal_CalcPowerLimitPerVbus()
{
  uint16_t wAux= 0;
  
  wAux = motorLimits01_Control.motorLimits01_Settings.outputPowerLimit_u16;
  wAux = (uint16_t)(wAux * (float)(VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS) /(float)hardwareSpecificParameters_Control.hardwareSpecificParameters_Settings.nominalVbusVolts_u16)) ;
  if(wAux > motorLimits01_Control.motorLimits01_Settings.outputPowerLimit_u16)
  { // Limit PL to outputPowerLimit_u16
    wAux = motorLimits01_Control.motorLimits01_Settings.outputPowerLimit_u16;
  }
  return(wAux);
}
