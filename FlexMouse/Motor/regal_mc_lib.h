/**
  ********************************************************************************************************************************
  * @file    regal_mc_lib.h
  * @author  Roel Pantonial
  * @brief   header file for the Regal Motor Control Library
  * @details Definitions for On-the-Fly, Non-regenerative braking and other Regal Motor Control Algorithms
  ********************************************************************************************************************************
  *                                     
  *
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _REGAL_MC_LIB_H_
#define _REGAL_MC_LIB_H_

/* Includes ------------------------------------------------------------------*/
#include "mc_tuning.h"
#include "power_stage_parameters.h"
#include "parameters_conversion.h"
    
#define REGAL_OTF 1u /* enable/disable of Regal On-the-fly feature */
#define FAULT_AUTOSTART 1u /* enable/disable auto-start when fault occurs */

#define HARDWARE_VERSION_1P3KW_230V_LS 1
#define HARDWARE_VERSION_1P3KW_230V_HS 2
#define HARDWARE_VERSION_1P3KW_460V_LS 3
#define HARDWARE_VERSION_1P3KW_460V_HS 4
#define HARDWARE_VERSION HARDWARE_VERSION_1P3KW_230V_LS

////////////////////////////////////////////////////////////////////////////
// RPa: Reference Bus Voltage Settings for non-regenerative braking
#define TORQUE2IQ_CONVERSION           338 //1.3229 shifted-
#define BRAKING_ENDSPEED                20 // reducing from 180rpm to 120rpm (can be further reduced to 60RPM) reduces the effect of inertia when low-sides are turned on, to be tested for 1.3kW
#define BRAKING_CURRENTSEEDING          500
#define FP16 65536
#define FP8  256

// RPa: Bus Voltage Control loop
#define PID_BRAKE_KP_DEFAULT          45//545
#define PID_BRAKE_KI_DEFAULT          10//160
#define PID_BRAKE_KD_DEFAULT          0
/* Brake PID parameter dividers */
#define BK_KPDIV                      16
#define BK_KIDIV                      256
#define BK_KDDIV                      16
#define BK_KPDIV_LOG                  LOG2(16)
#define BK_KIDIV_LOG                  LOG2(256)
#define BK_KDDIV_LOG                  LOG2(16)

////////////////////////////////////////////////////////////////////////////
// RPa: Imax Controller Settings for non-regenerative braking
#define RAMPEND_CURRENT              (int32_t) 10
#define RAMP_STEP                       10  
#define SPEED_TRANSITION                65
#define alpha_br                     (int16_t) (FP8*0.7)
#define BYTE_SHIFT                      8    
//#define BK_VBUS_ADD                   20
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
//      This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
//#define BK_RAMP_a          (int32_t) 13
//#define BK_RAMP_b          (int32_t) -6
//#define BK_RAMP_c          (int32_t) 1220

// RPa: Imax Control loop
#define PID_IMAX_KP_DEFAULT          55
#define PID_IMAX_KI_DEFAULT          10
#define PID_IMAX_KD_DEFAULT          0
/* Brake PID parameter dividers */
#define IMAX_KPDIV                      16
#define IMAX_KIDIV                      256
#define IMAX_KDDIV                      16
#define IMAX_KPDIV_LOG                  LOG2(16)
#define IMAX_KIDIV_LOG                  LOG2(256)
#define IMAX_KDDIV_LOG                  LOG2(16)
//////////////////////////////////////////////////////////////////////////////////////
// RPa: On-the-fly definitions
#define BEMF_DEC 8
#define TREF_SYNC (int16_t) (0.7 * PHASE3_FINAL_CURRENT )
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//Braking and On-the-fly definitions for default value
#define default_CONTROLLED_BRAKING      0u
#define default_DECEL_CONTROL           1u    
#if HARDWARE_VERSION == HARDWARE_VERSION_1P3KW_230V_LS
#define default_BK_VBUS_ADD             70 
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
// This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
// 230V LS Low BEMF settings
#define default_BK_RAMP_a               4
#define default_BK_RAMP_b               -2
#define default_BK_RAMP_c               500
#define default_OTF_DBEMFG              100
#define default_OTF_MAX_BEMFG           130
#define default_OTF_MIN_BEMFG           50
#define default_OTF_MAX_SYNC_SPEED      120
#define default_OTF_MIN_SYNC_SPEED      30
#endif
#define default_BK_ENDSPEED             10 //(uint16_t) (1110 / MOTOR_VOLTAGE_CONSTANT)   /* RPa: Ke, SNR, and voltage levels, arbitrary: 10V bemf level, but this needs to consider OCP */

#define MA_TO_COUNTS_CONVERSION        (uint32_t)((float)AMPLIFICATION_GAIN * (float)RSHUNT * FP16 * (FP16/(ADC_REFERENCE_VOLTAGE * 1000)))
#define COUNTS_TO_MA_CONVERSION        (int32_t) ( ADC_REFERENCE_VOLTAGE * 1000 /((float)AMPLIFICATION_GAIN * (float)RSHUNT))
#define MA_TO_COUNTS_PRECISION         16
#define COUNTS_TO_MA_PRECISION         18 /* For GMI, value is 10-bit integer and 6-bit fraction to accommodate 230V drive settings  */


// Windmilling settings
#define default_WM_MAX_REVERSE_SPEED            125//(uint16_t) (OBS_MINIMUM_SPEED_RPM / 5 ) /* above the observer min speed, raise a startup failure */
#define default_WM_CLAMP_DURATION               (uint16_t) (3* PHASE3_DURATION) 
#define WM_MAX_CLAMP_CURRENT                    (int16_t) NOMINAL_CURRENT /* provided by EE */
#define default_WM_CLAMP_CURRENT                (uint16_t) (PHASE3_FINAL_CURRENT + (0.5 * (WM_MAX_CLAMP_CURRENT - PHASE3_FINAL_CURRENT))) 
#define WM_PH5CURRENT_OL2CL                     (uint16_t) PHASE5_FINAL_CURRENT 
#define default_WM_BK_REVERSE_ENDSPEED          (uint16_t) (default_BK_ENDSPEED/2)
#define default_WM_BK_CLAMP_RAMP                (uint16_t) 100          
#define default_WM_SHORTCOIL_DURATION        (uint16_t) 5000

#if HARDWARE_VERSION == HARDWARE_VERSION_1P3KW_460V_LS  
#define default_BK_VBUS_ADD             100 
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
// This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
// 230V LS Low BEMF settings
#define default_BK_RAMP_a               4
#define default_BK_RAMP_b               -2
#define default_BK_RAMP_c               500
#define default_OTF_DBEMFG              100
#define default_OTF_MAX_BEMFG           130
#define default_OTF_MIN_BEMFG           50
#define default_OTF_MAX_SYNC_SPEED      120
#define default_OTF_MIN_SYNC_SPEED      30
#endif

//////////////////////////////////////////////////////////////////////////////////////
// RPa: On-the-fly definitions // 230V LS low BEMF motor parameters
//#define BEMF_DEC 5
//#define TREF_SYNC (int16_t) (0.7 * PHASE3_FINAL_CURRENT )
//#define OTF_DBEMFG 100
//#define OTF_MAX_BEMFG 130
//#define OTF_MIN_BEMFG 50
//#define OTF_MAX_SYNC_SPEED 120
//#define OTF_MIN_SYNC_SPEED 30
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum IMAX_PHASE {
    STARTRAMP = 0,
    RAMPUP,
    STEADYSTATE,
    RAMPDOWN,
    LOWSPEED_IQHOLD,
    TURNONLOWSIDE,    
    MAX_PHASE
}Imax_t;

typedef struct Braking_Handle
{
    Imax_t BrakingPhase;
    int16_t FilteredSpeed;
    int16_t rMeasuredSpeed;
    int32_t IMax_Ref;
    int16_t Nbar;
    int32_t FeedForward_term;
    int32_t Adapt_IMax;
    uint16_t Adapt_BusVoltageRef;
    uint16_t Vbus_Add;
}
Braking_Handle_t;

typedef struct OTF_Handle
{
  int8_t hdir;
  int16_t hSyncTRef;
  uint16_t CoilShortSpeed;
  uint16_t MaxSyncSpeed;
  uint16_t MinSyncSpeed;
  uint8_t seamless_transfer;
  int16_t bemfg_alpha;
  int16_t bemfg_beta;
  int16_t detect_bemfg;
  int16_t max_bemfg;
  int16_t min_bemfg;
}
OTF_Handle_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t FOC_BusVoltageControlM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, RDivider_Handle_t * pBSHandle);//RPa
int32_t FOC_ImaxCurrentControllerM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, FOCVars_t * pFOCHandle_t);//RPa
void BrakingStruct_Init(Braking_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTCHandle);//RPa
void MotorBraking_StateMachine(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, FOCVars_t * pFOCHandle_t, RDivider_Handle_t * pBSHandle);
void FOCStop_CalcCurrRef(Braking_Handle_t * pBrakeHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, FOCVars_t * pFOCHandle, RDivider_Handle_t * pBSHandle);
void RegenControlM1(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDSpeedHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, RDivider_Handle_t * pBSHandle);

bool Regal_OTF_Exec( RevUpCtrl_Handle_t * pHandle, OTF_Handle_t *pOTFHandle );

extern PID_Handle_t PIDBkHandle_M1;//RPa
extern PID_Handle_t PIDImHandle_M1;//RPa
extern Braking_Handle_t BrakeHandle_M1;
extern OTF_Handle_t OTFHandle_M1;

#endif /* _REGAL_MC_LIB_H_ */