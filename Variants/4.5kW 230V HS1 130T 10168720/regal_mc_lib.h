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
#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"

////////////////////////////////////////////////////////////////////////////
// RPa: Reference Bus Voltage Settings for non-regenerative braking
#define TORQUE2IQ_CONVERSION           338 //1.3229 shifted-
#define INDEFINITE_LOWSIDE              65535u
#define FP16 65536
#define FP8  256

// RPa: Bus Voltage Control loop
#define PID_BRAKE_KP_DEFAULT          25//545
#define PID_BRAKE_KI_DEFAULT          10//160
#define PID_BRAKE_KD_DEFAULT          0
/* Brake PID parameter dividers */
#define BK_KPDIV                      32
#define BK_KIDIV                      256
#define BK_KDDIV                      16
#define BK_KPDIV_LOG                  LOG2(32)
#define BK_KIDIV_LOG                  LOG2(256)
#define BK_KDDIV_LOG                  LOG2(16)

////////////////////////////////////////////////////////////////////////////
// RPa: Imax Controller Settings for non-regenerative braking
#define RAMPEND_CURRENT              (int32_t) 10
#define RAMP_STEP                       10  
#define SPEED_TRANSITION                (uint16_t) (OBS_MINIMUM_SPEED_RPM/6)
#define alpha_br                         (int16_t) (FP8*0.7)
#define BYTE_SHIFT                      8   

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
//////////////////////////////////////////////////////////
//RPa: parameters necessary for auto-tune
#define default_MOTOR_INERTIA           49.1 /* just for test on the blender */
#define default_MOTOR_VISCOUS_DAMPING   98.2 /* RPa: coefficient of dynamic friction */
#define TORQUE_CONSTANT                 (MOTOR_VOLTAGE_CONSTANT/ 60.459979) /* RPa: Ke/((3/sqrt(3))*(60/2/pi)/1000); Nm/A (rms) */
#define FAN_INERTIA                             100.0 /*RPa: g*m^2 */
#define TOTAL_INERTIA                   (float) ( default_MOTOR_INERTIA + FAN_INERTIA )
#define NOMINAL_DC_VOLTAGE              325 /*230V variant: 375, 460V variant: 650 */
//////////////////////////////////////////////////////////
//default settings for motor state machine
#define default_OTF_DIR_CHANGE          DISABLE /* GMI doesn't require On-the-fly direction changes */
#define default_FAULT_AUTOSTART         ENABLE /* enable/disable auto-start when fault occurs */
#define default_MIN_COMMANDABLE_SPEED   (uint16_t) OBS_MINIMUM_SPEED_RPM /* RPa: have it based on the Observer minimum speed */
#define default_MAX_COMMANDABLE_SPEED   (uint16_t) (0.9 * MAX_APPLICATION_SPEED_RPM)
#define default_SPEED_UP_RAMP_RATE      200 /* RPa: can be based on J*alpha */
#define default_SPEED_DOWN_RAMP_RATE    100 /* RPa: can be based on J*alpha */
#define default_CONSIDERED_STOPPED      (uint16_t) (0.5 * OBS_MINIMUM_SPEED_RPM) /* RPa: can be based on Ke and V-bus levels */
#define default_MOTSPINTIMEOUT          4
#define default_SPINPOLLPERIOD          PHASE1_DURATION + PHASE2_DURATION  + PHASE3_DURATION + PHASE4_DURATION + PHASE5_DURATION
#define default_NUMSTARTRETRY           6
#define default_START_RETRY_PERIOD      2000
#define default_START_PERIOD_INC        10000
#define default_OCP_THRESHOLD           (uint16_t) (1 * NOMINAL_CURRENT) //(uint16_t) (0.9 * NOMINAL_CURRENT) /* based on OCP settings */
#define default_OCP_RPM_REDUCE          10
#define default_OCP_DERATE_PERIOD       200
#define default_OVP_THRESHOLD           5000  /* based on mechanical power, include an efficiency */
#define default_OVP_RPM_REDUCE          10
#define default_OVP_DERATE_PERIOD       200
#define default_OTP_THRESHOLD           (uint16_t) (0.9 * OV_TEMPERATURE_THRESHOLD_C) /* could be based on some other settings, a factor of the winding class? */
#define default_OTP_RPM_REDUCE          10
#define default_OTP_DERATE_PERIOD       30000
//////////////////////////////////////////////////////////////////////////////////////
// RPa: On-the-fly definitions
#define BEMF_DEC 8
#define TREF_SYNC (int16_t) (0.7 * PHASE4_FINAL_CURRENT )
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Braking and On-the-fly definitions for default value
#define default_REGAL_OTF               ENABLE /* enable/disable of Regal On-the-fly feature */
#define default_CONTROLLED_BRAKING      ENABLE
#define default_DECEL_CONTROL           ENABLE
#define BRAKING_CURRENTSEEDING          (int16_t) (0.05 * PHASE4_FINAL_CURRENT )
#define default_BK_VBUS_ADD             (uint16_t) (0.25 * ( OV_VOLTAGE_THRESHOLD_V - NOMINAL_DC_VOLTAGE))   /* based on the difference between voltage levels and OVP setting */
#define default_BK_ENDSPEED             10 //(uint16_t) (1110 / MOTOR_VOLTAGE_CONSTANT)   /* RPa: Ke, SNR, and voltage levels, arbitrary: 10V bemf level, but this needs to consider OCP */
#define default_BK_LOWSIDE_DURATION        (uint16_t) (5000 + (uint16_t)(628.32 * POLE_PAIR_NUM * TOTAL_INERTIA * default_BK_ENDSPEED/ (TORQUE_CONSTANT * PHASE4_FINAL_CURRENT - default_MOTOR_VISCOUS_DAMPING)))
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
//      This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
#define MIN_CURRENT_SPD_TRANS           (int16_t) (0.07 * PHASE4_FINAL_CURRENT ) /* RPa: based on motor ellipse from spd_trans value */
#define IMAX_SPEED_RANGE                (float)(((float) default_MAX_COMMANDABLE_SPEED /6) - ((float)SPEED_TRANSITION))
#define IMAX_TRAJECTORY                 (float) (TREF_SYNC - MIN_CURRENT_SPD_TRANS)/(IMAX_SPEED_RANGE * IMAX_SPEED_RANGE)
#define default_BK_RAMP_a               (int16_t) (IMAX_TRAJECTORY * FP8) /* RPa: motor ellipse equation, 13 */
#define default_BK_RAMP_b               (int16_t) (-130 * IMAX_TRAJECTORY) /* RPa: motor ellipse equation, -6 */
#define default_BK_RAMP_c               (int16_t) ((4225 * IMAX_TRAJECTORY) + MIN_CURRENT_SPD_TRANS) /* RPa: motor ellipse equation , 1220*/
#define default_OTF_DBEMFG              325 /* RPa: Ke  and voltage levels */
#define default_OTF_MAX_BEMFG           (uint16_t) (1.2 * default_OTF_DBEMFG) /* RPa: Ke  and voltage levels */
#define default_OTF_MIN_BEMFG           (uint16_t) (0.8 * default_OTF_DBEMFG) /* RPa: Ke  and voltage levels */
#define default_OTF_MAX_SYNC_SPEED      (uint16_t) (OBS_MINIMUM_SPEED_RPM / 3 )/* RPa: Note that value is used as RPM/6 */
#define default_OTF_MIN_SYNC_SPEED      (uint16_t) (OBS_MINIMUM_SPEED_RPM / 12 ) /* RPa: Note that value is used as RPM/6 */
#define OTF_PHASE1_TIMEOUT              3000
#define OTF_PHASE1_FACTOR               (int16_t) (0.9 * FP8)
// RPa: set the powerup duration to 0 or small value, e.g. 100 when testing OTF and others for quicker startup when
#define default_POWERUP_DURATION     (uint32_t) (5000 + (uint32_t)(104.72 * POLE_PAIR_NUM * TOTAL_INERTIA *  (default_MAX_COMMANDABLE_SPEED - (2*OBS_MINIMUM_SPEED_RPM))/ (TORQUE_CONSTANT * NOMINAL_CURRENT - default_MOTOR_VISCOUS_DAMPING)))// 10000/* RPa: t (sec) = J*poles*w/(Kt*I);5000;628.3185=1000*2*pi*6/60 */
/////////////////////////////////////////////////////////
typedef enum IMAX_PHASE {
    STARTRAMP = 0,
    RAMPUP,
    STEADYSTATE,
    RAMPDOWN,
    LOWSPEED_IQHOLD,
    TURNONLOWSIDE,    
    MAX_PHASE
}Imax_t;

typedef enum TRANSITION_PHASE{
  NO_TRANSITION = 0,
  STOP_TO_START = 1,
  START_TO_STOP = 2
}StateTransition_t;

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
    StateTransition_t TransitionPhase;
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