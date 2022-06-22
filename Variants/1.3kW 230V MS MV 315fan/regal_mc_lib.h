
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
#include "parameters_conversion.h"
#include "mc_tuning.h"
#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "parameters_conversion.h"

////////////////////////////////////////////////////////////////////////////
// RPa: Reference Bus Voltage Settings for non-regenerative braking
#define TORQUE2IQ_CONVERSION           338 //1.3229 shifted-
#define INDEFINITE_LOWSIDE              65535u  // Constant used during braking stage to execute indefinite shorting of coils when motor is not running
#define FP16 65536                              // 16-bit shift factor 
#define FP8  256                                // 8-bit shift factor

// RPa: Bus Voltage Control loop
#define PID_BRAKE_KP_DEFAULT          45//545   // Proportional Gain for Bus Voltage Controller
#define PID_BRAKE_KI_DEFAULT          10//160   // Integral Gain for Bus Voltage Controller
#define PID_BRAKE_KD_DEFAULT          0         // Derivative Gain for Bus Voltage Controller
/* Brake PID parameter dividers */
#define BK_KPDIV                        16        //Proportional Gain divider for Bus Voltage Controller
#define BK_KIDIV                        256       //Integral Gain divider for Bus Voltage Controller
#define BK_KDDIV                        16        //Derivative Gain divider for Bus Voltage Controller
#define BK_KPDIV_LOG                    LOG2(16)  //Proportional Gain divider (in linear scale) for Bus Voltage Controller
#define BK_KIDIV_LOG                    LOG2(256) //Integral Gain divider (in linear scale) for Bus Voltage Controller
#define BK_KDDIV_LOG                    LOG2(16)  //Derivative Gain divider (in linear scale) for Bus Voltage Controller

////////////////////////////////////////////////////////////////////////////
// RPa: Imax Controller Settings for non-regenerative braking
#define default_RAMPEND_CURRENT         (int32_t) 10 //Final current before turning the low-sides ON
#define default_RAMP_STEP               10           //Current ramping steps for both ramping up and ramping down
#define default_SPEED_TRANSITION        (uint16_t) (OBS_MINIMUM_SPEED_RPM/6) //transition speed from steady-state to ramp down
#define alpha_br                        (int16_t) (FP8*0.7)                  //LPF gain used for filtering raw speed measurements
#define BYTE_SHIFT                      8   
#define default_NBar                    205
#define default_IMRef                   500
#define MAX_BRAKE_TORQUE                (int16_t) (0.5 * PHASE4_FINAL_CURRENT )
#define MIN_IQHOLD                      (int16_t) (0.01 * PHASE4_FINAL_CURRENT )
#define default_LOWERIQLIMIT            (int16_t) (0.25 * PHASE4_FINAL_CURRENT )

// RPa: Imax Control loop
#define PID_IMAX_KP_DEFAULT             55    //Proportional Gain for Copper Loss Controller
#define PID_IMAX_KI_DEFAULT             10    //Integral Gain for Copper Loss Controller
#define PID_IMAX_KD_DEFAULT             0     //Derivative Gain for Copper Loss Controller
/* Brake PID parameter dividers */
#define IMAX_KPDIV                      16      //Proportional Gain divider for Copper Loss Controller
#define IMAX_KIDIV                      256     //Integral Gain divider for Copper Loss Controller
#define IMAX_KDDIV                      16      //Derivative Gain divider for Copper Loss Controller
#define IMAX_KPDIV_LOG                  LOG2(16)  //Proportional Gain divider (in linear scale) for Copper Loss Controller  
#define IMAX_KIDIV_LOG                  LOG2(256) //Integral Gain divider (in linear scale) for Copper Loss Controller 
#define IMAX_KDDIV_LOG                  LOG2(16)  //Derivative Gain divider (in linear scale) for Copper Loss Controller
//////////////////////////////////////////////////////////
//RPa: parameters necessary for auto-tune
#define default_MOTOR_INERTIA           29.1 /* just for test on the blender */
#define default_MOTOR_VISCOUS_DAMPING   48.2 /* RPa: coefficient of dynamic friction */
#define TORQUE_CONSTANT                 (MOTOR_VOLTAGE_CONSTANT/ 60.459979) /* RPa: Ke/((3/sqrt(3))*(60/2/pi)/1000); Nm/A (rms) */
#define FAN_INERTIA                     50.0 /*RPa: g*m^2 */
#define TOTAL_INERTIA                   (float) ( default_MOTOR_INERTIA + FAN_INERTIA )
#define NOMINAL_DC_VOLTAGE              NOMINAL_BUS_VOLTAGE_V /*230V variant: 375, 460V variant: 650; nominal dc voltage as seen on the bus */
//////////////////////////////////////////////////////////
//default settings for motor state machine
#define default_OTF_DIR_CHANGE          DISABLE /* GMI doesn't require On-the-fly direction changes */
#define default_FAULT_AUTOSTART         ENABLE  /* enable/disable auto-start when fault occurs */
#define default_MIN_COMMANDABLE_SPEED   (uint16_t) OBS_MINIMUM_SPEED_RPM /* RPa: have it based on the Observer minimum speed */
#define default_MAX_COMMANDABLE_SPEED   (uint16_t) (0.95 * MAX_APPLICATION_SPEED_RPM) /* maximum commandable speed is based on the maximum application speed*/
#define default_SPEED_UP_RAMP_RATE      200 /* settings for ramp up rate */
#define default_SPEED_DOWN_RAMP_RATE    100 /* settings for ramp down rate */
#define default_CONSIDERED_STOPPED      (uint16_t) (0.5 * OBS_MINIMUM_SPEED_RPM) /* speeds below this is considered stopped */
#define default_MOTSPINTIMEOUT          4
#define default_SPINPOLLPERIOD          PHASE1_DURATION + PHASE2_DURATION  + PHASE3_DURATION + PHASE4_DURATION + PHASE5_DURATION
#define default_NUMSTARTRETRY           6
#define default_START_RETRY_PERIOD      2000
#define default_START_PERIOD_INC        10000
#define default_OCP_THRESHOLD           (uint16_t) (0.95 * NOMINAL_CURRENT) /* based on OCP settings */
#define default_OCP_RPM_REDUCE          10
#define default_OCP_DERATE_PERIOD       200
#if ((HARDWARE_VERSION == HARDWARE_VERSION_1p3KW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW_EXT_CRYSTAL)|| (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_MV))
#define default_OVP_THRESHOLD          (int16_t)(0.98 * 1650) /* based on mechanical power and efficiency */
#elif HARDWARE_VERSION == HARDWARE_VERSION_4p5KW
#define default_OVP_THRESHOLD          (int16_t)(0.98 * 4700) /* based on mechanical power and efficiency */
#elif HARDWARE_VERSION == HARDWARE_VERSION_8KW
#define default_OVP_THRESHOLD          (int16_t)(0.98 * 8100) /* based on mechanical power and efficiency */
#else
#define default_OVP_THRESHOLD          (int16_t)(0.98 * 1300) /* based on mechanical power and efficiency */
#endif

#define default_OVP_RPM_REDUCE         10
#define default_OVP_DERATE_PERIOD      200
#define default_OTP_THRESHOLD          (uint16_t)(0.93 * OV_TEMPERATURE_THRESHOLD_C) /* could be based on some other settings, a factor of the winding class? */
#define default_OTP_RPM_REDUCE         10
#define default_OTP_DERATE_PERIOD      30000
//////////////////////////////////////////////////////////////////////////////////////
// RPa: On-the-fly definitions
#define BEMF_DEC                       8 //the bemf gain decrement from max to min during detection and synchronisation phases
#define TREF_SYNC                      (int16_t) (0.7 * PHASE4_FINAL_CURRENT )       //current reference for synchronisation
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Braking and On-the-fly definitions for default value
#define default_REGAL_OTF              ENABLE                                                              /* enable/disable of Regal On-the-fly feature */
#define default_CONTROLLED_BRAKING     ENABLE                                                     /* enable/disable for the Regal Controlled Braking feature*/
#define default_DECEL_CONTROL          ENABLE  /* enable/disable for the Regal Controlled Deceleration feature */
#define default_BK_CURRENTSEEDING      (int16_t)(0.05 * PHASE4_FINAL_CURRENT)                      /* maximum current seed at the start controlled braking */
#define default_BK_VBUS_CLIP           (uint16_t)(0.80 * (OV_VOLTAGE_THRESHOLD_V - NOMINAL_DC_VOLTAGE)) /* based on the difference between voltage levels and OVP setting */
#define default_BK_ENDSPEED            30 //(uint16_t) (1110 / MOTOR_VOLTAGE_CONSTANT)   /* RPa: Ke, SNR, and voltage levels, arbitrary: 10V bemf level, but this needs to consider OCP */
#define default_BK_LOWSIDE_DURATION    (uint16_t)(5000 + (uint16_t)(628.32 * POLE_PAIR_NUM * TOTAL_INERTIA * default_BK_ENDSPEED / (TORQUE_CONSTANT * PHASE4_FINAL_CURRENT - default_MOTOR_VISCOUS_DAMPING)))
#define default_STOP_TIMEOUT           (uint32_t)30000 /* in msec, goes back to IDLE when timesout */
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
//      This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
#define MIN_CURRENT_SPD_TRANS          (int16_t) (0.07 * PHASE4_FINAL_CURRENT ) /* RPa: based on motor ellipse from spd_trans value */
#define IMAX_SPEED_RANGE               (float)(((float) default_MAX_COMMANDABLE_SPEED /6) - ((float)default_SPEED_TRANSITION))
#define IMAX_TRAJECTORY                (float) (TREF_SYNC - MIN_CURRENT_SPD_TRANS)/(IMAX_SPEED_RANGE * IMAX_SPEED_RANGE)
#define default_BK_RAMP_a              (int16_t)(IMAX_TRAJECTORY * FP8)  /* RPa: motor ellipse equation, 13 */
#define default_BK_RAMP_b              (int16_t)(-130 * IMAX_TRAJECTORY) /* RPa: motor ellipse equation, -6 */
#define default_BK_RAMP_c              (int16_t)((4225 * IMAX_TRAJECTORY) + MIN_CURRENT_SPD_TRANS) /* RPa: motor ellipse equation , 1220*/
#define default_OTF_DBEMFG             256 /* RPa: Ke (based on observer design  and voltage level under operation; please refer to On-the-fly document for more information */
#define default_OTF_MAX_BEMFG          (uint16_t)(1.2 * default_OTF_DBEMFG)   /* gain for maximum suppression of regenerating current */
#define default_OTF_MIN_BEMFG          (uint16_t)(0.8 * default_OTF_DBEMFG)   /* RPa: gain during synchronisation stage */
#define default_OTF_MAX_SYNC_SPEED     (uint16_t) (OBS_MINIMUM_SPEED_RPM / 3 )/* RPa: Note that value is used as RPM/6 */
#define default_OTF_MIN_SYNC_SPEED     (uint16_t)(OBS_MINIMUM_SPEED_RPM / 12) /* RPa: Note that value is used as RPM/6 */
#define OTF_PHASE1_TIMEOUT             5000
#define OTF_PHASE1_FACTOR              (int16_t)(0.9 * FP8)
#define OTF_PHASE1_REV_PERIOD          1500
// Windmilling settings
#define default_WM_MAX_REVERSE_SPEED   125//(uint16_t) (OBS_MINIMUM_SPEED_RPM / 5 ) /* above the observer min speed, raise a startup failure */
#define default_WM_CLAMP_DURATION      (uint16_t) (3* PHASE3_DURATION) 
#define WM_MAX_CLAMP_CURRENT           (int16_t) NOMINAL_CURRENT /* provided by EE */
#define default_WM_CLAMP_CURRENT       (uint16_t) (PHASE3_FINAL_CURRENT + (0.5 * (WM_MAX_CLAMP_CURRENT - PHASE3_FINAL_CURRENT))) 
#define WM_PH5CURRENT_OL2CL            (uint16_t) PHASE5_FINAL_CURRENT 
#define default_WM_BK_REVERSE_ENDSPEED (uint16_t) (default_BK_ENDSPEED)
#define default_WM_BK_CLAMP_RAMP       (uint16_t) 100          
#define default_WM_SHORTCOIL_DURATION  (uint16_t) 5000
// RPa: set the powerup duration to 0 or small value, e.g. 100 when testing OTF and others for quicker startup when
#define default_POWERUP_DURATION        3000/* factor in other delays on the system */
#define FAULT_WAIT_TIME                 (uint32_t) (5000 + (uint32_t)(104.72 * POLE_PAIR_NUM * TOTAL_INERTIA *  (default_MAX_COMMANDABLE_SPEED - (2*OBS_MINIMUM_SPEED_RPM))/ (TORQUE_CONSTANT * NOMINAL_CURRENT - default_MOTOR_VISCOUS_DAMPING)))/* RPa: t (sec) = J*poles*w/(Kt*I);5000;628.3185=1000*2*pi*6/60 */ // When fault occurs, give motor time to stabilise before starting back again, this would also give the fan to decelerate a bit for next start-up
#define STOP_WAIT_TIME                  1000      // Reduce this value when testing OTF

// RPa: Burn-in definitions
#define BURN_IN_TEST                   ENABLE
#define MAX_BURNIN_FREQUENCY           800
#define MA_TO_COUNTS_CONVERSION        (uint32_t)((float)AMPLIFICATION_GAIN * (float)RSHUNT * FP16 * (FP16/(ADC_REFERENCE_VOLTAGE * 1000)))
#define COUNTS_TO_MA_CONVERSION        (int32_t) ( ADC_REFERENCE_VOLTAGE * 1000 /((float)AMPLIFICATION_GAIN * (float)RSHUNT))
#define MA_TO_COUNTS_PRECISION         16
#define COUNTS_TO_MA_PRECISION         18 /* For GMI, value is 10-bit integer and 6-bit fraction to accommodate 230V drive settings  */
// RPa: Power measurements
#define POWER_BUFFER_SIZE              256u
#define KT_SLOPE                       (int16_t) (0.004 * FP8)
/////////////////////////////////////////////////////////
/** Controlled Braking states according to D-current references  **/
typedef enum IMAX_PHASE {
    CURRENT_STARTRAMP = 0,
    CURRENT_RAMPUP,
    CURRENT_STEADYSTATE,
    CURRENT_RAMPDOWN,
    LOWSPEED_IQHOLD,
    LOWSPEED_OL_IQRAMPDOWN,
    TURNONLOWSIDE,    
    MAX_PHASE
}Imax_t;

/** Taking care of Motor Control state transitions from start-stop and stop-start **/
typedef enum TRANSITION_PHASE{
  NO_TRANSITION = 0,
  STOP_TO_START = 1,
  START_TO_STOP = 2
}StateTransition_t;

typedef enum WINDMILL_STATE{
  WM_RESET = 0,
  WM_VERYLOWSPEED = -2,
  WM_HIGH_REVERSE = -1,
  WM_HIGH_FORWARD = 1,
  WM_CURRENT_CLAMP = -3,
  WM_LOW_FORWARD = 2
}Windmill_t;

/** Handle for Controlled Braking variables **/
typedef struct Braking_Handle
{
    Imax_t BrakingPhase;                // handle for the controlled braking states
    int16_t FilteredSpeed;              // LPF speed to be used at the very low speed region when detecting when to short coils and stop motor
    int16_t rMeasuredSpeed;             // raw measurements of speed
    int32_t IMax_Ref;                   // reference maximum current used for Id injection
    int16_t Nbar;                       // Feedforward gain used in copper loss controller
    int32_t FeedForward_term;           // Feedforward term augmented to improve transient response but doesn't affect the stability of the system
    int32_t Adapt_IMax;                 // reference maximum current based on the motor loss ellipse equation
    uint16_t Adapt_BusVoltageRef;       // dynamic bus voltage reference 
    uint16_t Vbus_Clip;                  // additional potential added to the v-bus for braking current injection
    StateTransition_t TransitionPhase;  // handle for the start-stop and stop-start transitions
}
Braking_Handle_t;

/** Handle for Controlled regeneration OTF variables **/
typedef struct OTF_Handle
{
  Windmill_t hdir;                  // bit-information for the actual direction of motor spin during detection
  int16_t hSyncTRef;            // Current reference seeding when transitioning to closed-loop speed regulation
  uint16_t CoilShortSpeed;      // Speed when it is safe to turn the low-sides ON and short the coils
  uint16_t MaxSyncSpeed;        // Maximum speed to safely synchronise speed and transition to closed-loop speed regulation
  uint16_t MinSyncSpeed;        // Minimum speed to safely synchronise speed and transition to closed-loop speed regulation
  uint8_t seamless_transfer;    // bit-control current seeding and smooth transfer to closed-loop speed regulation
  int16_t bemfg_alpha;          // BEMF gain for alpha-current
  int16_t bemfg_beta;           // BEMF gain for beta-current
  int16_t detect_bemfg;         // BEMF gain for detection of speed
  int16_t max_bemfg;            // maximum BEMF gain to counter motor's bemf when inverters are switched on
  int16_t min_bemfg;            // minimum BEMF gain to ensure stability of speed estimator while minimizing effects of motor's bemf
}
OTF_Handle_t;

typedef struct Burnin_Handle
{
  int8_t en_ctrl;                  // just enable or disable of burnin
  uint16_t frequency;
  uint16_t current;
}
Burnin_Handle_t;

/** Handle for Regal Power Measurements **/
typedef struct Power_Handle
{
  int32_t wConvFact; 
  int16_t hMeasBuffer[POWER_BUFFER_SIZE]; 
  uint16_t hNextMeasBufferIndex; 
  uint16_t hLastMeasBufferIndex; 
  int16_t hAvrgMotorPowerW; 
}
Power_Handle_t;

enum // Motor direction
{
  REVERSE_ROTATION = 6,
  STD_ROTATION = 9,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t FOC_BusVoltageControlM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, RDivider_Handle_t * pBSHandle);
int32_t FOC_ImaxCurrentControllerM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, FOCVars_t * pFOCHandle_t);
void BrakingStruct_Init(Braking_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTCHandle);
void MotorBraking_StateMachine(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, FOCVars_t * pFOCHandle_t, RDivider_Handle_t * pBSHandle);
void FOCStop_CalcCurrRef(Braking_Handle_t * pBrakeHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, FOCVars_t * pFOCHandle, RDivider_Handle_t * pBSHandle);
void RegenControlM1(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDSpeedHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, RDivider_Handle_t * pBSHandle);

bool Regal_OTF_Exec( RevUpCtrl_Handle_t * pHandle, OTF_Handle_t *pOTFHandle );
//////////////////////////////////////
void Burnin_InitProfile(Burnin_Handle_t *pBurninHandle, RevUpCtrl_Handle_t * pHandle);
void Power_Clear( Power_Handle_t * pHandle );
void Regal_CalcElecMotorPower( Power_Handle_t * pHandle, RDivider_Handle_t * pBSHandle, FOCVars_t * pFOCHandle);
int16_t Regal_GetAvrgMotorPowerW( Power_Handle_t * pHandle );

uint16_t Regal_ConvertmAToCounts(uint16_t milli_amps_u16);
int16_t Regal_ConvertCountsTomA(int16_t counts_i16);
///////////////////////////////////////
extern PID_Handle_t PIDBkHandle_M1;
extern PID_Handle_t PIDImHandle_M1;
extern Braking_Handle_t BrakeHandle_M1;
extern OTF_Handle_t OTFHandle_M1;

// RPa: additional features added on the product code
extern Burnin_Handle_t Burnin_M1;
extern Power_Handle_t EEPowerCalcHandle_M1;
#endif /* _REGAL_MC_LIB_H_ */