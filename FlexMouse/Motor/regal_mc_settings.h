/**
  ********************************************************************************************************************************
  * @file   regal_mc_settings.h
  * @author  Satya Akkina
  * @brief   Main driver module for flash storgae management.
  * @details This module initializes the flash 
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/


/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _FLASH_PARAMETERS_H_
#define _FLASH_PARAMETERS_H_


/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
//#include <assert.h>
//#include <limits.h>
//#include <stdio.h>
  
#include "main.h"
#include "typedef.h"
  
#include "structured_memory.h"
#include "scheduler.h"
#include "zz_module_flash.h"
  
#define PARAMETER_SETTING_SIZE 68
#define MAX_PRAMETER_GROUPS 30 // 30 * 68 + 4 (CRC) + 4(version) = 2048
#define FLASH_PARAMETER_MIN_GROUP_ID 0
 
  
#define TORQUE_CONV_FACTOR 10  //
#define RES_CONV_FACTOR 1000   // mOhms
#define IND_CONV_FACTOR 100000 // mH * 10
#define VBUS_PERCENT_FACTOR 10 // %vbus * 10
#define BEMF_KE_FACTOR 10      // Ke * 10
#define TORQUE_CONSTANT_FACTOR 1000 // (Ke/60.459979) * 1000
#define CURRENT_CONV_FACTOR 1000    //mA
#define VARIANCE_THRESHOLD_FACTOR 10
#define HUNDERED_MS_FACTOR 100

#define ONE_TENTH_FACTOR      (0.1)
#define ONE_HUNDEREDTH_FACTOR (0.01)
#define MILI_FACTOR           (0.001)
  
void init_flash_parmeters(void);
void Init_Default_Settings(void);

//typedef enum {
//  ID = FLASH_PARAMETER_MIN_GROUP_ID,
//  APPLICATION_SPECIFIC, // 1
//  START_UP_PARAM,    // 2
//  MOTOR_DATA,        // 3
//  TUNING,            // 4
//  LIMITS,            // 5
//  PROTECTIONS,       // 6
//  FAN_PARAM,         // 7
//  OTF_PARAM,         // 8
//  BRAKING_PARAM,     // 9
//  WIND_MILLING,      // 10
//  HARDWARE_SPECIFIC, // 11
//  HARMONIC_COMP1,    // 12
//  HARMONIC_COMP1,    // 13
//  HARMONIC_COMP1,    // 14
//  UNUSED1,           // 15
//  UNUSED2,           // 16
//  UNUSED3,           // 17
//  UNUSED4,           // 18
//  UNUSED5,           // 19
//  UNUSED6,           // 20
//  UNUSED7,           // 21
//  UNUSED8,           // 22
//  UNUSED0,           // 21
//  UNUSED10,          // 22
//  MISC1,             // 23
//  MISC2,             // 24
//  MISC3,             // 25
//  FLASH_PARAMETER_MAX_GROUP_ID //Always keep this last // This should be less than MAX_MODULES_IN_FLASH
//} FlashParameterIDs;

// Control modes
enum
{
  TORQUE_CONTROL_MODE = 0,
  SPEED_CONTROL_MODE,
  AIR_FLOW_CONTROL_MODE,
  SENSOR_SPEED_CONTROL_MODE,
  SENSOR_TORQUE_CONTROL_MODE,
  SENSOR_AIR_FLOW_CONTROL_MODE,
};

//////////// Start of "Identification" Parameters /////////////////

typedef struct
{ 
  // USER Access
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
} MotorIdUser_Flags1;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} MotorIdUser_Discretes1;

struct MotorId_Data
{
  // USER Access
  MotorIdUser_Discretes1 userDiscretes01;
  uint16_t unusedUserDiscretes02;       // Reserved for discrete flags
  uint16_t minorDriveFirmwareRev_u16;  // in ASCII YY (VV.XX.YY)
  uint16_t medianDriveFirmwareRev_u16; // In ASCII XX (VV.XX.YY)
  uint16_t majorDriveFirmwareRev_u16;  // In ASCII VV (VV.XX.YY)
  uint16_t minorDriveFirmwareCrc_u16;  // Motor code CRC
  uint16_t majorDriveFirmwareCrc_u16;  // Motor code CRC
  uint16_t minorDriveFlashRev_u16; // In ASCII, but each nible only 0-9 values are allowed
  uint16_t majorDriveFlashRev_u16; // In ASCII, but each nible only 0-9 values are allowed 00VV, XXYY
  uint16_t minorDriveFlashCrc_u16; // Motor flash settings CRC
  uint16_t majorDriveFlashCrc_u16; // Motor flash settings CRC
  
  uint16_t minorSafetyFirmwareRev_u16;  // in ASCII YY (VV.XX.YY)
  uint16_t medianSafetyFirmwareRev_u16; // In ASCII XX (VV.XX.YY)
  uint16_t majorSafetyFirmwareRev_u16;  // In ASCII VV (VV.XX.YY)
  uint16_t minorSafetyFirmwareCrc_u16;  // Motor code CRC
  uint16_t majorSafetyFirmwareCrc_u16;  // Motor code CRC
  //uint16_t minorSafetyFlashRev_u16; // In ASCII, but each nible only 0-9 values are allowed
  //uint16_t majorSafetyFlashRev_u16; // In ASCII, but each nible only 0-9 values are allowed 00VV, XXYY
  uint16_t minorSafetyFlashCrc_u16; // Motor flash settings CRC
  uint16_t majorSafetyFlashCrc_u16; // Motor flash settings CRC
  
  uint16_t productType_u16;     // Like GMI
  uint16_t loadVariant_u16;     // Fan/pump or any load coupled Version/Variant
  
  uint16_t unusedData21;
  uint16_t unusedData22;
  uint16_t unusedData23;
  uint16_t unusedData24;
  uint16_t unusedData25;
  uint16_t unusedData26;
  uint16_t unusedData27;
  uint16_t unusedData28;  
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData31;
  uint16_t unusedData32;
  uint16_t unusedData33;
  uint16_t unusedData34;
  uint16_t unusedData35;
  uint16_t unusedData36;
  uint16_t unusedData37;
};

struct MotorId_Settings
{
  // USER Access
  MotorIdUser_Flags1 userFlags01; // User accessable flags
  uint16_t unusedUserFlags02;     // Reserved for flags
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  uint16_t unusedSettings08;
  uint16_t unusedSettings09;
  uint16_t unusedSettings10;
  uint16_t unusedSettings11;
  uint16_t unusedSettings12;
  uint16_t unusedSettings13;
  uint16_t unusedSettings14;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01;    // Reserved for Admin flags
  uint16_t unusedAdminFlags02;    // Reserved for Admin flags
  //uint16_t hardwareVariant_u16; // Hardware Version/Variant Eg: 4.5kW 1.3kW
  //uint16_t hardwareVersion_u16; // Hardware Version/Variant
  uint16_t setProductType_u16;    // GMI/P22 and such
  uint16_t setLoadVariant_u16;    // Fan/pump or any load coupled Version/Variant
  uint16_t unusedSettings19;      // 
  uint16_t unusedSettings20;      // Added to prevent padding for flags
};

typedef struct
{
  struct MotorId_Settings motorId_Settings;
  struct MotorId_Data motorId_Data;  
}MotorId_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorId_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorId_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Identification" Parameters /////////////////

//////////// Start of "Start-up" Parameters //////////////////////////////////
typedef struct
{ 
  // USER Access
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
} StartupParametersUser01_Flags;

typedef struct
{ 
  // USER Access
  uint16_t is_alignmentPhaseComplete:1;   // True if alignment phase is complete
  uint16_t is_rampPhaseComplete:1;  // True if ramp up to observer min speed complete
  uint16_t is_openToClosedLoopTransComplete:1; // True if open to closed loop transition complete 
  uint16_t unusedDiscretes01:1; 
  uint16_t unusedDiscretes02:1;
  uint16_t unusedDiscretes03:1;
  uint16_t unusedDiscretes04:1; 
  uint16_t unusedDiscretes05:1; 
  uint16_t unusedDiscretes06:1; 
  uint16_t unusedDiscretes07:1;
  uint16_t unusedDiscretes08:1; 
  uint16_t unusedDiscretes12:1;    
  uint16_t unusedDiscretes13:1; 
  uint16_t unusedDiscretes14:1;
  uint16_t unusedDiscretes15:1;
  uint16_t unusedDiscretes16:1;  
} StartupParametersAdmin_Discretes1;

typedef struct
{ 
  // USER Access
  uint16_t unusedDiscretes01:1; 
  uint16_t unusedDiscretes02:1;
  uint16_t unusedDiscretes03:1;
  uint16_t unusedDiscretes04:1; 
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
} StartupParametersUser_Discretes1;

struct StartupParameters_Data
{
  // USER Access
  StartupParametersUser_Discretes1 userDiscretes01;
  uint16_t userDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  StartupParametersAdmin_Discretes1 adminDiscretes01;
  uint16_t adminDiscretes02;
  uint16_t unusedData13;
  uint16_t unusedData14;
};

struct StartupParameters_Settings
{
  // USER Access
  StartupParametersUser01_Flags userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  
  // ADMIN Access
  uint16_t adminFlags01;
  uint16_t detectionDuration_u16;  // Detection duration // PHASE1_DURATION
  uint16_t startupBrakingDuration_u16; // Braking duration while start-up. // PHASE2_DURATION
  uint16_t alignmentTime01_u16;    // Alignment Time in mSec
  uint16_t alignmentSpeed01_u16;
  uint16_t alignmentCurrent01_u16; // Alignment Current in mAmps
  uint16_t alignmentTime02_u16;  
  uint16_t alignmentSpeed02_u16;
  uint16_t alignmentCurrent02_u16;  
  uint16_t startRampTime_u16;     // Start-up Ramp Time in mSec
  uint16_t startRampToSpeed_u16;  // Start-up Ramp to Speed in RPM
  uint16_t startRampCurrent_u16;  // Start Ramp Current in mAmps  
  uint16_t transitionTime_u16;    // Transition time (open to closed loop) in mSec
  uint16_t nbConsecutiveTest_u16; // Number of consicutive tests before speed feedback fault
  uint16_t powerUpDelay_u16;      // Delay on power up before attempting startup
  uint16_t startRetyPeriod_u16;     // Delay before next restart attempt is made
  uint16_t startRetryIncPeriod_u16; // Increase delay by this ammount after every retry
  uint16_t startNumOfRetries_u16;   // Maximum number of retries allowed
  uint16_t unusedSettings25;
  uint16_t unusedSettings26;        // Added to prevent padding 
};

typedef struct
{
  struct StartupParameters_Settings startupParameters_Settings;
  struct StartupParameters_Data startupParameters_Data;  
}StartupParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct StartupParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"StartupParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Start-up" Parameters //////////////////////////////////

//////////// Start of "Motor" Parameters //////////////////////////////////
typedef struct
{ 
  // USER Access
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
} MotorParametersUser_Flags1;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} MotorParametersUser_Discretes1;

struct MotorParameters_Data
{
  // USER Access
  MotorParametersUser_Discretes1 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData11;
  uint16_t unusedData12;  
};

struct MotorParameters_Settings
{
  // USER Access
  MotorParametersUser_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01;
  uint16_t polePairs_u16;             // Pole Pairs
  uint16_t statorPhaseResistance_u16; // Stator Resistance (Rs (L-N)) in mOhms
  uint16_t resistanceFactor_u16;
  uint16_t phaseInductanceD_u16; // Ld (L-N) in miliHenry * 10, factor of 10000
  uint16_t phaseInductanceQ_u16; // Lq (L-N) in miliHenry * 10, factor of 10000
  uint16_t inductanceFactor_u16;
  uint16_t bemfConstant_u16;    // L-L BEMF voltage @ 1000RPM (Ke Voltage Constant), factor of ke*10
  uint16_t bemfFactor_u16;
  uint16_t motorInertia_u16;     // Motor Inertia
  uint16_t motorInertiaFactor_u16;
  uint16_t maxMotorCurrent_u16;  // Max Motor Current in mAmps
  uint16_t motorViscousDampingFactor_u16; // Motor Viscous Damping Factor
  uint16_t torqueConstant_u16; // Torque Constant (NM/A)
  uint16_t motorMaxRpm_u16;    // Electrical/mechanical speed limit of the motor in RPM
  uint16_t motorMinRpm_u16;    
  uint16_t pwmFrequency_u16;   // PWM output frequency in HZ
  uint16_t unusedSettings24;
  uint16_t unusedSettings25;
  uint16_t unusedSettings26;   // Added to prevent padding for flags  
};


typedef struct
{
  struct MotorParameters_Settings motorParameters_Settings;
  struct MotorParameters_Data motorParameters_Data;  
}MotorParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Motor" Parameters //////////////////////////////////

//////////// Start of "Motor Tuning 01" Parameters //////////////////////////

typedef struct
{ 
  // USER Access
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
} MotorTunning01User_Flags1;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} MotorTunning01User_Discretes1;

struct MotorTunning01_Data
{
  // USER Access
  MotorTunning01User_Discretes1 userDiscretes01;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01; // Reserved
  uint16_t unusedData10;
  uint16_t unusedData11;
  uint16_t unusedData12; 
};

struct MotorTunning01_Settings
{ 
  // USER Access
  MotorTunning01User_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01; // Reserved for flags
  int16_t  fluxKp_s16;         // Flux Kp
  uint16_t fluxKpFactor_u16;   // Flux Kp factor
  int16_t  fluxKi_s16;         // Flux Ki
  uint16_t fluxKiFactor_u16;   // Flux Ki factor
  int16_t  fluxKd_s16;         // Flux Kd
  uint16_t fluxKdFactor_u16;   // Flux Kd factor
  int16_t  torqueKp_s16;       // Torque Kp
  uint16_t torqueKpFactor_u16; // Torque Kp factor
  int16_t  torqueKi_s16;       // Torque Ki
  uint16_t torqueKiFactor_u16; // Torque Ki factor  
  int16_t  torqueKd_s16;       // Torque Kd
  uint16_t torqueKdFactor_u16; // Torque Kd factor
  int16_t  speedKp_s16;        // Speed Kp
  uint16_t speedKpFactor_u16;  // Speed Kp factor
  int16_t  speedKi_s16;        // Speed Ki
  uint16_t speedKiFactor_u16;  // Speed Ki factor
  int16_t  speedKd_s16;        // Speed Kd  
  uint16_t speedKdFactor_u16;  // Speed Kd factor
  uint16_t varianceThreshold_u16;// % variance of Est. and Mes. speed 
  uint16_t obsMeasErrorsBeforeFaults; // OBS_MEAS_ERRORS_BEFORE_FAULTS
  uint16_t unusedSettings29;   //
  uint16_t unusedSettings30;   // Added to prevent padding for flags 
  // MAX settings(60byteS) reached per module
};

typedef struct
{
  struct MotorTunning01_Settings motorTunning01_Settings;
  struct MotorTunning01_Data motorTunning01_Data;  
}MotorTunning01_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorTunning01_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorTunning01_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Motor Tuning 01" Parameters //////////////////////////

//////////// Start of "Motor Tuning" Parameters //////////////////////////

typedef struct
{ 
  // USER Access
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
} MotorTunning02User_Flags1;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} MotorTunning02User_Discretes1;

struct MotorTunning02_Data
{
  // USER Access
  MotorTunning02User_Discretes1 userDiscretes01;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01; // Reserved
  uint16_t unusedData10; 
  uint16_t unusedData11;
  uint16_t unusedData12; 
};

struct MotorTunning02_Settings
{ 
  // USER Access
  MotorTunning02User_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01_u16;       // Reserved for flags
  int16_t  bemfObservedGain01_s16;       // GAIN1/C2 in ST MC
  uint16_t bemfObserverGain01Factor_u16; // F1 in ST MC
  int16_t  bemfObservedGain02_s16;       // GAIN2/C4 in ST MC
  uint16_t bemfObserverGain02Factor_u16; // F2 in ST MC
  int16_t  bemfObservedGain03_s16;
  uint16_t bemfObserverGain03Factor_u16;
  int16_t  bemfObservedGain04_s16;
  uint16_t bemfObserverGain04Factor_u16;
  int16_t  bemfObservedGain05_s16;
  uint16_t bemfObserverGain05Factor_u16;  
  int16_t  angleEst01Gain_u16;       // Angle estimator gain 01 // PLL_KP_GAIN
  uint16_t angleEst01GainFactor_u16; // Angle Estimator Gain 01 Factor
  int16_t  angleEst02Gain_u16;       // Angle estimator gain 02 // PLL_KI_GAIN
  uint16_t angleEst02GainFactor_u16; // Angle Estimator Gain 02 Factor
  int16_t  angleEst03Gain_u16;       // Angle estimator gain 03
  uint16_t angleEst03GainFactor_u16; // Angle Estimator Gain 03 Factor
  uint16_t observerMinRpm_u16;   // Observer Min Speed
  uint16_t adcSamplingCycles_u16;// ADC sampling rate. 
  uint16_t regSamplingRate_u16;  // Number of PWM cycles before FOC calc is updated
  int16_t  gain1_s16;             // Gain1
  int16_t  gain2_s16;             // Gain2 
  uint16_t unusedSettings28;
  uint16_t unusedSettings29;
  uint16_t unusedSettings30;     // Added to prevent padding for flags
  // MAX settings(60byteS) reached per module
};

typedef struct
{
  struct MotorTunning02_Settings motorTunning02_Settings;
  struct MotorTunning02_Data motorTunning02_Data;  
}MotorTunning02_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorTunning02_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorTunning02_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Motor Tuning 01" Parameters //////////////////////////

////////////// Start of "Drive" Parameters //////////////////////////////////
//typedef struct
//{ 
//  // USER Access
//  uint16_t unusedFlags01:1;   
//  uint16_t unusedFlags02:1; 
//  uint16_t unusedFlags03:1;
//  uint16_t unusedFlags04:1; 
//  uint16_t unusedFlags05:1; 
//  uint16_t unusedFlags06:1; 
//  uint16_t unusedFlags07:1; 
//  uint16_t unusedFlags08:1;
//  
//  // ADMIN Access
//  uint16_t unusedFlags09:1; 
//  uint16_t unusedFlags10:1; 
//  uint16_t unusedFlags11:1;
//  uint16_t unusedFlags12:1; 
//  uint16_t unusedFlags13:1; 
//  uint16_t unusedFlags14:1;
//  uint16_t unusedFlags15:1;   
//  uint16_t unusedFlags16:1;
//} DriveParameters_Flags;
//
//struct DriveParameters_Settings
//{
//  uint16_t nominalVbus;          // Nominal Bus votlage
//  uint16_t maxDriveCurrent;      // Abs max IPM 
//  DriveParameters_Flags flags_u16;
//};
//typedef struct
//{  
//  // USER Access
//  uint16_t unusedDiscretes01:1;   
//  uint16_t unusedDiscretes02:1; 
//  uint16_t unusedDiscretes03:1; 
//  uint16_t unusedDiscretes04:1; 
//  uint16_t unusedDiscretes05:1; 
//  uint16_t unusedDiscretes06:1;
//  uint16_t unusedDiscretes07:1; 
//  uint16_t unusedDiscretes08:1; 
//  uint16_t unusedDiscretes09:1;
//  uint16_t unusedDiscretes10:1;
//  uint16_t unusedDiscretes11:1; 
//  uint16_t unusedDiscretes12:1;
//  
//  // ADMIN Access
//  uint16_t unusedDiscretes13:1; 
//  uint16_t unusedDiscretes14:1; 
//  uint16_t unusedDiscretes15:1;
//  uint16_t unusedDiscretes16:1;
//} DriveParameters_Discretes;
//
//struct DriveParameters_Data
//{
//  uint16_t unused01;
//  DriveParameters_Discretes discretes_u16;  
//};
//
//typedef struct
//{
//  struct DriveParameters_Settings driveParameters_Settings;
//  struct DriveParameters_Data driveParameters_Data;  
//}DriveParameters_Control;
//
//// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
//// Note "Padding" would affect the struct size
//static_assert( ( sizeof(struct DriveParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"DriveParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );
//
////////////// End of "Drive" Parameters //////////////////////////////////


//////////// Start of "Limits" (derating) Parameters //////////////////////////////

typedef struct
{ 
  // USER Access
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
} MotorLimits01User_Flags1;

typedef struct
{ 
  // Admin Access
  uint16_t is_powerLimitEnable:1;   
  uint16_t is_currentLimitEnable:1; 
  uint16_t is_ipmTemperatureLimit:1; 
  uint16_t is_pfcTemperatureLimit:1;
  uint16_t is_iclEnable:1;   
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
} MotorLimits01Admin_Flags1;

typedef struct
{ 
  // USER Access
  uint16_t is_powerLimitActive:1;   
  uint16_t is_currentLimitActive:1; 
  uint16_t is_ipmTempratureLimitActive:1; 
  uint16_t is_pfcTemperatureLimitActive:1;
  uint16_t is_iclActive:1; 
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
} MotorLimits01User_Discretes1;

struct MotorLimits01_Data
{
  // USER Access
  MotorLimits01User_Discretes1 userDiscretes01;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // Admin Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData10;
  uint16_t unusedData11;
  uint16_t unusedData12; 
};

struct MotorLimits01_Settings
{
  // USER Access
  MotorLimits01User_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  
  // ADMIN Access
  MotorLimits01Admin_Flags1 adminFlags01;
  
  uint16_t maxOutputPower_u16;              // Absolute Max. Power  
  uint16_t outputPowerLimit_u16;    // Power Limit Threshold for derate
  uint16_t outputPowerLimitHys_u16; // Hysteresis for power limit
  uint16_t outputPowerLimitDelay_u16;       // If the measured value > limit threshold before delay expired enable limit  
  uint16_t outputPowerLimitRpmReduce_u16;   // Reduce rpm by this amount for derate
  uint16_t outputPowerLimitRmpIncrease_u16; // Incease RPM by this amount for every LimitPeriod once the power is below limit threshold
  uint16_t outputPowerLimitPeriod_u16;      // Apply derate after this time period  
  
  uint16_t maxOutputCurrent_u16;            // Absolute Max. current IPM can support
  uint16_t outputCurrentLimit_u16;          // Current Limit Threshold for derate
  uint16_t outputCurrentLimitHys_u16;       // Hysteresis for power limit
  uint16_t outputCurrentLimitRpmReduce_u16; // Reduce rpm by this about for derate
  uint16_t outputCurrentLimitPeriod_u16;    // Apply derate after this time period
  
  uint16_t maxIpmTemperature_u16;           // Absolute Max. IPM Temperature
  uint16_t ipmTemperatureLimit_u16;         // IPM Temperature Limit Threshold for derate
  uint16_t ipmTemperatureLimitHys_u16;      // Hysteresis for power limit
  uint16_t ipmTempratureLimitRpmReduce_u16; // Reduce rpm by this about for derate
  uint16_t ipmTemperatureLimitPeriod_u16;   // Apply derate after this time period
  
  uint16_t maxPfcTemperature_u16;           // Absolute Max. PFC Temperature
  uint16_t pftTemperatureLimit_u16;         // PFC Temperature Limit Threshold for derate
  uint16_t pfcTemperatureLimitHys_u16;      // Hysteresis for power limit
  uint16_t pfcTempratureLimitRpmReduce_u16; // Reduce rpm by this about for derate
  uint16_t pfcTemperatureLimitPeriod_u16;   // Apply derate after this time period  
  
  uint16_t iclTime_u16;                     // Inrush current limit(ICL) time
  uint16_t unusedSettings30;                // Added to prevent padding for flags                 
  // MAX settings(60byteS) reached per module
};

typedef struct
{
  struct MotorLimits01_Settings motorLimits01_Settings;
  struct MotorLimits01_Data motorLimits01_Data;  
}MotorLimits01_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorLimits01_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorLimits01_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// Start of "Limits" Parameters //////////////////////////////

//////////// Start of "Protection01" Parameters //////////////////////////////
typedef struct
{ 
  // USER Access
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
} MotorProtections01User_Flags1;
typedef struct
{ 
  // ADMIN Access
  uint16_t is_overVoltageEnable:1;   
  uint16_t is_underVoltageEnable:1; 
  uint16_t is_overCurrentEnable:1; 
  uint16_t is_lossOfInputPhaseEnable:1;
  uint16_t is_faultAutoStartEnable:1;   
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
} MotorProtections01Admin_Flags;

typedef struct
{  
  uint16_t is_OverVoltageFault:1;   // TRUE if OV fault is active
  uint16_t is_UnderVoltageFault:1;  // TRUE if OV fault is active
  uint16_t is_OverCurrentFault:1;   // TRUE if OV fault is active
  uint16_t is_LossofInputPhaseFault:1;   // TRUE if Loss of input phase
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
} MotorProtections01User_Discretes;

struct MotorProtections01_Data
{
  // USER Access
  MotorProtections01User_Discretes userDiscretes01;
  uint16_t unusedData03; 
  uint16_t unusedData04; 
  uint16_t unusedData05; 
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10; 
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData13;
  uint16_t unusedData14;  
};

struct MotorProtections01_Settings
{
  // USER Access
  MotorProtections01User_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  
  // ADMIN Access
  MotorProtections01Admin_Flags adminFlags01;
  
  uint16_t overVoltageThreshold_u16;      // Over Voltage Threshold in %
  uint16_t overVoltageHysteresis_u16;     // Over Voltage Hysteresis in %
  uint16_t overVoltageMaxRetries_u16;     // Max retries before latching fault
  uint16_t overVoltageMinRetryDelay_u16;  // Delay between each retry. Each count is 100mSec
  uint16_t overVoltageMaxRetryDelay_u16;    // Max retries before latching fault. 0xFF is unlimited retries. Each count is 100mSec
  uint16_t overVoltageRetryDelayFactor_u16; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  uint16_t underVoltageThreshold_u16;     // Under Voltage Threshold in %
  uint16_t underVoltageHysteresis_u16;    // Under Voltage Hysteresis in %
  uint16_t underVoltageMaxRetries_u16;    // Max retries before latching fault
  uint16_t underVoltageMinRetryDelay_u16; // Delay between each retry. Each count is 100mSec
  uint16_t underVoltageMaxRetryDelay_u16;    // Max retries before latching fault. 0xFF is unlimited retries. Each count is 100mSec
  uint16_t underVoltageRetryDelayFactor_u16; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  uint16_t overCurrentThreshold_u16;      // Over Current threshold in %
  uint16_t overCurrentHysteresis_u16;     // Over Current Hysteresis in %
  uint16_t overCurrentMaxRetries_u16;     // Max retries before latching fault
  uint16_t overCurrentMinRetryDelay_u16;  // Delay between each retry. Each count is 100mSec
  uint16_t overCurrentMaxRetryDelay_u16;    // Max retries before latching fault. 0xFF is unlimited retries. Each count is 100mSec
  uint16_t overCurrentRetryDelayFactor_u16; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
  
  uint16_t lossOfInputPhaseMaxRetries_u16;      // Max retries before latching fault
  uint16_t lossOfInputPhaseMinRetryDelay_u16;   // Delay between each retry. Each count is 100mSec
  uint16_t lossOfInputPhaseMaxRetryDelay_u16;   // Max retries before latching fault. 0xFF is unlimited retries. Each count is 100mSec
  uint16_t lossOfInputPhaseRetryDelayFactor_u16; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1) 
  uint16_t lossOfInputPhaseMinPower_u16;         // Min Power required to measure ripple
  uint16_t lossOfInputPhaseMaxPower_u16;         // Max allowed power during loss of phase.
  uint16_t unusedSettings29;
  uint16_t unusedSettings30; // Added to prevent padding for flags
  // MAX settings(60byteS) reached per module
};


typedef struct
{
  struct MotorProtections01_Settings motorProtections01_Settings;
  struct MotorProtections01_Data motorProtections01_Data;  
}MotorProtections01_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorProtections01_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorProtections01_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Protection01" Parameters //////////////////////////////

//////////// Start of "Protection02" Parameters //////////////////////////////
typedef struct
{ 
  // USER Access
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
} MotorProtections02User_Flags1;
typedef struct
{ 
  // ADMIN Access
  uint16_t is_ipmOverTemperatureEnable:1;  
  uint16_t is_speedFeedBackFaultEnable:1;
  uint16_t is_startUpFaultEnable:1; 
  uint16_t is_softwareFaultEnable:1; 
  uint16_t is_focFaultEnable:1;
  uint16_t is_pfcOverTemperatureEnable:1;
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
} MotorProtections02Admin_Flags;

typedef struct
{  
  uint16_t is_IpmOverTemperatureFault:1; // TRUE if IPM temperature is over limit
  uint16_t is_speedFeedBackFault:1;
  uint16_t is_startUpFault:1; 
  uint16_t is_softwareFault:1; 
  uint16_t is_focFault:1;
  uint16_t is_pfcOverTemperatureFault:1;
  uint16_t unusedDiscretes11:1; 
  uint16_t unusedDiscretes12:1;
  uint16_t unusedDiscretes13:1; 
  uint16_t unusedDiscretes14:1;
  uint16_t unusedDiscretes15:1;
  uint16_t unusedDiscretes16:1;
} MotorProtections02User_Discretes;

struct MotorProtections02_Data
{
  // USER Access
  MotorProtections02User_Discretes userDiscretes01;
  uint16_t unusedData03; 
  uint16_t unusedData04; 
  uint16_t unusedData05; 
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10; 
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData13;
  uint16_t unusedData14;  
};

struct MotorProtections02_Settings
{
  // USER Access
  MotorProtections02User_Flags1 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  
  // ADMIN Access
  MotorProtections02Admin_Flags adminFlags01;
  
  uint16_t ipmOverTemperatureThreshold_u16;  //Over Temperature Threshold
  uint16_t ipmOverTemperatureHysteresis_u16; // Over Temperature Hysteresis
  uint16_t ipmOverTemperatureMaxRetries_u16;       // Max retries before latching fault
  uint16_t ipmOverTemperatureMinRetryDelay_u16;   // Delay between each retry
  uint16_t ipmOverTemperatureMaxRetryDelay_u16;    // Max retries before latching fault. 0xFF is unlimited retries
  uint16_t ipmOverTemperatureRetryDelayFactor_u16; // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)
 
  uint16_t pfcOverTemperatureThreshold_u16;  //Over Temperature Threshold
  uint16_t pfcOverTemperatureHysteresis_u16; // Over Temperature Hysteresis
  uint16_t pfcOverTemperatureMaxRetries_u16;      // Max retries before latching fault
  uint16_t pfcOverTemperatureMinRetryDelay_u16;   // Delay between each retry
  uint16_t pfcOverTemperatureMaxRetryDelay_u16;   // Max retries before latching fault. 0xFF is unlimited retries
  uint16_t pfcOverTemperatureRetryDelayFactor_u16;// 39 // Used to calcuated delay for next retry. Delay = minDelay + delayFactor * ( retry# - 1)  
  
  uint16_t unusedSettings18;
  uint16_t unusedSettings19;
  uint16_t unusedSettings20; // Added to prevent padding for flags
};


typedef struct
{
  struct MotorProtections02_Settings motorProtections02_Settings;
  struct MotorProtections02_Data motorProtections02_Data;  
}MotorProtections02_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct MotorProtections02_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"MotorProtections02_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Protection02" Parameters //////////////////////////////

//////////// Start of "Braking" Parameters ///////////////////////////

typedef struct
{ 
  // USER Access
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
} BrakingParametersUser_Flags01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_decelControlEnable:1; // Set to "1" to enable decel control  
  uint16_t is_brakingControlEnable:1; // Set to "1" to enable braking control
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
} BrakingParametersAdmin_Flags01;

typedef struct
{ 
  // USER Access
  uint16_t is_decelControlActive:1;   
  uint16_t is_brakingActive:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} BrakingParametersUser_Discretes01;

struct BrakingParameters_Data
{
  // USER Access
  BrakingParametersUser_Discretes01 userDiscretes01;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData11;
  uint16_t unusedData12; 
};

struct BrakingParameters_Settings
{
  // USER Access
  BrakingParametersUser_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  
  // ADMIN Access
  BrakingParametersAdmin_Flags01 adminFlags01;
  uint16_t lowSideOnDuration_u16;    // Low side on Duration
  uint16_t brakeEndSpeed_u16;        // Brake End Speed
  uint16_t brakeCurrentSeeding_u16;  // Brake Current seeding
  uint16_t brakingMaxVbus_u16; // Vbus is clipped to this % of nominal Vbus
  int16_t  brakingRampA_s16;   // Ramp a
  int16_t  brakingRampB_s16;   // Ramp b
  int16_t  brakingRampC_s16;   // Ramp c
  uint16_t brakingRampEndCurrent_u16; // Ramp end current
  uint16_t brakeSpeedTranstionToBrake_u16; // Speed Transition to Brake
  uint16_t brakeCurrentRampStep_u16; // Current ramp step
  uint16_t brakeLowCurrentLimit_u16; // Low current limit
//  uint16_t brakeKp_u16;
//  uint16_t brakeKpFactor_u16;
//  uint16_t brakeKi_u16;
//  uint16_t brakeKiFactor_u16;
//  uint16_t brakeKd_u16;
//  uint16_t brakeKdFactor_u16;
  uint16_t unusedSettings18;
  uint16_t unusedSettings19;
  uint16_t unusedSettings20; // Added to prevent padding for flags
};

typedef struct
{
  struct BrakingParameters_Settings brakingParameters_Settings;
  struct BrakingParameters_Data brakingParameters_Data;  
}BrakingParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct BrakingParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"BrakingParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Braking" Parameters ///////////////////////////

//////////// Start of "On the Fly" Parameters ///////////////////////////

typedef struct
{ 
  // USER Access
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
} OtfParametersUser_Flags01;

typedef struct
{ 
  // USER Access
  uint16_t is_otfControlEnable:1; // Set to "1" to enable OTF control 
  uint16_t is_windmillControlEnable:1; // Set to "1" to enable Windmill control 
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
} OtfParametersAdmin_Flags01;

typedef struct
{ 
  // USER Access
  uint16_t is_otfControlActive:1;
  uint16_t is_windmillActive:1;   
  uint16_t is_windmillReveseDir:1; 
  uint16_t unusedDiscretes04:1; 
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
} OtfParametersUser_Discretes01;

struct OtfParameters_Data
{
  // USER Access
  OtfParametersUser_Discretes01 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData11;
  uint16_t unusedData12;  
};

struct OtfParameters_Settings
{
  // USER Access
  OtfParametersUser_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  
  // ADMIN Access
  OtfParametersAdmin_Flags01 adminFlags01;
  uint16_t detectDuration_u16; // Detection phase duration // A_PHASE1_DURATION
  uint16_t detectBemfGain_u16; // DBEMFG
  uint16_t maxBemfGain_u16;    // Max BEMFG
  uint16_t minBemfGain_u16;    // Min BEMFG
  uint16_t maxSyncSpeed_u16;   // Max Sync Speed
  uint16_t minSyncSpeed_u16;   // Min Sync Speed
  uint16_t loadIntertia_u16;     // Fan/fan inertia
  uint16_t stopWaitTime_u16;   // Delay when OTF is enabled
  
  uint16_t wmMaxReverseSpeed_u16;   // Max Reverse Speed
  uint16_t wmClampDuration_u16;     // Clamp Duration
  uint16_t wmClampCurrent_u16;      // Clamp Current
  uint16_t wmMaxClampCurrent_u16;   // Max Clam current for windmilling
  uint16_t wmReverseEndSpeed_u16;   // Reverse End Speed
  uint16_t wmClampRamp_u16;         // Clamp Ramp
  uint16_t wmCoilShortDuration_u16; // Coil short Duration
  
  uint16_t unusedSettings22;
  uint16_t unusedSettings23;
  uint16_t unusedSettings24;   // Added to prevent padding for flags
};

typedef struct
{
  struct OtfParameters_Settings otfParameters_Settings;
  struct OtfParameters_Data otfParameters_Data;  
}OtfParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct OtfParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"OtfParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "On the Fly" Parameters ///////////////////////////

//////////// Start of "Windmill" Parameters ///////////////////////////

//typedef struct
//{ 
//  // USER Access
//  uint16_t unusedFlags01:1;   
//  uint16_t unusedFlags02:1; 
//  uint16_t unusedFlags03:1;
//  uint16_t unusedFlags04:1; 
//  uint16_t unusedFlags05:1; 
//  uint16_t unusedFlags06:1; 
//  uint16_t unusedFlags07:1; 
//  uint16_t unusedFlags08:1;
//  uint16_t unusedFlags09:1;
//  uint16_t unusedFlags10:1; 
//  uint16_t unusedFlags11:1;
//  uint16_t unusedFlags12:1; 
//  uint16_t unusedFlags13:1; 
//  uint16_t unusedFlags14:1;
//  uint16_t unusedFlags15:1;   
//  uint16_t unusedFlags16:1;
//} WindMillingParametersUser_Flags01;
//
//typedef struct
//{ 
//  // ADMIN Access
//  uint16_t is_windmillControlEnable:1; // Set to "1" to enable Windmill control    
//  uint16_t unusedFlags02:1; 
//  uint16_t unusedFlags03:1;
//  uint16_t unusedFlags04:1; 
//  uint16_t unusedFlags05:1; 
//  uint16_t unusedFlags06:1; 
//  uint16_t unusedFlags07:1; 
//  uint16_t unusedFlags08:1;
//  uint16_t unusedFlags09:1;   
//  uint16_t unusedFlags10:1; 
//  uint16_t unusedFlags11:1;
//  uint16_t unusedFlags12:1; 
//  uint16_t unusedFlags13:1; 
//  uint16_t unusedFlags14:1;
//  uint16_t unusedFlags15:1;   
//  uint16_t unusedFlags16:1;
//} WindMillingParametersAdmin_Flags01;
//
//typedef struct
//{ 
//  // USER Access
//  uint16_t is_windmillActive:1;   
//  uint16_t is_windmillReveseDir:1; 
//  uint16_t unusedDiscretes03:1; 
//  uint16_t unusedDiscretes04:1; 
//  uint16_t unusedDiscretes05:1; 
//  uint16_t unusedDiscretes06:1;
//  uint16_t unusedDiscretes07:1; 
//  uint16_t unusedDiscretes08:1;  
//  uint16_t unusedDiscretes09:1;
//  uint16_t unusedDiscretes10:1;
//  uint16_t unusedDiscretes11:1; 
//  uint16_t unusedDiscretes12:1;
//  uint16_t unusedDiscretes13:1; 
//  uint16_t unusedDiscretes14:1; 
//  uint16_t unusedDiscretes15:1;
//  uint16_t unusedDiscretes16:1;
//} WindMillingParametersUser_Discretes01;
//
//struct WindMillingParameters_Data
//{
//  // USER Access
//  WindMillingParametersUser_Discretes01 userDiscretes01;
//  uint16_t unusedData02;
//  uint16_t unusedData03;
//  uint16_t unusedData04;
//  uint16_t unusedData05;
//  
//  // ADMIN Access
//  uint16_t unusedAdminDiscretes01;
//  uint16_t unusedData07;
//  uint16_t unusedData08;
//};
//
//struct WindMillingParameters_Settings
//{
//  // USER Access
//  WindMillingParametersUser_Flags01 userFlags01;
//  uint16_t unusedSettings02;
//  uint16_t unusedSettings03;
//  uint16_t unusedSettings04;
//  uint16_t unusedSettings05;
//  uint16_t unusedSettings06;
//  uint16_t unusedSettings07;
//  
//  // ADMIN Access
//  WindMillingParametersAdmin_Flags01 adminFlags01;
//  uint16_t wmMaxReverseSpeed_u16;   // Max Reverse Speed
//  uint16_t wmClampDuration_u16;     // Clamp Duration
//  uint16_t wmClampCurrent_u16;      // Clamp Current
//  uint16_t wmMaxClampCurrent_u16;   // Max Clam current for windmilling
//  uint16_t wmReverseEndSpeed_u16;   // Reverse End Speed
//  uint16_t wmClampRamp_u16;         // Clamp Ramp
//  uint16_t wmCoilShortDuration_u16; // Coil short Duration
//  uint16_t unusedSettings16;  // Added to prevent padding for flags
//};
//
//typedef struct
//{
//  struct WindMillingParameters_Settings windMillingParameters_Settings;
//  struct WindMillingParameters_Data windMillingParameters_Data;  
//}WindMillingParameters_Control;
//
//// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
//// Note "Padding" would affect the struct size
//static_assert( ( sizeof(struct WindMillingParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"WindMillingParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Windmill" Parameters ///////////////////////////

//////////// Start of "Hardware Specific" Parameters /////////////////
typedef struct
{ 
  // USER Access
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
} HardwareSpecificParametersUser_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} HardwareSpecificParametersUser_Discretes01;

struct HardwareSpecificParameters_Data
{
  // USER Access
  HardwareSpecificParametersUser_Discretes01 userDiscretes01;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData13;
  uint16_t unusedData14; 
};

struct HardwareSpecificParameters_Settings
{
  // USER Access
  HardwareSpecificParametersUser_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01;
  uint16_t nominalVbusVolts_u16; // Vbus nominal voltage
  uint16_t iaCalSlope_u16;       // Ia Cal Slope
  uint16_t iaSlopeFactor_u16;    // Ia slope factor
  uint16_t iaCalOffset_u16;      // Ia Cal Offset
  uint16_t ibCalSlope_u16;       // Ib Cal Slope
  uint16_t ibSlopeFactor_u16;    // Ib slope factor
  uint16_t ibCalOffset_u16;      // Ib Cal Offset
  uint16_t vBusCalSlope_u16;     // Vbus Cal Slope
  uint16_t vBusSlopeFactor_u16;  // Vbus slope factor
  uint16_t vBusCalOffset_u16;    // Vbus Cal Offset
  uint16_t ipmCalSlope_u16;      // IPM Temp Cal Slope
  uint16_t ipmSlopeFactor_u16;   // IPM slope factor
  uint16_t ipmCalOffset_u16;     // IPM Temp Cal Offset
  uint16_t pfcCalSlope_u16;      // PFC Temp Cal Slope
  uint16_t pfcSlopeFactor_u16;   // PFC Temp slope factor
  uint16_t pfcCalOffset_u16;     // PFC temp Cal Offset
  uint16_t unusedSettings25;
  uint16_t unusedSettings26;     // Added to prevent padding
};

typedef struct
{
  struct HardwareSpecificParameters_Settings hardwareSpecificParameters_Settings;
  struct HardwareSpecificParameters_Data hardwareSpecificParameters_Data;  
}HardwareSpecificParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct HardwareSpecificParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"HardwareSpecificParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Hardware Specific" Parameters /////////////////

//////////// Start of "Application Specific" Parameters /////////////////
typedef struct
{ 
  // USER Access
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
} ApplicationSpecificParametersUser_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} ApplicationSpecificParametersUser_Discretes01;

struct ApplicationSpecificParameters_Data
{
  // USER Access
  ApplicationSpecificParametersUser_Discretes01 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData12;
  uint16_t unusedData14;  
};

struct ApplicationSpecificParameters_Settings
{
  // USER Access
  ApplicationSpecificParametersUser_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  uint16_t unusedSettings08;
  uint16_t unusedSettings09;
  
  // ADMIN Access
  uint16_t unusedAdminFlags01;
  uint16_t controlMode_u16;    // Control Mode, Speed/Torque/Airflow/Sensor
  uint16_t minApplicationRpm_u16; // Min. Application RPM
  uint16_t maxApplicationRpm_u16; // Max. Application RPM
  uint16_t rampUpRate_u16;   // Acceleration RPM/Sec
  uint16_t rampDownRate_u16; // Deceleration RPM/Sec
  uint16_t stopSpeed_u16;    // Speed at which motor is considered stopped
  uint16_t directionOfRotation_u16;   // Direction of Rotation
  uint16_t minApplicationTorque_u16;  // Min Torque
  uint16_t maxApplictionTorque_u16;   // Max Torque
  uint16_t minApplicationAirflow_u16; // Min Airflow
  uint16_t maxApplicationAirflow_u16; // Max Airflow
  uint16_t motorSpinPollPeriod_u16;   // Poll time for speed 
  uint16_t motorSpinTimeout_u16;      // Timeout if degired speed is not achieved within this time
  uint16_t unusedSettings24;
  uint16_t unusedSettings25;
  uint16_t unusedSettings26;          // Added to prevent padding for flags
};

typedef struct
{
  struct ApplicationSpecificParameters_Settings applicationSpecificParameters_Settings;
  struct ApplicationSpecificParameters_Data applicationSpecificParameters_Data;  
}ApplicationSpecificParameters_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct ApplicationSpecificParameters_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"ApplicationSpecificParameters_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Application Specific" Parameters /////////////////

//////////// Start of "Harmonic Compensation" Parameters ////////////////

typedef struct
{ 
  // USER Access
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
} HarmonicCompensation01User_Flags01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_harmonicCompensationModule1Enable:1; // 5-8 HC module enable
  uint16_t is_harmonicCompensation1Enable:1; // HC 1 enable
  uint16_t is_phaseInvert1Enable:1;            // Phase invert 1 enable
  uint16_t is_harmonicCompensation2Enable:1; // HC 2 enable
  uint16_t is_phaseInvert2Enable:1;            // Phase invert 2 enable
  uint16_t is_harmonicCompensation3Enable:1; // HC 3 enable
  uint16_t is_phaseInvert3Enable:1;            // Phase invert 3 enable
  uint16_t is_harmonicCompensation4Enable:1; // HC 4 enable
  uint16_t is_phaseInvert4Enable:1;            // Phase invert 4 enable
  uint16_t unusedFlags10:1;   
  uint16_t unusedFlags11:1; 
  uint16_t unusedFlags12:1; 
  uint16_t unusedFlags13:1;
  uint16_t unusedFlags14:1;   
  uint16_t unusedFlags15:1; 
  uint16_t unusedFlags16:1; 
} HarmonicCompensation01Admin_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} HarmonicCompensation01User_Discretes01;

struct HarmonicCompensation01_Data
{
  // USER Access
  HarmonicCompensation01User_Discretes01 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData13;
  uint16_t unusedData14;
};

struct HarmonicCompensation01_Settings
{
  // USER Access
  HarmonicCompensation01User_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  
  // ADMIN Access
  HarmonicCompensation01Admin_Flags01 adminFlags01;
  uint16_t angleOffsets1_u16; // Angle Offsets
  uint16_t angleMultlipliers1_u16; // Angle Multipliers
  uint16_t amplitudes1_u16;   // Amplitudes
  uint16_t minSpeeds1_u16;    // Min Speeds
  uint16_t maxSpeeds1_u16;    // Max Speeds
  
  uint16_t angleOffsets2_u16; // Angle Offsets
  uint16_t angleMultlipliers2_u16; // Angle Multipliers
  uint16_t amplitudes2_u16;   // Amplitudes
  uint16_t minSpeeds2_u16;    // Min Speeds
  uint16_t maxSpeeds2_u16;    // Max Speeds
  
  uint16_t angleOffsets3_u16; // Angle Offsets
  uint16_t angleMultlipliers3_u16; // Angle Multipliers
  uint16_t amplitudes3_u16;   // Amplitudes
  uint16_t minSpeeds3_u16;    // Min Speeds
  uint16_t maxSpeeds3_u16;    // Max Speeds
  
  uint16_t angleOffsets4_u16; // Angle Offsets
  uint16_t angleMultlipliers4_u16; // Angle Multipliers
  uint16_t amplitudes4_u16;   // Amplitudes
  uint16_t minSpeeds4_u16;    // Min Speeds
  uint16_t maxSpeeds4_u16;    // Max Speeds
  uint16_t unusedSettings29;  // 
  uint16_t unusedSettings30;  // Added to prevent padding for flags
  // Max settings (60 bytes) reached
};

typedef struct
{
  struct HarmonicCompensation01_Settings harmonicCompensation01_Settings;
  struct HarmonicCompensation01_Data harmonicCompensation01_Data;  
}HarmonicCompensation01_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct HarmonicCompensation01_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"HarmonicCompensation01_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );


typedef struct
{ 
  // USER Access
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
} HarmonicCompensation02User_Flags01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_harmonicCompensationModule2Enable:1; // 5-8 HC module enable
  uint16_t is_harmonicCompensation5Enable:1; // HC 5 enable
  uint16_t is_phaseInvert5Enable:1;            // Phase invert 5 enable
  uint16_t is_harmonicCompensation6Enable:1; // HC 6 enable
  uint16_t is_phaseInvert6Enable:1;            // Phase invert 6 enable
  uint16_t is_harmonicCompensation7Enable:1; // HC 7 enable
  uint16_t is_phaseInvert7Enable:1;            // Phase invert 7 enable
  uint16_t is_harmonicCompensation8Enable:1; // HC 8 enable
  uint16_t is_phaseInvert8Enable:1;            // Phase invert 8 enable
  uint16_t unusedFlags10:1;   
  uint16_t unusedFlags11:1; 
  uint16_t unusedFlags12:1; 
  uint16_t unusedFlags13:1;
  uint16_t unusedFlags14:1;   
  uint16_t unusedFlags15:1; 
  uint16_t unusedFlags16:1; 
} HarmonicCompensation02Admin_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} HarmonicCompensation02User_Discretes01;

struct HarmonicCompensation02_Data
{
  // USER Access
  HarmonicCompensation02User_Discretes01 userDiscretes01;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedData12;
  uint16_t unusedData13;
  uint16_t unusedData14;
};

struct HarmonicCompensation02_Settings
{
  // USER Access
  HarmonicCompensation02User_Flags01 userFlags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  
  // ADMIN Access
  HarmonicCompensation02Admin_Flags01 adminFlags01;
  
  uint16_t angleOffsets5_u16; // Angle Offsets
  uint16_t angleMultlipliers5_u16; // Angle Multipliers
  uint16_t amplitudes5_u16;   // Amplitudes
  uint16_t minSpeeds5_u16;    // Min Speeds
  uint16_t maxSpeeds5_u16;    // Max Speeds
  
  uint16_t angleOffsets6_u16; // Angle Offsets
  uint16_t angleMultlipliers6_u16; // Angle Multipliers
  uint16_t amplitudes6_u16;   // Amplitudes
  uint16_t minSpeeds6_u16;    // Min Speeds
  uint16_t maxSpeeds6_u16;    // Max Speeds
  
  uint16_t angleOffsets7_u16; // Angle Offsets
  uint16_t angleMultlipliers7_u16; // Angle Multipliers
  uint16_t amplitudes7_u16;   // Amplitudes
  uint16_t minSpeeds7_u16;    // Min Speeds
  uint16_t maxSpeeds7_u16;    // Max Speeds
  
  uint16_t angleOffsets8_u16; // Angle Offsets
  uint16_t angleMultlipliers8_u16; // Angle Multipliers
  uint16_t amplitudes8_u16;   // Amplitudes
  uint16_t minSpeeds8_u16;    // Min Speeds
  uint16_t maxSpeeds8_u16;    // Max Speeds
  uint16_t unusedSettings29;  // 
  uint16_t unusedSettings30;  // Added to prevent padding
  // Max settings (60 bytes) reached
};

typedef struct
{
  struct HarmonicCompensation02_Settings harmonicCompensation02_Settings;
  struct HarmonicCompensation02_Data harmonicCompensation02_Data;  
}HarmonicCompensation02_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct HarmonicCompensation02_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"HarmonicCompensation02_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

typedef struct
{ 
  // USER Access
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
} HarmonicCompensation03User_Flags01;

typedef struct
{ 
  // ADMIN Access
  uint16_t is_harmonicCompensationModule3Enable:1; // 9-12 HC module enable
  uint16_t is_harmonicCompensation9Enable:1;  // HC 9 enable
  uint16_t is_phaseInvert9Enable:1;             // Phase invert 9 enable
  uint16_t is_harmonicCompensation10Enable:1; // HC 10 enable
  uint16_t is_phaseInvert10Enable:1;            // Phase invert 10 enable
  uint16_t is_harmonicCompensation11Enable:1; // HC 11 enable
  uint16_t is_phaseInvert11Enable:1;            // Phase invert 11 enable
  uint16_t is_harmonicCompensation12Enable:1; // HC 12 enable
  uint16_t is_phaseInvert12Enable:1;            // Phase invert 12 enable
  uint16_t unusedFlags10:1;   
  uint16_t unusedFlags11:1; 
  uint16_t unusedFlags12:1; 
  uint16_t unusedFlags13:1;
  uint16_t unusedFlags14:1;   
  uint16_t unusedFlags15:1; 
  uint16_t unusedFlags16:1; 
} HarmonicCompensation03Admin_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} HarmonicCompensation03User_Discretes01;

struct HarmonicCompensation03_Data
{
  // USER Access
  HarmonicCompensation03User_Discretes01 userDiscretes01;
  uint16_t unusedData02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  uint16_t unusedData09;
  uint16_t unusedData10;
  
  // ADMIN Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData13;
  uint16_t unusedData14;
};

struct HarmonicCompensation03_Settings
{
  // USER Access
  HarmonicCompensation03User_Flags01 userFlags01;
  uint16_t unusedSettings02;  
  uint16_t unusedSettings03;
  uint16_t unusedSettings04;
  uint16_t unusedSettings05;
  uint16_t unusedSettings06;
  uint16_t unusedSettings07;
  
  // ADMIN Access  
  HarmonicCompensation03Admin_Flags01 adminFlags01;
  
  uint16_t angleOffsets9_u16; // Angle Offsets
  uint16_t angleMultlipliers9_u16; // Angle Multipliers
  uint16_t amplitudes9_u16;   // Amplitudes
  uint16_t minSpeeds9_u16;    // Min Speeds
  uint16_t maxSpeeds9_u16;    // Max Speeds
  
  uint16_t angleOffsets10_u16; // Angle Offsets
  uint16_t angleMultlipliers10_u16; // Angle Multipliers
  uint16_t amplitudes10_u16;   // Amplitudes
  uint16_t minSpeeds10_u16;    // Min Speeds
  uint16_t maxSpeeds10_u16;    // Max Speeds
  
  uint16_t angleOffsets11_u16; // Angle Offsets
  uint16_t angleMultlipliers11_u16; // Angle Multipliers
  uint16_t amplitudes11_u16;   // Amplitudes
  uint16_t minSpeeds11_u16;    // Min Speeds
  uint16_t maxSpeeds11_u16;    // Max Speeds
  
  uint16_t angleOffsets12_u16; // Angle Offsets
  uint16_t angleMultlipliers12_u16; // Angle Multipliers
  uint16_t amplitudes12_u16;   // Amplitudes
  uint16_t minSpeeds12_u16;    // Min Speeds
  uint16_t maxSpeeds12_u16;    // Max Speeds
  uint16_t unusedSettings29;   // 
  uint16_t unusedSettings30;   // Added to prevent padding for flags
  // Max Settings (60 bytes) reached
};

typedef struct
{
  struct HarmonicCompensation03_Settings harmonicCompensation03_Settings;
  struct HarmonicCompensation03_Data harmonicCompensation03_Data;  
}HarmonicCompensation03_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct HarmonicCompensation03_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"HarmonicCompensation03_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "Harmonic Compensation" Parameters ////////////////

//////////// Start of "ST MC Specific Paramters 1" Parameters ////////////////
typedef struct
{ 
  // USER Access
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
} StParameters01User_Flags01;

typedef struct
{  
  // USER Access
  uint16_t unusedDiscretes01:1;   
  uint16_t unusedDiscretes02:1; 
  uint16_t unusedDiscretes03:1; 
  uint16_t unusedDiscretes04:1; 
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
} StParameters01User_Discretes01;

struct StParameters01_Data
{
  StParameters01User_Discretes01 userDiscretes01;
  uint16_t unusedUserDiscretes02;
  uint16_t unusedData03;
  uint16_t unusedData04;
  uint16_t unusedData05;
  uint16_t unusedData06;
  uint16_t unusedData07;
  uint16_t unusedData08;
  
  // Admin Access
  uint16_t unusedAdminDiscretes01;
  uint16_t unusedAdminDiscretes02;
  uint16_t unusedData11;
  uint16_t unusedData12;
};

struct StParameters01_Settings
{
  // USER Access
  uint16_t StParameters01User_Flags01;
  uint16_t unusedSettings02;
  uint16_t unusedSettings03;
  uint16_t unusedSettings04; 
  
  // ADMIN Access
  uint16_t unusedAdminFlags01;
  //uint16_t slAlgoEnableFromPhase_u16; // ENABLE_SL_ALGO_FROM_PHASE // User start-up flag
  uint16_t phase1Speed_u16;         // RPM, PHASE1_FINAL_SPEED_UNIT * (_RPM/SPEED_UNIT)
  uint16_t phase2Speed_u16;         // RPM, PHASE2_FINAL_SPEED_UNIT * (_RPM/SPEED_UNIT)
  //uint16_t phase4Speed_u16;         // RPM, PHASE4_FINAL_SPEED_UNIT * (_RPM/SPEED_UNIT)// Second Allignemnt in startup
  uint16_t phase1FinalCurrent_u16;  // PHASE1_FINAL_CURRENT  
  uint16_t phase2FinalCurrent_u16;  // PHASE2_FINAL_CURRENT
  uint16_t nbConsecutiveTest_u16;   // NB_CONSECUTIVE_TESTS
  uint16_t pwmFreqScaling_u16;      // PWM_FREQ_SCALING
  uint16_t unusedSettings12;        // Added to prevent padding for flags 
};

typedef struct
{
  struct StParameters01_Settings stParameters01_Settings;
  struct StParameters01_Data stParameters01_Data;  
}StParameters01_Control;

// Assert compiler error when size of stuct > MAX_MODULE_STRUCTURE_SIZE_BYTES
// Note "Padding" would affect the struct size
static_assert( ( sizeof(struct StParameters01_Settings) <= MAX_MODULE_STRUCTURE_SIZE_BYTES) ,"StParameters01_Settings SIZE GREATER THEN MAX_MODULE_STRUCTURE_SIZE_BYTES" );

//////////// End of "ST MC Specific Paramters 1" Parameters ////////////////

void init_flash_parmeters(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _FLASH_PARAMETERS_H_ */