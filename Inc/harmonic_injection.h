/**
**************************************************************
* @file harmonic_injection.h
* @author First Last
* @brief Interface for module that provides noise canceling currents.
**************************************************************
*/
#ifndef __HARMONIC_INJECTION_H__
#define __HARMONIC_INJECTION_H__

#include "mc_type.h"

#define DQALL	(0)
#define DQ011	(11)
#define DQ013	(13)
#define DQ015	(15)
#define DQ017	(17)
#define DQ019	(19)
#define DQ021	(21)
#define DQ023	(23)
#define DQ025	(25)
#define DQ029	(29)
#define	TUNE_HI_OFF	0
#define	TUNE_HI_ON	1

#define	SYMAX_SRI_MOTOR	DQ013
#define	ARE_WE_TUNING	TUNE_HI_OFF

#define HARMONIC_INJ_MAX_HARMONICS 8 //if this is changed, modify HARMONIC_INJ_DEFAULTS
//#if(SYMAX_SRI_MOTOR == DQALL)

// No Harmonics
/****/
#define HARMONIC_INJ_AMPLITUDE_DEFAULTS 	{0   ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MULTIPLIER_DEFAULTS 	{0    ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_ANGLE_DEFAULTS 		{0,0,0,0,0,0,0,0}
#define HARMONIC_INJ_INVERSION_DEFAULTS 	{0     ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MIN_SPEED_DEFAULTS 	{0 ,0,0,0,0,0,0,0} // minimum speed values to apply harmonic injection algorithm
#define HARMONIC_INJ_MAX_SPEED_DEFAULTS 	{2250  ,2250, 2250, 2250, 2250, 2250, 2250, 2250} // maximum speed values to apply harmonic injection algorithm
/****/

// DQ019 R03
/***
#define HARMONIC_INJ_AMPLITUDE_DEFAULTS 	{210   ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MULTIPLIER_DEFAULTS 	{12    ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_ANGLE_DEFAULTS 		{-5100 ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_INVERSION_DEFAULTS 	{0     ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MIN_SPEED_DEFAULTS 	{-2250 ,0,0,0,0,0,0,0} // minimum speed values to apply harmonic injection algorithm
#define HARMONIC_INJ_MAX_SPEED_DEFAULTS 	{2250  ,2250, 2250, 2250, 2250, 2250, 2250, 2250} // maximum speed values to apply harmonic injection algorithm
***/

// DQ013 R40
/***
#define HARMONIC_INJ_AMPLITUDE_DEFAULTS 	{300   ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MULTIPLIER_DEFAULTS 	{12    ,0,0,0,0,0,0,0}
#define HARMONIC_INJ_ANGLE_DEFAULTS 		{-20650,0,0,0,0,0,0,0}
#define HARMONIC_INJ_INVERSION_DEFAULTS 	{0     ,0,0,0,0,0,0,0}#define HARMONIC_INJ_MIN_SPEED_DEFAULTS 	{-2250 ,0,0,0,0,0,0,0} // minimum speed values to apply harmonic injection algorithm#define HARMONIC_INJ_MAX_SPEED_DEFAULTS 	{2250  ,2250, 2250, 2250, 2250, 2250, 2250, 2250} // maximum speed values to apply harmonic injection algorithm
***/

//For DQ013
/***
#define HARMONIC_INJ_AMPLITUDE_DEFAULTS 	{310,0,0,0,0,0,0,0}
#define HARMONIC_INJ_MULTIPLIER_DEFAULTS 	{12,0,0,0,0,0,0,0}#define HARMONIC_INJ_ANGLE_DEFAULTS 		{-14500,0,0,0,0,0,0,0}
#define HARMONIC_INJ_INVERSION_DEFAULTS 	{0,0,0,0,0,0*,0}
#define HARMONIC_INJ_MIN_SPEED_DEFAULTS 	{0,0,0,0,0,0,0,0} // minimum speed values to apply harmonic injection algorithm
#define HARMONIC_INJ_MAX_SPEED_DEFAULTS 	{2250, 2250, 2250, 2250, 2250, 2250, 2250, 2250} // maximum speed values to apply harmonic injection algorithm
****/
//#else
//#define HARMONIC_INJ_AMPLITUDE_DEFAULTS 	{310,0,0,0,0,0,0,0}
//#define HARMONIC_INJ_MULTIPLIER_DEFAULTS 	{12,0,0,0,0,0,0,0}
//#define HARMONIC_INJ_ANGLE_DEFAULTS 		{-14500,0,0,0,0,0,0,0}
//#define HARMONIC_INJ_INVERSION_DEFAULTS 	{0,0,0,0,0,0,0,0}
//#define HARMONIC_INJ_MIN_SPEED_DEFAULTS 	{0,0,0,0,0,0,0,0} // minimum speed values to apply harmonic injection algorithm
//#define HARMONIC_INJ_MAX_SPEED_DEFAULTS 	{2250, 2250, 2250, 2250, 2250, 2250, 2250, 2250} // maximum speed values to apply harmonic injection algorithm
//#endif

#define PHASE_ADJUSTMENT_SHIFT 7
#define ANGLE_MASK 0xFFFF 
#define HARMONIC_ADJUSTMENT_DEBUG_ONLY_ONE_HARMONIC 1 // was 1 when ported over
//#define HARMONIC_COMPENSATION_PHASE_BEFORE_MULTIPLIER 0 // 1 did not do anything with HI
//(amplitude_s16=2 + angle_multiplier_u8=1 + angle_offset_u16=2 + 
//phase_invert_u8=1 + min_speed_u16=2x + max_speeds_u16=2x) = 10 (used below)
//is_harmonic_injection_allowed_u8 = 1 (used below)

#define HARMONIC_COMP_CONFIG_LENGTH_BYTES ((10 * HARMONIC_INJ_MAX_HARMONICS) + 1) 
// #define HARMONIC_COMP_PI_OVER_TWO_TABLE_ADJ 0 // TODO

// - Public Functions
/**
  * @brief  Called periodically after PID injection is calculated
  * @param uint16_t fundamental_angle_u16 motor rotor angle in s16degrees
  *        90 degrees is 16384, 180 degrees is 32767 etc.
  * @retval none
  * @note 
  */
extern void HarmonicInjection_Calculate(uint16_t fundamental_angle_u16);

/**
  * @brief  Accessor to read harmonic injection values
  * @param  uint8_t p_tx_buffer_u8  reference to location of input variables: 
  *         Enable byte, Amplitude, Angle Multipliers, Angle Offsets, 
  *         Phase Inverstion, Minimum Speeds and Maximum Speeds.       
  * @retval none
  * @note !!Referenced array must have continuous memory for HARMONIC_INJ_MAX_HARMONICS + 1 uint8_t!! 
  *         The data will be placed at the memory reference in the following order:
  *         Enable Byte, 
  *         --Then-- 
  *         Amplitude MSByte, Amplitude LSByte, Amplitude Multiplier, 
  *         Angle Offset MSByte, Angle Offset LSByte, Phase INversion, 
  *         Min Speed MSByte, Min Speed LSByte, Max Speed MSByte, Max Speed LSB
  *         --The previous 10 bytes are repeated HARMONIC_INJ_MAX_HARMONICS  times--
  */
void HarmonicInjection_GetSettings(uint8_t * p_tx_buffer_u8); 

/**
  * @brief  Accessor to set harmonic injection values
  * @param  uint8_t *p_rx_buffer_u8 reference to location of input variables: 
  *         Amplitude, Angle Multipliers, Angle Offsets, Phase Inverstion,
  *         Minimum Speeds and Maximum Speeds
  * @param  uint8_t index offset into the array referenced by *p_rx_buffer_u8
  * @retval none
  * @note   The values referenced by the parameter must be in the following order:
  *         Enable Byte, 
  *         --Then-- 
  *         Amplitude MSByte, Amplitude LSByte, Amplitude Multiplier, 
  *         Angle Offset MSByte, Angle Offset LSByte, Phase INversion, 
  *         Min Speed MSByte, Min Speed LSByte, Max Speed MSByte, Max Speed LSB
  *         --The previous 10 bytes are repeated HARMONIC_INJ_MAX_HARMONICS  times--
  */
void HarmonicInjection_UpdateSettings(uint8_t * p_rx_buffer_u8); 

/**
  * @brief  Enables harmonic inject if module is activated
  * @param none
  * @retval none
  * @note Should be called after motor is stabilized. Calling this method 
  *       does not ensure that the injection is enabled. The module must be
  *       activated by writing a non-zero to the first byte using the method
  *       HarmonicInjection_UpdateSettings(). If a non-zero is placed in the
  *       corrisponding flag then calling this method will enable injection.
  */
void HarmonicInjection_Enable(void);

/**
  * @brief  Disables harmonic injection
  * @param none
  * @retval none
  */
void HarmonicInjection_Disable(void);

/**
  * @brief  Accessor for phase A current adjustment
  * @param  none
  * @retval int32_t current_d_adjustment_u32
  */
int32_t HarmonicInjection_IdAdjustment(void);

/**
  * @brief  Accessor for phase B current adjustment
  * @param  none
  * @retval int32_t current_q_adjustment_u32
  */
int32_t HarmonicInjection_IqAdjustment(void);


#endif // ndef __HARMONIC_INJECTION_H__


