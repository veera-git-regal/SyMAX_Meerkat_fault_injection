/**
  ******************************************************************************
  * @file    harmonic_injection.c
  * @author First Last
  * @brief   This file provides firmware functions that implement the calculations
  *          for noise reduction by providing compensations for Phase A and Phase
  *          B currents.
  *          -Accessors for injection currents
  *          -Accessors for injection on/off flags
  *          -Accessors for injection parameters
  *          -Call to update injection current values
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "harmonic_injection.h"

#include "mc_math.h"
#include "mc_api.h"

/* Private Methods------------------------------------------------------------*/
 void Calculate(uint16_t fundamental_angle_u16);  
    
/* Constants------------------------------------------------------------------*/
// if set over this value, and harmonic component will be ignored
const int16_t HarmonicInjectionMaxAmplitude_s16 = 3000;

/* Private Variables----------------------------------------------------------*/
// Configurable/ Flashable
uint8_t is_harmonic_injection_allowed_u8 = 1; 
int16_t amplitude_s16[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_AMPLITUDE_DEFAULTS; 
uint8_t angle_multiplier_u8[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_MULTIPLIER_DEFAULTS; 
uint16_t angle_offset_u16[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_ANGLE_DEFAULTS;
uint8_t is_phase_inverted_u8[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_INVERSION_DEFAULTS;
int16_t min_speed_s16[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_MIN_SPEED_DEFAULTS;
int16_t max_speed_s16[HARMONIC_INJ_MAX_HARMONICS] = HARMONIC_INJ_MAX_SPEED_DEFAULTS;

// Realtime Values
int32_t current_d_adjustment_u32 = 0;
int32_t current_q_adjustment_u32 = 0;
uint8_t is_harmonic_injection_enabled_u8 = 1;
uint8_t p_harmonic_injection_parameters_rx_u8[HARMONIC_COMP_CONFIG_LENGTH_BYTES];

// TODO: Review use of float multiplier to allow more specific control
// TODO: Review use of u8 for phase, as angle seems to be iq16, meaning a phase offset of 256 is barely anything
// - example code uses a left shift of 22

/**
  * @brief  Called periodically after PID injection is calculated
  * @param uint16_t fundamental_angle motor rotor angle in s16degrees
  *        90 degrees is 16384, 180 degrees is 32767 etc.
  * @retval none
  * @note 
  */
void HarmonicInjection_Calculate(uint16_t fundamental_angle_u16){
  // Reset Adjustment Values
  current_d_adjustment_u32 = 0;
  current_q_adjustment_u32 = 0;
  if(is_harmonic_injection_enabled_u8){
    Calculate(fundamental_angle_u16);
  }
}

/**
  * @brief  Called periodically after PID injectino is calculated
  * @param uint16_t fundamental_angle motor rotor angle in s16degrees
  *        90 degrees is 16384, 180 degrees is 32767 etc.
  * @retval none
  * @note 
  */
void Calculate(uint16_t fundamental_angle_u16){
  Trig_Components Trig_Components_Results = {0,0};
  uint16_t harmonicCompensataion_u16 = 0;
  
  //#if(ARE_WE_TUNING == TUNING_HI_ON)
//  HarmonicInjection_UpdateSettings(p_harmonic_injection_parameters_rx_u8);  
  //#endif
  
  #if HARMONIC_ADJUSTMENT_DEBUG_ONLY_ONE_HARMONIC >= 1
  for(uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < 2; this_harmonic_u8++)  // this algorithm allows debugging of a single harmonic
  #else
  for(uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) 
  #endif
  {
    // Check if is in speed range for this harmonic. need speed demand in rpm
    int16_t current_speed_s16 = MC_GetLastRampFinalSpeedMotor1(); 
//uint16_t current_speed_u16 = MC_GetMecSpeedAverageMotor1();
    // Value returned from MC_GetLastRampFinalSpeedMotor1() is in 10th of Hz

	current_speed_s16 = current_speed_s16 * 6; // 5 pole pairs = 10 poles but also 0.1Hz is another factor of 10.  n[rpm] = 120*f[Hz]/P where P = number of poles
	if(current_speed_s16 < 0)
	  current_speed_s16 = -current_speed_s16;	// takes the absolute value if motor is spinning CCW
	if (current_speed_s16 < min_speed_s16[this_harmonic_u8]) {
      continue; 
    } else if (current_speed_s16 > max_speed_s16[this_harmonic_u8]) {
      continue; 
    }
	
    // Check for configurations that may generate errors, or signify that this harmonic should be disabled
    if (angle_multiplier_u8[this_harmonic_u8] <= 0 || amplitude_s16[this_harmonic_u8] == 0 || 
                 amplitude_s16[this_harmonic_u8] > HarmonicInjectionMaxAmplitude_s16){
      continue;
    } 

 
    #if HARMONIC_COMPENSATION_PHASE_BEFORE_MULTIPLIER == 0
    uint16_t this_multiplier_u16 = angle_multiplier_u8[this_harmonic_u8];
    uint16_t this_offset_u16 = angle_offset_u16[this_harmonic_u8];
    uint16_t this_angle_no_offset_u16 = fundamental_angle_u16 * this_multiplier_u16;
    uint16_t this_angle_with_offset_u16 = this_angle_no_offset_u16 + this_offset_u16;
    harmonicCompensataion_u16 = this_angle_with_offset_u16;
    #else
    uint16_t this_multiplier_u16 = angle_multiplier_u8[this_harmonic_u8];	
    uint16_t this_offset_u16 = angle_offset_u16[this_harmonic_u8];
    uint16_t this_angle_u16 = ( fundamental_angle_u16 + this_offset_u16)*this_multiplier_u16;
    // uint16_t this_angle_no_offset_u16 = ( fundamental_angle_u16*this_multiplier_u16);
    // uint16_t this_angle_with_offset = this_angle_no_offset_u16 + this_offset_u16;
    harmonicCompensataion_u16 = this_angle_u16;
    #endif

    // Calculate "Sine Wave" values at this angle for harmonic's angle using the Trig_Functions utility from the ODP Firmware
    Trig_Components_Results = MCM_Trig_Functions(harmonicCompensataion_u16); // calculate sine and cos values of the harmonic at this angle
    
    // Apply harmonic's amplitude factor to the sign wave
    // - note that the sine value is a signed 16-bit value
    // -- multiply sine by amplitude factors into a signed 32, it's value will be VERY large in many cases    
    // TODO: These amplitudes are too high, I think, but perhpas if they are just being 
    // - used as a comparison (iqAdj - lastIqAdj, for example), it works
    int32_t this_sin_val_u32 = (int32_t)(Trig_Components_Results.hSin) * amplitude_s16[this_harmonic_u8]; // TODO: Need to add a Division Factor
    int32_t this_cos_val_u32 = (int32_t)(Trig_Components_Results.hCos) * amplitude_s16[this_harmonic_u8]; // TODO: Need to add a division Factor

    // Invert Sine Wave value only if "Phase Invert" is enabled
    if (is_phase_inverted_u8[this_harmonic_u8]) {
      // - Note this differs from ECM just in that it multiplies -1 before multiplying the current scaling factor to this item
      this_sin_val_u32 = -1 * this_sin_val_u32;
    }

    // --- then divide by 32768 (to correct the sine value into it's desired format (a value from 0-1)
    current_q_adjustment_u32 = current_q_adjustment_u32 - (this_cos_val_u32 / 32768);  // TODO: Should be IQ Multiply
    current_d_adjustment_u32 = current_d_adjustment_u32 + (this_sin_val_u32 / 32768);  // TODO: Should be IQ Multiply
  }
}


/**
  * @brief  Enables harmonic injection if allowed
  * @param none
  * @retval none
  * @note Should be called after motor is stabilized. Calling this method 
  *       does not ensure that the injection is enabled. The module must be
  *       activated by writing a non-zero to the first byte using the method
  *       HarmonicInjection_UpdateSettings(). If a non-zero is placed in the
  *       corrisponding flag then calling this method will enable injection.
  */
void HarmonicInjection_Enable(void) {
  is_harmonic_injection_enabled_u8 = is_harmonic_injection_allowed_u8;
}

/**
  * @brief  Disables harmonic injectin
  * @param none
  * @retval none
  */
void HarmonicInjection_Disable(void) {
  is_harmonic_injection_enabled_u8 = 0;
}


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
void HarmonicInjection_GetSettings(uint8_t * p_tx_buffer_u8){
    uint8_t index = 0;

    // Enable/ Disable
    p_tx_buffer_u8[index++] = is_harmonic_injection_allowed_u8;

    // Amplitude
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value_U16 = ((uint16_t) amplitude_s16[this_harmonic_u8]);
        // big endian
        p_tx_buffer_u8[index++] = this_value_U16 >> 8 & 0xFF;
        p_tx_buffer_u8[index++] = this_value_U16 & 0xFF;
    }
    
    // Angle Multipliers (Harmonic Order)
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        p_tx_buffer_u8[index++] = angle_multiplier_u8[this_harmonic_u8];
    }
    
    // Angle Offsets
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value_U16 = ((uint16_t) angle_offset_u16[this_harmonic_u8]);
        // big endian
        p_tx_buffer_u8[index++] = this_value_U16 >> 8 & 0xFF;
        p_tx_buffer_u8[index++] = this_value_U16 & 0xFF;
    }
    
    // Phase Inversion
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        p_tx_buffer_u8[index++] = is_phase_inverted_u8[this_harmonic_u8];
    }  
    // Min Speeds in RPMs
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value_U16 = ((uint16_t) min_speed_s16[this_harmonic_u8]);
        // big endian
        p_tx_buffer_u8[index++] = this_value_U16 >> 8 & 0xFF;
        p_tx_buffer_u8[index++] = this_value_U16 & 0xFF;
    }  
    // Max Speeds in RPMs
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value_U16 = ((uint16_t) max_speed_s16[this_harmonic_u8]);
        // big endian
        p_tx_buffer_u8[index++] = this_value_U16 >> 8 & 0xFF;
        p_tx_buffer_u8[index++] = this_value_U16 & 0xFF;
    }  
}

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
void HarmonicInjection_UpdateSettings(uint8_t *p_rx_buffer_u8) {
    uint8_t index = 0;

    // Enable/ Disable
    is_harmonic_injection_allowed_u8 = p_rx_buffer_u8[index++];

    // Amplitude
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value = ((uint16_t)(p_rx_buffer_u8[index] << 8)) +  p_rx_buffer_u8[index+1];      // big endian
        amplitude_s16[this_harmonic_u8] = this_value;    // REVIEW: implicit conversion to int
        index += 2;
    }
    
    // Angle Multipliers (Harmonic Order)
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        angle_multiplier_u8[this_harmonic_u8] = p_rx_buffer_u8[index++];
    }
    
    // Angle Offsets
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        uint16_t this_value = ((uint16_t)(p_rx_buffer_u8[index] << 8)) +  p_rx_buffer_u8[index+1];      // big endian
        angle_offset_u16[this_harmonic_u8] = this_value;    // REVIEW: implicit conversion to int
        index += 2;
    }
    
    // Phase Inversion
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        is_phase_inverted_u8[this_harmonic_u8] = p_rx_buffer_u8[index++];
    }  

    // Min Speeds in RPMs
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        uint16_t this_value = ((uint16_t)(p_rx_buffer_u8[index] << 8)) +  p_rx_buffer_u8[index+1];      // big endian
        min_speed_s16[this_harmonic_u8] = (int16_t) this_value;
        index += 2;
    }  

    // Max Speeds in RPMs
    for (uint8_t this_harmonic_u8 = 0; this_harmonic_u8 < HARMONIC_INJ_MAX_HARMONICS; this_harmonic_u8++) {
        // big endian
        uint16_t this_value = ((uint16_t)(p_rx_buffer_u8[index] << 8)) +  p_rx_buffer_u8[index+1];      // big endian
        max_speed_s16[this_harmonic_u8] = (int16_t) this_value;
        index += 2;
    }  
}

/**
  * @brief  Accessor for phase A current
  * @param  none
  * @retval int32_t current_d_adjustment_u32
  */
int32_t HarmonicInjection_IdAdjustment(void){
  return current_d_adjustment_u32;
}
 
/**
  * @brief  Accessor for phase B current
  * @param  none
  * @retval int32_t current_q_adjustment_u32
  */
int32_t HarmonicInjection_IqAdjustment(void){
   return current_q_adjustment_u32;
}
    