/**
  ********************************************************************************************************************************
  * @file    module_voltage_doubler.c 
  * @author  Myron Mychal
  * @brief   This is based off the module_ap.c template
  * @details This module controls the behavior of the votlage doubling circuit in the Symax SRi
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_voltage_doubler.h"
#include "bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
#define	TIME_TO_ENGAGE_DOUBLER		500
#define	VBUS_TO_ENGAGE_DOUBLER		200
#define	TIME_TO_DISENGAGE_DOUBLER	500
#define VBUS_TO_DISENGAGE_DOUBLER	400

#define DemandPollPeriod 2000   
uint64_t tt_DemandDoublerTime;
Module_VoltageDoublerControl  module_VoltageDoublerControl;

//Usart1_Control* usart1Control_AppLocal;
uint64_t errorDoublerCounter = 0;
uint16_t doubler_comparison_voltage_u16 = 0;	// bus voltage to compare agaisnt for voltage doubler engagement
//bool isDoublerEngagementTiming = FALSE;
//bool isDoublerDisngagementTiming = FALSE;
//bool isDoublerEngaged = FALSE;

uint8_t tmpry4DoublerTest = true; /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
#define tmpDelayPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandtmpDoublerDelayTime;

enum AppStates {
    INIT_DOUBLER,
	BUS_IS_LOW_WITH_DOUBLER_DISENGAGED,
	BUS_IS_LOW_AND_ENGAGE_TIMING,
	BUS_IS_GOOD,
	BUS_IS_GOOD_AND_DISENGAGE_TIMING,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

uint8_t p_module_voltage_doubler_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8) {
    uint8_t return_state_u8 = 0;
	uint64_t		time_now_u64 = 0;
	static uint64_t	time_doubler_circuit_engages_u64 = 0;
	static uint64_t	time_doubler_circuit_disengages_u64 = 0;	
	static bool isDoublerEngagementTiming = FALSE;
	static bool isDoublerDisngagementTiming = FALSE;

    switch (next_State_u8) {
        case INIT_DOUBLER: {  
			LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
			/* GPIO Ports Clock Enable */
			//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

			// initialization for Voltage DOubler relay output pin
			GPIO_InitStruct.Pin 			= LL_GPIO_PIN_5;
			GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
			GPIO_InitStruct.Speed 			= LL_GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.OutputType 		= LL_GPIO_OUTPUT_PUSHPULL;
			GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
			LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);	// Doubler is off/disengaged
			module_VoltageDoublerControl.isDoublerEngaged = FALSE;
			module_VoltageDoublerControl.errorCode_u8 = 0;
            return_state_u8 = BUS_IS_LOW_WITH_DOUBLER_DISENGAGED;
            break;
		}
        case BUS_IS_LOW_WITH_DOUBLER_DISENGAGED: {
		  	doubler_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
		    if(doubler_comparison_voltage_u16 < VBUS_TO_ENGAGE_DOUBLER) {
			  // Doubling is needed because bus is too low.  Set timer for engagement
			  isDoublerEngagementTiming = TRUE;
			  time_doubler_circuit_engages_u64 = getSysCount() + TIME_TO_ENGAGE_DOUBLER;
			  return_state_u8 = BUS_IS_LOW_AND_ENGAGE_TIMING;
		    }
			else {
			  isDoublerEngagementTiming = FALSE;
			  time_doubler_circuit_engages_u64 = 0;
			  time_doubler_circuit_disengages_u64 = 0;
			  return_state_u8 = BUS_IS_GOOD;
			}
            break;
		}
		case BUS_IS_LOW_AND_ENGAGE_TIMING: {
		  	// as long as bus stays over threshold, keep timing
		  	time_now_u64 = getSysCount();
			doubler_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(doubler_comparison_voltage_u16 > VBUS_TO_ENGAGE_DOUBLER) {
			  // bus is good enough and we can reset timer - did not need doubler
			  isDoublerEngagementTiming = FALSE;
			  time_doubler_circuit_engages_u64 = 0;
			  time_doubler_circuit_disengages_u64 = 0;
			  return_state_u8 = BUS_IS_GOOD;
			}
		  	if ((time_now_u64 >= time_doubler_circuit_engages_u64) && (isDoublerEngagementTiming)) {
			  // bus has been low enough for long enough and can engage doubler
			  time_doubler_circuit_engages_u64 = 0;
			  isDoublerEngagementTiming = FALSE;
			  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5); // engage doubler relay
			  module_VoltageDoublerControl.isDoublerEngaged = TRUE;
              return_state_u8 = BUS_IS_GOOD;
            }
			else {
              return_state_u8 = BUS_IS_LOW_AND_ENGAGE_TIMING;		  	
			}
		  	break;			
        }
		case BUS_IS_GOOD: {
		  	// Bus is good and doubler might be engaged
		  	doubler_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
			if((doubler_comparison_voltage_u16 > VBUS_TO_DISENGAGE_DOUBLER) && (!isDoublerDisngagementTiming)) {
			  isDoublerDisngagementTiming = TRUE;
			  time_doubler_circuit_disengages_u64 = getSysCount() + TIME_TO_DISENGAGE_DOUBLER;
			  return_state_u8 = BUS_IS_GOOD_AND_DISENGAGE_TIMING;
			}
			else
			  return_state_u8 = BUS_IS_GOOD;
            break;
        }
		case BUS_IS_GOOD_AND_DISENGAGE_TIMING: {
		  	// as long as bus stays under threshold, keep timing
		  	time_now_u64 = getSysCount();
			doubler_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(doubler_comparison_voltage_u16 < VBUS_TO_DISENGAGE_DOUBLER) {
			  // bus has fallen to safe level and no need to remove doubler
			  isDoublerDisngagementTiming = FALSE;			  
			  time_doubler_circuit_engages_u64 = 0;
			  time_doubler_circuit_disengages_u64 = 0;
			  return_state_u8 = BUS_IS_GOOD;
			}
		  	else if ((time_now_u64 >= time_doubler_circuit_disengages_u64) && (isDoublerDisngagementTiming)) {
			  // bus has been high for long enough to disengage doubler
			  isDoublerDisngagementTiming = FALSE;
			  time_doubler_circuit_disengages_u64 = 0;			  
			  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5); // disengage doubler relay from circuit
			  module_VoltageDoublerControl.isDoublerEngaged = FALSE;
              return_state_u8 = BUS_IS_LOW_WITH_DOUBLER_DISENGAGED;
            }
			else
              return_state_u8 = BUS_IS_GOOD_AND_DISENGAGE_TIMING;		  	
		  	break;			
        }
        case IRQ_APP: {
            errorDoublerCounter++;
            return_state_u8 = BUS_IS_LOW_WITH_DOUBLER_DISENGAGED;
            break;
        }
        case STOP_APP: {
            return_state_u8 = STOP_APP;
            break;
        }
        default: {
            return_state_u8 = STOP_APP;
            break;
        }
    }
    return return_state_u8;
}
/** pam procedure #10 of Module insertion  :  add the module execution function end **/