/**
  ********************************************************************************************************************************
  * @file    module_ecm_icl.c 
  * @author  Myron Mychal
  * @brief   This is based off the module_ap.c template
  * @details This module controls the behavior of the inrush current limiting (ICL) circuit in the ECM 48 frame
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
//#include "driver_usart1.h"
#include "module_ecm_icl.h"
#include "mc_api.h"
#include "bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
#define	TIME_TO_ENGAGE_ICL		30000
#define	VBUS_TO_ENGAGE_ICL		20
#define	TIME_TO_DISENGAGE_ICL	1000
#define VBUS_TO_DISENGAGE_ICL	220

//#define	REMOVE_ICL_MODULE		1
#define	REMOVE_ICL_MODULE		0

#define DemandPollPeriod 2000   
uint64_t tt_DemandECMICLTime;
Module_InrushCurrentControl module_InrushCurrentControl;

//Usart1_Control* usart1Control_AppLocal;
uint8_t testECMICLCounter = 0;
uint64_t errorECMICLCounter = 0;
uint16_t icl_comparison_voltage_u16 = 0;		// bus voltage to compare against for inrush current limiting circuit engagement
bool isICLBusTiming = FALSE;

uint8_t tmpry4ECMICLTest = true; /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
#define tmpDelayPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandtmpECMICLDelayTime;


enum AppStates {
    INIT_ECM_ICL,
	BUS_LOW_WITH_ICL_ENGAGED,
	BUS_IS_LOW_BUT_TIMING,
	BUS_IS_GOOD_TO_RUN_MOTOR,
	BUS_IS_GOOD_BUT_TIMING,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

uint8_t p_moduleECM_ICL_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8) {
    uint8_t return_state_u8 = 0;
	uint64_t		time_now_u64 = 0;
	static uint64_t	time_icl_circuit_engages_u64 = 0;
	static uint64_t	time_icl_circuit_disengages_u64 = 0;
	
    switch (next_State_u8) {
        case INIT_ECM_ICL: {  
//#ifdef REMOVE_ICL_MODULE
//	        return_state_u8 = STOP_APP;
//            break;	  
//#else
			LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
			
			/* GPIO Ports Clock Enable */
			//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

			// initialization for ICL relay output pin
			GPIO_InitStruct.Pin 			= LL_GPIO_PIN_4;
			GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
			GPIO_InitStruct.Speed 			= LL_GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.OutputType 		= LL_GPIO_OUTPUT_PUSHPULL;
			GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
			LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);	// ICL is on/engaged	
			module_InrushCurrentControl.isICLEngaged = TRUE;
			module_InrushCurrentControl.errorCode_u8 = 0;
			return_state_u8 = BUS_LOW_WITH_ICL_ENGAGED;

            break;
//#endif
		}
        case BUS_LOW_WITH_ICL_ENGAGED: {			  
		  	// ICL is engaged because bus voltage is not high enough
		  	icl_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
			if((icl_comparison_voltage_u16 > VBUS_TO_DISENGAGE_ICL) && (!isICLBusTiming)) {
				isICLBusTiming = TRUE;
				time_icl_circuit_disengages_u64 = getSysCount() + TIME_TO_DISENGAGE_ICL;
				if(MC_GetOccurredFaultsMotor1())  //if any fault happen
      				MC_AcknowledgeFaultMotor1();     //clear the fault
				return_state_u8 = BUS_IS_GOOD_BUT_TIMING;
			}
			else
				return_state_u8 = BUS_LOW_WITH_ICL_ENGAGED;
            break;
		}
		case BUS_IS_GOOD_BUT_TIMING: {
		  	// as long as bus stays over threshold, keep timing
		  	time_now_u64 = getSysCount();
			icl_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(icl_comparison_voltage_u16 < VBUS_TO_ENGAGE_ICL) {
				// bus has fallen low enough to re-engage ICL
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);	// ICL is on/engaged
				module_InrushCurrentControl.isICLEngaged = TRUE;
			 	time_icl_circuit_engages_u64 = 0;
				time_icl_circuit_disengages_u64 = 0;
				isICLBusTiming = FALSE;
				return_state_u8 = BUS_LOW_WITH_ICL_ENGAGED;
			}
		  	else if ((time_now_u64 >= time_icl_circuit_disengages_u64) && (isICLBusTiming)) {
			  	// bus has been high enough for long enough and can disengage ICL
				time_icl_circuit_disengages_u64 = 0;
				isICLBusTiming = FALSE;
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4); // disengage ICL from circuit
				module_InrushCurrentControl.isICLEngaged = FALSE;
            	return_state_u8 = BUS_IS_GOOD_TO_RUN_MOTOR;
				if(MC_GetOccurredFaultsMotor1())  //if any fault happen
      				MC_AcknowledgeFaultMotor1();     //clear the fault				
            }
			else
            	return_state_u8 = BUS_IS_GOOD_BUT_TIMING;		  	
		  	break;			
        }
		case BUS_IS_GOOD_TO_RUN_MOTOR: {
		  	// ICL is not engaged because bus is high enough
		  	icl_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
			if((icl_comparison_voltage_u16 < VBUS_TO_ENGAGE_ICL) && (!isICLBusTiming)) {
				isICLBusTiming = TRUE;
				time_icl_circuit_engages_u64 = getSysCount() + TIME_TO_ENGAGE_ICL;
				return_state_u8 = BUS_IS_LOW_BUT_TIMING;
			}
			else
				return_state_u8 = BUS_IS_GOOD_TO_RUN_MOTOR;
            break;
        }
		case BUS_IS_LOW_BUT_TIMING: {
		  	// as long as bus stays under threshold, keep timing
		  	time_now_u64 = getSysCount();
			icl_comparison_voltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(icl_comparison_voltage_u16 > VBUS_TO_DISENGAGE_ICL) {
				// bus has risen and need to remove ICL
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);	// ICL is off/disaengaged
				module_InrushCurrentControl.isICLEngaged = FALSE;
			 	time_icl_circuit_engages_u64 = 0;
				time_icl_circuit_disengages_u64 = 0;
				isICLBusTiming = FALSE;
				return_state_u8 = BUS_IS_GOOD_TO_RUN_MOTOR;
			}
		  	else if ((time_now_u64 >= time_icl_circuit_engages_u64) && (isICLBusTiming)) {
			  	// bus has been low for long enough and can re-engage ICL
				time_icl_circuit_engages_u64 = 0;
				isICLBusTiming = FALSE;
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4); // engage ICL circuit
				module_InrushCurrentControl.isICLEngaged = TRUE;
            	return_state_u8 = BUS_LOW_WITH_ICL_ENGAGED;
            }
			else
            	return_state_u8 = BUS_IS_LOW_BUT_TIMING;		  	
		  	break;			
        }
        case IRQ_APP: {
            errorECMICLCounter++;
            return_state_u8 = BUS_LOW_WITH_ICL_ENGAGED;
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