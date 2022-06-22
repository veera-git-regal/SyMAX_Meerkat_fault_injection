/**
  ***************************************************************************************************
  * @file    module_dynamic.c 
  * @author  Doug Oda
  * @version V1.0
  * @date    11-May-2022
  * @brief   Handle real time info
  * @note    
  ***************************************************************************************************
  */

#include "module_dynamic.h"
#include "scheduler.h"
#include "mc_api.h"
#include "ab_module_Mc_StateMachine.h"
#include "regal_mc_lib.h"
#include "mc_tuning.h"
#include "mc_config.h"

extern ProcessInfo processInfoTable[];
extern MCT_Handle_t MCT[NBR_OF_MOTORS];
static MCT_Handle_t *pMCT = &MCT[M1];
DriveData_Control  dynamic_data;
Module_StateMachineControl *module_StateMachineControl_DynamicCtrl;

enum AppStates {
  MEM_INIT_APP,
  INIT_APP,
  RUN_APP,
  // additional states to be added here as necessary.
  IRQ_APP = DEFAULT_IRQ_STATE,
  STOP_APP = KILL_APP
};

/**
********************************************************************************
* @brief   Periodic function used to handle real time control via modbus
* @details 
* @param   drv_id_u8 The var used by the sceduler to identify this module
*          prev_state_u8 Unused by this module
*          next_state_u8 State to be executed 
*          irq_id_u8 Unused by this module
* @return  Current State 
* @note Unused parameters are maintained because this is called by a pointer whose
*       definition has four uint8_t parameters.
********************************************************************************
*/
uint8_t module_dynamic(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8)                 
{ 
  uint8_t returnStage = MEM_INIT_APP;  
  static uint32_t update_delay_u32 = 0;
  switch (next_State_u8)
    {
      case MEM_INIT_APP:                                                             
        {  
          returnStage = INIT_APP ;
          break;
        }        
      
      case INIT_APP:                                                             
        { 
          uint8_t Mc_StateMachineindex = getProcessInfoIndex(MODULE_MC_STATEMACHINE); //return Process index from processInfo array with the MC_statemachine module
          module_StateMachineControl_DynamicCtrl = (Module_StateMachineControl *)((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          update_delay_u32 = getSysCount() + DELAY_BETWEEN_DRIVE_DATA_UPDATES;
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          returnStage = RUN_APP;
          //update dynamic data if ready
          if ( update_delay_u32 > getSysCount() ){
            update_delay_u32 = getSysCount() + DELAY_BETWEEN_DRIVE_DATA_UPDATES;
            dynamic_data.driveData_Data.mcState_u16 = MC_GetSTMStateMotor1();
            dynamic_data.driveData_Data.mcFaults01_u16 = (*module_StateMachineControl_DynamicCtrl).errorCode_u16;//MC_GetOccurredFaultsMotor1(); // TODO: Clean pu errorCode_u16 and replace it with mcFaults01_u16
            dynamic_data.driveData_Data.mcFaults02_u16 = MeerkatInterface_FaultCode();;
            dynamic_data.driveData_Data.mcAppState_u16 = (*module_StateMachineControl_DynamicCtrl).current_State;
            if(MC_GetImposedDirectionMotor1() == 1)
            {
              dynamic_data.driveData_Data.motorActualDirection_u16 =  STD_ROTATION; // 9
            }else
            {
              dynamic_data.driveData_Data.motorActualDirection_u16 = REVERSE_ROTATION ; // 6
            }
            dynamic_data.driveData_Data.dcBusVoltage_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
        #if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_MV
            dynamic_data.driveData_Data.measuredSpeed_s16 = MC_GetMecSpeedAverageMotor1() * 6;
        #else
            dynamic_data.driveData_Data.measuredSpeed_s16 = MC_GetMecSpeedAverageMotor1() * -6;
        #endif
            dynamic_data.driveData_Data.measuredTorque_s16 = MC_GetTerefMotor1();
            dynamic_data.driveData_Data.measuredShaftPower_s16 = Regal_GetAvrgMotorPowerW(&EEPowerCalcHandle_M1);
            dynamic_data.driveData_Data.ipmTemperature_u16 = NTC_GetAvTemp_C(pMCT->pTemperatureSensor);
            dynamic_data.driveData_Data.phaseCurrentIa_s16 =  I_a_Peak;//(uint16_t) Regal_ConvertCountsTomA(Regal_GetAvrgMotorCurrentPhaseA()); //Regal_ConvertCountsTomA(PQD_MotorPowMeasM1.pFOCVars->Iab.a);
            dynamic_data.driveData_Data.phaseCurrentIb_s16 = I_b_Peak;//(uint16_t) Regal_ConvertCountsTomA(Regal_GetAvrgMotorCurrentPhaseB());  //Regal_ConvertCountsTomA(PQD_MotorPowMeasM1.pFOCVars->Iab.b);                     
          }
          break;
        }

      default:
      case STOP_APP:
        {
          returnStage = STOP_APP;
          break;
        }  
    }
  return returnStage;
}

/**
********************************************************************************
* @brief   Called when new drive dynamic data is received, updates speed direct etc
* @details 
* @param  
* @return   
********************************************************************************
*/  
void UpdateMotorDemand(void){
  if ( dynamic_data.driveData_Settings.direction_u16 == REVERSE_ROTATION){
    (*module_StateMachineControl_DynamicCtrl).motorDir = 1;
  }
  if ( dynamic_data.driveData_Settings.direction_u16 == STD_ROTATION){
    (*module_StateMachineControl_DynamicCtrl).motorDir = -1;
  }  
  (*module_StateMachineControl_DynamicCtrl).command_Speed = dynamic_data.driveData_Settings.commanded_speed_u16;
}

  
  