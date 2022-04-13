
/**
******************************************************************************
* @file    mc_tasks.c
* @author  Motor Control SDK Team, ST Microelectronics
* @brief   This file implements tasks definition
*
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "zz_module_flash.h"

/* USER CODE BEGIN Includes */
#include "bus_voltage_sensor.h"
#include "module_meerkat_interface.h"
#include "regal_mc_lib.h"
#include "harmonic_injection.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  100
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */
/* USER CODE END Private define */
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;

NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

/* USER CODE BEGIN Private Variables */
  PID_Handle_t *pPIDBk[NBR_OF_MOTORS]; //RPa
  PID_Handle_t *pPIDIm[NBR_OF_MOTORS]; //RPa
  Braking_Handle_t *pBrakeId[NBR_OF_MOTORS];

#if (REGAL_OTF == 1)
OTF_Handle_t *pOTFId[NBR_OF_MOTORS];
#endif

///////////////////////////////////////////////
// To be written to flash as settings
static volatile uint8_t decel_control = (uint8_t) default_DECEL_CONTROL;
//////////////////////////////////////////////

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void UI_Scheduler(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
* @brief  It initializes the whole MC core according to user defined
*         parameters.
* @param  pMCIList pointer to the vector of MCInterface objects that will be
*         created and initialized. The vector must have length equal to the
*         number of motor drives.
* @param  pMCTList pointer to the vector of MCTuning objects that will be
*         created and initialized. The vector must have length equal to the
*         number of motor drives.
* @retval None
*/
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);
  
  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_1_Init(&PWM_Handle_M1);
  /* USER CODE BEGIN MCboot 1 */
  /*********************************************************/
  /*    Braking and On-the-fly component initialization    */
  /*********************************************************/
  BrakeHandle_M1.Vbus_Add   = A_BK_VBUS_ADD;
  OTFHandle_M1.MaxSyncSpeed = A_OTF_MAX_SYNC_SPEED;
  OTFHandle_M1.MinSyncSpeed = A_OTF_MIN_SYNC_SPEED;
  OTFHandle_M1.detect_bemfg = A_OTF_DBEMFG;
  OTFHandle_M1.max_bemfg    = A_OTF_MAX_BEMFG;
  OTFHandle_M1.min_bemfg    = A_OTF_MIN_BEMFG;
  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();    

  /******************************************************/
  /*     Init Regal user flash setting parameters       */
  /******************************************************/
  RegalSetting_Init();
  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;
  
  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pSTC[M1] = &SpeednTorqCtrlM1;
  STO_PLL_Init (&STO_PLL_M1);
  

  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M1],pPIDSpeed[M1], &STO_PLL_M1._Super);
  
  /****************************************************/
  /*   Virtual speed sensor component initialization  */
  /****************************************************/ 
  VSS_Init (&VirtualSpeedSensorM1);
  
  /**************************************/
  /*   Rev-up component initialization  */
  /**************************************/
  RUC_Init(&RevUpControlM1,pSTC[M1],&VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);  
      
  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;
  
  /********************************************************/
  /*   Bus voltage sensor component initialization        */
  /********************************************************/
  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1);
  
  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];
  
  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  NTC_Init(&TempSensorParamsM1);    
  pTemperatureSensor[M1] = &TempSensorParamsM1;
    

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1] );
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = &RevUpControlM1;              /* only if M1 is sensorless*/
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &STO_PLL_M1; 
  MCT[M1].pSpeedSensorAux = MC_NULL;
  MCT[M1].pSpeedSensorVirtual = &VirtualSpeedSensorM1;  /* only if M1 is sensorless*/
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = MC_NULL;
  MCT[M1].pFF = MC_NULL;

  MCT[M1].pPosCtrl = MC_NULL;

  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];
 
  //DOUT_SetOutputState(&ICLDOUTParamsM1, INACTIVE);
  ICL_Init(&ICL_M1, &(pBusSensorM1->_Super), &ICLDOUTParamsM1);
  STM_NextState(&STM[M1],ICLWAIT);

  /* USER CODE BEGIN MCboot 2 */
    PID_HandleInit(&PIDBkHandle_M1);//RPa
    PID_HandleInit(&PIDImHandle_M1);//RPa
    BrakingStruct_Init(&BrakeHandle_M1, pSTC[M1] );
    pPIDBk[M1] = &PIDBkHandle_M1;//RPa
    pPIDIm[M1] = &PIDImHandle_M1;//RPa
    pBrakeId[M1] = &BrakeHandle_M1;
#if (REGAL_OTF==1)
  pOTFId[M1] = &OTFHandle_M1;
#endif
  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
* @brief Runs all the Tasks of the Motor Control cockpit
*
* This function is to be called periodically at least at the Medium Frequency task
* rate (It is typically called on the Systick interrupt). Exact invokation rate is 
* the Speed regulator execution rate set in the Motor Contorl Workbench.
*
* The following tasks are executed in this order:
*
* - Medium Frequency Tasks of each motors
* - Safety Task
* - Power Factor Correction Task (if enabled)
* - User Interface task. 
*/
__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that  
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
    

    /* ** User Interface Task ** */
    UI_Scheduler();
  }
}

/**
* @brief  Executes the Medium Frequency Task functions for each drive instance. 
*
* It is to be clocked at the Systick frequency.
*/
__weak void MC_Scheduler(void)
{
  /* USER CODE BEGIN MC_Scheduler 0 */
  
  /* USER CODE END MC_Scheduler 0 */
  
  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
* @brief Executes medium frequency periodic Motor Control tasks
*
* This function performs some of the control duties on Motor 1 according to the 
* present state of its state machine. In particular, duties requiring a periodic 
* execution at a medium frequency rate (such as the speed controller for instance) 
* are executed here.
*/
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  State_t StateM1;
  int16_t wAux = 0;

  ICL_State_t ICLstate = ICL_Exec( &ICL_M1 );
  bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeedUnit( &STO_PLL_M1, &wAux );
  PQD_CalcElMotorPower( pMPM[M1] );

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {
  case ICLWAIT:
    if ( ICLstate == ICL_INACTIVE )
    {
      /* If ICL is Inactive, move to IDLE */
      STM_NextState( &STM[M1], IDLE );
    }
    break;

  case IDLE_START:
    RUC_Clear( &RevUpControlM1, MCI_GetImposedMotorDirection( oMCInterface[M1] ) );
    PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );
    STM_NextState(&STM[M1],OFFSET_CALIB);
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], CLEAR );
    }
    break;

  case CLEAR:
    /* In a sensorless configuration. Initiate the Revup procedure */
    FOCVars[M1].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
    STO_PLL_Clear( &STO_PLL_M1 );

    if ( STM_NextState( &STM[M1], START ) == true )
    {
      if ((A_CONTROLLED_BRAKING==1)||(decel_control))
      {
        BrakeHandle_M1.Adapt_BusVoltageRef = VBS_GetAvBusVoltage_V(&(pBusSensorM1->_Super)) + BrakeHandle_M1.Vbus_Add;
        
        pPIDSpeed[M1]->wUpperIntegralLimit = (int32_t)A_IQMAX * (int32_t)SP_KIDIV;
        pPIDSpeed[M1]->wLowerIntegralLimit = -(int32_t)A_IQMAX * (int32_t)SP_KIDIV;
        pPIDSpeed[M1]->hUpperOutputLimit = (int16_t)A_IQMAX;
        pPIDSpeed[M1]->hLowerOutputLimit = -(int16_t)A_IQMAX;
      } 
      
      if (decel_control == 0)
      {
        if (MC_GetImposedDirectionMotor1() == 1) //CW direction
        {
          pPIDSpeed[M1]->wLowerIntegralLimit = 0;
          pPIDSpeed[M1]->hLowerOutputLimit = 0;
          pPIDSpeed[M1]->wUpperIntegralLimit = (int32_t)A_IQMAX * (int32_t)SP_KIDIV;
          pPIDSpeed[M1]->hUpperOutputLimit = (int16_t)A_IQMAX;
        }
        else if (MC_GetImposedDirectionMotor1()== -1 ) //CCW direction
        {
          pPIDSpeed[M1]->hUpperOutputLimit = 0;
          pPIDSpeed[M1]->wUpperIntegralLimit = 0;
          pPIDSpeed[M1]->wLowerIntegralLimit = -(int32_t)A_IQMAX * (int32_t)SP_KIDIV;
          pPIDSpeed[M1]->hLowerOutputLimit = -(int16_t)A_IQMAX;
        }
      }
#if (REGAL_OTF == 1)
      pOTFId[M1]->bemfg_alpha = pOTFId[M1]->max_bemfg;
      pOTFId[M1]->bemfg_beta = pOTFId[M1]->max_bemfg;
      OTFHandle_M1.seamless_transfer = 0;
      OTFHandle_M1.hdir = 0;
#endif      
      FOC_Clear( M1 );

      R3_1_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case START:
    {
  
      /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
      int16_t hForcedMecSpeedUnit;
      qd_t IqdRef;
      bool ObserverConverged = false;

      /* Execute the Rev Up procedure */
#if (REGAL_OTF==1)
      if ( ! Regal_OTF_Exec( &RevUpControlM1, &OTFHandle_M1 ))
#else
        if ( ! RUC_Exec( &RevUpControlM1 ))
#endif
        {
          /* The time allowed for the startup sequence has expired */
          STM_FaultProcessing( &STM[M1], MC_START_UP, 0 );  
        }
        else
        {
          /* Execute the torque open loop current start-up ramp:
          * Compute the Iq reference current as configured in the Rev Up sequence */
          IqdRef.q = STC_CalcTorqueReference( pSTC[M1] );
          IqdRef.d = FOCVars[M1].UserIdref;
          /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
          FOCVars[M1].Iqdref = IqdRef;
        }
      
      (void) VSS_CalcAvrgMecSpeedUnit( &VirtualSpeedSensorM1, &hForcedMecSpeedUnit );

      {
        ObserverConverged = STO_PLL_IsObserverConverged( &STO_PLL_M1,hForcedMecSpeedUnit );
        STO_SetDirection(&STO_PLL_M1, MCI_GetImposedMotorDirection( &Mci[M1]));
        (void) VSS_SetStartTransition( &VirtualSpeedSensorM1, ObserverConverged );
      }
      
#if (REGAL_OTF == 1)
      if(pOTFId[M1]->seamless_transfer == 1)
      {
        pOTFId[M1]->seamless_transfer = 2;
      }
      
      /////////////////////////////////////////////////////////////////
      if (RevUpControlM1.bStageCnt == 0)
      {
        if (pOTFId[M1]->bemfg_alpha >= pOTFId[M1]->detect_bemfg)
        {pOTFId[M1]->bemfg_alpha -= BEMF_DEC;}
        if (pOTFId[M1]->bemfg_beta >= pOTFId[M1]->detect_bemfg)
        {pOTFId[M1]->bemfg_beta -= BEMF_DEC;}
      }
      else if (RevUpControlM1.bStageCnt == 1)
      {
        if (pOTFId[M1]->bemfg_alpha >= pOTFId[M1]->min_bemfg)
        {pOTFId[M1]->bemfg_alpha -= BEMF_DEC;}
        if (pOTFId[M1]->bemfg_beta >=pOTFId[M1]->min_bemfg)
        {pOTFId[M1]->bemfg_beta -= BEMF_DEC;}
      }
#endif
      //////////////////////////////////////////////////////////////////
      if ( ObserverConverged )
      {
        qd_t StatorCurrent = MCM_Park( FOCVars[M1].Ialphabeta, SPD_GetElAngle( &STO_PLL_M1._Super ) );

        /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC. */
        REMNG_Init( pREMNG[M1] );
        REMNG_ExecRamp( pREMNG[M1], FOCVars[M1].Iqdref.q, 0 );
        REMNG_ExecRamp( pREMNG[M1], StatorCurrent.q, A_TRANSITION_DURATION );
        
        STM_NextState( &STM[M1], SWITCH_OVER );
      }
    }
    break;
    
 
  case SWITCH_OVER:
    {
      bool LoopClosed;
      int16_t hForcedMecSpeedUnit;
      
      
#if (REGAL_OTF == 1)
      if ( ! Regal_OTF_Exec( &RevUpControlM1, &OTFHandle_M1 ))
#else
        if ( ! RUC_Exec( &RevUpControlM1 ))
#endif
        {
          /* The time allowed for the startup sequence has expired */
          STM_FaultProcessing( &STM[M1], MC_START_UP, 0 );  
        } 
        else
        { 
          /* Compute the virtual speed and positions of the rotor. 
          The function returns true if the virtual speed is in the reliability range */
          LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1,&hForcedMecSpeedUnit);
          /* Check if the transition ramp has completed. */ 
          LoopClosed |= VSS_TransitionEnded( &VirtualSpeedSensorM1 );
          
          /* If any of the above conditions is true, the loop is considered closed. 
          The state machine transitions to the START_RUN state. */
          if ( LoopClosed == true ) 
          {
#if (REGAL_OTF == 1)          
            if(pOTFId[M1]->seamless_transfer == 2)
            {
              /////////////////////////////////////////////////////
              // The following lines are for seeding the q and d integrators
              alphabeta_t Vab = FOCVars[M1].Valphabeta;
              int16_t angle = FOCVars[M1].hElAngle;
              //seeding
              qd_t Vdq_seed = MCM_Park(Vab,angle);
              int16_t h_Vd_seed = Vdq_seed.d;
              int16_t h_Vq_seed = Vdq_seed.q;
              PID_SetIntegralTerm(pPIDIq[M1], (int32_t) ((int32_t) h_Vq_seed * PID_GetKIDivisor(pPIDIq[M1])));
              PID_SetIntegralTerm(pPIDId[M1], (int32_t) ((int32_t) h_Vd_seed * PID_GetKIDivisor(pPIDId[M1])));   
              //////////////////////////////////////////////////////////////////////////////////////////////////
              pOTFId[M1]->seamless_transfer = 3;
            }
#endif     
#if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )  
            PID_SetIntegralTerm( pPIDSpeed[M1], 0 );
#else
            PID_SetIntegralTerm( pPIDSpeed[M1],
                                (int32_t) ( FOCVars[M1].Iqdref.q * PID_GetKIDivisor(pPIDSpeed[M1]) /
                                           PID_SPEED_INTEGRAL_INIT_DIV ) );
#endif
            
            STM_NextState( &STM[M1], START_RUN );
          }  
        }
    }

    break;

  case START_RUN:
    /* only for sensor-less control */
    STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /*Observer has converged*/
    {
      /* USER CODE BEGIN MediumFrequencyTask M1 1 */
      if (decel_control) RegenControlM1(pBrakeId[M1], pPIDBk[M1],pPIDSpeed[M1], pSTC[M1], pBusSensorM1);//Regenerative control during motor run state    

	  //HarmonicCompensation
      HarmonicInjection_Enable();
	  
      /* USER CODE END MediumFrequencyTask M1 1 */      
	  FOC_InitAdditionalMethods(M1);
      FOC_CalcCurrRef( M1 );
      STM_NextState( &STM[M1], RUN );
    }
    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */
	
    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M1 2 */
#if (REGAL_OTF == 1)     
    if (pOTFId[M1]->seamless_transfer == 3)
    {
      /////////////////////////////////////////////////////
      // The following lines are for seeding the q and d integrators
      alphabeta_t Vab = FOCVars[M1].Valphabeta;
      int16_t angle = FOCVars[M1].hElAngle;
      //seeding
      qd_t Vdq_seed = MCM_Park(Vab,angle);
      int16_t h_Vd_seed = Vdq_seed.d;
      int16_t h_Vq_seed = Vdq_seed.q;
      PID_SetIntegralTerm(pPIDIq[M1], (int32_t) ((int32_t) h_Vq_seed * PID_GetKIDivisor(pPIDIq[M1])));
      PID_SetIntegralTerm(pPIDId[M1], (int32_t) ((int32_t) h_Vd_seed * PID_GetKIDivisor(pPIDId[M1])));   
      pOTFId[M1]->seamless_transfer = 4;
    }
#endif
    if (decel_control) RegenControlM1(pBrakeId[M1], pPIDBk[M1],pPIDSpeed[M1], pSTC[M1], pBusSensorM1);//Regenerative control during motor run state  
    /* USER CODE END MediumFrequencyTask M1 2 */

    MCI_ExecBufferedCommands( oMCInterface[M1] );
    FOC_CalcCurrRef( M1 );
	
    if( !IsSpeedReliable )
    {
      STM_FaultProcessing( &STM[M1], MC_SPEED_FDBK, 0 );
    }
	
    /* USER CODE BEGIN MediumFrequencyTask M1 3 */

    /* USER CODE END MediumFrequencyTask M1 3 */
    break;

  case ANY_STOP:
    if(A_CONTROLLED_BRAKING==0)
    {
      R3_1_SwitchOffPWM( pwmcHandle[M1] );
      FOC_Clear( M1 );
      MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
      TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );
    }
      /* USER CODE BEGIN MediumFrequencyTask M1 4 */
    else if (A_CONTROLLED_BRAKING==1)
    {
      pBrakeId[M1]->IMax_Ref = BRAKING_CURRENTSEEDING; //RPa: seeding of current reference
      pBrakeId[M1]->BrakingPhase = STARTRAMP;
      pBrakeId[M1]->rMeasuredSpeed = SPD_GetAvrgMecSpeedUnit( pSTC[M1]->SPD );
      pBrakeId[M1]->Adapt_IMax = (int32_t)(((int32_t)A_BK_RAMP_a * (int32_t) pBrakeId[M1]->rMeasuredSpeed * (int32_t) pBrakeId[M1]->rMeasuredSpeed)>>BYTE_SHIFT) + \
        (int32_t)((int32_t)A_BK_RAMP_b * (int32_t)pBrakeId[M1]->rMeasuredSpeed) + (int32_t)A_BK_RAMP_c;
    }
    /* USER CODE END MediumFrequencyTask M1 4 */

    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
    if(A_CONTROLLED_BRAKING == 1)
    {
      //RPa: Non-regenerative braking
      MotorBraking_StateMachine(pBrakeId[M1], pPIDBk[M1], pPIDIm[M1], pSTC[M1], &FOCVars[M1], pBusSensorM1 );
      if (pBrakeId[M1]->BrakingPhase == STARTRAMP)//RPa: this state can only be called from LOWSPEED_IQHOLD
      {
        FOC_Clear( M1 );
        MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] ); 
        STM_NextState( &STM[M1], STOP_IDLE );      
      }
    }
    else
    {
      if ( TSK_StopPermanencyTimeHasElapsedM1())
      {
        FOC_Clear( M1 );
        MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] ); 
        STM_NextState( &STM[M1], STOP_IDLE );
      }
    }
    break;

  case STOP_IDLE:
    STC_SetSpeedSensor( pSTC[M1],&VirtualSpeedSensorM1._Super );  	/*  sensor-less */
    VSS_Clear( &VirtualSpeedSensorM1 ); /* Reset measured speed in IDLE */

    /* USER CODE BEGIN MediumFrequencyTask M1 5 */
    R3_1_SwitchOffPWM( pwmcHandle[M1] );
    /* USER CODE END MediumFrequencyTask M1 5 */
    STM_NextState( &STM[M1], ICLWAIT );
    break;

  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
* @brief  It re-initializes the current and voltage variables. Moreover
*         it clears qd currents PI controllers, voltage sensor and SpeednTorque
*         controller. It must be called before each motor restart.
*         It does not clear speed sensor.
* @param  bMotor related motor it can be M1 or M2
* @retval none
*/
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  ab_t NULL_ab = {(int16_t)0, (int16_t)0};
  qd_t NULL_qd = {(int16_t)0, (int16_t)0};
  alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};
  
  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
  FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */
  
  //HarmonicCompensation
  HarmonicInjection_Disable();

  

  /* USER CODE END FOC_Clear 1 */
}

/**
* @brief  Use this method to initialize additional methods (if any) in
*         START_TO_RUN state
* @param  bMotor related motor it can be M1 or M2
* @retval none
*/
__weak void FOC_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
* @brief  It computes the new values of Iqdref (current references on qd
*         reference frame) based on the required electrical torque information
*         provided by oTSC object (internally clocked).
*         If implemented in the derived class it executes flux weakening and/or
*         MTPA algorithm(s). It must be called with the periodicity specified
*         in oTSC parameters
* @param  bMotor related motor it can be M1 or M2
* @retval none
*/
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
    
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;

  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
* @brief  It set a counter intended to be used for counting the delay required
*         for drivers boot capacitors charging of motor 1
* @param  hTickCount number of ticks to be counted
* @retval void
*/
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
  hBootCapDelayCounterM1 = hTickCount;
}

/**
* @brief  Use this function to know whether the time required to charge boot
*         capacitors of motor 1 has elapsed
* @param  none
* @retval bool true if time has elapsed, false otherwise
*/
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
* @brief  It set a counter intended to be used for counting the permanency
*         time in STOP state of motor 1
* @param  hTickCount number of ticks to be counted
* @retval void
*/
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
* @brief  Use this function to know whether the permanency time in STOP state
*         of motor 1 has elapsed
* @param  none
* @retval bool true if time is elapsed, false otherwise
*/
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
* @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
*
*  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
* subsystem (see the state machine(s)).
*
* @retval Number of the  motor instance which FOC loop was executed.
*/
__weak uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
 

  uint16_t hState;  /*  only if sensorless main*/
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main*/

  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
  if ( STM[M1].bState == SWITCH_OVER )
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = REMNG_Calc(pREMNG[M1]);
    }
  }
  if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM1))
  {
    hFOCreturn = FOC_CurrControllerM1();
    hState = STM_GetState(&STM[M1]);
  }
  else
  {
    hFOCreturn = MC_NO_ERROR;
  }
  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
  }
  else
  {
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1); 
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(pBusSensorM1->_Super)); /*  only for sensorless*/
    STO_PLL_CalcElAngle (&STO_PLL_M1, &STO_Inputs);
    STO_PLL_CalcAvrgElSpeedDpp (&STO_PLL_M1); /*  Only in case of Sensor-less */
    if (IsAccelerationStageReached == false)
    {
      STO_ResetPLL(&STO_PLL_M1);
    }  
    hState = STM_GetState(&STM[M1]);
    if((hState == START) || (hState == SWITCH_OVER) || (hState == START_RUN)) /*  only for sensor-less*/
    {
      int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);      
      VSS_CalcElAngle(&VirtualSpeedSensorM1,&hObsAngle);  
    }
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */  
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
* @brief It executes the core of FOC drive that is the controllers for Iqd
*        currents regulation. Reference frame transformations are carried out
*        accordingly to the active speed sensor. It must be called periodically
*        when new motor currents have been converted
* @param this related object of class CFOC.
* @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
*         next PWM Update event, MC_FOC_DURATION otherwise
*/
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;

  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1],
            (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

  Vqd.d = PI_Controller(pPIDId[M1],
            (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  
  /* USER CODE BEGIN CurrControllerM1 */
  //Harmonic Injection will provide injection values of zero if not enabled
  //HarmonicInjection_Enable();
  HarmonicInjection_Calculate(hElAngle);
  Vqd.q = Vqd.q + HarmonicInjection_IqAdjustment();   
  Vqd.d = Vqd.d + HarmonicInjection_IdAdjustment();
  /* USER CODE END CurrControllerM1 */ 
  

  Vqd = Circle_Limitation(pCLM[M1], Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
   
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;
  
   #if DISABLE_MEERKAT_SAFETY_MODULE <= 0
  // Note: ST Motor Control always converts active calculations to a and b (even if 'c' was read by adc)
  // - ia + ib + ic = 0
  MeerkatInterface_AddShuntCurrentSample(Iab.a, Iab.b, 0 - Iab.a - Iab.b);
  #endif //DISABLE_MEERKAT_SAFETY_MODULE <= 0
  
  return(hCodeError);
}

/**
* @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
*
* Faults flags are updated here.
*/
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {  
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    RCM_ExecUserConv ();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
* @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
* @param  bMotor Motor reference number defined
*         \link Motors_reference_number here \endlink
* @retval None
*/
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS 
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if(bMotor == M1)
  {
    CodeReturn |=  errMask[bMotor] & RVBS_CalcAvVbusFilt(pBusSensorM1);
  }

  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
    break;
  case FAULT_OVER:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
	/* USER CODE BEGIN TSK_SafetyTask_PWMOFF 2 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 2 */
    break;
  default:
    break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
* @brief  This function returns the reference of the MCInterface relative to
*         the selected drive.
* @param  bMotor Motor reference number defined
*         \link Motors_reference_number here \endlink
* @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
*         Note: it can be MC_NULL if MCInterface of selected drive is not
*         allocated.
*/
__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
* @brief  This function returns the reference of the MCTuning relative to
*         the selected drive.
* @param  bMotor Motor reference number defined
*         \link Motors_reference_number here \endlink
* @retval MCT_Handle_t motor control tuning handler for the selected drive.
*         Note: it can be MC_NULL if MCInterface of selected drive is not
*         allocated.
*/
__weak MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
* @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
*
*  This function is to be executed when a general hardware failure has been detected  
* by the microcontroller and is used to put the system in safety condition.
*/
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  
  R3_1_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
/**
* @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
*/
__weak void mc_lock_pins (void)
{
  LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
  LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
  LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
  //LL_GPIO_LockPin(M1_OVP_GPIO_Port, M1_OVP_Pin);
  LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
  LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
  LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
  LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
  //LL_GPIO_LockPin(M1_ICL_SHUT_OUT_GPIO_Port, M1_ICL_SHUT_OUT_Pin);
  LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
  //LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
  LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
  LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
  LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
