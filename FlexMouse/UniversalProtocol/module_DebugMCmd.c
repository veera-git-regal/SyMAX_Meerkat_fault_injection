/**
  ***************************************************************************************************
  * @file    module_DebugMCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    27-Aug-2020
  * @brief   Decode and perform debug mode CMD
  * @note    This App decode debug mode CMD in Universal protocol
  ***************************************************************************************************
  */
#include "module_DebugMCmd.h"
#include "state_machine.h"
#include "mc_interface.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "mc_api.h"
#include "driver_usart1.h"
#include "regal_mc_lib.h"
#include "parameters_conversion.h"

extern MCI_Handle_t *pMCI[NBR_OF_MOTORS];
extern MCT_Handle_t *pMCT[NBR_OF_MOTORS];
extern uint32_t wConfig[NBR_OF_MOTORS]; //motor setting header in parameters_conversion.h
extern UI_Handle_t UI_Params;
extern DAC_UI_Handle_t * pDAC;

extern uint64_t getSysCount(void);
/* SysCon handle declaration */
extern ProcessInfo processInfoTable[];

Usart1_Control* usart1Control_DebugMCmd;
//Module_StateMachineControl*  module_StateMachineControl_ShortCmd;

enum DebugMSuccess 
{
  dbmFail,
  dbmSuccess,
  dbmRegRd,
  dbmLongDatSuccess
};

enum AppStates {
    INIT_APP,
    RUN_APP,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

//uint16_t adcFilterVal = 0;
unsigned char protocolBuf_DebugMCmd[100] ;
int32_t tmpryOPData;
int32_t tmpryDebugData;
int16_t API_Speed;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t moduleDebugMCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8)                 
{ 
  uint8_t     returnStage = 0;  

  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_DebugMCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); 
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_DebugMCmd).seqMemRXG4L_u32) >= DataLen2 )
          {            
            RingBuf_Observe((*usart1Control_DebugMCmd).seqMemRXG4L_u32, protocolBuf_DebugMCmd, 0, &DataLen2);  
            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_DebugMCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
            RingBuf_ReadBlock((*usart1Control_DebugMCmd).seqMemRXG4L_u32, protocolBuf_DebugMCmd, &DataLen2); //extract the whole frame
            //decode and perform the CMD function
            if(protocolBuf_DebugMCmd[2] == 0x70) //read registers
            {
              uint8_t outBuff[8];
              uint8_t success = dbmFail;
              
              tmpryOPData = DebugMRegRd((uint8_t) protocolBuf_DebugMCmd[5], &success, outBuff);
              
              if(success == dbmSuccess)
              { //send out the result through Universal protocol
                unsigned char TmpryBufTx[] = {0x55, 0x05, 0x70, 0x00, 0x00, protocolBuf_DebugMCmd[5], 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC};
                TmpryBufTx[9] = tmpryOPData & 0xff;
                TmpryBufTx[8] = (tmpryOPData >> 8) & 0xff;
                TmpryBufTx[7] = (tmpryOPData >> 16) & 0xff;
                TmpryBufTx[6] = (tmpryOPData >> 24) & 0xff;                
                uint32_t TxLen = sizeof(TmpryBufTx);
                RingBuf_WriteBlock((*usart1Control_DebugMCmd).seqMemTX_u32, TmpryBufTx, &TxLen); 
              }else{
                if(success == dbmLongDatSuccess)
                { // this is only for read Start Up data 
                  unsigned char TmpryBufTx[] = {0x55, 0x0A, 0x70, 0x00, 0x00, protocolBuf_DebugMCmd[5], protocolBuf_DebugMCmd[6], outBuff[3], outBuff[2], outBuff[1], outBuff[0], outBuff[5], outBuff[4], outBuff[7], outBuff[6], 0xCC, 0xCC};              
                  uint32_t TxLen = sizeof(TmpryBufTx);
                  RingBuf_WriteBlock((*usart1Control_DebugMCmd).seqMemTX_u32, TmpryBufTx, &TxLen); 
                }
              }
            }
            else
              if(protocolBuf_DebugMCmd[2] == 0x71) //write registers
              { //write register decoder
                int32_t wValue = protocolBuf_DebugMCmd[6] << 8;
                wValue = (wValue + protocolBuf_DebugMCmd[7])<< 8;
                wValue = (wValue + protocolBuf_DebugMCmd[8])<< 8;
                wValue += protocolBuf_DebugMCmd[9];
                switch(protocolBuf_DebugMCmd[5])
                {
                    case 0x01: /** @this function not implemented **/  //write motor error flags
                    case 0x02:      //write Status
                    case 0x04:      //write current mechanical rotor speed reference 
                    case 0x16:    //write Flux weakening Kp                                       
                    case 0x17:    //write Flux weakening Ki                                         
                    case 0x18:    //write Flux weakening BUS Voltage allowed percentage reference   
                    case 0x19:    //write Bus Voltage                                               
                    case 0x1A:    //write Heatsink Temperature       
                    case 0x1B:    //write Motor Power  
                    case 0x1E:    //write Speed measured                                             
                    case 0x1F:    //write Torque measured (Iq)                                        
                    case 0x20:    //write Flux measured (Id)                                    
                    case 0x21:    //write Flux weakening BUS Voltage allowed percentage measured     
                    case 0x22:    //write Revup stage numbers                                      
                    case 0x23:    //write Ia                                                         
                    case 0x24:    //write Ib                                                       
                    case 0x25:    //write Ialpha                                                   
                    case 0x26:    //write Ibeta                                                    
                    case 0x27:    //write Iq                                                         
                    case 0x28:    //write Id                                                        
                    case 0x29:    //write Iq reference                                               
                    case 0x2A:    //write Id reference                                              
                    case 0x2B:    //write Vq                                                        
                    case 0x2C:    //write Vd                                                         
                    case 0x2D:    //write Valpha                                                    
                    case 0x2E:    //write Vbeta                                                     
                    case 0x2F:    //write Measured electrical angle                               
                    case 0x30:    //write Measured rotor speed                                   
                    case 0x31:    //read Observer electrical angle (PLL)                         
                    case 0x32:    //write Observer rotor speed (PLL)                          
                    case 0x33:    //write Observer Ialpha (PLL)                                    
                    case 0x34:    //write Observer Ibeta (PLL)                                   
                    case 0x35:    //write Observer BEMF alpha (PLL)                                
                    case 0x36:    //write Observer BEMF beta (PLL)     
                    case 0x3D:	//write User defined DAC 1       
                    case 0x3E:    //write User defined DAC 2        
                    case 0x3F:    //write Maximum application speed 
                    case 0x40:    //write Minimum application speed 
                    case 0x42:    //write Expected BEMF level (PLL) 
                    case 0x43:    //write Observed BEMF level (PLL) 
                    case 0x46:    //write Feedforward (1Q)   
                    case 0x47:    //write Feedforward (1D)        
                    case 0x48:    //write Feedforward (2)         
                    case 0x49:    //write Feedforward (VQ)         
                    case 0x4A:    //write Feedforward (VD)       
                    case 0x4B:    //write Feedforward (VQ PI out)   
                    case 0x4C:    //write Feedforward (VD PI out)   
                    case 0x4D:    //write PFC Status              
                    case 0x4E:    //write PFC Flags              
                    case 0x4F:    //write PFC DC bus reference 
                    case 0x50:    //write PFC DC bus measured      
                    case 0x51:    //write AC Mains frequency   
                    case 0x52:    //write AC Mains voltage 0-to-pk  
                    case 0x53:    //write PFC Current loop Kp  
                    case 0x54:    //write PFC Current loop Ki    
                    case 0x55:    //write PFC Current loop Kd      
                    case 0x56:    //write PFC Voltage loop Kp   
                    case 0x57:    //write PFC Voltage loop Ki  
                    case 0x58:    //write PFC Voltage loop Kd     
                    case 0x59:    //write PFC startup duration     
                    case 0x5A:    //write PFC abilitation status 
                    case 0x5C:    //write Ramp duration  
                      {     /** @this function not implemented **/
                        
                        break;
                      }     
                    case 0x03:      //write control mode
                      {                     
                        if ((STC_Modality_t)wValue == STC_TORQUE_MODE)
                        {
                          MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
                        }
                        if ((STC_Modality_t)wValue == STC_SPEED_MODE)
                        {
                          MCI_ExecSpeedRamp(pMCI[M1], MCI_GetMecSpeedRefUnit(pMCI[M1]),0);
                        }
                        break;
                      }
                    
                    case 0x05:      //write Speed Kp
                      {                     
                        PID_SetKP(pMCT[M1]->pPIDSpeed,(int16_t)wValue);
                        break;
                      }
                    case 0x06:      //write Speed Ki
                      { 
                        PID_SetKI(pMCT[M1]->pPIDSpeed,(int16_t)wValue);
                        break;
                      }
                    case 0x07:      //write Speed Kd
                      { 
                        PID_SetKD(pMCT[M1]->pPIDSpeed,(int16_t)wValue);
                        break;
                      }
                    case 0x08:      //write Torque reference (Iq)
                      {     
                        qd_t currComp;
                        currComp = MCI_GetIqdref(pMCI[M1]);
                        currComp.q = (int16_t)wValue;
                        MCI_SetCurrentReferences(pMCI[M1],currComp);
                        break;
                      }
                    case 0x09:      //write Torque Kp
                      {     
                        PID_SetKP(pMCT[M1]->pPIDIq,(int16_t)wValue);
                        break;
                      }
                    case 0x0A:      //write Torque Ki
                      {     
                        PID_SetKI(pMCT[M1]->pPIDIq,(int16_t)wValue);
                        break;
                      }
                    case 0x0B:      //write Torque Kd
                      {     
                        PID_SetKD(pMCT[M1]->pPIDIq,(int16_t)wValue);
                        break;
                      }
                    case 0x0C:      //write Flux reference (Id)
                      {     
                        qd_t currComp;
                        currComp = MCI_GetIqdref(pMCI[M1]);
                        currComp.d = (int16_t)wValue;
                        MCI_SetCurrentReferences(pMCI[M1],currComp);
                        break;
                      }
                    case 0x0D:
                      {     //Flux Kp
                        PID_SetKP(pMCT[M1]->pPIDId,(int16_t)wValue);
                        break;
                      }
                    case 0x0E:
                      {     //Flux Ki
                        PID_SetKI(pMCT[M1]->pPIDId,(int16_t)wValue);
                        break;
                      }
                    case 0x0F:
                      {     //Flux Kd
                        PID_SetKD(pMCT[M1]->pPIDId,(int16_t)wValue);
                        break;
                      }
                    case 0x10:
                      {     //Observer C1
                        uint32_t hUICfg = wConfig[M1]; //motor setting header in parameters_conversion.h
                        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
                        int16_t hC1,hC2;
                        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorMain;
                        }
                        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorAux;
                        }
                        if (pSPD != MC_NULL)
                        {
                          STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
                          STO_PLL_SetObserverGains((STO_PLL_Handle_t*)pSPD,(int16_t)wValue,hC2);
                        }
                        break;
                      }
                    case 0x11:
                      {     //Observer C2
                        uint32_t hUICfg = wConfig[M1];
                        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
                        int16_t hC1,hC2;
                        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorMain;
                        }
                        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorAux;
                        }
                        if (pSPD != MC_NULL)
                        {
                          STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
                          STO_PLL_SetObserverGains((STO_PLL_Handle_t*)pSPD,hC1,(int16_t)wValue);
                        }
                        break;
                      }
                    case 0x14:    //write PLL Ki      
                      {   
                        uint32_t hUICfg = wConfig[M1];
                        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
                        int16_t hPgain, hIgain;
                        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorMain;
                        }
                        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorAux;
                        }
                        if (pSPD != MC_NULL)
                        {
                          STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
                          STO_SetPLLGains((STO_PLL_Handle_t*)pSPD,hPgain,(int16_t)wValue);
                        }
                        break;
                      }                                           
                    case 0x15:    //write PLL Kp     
                      {     
                        uint32_t hUICfg = wConfig[M1];
                        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
                        int16_t hPgain, hIgain;
                        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorMain;
                        }
                        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
                        {
                          pSPD = pMCT[M1]->pSpeedSensorAux;
                        }
                        if (pSPD != MC_NULL)
                        {
                          STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
                          STO_SetPLLGains((STO_PLL_Handle_t*)pSPD,(int16_t)wValue,hIgain);
                        }            
                        break;
                      }                                        
                     
                    case 0x1C:    //write DAC Out 1       
                      {     
                        pDAC->bChannel_variable[0] = (MC_Protocol_REG_t) protocolBuf_DebugMCmd[6];
                        break;
                      }                                            
                    case 0x1D:    //write DAC Out 2      
                      {     
                        pDAC->bChannel_variable[1] = (MC_Protocol_REG_t) protocolBuf_DebugMCmd[6];
                        break;
                      }                                             
                    
                    case 0x41:    //write Id reference in speed mode
                      {
                        MCI_SetIdref(pMCI[M1],(int16_t)wValue);
                        break;
                      }
                    case 0x5B:    //write Ramp final speed 
                      {
                        MCI_ExecSpeedRamp(pMCI[M1],(int16_t)((wValue*SPEED_UNIT)/_RPM),0);
                        break;
                      }               
                    case 0xC1:    //write Start Up data             
                      { 
                        uint8_t bStage = protocolBuf_DebugMCmd[6];
                        uint16_t hDurationms = protocolBuf_DebugMCmd[13];
                        int16_t hFinalMecSpeedUnit;
                        int16_t hFinalTorque = protocolBuf_DebugMCmd[11];
                        int32_t rpm = protocolBuf_DebugMCmd[7];
                        rpm += ((int32_t)protocolBuf_DebugMCmd[8])<< 8 ;
                        rpm += ((int32_t)protocolBuf_DebugMCmd[9])<< 8 ;
                        rpm += ((int32_t)protocolBuf_DebugMCmd[10])<< 8 ;                     
                        hDurationms += ((int16_t)protocolBuf_DebugMCmd[14])<< 8;
                        hFinalMecSpeedUnit = (rpm * SPEED_UNIT ) / _RPM ;
                        hFinalTorque += ((int16_t)protocolBuf_DebugMCmd[12])<< 8;
                        
                        RevUpCtrl_Handle_t *pRevupCtrl = pMCT[M1]->pRevupCtrl;
                        RUC_SetPhaseDurationms(pRevupCtrl, bStage, hDurationms);
                        RUC_SetPhaseFinalMecSpeedUnit(pRevupCtrl, bStage, hFinalMecSpeedUnit);
                        RUC_SetPhaseFinalTorque(pRevupCtrl, bStage, hFinalTorque);
                        break;
                      }
                    default:
                      break;
                }      
              }
          }
          returnStage = RUN_APP;
          break;
        }
      case IRQ_APP:
        {
          //if more than 1 driver interrupt attached to this APP
//           uint8_t index = getProcessInfoIndex(interruptIdentfer);         //return Process index from processInfo array of the driver interrupt call, APP can response respectively
          returnStage = RUN_APP;
          break;
        }               
      case STOP_APP:
        {
          returnStage = INIT_APP;
          break;
        }
      default:
        returnStage = STOP_APP;   
    }
  return returnStage;
}


int32_t DebugMRegRd(uint8_t bCh_var, uint8_t *success, uint8_t *p_outBuff)
{
  int32_t p_tmpryOPData;
  switch(bCh_var)
  {//read register decoder
    case 0x01:
      { //read motor error flags
        p_tmpryOPData = (int32_t)STM_GetFaultState(pMCI[M1]->pSTM);
        *success = dbmSuccess;
        break;
      }
    case 0x02:      //read Status  ///tested with mc_api.c command 
      { 
        p_tmpryOPData = (int32_t)STM_GetState(pMCI[M1]->pSTM);
        *success = dbmSuccess;
        break;
      }
    case 0x03:      //read control mode //tested with mc_api.c command
      {  
        p_tmpryOPData = (int32_t)MCI_GetControlMode(pMCI[M1]);                  
        *success = dbmSuccess;
        break;
      }
    case 0x04:
      { //read current mechanical rotor speed reference 
         p_tmpryOPData = (int32_t)((MCI_GetMecSpeedRefUnit(pMCI[M1])*_RPM)/SPEED_UNIT);
         *success = dbmSuccess;
         break;
      }
    case 0x05:
      { //read Speed Kp
        p_tmpryOPData = (int32_t)PID_GetKP(pMCT[M1]->pPIDSpeed);
        *success = dbmSuccess;
        break;
      }
    case 0x06:
      { //read Speed Ki
        p_tmpryOPData = (int32_t)PID_GetKI(pMCT[M1]->pPIDSpeed);
        *success = dbmSuccess;
        break;
      }
    case 0x07:
      { //read Speed Kd
        p_tmpryOPData = (int32_t)PID_GetKD(pMCT[M1]->pPIDSpeed);
        *success = dbmSuccess;
        break;
      }
    case 0x08:
      {     //Torque reference (Iq)
        qd_t currComp;
        currComp = MCI_GetIqdref(pMCI[M1]);
        p_tmpryOPData = (int32_t)currComp.q;
        *success = dbmSuccess;
        break;
      }
    case 0x09:
      {     //Torque Kp
        p_tmpryOPData = (int32_t)PID_GetKP(pMCT[M1]->pPIDIq); 
        *success = dbmSuccess;
        break;
      }
    case 0x0A:
      {     //Torque Ki
        p_tmpryOPData = (int32_t)PID_GetKI(pMCT[M1]->pPIDIq);
        *success = dbmSuccess;
        break;
      }
    case 0x0B:
      {     //Torque Kd
        p_tmpryOPData = (int32_t)PID_GetKD(pMCT[M1]->pPIDIq);
        *success = dbmSuccess;
        break;
      }
    case 0x0C:
      {     //Flux reference (Id)
        qd_t currComp;
        currComp = MCI_GetIqdref(pMCI[M1]);
        p_tmpryOPData = (int32_t)currComp.d;
        *success = dbmSuccess;
        break;
      }
    case 0x0D:
      {     //Flux Kp
        p_tmpryOPData = (int32_t)PID_GetKP(pMCT[M1]->pPIDId);
        *success = dbmSuccess;
        break;
      }
    case 0x0E:
      {     //Flux Ki
        p_tmpryOPData = (int32_t)PID_GetKI(pMCT[M1]->pPIDId);
        *success = dbmSuccess;
        break;
      }
    case 0x0F:
      {     //Flux Kd
        p_tmpryOPData = (int32_t)PID_GetKD(pMCT[M1]->pPIDId);
        *success = dbmSuccess;
        break;
      }
    case 0x10:
      {     //Observer C1
        uint32_t hUICfg = wConfig[M1]; //motor setting header in parameters_conversion.h
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        int16_t hC1,hC2;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
        }
        p_tmpryOPData = (int32_t)hC1;
        *success = dbmSuccess;
        break;
      }
    case 0x11:
      {     //Observer C2
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        int16_t hC1,hC2;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
        }
        p_tmpryOPData = (int32_t)hC2;
        *success = dbmSuccess;
        break;
      }
    case 0x14:    //read PLL Ki      
      {   
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        int16_t hPgain, hIgain;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
        }
        p_tmpryOPData = (int32_t)hIgain;
        *success = dbmSuccess;
        break;
      }                                           
    case 0x15:    //read PLL Kp     
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        int16_t hPgain, hIgain;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
        }
        p_tmpryOPData = (int32_t)hPgain;    
        *success = dbmSuccess;
        break;
      }                                        
    case 0x16:    /** @this function not implemented **/   //read Flux weakening Kp                             
    case 0x17:    //read Flux weakening Ki                                        
    case 0x18:    //read Flux weakening BUS Voltage allowed percentage reference                
    case 0x21:    //read Flux weakening BUS Voltage allowed percentage measured   
    case 0x2F:    //read Measured electrical angle                                
    case 0x30:    //read Measured rotor speed    
    case 0x46:    //read Feedforward (1Q)    
    case 0x47:    //read Feedforward (1D)     
    case 0x48:    //read Feedforward (2)       
    case 0x49:    //read Feedforward (VQ)    
    case 0x4A:    //read Feedforward (VD)      
    case 0x4B:    //read Feedforward (VQ PI out)   
    case 0x4C:    //read Feedforward (VD PI out)   
    case 0x4D:    //read PFC Status             
    case 0x4E:    //read PFC Flags               
    case 0x4F:    //read PFC DC bus reference    
    case 0x50:    //read PFC DC bus measured      
    case 0x51:    //read AC Mains frequency       
    case 0x52:    //read AC Mains voltage 0-to-pk  
    case 0x53:    //read PFC Current loop Kp     
    case 0x54:    //read PFC Current loop Ki     
    case 0x55:    //read PFC Current loop Kd      
    case 0x56:    //read PFC Voltage loop Kp      
    case 0x57:    //read PFC Voltage loop Ki     
    case 0x58:    //read PFC Voltage loop Kd    
    case 0x59:    //read PFC startup duration    
    case 0x5A:    //read PFC abilitation status  
    case 0x5C:    //read Ramp duration  
      {     /** @this function not implemented **/
        /** @todolist may put in the non implemented registers **/
        break;
      } 
    case 0x19:    //read Bus Voltage     
      {     
        p_tmpryOPData = (int32_t)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
        *success = dbmSuccess;
        break;
      }                                            
    case 0x1A:    //read Heatsink Temperature       
      {     
        p_tmpryOPData = (int32_t)NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
        *success = dbmSuccess;
        break;
      }                                 
    case 0x1B:    //read Motor Power  
      {     
    //    p_tmpryOPData = Regal_GetAvrgMotorPowerW( &EEPowerCalcHandle_M1 );
        *success = dbmSuccess;
        break;
      }                                               
    case 0x1C:    //read DAC Out 1       
      {     
        p_tmpryOPData =(int32_t) pDAC->bChannel_variable[0];
        *success = dbmSuccess;
        break;
      }                                            
    case 0x1D:    //read DAC Out 2      
      {     
        p_tmpryOPData =(int32_t) pDAC->bChannel_variable[1];
        *success = dbmSuccess;
        break;
      }                                             
    case 0x1E:    //read Speed measured   
      {     
        p_tmpryOPData = (int32_t)((MCI_GetAvrgMecSpeedUnit(pMCI[M1]) * _RPM)/SPEED_UNIT);
        *success = dbmSuccess;
        break;
      }                                           
    case 0x1F:    //read Torque measured (Iq)    
      {     
        p_tmpryOPData = MCI_GetIqd(pMCI[M1]).q;
        *success = dbmSuccess;
        break;
      }                                    
    case 0x20:    //read Flux measured (Id)    
      {     
        p_tmpryOPData = MCI_GetIqd(pMCI[M1]).d;
        *success = dbmSuccess;
        break;
      }                                        
    case 0x22:    //read Revup stage numbers    
      {   
        if (pMCT[M1]->pRevupCtrl)
        {
          p_tmpryOPData = (int32_t)RUC_GetNumberOfPhases(pMCT[M1]->pRevupCtrl);
        }
        else
        {
          p_tmpryOPData = (uint32_t) 0;
        }
        *success = dbmSuccess;
        break;
      }                                     
    case 0x23:    //read Ia
      {     
        p_tmpryOPData = MCI_GetIab(pMCI[M1]).a;
        *success = dbmSuccess;
        break;
      }                                                          
    case 0x24:    //read Ib   
      {     
        p_tmpryOPData = MCI_GetIab(pMCI[M1]).b;
        *success = dbmSuccess;
        break;
      }                                                       
    case 0x25:    //read Ialpha    
      {     
        p_tmpryOPData = MCI_GetIalphabeta(pMCI[M1]).alpha;
        *success = dbmSuccess;
        break;
      }                                                  
    case 0x26:    //read Ibeta    
      {     
        p_tmpryOPData = MCI_GetIalphabeta(pMCI[M1]).beta;
        *success = dbmSuccess;
        break;
      }                                                   
    case 0x27:    //read Iq   
      {     
        p_tmpryOPData = MCI_GetIqd(pMCI[M1]).q;
        *success = dbmSuccess;
        break;
      }                                                       
    case 0x28:    //read Id 
      {     
        p_tmpryOPData = MCI_GetIqd(pMCI[M1]).d;
        *success = dbmSuccess;
        break;
      }                                                         
    case 0x29:    //read Iq reference  
      {     
        p_tmpryOPData = MCI_GetIqdref(pMCI[M1]).q;
        *success = dbmSuccess;
        break;
      }                                              
    case 0x2A:    //read Id reference     
      {     
        p_tmpryOPData = MCI_GetIqdref(pMCI[M1]).d;
        *success = dbmSuccess;
        break;
      }                                           
    case 0x2B:    //read Vq 
      {     
        p_tmpryOPData = MCI_GetVqd(pMCI[M1]).q;
        *success = dbmSuccess;
        break;
      }                                                         
    case 0x2C:    //read Vd  
      {     
        p_tmpryOPData = MCI_GetVqd(pMCI[M1]).d;
        *success = dbmSuccess;
        break;
      }                                                        
    case 0x2D:    //read Valpha 
      {     
        p_tmpryOPData = MCI_GetValphabeta(pMCI[M1]).alpha;
        *success = dbmSuccess;
        break;
      }                                                     
    case 0x2E:    //read Vbeta    
      {   
        p_tmpryOPData = MCI_GetValphabeta(pMCI[M1]).alpha;
        *success = dbmSuccess;
        break;
      }                                                                                     
    case 0x31:    //read Observer electrical angle (PLL)     
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = SPD_GetElAngle(pSPD);
          *success = dbmSuccess;
        }                    
        break;
      }                        
    case 0x32:    //read Observer rotor speed (PLL)    
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = SPD_GetS16Speed(pSPD);
          *success = dbmSuccess;
        }
        break;
      }                              
    case 0x33:    //read Observer Ialpha (PLL)    
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetEstimatedCurrent((STO_PLL_Handle_t*)pSPD).alpha;
          *success = dbmSuccess;
        }
        break;
      }                                   
    case 0x34:    //read Observer Ibeta (PLL)    
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetEstimatedCurrent((STO_PLL_Handle_t*)pSPD).beta;
          *success = dbmSuccess;
        }
        break;
      }                                    
    case 0x35:    //read Observer BEMF alpha (PLL)  
      {     
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD =  pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetEstimatedBemf((STO_PLL_Handle_t*)pSPD).alpha;
          *success = dbmSuccess;
        }
        break;
      }                                 
    case 0x36:    //read Observer BEMF beta (PLL)     
      { 
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
         pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetEstimatedBemf((STO_PLL_Handle_t*)pSPD).beta;
          *success = dbmSuccess;
        }
        break;
      }  
    case 0x3D:		//read User defined DAC 1       
      {
        if (UI_Params.pFctDACGetUserChannelValue)
        {
          p_tmpryOPData = (int32_t) UI_Params.pFctDACGetUserChannelValue(&UI_Params, 0);
        }
        else
        {
          p_tmpryOPData = (uint32_t) 0;
        }
        *success = dbmSuccess;
        break;
      } 
    case 0x3E:    //read User defined DAC 2        
      {
        if (UI_Params.pFctDACGetUserChannelValue)
        {
          p_tmpryOPData = (int32_t) UI_Params.pFctDACGetUserChannelValue(&UI_Params, 1);
        }
        else
        {
          p_tmpryOPData = (uint32_t) 0;
        }
        *success = dbmSuccess;
        break;
      }
    case 0x3F:    //read Maximum application speed 
      {
        p_tmpryOPData = (STC_GetMaxAppPositiveMecSpeedUnit(pMCT[M1]->pSpeednTorqueCtrl) * _RPM)/SPEED_UNIT ;
        *success = dbmSuccess;
        break;
      }
    case 0x40:    //read Minimum application speed 
      {
        p_tmpryOPData = (STC_GetMinAppNegativeMecSpeedUnit(pMCT[M1]->pSpeednTorqueCtrl)  * _RPM)/SPEED_UNIT ;
        *success = dbmSuccess;
        break;
      }
    case 0x41:    //read Id reference in speed mode
      {
        qd_t currComp;
        currComp = MCI_GetIqdref(pMCI[M1]);
        p_tmpryOPData  = (int32_t)currComp.d;
        *success = dbmSuccess;
        break;
      }
    case 0x42:    //read Expected BEMF level (PLL) 
      {
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetEstimatedBemfLevel((STO_PLL_Handle_t*)pSPD) >> 16;
          *success = dbmSuccess;
        }
        break;
      }
    case 0x43:    //read Observed BEMF level (PLL) 
      {
        uint32_t hUICfg = wConfig[M1];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorMain;
        }
        if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
        {
          pSPD = pMCT[M1]->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          p_tmpryOPData = STO_PLL_GetObservedBemfLevel((STO_PLL_Handle_t*)pSPD) >> 16;
          *success = dbmSuccess;
        }
        break;
      } 
    case 0x5B:    //read Ramp final speed 
      {
        if (MCI_GetControlMode(pMCI[M1]) == STC_SPEED_MODE)
        {
          p_tmpryOPData = (int32_t)((MCI_GetLastRampFinalSpeed(pMCI[M1]) * _RPM)/SPEED_UNIT) ;
        }
        else
        {
          p_tmpryOPData = (int32_t)((MCI_GetMecSpeedRefUnit(pMCI[M1]) * _RPM)/SPEED_UNIT) ;
          *success = dbmSuccess;
        }
        break;
      }         

    case 0xC0:    //read Start Up data    //Long data reply command    
      {             
        uint16_t Durationms;
        int16_t FinalMecSpeedUnit;
        int16_t FinalTorque;
        int32_t rpm;
        RevUpCtrl_Handle_t *pRevupCtrl = pMCT[M1]->pRevupCtrl;
        if (pRevupCtrl)
        {
          Durationms = RUC_GetPhaseDurationms(pRevupCtrl, protocolBuf_DebugMCmd[6]);
          FinalMecSpeedUnit = RUC_GetPhaseFinalMecSpeedUnit(pRevupCtrl, protocolBuf_DebugMCmd[6]);
          FinalTorque = RUC_GetPhaseFinalTorque(pRevupCtrl, protocolBuf_DebugMCmd[6]);
        }
        rpm = (FinalMecSpeedUnit * _RPM) / SPEED_UNIT;
        p_outBuff[0] = (uint8_t)(rpm);
        p_outBuff[1] = (uint8_t)(rpm >> 8);
        p_outBuff[2] = (uint8_t)(rpm >> 16);
        p_outBuff[3] = (uint8_t)(rpm >> 24);
        p_outBuff[4] = (uint8_t)(FinalTorque);
        p_outBuff[5] = (uint8_t)(FinalTorque >> 8);
        p_outBuff[6] = (uint8_t)(Durationms);
        p_outBuff[7] = (uint8_t)(Durationms >> 8);
        p_tmpryOPData = -1;
        *success = dbmLongDatSuccess;
        break;
      }
    default:
      break;
  }
  return p_tmpryOPData;
}