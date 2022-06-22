/**
  ********************************************************************************************************************************
  * @file    interrupt_wrappers.c
  * @author  Justin Moon
  * @brief   Implementation of c++ function/s to allow locking of interrupt vector table
  * @details Interrupt Vector table is fixed in this firmware to 'brick-proof' the device's internal bootloader, by
  *             The table calls these wrapper functions, which only contain calls to the real function
  *             Since they are only function calls, their locations can be fixed in ROM.
  *             In order to operate with wrappers, these must be placed in the vector table in the projects 'startup_*.s' file
  *             - startup_*.s is typically located in the EWARM folder.
  ********************************************************************************************************************************
  */


// Compiler Version Checking
// The version number in integer format; for example, version 6.21.2 is returned as 6021002.
#if __VER__ == 8050009 // 8.50.9 -> 8 050 009
  // #warning Compiler Version 8.50.9 detected.
#else
  #error Compiler version is not supported.
#endif

// #define INTERRUPT_VECTOR_WRAP_BASE_ADDRESS 0x08001000
// #define INTERRUPT_VECTOR_WRAPPER_SIZE 4

// __vector_table
// PROTECTED AREA - Do not Create wrappers for this area
//         DCD     sfe(CSTACK) 
//         DCD     Reset_Handler             ; Reset Handler

// NON PROTECTED AREA - Create wrappers for this area
//         DCD     NMI_Handler               ; NMI Handler
extern void NMI_Handler(void);
//         DCD     HardFault_Handler         ; Hard Fault Handler
extern void HardFault_Handler(void);
//         DCD     MemManage_Handler         ; MPU Fault Handler
extern void MemManage_Handler(void);
//         DCD     BusFault_Handler          ; Bus Fault Handler
extern void BusFault_Handler(void);
//         DCD     UsageFault_Handler        ; Usage Fault Handler
extern void UsageFault_Handler(void);
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     SVC_Handler               ; SVCall Handler
extern void SVC_Handler(void);
//         DCD     DebugMon_Handler          ; Debug Monitor Handler
extern void DebugMon_Handler(void);
//         DCD     0                         ; Reserved
//         DCD     PendSV_Handler            ; PendSV Handler
extern void PendSV_Handler(void);
//         DCD     SysTick_Handler           ; SysTick Handler
extern void SysTick_Handler(void);

//         ; External Interrupts
//         DCD     WWDG_IRQHandler                   ; Window WatchDog
extern void WWDG_IRQHandler(void);
//         DCD     PVD_IRQHandler                    ; PVD through EXTI Line detection
extern void PVD_IRQHandler(void);
//         DCD     TAMP_STAMP_IRQHandler             ; Tamper and TimeStamps through the EXTI line
extern void TAMP_STAMP_IRQHandler(void);
//         DCD     RTC_WKUP_IRQHandler               ; RTC Wakeup through the EXTI line
extern void RTC_WKUP_IRQHandler(void);
//         DCD     FLASH_IRQHandler                  ; FLASH
extern void FLASH_IRQHandler(void);
//         DCD     RCC_IRQHandler                    ; RCC
extern void RCC_IRQHandler(void);
//         DCD     EXTI0_IRQHandler                  ; EXTI Line0
extern void EXTI0_IRQHandler(void);
//         DCD     EXTI1_IRQHandler                  ; EXTI Line1
extern void EXTI1_IRQHandler(void);
//         DCD     EXTI2_TSC_IRQHandler              ; EXTI Line2 and Touch Sense controller
extern void EXTI2_TSC_IRQHandler(void);
//         DCD     EXTI3_IRQHandler                  ; EXTI Line3
extern void EXTI3_IRQHandler(void);
//         DCD     EXTI4_IRQHandler                  ; EXTI Line4
extern void EXTI4_IRQHandler(void);
//         DCD     DMA1_Channel1_IRQHandler          ; DMA1 Channel 1
extern void DMA1_Channel1_IRQHandler(void);
//         DCD     DMA1_Channel2_IRQHandler          ; DMA1 Channel 2
extern void DMA1_Channel2_IRQHandler(void);
//         DCD     DMA1_Channel3_IRQHandler          ; DMA1 Channel 3
extern void DMA1_Channel3_IRQHandler(void);
//         DCD     DMA1_Channel4_IRQHandler          ; DMA1 Channel 4
extern void DMA1_Channel4_IRQHandler(void);
//         DCD     DMA1_Channel5_IRQHandler          ; DMA1 Channel 5
extern void DMA1_Channel5_IRQHandler(void);
//         DCD     DMA1_Channel6_IRQHandler          ; DMA1 Channel 6
extern void DMA1_Channel6_IRQHandler(void);
//         DCD     DMA1_Channel7_IRQHandler          ; DMA1 Channel 7
extern void DMA1_Channel7_IRQHandler(void);
//         DCD     ADC1_IRQHandler                   ; ADC1
extern void ADC1_IRQHandler(void);
//         DCD     USB_HP_CAN_TX_IRQHandler          ; USB Device High Priority or CAN TX
extern void USB_HP_CAN_TX_IRQHandler(void);
//         DCD     USB_LP_CAN_RX0_IRQHandler         ; USB Device Low Priority or CAN RX0
extern void USB_LP_CAN_RX0_IRQHandler(void);
//         DCD     CAN_RX1_IRQHandler                ; CAN RX1
extern void CAN_RX1_IRQHandler(void);
//         DCD     CAN_SCE_IRQHandler                ; CAN SCE
extern void CAN_SCE_IRQHandler(void);
//         DCD     EXTI9_5_IRQHandler                ; External Line[9:5]s
extern void EXTI9_5_IRQHandler(void);
//         DCD     TIM1_BRK_TIM15_IRQHandler         ; TIM1 Break and TIM15
extern void TIM1_BRK_TIM15_IRQHandler(void);
//         DCD     TIM1_UP_TIM16_IRQHandler          ; TIM1 Update and TIM16
extern void TIM1_UP_TIM16_IRQHandler(void);
//         DCD     TIM1_TRG_COM_TIM17_IRQHandler     ; TIM1 Trigger and Commutation and TIM17
extern void TIM1_TRG_COM_TIM17_IRQHandler(void);
//         DCD     TIM1_CC_IRQHandler                ; TIM1 Capture Compare
extern void TIM1_CC_IRQHandler(void);
//         DCD     TIM2_IRQHandler                   ; TIM2
extern void TIM2_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     I2C1_EV_IRQHandler                ; I2C1 Event
extern void I2C1_EV_IRQHandler(void);
//         DCD     I2C1_ER_IRQHandler                ; I2C1 Error
extern void I2C1_ER_IRQHandler(void);
//         DCD     I2C2_EV_IRQHandler                ; I2C2 Event
extern void I2C2_EV_IRQHandler(void);
//         DCD     I2C2_ER_IRQHandler                ; I2C2 Error
extern void I2C2_ER_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     SPI2_IRQHandler                   ; SPI2
extern void SPI2_IRQHandler(void);
//         DCD     USART1_IRQHandler_w                 ; USART1
extern void USART1_IRQHandler(void);
//         DCD     USART2_IRQHandler_w                 ; USART2
extern void USART2_IRQHandler(void);
//         DCD     USART3_IRQHandler_w                 ; USART3
extern void USART3_IRQHandler(void);

//         DCD     EXTI15_10_IRQHandler              ; External Line[15:10]s
extern void EXTI15_10_IRQHandler(void);
//         DCD     RTC_Alarm_IRQHandler              ; RTC Alarm (A and B) through EXTI Line
extern void RTC_Alarm_IRQHandler(void);
//         DCD     USBWakeUp_IRQHandler              ; USB Wakeup through EXTI line
extern void USBWakeUp_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     SPI3_IRQHandler                   ; SPI3
extern void SPI3_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     TIM6_DAC_IRQHandler               ; TIM6 and DAC1 underrun errors
extern void TIM6_DAC_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     COMP2_IRQHandler                  ; COMP2
extern void COMP2_IRQHandler(void);
//         DCD     COMP4_6_IRQHandler                ; COMP4 and COMP6
extern void COMP4_6_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     I2C3_EV_IRQHandler                ; I2C3 Event                                             
extern void I2C3_EV_IRQHandler(void);
//         DCD     I2C3_ER_IRQHandler                ; I2C3 Error                                             
extern void I2C3_ER_IRQHandler(void);
//         DCD     USB_HP_IRQHandler                 ; USB High Priority remap
extern void USB_HP_IRQHandler(void);
//         DCD     USB_LP_IRQHandler                 ; USB Low Priority remap
extern void USB_LP_IRQHandler(void);
//         DCD     USBWakeUp_RMP_IRQHandler          ; USB Wakeup remap through EXTI
extern void USBWakeUp_RMP_IRQHandler(void);
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     FPU_IRQHandler                    ; FPU
extern void FPU_IRQHandler(void);


// PROTECTED AREA - Do not Create wrappers for this area
//         DCD     sfe(CSTACK) 
//         DCD     Reset_Handler             ; Reset Handler

// NON PROTECTED AREA - Create wrappers for this area
//         DCD     NMI_Handler               ; NMI Handler
void NMI_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      NMI_Handler();
}
//         DCD     HardFault_Handler         ; Hard Fault Handler
void HardFault_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      HardFault_Handler();
}
//         DCD     MemManage_Handler         ; MPU Fault Handler
void MemManage_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      MemManage_Handler();
}
//         DCD     BusFault_Handler          ; Bus Fault Handler
void BusFault_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      BusFault_Handler();
}
//         DCD     UsageFault_Handler        ; Usage Fault Handler
void UsageFault_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      UsageFault_Handler();
}
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     0                         ; Reserved
//         DCD     SVC_Handler               ; SVCall Handler
void SVC_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      SVC_Handler();
}
//         DCD     DebugMon_Handler          ; Debug Monitor Handler
void DebugMon_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      DebugMon_Handler();
}
//         DCD     0                         ; Reserved
//         DCD     PendSV_Handler            ; PendSV Handler
void PendSV_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      PendSV_Handler();
}
//         DCD     SysTick_Handler           ; SysTick Handler
void SysTick_Handler_w(void) @ "section_interrupt_vector_wrappers"
{
      SysTick_Handler();
}
//         ; External Interrupts
//         DCD     WWDG_IRQHandler                   ; Window WatchDog
void WWDG_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      WWDG_IRQHandler();
}
//         DCD     PVD_IRQHandler                    ; PVD through EXTI Line detection
void PVD_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      PVD_IRQHandler();
}
//         DCD     TAMP_STAMP_IRQHandler             ; Tamper and TimeStamps through the EXTI line
void TAMP_STAMP_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TAMP_STAMP_IRQHandler();
}
//         DCD     RTC_WKUP_IRQHandler               ; RTC Wakeup through the EXTI line
void RTC_WKUP_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      RTC_WKUP_IRQHandler();
}
//         DCD     FLASH_IRQHandler                  ; FLASH
void FLASH_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      FLASH_IRQHandler();
}
//         DCD     RCC_IRQHandler                    ; RCC
void RCC_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      RCC_IRQHandler();
}
//         DCD     EXTI0_IRQHandler                  ; EXTI Line0
void EXTI0_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI0_IRQHandler();
}
//         DCD     EXTI1_IRQHandler                  ; EXTI Line1
void EXTI1_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI1_IRQHandler();
}
//         DCD     EXTI2_TSC_IRQHandler              ; EXTI Line2 and Touch Sense controller
void EXTI2_TSC_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI2_TSC_IRQHandler();
}
//         DCD     EXTI3_IRQHandler                  ; EXTI Line3
void EXTI3_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI3_IRQHandler();
}
//         DCD     EXTI4_IRQHandler                  ; EXTI Line4
void EXTI4_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI4_IRQHandler();
}
//         DCD     DMA1_Channel1_IRQHandler          ; DMA1 Channel 1
void DMA1_Channel1_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel1_IRQHandler();
}
//         DCD     DMA1_Channel2_IRQHandler          ; DMA1 Channel 2
void DMA1_Channel2_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel2_IRQHandler();
}
//         DCD     DMA1_Channel3_IRQHandler          ; DMA1 Channel 3
void DMA1_Channel3_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel3_IRQHandler();
}
//         DCD     DMA1_Channel4_IRQHandler          ; DMA1 Channel 4
void DMA1_Channel4_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel4_IRQHandler();
}
//         DCD     DMA1_Channel5_IRQHandler          ; DMA1 Channel 5
void DMA1_Channel5_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel5_IRQHandler();
}
//         DCD     DMA1_Channel6_IRQHandler          ; DMA1 Channel 6
void DMA1_Channel6_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel6_IRQHandler();
}
//         DCD     DMA1_Channel7_IRQHandler          ; DMA1 Channel 7
void DMA1_Channel7_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      DMA1_Channel7_IRQHandler();
}
//         DCD     ADC1_IRQHandler                   ; ADC1
void ADC1_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      ADC1_IRQHandler();
}
//         DCD     USB_HP_CAN_TX_IRQHandler          ; USB Device High Priority or CAN TX
void USB_HP_CAN_TX_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USB_HP_CAN_TX_IRQHandler();
}
//         DCD     USB_LP_CAN_RX0_IRQHandler         ; USB Device Low Priority or CAN RX0
void USB_LP_CAN_RX0_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USB_LP_CAN_RX0_IRQHandler();
}
//         DCD     CAN_RX1_IRQHandler                ; CAN RX1
void CAN_RX1_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      CAN_RX1_IRQHandler();
}
//         DCD     CAN_SCE_IRQHandler                ; CAN SCE
void CAN_SCE_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      CAN_SCE_IRQHandler();
}
//         DCD     EXTI9_5_IRQHandler                ; External Line[9:5]s
void EXTI9_5_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI9_5_IRQHandler();
}
//         DCD     TIM1_BRK_TIM15_IRQHandler         ; TIM1 Break and TIM15
void TIM1_BRK_TIM15_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM1_BRK_TIM15_IRQHandler();
}
//         DCD     TIM1_UP_TIM16_IRQHandler          ; TIM1 Update and TIM16
void TIM1_UP_TIM16_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM1_UP_TIM16_IRQHandler();
}
//         DCD     TIM1_TRG_COM_TIM17_IRQHandler     ; TIM1 Trigger and Commutation and TIM17
void TIM1_TRG_COM_TIM17_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM1_TRG_COM_TIM17_IRQHandler();
}
//         DCD     TIM1_CC_IRQHandler                ; TIM1 Capture Compare
void TIM1_CC_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM1_CC_IRQHandler();
}
//         DCD     TIM2_IRQHandler                   ; TIM2
void TIM2_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM2_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     I2C1_EV_IRQHandler                ; I2C1 Event
void I2C1_EV_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C1_EV_IRQHandler();
}
//         DCD     I2C1_ER_IRQHandler                ; I2C1 Error
void I2C1_ER_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C1_ER_IRQHandler();
}
//         DCD     I2C2_EV_IRQHandler                ; I2C2 Event
void I2C2_EV_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C2_EV_IRQHandler();
}
//         DCD     I2C2_ER_IRQHandler                ; I2C2 Error
void I2C2_ER_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C2_ER_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     SPI2_IRQHandler                   ; SPI2
void SPI2_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      SPI2_IRQHandler();
}
//         DCD     USART1_IRQHandler_w                 ; USART1
void USART1_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USART1_IRQHandler();
}
//         DCD     USART2_IRQHandler_w                 ; USART2
void USART2_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USART2_IRQHandler();
}
//         DCD     USART3_IRQHandler_w                 ; USART3
void USART3_IRQHandler_w(void)  @ "section_interrupt_vector_wrappers"
{
      USART3_IRQHandler();
}
//         DCD     EXTI15_10_IRQHandler              ; External Line[15:10]s
void EXTI15_10_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      EXTI15_10_IRQHandler();
}
//         DCD     RTC_Alarm_IRQHandler              ; RTC Alarm (A and B) through EXTI Line
void RTC_Alarm_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      RTC_Alarm_IRQHandler();
}
//         DCD     USBWakeUp_IRQHandler              ; USB Wakeup through EXTI line
void USBWakeUp_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USBWakeUp_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     SPI3_IRQHandler                   ; SPI3
void SPI3_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      SPI3_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     TIM6_DAC_IRQHandler               ; TIM6 and DAC1 underrun errors
void TIM6_DAC_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      TIM6_DAC_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     COMP2_IRQHandler                  ; COMP2
void COMP2_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      COMP2_IRQHandler();
}
//         DCD     COMP4_6_IRQHandler                ; COMP4 and COMP6
void COMP4_6_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      COMP4_6_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     I2C3_EV_IRQHandler                ; I2C3 Event       
void I2C3_EV_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C3_EV_IRQHandler();
}                                      
//         DCD     I2C3_ER_IRQHandler                ; I2C3 Error        
void I2C3_ER_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      I2C3_ER_IRQHandler();
}                                     
//         DCD     USB_HP_IRQHandler                 ; USB High Priority remap
void USB_HP_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USB_HP_IRQHandler();
}
//         DCD     USB_LP_IRQHandler                 ; USB Low Priority remap
void USB_LP_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USB_LP_IRQHandler();
}
//         DCD     USBWakeUp_RMP_IRQHandler          ; USB Wakeup remap through EXTI
void USBWakeUp_RMP_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      USBWakeUp_RMP_IRQHandler();
}
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     0                                 ; Reserved
//         DCD     FPU_IRQHandler                    ; FPU
void FPU_IRQHandler_w(void) @ "section_interrupt_vector_wrappers"
{
      FPU_IRQHandler();
}
