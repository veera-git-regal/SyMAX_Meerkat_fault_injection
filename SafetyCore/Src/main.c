/**
  ******************************************************************************
  * @file    main.c 
  * @author  Kyle McBrady
  * @version V1.1.0
  * @date    07-Dec-2018
  * @brief   Main program body to test saftey core
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "clock_check.h"

uint32_t sysTickCount;

// Function Prototypes
#if ENABLE_BUILD_AS_APPLICATION <= 0
    extern void __iar_data_init3(void); // IAR function for initializing variables
    int lib_init(void);

#else // ENABLE_BUILD_AS_APPLICATION > 0
    #warning Building Safety Core as an Application. This is for Debugging purposes only!
    static void LL_Init(void);
    uint32_t getSysTick(void);
    void SystemClock_Config(void);
    int app_init(void);
    
#endif //ENABLE_BUILD_AS_APPLICATION > 0

/**
  * @brief  RAM Initialization routine.
  *
  * @retval None
  * Note: This function must be manually called by the application that uses this library.
  */    
__root int lib_init(void) // Library initialization
{
    // This function must be manually called by the application that uses this library.
    
    // Perform zero initializion and copy initialized data from ROM to RAM
    __iar_data_init3(); 

    return 0;
}


/**
  * @brief  The application entry point.
  *
  * @retval None
  * Note: This is used as entry point when built as an application
  * - This function should never be called by the application that uses this library
  */
int main(void) 
// 
// - when used never be called by our firmware, in the future we can get rid of many of these functions
{
#if ENABLE_BUILD_AS_APPLICATION <= 0
    lib_init();
#else // ENABLE_BUILD_AS_APPLICATION > 0
    app_init();
#endif // ENABLE_BUILD_AS_APPLICATION > 0
}



// === End of Base Code ===
// - Code in this file below this line is only for when building meerkat as a standalone application
// -- this is for debugging purposes only.

#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
int app_init(void) {
    // This condition is for debugging purposes only.
    /* MCU Configuration----------------------------------------------------------*/
    sysTickCount = 0;
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    SysTick_Config(72000000 / 1000);

    LED_Init();

    // Set Up Function Wrappers
    uint32_t absoluteAddress_u32;

    // - Init
    typedef void (*SuperVisorInitPointer)(void);
    SuperVisorInitPointer SuperVisorInitPtr;
    absoluteAddress_u32 = (uint32_t) &SupervisorInitialize_wrapper;
    SuperVisorInitPtr = ((SuperVisorInitPointer)(absoluteAddress_u32));
    
    // - Init Shared Ram
    SuperVisorInitPointer SuperVisorInitSharedRamPtr;
    absoluteAddress_u32 = (uint32_t) &SupervisorInitializeSharedRam_wrapper;
    SuperVisorInitSharedRamPtr = ((SuperVisorInitPointer)(absoluteAddress_u32));
    
    // - Run
    typedef uint64_t (*SuperVisorRunPointer)(uint8_t);
    SuperVisorRunPointer SuperVisorRunPtr;
    absoluteAddress_u32 = (uint32_t) &SupervisorRun_wrapper;
    SuperVisorRunPtr = ((SuperVisorRunPointer)(absoluteAddress_u32));

    // - Add Current Sample
    typedef void (*SuperVisor_ADCCheck_AddCurrentSample_Pointer)(int16_t, int16_t, int16_t);
    SuperVisor_ADCCheck_AddCurrentSample_Pointer SuperVisor_ADCCheck_AddCurrentSamplePtr;
    absoluteAddress_u32 = (uint32_t) &Supervisor_ADCCheck_AddCurrentSample_wrapper;
    SuperVisor_ADCCheck_AddCurrentSamplePtr = ((SuperVisor_ADCCheck_AddCurrentSample_Pointer)(absoluteAddress_u32));

    SuperVisorInitPointer SuperVisorIwdgPtr;
    absoluteAddress_u32 = (uint32_t) &SupervisorCheckIwdgResetFlag_wrapper;
    SuperVisorIwdgPtr = ((SuperVisorInitPointer)(absoluteAddress_u32));

//    // - Add Speed Sample
//    typedef void (*SuperVisor_ADCCheck_AddSpeedSample_Pointer)(int16_t);
//    SuperVisor_ADCCheck_AddSpeedSample_Pointer SuperVisor_ADCCheck_AddSpeedSamplePtr;
//    absoluteAddress_u32 = (uint32_t) &Supervisor_ADCCheck_AddSpeedSample_wrapper;
//    SuperVisor_ADCCheck_AddSpeedSamplePtr = ((SuperVisor_ADCCheck_AddSpeedSample_Pointer)(absoluteAddress_u32));
    
    // Initialize Safety Core
    SuperVisorInitSharedRamPtr(); // note must come after checking IWDG Reset Flag
    SuperVisorInitPtr();
    SuperVisor_ADCCheck_AddCurrentSamplePtr(0,0,0);
    SuperVisorIwdgPtr();
//    SuperVisor_ADCCheck_AddSpeedSamplePtr(0);
    uint32_t lastExecutionTick = getSysTick();
    
    while (1) { // main loop
        uint32_t thisExecutionTick = getSysTick();
        if (lastExecutionTick != thisExecutionTick) { // only allow running a maximum of every 1ms
            SuperVisorRunPtr(0); // Run Safety Core  (0=motor is idle)          
            lastExecutionTick = thisExecutionTick;
        }
    }
}
#endif


#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
static void LL_Init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* BusFault_IRQn interrupt configuration */
    NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* UsageFault_IRQn interrupt configuration */
    NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* SVCall_IRQn interrupt configuration */
    NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* DebugMonitor_IRQn interrupt configuration */
    NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* PendSV_IRQn interrupt configuration */
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}
#endif //ENABLE_BUILD_AS_APPLICATION > 0


#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {
    }
    LL_RCC_HSI_SetCalibTrimming(16);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);

    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

    LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_Init1msTick(72000000);

    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

    LL_SetSystemCoreClock(72000000);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}
#endif // ENABLE_BUILD_AS_APPLICATION > 0


#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
void IncSysTick(void) {
    sysTickCount++;
}
#endif // ENABLE_BUILD_AS_APPLICATION > 0

#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
uint32_t getSysTick(void) {
    return sysTickCount;
}
#endif // ENABLE_BUILD_AS_APPLICATION > 0


#if ENABLE_BUILD_AS_APPLICATION > 0
// For Debugging only!
/**
  * @brief  Initialize LED1.
  * @param  None
  * @retval None
  */
void LED_Init(void) {
    /* Enable the LED1 Clock */
    LED1_GPIO_CLK_ENABLE();

    /* Configure IO in output push-pull mode to drive external LED2 */
    LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
    /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
    LL_GPIO_SetPinOutputType(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
    LL_GPIO_SetPinSpeed(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_SPEED_FREQ_LOW);
    /* Reset value is LL_GPIO_PULL_NO */
    LL_GPIO_SetPinPull(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_PULL_NO);
}
///**
//  * @brief  Turn-on LED2.
//  * @param  None
//  * @retval None
//  */
//void LED_On(void) // REVIEW: Unused
//{
//    /* Turn LED2 on */
//    LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_PIN);
//}
//
///**
//  * @brief  Turn-off LED2.
//  * @param  None
//  * @retval None
//  */
//void LED_Off(void) // REVIEW: Unusued
//{
//    /* Turn LED2 off */
//    LL_GPIO_ResetOutputPin(LED1_GPIO_PORT, LED1_PIN);
//}

#endif // ENABLE_BUILD_AS_APPLICATION > 0
