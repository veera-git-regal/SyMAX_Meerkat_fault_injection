/**
 ********************************************************************************************************************************
 * @file    driver_usart2.c
 * @author  Doug Oda
 * @brief   Hardware and functional handers for USART2
 * @details
 ********************************************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "driver_usart2.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "hardware_config.h"
#include "module_modbus.h"

/* Private variables ---------------------------------------------------------*/
volatile uint8_t Usart2_TxIndex_u8 = 0;
uint8_t Usart2_characters_to_send_u8 = 0;
uint8_t Usart2_TxBuffer[USART2_TX_RX_BUF_SIZE];
Usart2_Control usart2Control;
extern ModbusRtu_Control modbusRtu_Control;
const uint16_t baud_rates_table_u16[] = {24, 48, 72, 96, 144, 192, 384, 576, 1152};
/* Prototypes ----------------------------------------------------------------*/
void USART2_CharReception_Callback(void);
void Usart2_ErrorCallback(void);
void USART2_CharTransmitComplete_Callback(void);
void USART2_TXEmpty_Callback(void);
void MX_USART2_UART_Init(void);

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Wrapper for USART2 Initialization Function
 * @param None
 * @retval None
 */
void Usart2_Init(void)
{
  MX_USART2_UART_Init();
}

/**
 * @brief Periodic call to process outgoing data
 * @param None
 * @retval None
 */
void TxProcess_Usart2(void)
{
  if ((Usart2_TxIndex_u8 == 0) && (LL_USART_IsActiveFlag_TXE(USART2)))
  {
    Usart2_characters_to_send_u8 = RingBuf_GetUsedNumOfElements((usart2Control).seqMemTX); // set TX length
    uint32_t buffer_length_u8 = Usart2_characters_to_send_u8;
    RingBuf_ReadBlock((usart2Control).seqMemTX, Usart2_TxBuffer, &buffer_length_u8); // copy new message into tx buffer
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART2, Usart2_TxBuffer[Usart2_TxIndex_u8++]);
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART2);
  }
}

/**
 * @brief Periodic call to copy data to buffer for processing
 * @param None
 * @retval None
 */
void RxProcess_Usart2(void)
{
  uint8_t temporary_buffer[USART2_TX_RX_BUF_SIZE];
  uint32_t length_u32 = RingBuf_GetUsedNumOfElements((usart2Control).seqMem_RawRx);
  if (length_u32 > USART2_TX_RX_BUF_SIZE)
  {
    // this is very bad. Clear everything and start over
    RingBuf_ClearContents((usart2Control).seqMem_RawRx);
    RingBuf_ClearContents((usart2Control).seqMemModbus);
  }
  else
  {
    // life is good, copy the data from the raw buffer to the UP evaluation buffer
    RingBuf_ReadBlock((usart2Control).seqMem_RawRx, &temporary_buffer[0], &length_u32);
    RingBuf_WriteBlock((usart2Control).seqMemModbus, &temporary_buffer[0], &length_u32);
  }
}

/* Interrupt Handlers --------------------------------------------------------*/
/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
 */
void USART2_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART2_CharReception_Callback();
  }

  if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
  {
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART2_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART2);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART2_CharTransmitComplete_Callback();
  }

  if (LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
  {
    /* Call Error function */
    Usart2_ErrorCallback();
  }

  LL_USART_ClearFlag_ORE(USART2); // TODO: Upgrade this temporary patch that keeps us from getting stuck in this ISR
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  // USART_InitStruct.BaudRate = DEFAULT_BAUD_RATE;
  if (modbusRtu_Control.modbusRtu_Settings.baudRate_u16 == 0 || modbusRtu_Control.modbusRtu_Settings.baudRate_u16 == 0xFFFF)
  {
    USART_InitStruct.BaudRate = DEFAULT_BAUD_RATE;
  }
  else
  {
    // Look up user set baud rate to ensure it is a valid baudrate
    uint8_t is_baudRateFound = FALSE;
    for (uint8_t index_u8 = 0; index_u8 < sizeof(baud_rates_table_u16); index_u8++)
    { // Look up user baud rate in look up table
      if (baud_rates_table_u16[index_u8] == modbusRtu_Control.modbusRtu_Settings.baudRate_u16)
        is_baudRateFound = TRUE;
    }
    if (is_baudRateFound == TRUE)
    { // user baud rate is valid
      USART_InitStruct.BaudRate = modbusRtu_Control.modbusRtu_Settings.baudRate_u16 * BAUD_RATE_FACTOR;
    }
    else
    { // User baud rate is not valid
      USART_InitStruct.BaudRate = DEFAULT_BAUD_RATE;
      modbusRtu_Control.modbusRtu_Settings.baudRate_u16 = 1152;
    }
  }
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */
  LL_USART_ClearFlag_ORE(USART2); // reset all usart error bit
  /* Polling USART2 initialisation */
  while ((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief  Function called from USART IRQ Handler when RXNE flag is set
 *         Function is in charge of reading character received on USART RX line.
 * @param  None
 * @retval None
 */
void USART2_CharReception_Callback(void)
{
  uint8_t rx_data_u8;
  rx_data_u8 = LL_USART_ReceiveData8(USART2);
  RingBuf_WriteCharacter((usart2Control).seqMem_RawRx, &rx_data_u8);
}

/**
 * @brief  Function called for achieving next TX Byte sending
 * @param  None
 * @retval None
 */
void USART2_TXEmpty_Callback(void)
{
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART2, Usart2_TxBuffer[Usart2_TxIndex_u8++]);

  if (Usart2_TxIndex_u8 == (Usart2_characters_to_send_u8))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART2);
    Usart2_TxIndex_u8 = 0;
  }
}

/**
 * @brief  Function called at completion of last byte transmission
 * @param  None
 * @retval None
 */
void USART2_CharTransmitComplete_Callback(void)
{
  // nothign to do here, this is handled when we stop loading from the
  // buffer to the transmit register
}

/**
 * @brief  Function called in case of error detected in USART IT Handler
 * @param  None
 * @retval None
 */
void Usart2_ErrorCallback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART2_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART2, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
    //    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    //    LED_Blinking(LED_BLINK_ERROR);
  }
}
