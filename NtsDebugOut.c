#if USE_USART

#pragma GCC optimize ("Os")

#define USE_PRECALC 1

#define STM32H725xx
#include "stm32h725xx.h"
#include "core_cm7.h"

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

#include <stdarg.h>
#include <stdlib.h>


extern uint8_t * itoa(int n, char *buffer, int radix);
extern uint8_t * utoa(unsigned int n, char *buffer, int radix);

const uint8_t LL_RCC_PrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

#define assert_param(expr) ((void)0U)

#define CHECK_NAN_INF 1

#if !USE_PRECALC
/**
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
static uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return LL_RCC_CALC_HCLK_FREQ(SYSCLK_Frequency, LL_RCC_GetAHBPrescaler());
}

/**
  * @brief  Return PCLK1 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK1 clock frequency (in Hz)
  */
static uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK1 clock frequency */
  return LL_RCC_CALC_PCLK1_FREQ(HCLK_Frequency, LL_RCC_GetAPB1Prescaler());
}

/**
  * @brief  Return PCLK2 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK2 clock frequency (in Hz)
  */
static uint32_t RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK2 clock frequency */
  return LL_RCC_CALC_PCLK2_FREQ(HCLK_Frequency, LL_RCC_GetAPB2Prescaler());
}

/**
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
static uint32_t RCC_GetSystemClockFreq(void)
{
  uint32_t frequency = 0U;
  LL_PLL_ClocksTypeDef PLL_Clocks;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (LL_RCC_GetSysClkSource())
  {
    /* No check on Ready: Won't be selected by hardware if not */
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:
      frequency = HSI_VALUE >> (LL_RCC_HSI_GetDivider() >> RCC_CR_HSIDIV_Pos);
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_CSI:
      frequency = CSI_VALUE;
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:
      frequency = HSE_VALUE;
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL1:
      LL_RCC_GetPLL1ClockFreq(&PLL_Clocks);
      frequency = PLL_Clocks.PLL_P_Frequency;
      break;

    default:
      /* Nothing to do */
      break;
  }

  return frequency;
}

/**
  * @brief  Helper function to calculate the PLL frequency output
  * @note ex: @ref LL_RCC_CalcPLLClockFreq (HSE_VALUE, @ref LL_RCC_PLL1_GetM (),
  *             @ref LL_RCC_PLL1_GetN (), @ref LL_RCC_PLL1_GetFRACN (), @ref LL_RCC_PLL1_GetP ());
  * @param  PLLInputFreq PLL Input frequency (based on HSE/(HSI/HSIDIV)/CSI)
  * @param  M      Between 1 and 63
  * @param  N      Between 4 and 512
  * @param  FRACN  Between 0 and 0x1FFF
  * @param  PQR    VCO output divider (P, Q or R)
  *                Between 1 and 128, except for PLL1P Odd value not allowed
  * @retval PLL1 clock frequency (in Hz)
  */
uint32_t LL_RCC_CalcPLLClockFreq(uint32_t PLLInputFreq, uint32_t M, uint32_t N, uint32_t FRACN, uint32_t PQR)
{
  float_t freq;

  freq = ((float_t)PLLInputFreq / (float_t)M) * ((float_t)N + ((float_t)FRACN / (float_t)0x2000));

  freq = freq / (float_t)PQR;

  return (uint32_t)freq;
}

/**
  * @brief  Return PLL1 clocks frequencies
  * @note   LL_RCC_PERIPH_FREQUENCY_NO returned for non activated output or oscillator not ready
  * @retval None
  */
void LL_RCC_GetPLL1ClockFreq(LL_PLL_ClocksTypeDef *PLL_Clocks)
{
  uint32_t pllinputfreq = LL_RCC_PERIPH_FREQUENCY_NO, pllsource;
  uint32_t m, n, fracn = 0U;

  /* PLL_VCO = (HSE_VALUE, CSI_VALUE or HSI_VALUE/HSIDIV) / PLLM * (PLLN + FRACN)
     SYSCLK = PLL_VCO / PLLP
  */
  pllsource = LL_RCC_PLL_GetSource();

  switch (pllsource)
  {
    case LL_RCC_PLLSOURCE_HSI:
      if (LL_RCC_HSI_IsReady() != 0U)
      {
        pllinputfreq = HSI_VALUE >> (LL_RCC_HSI_GetDivider() >> RCC_CR_HSIDIV_Pos);
      }
      break;

    case LL_RCC_PLLSOURCE_CSI:
      if (LL_RCC_CSI_IsReady() != 0U)
      {
        pllinputfreq = CSI_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_HSE:
      if (LL_RCC_HSE_IsReady() != 0U)
      {
        pllinputfreq = HSE_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_NONE:
    default:
      /* PLL clock disabled */
      break;
  }

  PLL_Clocks->PLL_P_Frequency = 0U;
  PLL_Clocks->PLL_Q_Frequency = 0U;
  PLL_Clocks->PLL_R_Frequency = 0U;

  m = LL_RCC_PLL1_GetM();
  n = LL_RCC_PLL1_GetN();
  if (LL_RCC_PLL1FRACN_IsEnabled() != 0U)
  {
    fracn = LL_RCC_PLL1_GetFRACN();
  }

  if (m != 0U)
  {
    if (LL_RCC_PLL1P_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_P_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL1_GetP());
    }

    if (LL_RCC_PLL1Q_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_Q_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL1_GetQ());
    }

    if (LL_RCC_PLL1R_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_R_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL1_GetR());
    }
  }
}

/**
  * @brief  Return PLL2 clocks frequencies
  * @note   LL_RCC_PERIPH_FREQUENCY_NO returned for non activated output or oscillator not ready
  * @retval None
  */
void LL_RCC_GetPLL2ClockFreq(LL_PLL_ClocksTypeDef *PLL_Clocks)
{
  uint32_t pllinputfreq = LL_RCC_PERIPH_FREQUENCY_NO, pllsource;
  uint32_t m, n, fracn = 0U;

  /* PLL_VCO = (HSE_VALUE, CSI_VALUE or HSI_VALUE/HSIDIV) / PLLM * (PLLN + FRACN)
     SYSCLK = PLL_VCO / PLLP
  */
  pllsource = LL_RCC_PLL_GetSource();

  switch (pllsource)
  {
    case LL_RCC_PLLSOURCE_HSI:
      if (LL_RCC_HSI_IsReady() != 0U)
      {
        pllinputfreq = HSI_VALUE >> (LL_RCC_HSI_GetDivider() >> RCC_CR_HSIDIV_Pos);
      }
      break;

    case LL_RCC_PLLSOURCE_CSI:
      if (LL_RCC_CSI_IsReady() != 0U)
      {
        pllinputfreq = CSI_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_HSE:
      if (LL_RCC_HSE_IsReady() != 0U)
      {
        pllinputfreq = HSE_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_NONE:
    default:
      /* PLL clock disabled */
      break;
  }

  PLL_Clocks->PLL_P_Frequency = 0U;
  PLL_Clocks->PLL_Q_Frequency = 0U;
  PLL_Clocks->PLL_R_Frequency = 0U;

  m = LL_RCC_PLL2_GetM();
  n = LL_RCC_PLL2_GetN();
  if (LL_RCC_PLL2FRACN_IsEnabled() != 0U)
  {
    fracn = LL_RCC_PLL2_GetFRACN();
  }

  if (m != 0U)
  {
    if (LL_RCC_PLL2P_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_P_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL2_GetP());
    }

    if (LL_RCC_PLL2Q_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_Q_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL2_GetQ());
    }

    if (LL_RCC_PLL2R_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_R_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL2_GetR());
    }
  }
}

/**
  * @brief  Return PLL3 clocks frequencies
  * @note   LL_RCC_PERIPH_FREQUENCY_NO returned for non activated output or oscillator not ready
  * @retval None
  */
void LL_RCC_GetPLL3ClockFreq(LL_PLL_ClocksTypeDef *PLL_Clocks)
{
  uint32_t pllinputfreq = LL_RCC_PERIPH_FREQUENCY_NO, pllsource;
  uint32_t m, n, fracn = 0U;

  /* PLL_VCO = (HSE_VALUE, CSI_VALUE or HSI_VALUE/HSIDIV) / PLLM * (PLLN + FRACN)
     SYSCLK = PLL_VCO / PLLP
  */
  pllsource = LL_RCC_PLL_GetSource();

  switch (pllsource)
  {
    case LL_RCC_PLLSOURCE_HSI:
      if (LL_RCC_HSI_IsReady() != 0U)
      {
        pllinputfreq = HSI_VALUE >> (LL_RCC_HSI_GetDivider() >> RCC_CR_HSIDIV_Pos);
      }
      break;

    case LL_RCC_PLLSOURCE_CSI:
      if (LL_RCC_CSI_IsReady() != 0U)
      {
        pllinputfreq = CSI_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_HSE:
      if (LL_RCC_HSE_IsReady() != 0U)
      {
        pllinputfreq = HSE_VALUE;
      }
      break;

    case LL_RCC_PLLSOURCE_NONE:
    default:
      /* PLL clock disabled */
      break;
  }

  PLL_Clocks->PLL_P_Frequency = 0U;
  PLL_Clocks->PLL_Q_Frequency = 0U;
  PLL_Clocks->PLL_R_Frequency = 0U;

  m = LL_RCC_PLL3_GetM();
  n = LL_RCC_PLL3_GetN();
  if (LL_RCC_PLL3FRACN_IsEnabled() != 0U)
  {
    fracn = LL_RCC_PLL3_GetFRACN();
  }

  if ((m != 0U) && (pllinputfreq != 0U))
  {
    if (LL_RCC_PLL3P_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_P_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL3_GetP());
    }

    if (LL_RCC_PLL3Q_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_Q_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL3_GetQ());
    }

    if (LL_RCC_PLL3R_IsEnabled() != 0U)
    {
      PLL_Clocks->PLL_R_Frequency = LL_RCC_CalcPLLClockFreq(pllinputfreq, m, n, fracn, LL_RCC_PLL3_GetR());
    }
  }
}

#endif



/**
  * @brief  Return USARTx clock frequency
  * @param  USARTxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USART16_CLKSOURCE
  *         @arg @ref LL_RCC_USART234578_CLKSOURCE
  * @retval USART clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
uint32_t LL_RCC_GetUSARTClockFreq(__attribute__ ((unused)) uint32_t USARTxSource)
{
#if USE_PRECALC
  return 143229168;
#else
  uint32_t usart_frequency = LL_RCC_PERIPH_FREQUENCY_NO;
  LL_PLL_ClocksTypeDef PLL_Clocks;

  /* Check parameter */
  assert_param(IS_LL_RCC_USART_CLKSOURCE(USARTxSource));

  switch (LL_RCC_GetUSARTClockSource(USARTxSource))
  {
    case LL_RCC_USART16_CLKSOURCE_PCLK2:
      usart_frequency = RCC_GetPCLK2ClockFreq(RCC_GetHCLKClockFreq(LL_RCC_CALC_SYSCLK_FREQ(RCC_GetSystemClockFreq(), LL_RCC_GetSysPrescaler())));
      break;

    case LL_RCC_USART234578_CLKSOURCE_PCLK1:
      usart_frequency = RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(LL_RCC_CALC_SYSCLK_FREQ(RCC_GetSystemClockFreq(), LL_RCC_GetSysPrescaler())));
      break;

    case LL_RCC_USART16_CLKSOURCE_PLL2Q:
    case LL_RCC_USART234578_CLKSOURCE_PLL2Q:
      if (LL_RCC_PLL2_IsReady() != 0U)
      {
        LL_RCC_GetPLL2ClockFreq(&PLL_Clocks);
        usart_frequency = PLL_Clocks.PLL_Q_Frequency;
      }
      break;

    case LL_RCC_USART16_CLKSOURCE_PLL3Q:
    case LL_RCC_USART234578_CLKSOURCE_PLL3Q:
      if (LL_RCC_PLL3_IsReady() != 0U)
      {
        LL_RCC_GetPLL3ClockFreq(&PLL_Clocks);
        usart_frequency = PLL_Clocks.PLL_Q_Frequency;
      }
      break;

    case LL_RCC_USART16_CLKSOURCE_HSI:
    case LL_RCC_USART234578_CLKSOURCE_HSI:
      if (LL_RCC_HSI_IsReady() != 0U)
      {
        usart_frequency = HSI_VALUE >> (LL_RCC_HSI_GetDivider() >> RCC_CR_HSIDIV_Pos);
      }
      break;

    case LL_RCC_USART16_CLKSOURCE_CSI:
    case LL_RCC_USART234578_CLKSOURCE_CSI:
      if (LL_RCC_CSI_IsReady() != 0U)
      {
        usart_frequency = CSI_VALUE;
      }
      break;

    case LL_RCC_USART16_CLKSOURCE_LSE:
    case LL_RCC_USART234578_CLKSOURCE_LSE:
      if (LL_RCC_LSE_IsReady() != 0U)
      {
        usart_frequency = LSE_VALUE;
      }
      break;

    default:
      /* Kernel clock disabled */
      break;
  }

  return usart_frequency;
#endif
}

/**
  * @brief  Initialize USART registers according to the specified
  *         parameters in USART_InitStruct.
  * @note   As some bits in USART configuration registers can only be written when
  *         the USART is disabled (USART_CR1_UE bit =0), USART Peripheral should be in disabled state prior calling
  *         this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in USART_InitStruct BaudRate field, should be valid (different from 0).
  * @param  USARTx USART Instance
  * @param  USART_InitStruct pointer to a LL_USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers are initialized according to USART_InitStruct content
  *          - ERROR: Problem occurred during USART Registers initialization
  */
ErrorStatus LL_USART_Init(USART_TypeDef *USARTx, const LL_USART_InitTypeDef *USART_InitStruct)
{
#if USE_PRECALC
  ErrorStatus status = ERROR;
  uint32_t periphclk = LL_RCC_PERIPH_FREQUENCY_NO;

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CRx registers */
  if (LL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CR1 Configuration ---------------------
     * Configure USARTx CR1 (USART Word Length, Parity, Mode and Oversampling bits) with parameters:
     * - DataWidth:          USART_CR1_M bits according to USART_InitStruct->DataWidth value
     * - Parity:             USART_CR1_PCE, USART_CR1_PS bits according to USART_InitStruct->Parity value
     * - TransferDirection:  USART_CR1_TE, USART_CR1_RE bits according to USART_InitStruct->TransferDirection value
     * - Oversampling:       USART_CR1_OVER8 bit according to USART_InitStruct->OverSampling value.
     */
    MODIFY_REG(USARTx->CR1,
               (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS |
                USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8),
               (USART_InitStruct->DataWidth | USART_InitStruct->Parity |
                USART_InitStruct->TransferDirection | USART_InitStruct->OverSampling));

    /*---------------------------- USART CR2 Configuration ---------------------
     * Configure USARTx CR2 (Stop bits) with parameters:
     * - Stop Bits:          USART_CR2_STOP bits according to USART_InitStruct->StopBits value.
     * - CLKEN, CPOL, CPHA and LBCL bits are to be configured using LL_USART_ClockInit().
     */
    LL_USART_SetStopBitsLength(USARTx, USART_InitStruct->StopBits);

    /*---------------------------- USART CR3 Configuration ---------------------
     * Configure USARTx CR3 (Hardware Flow Control) with parameters:
     * - HardwareFlowControl: USART_CR3_RTSE, USART_CR3_CTSE bits according to
     *   USART_InitStruct->HardwareFlowControl value.
     */
    LL_USART_SetHWFlowCtrl(USARTx, USART_InitStruct->HardwareFlowControl);

    /*---------------------------- USART BRR Configuration ---------------------
     * Retrieve Clock frequency used for USART Peripheral
     */
    
    periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);

    /* Configure the USART Baud Rate :
       - prescaler value is required
       - valid baud rate value (different from 0) is required
       - Peripheral clock as returned by RCC service, should be valid (different from 0).
    */
    if ((periphclk != LL_RCC_PERIPH_FREQUENCY_NO)
        && (USART_InitStruct->BaudRate != 0U))
    {
      status = SUCCESS;
      LL_USART_SetBaudRate(USARTx,
                           periphclk,
                           USART_InitStruct->PrescalerValue,
                           USART_InitStruct->OverSampling,
                           USART_InitStruct->BaudRate);

      /* Check BRR is greater than or equal to 16d */
      assert_param(IS_LL_USART_BRR_MIN(USARTx->BRR));
    }

    /*---------------------------- USART PRESC Configuration -----------------------
     * Configure USARTx PRESC (Prescaler) with parameters:
     * - PrescalerValue: USART_PRESC_PRESCALER bits according to USART_InitStruct->PrescalerValue value.
     */
    LL_USART_SetPrescaler(USARTx, USART_InitStruct->PrescalerValue);
  }
  /* Endif (=> USART not in Disabled state => return ERROR) */

  return (status);
#else
  ErrorStatus status = ERROR;
  uint32_t periphclk = LL_RCC_PERIPH_FREQUENCY_NO;

  /* Check the parameters */
  assert_param(IS_UART_INSTANCE(USARTx));
  assert_param(IS_LL_USART_PRESCALER(USART_InitStruct->PrescalerValue));
  assert_param(IS_LL_USART_BAUDRATE(USART_InitStruct->BaudRate));
  assert_param(IS_LL_USART_DATAWIDTH(USART_InitStruct->DataWidth));
  assert_param(IS_LL_USART_STOPBITS(USART_InitStruct->StopBits));
  assert_param(IS_LL_USART_PARITY(USART_InitStruct->Parity));
  assert_param(IS_LL_USART_DIRECTION(USART_InitStruct->TransferDirection));
  assert_param(IS_LL_USART_HWCONTROL(USART_InitStruct->HardwareFlowControl));
  assert_param(IS_LL_USART_OVERSAMPLING(USART_InitStruct->OverSampling));

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CRx registers */
  if (LL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CR1 Configuration ---------------------
     * Configure USARTx CR1 (USART Word Length, Parity, Mode and Oversampling bits) with parameters:
     * - DataWidth:          USART_CR1_M bits according to USART_InitStruct->DataWidth value
     * - Parity:             USART_CR1_PCE, USART_CR1_PS bits according to USART_InitStruct->Parity value
     * - TransferDirection:  USART_CR1_TE, USART_CR1_RE bits according to USART_InitStruct->TransferDirection value
     * - Oversampling:       USART_CR1_OVER8 bit according to USART_InitStruct->OverSampling value.
     */
    MODIFY_REG(USARTx->CR1,
               (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS |
                USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8),
               (USART_InitStruct->DataWidth | USART_InitStruct->Parity |
                USART_InitStruct->TransferDirection | USART_InitStruct->OverSampling));

    /*---------------------------- USART CR2 Configuration ---------------------
     * Configure USARTx CR2 (Stop bits) with parameters:
     * - Stop Bits:          USART_CR2_STOP bits according to USART_InitStruct->StopBits value.
     * - CLKEN, CPOL, CPHA and LBCL bits are to be configured using LL_USART_ClockInit().
     */
    LL_USART_SetStopBitsLength(USARTx, USART_InitStruct->StopBits);

    /*---------------------------- USART CR3 Configuration ---------------------
     * Configure USARTx CR3 (Hardware Flow Control) with parameters:
     * - HardwareFlowControl: USART_CR3_RTSE, USART_CR3_CTSE bits according to
     *   USART_InitStruct->HardwareFlowControl value.
     */
    LL_USART_SetHWFlowCtrl(USARTx, USART_InitStruct->HardwareFlowControl);

    /*---------------------------- USART BRR Configuration ---------------------
     * Retrieve Clock frequency used for USART Peripheral
     */
    if (USARTx == USART1)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART16_CLKSOURCE);
    }
    else if (USARTx == USART2)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
    else if (USARTx == USART3)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
    else if (USARTx == UART4)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
    else if (USARTx == UART5)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
    else if (USARTx == USART6)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART16_CLKSOURCE);
    }
    else if (USARTx == UART7)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
    else if (USARTx == UART8)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART234578_CLKSOURCE);
    }
#if defined(UART9)
    else if (USARTx == UART9)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART16_CLKSOURCE);
    }
#endif /* UART9 */
#if defined(USART10)
    else if (USARTx == USART10)
    {
      periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART16_CLKSOURCE);
    }
#endif /* USART10 */
    else
    {
      /* Nothing to do, as error code is already assigned to ERROR value */
    }

    /* Configure the USART Baud Rate :
       - prescaler value is required
       - valid baud rate value (different from 0) is required
       - Peripheral clock as returned by RCC service, should be valid (different from 0).
    */
    if ((periphclk != LL_RCC_PERIPH_FREQUENCY_NO)
        && (USART_InitStruct->BaudRate != 0U))
    {
      status = SUCCESS;
      LL_USART_SetBaudRate(USARTx,
                           periphclk,
                           USART_InitStruct->PrescalerValue,
                           USART_InitStruct->OverSampling,
                           USART_InitStruct->BaudRate);

      /* Check BRR is greater than or equal to 16d */
      assert_param(IS_LL_USART_BRR_MIN(USARTx->BRR));
    }

    /*---------------------------- USART PRESC Configuration -----------------------
     * Configure USARTx PRESC (Prescaler) with parameters:
     * - PrescalerValue: USART_PRESC_PRESCALER bits according to USART_InitStruct->PrescalerValue value.
     */
    LL_USART_SetPrescaler(USARTx, USART_InitStruct->PrescalerValue);
  }
  /* Endif (=> USART not in Disabled state => return ERROR) */

  return (status);
#endif
}

/**
  * @brief  Initialize GPIO registers according to the specified parameters in GPIO_InitStruct.
  * @param  GPIOx GPIO Port
  * @param GPIO_InitStruct pointer to a @ref LL_GPIO_InitTypeDef structure
  *         that contains the configuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to GPIO_InitStruct content
  *          - ERROR:   Not applicable
  */
ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint32_t pinpos, currentpin;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_LL_GPIO_PIN(GPIO_InitStruct->Pin));
  assert_param(IS_LL_GPIO_MODE(GPIO_InitStruct->Mode));
  assert_param(IS_LL_GPIO_PULL(GPIO_InitStruct->Pull));

  /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */
  pinpos = POSITION_VAL(GPIO_InitStruct->Pin);

  /* Configure the port pins */
  while (((GPIO_InitStruct->Pin) >> pinpos) != 0x00000000U)
  {
    /* Get current io position */
    currentpin = (GPIO_InitStruct->Pin) & (0x00000001UL << pinpos);

    if (currentpin != 0x00000000U)
    {

      if ((GPIO_InitStruct->Mode == LL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE))
      {
        /* Check Speed mode parameters */
        assert_param(IS_LL_GPIO_SPEED(GPIO_InitStruct->Speed));

        /* Speed mode configuration */
        LL_GPIO_SetPinSpeed(GPIOx, currentpin, GPIO_InitStruct->Speed);

        /* Check Output mode parameters */
        assert_param(IS_LL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

        /* Output mode configuration*/
        LL_GPIO_SetPinOutputType(GPIOx, GPIO_InitStruct->Pin, GPIO_InitStruct->OutputType);

      }

      /* Pull-up Pull down resistor configuration*/
      LL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);

      if (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE)
      {
        /* Check Alternate parameter */
        assert_param(IS_LL_GPIO_ALTERNATE(GPIO_InitStruct->Alternate));

        /* Alternate function configuration */
        if (currentpin < LL_GPIO_PIN_8)
        {
          LL_GPIO_SetAFPin_0_7(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
        else
        {
          LL_GPIO_SetAFPin_8_15(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
      }

      /* Pin Mode configuration */
      LL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);
    }
    pinpos++;
  }

  return (SUCCESS);
}



void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
}

void DebugOut(const char *pStr)
{
  const uint8_t *p = (uint8_t *)pStr;
  while(*p)
  {
    while(!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, *p);
    p++;
  }
}

void DebugOutParams(const char *pStr, ...)
{
  

  va_list ap;
  va_start(ap, pStr);

  char buffer[sizeof(int)*8+1];
  const uint8_t *p = (uint8_t *)pStr;

  while(*p)
  {
    if(*p == '%')
    {
      p++;
      switch(*p)
      {
        case 'd' :
        {
          int n = va_arg(ap, int);
          itoa(n, buffer, 10);
          DebugOut(buffer);
        }
        break;

        case 'u' :
        {
          int n = va_arg(ap, int);
          utoa(n, buffer, 10);
          DebugOut(buffer);
        }
        break;

        case 'x' :
        {
          int n = va_arg(ap, int);
          utoa(n, buffer, 10);
          DebugOut(buffer);
        }
        break;

        case 'f'  :
        {
          double f = va_arg(ap, double);
#if CHECK_NAN_INF
          if(isnan(f))
            DebugOut("nan");
          else if(isinf(f))
            DebugOut("inf");
          else
#endif          
          {
            int nInt = (int) f;
            itoa(nInt, buffer, 10);
            DebugOut(buffer);
            DebugOut(".");
            
            unsigned int nFrac = (unsigned int) ((f - (float)nInt) * 1000.0f); 
            utoa(nFrac, buffer, 10);
            DebugOut(buffer);
          }
        }
        break;

        case 's' :
        {
          char *pBuffer = va_arg(ap, char*);
          DebugOut(pBuffer);
        }
      }
    }
    else
    {
      while(!LL_USART_IsActiveFlag_TXE(USART2))
        ;
      LL_USART_TransmitData8(USART2, *p);
    }

    p++;
  }
}
#else
  void DebugOut(__attribute__ ((unused)) const char *pStr)
  {
  }

  void DebugOutParams(__attribute__ ((unused)) const char *pStr, ...)
  {
  }

#endif
