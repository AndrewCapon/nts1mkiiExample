/*
    BSD 3-Clause License

    Copyright (c) 2023, KORG INC.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//*/

/*
 *  File: unit.cc
 *
 *  NTS-1 mkII oscillator unit interface
 *
 */
#include <stdarg.h>
#include <stdlib.h>

#include "unit_osc.h"                       // Note: Include base definitions for osc units
#include "unit.h"
#include "osc.h"                            // Note: Include custom osc code

static Osc s_osc_instance;                  // Note: In this example, actual instance of custom osc object.

#if USE_USART
  extern "C" 
  {
    #define STM32H725xx
    #include "stm32h725xx.h"
    #include "core_cm7.h"

    #include "stm32h7xx_ll_rcc.h"
    #include "stm32h7xx_ll_crs.h"
    #include "stm32h7xx_ll_bus.h"
    #include "stm32h7xx_ll_system.h"
    #include "stm32h7xx_ll_exti.h"
    #include "stm32h7xx_ll_cortex.h"
    #include "stm32h7xx_ll_utils.h"
    #include "stm32h7xx_ll_pwr.h"
    #include "stm32h7xx_ll_dma.h"
    #include "stm32h7xx_ll_usart.h"
    #include "stm32h7xx_ll_gpio.h"

    extern uint8_t * itoa(int n, char *buffer, int radix);
    extern uint8_t * utoa(unsigned int n, char *buffer, int radix);
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

            if(isnan(f))
              DebugOut("nan");
            else if(isinf(f))
              DebugOut("inf");
            else
            {
              int nInt = (int) f;
              itoa(nInt, buffer, 10);
              DebugOut(buffer);
              DebugOut(".");
              unsigned int nFrac =  (int) ((f - (float)nInt) * 1000); 
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
  void DebugOut(const char *pStr)
  {
  }

  void DebugOutParams(const char *pStr, ...)
  {
  }

#endif

// ---- Callbacks exposed to runtime ----------------------------------------------

__unit_callback int8_t unit_init(const unit_runtime_desc_t * desc) {
  #if USE_USART
      MX_USART2_UART_Init();
  #endif

  return s_osc_instance.Init(desc);
}

__unit_callback void unit_teardown() {
  s_osc_instance.Teardown();
}

__unit_callback void unit_reset() {
  s_osc_instance.Reset();
}

__unit_callback void unit_resume() {
  s_osc_instance.Resume();
}

__unit_callback void unit_suspend() {
  s_osc_instance.Suspend();
}

__unit_callback void unit_render(const float * in, float * out, uint32_t frames) {
  s_osc_instance.Process(in, out, frames);
}

__unit_callback void unit_set_param_value(uint8_t id, int32_t value) {
  s_osc_instance.setParameter(id, value);
}

__unit_callback int32_t unit_get_param_value(uint8_t id) {
  return s_osc_instance.getParameterValue(id);
}

__unit_callback const char * unit_get_param_str_value(uint8_t id, int32_t value) {
  return s_osc_instance.getParameterStrValue(id, value);
}

__unit_callback void unit_set_tempo(uint32_t tempo) {
  s_osc_instance.setTempo(tempo);
}

__unit_callback void unit_tempo_4ppqn_tick(uint32_t counter) {
  s_osc_instance.tempo4ppqnTick(counter);
}

__unit_callback void unit_note_on(uint8_t note, uint8_t velo) {
  s_osc_instance.NoteOn(note, velo);
}

__unit_callback void unit_note_off(uint8_t note) {
  s_osc_instance.NoteOff(note);
}

__unit_callback void unit_all_note_off() {
  s_osc_instance.AllNoteOff();
}

__unit_callback void unit_pitch_bend(uint16_t bend) {
  s_osc_instance.PitchBend(bend);
}

__unit_callback void unit_channel_pressure(uint8_t press) {
  s_osc_instance.ChannelPressure(press);
}

__unit_callback void unit_aftertouch(uint8_t note, uint8_t press) {
  s_osc_instance.AfterTouch(note, press);
}
