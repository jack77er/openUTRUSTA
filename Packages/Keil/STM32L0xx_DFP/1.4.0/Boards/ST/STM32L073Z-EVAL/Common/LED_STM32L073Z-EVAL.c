/*-----------------------------------------------------------------------------
 * Name:    LED_STM32L073Z-EVAL.c
 * Purpose: LED interface for STM32L073Z-EVAL evaluation board
 * Rev.:    1.00
 *----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include "stm32l0xx.h"                  // Device header
#include "Board_LED.h"                  // ::Board Support:LED

/* GPIO Pin identifier */
typedef struct _GPIO_PIN {
  GPIO_TypeDef *port;
  uint16_t      pin;
} GPIO_PIN;

/* LED GPIO Pins */
static const GPIO_PIN LED_PIN[] = {
  { GPIOE, 4 },
  { GPIOE, 5 },
  { GPIOD, 1 },
  { GPIOE, 7 }
};

#define LED_NUM (sizeof(LED_PIN)/sizeof(GPIO_PIN))


/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Initialize (void) {
  uint32_t i;

  RCC->IOPENR |=  ((1UL << 3) |               /* Enable GPIOD clock         */
                   (1UL << 4)  );             /* Enable GPIOE clock         */

  /* Configure LED pins as push-pull outputs, No pull-up, pull-down */
  for (i = 0; i < LED_NUM; i++) {
    LED_PIN[i].port->MODER   &= ~((3UL << 2*LED_PIN[i].pin));
    LED_PIN[i].port->MODER   |=  ((1UL << 2*LED_PIN[i].pin));
    LED_PIN[i].port->OTYPER  &= ~((1UL <<   LED_PIN[i].pin));
    LED_PIN[i].port->OSPEEDR &= ~((3UL << 2*LED_PIN[i].pin));
    LED_PIN[i].port->OSPEEDR |=  ((1UL << 2*LED_PIN[i].pin));
    LED_PIN[i].port->PUPDR   &= ~((3UL << 2*LED_PIN[i].pin));
  }

  LED_SetOut (0);
  return (0);
}


/**
  \fn          int32_t LED_Uninitialize (void)
  \brief       De-initialize I/O interface for LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Uninitialize (void) {

  return (0);
}


/**
  \fn          int32_t LED_On (uint32_t num)
  \brief       Turn on requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_On (uint32_t num) {

  if (num < LED_NUM) {
    LED_PIN[num].port->BSRR |= (1U << (LED_PIN[num].pin + 16));
  }
  return (0);
}


/**
  \fn          int32_t LED_Off (uint32_t num)
  \brief       Turn off requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Off (uint32_t num) {

  if (num < LED_NUM) {
    LED_PIN[num].port->BSRR |= (1U << (LED_PIN[num].pin     ));
  }
  return (0);
}


/**
  \fn          int32_t LED_SetOut (uint32_t val)
  \brief       Write value to LEDs
  \param[in]   val  value to be displayed on LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_SetOut (uint32_t val) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (val & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
  return (0);
}


/**
  \fn          uint32_t LED_GetCount (void)
  \brief       Get number of LEDs
  \return      Number of available LEDs
*/
uint32_t LED_GetCount (void) {
  return LED_NUM;
}
