/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2015 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/


#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX

#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons

#include "stm32l0xx.h"                  // Device header

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSE is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  // PLL configuration: PLLCLK = (HSI * 4)/2 = 32 MHz
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC |
                 RCC_CFGR_PLLMUL |
                 RCC_CFGR_PLLDIV  );
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI |
                 RCC_CFGR_PLLMUL4    |
                 RCC_CFGR_PLLDIV2     );

  FLASH->ACR |= FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_LATENCY;                         // Flash 1 wait state

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;                       // Enable the PWR APB1 Clock
  PWR->CR = PWR_CR_VOS_0;                                  // Select the Voltage Range 1 (1.8V)
  while((PWR->CSR & PWR_CSR_VOSF) != 0);                   // Wait for Voltage Regulator Ready

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // PCLK1 = HCLK
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // PCLK2 = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_LED': Sample thread
 *---------------------------------------------------------------------------*/

void Thread_LED (void const *argument);                             // thread function
osThreadId tid_Thread_LED;                                          // thread id
osThreadDef (Thread_LED, osPriorityNormal, 1, 0);                   // thread object

int Init_Thread_LED (void) {

  tid_Thread_LED = osThreadCreate (osThread(Thread_LED), NULL);
  if(!tid_Thread_LED) return(-1);

  return(0);
}

void Thread_LED (void const *argument) {
  uint32_t button_msk = (1U << Buttons_GetCount()) - 1;
  uint32_t led_max    = LED_GetCount();
  uint32_t led_num    = 0;


  while (1) {
    LED_On(led_num);                                                // Turn specified LED on
    osSignalWait(0x0001, osWaitForever);
    LED_Off(led_num);                                               // Turn specified LED off
    osSignalWait(0x0001, osWaitForever);

    led_num++;                                                      // Change LED number
    if (led_num >= led_max) {
      led_num = 0;                                                  // Restart with first LED
    }

    osThreadYield();                                                // suspend thread
  }
}



/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
int main (void) {
  uint32_t button_msk = (1U << Buttons_GetCount()) - 1;

  osKernelInitialize ();                                            // initialize CMSIS-RTOS

  SystemCoreClockConfigure();                                       // configure System Clock
  SystemCoreClockUpdate();

  LED_Initialize();                                                 // initalize LEDs
  Buttons_Initialize();                                             // initalize Buttons

  Init_Thread_LED();                                                // create LED thread

  osKernelStart ();                                                 // start thread execution

  for (;;) {                                                        // main must not be terminated!
    osDelay(500);
    while (Buttons_GetState() & (button_msk));                      // Wait while holding USER button
    osSignalSet(tid_Thread_LED, 0x0001);
  }
}
