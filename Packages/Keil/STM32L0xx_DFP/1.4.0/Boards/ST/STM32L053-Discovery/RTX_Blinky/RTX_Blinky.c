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
                             (HSE is not populated on Discovery board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  // PLL configuration: PLLCLK = (HSI * 6)/3 = 32 MHz
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
 * blinkLED: blink LED and check button state
 *----------------------------------------------------------------------------*/
void blinkLED(void const *argument) {
  int32_t max_num = LED_GetCount();
  int32_t num = 0;

  for (;;) {
    LED_On(num);                                           // Turn specified LED on
    osSignalWait(0x0001, osWaitForever);
    LED_Off(num);                                          // Turn specified LED off
    osSignalWait(0x0001, osWaitForever);

    num++;                                                 // Change LED number
    if (num >= max_num) {
      num = 0;                                             // Restart with first LED
    }
  }

}

osThreadId tid_blinkLED;
osThreadDef (blinkLED, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
 * main: initialize and start the system
 *----------------------------------------------------------------------------*/
int main (void) {
  uint32_t button_msk = (1U << Buttons_GetCount()) - 1;

  osKernelInitialize ();                                   // initialize CMSIS-RTOS

  // initialize peripherals
  SystemCoreClockConfigure();                              // configure System Clock
  SystemCoreClockUpdate();

  LED_Initialize();                                        // LED Initialization
  Buttons_Initialize();                                    // Buttons Initialization

  // create threads
  tid_blinkLED = osThreadCreate (osThread(blinkLED), NULL);

  osKernelStart ();                                        // start thread execution

  for (;;) {                                               // main must not be terminated!
    osDelay(500);
    while (Buttons_GetState() & (button_msk));             // Wait while holding USER button
    osSignalSet(tid_blinkLED, 0x0001);
  }
}
