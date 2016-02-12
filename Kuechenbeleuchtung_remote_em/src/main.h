/*
 * main.h
 *
 *  Created on: 25.01.2015
 *      Author: Jacob
 */

#ifndef MAIN_H_
#define MAIN_H_
#include <stdbool.h>

enum return_constant {RC_OK, RC_FAILED, RC_TIMEOUT, RC_UNKNOWN} ;
typedef enum return_constant rc_t;

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
int main(void);
rc_t utrusta_set_light_off(void);
rc_t utrusta_set_light_low(void);
rc_t utrusta_set_light_full(void);
rc_t utrusta_send_programm(void);
void init_cc2500(void);
rc_t cc2500_xfer_tx(bool burst, uint8_t* data, uint8_t len);
rc_t cc2500_xfer_cmd(uint8_t cmd);
rc_t cc2500_xfer_reg(uint8_t cmd, uint8_t val);
#endif /* MAIN_H_ */
