/*
 * libutrusta.h
 *
 *  Created on: 25.01.2015
 *      Author: Jacob
 */

#ifndef LIBUTRUSTA_H_
#define LIBUTRUSTA_H_

#define UTRUSTA_REMOTE_ID	0x319E

#define UTRUSTA_REPEAT_COUNTER 			1  		// repead each command N times
#define UTRUSTA_REMOTE_STATE_HIGH		0x03	// 100 %
#define UTRUSTA_REMOTE_STATE_MID		0x02	// 25 %
#define UTRUSTA_REMOTE_STATE_OFF		0x01	// 0 % - off
#define UTRUSTA_REMOTE_STATE_PROGRAM	0xFF	// program lightning to this id

#define UTRUSTA_REMOTE_START	0x5501			// start command
#define UTRUSTA_REMOTE_END		0xAAFF			// end command

#define UTRUSTA_INTER_FRAME_DELAY	300 		// wait 300 us between SPI commands
#define UTRUSTA_INTER_FRAME_DELAY_INIT	100		// wait 30 us between SPI commands

#define GLOBAL_LOOP_CNT_TRESHOLD	30			// number of repetitions of a single spi command send until the system gets reset
#endif /* LIBUTRUSTA_H_ */
