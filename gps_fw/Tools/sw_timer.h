/*
 * sw_timer.h
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 *
 *  A software timer can be used to wait for timeouts or to execute
 *  tasks periodically.
 *
 *  USE
 *  ---
 *  SW_TIMER_MAX_NUMBER_TIMERS must be set to the number of timers needed
 *
 */

#ifndef SW_TIMER_H_
#define SW_TIMER_H_


/* Includes ------------------------------------------------------------------*/

#include "main.h"


/* Public defines ------------------------------------------------------------*/

#define SW_TIMER_MAX_NUMBER_TIMERS      (1) // this number must be set to
                                            // the number of timers needed

/* Public typedef ------------------------------------------------------------*/

typedef void (*sw_timer_callback_func_ptr_t)(void); // timer callback function

typedef enum {
    SINGLE,
    CONTINUOUS
} sw_timer_operation_mode_t;



#endif /* SW_TIMER_H_ */
