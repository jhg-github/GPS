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

typedef struct{
    uint32_t interval_ms;                       // after this interval the callback function will be called [ms]
    uint32_t interval_start_time_ms;            // timestamp of interval start [ms]
    sw_timer_callback_func_ptr_t callback_func; // this is called after the period has passed
    sw_timer_operation_mode_t mode;             // operation mode: single or continuous
} sw_timer_t;


/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes sw timer module
 *  Enables systick interrupt in order to get 1ms interrupt
 */
void sw_timer_init( void );

/**
 *  @brief Checks all timers in use to check if their intervals has expired
 *  Note: must be called continuously from main
 */
void sw_timer_process( void );

/**
 * @brief Timer constructor gets one of the available timers and initializes it.
 *
 * @param interval_ms,   timer interval
 * @param callback_func, callback function that will be called after interval_ms
 * @param mode,          single or continuous mode
 * @return sw_timer_t *, pointer to a timer or NULL if no timers available
 */
sw_timer_t *sw_timer_timer_ctr( uint32_t interval_ms, sw_timer_callback_func_ptr_t callback_func, sw_timer_operation_mode_t mode );


#endif /* SW_TIMER_H_ */
