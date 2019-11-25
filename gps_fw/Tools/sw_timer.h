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

typedef enum {
    SW_TIMER_STATE_STOPPED,
    SW_TIMER_STATE_RUNNING
} sw_timer_state_t;

typedef enum {
    SW_TIMER_MODE_SINGLE,
    SW_TIMER_MODE_CONTINUOUS
} sw_timer_mode_t;

typedef void (*sw_timer_callback_func_ptr_t)(void); // timer callback function

typedef struct{
    sw_timer_state_t state;                     // timer stopped or running
    uint32_t interval_ms;                       // after this interval the callback function will be called [ms]
    uint32_t interval_start_time_ms;            // timestamp of interval start [ms]
    sw_timer_callback_func_ptr_t callback_func; // this is called after the period has passed
    sw_timer_mode_t mode;                       // operation mode: single or continuous
} sw_timer_t;


/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes sw timer module
 *  Enables systick interrupt in order to get 1ms interrupt
 */
void sw_timer_init( void );

/**
 * @brief Checks all timers in use to check if their intervals has expired.
 *      If the timer runs in single mode, it will be stopped.
 *      If the timer runs in continuous mode, the start time will be updated
 *
 * Note: must be called continuously from main
 */
void sw_timer_process( void );

/**
 * @brief Timer constructor gets one of the available timers and initializes it.
 *      The array sw_timer_t.timers[SW_TIMER_MAX_NUMBER_TIMERS] is used in order
 *      to avoid use dynamic memory allocation
 *
 * @return sw_timer_t *, pointer to a timer or NULL if no timers available
 */
sw_timer_t *sw_timer_timer_ctr( void );

/**
 * @brief Initializes the timer and sets it into running mode
 * @param p_timer:          timer self-pointer
 * @param interval_ms:      after this interval the callback function will be called [ms]
 * @param callback_func:    callback function
 * @param mode:             continuous or single
 */
void sw_timer_timer_start( sw_timer_t *p_timer, uint32_t interval_ms, sw_timer_callback_func_ptr_t callback_func, sw_timer_mode_t mode );


#endif /* SW_TIMER_H_ */
