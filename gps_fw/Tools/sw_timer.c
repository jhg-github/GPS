/*
 * sw_timer.c
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 *
 *  A software timer can be used to wait for timeouts or to execute
 *  tasks periodically.
 *
 */


/* Includes ------------------------------------------------------------------*/

#include "sw_timer.h"


/* Private typedef -----------------------------------------------------------*/

typedef struct{
    uint32_t interval_ms;                       // after this interval the callback function will be called [ms]
    uint32_t interval_start_time_ms;            // timestamp of interval start [ms]
    sw_timer_callback_func_ptr_t callback_func; // this is called after the period has passed
    sw_timer_operation_mode_t mode;             // operation mode: single or continuous
} sw_timer_t;


/* Private variables ---------------------------------------------------------*/

static struct sw_timer_mod_t {                      // sw timer module module structure
    sw_timer_t timers[SW_TIMER_MAX_NUMBER_TIMERS];  // array of all sw timers
    uint8_t timers_used;                            // number of timers in use
} sw_timer_mod;


/* Public functions ----------------------------------------------------------*/

/**
 * Initializes sw timer module
 */
void sw_timer_init( void ){
    sw_timer_mod.timers_used = 0;
}

/**
 * Checks all timers in use to check if their intervals has expired
 * Note: must be called continuously from main
 */
void sw_timer_process( void ){
    uint32_t time_now_ms =
}
