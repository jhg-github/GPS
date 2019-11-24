/*
 * sw_timer.c
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 *
 *  A software timer can be used to wait for timeouts or to execute
 *  tasks periodically.
 *
 *  The array sw_timer_t timers[SW_TIMER_MAX_NUMBER_TIMERS] is used in order
 *  to avoid use dynamic memory allocation
 *
 */


/* Includes ------------------------------------------------------------------*/

#include "sw_timer.h"
#include "../Tools/my_assert.h"


/* Private variables ---------------------------------------------------------*/

static struct sw_timer_mod_t {                      // sw timer module module structure
    uint32_t time_ms;                               // variable to keep track of time
    sw_timer_t timers[SW_TIMER_MAX_NUMBER_TIMERS];  // array of all sw timers
    uint8_t timers_used;                            // number of timers in use
} sw_timer_mod;


/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes sw timer module
 *  Enables systick interrupt in order to get 1ms interrupt
 */
void sw_timer_init( void ){
    // init time track
    sw_timer_mod.time_ms = 0;
    LL_SYSTICK_EnableIT();
    // init timers
    sw_timer_mod.timers_used = 0;
}

/**
 * @brief Checks all timers in use to check if their intervals has expired
 * Note: must be called continuously from main
 */
void sw_timer_process( void ){
}

/**
 * @brief Timer constructor gets one of the available timers and initializes it.
 *
 * @param interval_ms,   timer interval
 * @param callback_func, callback function that will be called after interval_ms
 * @param mode,          single or continuous mode
 * @return sw_timer_t *, pointer to a timer or NULL if no timers available
 */
sw_timer_t *sw_timer_timer_ctr( uint32_t interval_ms, sw_timer_callback_func_ptr_t callback_func, sw_timer_operation_mode_t mode ){
    MY_ASSERT (NULL != callback_func);

    if( SW_TIMER_MAX_NUMBER_TIMERS <= sw_timer_mod.timers_used ){
        return NULL;
    }
    sw_timer_mod.timers_used++;
    sw_timer_mod.timers[sw_timer_mod.timers_used-1].interval_start_time_ms = sw_timer_mod.time_ms;
    sw_timer_mod.timers[sw_timer_mod.timers_used-1].interval_ms = interval_ms;
    sw_timer_mod.timers[sw_timer_mod.timers_used-1].callback_func = callback_func;
    sw_timer_mod.timers[sw_timer_mod.timers_used-1].mode = mode;
    return &sw_timer_mod.timers[sw_timer_mod.timers_used-1];
}


/* ISR -----------------------------------------------------------------------*/

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

    sw_timer_mod.time_ms++;

    /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
