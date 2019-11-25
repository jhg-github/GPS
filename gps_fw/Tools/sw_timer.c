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

static struct sw_timer_mod_t {               // sw timer module module structure
  volatile uint32_t time_ms;                   // variable to keep track of time
  sw_timer_t timers[SW_TIMER_MAX_NUMBER_TIMERS];  // array of all sw timers
  uint8_t timers_used;                            // number of timers in use
} sw_timer_mod;

/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes sw timer module
 *  Enables systick interrupt in order to get 1ms interrupt
 */
void sw_timer_init(void) {
  uint8_t i;

  // init time track
  sw_timer_mod.time_ms = 0;
  LL_SYSTICK_EnableIT();
  // init timers
  sw_timer_mod.timers_used = 0;
  for (i = 0; i < SW_TIMER_MAX_NUMBER_TIMERS; i++) {
    sw_timer_mod.timers[i].state = SW_TIMER_STATE_STOPPED;
  }
}

/**
 * @brief Checks all timers in use to check if their intervals has expired.
 *      If the timer runs in single mode, it will be stopped.
 *      If the timer runs in continuous mode, the start time will be updated
 *
 * Note: must be called continuously from main
 */
void sw_timer_process(void) {
  uint8_t i;

  for (i = 0; i < sw_timer_mod.timers_used; i++) {
    if (SW_TIMER_STATE_RUNNING == sw_timer_mod.timers[i].state) {
      if ((sw_timer_mod.time_ms - sw_timer_mod.timers[i].interval_start_time_ms) >= sw_timer_mod.timers[i].interval_ms) {
        if (SW_TIMER_MODE_SINGLE == sw_timer_mod.timers[i].mode) {
          sw_timer_mod.timers[i].state = SW_TIMER_STATE_STOPPED;
        } else {
          sw_timer_mod.timers[i].interval_start_time_ms = sw_timer_mod.time_ms; // this will accumulate delays
        }
        sw_timer_mod.timers[i].callback_func(); // callback was asserted not NULL on sw_timer_timer_start()
      }
    }
  }
}

/**
 * @brief Timer constructor gets one of the available timers and initializes it.
 *      The array sw_timer_t.timers[SW_TIMER_MAX_NUMBER_TIMERS] is used in order
 *      to avoid use dynamic memory allocation
 *
 * @return sw_timer_t *, pointer to a timer or NULL if no timers available
 */
sw_timer_t *sw_timer_timer_ctr(void) {
  if ( SW_TIMER_MAX_NUMBER_TIMERS <= sw_timer_mod.timers_used) {
    return NULL;
  }
  sw_timer_mod.timers_used++;
  return &sw_timer_mod.timers[sw_timer_mod.timers_used - 1];
}

/**
 * @brief Initializes the timer and sets it into running mode
 * @param p_timer:          timer self-pointer
 * @param interval_ms:      after this interval the callback function will be called [ms]
 * @param callback_func:    callback function
 * @param mode:             continuous or single
 */
void sw_timer_timer_start(sw_timer_t *p_timer, uint32_t interval_ms,
    sw_timer_callback_func_ptr_t callback_func, sw_timer_mode_t mode) {
  MY_ASSERT(NULL != callback_func);

  p_timer->interval_start_time_ms = sw_timer_mod.time_ms;
  p_timer->state = SW_TIMER_STATE_RUNNING;
  p_timer->interval_ms = interval_ms;
  p_timer->callback_func = callback_func;
  p_timer->mode = mode;
}

/* ISR -----------------------------------------------------------------------*/

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  sw_timer_mod.time_ms++;     // increment time in 1ms

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
