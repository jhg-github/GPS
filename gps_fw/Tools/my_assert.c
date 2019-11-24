/*
 * my_assert.c
 *
 *  Created on: Nov 24, 2019
 *      Author: javi
 */


/* Includes ------------------------------------------------------------------*/

#include "my_assert.h"


/* Public functions ----------------------------------------------------------*/

/**
 * @brief This function is called when an assertion fails.
 *          It waits for ever until MCU resets via watchdog
 * @param file: file where the assertion failed
 * @param line: line where the assertion failed
 */
void my_assert_failed(char *file, int line){
    while(1);   // wait for watchdog reset
}
