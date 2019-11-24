/*
 * my_assert.h
 *
 *  Created on: Nov 24, 2019
 *      Author: javi
 */

#ifndef MY_ASSERT_H_
#define MY_ASSERT_H_


/**
 * @brief Macro that calls assert failed function
 * @param file
 * @param line
 */
#define MY_ASSERT(expr) \
    if (!(expr)) \
        my_assert_failed(__FILE__, __LINE__)


/* Public functions ----------------------------------------------------------*/

/**
 * @brief This function is called when an assertion fails.
 *          It waits for ever until MCU resets via watchdog
 * @param file: file where the assertion failed
 * @param line: line where the assertion failed
 */
void my_assert_failed(char *file, int line);


#endif /* MY_ASSERT_H_ */
