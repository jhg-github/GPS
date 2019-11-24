/*
 * lcd_driver.h
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 *
 *  LCD low level driver. This module interacts directly with the HW to control the LCD.
 *
 *  Main functions:
 *  - Refresh LCD. State machine that has to be called continuously
 *  - Clear LCD
 *  - Write text
 *  - Draw points
 *
 */

#ifndef LCD_LCD_DRIVER_H_
#define LCD_LCD_DRIVER_H_




/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes lcd driver module
 */
void lcd_driver_init( void );

/**
 *  @brief Executes the state machine in charge of refreshing the lcd
 *  Note: must be called continuously from main
 */
void lcd_driver_process( void );

#endif /* LCD_LCD_DRIVER_H_ */
