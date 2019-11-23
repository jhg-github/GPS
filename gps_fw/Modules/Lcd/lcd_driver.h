/*
 * lcd_driver.h
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 */

#ifndef LCD_LCD_DRIVER_H_
#define LCD_LCD_DRIVER_H_




/* Public functions ----------------------------------------------------------*/

/**
 * Initializes lcd driver module
 */
extern void lcd_driver_init( void );

/**
 * Executes the state machine in charge of refreshing the lcd
 * Note: must be called continuously from main
 */
extern void lcd_driver_process( void );

#endif /* LCD_LCD_DRIVER_H_ */
