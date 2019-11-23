/*
 * lcd.c
 *
 *  Created on: Nov 23, 2019
 *      Author: javi
 *
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


/* Includes ------------------------------------------------------------------*/

#include "lcd_driver.h"
#include "spi.h"


/* Private defines -----------------------------------------------------------*/

#define LCD_DRIVER_TX_BUF_SIZE_BYTES    (20)    // size = address(1) + line(18) + trail(1)


/* Private variables ---------------------------------------------------------*/

static struct lcd_driver_t {                                // LCD driver module structure
//  uint8_t bitmap_array[LCD_DRIVER_BITMAP_ARRAY_SIZE_BYTES]; // LCD bitmap array
//  uint8_t vcom_bit;                                         // vcom bit must be toggled to keep LCD running
//  uint8_t lineAddress;                                      // line to write
  uint8_t tx_buf[LCD_DRIVER_TX_BUF_SIZE_BYTES];               // spi tx buffer
//  uint8_t isTxRunning_flag;                                 // tx running flag
} lcd_driver;


/* Private function prototypes -----------------------------------------------*/

static void init_spi_dma( void );


/* Public functions ----------------------------------------------------------*/

/**
 * Initializes lcd driver module
 */
void lcd_driver_init( void ){
    init_spi_dma();
}


/**
 * Executes the state machine in charge of refreshing the lcd
 * Note: must be called continuously from main
 */
void lcd_driver_process( void ){

}


/* Private functions ---------------------------------------------------------*/

/**
 * Initializes SPI and SPI's DMA
 */
static void init_spi_dma( void ){
  // chip select
  LL_GPIO_ResetOutputPin(LCD_CS_GPIO_Port, LCD_CS_Pin);

  /* Configure SPI1 DMA transfer interrupts */
  /* Enable DMA TX Interrupt */
  LL_SPI_EnableDMAReq_TX(SPI1);

  /* Enable DMA interrupts complete/error */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

  /* Configure the DMA1_Channel2_3 functional parameters */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_MEDIUM | LL_DMA_MODE_NORMAL |
                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                        LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)lcd_driver.tx_buf, LL_SPI_DMA_GetRegAddr(SPI1),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);

  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
}
