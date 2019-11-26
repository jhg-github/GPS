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
#include "../../Tools/sw_timer.h"


/* Private defines -----------------------------------------------------------*/

#define LCD_DRIVER_TX_BUF_SIZE_BYTES    (20)    // size = address(1) + line(18) + trail(1)


/* Private typedef -----------------------------------------------------------*/

typedef enum {
  REFRESH_LCD_ST_IDLE,
  REFRESH_LCD_ST_SET_CS,
  REFRESH_LCD_ST_SEND_CMD_ENTRY,
  REFRESH_LCD_ST_SEND_CMD,
  REFRESH_LCD_ST_SEND_LINE_ENTRY,
  REFRESH_LCD_ST_SEND_LINE,
  REFRESH_LCD_ST_SEND_TRAILER,
} refresh_lcd_state_t;

typedef enum {
  TIMER_EXPIRED_NO,
  TIMER_EXPIRED_YES
} timer_expired_t;


typedef struct {                    // refresh lcd state machine
  refresh_lcd_state_t next_state;   // tate machine next state
  sw_timer_t *p_timer;              // state machine timer
  timer_expired_t is_timer_expired; // flag to check timer expired
} refresh_lcd_state_machine_t;


/* Private variables ---------------------------------------------------------*/

static struct lcd_driver_mod_t {                                // LCD driver module structure
//  uint8_t bitmap_array[LCD_DRIVER_BITMAP_ARRAY_SIZE_BYTES];   // LCD bitmap array
//  uint8_t vcom_bit;                                           // vcom bit must be toggled to keep LCD running
//  uint8_t lineAddress;                                        // line to write
//  uint8_t isTxRunning_flag;                                   // tx running flag

  uint8_t tx_buf[LCD_DRIVER_TX_BUF_SIZE_BYTES];                 // spi tx buffer
  refresh_lcd_state_machine_t refres_lcd_state_machine;         // refresh lcd state machine
} lcd_driver_mod;


/* Private function prototypes -----------------------------------------------*/

static void init_spi_dma( void );
static void init_refresh_lcd_state_machine( void );
static void refresh_lcd_state_machine( void );


/* Public functions ----------------------------------------------------------*/

/**
 *  @brief Initializes lcd driver module:
 *        - DMA
 *        - State machine
 */
void lcd_driver_init( void ){
  init_spi_dma();
  init_refresh_lcd_state_machine();
}


/**
 *  @brief Executes the state machine in charge of refreshing the lcd
 *  Note: must be called continuously from main
 */
void lcd_driver_process( void ){
  refresh_lcd_state_machine();
}


/* Private functions ---------------------------------------------------------*/

/**
 *  @brief Initializes SPI and SPI's DMA
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
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)lcd_driver_mod.tx_buf, LL_SPI_DMA_GetRegAddr(SPI1),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);

  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
}


static void init_refresh_lcd_state_machine( void ){
  lcd_driver_mod.refres_lcd_state_machine.next_state = REFRESH_LCD_ST_IDLE;
  lcd_driver_mod.refres_lcd_state_machine.p_timer = sw_timer_timer_ctr();
  MY_ASSERT (NULL != lcd_driver_mod.refres_lcd_state_machine.p_timer);
  sw_timer_timer_start( lcd_driver_mod.refres_lcd_state_machine.p_timer, 10, timer1_callback, SW_TIMER_MODE_CONTINUOUS );
}

static void refresh_lcd_state_machine( void ){
  switch( lcd_driver_mod.refresh_lcd_next_state ){
  case REFRESH_LCD_ST_IDLE:
    // output
    LL_GPIO_ResetOutputPin(LCD_CS_GPIO_Port, LCD_CS_Pin);   // CS deassert

    // next state
    break;
  }
}
