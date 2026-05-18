#ifndef _DISPLAY_H_
#define _DISPLAY_H_

// Constants
#define SSD1306_ADDR 0x3C

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

#define DISPLAY_PAGES (DISPLAY_HEIGHT / 8)

#define FRAMEBUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_PAGES)

// Command constants
#define CMD_SET_CONTRAST 0x81
#define CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define CMD_DISPLAY_ALL_ON 0xA5
#define CMD_NORMAL_DISPLAY 0xA6
#define CMD_INVERT_DISPLAY 0xA7
#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_ON 0xAF
#define CMD_SET_DISPLAY_OFFSET 0xD3
#define CMD_SET_COM_PINS 0xDA
#define CMD_SET_VCOM_DETECT 0xDB
#define CMD_SET_DISPLAY_CLOCK_DIV 0xD5
#define CMD_SET_PRECHARGE 0xD9
#define CMD_SET_MULTIPLEX 0xA8
#define CMD_SET_LOW_COLUMN 0x00
#define CMD_SET_HIGH_COLUMN 0x10
#define CMD_SET_START_LINE 0x40
#define CMD_MEMORY_MODE 0x20
#define CMD_COLUMN_ADDR 0x21
#define CMD_PAGE_ADDR 0x22
#define CMD_COM_SCAN_INC 0xC0
#define CMD_COM_SCAN_DEC 0xC8
#define CMD_SEG_REMAP 0xA0
#define CMD_CHARGE_PUMP 0x8D
#define CMD_DEACTIVATE_SCROLL 0x2E

#define CONTROL_CMD_SINGLE 0x80
#define CONTROL_CMD_STREAM 0x00
#define CONTROL_DATA_STREAM 0x40

// Error enum
typedef enum 
{
  DISPLAY_ERROR_OK = 0,
  DISPLAY_ERROR_I2C,
  DISPLAY_ERROR_INVALID_COORDINATES,
  DISPLAY_ERROR_BUFFER_OVERFLOW
} DisplayError;

// I2C interface - function pointer abstraction for embedded_hal::i2c::I2c trait
typedef struct 
{
  void* hw_handle;  // Hardware-specific I2C handle (e.g., I2C_HandleTypeDef* for STM32)
  DisplayError (*write)(void* hw_handle, uint8_t addr, const uint8_t* data, size_t len);
} I2cInterface;

// Display structure
typedef struct 
{
  I2cInterface i2c;
  uint8_t framebuffer[FRAMEBUFFER_SIZE];
  bool power_on;
  bool inverted;
  uint8_t contrast;
} Display;

// StatusContent structure
typedef struct 
{
  const char* protocol;
  uint32_t node_id;
  uint8_t battery_pct;
  uint32_t rx_count;
  uint32_t tx_count;
  int16_t rssi;
  bool connected;
} StatusContent;

// StatusDisplay structure
typedef struct 
{
  Display display;
} StatusDisplay;



#endif
