#include "display.h"


// 5x7 Font data
static const uint8_t FONT_5X7[320] = {
    // Space
    0x00, 0x00, 0x00, 0x00, 0x00,
    // !
    0x00, 0x00, 0x5F, 0x00, 0x00,
    // "
    0x00, 0x07, 0x00, 0x07, 0x00,
    // #
    0x14, 0x7F, 0x14, 0x7F, 0x14,
    // $
    0x24, 0x2A, 0x7F, 0x2A, 0x12,
    // %
    0x23, 0x13, 0x08, 0x64, 0x62,
    // &
    0x36, 0x49, 0x55, 0x22, 0x50,
    // '
    0x00, 0x05, 0x03, 0x00, 0x00,
    // (
    0x00, 0x1C, 0x22, 0x41, 0x00,
    // )
    0x00, 0x41, 0x22, 0x1C, 0x00,
    // *
    0x08, 0x2A, 0x1C, 0x2A, 0x08,
    // +
    0x08, 0x08, 0x3E, 0x08, 0x08,
    // ,
    0x00, 0x50, 0x30, 0x00, 0x00,
    // -
    0x08, 0x08, 0x08, 0x08, 0x08,
    // .
    0x00, 0x60, 0x60, 0x00, 0x00,
    // /
    0x20, 0x10, 0x08, 0x04, 0x02,
    // 0
    0x3E, 0x51, 0x49, 0x45, 0x3E,
    // 1
    0x00, 0x42, 0x7F, 0x40, 0x00,
    // 2
    0x42, 0x61, 0x51, 0x49, 0x46,
    // 3
    0x21, 0x41, 0x45, 0x4B, 0x31,
    // 4
    0x18, 0x14, 0x12, 0x7F, 0x10,
    // 5
    0x27, 0x45, 0x45, 0x45, 0x39,
    // 6
    0x3C, 0x4A, 0x49, 0x49, 0x30,
    // 7
    0x01, 0x71, 0x09, 0x05, 0x03,
    // 8
    0x36, 0x49, 0x49, 0x49, 0x36,
    // 9
    0x06, 0x49, 0x49, 0x29, 0x1E,
    // :
    0x00, 0x36, 0x36, 0x00, 0x00,
    // ;
    0x00, 0x56, 0x36, 0x00, 0x00,
    // <
    0x00, 0x08, 0x14, 0x22, 0x41,
    // =
    0x14, 0x14, 0x14, 0x14, 0x14,
    // >
    0x41, 0x22, 0x14, 0x08, 0x00,
    // ?
    0x02, 0x01, 0x51, 0x09, 0x06,
    // @
    0x32, 0x49, 0x79, 0x41, 0x3E,
    // A
    0x7E, 0x11, 0x11, 0x11, 0x7E,
    // B
    0x7F, 0x49, 0x49, 0x49, 0x36,
    // C
    0x3E, 0x41, 0x41, 0x41, 0x22,
    // D
    0x7F, 0x41, 0x41, 0x22, 0x1C,
    // E
    0x7F, 0x49, 0x49, 0x49, 0x41,
    // F
    0x7F, 0x09, 0x09, 0x01, 0x01,
    // G
    0x3E, 0x41, 0x41, 0x51, 0x32,
    // H
    0x7F, 0x08, 0x08, 0x08, 0x7F,
    // I
    0x00, 0x41, 0x7F, 0x41, 0x00,
    // J
    0x20, 0x40, 0x41, 0x3F, 0x01,
    // K
    0x7F, 0x08, 0x14, 0x22, 0x41,
    // L
    0x7F, 0x40, 0x40, 0x40, 0x40,
    // M
    0x7F, 0x02, 0x04, 0x02, 0x7F,
    // N
    0x7F, 0x04, 0x08, 0x10, 0x7F,
    // O
    0x3E, 0x41, 0x41, 0x41, 0x3E,
    // P
    0x7F, 0x09, 0x09, 0x09, 0x06,
    // Q
    0x3E, 0x41, 0x51, 0x21, 0x5E,
    // R
    0x7F, 0x09, 0x19, 0x29, 0x46,
    // S
    0x46, 0x49, 0x49, 0x49, 0x31,
    // T
    0x01, 0x01, 0x7F, 0x01, 0x01,
    // U
    0x3F, 0x40, 0x40, 0x40, 0x3F,
    // V
    0x1F, 0x20, 0x40, 0x20, 0x1F,
    // W
    0x7F, 0x20, 0x18, 0x20, 0x7F,
    // X
    0x63, 0x14, 0x08, 0x14, 0x63,
    // Y
    0x03, 0x04, 0x78, 0x04, 0x03,
    // Z
    0x61, 0x51, 0x49, 0x45, 0x43,
    // [
    0x00, 0x00, 0x7F, 0x41, 0x41,
    // backslash
    0x02, 0x04, 0x08, 0x10, 0x20,
    // ]
    0x41, 0x41, 0x7F, 0x00, 0x00,
    // ^
    0x04, 0x02, 0x01, 0x02, 0x04,
    // _
    0x40, 0x40, 0x40, 0x40, 0x40,
};

// Moon phase bitmaps (24x24, 72 bytes each)
static const uint8_t MOON_PHASES[8][72] = {
    // New moon
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0x38,0x1C,0x0C,0x06,0x06,
        0x06,0x06,0x0C,0x1C,0x38,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xE0,0x80,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1C,0x38,0x30,0x60,0x60,
        0x60,0x60,0x30,0x38,0x1C,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Waxing crescent
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0x38,0x1C,0x0C,0x06,0x06,
        0x06,0x06,0x0C,0x1C,0x38,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,0xE0,
        0xC0,0x80,0x00,0x00,0x00,0x80,0xE0,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1C,0x38,0x31,0x63,0x63,
        0x63,0x63,0x31,0x38,0x1C,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // First quarter
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0x38,0x1C,0x0C,0x06,0x06,
        0xFE,0xFE,0xFC,0xFC,0xF8,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,0xE0,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1C,0x38,0x30,0x60,0x60,
        0x7F,0x7F,0x3F,0x3F,0x1F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Waxing gibbous
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFE,
        0xFE,0xFE,0xFC,0xFC,0xF8,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,0x7F,
        0x7F,0x7F,0x3F,0x3F,0x1F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Full moon
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFE,
        0xFE,0xFE,0xFC,0xFC,0xF8,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,0x7F,
        0x7F,0x7F,0x3F,0x3F,0x1F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Waning gibbous
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFE,
        0xFE,0xFE,0xFC,0xFC,0xF8,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,0x7F,
        0x7F,0x7F,0x3F,0x3F,0x1F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Last quarter
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFE,
        0x06,0x06,0x0C,0x1C,0x38,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,0x7F,
        0x60,0x60,0x30,0x38,0x1C,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
    // Waning crescent
    {
        0x00,0x00,0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFE,
        0x06,0x06,0x0C,0x1C,0x38,0xF0,0xE0,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0x7F,0xFF,0xFF,0x07,0x03,0x01,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0x7F,0x1F,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x3F,0x3F,0x7F,0x7F,
        0x60,0x60,0x30,0x38,0x1C,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,
    },
};

// YOURS face logo (32x32, 128 bytes)
static const uint8_t YOURS_FACE[128] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF0,0xF8,0xF8,0xF8,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xF8,0xF8,0xF0,0xF0,0xE0,0xE0,0xE0,0xC0,0x80,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0xFF,0xFF,0xFF,0xFF,0xFF,0xEF,0xE7,0xE7,0xE7,0xC7,0xC7,0xC7,0xC7,0xC7,0xFF,0xFF,0xFF,0xFF,0x0F,0x07,0x03,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x0F,0x7F,0xFF,0xFD,0xF9,0xC1,0xC1,0xDE,0xFE,0xF0,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0F,0x1F,0x1F,0x1D,0x1D,0x0F,0x0F,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};



// Forward declarations
static DisplayError display_write_command(Display* disp, uint8_t cmd);
static DisplayError display_write_commands(Display* disp, const uint8_t* cmds, size_t count);

// Helper string formatting functions
static void format_battery(uint8_t pct, char* buf, size_t buf_size) 
{
  if (buf_size < 5) return;  // Need at least 4 chars + null
  size_t idx = 0;
  if (pct >= 100) 
  {
    buf[idx++] = '1';
    buf[idx++] = '0';
    buf[idx++] = '0';
  } 
  else if (pct >= 10) 
  {
    buf[idx++] = '0' + pct / 10;
    buf[idx++] = '0' + pct % 10;
  } 
  else buf[idx++] = '0' + pct;
  buf[idx++] = '%';
  buf[idx] = '\0';
}

static void format_hex32(uint32_t val, char* buf, size_t buf_size) 
{
  if (buf_size < 9) return;  // Need 8 chars + null
  const char HEX[] = "0123456789ABCDEF";
  for (int i = 7; i >= 0; i--) 
  {
    uint8_t nibble = (val >> (i * 4)) & 0xF;
    buf[7 - i] = HEX[nibble];
  }
  buf[8] = '\0';
}

static void format_u32(uint32_t val, char* buf, size_t buf_size) 
{
  if (buf_size < 11) return;  // Need up to 10 digits + null
  if (val == 0) 
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  uint8_t digits[10];
  size_t count = 0;
  uint32_t v = val;
  while (v > 0) 
  {
    digits[count] = v % 10;
    v /= 10;
    count++;
  }
  for (size_t i = 0; i < count; i++) buf[i] = '0' + digits[count - 1 - i];
  buf[count] = '\0';
}

static void format_i16(int16_t val, char* buf, size_t buf_size) 
{
  if (buf_size < 7) return;  // Need up to 6 chars + null
  size_t idx = 0;
  if (val < 0) 
  {
    buf[idx++] = '-';
    uint32_t abs_val = (uint32_t)(-(int32_t)val);
    char temp[11];
    format_u32(abs_val, temp, sizeof(temp));
    size_t temp_len = strlen(temp);
    if (idx + temp_len < buf_size) strcpy(&buf[idx], temp);
  } 
  else format_u32((uint32_t)val, buf, buf_size);
}

// Display implementation

void display_init_struct(Display* disp, I2cInterface i2c) 
{
  disp->i2c = i2c;
  memset(disp->framebuffer, 0, FRAMEBUFFER_SIZE);
  disp->power_on = false;
  disp->inverted = false;
  disp->contrast = 0x7F;
}

Display display_new(I2cInterface i2c) 
{
  Display disp;
  display_init_struct(&disp, i2c);
  return disp;
}

DisplayError display_init(Display* disp) 
{
  // Initialization command sequence
  const uint8_t init_cmds[] = 
  {
    CMD_DISPLAY_OFF,
    CMD_SET_DISPLAY_CLOCK_DIV, 0x80,
    CMD_SET_MULTIPLEX, 0x3F,
    CMD_SET_DISPLAY_OFFSET, 0x00,
    CMD_SET_START_LINE | 0x00,
    CMD_CHARGE_PUMP, 0x14,
    CMD_MEMORY_MODE, 0x00,
    CMD_SEG_REMAP | 0x01,
    CMD_COM_SCAN_DEC,
    CMD_SET_COM_PINS, 0x12,
    CMD_SET_CONTRAST, disp->contrast,
    CMD_SET_PRECHARGE, 0xF1,
    CMD_SET_VCOM_DETECT, 0x40,
    CMD_DISPLAY_ALL_ON_RESUME,
    CMD_NORMAL_DISPLAY,
    CMD_DEACTIVATE_SCROLL,
    CMD_DISPLAY_ON,
  };
  for (size_t i = 0; i < sizeof(init_cmds); i++) 
  {
    DisplayError err = display_write_command(disp, init_cmds[i]);
    if (err != DISPLAY_ERROR_OK) return err;
  }
  disp->power_on = true;
  display_clear(disp);
  DisplayError err = display_flush(disp);
  if (err != DISPLAY_ERROR_OK) return err;
  return DISPLAY_ERROR_OK;
}

static DisplayError display_write_command(Display* disp, uint8_t cmd) 
{
  uint8_t buf[2] = {CONTROL_CMD_SINGLE, cmd};
  return disp->i2c.write(disp->i2c.hw_handle, SSD1306_ADDR, buf, 2);
}

static DisplayError display_write_commands(Display* disp, const uint8_t* cmds, size_t count) 
{
  for (size_t i = 0; i < count; i++) 
  {
    DisplayError err = display_write_command(disp, cmds[i]);
    if (err != DISPLAY_ERROR_OK) return err;
  }
  return DISPLAY_ERROR_OK;
}

DisplayError display_flush(Display* disp) 
{
  // Set column address range
  const uint8_t col_cmds[] = {CMD_COLUMN_ADDR, 0, (DISPLAY_WIDTH - 1)};
  DisplayError err = display_write_commands(disp, col_cmds, 3);
  if (err != DISPLAY_ERROR_OK) return err;
  // Set page address range
  const uint8_t page_cmds[] = {CMD_PAGE_ADDR, 0, (DISPLAY_PAGES - 1)};
  err = display_write_commands(disp, page_cmds, 3);
  if (err != DISPLAY_ERROR_OK) return err;
  // Send framebuffer data in chunks
  const size_t CHUNK_SIZE = 128;
  uint8_t buf[CHUNK_SIZE + 1];
  for (size_t offset = 0; offset < FRAMEBUFFER_SIZE; offset += CHUNK_SIZE) 
  {
    size_t chunk_len = CHUNK_SIZE;
    if (offset + chunk_len > FRAMEBUFFER_SIZE) chunk_len = FRAMEBUFFER_SIZE - offset;
    buf[0] = CONTROL_DATA_STREAM;
    memcpy(&buf[1], &disp->framebuffer[offset], chunk_len);
    err = disp->i2c.write(disp->i2c.hw_handle, SSD1306_ADDR, buf, 1 + chunk_len);
    if (err != DISPLAY_ERROR_OK) return err;
  }
  return DISPLAY_ERROR_OK;
}

void display_clear(Display* disp) 
{
  memset(disp->framebuffer, 0, FRAMEBUFFER_SIZE);
}

void display_fill(Display* disp) 
{
  memset(disp->framebuffer, 0xFF, FRAMEBUFFER_SIZE);
}

void display_set_pixel(Display* disp, size_t x, size_t y, bool on) 
{
  if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) return;
  size_t page = y / 8;
  size_t bit = y % 8;
  size_t idx = page * DISPLAY_WIDTH + x;
  if (on) disp->framebuffer[idx] |= (1 << bit);
  else disp->framebuffer[idx] &= ~(1 << bit);
}

bool display_get_pixel(const Display* disp, size_t x, size_t y) 
{
  if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) return false;
  size_t page = y / 8;
  size_t bit = y % 8;
  size_t idx = page * DISPLAY_WIDTH + x;
  return ((disp->framebuffer[idx] >> bit) & 1) != 0;
}

void display_draw_hline(Display* disp, size_t x, size_t y, size_t width, bool on) 
{
  for (size_t dx = 0; dx < width; dx++) display_set_pixel(disp, x + dx, y, on);
}

void display_draw_vline(Display* disp, size_t x, size_t y, size_t height, bool on) 
{
  for (size_t dy = 0; dy < height; dy++) display_set_pixel(disp, x, y + dy, on);
}

void display_draw_rect(Display* disp, size_t x, size_t y, size_t width, size_t height, bool on) 
{
  display_draw_hline(disp, x, y, width, on);
  display_draw_hline(disp, x, y + height - 1, width, on);
  display_draw_vline(disp, x, y, height, on);
  display_draw_vline(disp, x + width - 1, y, height, on);
}

void display_fill_rect(Display* disp, size_t x, size_t y, size_t width, size_t height, bool on) 
{
  for (size_t dy = 0; dy < height; dy++) display_draw_hline(disp, x, y + dy, width, on);
}

size_t display_draw_char(Display* disp, size_t x, size_t y, char c) 
{
  uint8_t c_val = (uint8_t)c;
  if (c_val < 32 || c_val > 95 + 32) return 0;
  size_t idx = ((size_t)(c_val - 32)) * 5;
  if (idx + 5 > sizeof(FONT_5X7)) return 0;
  for (size_t col = 0; col < 5; col++) 
  {
    uint8_t bits = FONT_5X7[idx + col];
    for (size_t row = 0; row < 7; row++) 
    {
      bool pixel_on = ((bits >> row) & 1) != 0;
      display_set_pixel(disp, x + col, y + row, pixel_on);
    }
  }
  return 6;
}

void display_draw_text(Display* disp, size_t x, size_t y, const char* text) 
{
  size_t cx = x;
  for (const char* p = text; *p != '\0'; p++) 
  {
    if (cx >= DISPLAY_WIDTH) break;
    cx += display_draw_char(disp, cx, y, *p);
  }
}

void display_draw_text_centered(Display* disp, size_t y, const char* text) 
{
  size_t text_len = strlen(text);
  size_t width = text_len * 6;
  size_t x = (width < DISPLAY_WIDTH) ? ((DISPLAY_WIDTH - width) / 2) : 0;
  display_draw_text(disp, x, y, text);
}

DisplayError display_set_contrast(Display* disp, uint8_t contrast) 
{
  disp->contrast = contrast;
  const uint8_t cmds[] = {CMD_SET_CONTRAST, contrast};
  return display_write_commands(disp, cmds, 2);
}

DisplayError display_power_on(Display* disp) 
{
  DisplayError err = display_write_command(disp, CMD_DISPLAY_ON);
  if (err == DISPLAY_ERROR_OK) disp->power_on = true;
  return err;
}

DisplayError display_power_off(Display* disp) 
{
  DisplayError err = display_write_command(disp, CMD_DISPLAY_OFF);
  if (err == DISPLAY_ERROR_OK) disp->power_on = false;
  return err;
}

DisplayError display_invert(Display* disp, bool invert) 
{
  disp->inverted = invert;
  uint8_t cmd = invert ? CMD_INVERT_DISPLAY : CMD_NORMAL_DISPLAY;
  return display_write_command(disp, cmd);
}

bool display_is_on(const Display* disp) 
{
  return disp->power_on;
}

const uint8_t* display_framebuffer(const Display* disp) 
{
  return disp->framebuffer;
}

uint8_t* display_framebuffer_mut(Display* disp) 
{
  return disp->framebuffer;
}

void display_draw_bitmap_24x24(Display* disp, size_t x, size_t y, const uint8_t bitmap[72]) 
{
  for (size_t col = 0; col < 24; col++) 
  {
    for (size_t page = 0; page < 3; page++) 
    {
      uint8_t byte = bitmap[page * 24 + col];
      for (size_t bit = 0; bit < 8; bit++) 
      {
        size_t px_y = y + page * 8 + bit;
        size_t px_x = x + col;
        if (px_x < DISPLAY_WIDTH && px_y < DISPLAY_HEIGHT) display_set_pixel(disp, px_x, px_y, ((byte >> bit) & 1) != 0);
      }
    }
  }
}

void display_draw_bitmap_32x32(Display* disp, size_t x, size_t y, const uint8_t bitmap[128]) 
{
  for (size_t col = 0; col < 32; col++) 
  {
    for (size_t page = 0; page < 4; page++) 
    {
      uint8_t byte = bitmap[page * 32 + col];
      for (size_t bit = 0; bit < 8; bit++) 
      {
        size_t px_y = y + page * 8 + bit;
        size_t px_x = x + col;
        if (px_x < DISPLAY_WIDTH && px_y < DISPLAY_HEIGHT) display_set_pixel(disp, px_x, px_y, ((byte >> bit) & 1) != 0);
      }
    }
  }
}

// StatusDisplay implementation

void status_display_init_struct(StatusDisplay* status_disp, I2cInterface i2c) 
{
  display_init_struct(&status_disp->display, i2c);
}

StatusDisplay status_display_new(I2cInterface i2c) 
{
  StatusDisplay status_disp;
  status_display_init_struct(&status_disp, i2c);
  return status_disp;
}

DisplayError status_display_init(StatusDisplay* status_disp) 
{
  return display_init(&status_disp->display);
}

DisplayError status_display_show_splash(StatusDisplay* status_disp) 
{
  display_clear(&status_disp->display);
  // Draw border
  display_draw_rect(&status_disp->display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, true);
  // Draw text
  display_draw_text_centered(&status_disp->display, 8, "LunarCore");
  display_draw_text_centered(&status_disp->display, 18, "v1.0.0");
  display_draw_text_centered(&status_disp->display, 32, "Unified Mesh");
  display_draw_text_centered(&status_disp->display, 42, "Bridge Firmware");
  display_draw_text_centered(&status_disp->display, 54, "MC | MT | RN");
  return display_flush(&status_disp->display);
}

DisplayError status_display_show_status(StatusDisplay* status_disp, const StatusContent* status) 
{
  display_clear(&status_disp->display);
  // Protocol name
  display_draw_text(&status_disp->display, 0, 0, status->protocol);
  // Battery percentage
  char batt_str[5];
  format_battery(status->battery_pct, batt_str, sizeof(batt_str));
  display_draw_text(&status_disp->display, DISPLAY_WIDTH - 24, 0, batt_str);
  // Separator line
  display_draw_hline(&status_disp->display, 0, 9, DISPLAY_WIDTH, true);
  // Node ID
  display_draw_text(&status_disp->display, 0, 12, "ID:");
  char id_str[9];
  format_hex32(status->node_id, id_str, sizeof(id_str));
  display_draw_text(&status_disp->display, 24, 12, id_str);
  // RX count
  display_draw_text(&status_disp->display, 0, 22, "RX:");
  char rx_str[11];
  format_u32(status->rx_count, rx_str, sizeof(rx_str));
  display_draw_text(&status_disp->display, 24, 22, rx_str);
  // TX count
  display_draw_text(&status_disp->display, 64, 22, "TX:");
  char tx_str[11];
  format_u32(status->tx_count, tx_str, sizeof(tx_str));
  display_draw_text(&status_disp->display, 88, 22, tx_str);
  // RSSI
  display_draw_text(&status_disp->display, 0, 32, "RSSI:");
  char rssi_str[7];
  format_i16(status->rssi, rssi_str, sizeof(rssi_str));
  display_draw_text(&status_disp->display, 36, 32, rssi_str);
  display_draw_text(&status_disp->display, 72, 32, "dBm");
  // Connection status
  display_draw_text(&status_disp->display, 0, 44, "Status:");
  if (status->connected) display_draw_text(&status_disp->display, 48, 44, "CONNECTED");
  else display_draw_text(&status_disp->display, 48, 44, "WAITING");
  // Bottom separator
  display_draw_hline(&status_disp->display, 0, 54, DISPLAY_WIDTH, true);
  // Footer
  display_draw_text_centered(&status_disp->display, 56, "github.com/yours");
  return display_flush(&status_disp->display);
}

DisplayError status_display_show_error(StatusDisplay* status_disp, const char* msg) 
{
  display_clear(&status_disp->display);
  display_draw_text_centered(&status_disp->display, 20, "ERROR");
  display_draw_hline(&status_disp->display, 20, 30, DISPLAY_WIDTH - 40, true);
  display_draw_text_centered(&status_disp->display, 36, msg);
  return display_flush(&status_disp->display);
}

DisplayError status_display_show_message(StatusDisplay* status_disp, const char* line1, const char* line2) 
{
  display_clear(&status_disp->display);
  display_draw_text_centered(&status_disp->display, 24, line1);
  display_draw_text_centered(&status_disp->display, 36, line2);
  return display_flush(&status_disp->display);
}

DisplayError status_display_power_off(StatusDisplay* status_disp) 
{
  return display_power_off(&status_disp->display);
}

DisplayError status_display_power_on(StatusDisplay* status_disp) 
{
  return display_power_on(&status_disp->display);
}

Display* status_display_get_display(StatusDisplay* status_disp) 
{
  return &status_disp->display;
}

DisplayError status_display_boot_animation(StatusDisplay* status_disp, void (*delay_fn)(uint32_t)) 
{
  // Two cycles through all moon phases
  for (size_t cycle = 0; cycle < 2; cycle++) 
  {
    for (size_t phase = 0; phase < 8; phase++) 
    {
      display_clear(&status_disp->display);
      // Draw moon phase
      size_t moon_x = (DISPLAY_WIDTH - 24) / 2;
      size_t moon_y = 8;
      display_draw_bitmap_24x24(&status_disp->display, moon_x, moon_y, MOON_PHASES[phase]);
      // Draw text
      display_draw_text_centered(&status_disp->display, 40, "LUNARCORE");
      // Draw progress dots
      size_t dots_x = (DISPLAY_WIDTH - 8 * 4) / 2;
      for (size_t i = 0; i < 8; i++) 
      {
        bool dot_on = i <= phase;
        display_fill_rect(&status_disp->display, dots_x + i * 4, 54, 2, 2, dot_on);
      }
      DisplayError err = display_flush(&status_disp->display); return err;
      if (err != DISPLAY_ERROR_OK) 
      // Delay
      uint32_t delay_ms = (cycle == 1) ? 120 : 80;
      delay_fn(delay_ms);
    }
  }
  // Show branding
  DisplayError err = status_display_show_branding(status_disp);
  if (err != DISPLAY_ERROR_OK) return err;
  delay_fn(1500);
  return DISPLAY_ERROR_OK;
}

DisplayError status_display_show_branding(StatusDisplay* status_disp) 
{
  display_clear(&status_disp->display);
  // Draw logo
  size_t logo_x = (DISPLAY_WIDTH - 32) / 2;
  size_t logo_y = 4;
  display_draw_bitmap_32x32(&status_disp->display, logo_x, logo_y, YOURS_FACE);
  // Draw text
  display_draw_text_centered(&status_disp->display, 42, "[ YOURS ]");
  display_draw_text_centered(&status_disp->display, 52, "x [ LUNARCORE ]");
  return display_flush(&status_disp->display);
}

DisplayError status_display_show_init_progress(StatusDisplay* status_disp, const char* step, uint8_t progress) 
{
  display_clear(&status_disp->display);
  display_draw_text_centered(&status_disp->display, 8, "INITIALIZING");
  display_draw_text_centered(&status_disp->display, 24, step);
  // Progress bar
  const size_t bar_width = 100;
  size_t bar_x = (DISPLAY_WIDTH - bar_width) / 2;
  size_t bar_y = 40;
  size_t filled = (bar_width * progress) / 100;
  display_draw_rect(&status_disp->display, bar_x, bar_y, bar_width, 8, true);
  // Draw filled portion (with bounds check)
  if (filled > 2) display_fill_rect(&status_disp->display, bar_x + 1, bar_y + 1, filled - 2, 6, true);
  // Progress percentage
  char pct_str[5];
  format_battery(progress, pct_str, sizeof(pct_str));
  display_draw_text_centered(&status_disp->display, 52, pct_str);
  return display_flush(&status_disp->display);
}

