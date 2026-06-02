#include "uart_helpers.h"

// Helper function implementations

static void write_hex8(uart_port_t uart, uint8_t value) 
{
  char buf[3];
  sprintf(buf, "%02X", value);
  uart_write_bytes(uart, buf, 2);
}

static void write_hex32(uart_port_t uart, uint32_t value) 
{
  char buf[9];
  sprintf(buf, "%08X", value);
  uart_write_bytes(uart, buf, 8);
}

static void write_u32(uart_port_t uart, uint32_t value) 
{
  char buf[12];
  sprintf(buf, "%u", value);
  uart_write_bytes(uart, buf, strlen(buf));
}

static void write_i16(uart_port_t uart, int16_t value) 
{
  char buf[8];
  sprintf(buf, "%d", value);
  uart_write_bytes(uart, buf, strlen(buf));
}

static const char* format_battery(const BatteryState *battery, char *buf) 
{
  sprintf(buf, "Battery: %dmV (%d%%)", battery->voltage_mv, battery->percentage);
  return buf;
}

static bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result) 
{
  *result = 0;
  for (size_t i = 0; i < len; i++) 
  {
    if (cmd[i] >= '0' && cmd[i] <= '9') *result = *result * 10 + (cmd[i] - '0');
    else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) return true;
    else return false;
  }
  return true;
}

static bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result) 
{
  bool negative = false;
  size_t start = 0;
    
  if (len > 0 && cmd[0] == '-') 
  {
    negative = true;
    start = 1;
  }
    
  int32_t value = 0;
  for (size_t i = start; i < len; i++) 
  {
    if (cmd[i] >= '0' && cmd[i] <= '9') value = value * 10 + (cmd[i] - '0');
    else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) break;
    else return false;
  }
    
  if (negative) value = -value;
  if (value < -128 || value > 127) return false;
    
  *result = (int8_t)value;
  return true;
}

/**
 * A null implementation of vprintf that does nothing and returns 0.
 * Matches the signature of vprintf_like_t.
 */
int null_vprintf(const char *_fmt,va_list _args) 
{
  return 0;
}