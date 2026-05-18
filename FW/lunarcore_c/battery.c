#include "battery.h"

// Implementation: BatteryState functions

static void battery_state_new(BatteryState *battery) 
{
  battery->voltage_mv = 0;
  battery->percentage = 0;
  battery->is_charging = false;
  battery->is_low = false;
  battery->is_critical = false;
}

static uint8_t battery_state_voltage_to_percentage(uint32_t mv) 
{
  // Voltage to percentage lookup curve
  typedef struct 
  {
    uint32_t voltage;
    uint8_t percentage;
  } CurvePoint;
    
  static const CurvePoint curve[11] = 
  {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 70}, {3800, 60},
    {3700, 50}, {3600, 40}, {3500, 30}, {3400, 20}, {3300, 10}, {3000, 0}
  };
    
    if (mv >= curve[0].voltage) return 100;
    if (mv <= curve[10].voltage) return 0;
    
    // Linear interpolation between curve points
    for (int i = 0; i < 10; i++) 
    {
      if (mv >= curve[i + 1].voltage && mv <= curve[i].voltage) {
      uint32_t v_range = curve[i].voltage - curve[i + 1].voltage;
      uint8_t p_range = curve[i].percentage - curve[i + 1].percentage;
      uint32_t v_offset = mv - curve[i + 1].voltage;
  
      return curve[i + 1].percentage + ((v_offset * p_range) / v_range);
    }
  }
  return 50;
}

static void battery_state_update(BatteryState *battery, uint32_t adc_value) 
{
  // Convert ADC reading to voltage
  uint32_t vadc_mv = (adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;
  battery->voltage_mv = (uint32_t)(vadc_mv * BATTERY_DIVIDER_RATIO);
    
  // Calculate percentage
  battery->percentage = battery_state_voltage_to_percentage(battery->voltage_mv);
    
  // Update status flags
  battery->is_low = battery->voltage_mv < BATTERY_LOW_MV;
  battery->is_critical = battery->voltage_mv < BATTERY_CRITICAL_MV;
  battery->is_charging = battery->voltage_mv > 4200;
}

static void lunar_core_update_battery(LunarCore *core, uint32_t adc_value) 
{
  battery_state_update(&core->battery, adc_value);
    
  if (core->battery.is_critical) 
  {
    led_controller_set_error(&core->led);
    ESP_LOGW("LunarCore", "Battery critical: %dmV", core->battery.voltage_mv);
  } 
  else if (core->battery.is_low) 
  {
    ESP_LOGI("LunarCore", "Battery low: %dmV (%d%%)",core->battery.voltage_mv, core->battery.percentage);
  }
}