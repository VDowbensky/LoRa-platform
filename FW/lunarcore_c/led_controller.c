
// Implementation: LedController functions

static void led_controller_new(LedController *led) 
{
  led->is_on = false;
  led->last_toggle = 0;
  led->interval_ms = LED_BLINK_IDLE;
  led->blink_count = 0;
  led->remaining = 0;
}

static void led_controller_set_idle(LedController *led) 
{
  led->interval_ms = LED_BLINK_IDLE;
  led->blink_count = 0;
}

static void led_controller_set_active(LedController *led) 
{
  led->interval_ms = LED_BLINK_ACTIVE;
  led->blink_count = 0;
}

static void led_controller_set_error(LedController *led) 
{
  led->interval_ms = LED_BLINK_ERROR;
  led->blink_count = 0;
}

static void led_controller_flash(LedController *led, uint8_t count) 
{
  led->blink_count = count;
  led->remaining = count * 2;
  led->interval_ms = 100;
}

static bool led_controller_update(LedController *led, uint32_t current_time) 
{
  uint32_t elapsed = current_time - led->last_toggle;
  if (elapsed >= led->interval_ms) 
  {
    led->last_toggle = current_time;
        
    if (led->blink_count > 0) 
    {
      if (led->remaining > 0) 
      {
        led->remaining--;
        led->is_on = !led->is_on;
        return true;
      } else 
      {
        // Blink sequence complete, return to idle
        led_controller_set_idle(led);
      }
    } 
    else 
    {
      led->is_on = !led->is_on;
      return true;
    }
  }
  return false;
}