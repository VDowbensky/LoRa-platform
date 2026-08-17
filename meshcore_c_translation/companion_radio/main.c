#include <Mesh.h>
#include "MyMesh.h"

void init(void)
{
  serial_init();
  display_init();
  display_showstring("Loading...");
  if(radio_init() != RADIO_OK)
  {
    radio_error_handler();
  }
  rng_init();//  fast_rng.begin(radio_driver.getRngSeed());
  filesystem_init();
  sensors_init();
  gps_init();
  ui_init();
  indicate_boot_complete();
}

void main(void) 
{
  init();
  while(1)
  {
    mesh_proc(); //the_mesh.loop();
    serial_interface_proc(); //interface_manager.loop();
    sensors_proc(); sensors.loop();
    ui_proc(); //ui_task.loop();
    rtc_tick_proc(); //rtc_clock.tick();
    watchdog_feed();
    //if (!the_mesh.hasPendingWork())
    //sleep if possible
  }

}
