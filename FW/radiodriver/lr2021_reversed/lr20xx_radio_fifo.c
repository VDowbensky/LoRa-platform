
lr20xx_status_t lr20xx_radio_fifo_read_rx(void *context,uint8_t *buffer,uint16_t length)
{
  lr20xx_hal_status_t lVar1;
  uint16_t length_local;
  uint8_t *buffer_local;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\0';
  cbuffer[1] = '\x01';
  lVar1 = lr20xx_hal_direct_read_fifo(context,cbuffer,2,buffer,length);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_fifo_write_tx(void *context,uint8_t *buffer,uint16_t length)
{
  lr20xx_hal_status_t lVar1;
  uint16_t length_local;
  uint8_t *buffer_local;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\0';
  cbuffer[1] = '\x02';
  lVar1 = lr20xx_hal_write(context,cbuffer,2,buffer,length);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_fifo_clear_rx(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = 0x1;
  cbuffer[1] = 0x1e;
  lVar1 = lr20xx_hal_write(context,cbuffer,2,(uint8_t *)0x0,0);
  return lVar1;
}