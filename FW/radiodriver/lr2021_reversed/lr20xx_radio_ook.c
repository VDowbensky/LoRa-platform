


lr20xx_status_t
lr20xx_radio_ook_set_modulation_params(void *context,lr20xx_radio_ook_mod_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_ook_mod_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [9];
  lr20xx_status_t write_status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x81;
  cbuffer[2] = (uint8_t)(params->br >> 0x18);
  cbuffer[3] = (uint8_t)(params->br >> 0x10);
  cbuffer[4] = (uint8_t)(params->br >> 8);
  cbuffer[5] = (uint8_t)params->br;
  cbuffer[6] = pulse_shape_to_byte(&params->pulse_shape);
  cbuffer[7] = params->bw;
  cbuffer[8] = params->mag_depth;
  lVar1 = lr20xx_hal_write(context,cbuffer,9,(uint8_t *)0x0,0);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lVar1 = lr20xx_workarounds_dcdc_configure(context);
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_packet_params(void *context,lr20xx_radio_ook_pkt_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_ook_pkt_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [8];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x82;
  cbuffer[2] = (uint8_t)(params->pbl_length_in_bit >> 8);
  cbuffer[3] = (uint8_t)params->pbl_length_in_bit;
  cbuffer[4] = params->header_mode + params->address_filtering * '\x04';
  cbuffer[5] = (uint8_t)(params->payload_length >> 8);
  cbuffer[6] = (uint8_t)params->payload_length;
  cbuffer[7] = params->encoding + params->crc * '\x10';
  lVar1 = lr20xx_hal_write(context,cbuffer,8,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_crc_params(void *context,uint32_t crc_polynomial,uint32_t crc_seed)
{
  lr20xx_hal_status_t lVar1;
  uint32_t crc_seed_local;
  uint32_t crc_polynomial_local;
  void *context_local;
  uint8_t cbuffer [10];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x83;
  cbuffer[2] = (uint8_t)(crc_polynomial >> 0x18);
  cbuffer[3] = (uint8_t)(crc_polynomial >> 0x10);
  cbuffer[4] = (uint8_t)(crc_polynomial >> 8);
  cbuffer[5] = (uint8_t)crc_polynomial;
  cbuffer[6] = (uint8_t)(crc_seed >> 0x18);
  cbuffer[7] = (uint8_t)(crc_seed >> 0x10);
  cbuffer[8] = (uint8_t)(crc_seed >> 8);
  cbuffer[9] = (uint8_t)crc_seed;
  lVar1 = lr20xx_hal_write(context,cbuffer,10,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_syncword
          (void *context,uint8_t *syncword,uint8_t nb_bits,
          lr20xx_radio_ook_syncword_bit_order_t bit_order)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_ook_syncword_bit_order_t bit_order_local;
  uint8_t nb_bits_local;
  uint8_t *syncword_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x84;
  cbuffer[2] = *syncword;
  cbuffer[3] = syncword[1];
  cbuffer[4] = syncword[2];
  cbuffer[5] = syncword[3];
  cbuffer[6] = nb_bits + bit_order * -0x80;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_addresses(void *context,uint8_t node_address,uint8_t broadcast_address)
{
  lr20xx_hal_status_t lVar1;
  uint8_t broadcast_address_local;
  uint8_t node_address_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x85;
  cbuffer[2] = node_address;
  cbuffer[3] = broadcast_address;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_rx_detector(void *context,lr20xx_radio_ook_rx_detector_t *rx_detector)
{
  lr20xx_hal_status_t lVar1;
  byte bVar2;
  lr20xx_radio_ook_rx_detector_t *rx_detector_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x88;
  cbuffer[2] = (uint8_t)(rx_detector->pattern >> 8);
  cbuffer[3] = (uint8_t)rx_detector->pattern;
  cbuffer[4] = rx_detector->pattern_length_in_bit;
  cbuffer[5] = rx_detector->pattern_repeat_nb;
  if (rx_detector->is_syncword_encoded == false)
  {
    bVar2 = 0x20;
  }
  else
  {
    bVar2 = 0;
  }
  cbuffer[6] = rx_detector->sfd_length_in_bit | rx_detector->sfd_type << 4 | bVar2;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_ook_set_whitening_params(void *context,lr20xx_radio_ook_whitening_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_ook_whitening_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 0x89;
  cbuffer[2] = (byte)(params->polynomial >> 8) | params->bit_index << 4;
  cbuffer[3] = (uint8_t)params->polynomial;
  cbuffer[4] = (uint8_t)(params->seed >> 8);
  cbuffer[5] = (uint8_t)params->seed;
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



uint8_t pulse_shape_to_byte(lr20xx_radio_ook_pulse_shape_t *pulse_shape)
{
  lr20xx_radio_ook_pulse_shape_t *pulse_shape_local;
  
  return pulse_shape->filter * '\b' + pulse_shape->bt;
}