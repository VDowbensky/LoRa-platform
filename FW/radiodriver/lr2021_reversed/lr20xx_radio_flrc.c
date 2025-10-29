
lr20xx_status_t
lr20xx_radio_flrc_set_modulation_params(void *context,lr20xx_radio_flrc_mod_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_flrc_mod_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [4];
  lr20xx_status_t write_status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'H';
  cbuffer[2] = params->br_bw;
  cbuffer[3] = params->shape + params->cr * '\x10';
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lVar1 = lr20xx_workarounds_dcdc_configure(context);
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_flrc_set_pkt_params(void *context,lr20xx_radio_flrc_pkt_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_flrc_pkt_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'I';
  cbuffer[2] = params->preamble_len * '\x04' + params->sync_word_len;
  cbuffer[3] = params->tx_syncword * '@' +
               params->match_sync_word * '\b' + params->header_type * '\x04' + params->crc_type;
  cbuffer[4] = (uint8_t)(params->pld_len_in_bytes >> 8);
  cbuffer[5] = (uint8_t)params->pld_len_in_bytes;
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_flrc_set_syncword(void *context,uint8_t syncword_index,uint8_t *syncword)
{
  lr20xx_status_t lVar1;
  uint8_t *syncword_local;
  uint8_t syncword_index_local;
  void *context_local;
  
  lVar1 = lr20xx_radio_flrc_set_syncword_base(context,syncword_index,syncword,'\x04');
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_flrc_set_syncword_base
          (void *context,uint8_t syncword_index,uint8_t *syncword,uint8_t syncword_length)
{
  lr20xx_hal_status_t lVar1;
  uint8_t *syncword_local;
  uint8_t syncword_length_local;
  uint8_t syncword_index_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'L';
  cbuffer[2] = syncword_index;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,syncword,(ushort)syncword_length);
  return lVar1;
}

