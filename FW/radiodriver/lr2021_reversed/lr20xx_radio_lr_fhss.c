lr20xx_status_t
lr20xx_radio_lr_fhss_build_frame
          (void *context,lr20xx_radio_lr_fhss_params_t *lr_fhss_params,uint16_t hop_sequence_id,
          uint8_t *payload,uint8_t payload_length)
{
  lr20xx_hal_status_t lVar1;
  char cVar2;
  uint8_t *payload_local;
  uint16_t hop_sequence_id_local;
  lr20xx_radio_lr_fhss_params_t *lr_fhss_params_local;
  void *context_local;
  uint8_t cbuffer [8];
  lr20xx_status_t status;
  
  lVar1 = lr20xx_radio_lr_fhss_set_sync_word(context,(lr_fhss_params->lr_fhss_params).sync_word);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    cbuffer[0] = '\x02';
    cbuffer[1] = 'V';
    cbuffer[2] = (lr_fhss_params->lr_fhss_params).cr +
                 (lr_fhss_params->lr_fhss_params).header_count * '\x10';
    cbuffer[3] = (lr_fhss_params->lr_fhss_params).grid +
                 (lr_fhss_params->lr_fhss_params).modulation_type * '\x10';
    if ((lr_fhss_params->lr_fhss_params).enable_hopping == false)
    {
      cVar2 = '\0';
    }
    else
    {
      cVar2 = '\x10';
    }
    cbuffer[4] = (lr_fhss_params->lr_fhss_params).bw + cVar2;
    cbuffer[5] = (uint8_t)(hop_sequence_id >> 8);
    cbuffer[6] = (uint8_t)hop_sequence_id;
    cbuffer[7] = lr_fhss_params->device_offset;
    lVar1 = lr20xx_hal_write(context,cbuffer,8,payload,(ushort)payload_length);
  }
  return lVar1;
}



lr20xx_status_t lr20xx_radio_lr_fhss_set_sync_word(void *context,uint8_t *sync_word)
{
  lr20xx_hal_status_t lVar1;
  uint8_t *sync_word_local;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'W';
  lVar1 = lr20xx_hal_write(context,cbuffer,2,sync_word,4);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_lr_fhss_init(void *context)
{
  lr20xx_status_t lVar1;
  void *context_local;
  
  lVar1 = lr20xx_radio_common_set_pkt_type(context,LR20XX_RADIO_COMMON_PKT_TYPE_LRFHSS);
  return lVar1;
}



uint32_t lr20xx_radio_lr_fhss_get_time_on_air_in_ms
                   (lr20xx_radio_lr_fhss_params_t *params,uint16_t payload_length)
{
  uint16_t uVar1;
  uint16_t payload_length_local;
  lr20xx_radio_lr_fhss_params_t *params_local;
  
  uVar1 = lr20xx_radio_lr_fhss_get_nb_bits(&params->lr_fhss_params,payload_length);
  return ((uint)uVar1 * 0x100 + 0x7c) / 0x7d;
}



uint lr20xx_radio_lr_fhss_get_hop_sequence_count(lr20xx_radio_lr_fhss_params_t *lr_fhss_params)
{
  uint uVar1;
  lr20xx_radio_lr_fhss_params_t *lr_fhss_params_local;
  
  if (((lr_fhss_params->lr_fhss_params).grid == LR_FHSS_V1_GRID_25391_HZ) ||
     (((lr_fhss_params->lr_fhss_params).grid == LR_FHSS_V1_GRID_3906_HZ &&
      ((lr_fhss_params->lr_fhss_params).bw < LR_FHSS_V1_BW_335938_HZ))))
  {
    uVar1 = 0x180;
  }
  else
  {
    uVar1 = 0x200;
  }
  return uVar1;
}



uint16_t lr20xx_radio_lr_fhss_get_nb_bits(lr_fhss_v1_params_t *params,uint16_t payload_length)
{
  uint16_t payload_length_local;
  lr_fhss_v1_params_t *params_local;
  uint16_t last_block_bits;
  uint16_t payload_bits;
  uint16_t length_bits;
  
  length_bits = (payload_length + 2) * 8 + 6;
  switch(params->cr)
  {
  case LR_FHSS_V1_CR_5_6:
    length_bits = (uint16_t)(((uint)length_bits * 6 + 4) / 5);
    break;
  case LR_FHSS_V1_CR_2_3:
    length_bits = (uint16_t)((uint)length_bits * 3 >> 1);
    break;
  case LR_FHSS_V1_CR_1_2:
    length_bits *= 2;
    break;
  case LR_FHSS_V1_CR_1_3:
    length_bits *= 3;
  }
  payload_bits = (length_bits / 0x30) * 0x32;
  if (length_bits % 0x30 != 0)
  {
    payload_bits = payload_bits + length_bits % 0x30 + 2;
  }
  return payload_bits + (ushort)params->header_count * 0x72;
}
