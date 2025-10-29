



lr20xx_status_t
lr20xx_radio_fsk_set_modulation_params(void *context,lr20xx_radio_fsk_mod_params_t *mod_params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_fsk_mod_params_t *mod_params_local;
  void *context_local;
  uint8_t cbuffer [11];
  lr20xx_status_t write_status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '@';
  cbuffer[2] = (uint8_t)(mod_params->br >> 0x18);
  cbuffer[3] = (uint8_t)(mod_params->br >> 0x10);
  cbuffer[4] = (uint8_t)(mod_params->br >> 8);
  cbuffer[5] = (uint8_t)mod_params->br;
  cbuffer[6] = mod_params->pulse_shape;
  cbuffer[7] = mod_params->bw;
  cbuffer[8] = (uint8_t)(mod_params->fdev_in_hz >> 0x10);
  cbuffer[9] = (uint8_t)(mod_params->fdev_in_hz >> 8);
  cbuffer[10] = (uint8_t)mod_params->fdev_in_hz;
  lVar1 = lr20xx_hal_write(context,cbuffer,0xb,(uint8_t *)0x0,0);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lVar1 = lr20xx_workarounds_dcdc_configure(context);
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_set_packet_params(void *context,lr20xx_radio_fsk_pkt_params_t *pkt_params)
{
  lr20xx_hal_status_t lVar1;
  char cVar2;
  lr20xx_radio_fsk_pkt_params_t *pkt_params_local;
  void *context_local;
  uint8_t cbuffer [9];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'A';
  cbuffer[2] = (uint8_t)(pkt_params->pbl_length_in_bit >> 8);
  cbuffer[3] = (uint8_t)pkt_params->pbl_length_in_bit;
  cbuffer[4] = pkt_params->preamble_detector;
  if (pkt_params->long_preamble_enabled == false)
  {
    cVar2 = '\0';
  }
  else
  {
    cVar2 = ' ';
  }
  cbuffer[5] = pkt_params->header_mode +
               pkt_params->address_filtering * '\x04' +
               pkt_params->payload_length_unit * '\x10' + cVar2;
  cbuffer[6] = (uint8_t)(pkt_params->payload_length >> 8);
  cbuffer[7] = (uint8_t)pkt_params->payload_length;
  cbuffer[8] = pkt_params->whitening + pkt_params->crc * '\x10';
  lVar1 = lr20xx_hal_write(context,cbuffer,9,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_set_whitening_params
          (void *context,lr20xx_radio_fsk_whitening_compatibility_t whitening_type,
          uint16_t whitening_seed)
{
  lr20xx_hal_status_t lVar1;
  uint16_t whitening_seed_local;
  lr20xx_radio_fsk_whitening_compatibility_t whitening_type_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'B';
  cbuffer[2] = ((byte)(whitening_seed >> 8) & 0xf) + whitening_type * '\x10';
  cbuffer[3] = (uint8_t)whitening_seed;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_set_crc_params(void *context,uint32_t crc_polynomial,uint32_t crc_seed)
{
  lr20xx_hal_status_t lVar1;
  uint32_t crc_seed_local;
  uint32_t crc_polynomial_local;
  void *context_local;
  uint8_t cbuffer [10];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'C';
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
lr20xx_radio_fsk_set_syncword
          (void *context,uint8_t *syncword,uint8_t nb_bits,
          lr20xx_radio_fsk_syncword_bit_order_t bit_order)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_fsk_syncword_bit_order_t bit_order_local;
  uint8_t nb_bits_local;
  uint8_t *syncword_local;
  void *context_local;
  uint8_t cbuffer [11];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'D';
  cbuffer[2] = *syncword;
  cbuffer[3] = syncword[1];
  cbuffer[4] = syncword[2];
  cbuffer[5] = syncword[3];
  cbuffer[6] = syncword[4];
  cbuffer[7] = syncword[5];
  cbuffer[8] = syncword[6];
  cbuffer[9] = syncword[7];
  cbuffer[10] = nb_bits + bit_order * -0x80;
  lVar1 = lr20xx_hal_write(context,cbuffer,0xb,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_set_addresses(void *context,uint8_t node_address,uint8_t broadcast_address)
{
  lr20xx_hal_status_t lVar1;
  uint8_t broadcast_address_local;
  uint8_t node_address_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'E';
  cbuffer[2] = node_address;
  cbuffer[3] = broadcast_address;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_get_packet_status(void *context,lr20xx_radio_fsk_packet_status_t *pkt_status)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_fsk_packet_status_t *pkt_status_local;
  void *context_local;
  uint8_t rbuffer [6];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'G';
  rbuffer[0] = '\0';
  rbuffer[1] = '\0';
  rbuffer[2] = '\0';
  rbuffer[3] = '\0';
  rbuffer[4] = '\0';
  rbuffer[5] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rbuffer,6);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    pkt_status->packet_length_bytes = (ushort)rbuffer[1] + (ushort)rbuffer[0] * 0x100;
    pkt_status->rssi_avg_in_dbm = -(ushort)rbuffer[2];
    pkt_status->rssi_sync_in_dbm = -(ushort)rbuffer[3];
    pkt_status->is_addr_match_broadcast = (rbuffer[4] >> 5 & 1) != 0;
    pkt_status->is_addr_match_node = (rbuffer[4] >> 4 & 1) != 0;
    pkt_status->rssi_avg_half_dbm_count = rbuffer[4] >> 2 & 1;
    pkt_status->rssi_sync_half_dbm_count = rbuffer[4] & 1;
    pkt_status->link_quality_indicator = rbuffer[5];
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_fsk_get_rx_bandwidth(uint32_t bw_in_hz,lr20xx_radio_fsk_common_bw_t *bw_parameter)
{
  lr20xx_radio_fsk_common_bw_t *bw_parameter_local;
  uint32_t bw_in_hz_local;
  uint8_t i;
  
  i = '\0';
  while( true )
  {
    if (0x58 < i)
    {
      return LR20XX_STATUS_ERROR;
    }
    if (bw_in_hz <= lr20xx_radio_gfsk_bw_values[i].bw_in_hz) break;
    i += '\x01';
  }
  *bw_parameter = lr20xx_radio_gfsk_bw_values[i].param;
  return LR20XX_STATUS_OK;
}



uint32_t lr20xx_radio_fsk_get_time_on_air_numerator
                   (lr20xx_radio_fsk_pkt_params_t *pkt_p,uint8_t syncword_len_in_bit)
{
  lr20xx_radio_fsk_header_mode_t lVar1;
  lr20xx_radio_fsk_address_filtering_t lVar2;
  uint16_t uVar3;
  ushort uVar4;
  uint32_t uVar5;
  uint8_t syncword_len_in_bit_local;
  lr20xx_radio_fsk_pkt_params_t *pkt_p_local;
  uint8_t header_len_in_bits;
  
  lVar1 = pkt_p->header_mode;
  if (lVar1 == LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE)
  {
    header_len_in_bits = '\t';
  }
  else
  {
    if (LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE < lVar1)
    {
      return 0;
    }
    if (lVar1 == LR20XX_RADIO_FSK_HEADER_IMPLICIT)
    {
      header_len_in_bits = '\0';
    }
    else
    {
      if (lVar1 != LR20XX_RADIO_FSK_HEADER_8BITS)
      {
        return 0;
      }
      header_len_in_bits = '\b';
    }
  }
  uVar3 = pkt_p->pbl_length_in_bit;
  uVar4 = pkt_p->payload_length;
  lVar2 = pkt_p->address_filtering;
  uVar5 = lr20xx_radio_fsk_get_crc_len_in_bytes(pkt_p->crc);
  return (uVar5 + (uint)uVar4 + (uint)(lVar2 != LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED)) * 8 +
         ((uint)syncword_len_in_bit + (uint)(ushort)(header_len_in_bits + uVar3) & 0xffff);
}



uint32_t lr20xx_radio_fsk_get_time_on_air_in_ms
                   (lr20xx_radio_fsk_pkt_params_t *pkt_p,lr20xx_radio_fsk_mod_params_t *mod_p,
                   uint8_t syncword_len_in_bit)
{
  uint32_t uVar1;
  uint8_t syncword_len_in_bit_local;
  lr20xx_radio_fsk_mod_params_t *mod_p_local;
  lr20xx_radio_fsk_pkt_params_t *pkt_p_local;
  uint32_t denominator;
  uint32_t numerator;
  
  uVar1 = lr20xx_radio_fsk_get_time_on_air_numerator(pkt_p,syncword_len_in_bit);
  return ((mod_p->br + uVar1 * 1000) - 1) / mod_p->br;
}



uint32_t lr20xx_radio_fsk_get_crc_len_in_bytes(lr20xx_radio_fsk_crc_t crc_type)
{
  uint32_t uVar1;
  lr20xx_radio_fsk_crc_t crc_type_local;
  
  if (false)
  {
switchD_08025d8e_caseD_5:
    uVar1 = 0;
  }
  else
  {
    switch(crc_type)
    {
    case LR20XX_RADIO_FSK_CRC_OFF:
      uVar1 = 0;
      break;
    case LR20XX_RADIO_FSK_CRC_1_BYTE:
      uVar1 = 1;
      break;
    case LR20XX_RADIO_FSK_CRC_2_BYTES:
      uVar1 = 2;
      break;
    case LR20XX_RADIO_FSK_CRC_3_BYTES:
      uVar1 = 3;
      break;
    case LR20XX_RADIO_FSK_CRC_4_BYTES:
      uVar1 = 4;
      break;
    default:
      goto switchD_08025d8e_caseD_5;
    case LR20XX_RADIO_FSK_CRC_1_BYTE_INVERTED:
      uVar1 = 1;
      break;
    case LR20XX_RADIO_FSK_CRC_2_BYTES_INVERTED:
      uVar1 = 2;
      break;
    case LR20XX_RADIO_FSK_CRC_3_BYTES_INVERTED:
      uVar1 = 3;
      break;
    case LR20XX_RADIO_FSK_CRC_4_BYTES_INVERTED:
      uVar1 = 4;
    }
  }
  return uVar1;
}