



lr20xx_status_t
lr20xx_radio_lora_set_modulation_params(void *context,lr20xx_radio_lora_mod_params_t *mod_params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_lora_mod_params_t *mod_params_local;
  void *context_local;
  uint8_t cbuffer [4];
  lr20xx_status_t write_status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = ' ';
  cbuffer[2] = mod_params->bw + mod_params->sf * '\x10';
  cbuffer[3] = mod_params->ppm + mod_params->cr * '\x10';
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lVar1 = lr20xx_workarounds_dcdc_configure(context);
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_lora_set_packet_params(void *context,lr20xx_radio_lora_pkt_params_t *pkt_params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_lora_pkt_params_t *pkt_params_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '!';
  cbuffer[2] = (uint8_t)(pkt_params->preamble_len_in_symb >> 8);
  cbuffer[3] = (uint8_t)pkt_params->preamble_len_in_symb;
  cbuffer[4] = pkt_params->pld_len_in_bytes;
  cbuffer[5] = pkt_params->iq + pkt_params->crc * '\x02' + pkt_params->pkt_mode * '\x04';
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_lora_configure_timeout_by_number_of_symbols(void *context,uint8_t n_symbols)
{
  lr20xx_status_t lVar1;
  uint8_t n_symbols_local;
  void *context_local;
  
  lVar1 = abstract_search_symbols(context,n_symbols,SEARCH_SYMBOL_FORMAT_NUMBER);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_lora_set_syncword(void *context,uint8_t syncword)
{
  lr20xx_hal_status_t lVar1;
  uint8_t syncword_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '#';
  cbuffer[2] = syncword;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_lora_configure_cad_params(void *context,lr20xx_radio_lora_cad_params_t *cad_params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_lora_cad_params_t *cad_params_local;
  void *context_local;
  uint8_t cbuffer [9];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\'';
  cbuffer[2] = cad_params->cad_symb_nb;
  cbuffer[3] = cad_params->pnr_delta;
  cbuffer[4] = cad_params->cad_exit_mode;
  cbuffer[5] = (uint8_t)(cad_params->cad_timeout_in_pll_step >> 0x10);
  cbuffer[6] = (uint8_t)(cad_params->cad_timeout_in_pll_step >> 8);
  cbuffer[7] = (uint8_t)cad_params->cad_timeout_in_pll_step;
  cbuffer[8] = cad_params->cad_detect_peak;
  lVar1 = lr20xx_hal_write(context,cbuffer,9,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_lora_set_cad(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '(';
  lVar1 = lr20xx_hal_write(context,cbuffer,2,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_lora_get_packet_status(void *context,lr20xx_radio_lora_packet_status_t *pkt_status)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_lora_packet_status_t *pkt_status_local;
  void *context_local;
  uint8_t rbuffer [6];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '*';
  rbuffer[0] = '\0';
  rbuffer[1] = '\0';
  rbuffer[2] = '\0';
  rbuffer[3] = '\0';
  rbuffer[4] = '\0';
  rbuffer[5] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rbuffer,6);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    pkt_status->crc = rbuffer[0] >> 4 & LR20XX_RADIO_LORA_CRC_ENABLED;
    pkt_status->cr =
         rbuffer[0] & (LR20XX_RADIO_LORA_CR_LI_CONVOLUTIONAL_4_6|LR20XX_RADIO_LORA_CR_LI_4_8);
    pkt_status->packet_length_bytes = rbuffer[1];
    pkt_status->snr_pkt_raw = rbuffer[2];
    pkt_status->rssi_pkt_in_dbm = -(ushort)rbuffer[3];
    pkt_status->rssi_signal_pkt_in_dbm = -(rbuffer._4_2_ & 0xff);
    pkt_status->detector = rbuffer[5] >> 2 & 0xf;
    pkt_status->rssi_pkt_half_dbm_count = rbuffer[5] >> 1 & 1;
    pkt_status->rssi_signal_pkt_half_dbm_count = rbuffer[5] & 1;
  }
  return lVar1;
}



uint32_t lr20xx_radio_lora_get_time_on_air_numerator
                   (lr20xx_radio_lora_pkt_params_t *pkt_p,lr20xx_radio_lora_mod_params_t *mod_p)
{
  bool bVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  int iVar8;
  int iVar9;
  int iVar10;
  bool bVar11;
  lr20xx_radio_lora_mod_params_t *mod_p_local;
  lr20xx_radio_lora_pkt_params_t *pkt_p_local;
  int32_t intermed;
  int32_t tx_codedbits_header;
  int32_t tx_bits_symbol_start;
  int32_t fec_rate_denominator;
  int32_t fec_rate_numerator;
  int32_t tx_bits_symbol;
  int32_t total_bytes_nb;
  _Bool long_interleaving;
  int32_t fine_synch;
  _Bool pld_is_fix;
  int32_t sf;
  int32_t pld_len_in_bytes;
  int32_t tx_infobits_payload;
  int32_t tx_infobits_header;
  int32_t symbols_nb_data;
  int32_t ceil_denominator;
  int32_t ceil_numerator;
  
  uVar5 = (uint)pkt_p->pld_len_in_bytes;
  uVar6 = (uint)mod_p->sf;
  bVar11 = pkt_p->pkt_mode != LR20XX_RADIO_LORA_PKT_IMPLICIT;
  uVar7 = (uint)(uVar6 < 7);
  bVar1 = mod_p->cr < LR20XX_RADIO_LORA_CR_LI_4_5;
  if (pkt_p->crc == LR20XX_RADIO_LORA_CRC_ENABLED)
  {
    iVar4 = 2;
  }
  else
  {
    iVar4 = 0;
  }
  iVar4 = uVar5 + iVar4;
  if (mod_p->ppm == LR20XX_RADIO_LORA_NO_PPM)
  {
    iVar8 = 0;
  }
  else
  {
    iVar8 = 2;
  }
  iVar8 = uVar6 - iVar8;
  if (bVar1)
  {
    tx_infobits_header = (uVar6 + uVar7 * 2 + -2) * 4;
    if (bVar11)
    {
      tx_infobits_header += -0x14;
    }
    tx_infobits_payload = iVar4 * 8 - tx_infobits_header;
    if (tx_infobits_payload < 0)
    {
      tx_infobits_payload = 0;
    }
    ceil_numerator = tx_infobits_payload;
    ceil_denominator = iVar8 * 4;
  }
  else
  {
    iVar9 = (uint)(mod_p->cr == LR20XX_RADIO_LORA_CR_LI_4_8) + (uint)mod_p->cr;
    if (bVar11)
    {
      tx_infobits_header = (uVar6 + uVar7 * 2 + -7) * 4 & 0xfffffff8;
      if ((tx_infobits_header + iVar4 * -8 < 0 != SBORROW4(tx_infobits_header,iVar4 * 8)) &&
         (tx_infobits_header != uVar5 * 8 &&
          (int)(tx_infobits_header + uVar5 * -8) < 0 == SBORROW4(tx_infobits_header,uVar5 * 8)))
      {
        tx_infobits_header = uVar5 << 3;
      }
      tx_infobits_payload = iVar4 * 8 - tx_infobits_header;
      if (tx_infobits_payload < 0)
      {
        tx_infobits_payload = 0;
      }
      ceil_numerator = iVar8 * 0x20 + iVar9 * tx_infobits_payload;
      ceil_denominator = iVar8 * 4;
    }
    else
    {
      iVar10 = uVar7 * 2 + (uVar6 - 2);
      iVar3 = iVar9 * iVar4 * 8;
      iVar2 = iVar3 + iVar10 * -0x1c;
      if (iVar2 == 0 || iVar2 < 0 != SBORROW4(iVar3,iVar10 * 0x1c))
      {
        ceil_numerator = iVar9 * iVar4 * 8;
        ceil_denominator = iVar10 * 4;
      }
      else
      {
        ceil_numerator = (iVar9 * iVar4 + iVar8 * 4) * 8 + iVar10 * -0x20;
        ceil_denominator = iVar8 * 4;
      }
    }
  }
  symbols_nb_data = (ceil_denominator + ceil_numerator + -1) / ceil_denominator;
  if (bVar1)
  {
    symbols_nb_data = symbols_nb_data * (mod_p->cr + 4) + 8;
  }
  return ((uVar7 * 2 + pkt_p->preamble_len_in_symb + 4 + symbols_nb_data) * 4 + 1 <<
         (uVar6 - 2 & 0xff)) - 1;
}



uint32_t lr20xx_radio_lora_get_bw_in_hz(lr20xx_radio_lora_bw_t bw)
{
  lr20xx_radio_lora_bw_t bw_local;
  uint32_t bw_in_hz;
  
  bw_in_hz = 0;
  switch(bw)
  {
  case LR20XX_RADIO_LORA_BW_7:
    bw_in_hz = 0x1e84;
    break;
  case LR20XX_RADIO_LORA_BW_15:
    bw_in_hz = 0x3d09;
    break;
  case LR20XX_RADIO_LORA_BW_31:
    bw_in_hz = 0x7a12;
    break;
  case LR20XX_RADIO_LORA_BW_62:
    bw_in_hz = 0xf424;
    break;
  case LR20XX_RADIO_LORA_BW_125:
    bw_in_hz = 0x1e848;
    break;
  case LR20XX_RADIO_LORA_BW_250:
    bw_in_hz = 250000;
    break;
  case LR20XX_RADIO_LORA_BW_500:
    bw_in_hz = 500000;
    break;
  case LR20XX_RADIO_LORA_BW_1000:
    bw_in_hz = 1000000;
    break;
  case LR20XX_RADIO_LORA_BW_10:
    bw_in_hz = 0x28b1;
    break;
  case LR20XX_RADIO_LORA_BW_20:
    bw_in_hz = 0x5161;
    break;
  case LR20XX_RADIO_LORA_BW_41:
    bw_in_hz = 0xa2c3;
    break;
  case LR20XX_RADIO_LORA_BW_83:
    bw_in_hz = 0x1458c;
    break;
  case LR20XX_RADIO_LORA_BW_101:
    bw_in_hz = 0x18cbb;
    break;
  case LR20XX_RADIO_LORA_BW_203:
    bw_in_hz = 0x318f8;
    break;
  case LR20XX_RADIO_LORA_BW_406:
    bw_in_hz = 0x631f0;
    break;
  case LR20XX_RADIO_LORA_BW_812:
    bw_in_hz = 0xc63e0;
  }
  return bw_in_hz;
}



uint32_t lr20xx_radio_lora_get_time_on_air_in_ms
                   (lr20xx_radio_lora_pkt_params_t *pkt_p,lr20xx_radio_lora_mod_params_t *mod_p)
{
  uint32_t uVar1;
  uint32_t uVar2;
  lr20xx_radio_lora_mod_params_t *mod_p_local;
  lr20xx_radio_lora_pkt_params_t *pkt_p_local;
  uint32_t denominator;
  uint32_t numerator;
  
  uVar1 = lr20xx_radio_lora_get_time_on_air_numerator(pkt_p,mod_p);
  uVar2 = lr20xx_radio_lora_get_bw_in_hz(mod_p->bw);
  return ((uVar2 + uVar1 * 1000) - 1) / uVar2;
}
