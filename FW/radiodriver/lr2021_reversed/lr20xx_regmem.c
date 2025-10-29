


lr20xx_status_t
lr20xx_regmem_write_regmem32(void *context,uint32_t address,uint32_t *buffer,uint8_t length)
{
  lr20xx_hal_status_t lVar1;
  uint8_t length_local;
  uint32_t *buffer_local;
  uint32_t address_local;
  void *context_local;
  uint8_t cdata [256];
  uint8_t cbuffer [5];
  
  if (length < 0x21)
  {
    lr20xx_regmem_fill_cbuffer_cdata_opcode_address_data(cbuffer,cdata,0x104,address,buffer,length);
    lVar1 = lr20xx_hal_write(context,cbuffer,5,cdata,(ushort)length << 2);
  }
  else
  {
    lVar1 = LR20XX_HAL_STATUS_ERROR;
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_regmem_write_regmem32_mask(void *context,uint32_t address,uint32_t mask,uint32_t data)
{
  lr20xx_hal_status_t lVar1;
  uint32_t data_local;
  uint32_t mask_local;
  uint32_t address_local;
  void *context_local;
  uint8_t cbuffer [13];
  
  lr20xx_regmem_fill_cbuffer_opcode_address(cbuffer,0x105,address);
  cbuffer[5] = (uint8_t)(mask >> 0x18);
  cbuffer[6] = (uint8_t)(mask >> 0x10);
  cbuffer[7] = (uint8_t)(mask >> 8);
  cbuffer[8] = (uint8_t)mask;
  cbuffer[9] = (uint8_t)(data >> 0x18);
  cbuffer[10] = (uint8_t)(data >> 0x10);
  cbuffer[0xb] = (uint8_t)(data >> 8);
  cbuffer[0xc] = (uint8_t)data;
  lVar1 = lr20xx_hal_write(context,cbuffer,0xd,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_regmem_read_regmem32(void *context,uint32_t address,uint32_t *buffer,uint8_t length)
{
  lr20xx_hal_status_t lVar1;
  uint8_t length_local;
  uint32_t *buffer_local;
  uint32_t address_local;
  void *context_local;
  uint8_t cbuffer [6];
  lr20xx_status_t status;
  
  if (length < 0x21)
  {
    lr20xx_regmem_fill_cbuffer_opcode_address_length(cbuffer,0x106,address,length);
    lVar1 = lr20xx_hal_read(context,cbuffer,6,(uint8_t *)buffer,(ushort)length << 2);
    if (lVar1 == LR20XX_HAL_STATUS_OK)
    {
      lr20xx_regmem_fill_out_buffer_from_raw_buffer(buffer,(uint8_t *)buffer,length);
    }
  }
  else
  {
    lVar1 = LR20XX_HAL_STATUS_ERROR;
  }
  return lVar1;
}



void lr20xx_regmem_fill_cbuffer_opcode_address(uint8_t *cbuffer,uint16_t opcode,uint32_t address)
{
  uint32_t address_local;
  uint16_t opcode_local;
  uint8_t *cbuffer_local;
  
  *cbuffer = (uint8_t)(opcode >> 8);
  cbuffer[1] = (uint8_t)opcode;
  cbuffer[2] = (uint8_t)(address >> 0x10);
  cbuffer[3] = (uint8_t)(address >> 8);
  cbuffer[4] = (uint8_t)address;
  return;
}



void lr20xx_regmem_fill_cbuffer_opcode_address_length
               (uint8_t *cbuffer,uint16_t opcode,uint32_t address,uint8_t length)
{
  uint32_t address_local;
  uint8_t length_local;
  uint16_t opcode_local;
  uint8_t *cbuffer_local;
  
  lr20xx_regmem_fill_cbuffer_opcode_address(cbuffer,opcode,address);
  cbuffer[5] = length;
  return;
}



void lr20xx_regmem_fill_cdata(uint8_t *cdata,uint32_t *data,uint8_t data_length)
{
  uint8_t *puVar1;
  uint8_t data_length_local;
  uint32_t *data_local;
  uint8_t *cdata_local_1;
  uint8_t *cdata_local;
  uint16_t index;
  
  for (index = 0; index < data_length; index += 1)
  {
    puVar1 = cdata + (uint)index * 4;
    *puVar1 = (uint8_t)(data[index] >> 0x18);
    puVar1[1] = (uint8_t)(data[index] >> 0x10);
    puVar1[2] = (uint8_t)(data[index] >> 8);
    puVar1[3] = (uint8_t)data[index];
  }
  return;
}



void lr20xx_regmem_fill_cbuffer_cdata_opcode_address_data
               (uint8_t *cbuffer,uint8_t *cdata,uint16_t opcode,uint32_t address,uint32_t *data,
               uint8_t data_length)
{
  uint32_t address_local;
  uint16_t opcode_local;
  uint8_t *cdata_local;
  uint8_t *cbuffer_local;
  
  lr20xx_regmem_fill_cbuffer_opcode_address(cbuffer,opcode,address);
  lr20xx_regmem_fill_cdata(cdata,data,data_length);
  return;
}



void lr20xx_regmem_fill_out_buffer_from_raw_buffer
               (uint32_t *out_buffer,uint8_t *raw_buffer,uint8_t out_buffer_length)
{
  byte *pbVar1;
  uint8_t out_buffer_length_local;
  uint8_t *raw_buffer_local_1;
  uint32_t *out_buffer_local;
  uint8_t *raw_buffer_local;
  uint8_t out_index;
  
  for (out_index = '\0'; out_index < out_buffer_length; out_index += '\x01')
  {
    pbVar1 = raw_buffer + (uint)out_index * 4;
    out_buffer[out_index] =
         (uint)*pbVar1 * 0x1000000 + (uint)pbVar1[1] * 0x10000 + (uint)pbVar1[2] * 0x100 +
         (uint)pbVar1[3];
  }
  return;
}
