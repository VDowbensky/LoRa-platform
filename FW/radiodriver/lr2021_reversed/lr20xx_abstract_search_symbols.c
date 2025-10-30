

lr20xx_status_t abstract_search_symbols(void *context,uint8_t n_symbols,search_symbol_format_t format)
{
  uint8_t cbuffer [4];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = '\"';
  cbuffer[2] = n_symbols;
  cbuffer[3] = format;
  return lr20xx_hal_write(context,cbuffer,4,NULL,0);
}
