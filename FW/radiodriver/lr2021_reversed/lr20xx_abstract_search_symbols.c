

lr20xx_status_t
abstract_search_symbols(void *context,uint8_t n_symbols,search_symbol_format_t format)
{
  lr20xx_hal_status_t lVar1;
  search_symbol_format_t format_local;
  uint8_t n_symbols_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\"';
  cbuffer[2] = n_symbols;
  cbuffer[3] = format;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}
