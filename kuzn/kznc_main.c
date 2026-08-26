void main(void)

{
  byte *pbVar1;
  byte bVar2;
  uint8_t uVar3;
  uint16_t uVar4;
  int extraout_r0;
  undefined4 uVar5;
  int iVar6;
  undefined4 extraout_r2;
  uint8_t *puVar7;
  uint uVar8;
  undefined4 *puVar9;
  ushort *puVar11;
  char *pcVar12;
  int iVar13;
  uint uVar14;
  undefined2 *puVar15;
  undefined4 *puVar16;
  bool bVar17;
  bool bVar18;
  undefined8 uVar19;
  undefined8 uVar20_00;
  lptimer_init_t init;
  char *local_44;
  char *local_40;
  undefined2 *local_38;
  undefined4 local_30;
  undefined2 local_2c;
  undefined4 *puVar10;
  
  *SCB_VTOR = &PTR_DAT_08004000;
  DAT_e000ed22 = 0xe0;
  iVar13 = 0;
  do
  {
    iVar6 = iVar13 + 1;
    (&DAT_e000e400)[iVar13] = 0xc0;
    iVar13 = iVar6;
  } while (iVar6 != 0x25);
  iVar13 = 0;
  DAT_e000ed23 = 0;
  rcc_enable_oscillator(RCC_OSC_XO32K,true);
  rcc_enable_oscillator(RCC_OSC_RCO32K,true);
  rcc_enable_oscillator(RCC_OSC_XO32M,true);
  rcc_set_iwdg_clk_source(1);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_IWDG,true);
  iwdg_stop();
  iwdg_init(true);
  iwdg_set_prescaler(0);
  iwdg_set_reload(0x3ff);
  iwdg_start();
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART1,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG,true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_TIMER0,true);
  SYSTICK_RVR = 23998;
  DAT_e000ed23 = 0xe0;
  SYSTICK_CVR = 0;
  SYSTICK_CSR = 7;
  uVar8 = GPIOA.OER;
  GPIOA.OER = uVar8 & 0xfffff6ff;
  uVar8 = GPIOA.IER;
  GPIOA.IER = uVar8 & 0xfffff6ff;
  uVar8 = GPIOA.OTYPER;
  GPIOA.OTYPER = uVar8 & 0xfffff6ff;
  uVar8 = GPIOA.ODR;
  GPIOA.ODR = uVar8 | 0x900;
  gpio_init((gpio_t *)&GPIOA,15,GPIO_MODE_INPUT_PULL_UP);
  gpio_init((gpio_t *)&GPIOA,14,GPIO_MODE_INPUT_PULL_UP);
  gpio_init((gpio_t *)&GPIOA,2,GPIO_MODE_OUTPUT_PP_HIGH);
  DAT_200002c8 = 0;
  DAT_20000230 = 0;
  DAT_200002c6 = 120;
  DAT_200002c7 = 120;
  DAT_200002c9 = 0;
  DAT_200002f6 = 0;
  DAT_200002f4 = 120;
  DAT_200002f5 = 120;
  DAT_200002f7 = 0;
  DAT_200003b7 = 0;
  DAT_200003b5 = 120;
  DAT_200003b6 = 120;
  DAT_200003b8 = 0;
  uarts_init(0);
  DAT_20000320 = &PARAM_ADDR;
  local_44 = &DAT_200002f4;
  local_40 = &DAT_200003b5;
  uVar4 = 0xffff;
  do
  {
    pbVar1 = &PARAM_ADDR + iVar13;
    iVar13 = iVar13 + 1;
    uVar4 = crc16(uVar4,*pbVar1);
  } while (iVar13 != 0x3e);
  if (param_crc == uVar4)
  {
    DAT_200001b0 = 0xb377b76b;
    DAT_200001b4 = 0xb74c860c;
    DAT_200001b8 = 0xf43f41fa;
    DAT_200001bc = 0x653dbeae;
    StartFreq = DEFAULT_FREQ_L;
    DAT_200001c4 = 250000;
    DAT_200001c8 = 0x54f0c64;
    DAT_200001cc = 3;
    DAT_200001d0 = 0;
    DAT_200001d4 = 0;
    DAT_200001d8 = 0;
    DAT_200001dc = 0;
    DAT_200001e0 = 0;
    DAT_200001e4 = 0;
    DAT_200001e8 = 0;
    DAT_200001ec = 0x5a5f0000;
  }
  else
  {
    memset((uint8_t *)&DAT_200001b8,0,0x38);
    *(undefined4 *)(extraout_r0 + -8) = 0x3020100;
    DAT_200001b4 = 0x7060504;
    StartFreq = DEFAULT_FREQ_H;
    DAT_200001c4 = 1000000;
    DAT_200001c8 = CONCAT13(DAT_200001c8._3_1_,0x17071e);
  }
  if (5 < (DAT_200001cc & 0xff))
  {
    DAT_200001cc = DAT_200001cc & 0xffffff00;
  }
  uVar3 = gpio_read((gpio_t *)&GPIOA,15);
  if (uVar3 == 0)
  {
    uVar8 = GPIOA.BRR;
    GPIOA.BRR = uVar8 | 0x800;
    _working_mode = 0;
  }
  DAT_200003b0 = (uint)(uVar3 == 0);
  DAT_2000039b = 1;
  DAT_20000290 = 0;
  SX126xInit();
  SX126xSetStandby(STDBY_RC);
  SX126xSetRegulatorMode(0);
  SX126xSetBufferBaseAddress(0,0);
  SX126xSetTxParams(0,RADIO_RAMP_200_US);
  SX126xSetDioIrqParams(0,0,0,0);
  DAT_e000e100 = 0x200000;
  DAT_e000e415 = 0x20;
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPTIMER0,false);
  rcc_rst_peripheral(RCC_PERIPHERAL_LPTIMER0,true);
  rcc_rst_peripheral(RCC_PERIPHERAL_LPTIMER0,false);
  rcc_set_lptimer0_clk_source(0x80);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPTIMER0,true);
  local_30 = 0;
  local_2c = 0;
  init._4_4_ = extraout_r2;
  init._0_4_ = &local_30;
  lptimer_init((lptimer_t *)&LPTIMER0,init);
  lptimer_config_interrupt((lptimer_t *)&LPTIMER0,LPTIMER_IT_CMPM,1);
  lptimer_cmd((lptimer_t *)&LPTIMER0,true);
  lptimer_set_arr_register((lptimer_t *)&LPTIMER0,0xffff);
  lptimer_set_cmp_register((lptimer_t *)&LPTIMER0,(&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8]);
  lptimer_config_count_mode((lptimer_t *)&LPTIMER0,LPTIMER_MODE_CNTSTRT,1);
  DAT_e000e100 = 0x20000000;
  DAT_e000e41d = 0x40;
  DAT_2000035c = (&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8];
  do
  {
    if (RadioRxFlag != 0)
    {
      RadioRxFlag = 0;
      if (RxPayloadLen == 9)
      {
        DAT_20000328._0_1_ = RxPayload[0];
        DAT_20000328._1_1_ = RxPayload[1];
        DAT_20000328._2_1_ = RxPayload[2];
        DAT_20000328._3_1_ = RxPayload[3];
        DAT_2000032c._0_1_ = RxPayload[4];
        DAT_2000032c._1_1_ = RxPayload[5];
        DAT_2000032c._2_1_ = RxPayload[6];
        DAT_2000032c._3_1_ = RxPayload[7];
        DAT_20000330 = RxPayload[8];
        if ((char)DAT_200001cc == 0)
        {
          FUN_08005a60(0x20,&DAT_20000328,&DAT_200001b0);
          iVar13 = 1;
          uVar4 = 0xffff;
          do
          {
            puVar7 = (uint8_t *)((int)&DAT_20000328 + iVar13);
            iVar13 = iVar13 + 1;
            uVar4 = crc16(uVar4,*puVar7);
          } while (iVar13 != 8);
          if (((DAT_20000328 & 0xff) == (uint)(byte)uVar4) && ((ushort)DAT_20000330 == uVar4 >> 8))
          {
            if (DAT_20000392 == 0) goto LAB_080063e4;
            DAT_200002f0 = DAT_200002f0 | 1;
            uVar8 = (uint)DAT_200001f0;
            if (DAT_200001f0 < DAT_20000228)
            {
              uVar8 = uVar8 + 0x10000;
            }
            iVar13 = FUN_08007144((int *)&DAT_20000308,
                                  ((uVar8 - DAT_20000228) -
                                  (int)*(short *)((DAT_200001cc & 0xff) * 0x10 + 0x8007b9c)) +
                                  (uint)(&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8]);
            DAT_20000198 = DAT_2000035c - (short)iVar13;
            if (DAT_2000032c._3_1_ == 240)
            {
              if (DAT_200001f4 == 0)
              {
                if (DAT_20000028 < 50)
                {
                  DAT_20000028 = DAT_20000028 + 3;
                }
              }
              else if (0 < DAT_20000028)
              {
                DAT_20000028 = DAT_20000028 + -1;
              }
            }
            else
            {
              if (DAT_20000028 < 50)
              {
                DAT_20000028 = DAT_20000028 + 2;
              }
              if (-1 < (int)((uint)DAT_2000032c._3_1_ << 0x18))
              {
                uVar8 = 4;
                puVar15 = &DAT_20000338;
                do
                {
                  uVar19 = FUN_0800572c(uVar8,10);
                  uVar8 = uVar8 + 10 & 0xff;
                  *puVar15 = (short)((int)(((uint)(ushort)uVar19 * 0xcce) / 0x3ff + 0x159) >> 1);
                  puVar15 = puVar15 + 1;
                } while (uVar8 != 0x2c);
                uVar19 = FUN_0800572c(0x2c,1);
                if ((char)uVar19 == 0)
                {
                  DAT_20000340 = 0xbf;
                }
                else
                {
                  DAT_20000340 = 0x700;
                }
                uVar19 = FUN_0800572c(1,3);
                if ((byte)uVar19 < 7)
                {
                  uVar20_00 = FUN_0800572c(0x2d,7);
                  (&DAT_20000338)[(byte)uVar19 + 5] =
                       (short)((int)((uint)(ushort)uVar20_00 * 0x641) >> 7) + 0xbf;
                }
                iVar13 = 0;
                puVar16 = &DAT_20000208;
                puVar15 = &DAT_20000338;
                do
                {
                  iVar13 = iVar13 + 1;
                  *(undefined2 *)puVar16 = *puVar15;
                  puVar16 = (undefined4 *)((int)puVar16 + 2);
                  puVar15 = puVar15 + 1;
                } while (iVar13 != 0x10);
                FUN_08005bf4();
                DAT_200001fc = system_msec;
                DAT_20000399 = 1;
                DAT_20000030 = 0;
                DAT_20000034 = 0;
                DAT_2000039b = 0;
              }
            }
            DAT_20000380 = DAT_20000384;
          }
        }
        else if (DAT_20000392 == 0)
        {
          FUN_08005a60(0x20,&DAT_20000328,&DAT_200001b0);
          iVar13 = 1;
          uVar4 = 0xffff;
          do
          {
            puVar7 = (uint8_t *)((int)&DAT_20000328 + iVar13);
            iVar13 = iVar13 + 1;
            uVar4 = crc16(uVar4,*puVar7);
          } while (iVar13 != 8);
          if (((DAT_20000328 & 0xff) == (uint)(byte)uVar4) && ((ushort)DAT_20000330 == uVar4 >> 8))
          {
LAB_080063e4:
            if (DAT_2000032c._3_1_ == -0x10)
            {
              disableIRQinterrupts();
              lptimer_set_cmp_register
                        ((lptimer_t *)&LPTIMER0,
                         *(short *)((DAT_200001cc & 0xff) * 0x10 + 0x8007b9c) + DAT_20000228);
              lptimer_clear_interrupt((lptimer_t *)&LPTIMER0,LPTIMER_IT_CMPM);
              enableIRQinterrupts();
              DAT_2000035c = (&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8];
              uVar19 = FUN_0800572c(8,0x20);
              uVar8 = (uint)uVar19;
              uVar14 = uVar8 ^ uVar8 >> 0x10;
              if (((DAT_20000328 >> 0x10 & 0xff) == (uVar14 & 0xffff) >> 8) &&
                 ((DAT_20000328 >> 8 & 0xff) == (uVar14 & 0xff)))
              {
                if (DAT_20000020 << 0x1f < 0)
                {
                  bVar2 = DAT_200001c8._2_1_;
                }
                else
                {
                  bVar2 = DAT_200001c8._1_1_;
                }
                DAT_200002c0 = (uint)bVar2;
                DAT_20000198 = 0;
                DAT_20000392 = 1;
                DAT_20000280 = uVar8;
                FUN_0800713a((int)&DAT_20000308,2,5);
                DAT_2000035c = (&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8];
                DAT_200001f4 = 0;
                DAT_20000028 = 3;
                DAT_200002f0 = 1;
              }
            }
          }
        }
        else if (DAT_200001f4 == 0)
        {
          if (0 < DAT_20000028)
          {
            DAT_20000028 = DAT_20000028 + -1;
          }
          iVar13 = 1;
          FUN_08005a60(0x20,&DAT_20000328,&DAT_200001b0);
          uVar4 = 0xffff;
          do
          {
            puVar7 = (uint8_t *)((int)&DAT_20000328 + iVar13);
            iVar13 = iVar13 + 1;
            uVar4 = crc16(uVar4,*puVar7);
          } while (iVar13 != 8);
          if ((((DAT_20000328 & 0xff) == (uint)(byte)uVar4) && ((ushort)DAT_20000330 == uVar4 >> 8))
             && (DAT_2000032c._3_1_ == -0x10))
          {
            FUN_08006fbc(0x53);
            DAT_200002f0 = DAT_200002f0 | 1;
            if (DAT_20000028 < 50)
            {
              DAT_20000028 = DAT_20000028 + 3;
            }
          }
        }
        else
        {
          DAT_20000328 = RxPayload._0_4_ ^ DAT_20000280;
          DAT_2000032c = RxPayload._4_4_ ^ DAT_20000280;
          iVar13 = 1;
          FUN_08005a60(0x20,&DAT_20000328,&DAT_200001b0);
          uVar4 = 0xffff;
          do
          {
            puVar7 = (uint8_t *)((int)&DAT_20000328 + iVar13);
            iVar13 = iVar13 + 1;
            uVar4 = crc16(uVar4,*puVar7);
          } while (iVar13 != 8);
          if (((DAT_20000328 & 0xff) == (uint)(byte)uVar4) && ((ushort)DAT_20000330 == uVar4 >> 8))
          {
            DAT_200002f0 = DAT_200002f0 | 1;
            puVar11 = &DAT_200001f0;
            DAT_2000031c = (uint)DAT_200001f0;
            if (DAT_200001f0 < DAT_20000228)
            {
              puVar11 = (ushort *)(uint)DAT_200001f0;
            }
            if (DAT_200001f0 < DAT_20000228)
            {
              DAT_2000031c = ((uint)puVar11 & 0xffff) + 0x10000;
            }
            DAT_2000031c = ((DAT_2000031c - DAT_20000228) -
                           (int)*(short *)((DAT_200001cc & 0xff) * 0x10 + 0x8007b9c)) +
                           (uint)(&WORD_08007b9e)[(DAT_200001cc & 0xff) * 8];
            iVar13 = FUN_08007144((int *)&DAT_20000308,DAT_2000031c);
            DAT_20000198 = DAT_2000035c - (short)iVar13;
            if (DAT_20000028 < 0x32)
            {
              DAT_20000028 = DAT_20000028 + 2;
            }
            if (-1 < (int)DAT_2000032c)
            {
              local_38 = &DAT_20000338;
              uVar8 = 4;
              puVar15 = &DAT_20000338;
              do
              {
                uVar19 = FUN_0800572c(uVar8,10);
                uVar8 = uVar8 + 10 & 0xff;
                *puVar15 = (short)((int)(((uint)(ushort)uVar19 * 0xcce) / 0x3ff + 0x159) >> 1);
                puVar15 = puVar15 + 1;
              } while (uVar8 != 0x2c);
              uVar19 = FUN_0800572c(0x2c,1);
              if ((char)uVar19 == 0)
              {
                DAT_20000340 = 0xbf;
              }
              else
              {
                DAT_20000340 = 0x700;
              }
              uVar19 = FUN_0800572c(1,3);
              uVar8 = (uint)(byte)uVar19;
              if (uVar8 == 6)
              {
                uVar19 = FUN_0800572c(0x2d,7);
                DAT_200002c4 = DAT_200002c4 & 0x7f | (short)uVar19 << 7;
              }
              else if (uVar8 == 7)
              {
                uVar19 = FUN_0800572c(0x2d,7);
                uVar14 = (uint)uVar19 & 0xffff | DAT_200002c4 & 0xffffff80;
                DAT_200002c4 = (ushort)uVar14;
                uVar8 = FUN_08005c28(uVar14 >> 3);
                if (uVar14 == uVar8)
                {
                  DAT_2000034e = (undefined2)(uVar14 >> 3);
                }
              }
              else
              {
                uVar19 = FUN_0800572c(0x2d,7);
                (&DAT_20000338)[uVar8 + 5] = (short)((int)((uint)(ushort)uVar19 * 1601) >> 7) + 0xbf
                ;
              }
              iVar13 = 0;
              puVar16 = &DAT_20000208;
              do
              {
                iVar13 = iVar13 + 1;
                *(undefined2 *)puVar16 = *local_38;
                puVar16 = (undefined4 *)((int)puVar16 + 2);
                local_38 = local_38 + 1;
              } while (iVar13 != 0x10);
              FUN_08005bf4();
              DAT_200001fc = system_msec;
              DAT_20000399 = 1;
              DAT_20000030 = 0;
              DAT_20000034 = 0;
              DAT_2000039b = 0;
            }
          }
        }
        DAT_200002c6 = -PacketStatus.LoRaFreqError._2_1_;
        DAT_200002f7 = PacketStatus.LoRaFreqError._1_1_;
        DAT_200002f4 = DAT_200002c6;
        if (DAT_20000230 == 1)
        {
          DAT_20000222 = 0xad;
          DAT_20000224 = 0xad;
          DAT_20000226 = (short)(((uint)DAT_200002f6 * 1640) / 100) + 0xad;
        }
      }
      else if (DAT_200003b0 != 0)
      {
        if (_working_mode == 0)
        {
          if ((RxPayload[0] == 0xaa) && (RxPayload[1] == 0x55))
          {
            puVar16 = &DAT_200001b0;
            puVar7 = RxPayload + 2;
            do
            {
              uVar5 = *(undefined4 *)(puVar7 + 4);
              *puVar16 = *(undefined4 *)puVar7;
              puVar16[1] = uVar5;
              puVar16 = puVar16 + 2;
              puVar7 = puVar7 + 8;
            } while (puVar7 != RxPayload + 66);
            iVar13 = 0;
            uVar4 = 0xffff;
            puVar7 = (uint8_t *)&DAT_200001b0;
            do
            {
              iVar13 = iVar13 + 1;
              uVar4 = crc16(uVar4,*puVar7);
              puVar7 = puVar7 + 1;
            } while (iVar13 != 30);
            if (DAT_200001cc._2_2_ == uVar4)
            {
              uVar4 = 0xffff;
              iVar13 = 0;
              puVar7 = (uint8_t *)&DAT_200001b0;
              do
              {
                iVar13 = iVar13 + 1;
                uVar4 = crc16(uVar4,*puVar7);
                puVar7 = puVar7 + 1;
              } while (iVar13 != 0x3e);
              DAT_200001ec = CONCAT22(uVar4,(undefined2)DAT_200001ec);
            }
            else
            {
              FUN_08005a60(0x20,&DAT_200001b0,(int *)&DAT_08007c04);
              FUN_08005a60(0x20,&DAT_200001b8,(int *)&DAT_08007c04);
              FUN_08005a60(0x20,&StartFreq,(int *)&DAT_08007c04);
              FUN_08005a60(0x20,&DAT_200001c8,(int *)&DAT_08007c04);
              uVar4 = 0xffff;
              iVar13 = 0;
              puVar7 = (uint8_t *)&DAT_200001b0;
              do
              {
                iVar13 = iVar13 + 1;
                uVar4 = crc16(uVar4,*puVar7);
                puVar7 = puVar7 + 1;
              } while (iVar13 != 0x3e);
              if (DAT_200001ec._2_2_ != uVar4) goto LAB_0800603a;
            }
            flash_erase_page(0x8010000);
            flash_program_bytes(0x8010000,(uint8_t *)&DAT_200001b0,0x40);
            _working_mode = 1;
          }
        }
        else if (_working_mode == 1)
        {
          DataSynchronizationBarrier(0xf);
          DataSynchronizationBarrier(0xf);
          do
          {
                    // WARNING: Do nothing block with infinite loop
          } while( true );
        }
      }
LAB_0800603a:
      DAT_20000290 = 2;
    }
    if (DAT_2000039c != 0)
    {
      DAT_2000039c = 0;
      DAT_200003a4 = system_msec;
      if (DAT_200001fc + (ushort)((ushort)UART_clk_freqs[(DAT_200001cc & 0xff) * 4 + 3] >> 1) <
          system_msec)
      {
        puVar16 = &DAT_20000208;
        puVar9 = &DAT_200003ce;
        do
        {
          puVar10 = puVar9 + 1;
          *puVar16 = *puVar9;
          puVar16 = puVar16 + 1;
          puVar9 = puVar10;
        } while (puVar10 != (undefined4 *)0x200003ee);
        DAT_20000399 = 1;
        DAT_20000030 = 0;
        DAT_20000034 = 0;
      }
    }
    if (DAT_2000039d != 0)
    {
      DAT_2000039d = 0;
      DAT_2000022c = system_msec;
    }
    if (DAT_20000230 == 0)
    {
      uVar8 = (uint)DAT_20000399;
      if (uVar8 == 0)
      {
        if ((DAT_20000397 != 0) && (2 < (int)DAT_20000324))
        {
          DAT_20000397 = DAT_20000399;
          DAT_20000324 = uVar8;
          if (DAT_20000396 == 0)
          {
            if (DAT_2000022c + 300 < system_msec)
            {
              DAT_200003b7 = 0;
              DAT_200003b5 = 120;
              DAT_200003b6 = 120;
              DAT_200003b8 = 0;
            }
            if (DAT_20000274._1_1_ < (byte)DAT_20000278)
            {
              DAT_200002c6 = DAT_200003b5;
              DAT_200002c9 = DAT_200003b8;
              pcVar12 = local_40;
            }
            else
            {
              DAT_200002c6 = DAT_200002f4;
              DAT_200002c9 = DAT_200002f7;
              pcVar12 = local_44;
            }
            DAT_200002c8 = pcVar12[2];
            FUN_08006db0();
          }
          else
          {
            DAT_20000396 = DAT_20000399;
            DAT_20000274._0_1_ = DAT_200002f4;
            DAT_20000274._1_1_ = DAT_200002f6;
            DAT_20000274._2_1_ = DAT_200002f7;
            DAT_20000274._3_1_ = DAT_200003b5;
            DAT_20000278 = (uint)CONCAT11(DAT_200003b8,DAT_200003b7);
            DAT_2000027c = uVar8;
            FUN_08006e14();
          }
        }
      }
      else if (2 < (int)DAT_20000324)
      {
        DAT_20000399 = 0;
        DAT_20000324 = DAT_20000230;
        FUN_08006ca0((ushort *)&DAT_20000208);
      }
    }
    else if ((DAT_20000230 == 1) && (DAT_20000399 != 0))
    {
      DAT_20000399 = 0;
      FUN_08006fd4((ushort *)&DAT_20000208);
    }
    if (RxFlag != 0)
    {
      RxFlag = 0;
      FUN_08004cd8();
      DAT_20000300 = DAT_20000300 + 1;
      if (1000 < DAT_20000300)
      {
        DAT_20000300 = 0;
        DAT_20000396 = 1;
      }
      DAT_20000324 = DAT_20000324 + 1;
      if (DAT_20000230 == 0)
      {
        DAT_20000034 = DAT_20000034 + 1;
        DAT_20000030 = DAT_20000030 + 1;
        if (0x31 < DAT_20000034)
        {
          if (DAT_20000030 < 2000)
          {
            DAT_20000399 = 1;
          }
          else
          {
            DAT_20000397 = 1;
            DAT_2000039b = 1;
            DAT_200002c8 = 0;
          }
          DAT_20000034 = 0;
        }
        DAT_200001f8 = DAT_200001f8 + 1;
        if (100 < DAT_200001f8)
        {
          DAT_200001f8 = 0;
          DAT_20000397 = 1;
        }
      }
      else if (DAT_20000230 == 1)
      {
        DAT_20000030 = DAT_20000030 + 1;
        if (2000 < DAT_20000030)
        {
          DAT_200002c8 = 0;
          DAT_20000226 = 0xad;
          DAT_2000039b = 1;
        }
        DAT_20000034 = DAT_20000034 + 1;
        if (0x14 < DAT_20000034)
        {
          DAT_20000034 = 0;
          DAT_20000226 = (short)(((uint)DAT_200002c8 * 0x668) / 100) + 0xad;
          DAT_20000399 = 1;
        }
      }
      if (DAT_20000395 == 0)
      {
        if (DAT_20000038 < 100)
        {
          DAT_20000038 = DAT_20000038 + 1;
        }
        else
        {
          if (DAT_200003b0 != 0)
          {
            DAT_200001cc = DAT_200001cc & 0xffffff00;
          }
          Radio_Init(DAT_200003b0);
          DAT_200002ca = 0;
          DAT_200002cb = 0;
          DAT_200002cc = 6;
          DAT_20000395 = 1;
        }
      }
      if (DAT_200003b0 == 0)
      {
        if (DAT_2000039b != 0) goto LAB_08006114;
        uVar8 = GPIOA.BRR;
        GPIOA.BRR = uVar8 | 0x100;
      }
      else
      {
        if (_working_mode == 0)
        {
LAB_08006114:
          bVar18 = SBORROW4(DAT_20000304 + 1,50);
          iVar13 = DAT_20000304 + -49;
          bVar17 = DAT_20000304 + 1 == 50;
        }
        else
        {
          bVar18 = SBORROW4(DAT_20000304 + 1,1000);
          iVar13 = DAT_20000304 + -999;
          bVar17 = DAT_20000304 + 1 == 1000;
        }
        DAT_20000304 = DAT_20000304 + 1;
        if (!bVar17 && iVar13 < 0 == bVar18)
        {
          DAT_20000304 = 0;
          gpio_toggle((gpio_t *)&GPIOA,8);
        }
      }
    }
    if (DAT_20000398 != 0)
    {
      DAT_20000398 = 0;
      FUN_0800694c();
    }
  } while( true );
}






