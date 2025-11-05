#include "radio_func.h"

//globals
int8_t minpower;
int8_t maxpower;

//buffers
uint8_t radio_txbuffer[RADIO_TXBUF_SIZE];
uint8_t txlen;
uint8_t radio_rxbuffer[RADIO_RXBUF_SIZE];
uint8_t rxlen;

bool packet_sent = false;
bool packet_received = false;
bool crc_error = false; 

//radio configuration structure
radioconfig_t radioconfig;
rxpacketstatus_t pktstatus;

//init radio
int8_t radio_initconfig(uint16_t chip,uint8_t tcxo)
{
  switch(chip)
  {
    case 1262:
    radioconfig.chip = 1262;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LORA_SF_11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LORA_CR_4_8;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x24b4;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
		if(tcxo) 
		{
			sx126x_tcxo = 1;
			sx126x_tcxo_voltage = 2; //1.8V
		}
		else sx126x_tcxo = 0;
    //return RADIO_OK;
		break;
		
		case 1280:
    radioconfig.chip = 1280;
    radioconfig.id = 0;
    radioconfig.freq = 2400000000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LORA_SF_11;
    radioconfig.bw_index = 0; //SX128X_LORA_BW_200
    radioconfig.cr = LORA_CR_4_8;
    radioconfig.ldropt = 0; //not used in 1280
    //packet
    radioconfig.sync = 0x24b4;
    radioconfig.prelen = 0x23; //len = 12. mant = 3, exp = 2
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
//		if(tcxo) 
//		{
//			sx128x_tcxo = 1;
//			sx128x_tcxo_voltage = 2; //1.8V
//		}
//		else sx128x_tcxo = 0;
		//return RADIO_OK;
		break;

    case 1121:
    radioconfig.chip = 1121;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LORA_SF_11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LORA_CR_4_8;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x24b4;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
		if(tcxo) 
		{
			lr112x_tcxo = 1;
			lr112x_tcxo_voltage = 2; //1.8V
		}
		else lr112x_tcxo = 0;
    //return RADIO_OK;
		break;
		
    case 3029:
    radioconfig.chip = 3029;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LORA_SF_11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LORA_CR_4_8;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x24b4; //0x2b
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 255; //???
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
//		if(tcxo) 
//		{
//			sx126x_tcxo = 1;
//			sx126x_tcxo_voltage = 2; //1.8V
//		}
//		else sx126x_tcxo = 0;
    //return RADIO_OK;
		break;
    
    default:
    return INVALID_CHIP;
  }
	writeconfig();
	radio_init();
	return RADIO_OK;
}

int8_t radio_init(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
    SX126X_reset();
    SX126X_Wakeup();
    delay_ms(10);
    SX126X_setopmode(RADIO_OPMODE_STBYRC);
    SX126X_SetRegulatorMode(true);
    //set TCXO here if needed
		if(sx126x_tcxo != 0)
		{
			SX126X_SetDIO3AsTCXOCtrl(sx126x_tcxo_voltage,1000);
		}
    SX126X_SetPacketType(SX126X_MODEM_LORA);
    SX126X_SetRfFrequency((uint32_t)(radioconfig.freq / SX126X_SYNTH_STEP));
    SX126X_SetBufferBaseAddress(0,0);
    SX126X_SetLoRaModParams(SX126X_bw[radioconfig.bw_index],radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
    SX126X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.header,radioconfig.paylen,radioconfig.crc,radioconfig.invertiq);
    SX126X_writeReg(SX126X_REG_LRSYNC_H,(radioconfig.sync & 0xff00) >> 8);
    SX126X_writeReg(SX126X_REG_LRSYNC_L,radioconfig.sync & 0xff);
    SX126X_SetRx(0xffffff);
    //SX126X_setopmode(OPMODE_STBYXOSC);
    SX126X_CalibrateIR();
		if(sx126x_tcxo == 0) //TCXO off
		{
			SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
			SX126X_writeReg(SX126X_REG_XTATRIM,sx126x_xtatrim);
			SX126X_writeReg(SX126X_REG_XTBTRIM,sx126x_xtbtrim);
		}
    SX126X_SetTxParams();
    SX126X_LNAboost(true);
    SX126X_SetDIO2AsRfSwitchCtrl(true);
    SX126X_SetDioIrqParams(SX126X_TXDONE_IRQMSK | SX126X_RPEDET_IRQMSK | SX126X_SYNCDET_IRQMSK | SX126X_RXDONE_IRQMSK | SX126X_CRCERR_IRQMSK, SX126X_TXDONE_IRQMSK | SX126X_RPEDET_IRQMSK | SX126X_SYNCDET_IRQMSK | SX126X_RXDONE_IRQMSK, 0, 0);
    SX126X_setopmode(RADIO_OPMODE_RX);
    return RADIO_OK;
		
		case 1280:
		{
			//SX128X_setopmode(SX128X_OPMODE_STBYRC);
			SX128X_SetRegulatorMode(1);
			SX128X_SetPacketType(SX128X_MODEM_LORA);
			uint8_t buf[2];
			uint8_t header;
			//uint8_t prelen;
			uint8_t crc;
			uint8_t invertiq;
			//uint8_t tmp;

			SX128X_LORA_SetModulationParams(radioconfig.sf << 4,SX128X_bw[radioconfig.bw_index],radioconfig.cr); //index!!!
			//calculate preamble length patameter from config->loraprelen
			if(radioconfig.header == false) header = SX128X_HEADER_EXPLICIT;
			else header = SX128X_HEADER_IMPLICIT;
			if(radioconfig.crc == true) crc = SX128X_CRC_ENABLE;
			else crc = SX128X_CRC_DISABLE;
			if(radioconfig.invertiq == true) invertiq = SX128X_IQ_INVERTED;
			else invertiq = SX128X_IQ_STD;
			SX128X_LORA_SetPacketParams(radioconfig.prelen, header,radioconfig.paylen,crc,invertiq); //check!!!
			//set sync
			//It is also possible to configure the LoRa SyncWord. With the 1 byte SynchWord taking the format 0xXY, it is written across the MSB of two registers as described below:
			//write X in the register@0x944, in position [7:4] without modifying [3:0] (using a Read / Modify / Write operation)
			//write Y in the register@0x945, in position [7:4] without modifying [3:0] (using a Read / Modify / Write operation
			//SX128X_readRegs(sx,SX128X_REG_LR_SYNC_1,buf,2);
			//buf[0] &= 0x0f;
			//buf[1] &= 0x0f;
			//tmp = config->lorasync & 0xf0;
			//buf[0] |= tmp;
			//tmp = (config->lorasync & 0x0f) << 4;
			//buf[1] |= tmp;
			buf[0] = radioconfig.sync >> 8;
			buf[1] = radioconfig.sync & 0xff;
			SX128X_writeRegs(SX128X_REG_LR_SYNC_1, buf,2);
			SX128X_SetRfFrequency(radioconfig.freq / SX128X_SYNTH_STEP);
			SX128X_SetBufferBaseAddress(0, 0x00);
			SX128X_SetTxParams(radioconfig.txpower + 18,0); //set correct ramptime here
			//SX128X_CalibrateIR();
			//SX128X_LNAboost(true); //???
			SX128X_SetDioIrqParams(SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE | SX128X_IRQ_CRC_ERROR,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE,0,0);
			return RADIO_OK;
		}
		
		case 1121:
		{
			LR112X_reset();
			delay_ms(500);
			LR112X_Wakeup();
			//delay_ms(500);
			LR112X_SetRegMode(false);
			//LR112X_printstatus();
			//LR112X_SetStandby(0);
			//disable SPI CRC
			//LR112X_EnableSpiCrc(false);
			//LR112X_printstatus();
			LR112X_SetDioAsRfSwitch(LR112X_RFSW_ENABLE,LR112X_RFSW_STBY,LR112X_RFSW_RX,LR112X_RFSW_SUBG_TX,LR112X_RFSW_SUBG_TX_HP,LR112X_RFSW_HF_TX,LR112X_RFSW_GNSS,LR112X_RFSW_HF_RX); //for E80 - different
			//printf("SW:en=%d,stby=%d,rx=%d,tx=%d,txhp=%d\r\n",LR112X_RFSW_ENABLE,LR112X_RFSW_STBY,LR112X_RFSW_RX,LR112X_RFSW_SUBG_TX,LR112X_RFSW_SUBG_TX_HP);
			LR112X_printstatus();
			LR112X_ClearErrors();
			if(lr112x_tcxo) LR112X_SetTcxoMode(TCXO_1V8,320);
			delay_ms(10);
			//LR112X_ConfigLfClock(LR112X_LF_CLK_XOSC,true);
			LR112X_ConfigLfClock(LR112X_LF_CLK_RC,true);
			LR112X_ClearIrq(LR112X_IRQMASK_ALL);
			LR112X_SetStandby(1);
			delay_ms(10);
			LR112X_ClearErrors();
			LR112X_Calibrate(0x3f);
			LR112X_ClearErrors();
			LR112X_ClearIrq(LR112X_IRQMASK_ALL);
			//lr11xx_system_get_version(NULL, &version);
			LR112X_SetPacketType(LR112X_PACKET_TYPE_LORA);
			LR112X_SetLoRaModulationParams(radioconfig.sf,LR112X_bw[radioconfig.bw_index],radioconfig.cr,radioconfig.ldropt);
			LR112X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.header,radioconfig.paylen,radioconfig.crc,radioconfig.invertiq);
			LR112X_SetLoRaSyncWord(radioconfig.sync);
			LR112X_SetRfFrequency(radioconfig.freq);
			//if(radioconfig.freq > LR112X_SEPARATION_FREQ) LR112X_SetPaConfig(LR11XX_PA_SEL_HF,LR11XX_PA_REG_SUPPLY_VREG,LR11XX_PA_DUTYCYCLE_HF,LR11XX_PA_HPSEL_HF);
			//else LR112X_SetPaConfig(LR11XX_PA_SEL_SUBG_HP,LR11XX_PA_REG_SUPPLY_VBAT,LR11XX_PA_DUTYCYCLE_SUBG,LR11XX_PA_HPSEL_SUBG);
			radio_set_power(radioconfig.txpower);
			//LR112X_SetTxParams(radioconfig.txpower,LR112X_PA_RAMP_48U);
			//calibrate image here
			//calibrate RSSI
			LR112X_RssiCal(radioconfig.freq);
			LR112X_SetDioIrqParams(LR112X_TX_DONE | LR112X_RX_DONE | LR112X_CRC_ERROR,0);
			LR112X_SetRxTxFallbackMode(LR112X_FALLBACK_STBY_XOSC);
			LR112X_SetRxBoosted(false);
			//LR112X_SetDioAsRfSwitch(LR112X_RFSW_ENABLE,LR112X_RFSW_STBY,LR112X_RFSW_RX,LR112X_RFSW_SUBG_TX,LR112X_RFSW_SUBG_TX_HP,LR112X_RFSW_HF_TX,LR112X_RFSW_GNSS,LR112X_RFSW_HF_RX); 
			radio_rx();
			//LR112X_printstatus();
			//The workaround is to set the bit 4 in the register 0x00F30024 when the chip ends a reception in the 2.4GHz band before launching a GNSS scan. - ???
			//LR112X_ReadRegMem32(lr,0x00f30024,buf,1);
			//buf[0] |= 0x10;
			//LR112X_WriteRegMem32(lr,0x00f30024,buf,1);
			return RADIO_OK;
		}
		
		case 3029:
		if(PAN_Init() != PAN_OK) return RADIO_INVALID_MODE; //in STB3 state
		//PAN_SetTxPower(radioconfig.txpower);                    /* Set the power level */
		PAN_setpower(radioconfig.txpower);
		PAN_SetFreq(radioconfig.freq);          /* Set the frequency */
		PAN_SetBW(PAN_bw[radioconfig.bw_index]);              /* Set the bandwidth */
		PAN_SetSF(radioconfig.sf);              /* Set the spreading factor */
		PAN_SetCR(radioconfig.cr);              /* Set the channel coding rate */
		PAN_SetCRC(radioconfig.crc);            /* Set the CRC check */
		PAN_SetLDR(radioconfig.ldropt);            /* Set the low-rate mode */
		PAN_SetPreamLen(radioconfig.prelen);  /* Set the preamble length */
		PAN_SetInvertIQ(radioconfig.invertiq); /* Set IQ to non-inverted */
		PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
		PAN_SetChipMode(CHIPMODE_MODE0);       /* Set the chip mode to MODE0 */
		//STB3 now
		radio_rx();
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
	}
}


//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
  switch(radioconfig.chip)
  {
    case 1262:
		radioconfig.freq = khz * 1000;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
    SX126X_SetRfFrequency((uint32_t)(radioconfig.freq / SX126X_SYNTH_STEP));
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
		case 1280:
		radioconfig.freq = khz * 1000;
		prevopmode = opmode;
		SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
		SX128X_SetRfFrequency((uint32_t)(radioconfig.freq / SX128X_SYNTH_STEP));
		SX128X_setopmode(prevopmode);
		return RADIO_OK;
		
		case 1121:
		radioconfig.freq = khz * 1000;
		prevopmode = opmode;
		LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
 		if(radioconfig.freq > LR112X_SEPARATION_FREQ)
		{
			LR112X_SetPaConfig(LR11XX_PA_SEL_HF,LR11XX_PA_REG_SUPPLY_VREG,LR11XX_PA_DUTYCYCLE_HF,LR11XX_PA_HPSEL_HF);
		}
		else
		{
			//LR112X_SetPaConfig(LR11XX_PA_SEL_SUBG_HP,LR11XX_PA_REG_SUPPLY_VBAT,LR11XX_PA_DUTYCYCLE_SUBG,LR11XX_PA_HPSEL_SUBG);
			LR112X_SetPaConfig(LR11XX_PA_SEL_SUBG_LP,LR11XX_PA_REG_SUPPLY_VREG,LR11XX_PA_DUTYCYCLE_SUBG,LR11XX_PA_HPSEL_SUBG);
		}
		LR112X_SetRfFrequency(radioconfig.freq);
		LR112X_setopmode(prevopmode);
		return RADIO_OK;
		
		case 3029:
		//prevopmode = opmode;
		if(PAN_SetFreq(radioconfig.freq) == PAN_FAIL) return RADIO_INVALID_PARAMETER; 
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
}

//set tx power
int8_t radio_set_power(int8_t dbm)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if((dbm < -9) || (dbm > 22)) return RADIO_INVALID_PARAMETER;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		radioconfig.txpower = dbm;
    SX126X_SetTxParams();
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
		case 1280:
		{
			int8_t pwr;
			if((dbm < -18) || (dbm > 13)) return RADIO_INVALID_PARAMETER;
			//prevopmode = opmode;
			//SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
			radioconfig.txpower = dbm;
			pwr = dbm + 18;
			printf("dbm=%d,pwr=%d\r\n",dbm,pwr);
			SX128X_SetTxParams(pwr,SX128X_RAMP_02_US); //temporary
			//SX128X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 1121:
		//prevopmode = opmode;
		//LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
 		if(radioconfig.freq > LR112X_SEPARATION_FREQ)
		{
			LR112X_SetPaConfig(LR11XX_PA_SEL_HF,LR11XX_PA_REG_SUPPLY_VREG,LR11XX_PA_DUTYCYCLE_HF,LR11XX_PA_HPSEL_HF);
			//check value
			if((dbm < -18) || (dbm > 13)) return RADIO_INVALID_PARAMETER;
			radioconfig.txpower = dbm;
			LR112X_SetTxParams(dbm,LR112X_PA_RAMP_48U);
			//LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		else
		{
			LR112X_SetPaConfig(LR11XX_PA_SEL_SUBG_HP,LR11XX_PA_REG_SUPPLY_VBAT,LR11XX_PA_DUTYCYCLE_SUBG,LR11XX_PA_HPSEL_SUBG);
			//check value
			if((dbm < -9) || (dbm > 22)) return RADIO_INVALID_PARAMETER;
			radioconfig.txpower = dbm;
			LR112X_SetTxParams(dbm,LR112X_PA_RAMP_48U);
			//LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 3029:
		if(dbm < 0) return RADIO_INVALID_PARAMETER;
		if(dbm > 20) return RADIO_INVALID_PARAMETER;
		radioconfig.txpower = dbm;
		PAN_setpower(dbm);
		return RADIO_OK;
	
    default:
    return INVALID_CHIP;
  }
}

//set modulation parameters
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t bw_index;
			//BW in kHz
			if((bw_khz < 7) || (bw_khz > 500)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 4) return RADIO_INVALID_PARAMETER;
			if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 8) bw_index = 0;
			else if(bw_khz <= 11) bw_index = 1;
			else if(bw_khz <= 16) bw_index = 2;
			else if(bw_khz <= 21) bw_index = 3;
			else if(bw_khz <= 32) bw_index = 4;
			else if(bw_khz <= 42) bw_index = 5;
			else if(bw_khz <= 63) bw_index = 6;
			else if(bw_khz <= 125) bw_index = 7;
			else if(bw_khz <= 250) bw_index = 8;
			else bw_index = 9;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
			SX126X_SetLoRaModParams(sf,SX126X_bw[bw_index],cr,ldropt); //
			SX126X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t bw_index;
			//BW in kHz
			if((bw_khz < 7) || (bw_khz > 500)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 8) return RADIO_INVALID_PARAMETER;
			if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 204) bw_index = 0;
			else if(bw_khz <= 407) bw_index = 1;
			else if(bw_khz <= 813) bw_index = 2;
			else bw_index = 3;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			//radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
			SX128X_LORA_SetModulationParams(sf,SX128X_bw[bw_index],cr); //
			SX128X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
    case 1121:
		{
			uint8_t bw_index;
			//BW in kHz
			if((bw_khz < 200) || (bw_khz > 1625)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 8) return RADIO_INVALID_PARAMETER;
			//if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 8) bw_index = 0;
			else if(bw_khz <= 11) bw_index = 1;
			else if(bw_khz <= 16) bw_index = 2;
			else if(bw_khz <= 21) bw_index = 3;
			else if(bw_khz <= 32) bw_index = 4;
			else if(bw_khz <= 42) bw_index = 5;
			else if(bw_khz <= 63) bw_index = 6;
			else if(bw_khz <= 125) bw_index = 7;
			else if(bw_khz <= 204) bw_index = 10;
			else if(bw_khz <= 251) bw_index = 8;
			else if(bw_khz <= 407) bw_index = 11;
			else if(bw_khz <= 501) bw_index = 9;
			else bw_index = 12;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
			LR112X_SetLoRaModulationParams(sf,LR112X_bw[bw_index],cr,(bool)ldropt); //
			LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_getmodparams(uint16_t *bw_khz,uint8_t *sf,uint8_t *cr,uint8_t *ldropt)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			*sf = radioconfig.sf;
			*bw_khz = SX126X_bw_kHz[radioconfig.bw_index];
			*cr = radioconfig.cr;
			*ldropt = radioconfig.ldropt;
			return RADIO_OK;
		}
		
    case 1280:
		{
			*sf = radioconfig.sf;
			*bw_khz = SX128X_bw_kHz[radioconfig.bw_index];
			*cr = radioconfig.cr;
			*ldropt = 0;
			return RADIO_OK;
		}
		
    case 1121:
		{
			*sf = radioconfig.sf;
			*bw_khz = LR112X_bw_kHz[radioconfig.bw_index];
			*cr = radioconfig.cr;
			*ldropt = radioconfig.ldropt;
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if(header > 1) return RADIO_INVALID_PARAMETER;
		if(crc > 1) return RADIO_INVALID_PARAMETER;
		if(invertiq > 1) return RADIO_INVALID_PARAMETER;
		radioconfig.sync = sync;
		radioconfig.prelen = prelen;
		radioconfig.paylen = paylen;
		radioconfig.header = header;
		radioconfig.crc = crc;
		radioconfig.invertiq = invertiq;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
    SX126X_SetLoRaPacketParams(prelen,paylen,(bool)header,(bool)crc,(bool)invertiq);
    SX126X_writeReg(SX126X_REG_LRSYNC_H,(sync & 0xff00) >> 8);
    SX126X_writeReg(SX126X_REG_LRSYNC_L,sync & 0xff);
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
    case 1280:
		if(header > 1) return RADIO_INVALID_PARAMETER;
		if(crc > 1) return RADIO_INVALID_PARAMETER;
		if(invertiq > 1) return RADIO_INVALID_PARAMETER;
		radioconfig.sync = sync;
		radioconfig.prelen = prelen;
		radioconfig.paylen = paylen;
		radioconfig.header = header;
		radioconfig.crc = crc;
		radioconfig.invertiq = invertiq;
		prevopmode = opmode;
		SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
    SX128X_LORA_SetPacketParams(prelen,header,paylen,crc,invertiq);
    SX128X_writeReg(SX128X_REG_LR_SYNC_1,(sync & 0xff00) >> 8);
    SX128X_writeReg(SX128X_REG_LR_SYNC_0,sync & 0xff);
		SX128X_setopmode(prevopmode);
    return RADIO_OK;
		
    case 1121:
		if(header > 1) return RADIO_INVALID_PARAMETER;
		if(crc > 1) return RADIO_INVALID_PARAMETER;
		if(invertiq > 1) return RADIO_INVALID_PARAMETER;
		radioconfig.sync = sync;
		radioconfig.prelen = prelen;
		radioconfig.paylen = paylen;
		radioconfig.header = header;
		radioconfig.crc = crc;
		radioconfig.invertiq = invertiq;
		prevopmode = opmode;
		LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
		LR112X_SetLoRaPacketParams(prelen,header,paylen,crc,invertiq);
		LR112X_SetLoRaSyncWord(radioconfig.sync);
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

//send one packet
int8_t radio_sendpacket(uint8_t *buf)
{
  switch(radioconfig.chip)
  {
    case 1262:
    SX126X_SetLoRaPacketParams(radioconfig.prelen,txlen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
    SX126X_writeBuffer(0,buf,txlen);
    SX126X_setopmode(RADIO_OPMODE_TX);
    return RADIO_OK;
		
    case 1280:
    SX128X_LORA_SetPacketParams(radioconfig.prelen,radioconfig.header,txlen,radioconfig.crc,radioconfig.invertiq);
    SX128X_writeBuffer(0,buf,txlen);
    SX126X_setopmode(RADIO_OPMODE_TX);
    return RADIO_OK;
		
    case 1121:
		LR112X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.header,txlen,radioconfig.crc,radioconfig.invertiq);
    LR112X_writeBuffer8(buf,txlen);
    LR112X_setopmode(RADIO_OPMODE_TX);
    return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}
//retrieve packet info
int8_t radio_getpktstatus(rxpacketstatus_t *status)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t dummy;
			uint8_t rssipkt;
			int16_t snrpkt;
			uint8_t signalrssi;
		
			SX126X_GetLoRaPacketStatus(&dummy,&rssipkt,&snrpkt,&signalrssi);
			status->rssi_pkt = -((float)rssipkt/2);
			if(snrpkt < 128) status->snr_pkt = ((float)snrpkt)/4;
			else status->snr_pkt = (float)(snrpkt - 256)/4;
			status->signal_rssi_pkt = -((float)signalrssi/2);
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t rssi_sync;
			int16_t snrpkt;

			SX128X_LORA_GetPacketStatus(&rssi_sync,&snrpkt);
			status->rssi_pkt = -((float)rssi_sync/2);
			if(snrpkt < 128) status->snr_pkt = ((float)snrpkt)/4;
			else status->snr_pkt = (float)(snrpkt - 256)/4;
			status->signal_rssi_pkt = status->rssi_pkt + status->snr_pkt; //???
			return RADIO_OK;
		}
		
    case 1121:
		{
			LR112X_GetLoRaPacketStatus(&status->rssi_pkt,&status->snr_pkt,&status->signal_rssi_pkt);
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;		
    
    default:
    return INVALID_CHIP;
  }
}

//receive one packet
int8_t radio_getpacket(uint8_t *buf)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t dummy;
			uint8_t rxpointer;
			SX126X_GetRxBufferStatus(&dummy,&rxlen,&rxpointer);
			SX126X_readBuffer(rxpointer,buf,rxlen);
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t rxpointer;
			SX128X_GetRxBufferStatus(&rxlen,&rxpointer);
			SX128X_readBuffer(rxpointer,buf,rxlen);
			return RADIO_OK;
		}
		
    case 1121:
		{
			uint8_t rxpointer;
			LR112X_GetRxBufferStatus(&rxlen,&rxpointer);
			LR112X_readBuffer8(rxpointer,buf,rxlen);
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;
   
    default:
    return INVALID_CHIP;
  }
}

//helpers
int8_t radio_getstats(rxstats_t *stats)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t dummy;
			SX126X_LoRaGetStats(&dummy,&stats->pkt_received,&stats->crc_error,&stats->header_error);
			stats->false_sync = 0;
		}
    return RADIO_OK;
		
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
		LR112X_GetStats(&stats->pkt_received,&stats->crc_error,&stats->header_error,&stats->false_sync);
    return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_clearstats(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
    SX126X_ResetStats();
    return RADIO_OK;
		
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
    LR112X_ResetStats();
    return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

//irq handler
void radio_irq_handler(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			//read SX126x status
			uint16_t irqstatus = SX126X_GetIrqStatus();
			SX126X_ClearIrqStatus(SX126X_ALL_IRQMSK);
			if(irqstatus & SX126X_TXDONE_IRQMSK) packet_sent = true;
			if(irqstatus & SX126X_RXDONE_IRQMSK) packet_received = true;
			if(irqstatus & SX126X_CRCERR_IRQMSK) crc_error = true;
//			if(irqstatus & SX126X_RPEDET_IRQMSK) {};
//			if(irqstatus & SX126X_SYNCDET_IRQMSK) {};
//			if(irqstatus & SX126X_HEADERDET_IRQMSK) {};
//			if(irqstatus & SX126X_HEADERERR_IRQMSK) {};
//			if(irqstatus & SX126X_CADDONE_IRQMSK) {};
//			if(irqstatus & SX126X_CADDET_IRQMSK) {};
//			if(irqstatus & SX126X_TIMEOUT_IRQMSK) {};
//			if(irqstatus & SX126X_LRFHSSHOP_IRQMSK) {};
			break;   
		}
		
    case 1280:
		{
			//read status
			uint16_t irqstatus = SX128X_GetIrqStatus();
			SX128X_ClearIrqStatus(SX128X_IRQ_ALL);
			if(irqstatus & SX128X_IRQ_TX_DONE) packet_sent = true;
			if(irqstatus & SX128X_IRQ_RX_DONE) packet_received = true;
			if(irqstatus & SX128X_IRQ_CRC_ERROR) crc_error = true;
//			if(irqstatus & SX128X_IRQ_SYNCWORD_VALID) {};
//			if(irqstatus & SX128X_IRQ_SYNCWORD_ERROR) {};
//			if(irqstatus & SX128X_IRQ_HEADER_VALID) {};
//			if(irqstatus & SX128X_IRQ_HEADER_ERROR) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) {};
//			if(irqstatus & SX128X_IRQ_RANGING_MASTER_RESULT_VALID) {};
//			if(irqstatus & SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID) {};
//			if(irqstatus & SX128X_IRQ_CAD_DONE) {};
//			if(irqstatus & SX128X_IRQ_CAD_ACTIVITY_DETECTED) {};
//			if(irqstatus & SX128X_IRQ_RX_TX_TIMEOUT) {};
//			if(irqstatus & SX128X_IRQ_PREAMBLE_DETECTED) {};
			break;   
		}
		
    case 1121:
		{
			//read status
			uint32_t irqstatus = SX126X_GetIrqStatus();
			LR112X_ClearIrq(LR112X_IRQMASK_ALL);
			if(irqstatus & LR112X_TX_DONE) packet_sent = true;
			if(irqstatus & LR112X_RX_DONE) packet_received = true;
			if(irqstatus & LR112X_CRC_ERROR) crc_error = true;
//			if(irqstatus & LR112X_PRE_DET) {};
//			if(irqstatus & LR112X_SYNC_DET) {};
//			if(irqstatus & LR112X_HEADER_ERROR) {};
//			if(irqstatus & LR112X_CAD_DONE) {};
//			if(irqstatus & LR112X_CAD_DETECTED) {};
//			if(irqstatus & LR112X_TIMEOUT) {};
//			if(irqstatus & LR112X_LR_FHSS_HOP) {};
//			if(irqstatus & LR112X_LBD) {};	
//			if(irqstatus & LR112X_CMD_ERROR) {};	
//			if(irqstatus & LR112X_ERROR) {};	
//			if(irqstatus & LR112X_FSK_LEN_ERROR) {};	
//			if(irqstatus & LR112X_FSK_ADDR_ERROR) {};		
//			if(irqstatus & LR112X_LR_RX_TIMESTAMP) {};				
			break;   
		}
		
		case 3029:
		break;
		
    default:
    break;
  }
}

//working modes
int8_t radio_rx(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
    SX126X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
    SX126X_setopmode(RADIO_OPMODE_RX);
		return RADIO_OK;
		
    case 1280:
		SX128X_LORA_SetPacketParams(radioconfig.prelen,radioconfig.header,radioconfig.paylen,radioconfig.crc,radioconfig.invertiq);
    SX128X_setopmode(RADIO_OPMODE_RX);
		return RADIO_OK;
		
    case 1121:
		LR112X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.header,radioconfig.paylen,radioconfig.crc,radioconfig.invertiq);
    LR112X_setopmode(RADIO_OPMODE_RX);
		return RADIO_OK;
		
		case 3029:
		PAN_EnterContinousRxState();
		return RADIO_OK;
		
		default:
    return INVALID_CHIP;
  }
}

int8_t radio_getrssi(float *dbm)
{
  switch(radioconfig.chip)
  {
    case 1262:
    *dbm = -((float)SX126X_GetRssiInst()/2);
    return RADIO_OK;
		
    case 1280:
    *dbm = -((float)SX128X_GetRssiInst()/2);
    return RADIO_OK;
		
    case 1121:
    *dbm = -((float)LR112X_GetRssiInst()/2);
    return RADIO_OK;
		
		case 3029:
		*dbm = (float)PAN_GetRealTimeRssi();
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_stream(uint8_t stream)
{
  if(stream == 0) 
	{
		txmode = 0;
		txled_off();
		if(radioconfig.chip == 3029) 
		{
			PAN_StopTxContinuousWave();
		}
		return radio_rx();
	}
  switch(radioconfig.chip)
  {
    case 1262:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			SX126X_SetCW();
			txmode = 1;
		}
    else 
		{
			SX126X_SetTxInfinitePreamble();
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
    case 1280:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			SX128X_SetCW();
			txmode = 1;
		}
    else 
		{
			SX128X_SetPRE();
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
    case 1121:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			LR112X_SetTxCw();
			txmode = 1;
		}
    else 
		{
			LR112X_SetTxInfinitePreamble();
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
		case 3029:
		if(stream == 1)
		{
			PAN_TurnonTxAnt();
			PAN_StartTxContinuousWave();
			txmode = 1;
			txled_on();
			return RADIO_OK;
		}
		else return RADIO_INVALID_PARAMETER;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_sleep(uint8_t node)
{
  switch(radioconfig.chip)
  {
    case 1262:
    return RADIO_TODO;
		
    case 1280:
    return RADIO_TODO;
		
    case 1121:
    return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_wakeup(uint8_t mode)
{
  switch(radioconfig.chip)
  {
    case 1262:
    return RADIO_TODO;
		
    case 1280:
    return RADIO_TODO;
		
    case 1121:
    return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_setxotrim(uint8_t trim)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if(trim > 94) return RADIO_INVALID_PARAMETER;
		sx126x_xtatrim = trim / 2;
		sx126x_xtbtrim = trim - sx126x_xtatrim;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		SX126X_writeReg(SX126X_REG_XTATRIM,sx126x_xtatrim);
		SX126X_writeReg(SX126X_REG_XTBTRIM,sx126x_xtbtrim);
		SX126X_setopmode(prevopmode);
		return RADIO_OK;
		
    case 1280: //0x093c
		return RADIO_TODO;
		
    case 1121:
    return FEATURE_NOT_SUPPORTED;
		
		case 3029:
		return RADIO_TODO;

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_getxotrim(uint8_t *trim)
{
  switch(radioconfig.chip)
  {
    case 1262:
		*trim = SX126X_readReg(SX126X_REG_XTATRIM) + SX126X_readReg(SX126X_REG_XTBTRIM);
		return RADIO_OK;
		
    case 1280: //0x093c
		return RADIO_TODO;
		
    case 1121:
    return FEATURE_NOT_SUPPORTED;
		
		case 3029:
		return RADIO_TODO;

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_readreg(uint32_t reg,uint32_t *val)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
		*val = SX126X_readReg(reg);
		return RADIO_OK;
		
    case 1280:
		if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
		*val = SX128X_readReg(reg);
		return RADIO_OK;
		
    case 1121:
		if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
		return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_writereg(uint32_t reg,uint32_t val)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
		if(val > 0xff) return RADIO_INVALID_PARAMETER;
		SX126X_writeReg(reg,val);
		return RADIO_OK;
		
    case 1280:
		if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
		if(val > 0xff) return RADIO_INVALID_PARAMETER;
		SX128X_writeReg(reg,val);
		return RADIO_OK;
		
    case 1121:
		if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
		return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_get_chip_version(uint8_t *hw,uint8_t *use_case,uint8_t *fw_major,uint8_t *fw_minor)
{
  switch(radioconfig.chip)
  {
    case 1262:
		return RADIO_TODO;
		
    case 1280:
		return RADIO_TODO;
		
    case 1121:
		LR112X_GetVersion(hw,use_case,fw_major,fw_minor);
		return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_get_status(uint8_t *stat1,uint8_t *stat2)
{
  switch(radioconfig.chip)
  {
    case 1262:
		return RADIO_TODO;
		
    case 1280:
		return RADIO_TODO;
		
    case 1121:
		LR112X_GetStatus(stat1,stat2);
		return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}

uint8_t radio_setopmode(uint8_t mode)
{
  switch(radioconfig.chip)
  {
    case 1262:
		return RADIO_TODO;
		
    case 1280:
		return RADIO_TODO;
		
    case 1121:
		LR112X_setopmode(mode);
		opmode = mode;
		return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}



