#ifndef _SX1276_REGS_H_
#define _SX1276_REGS_H_


//////////////////////////////////////////////////
/************** Common **************************/
//////////////////////////////////////////////////

/******* FIFO *************/
#define REG_FIFO															0x00

/************** OPMODE ************/
#define REG_OPMODE														0x01
#define	OPMODE_LORA														0x80
#define	OPMODE_FSKOOK													0
#define	OPMODE_FSK														0
#define	OPMODE_OOK														0x20
#define	OPMODE_HF															0 //access to HF test registers
#define	OPMODE_LF															0x08 //access to LF test registers
#define OPMODE_MSK														0x07
#define	OPMODE_SLEEP													0
#define	OPMODE_STBY														0x01
#define	OPMODE_FSTX														0x02
#define	OPMODE_TX															0x03
#define	OPMODE_FSRX														0x04
//common
#define OPMODE_RX															0x05
//FSK specific
#define	FSK_OPMODE_RX													0x05
//LR specific
#define	LR_OPMODE_RXCONT											0x05
#define	LR_OPMODE_RXSINGLE										0x06
#define	LR_OPMODE_CAD													0x07
//artifical
#define	OPMODE_TXSTREAMCW											0x80
#define	OPMODE_TXSTREAMPRE										0x81

/******RF frequency *******/
#define	REG_FRF_MSB														0x06
#define	REG_FRF_MID														0x07
#define	REG_FRF_LSB														0x08

/******PA config *******/
#define	REG_PACONFIG													0x09
#define	PACONFIG_RFO													0
#define	PACONFIG_PABOOST											0x80
#define	PACONFIG_MAXPWR_MSK										0x70
#define	PACONFIG_PWR_MSK											0x0f

/*********PA RAMP *****************/
#define	REG_PARAMP														0x0a
//FSK-OOK mode
#define	PARAMP_FSK_SH_NONE										0
#define	PARAMP_FSK_SH_G1P0										0x20
#define	PARAMP_FSK_SH_G0P5										0x40
#define	PARAMP_FSK_SH_G0P3										0x60
#define	PARAMP_OOK_SH_NONE										0
#define	PARAMP_OOK_SH_BR											0x20
#define	PARAMP_OOK_SH_2BR											0x40
//Common
#define	PARAMP_MSK														0x0f
#define	PARAMP_3400														0
#define	PARAMP_2000														0x01
#define	PARAMP_1000														0x02
#define	PARAMP_500														0x03
#define	PARAMP_250														0x04
#define	PARAMP_125														0x05
#define	PARAMP_100														0x06
#define	PARAMP_62															0x07
#define	PARAMP_50															0x08
#define	PARAMP_40															0x09
#define	PARAMP_31															0x0a
#define	PARAMP_25															0x0b
#define	PARAMP_20															0x0c
#define	PARAMP_15															0x0d
#define	PARAMP_12															0x0e
#define	PARAMP_10															0x0f

/********* OCP ************/
#define	REG_OCP																0x0b
#define	OCP_OFF																0
#define	OCP_ON																0x20
#define	OCP_TRIM_MSK													0x1f
#define OCP_TRIM_45                           0x00
#define OCP_TRIM_50                           0x01   
#define OCP_TRIM_55                           0x02 
#define OCP_TRIM_60                           0x03 
#define OCP_TRIM_65                           0x04 
#define OCP_TRIM_70                           0x05 
#define OCP_TRIM_75                           0x06 
#define OCP_TRIM_80                           0x07  
#define OCP_TRIM_85                           0x08
#define OCP_TRIM_90                           0x09 
#define OCP_TRIM_95                           0x0A 
#define OCP_TRIM_100                          0x0B  // Default
#define OCP_TRIM_105                          0x0C 
#define OCP_TRIM_110                          0x0D 
#define OCP_TRIM_115                          0x0E 
#define OCP_TRIM_120                          0x0F 
#define OCP_TRIM_130                          0x10
#define OCP_TRIM_140                          0x11   
#define OCP_TRIM_150                          0x12 
#define OCP_TRIM_160                          0x13 
#define OCP_TRIM_170                          0x14 
#define OCP_TRIM_180                          0x15 
#define OCP_TRIM_190                          0x16 
#define OCP_TRIM_200                          0x17  
#define OCP_TRIM_210                          0x18
#define OCP_TRIM_220                          0x19 
#define OCP_TRIM_230                          0x1A 
#define OCP_TRIM_240                          0x1B


/********* LNA ************/
#define	REG_LNA																0x0c
#define	LNA_GAIN_MSK													0xe0
#define	LNA_G1																0x20
#define	LNA_G2																0x40
#define	LNA_G3																0x60
#define	LNA_G4																0x80
#define	LNA_G5																0xa0
#define	LNA_G6																0xc0
#define	LNA_BOOSTLF_MSK												0x18
#define	LNA_BOOSTHF_MSK												0x03
#define	LNA_BOOSTHF_OFF												0
#define	LNA_BOOSTHF_ON												0x03

/********** Chip version ************************/
#define	REG_VERSION														0x42

/********** Clock source ************************/
#define	REG_TCXO															0x4b
#define	TCXO_CRYSTAL													0x00
#define	TCXO_TCXO															0x10

/********** High power PA settings **************/
#define	REG_PADAC															0x4d
#define	PADAC_MSK															0x07 //default: 0x04, +20 dBm: 0x07

/********** Previous temperature ****************/
#define	REG_FORMERTEMP												0x5b

/********** AGC reference ***********************/
#define	REG_AGC_REF														0x61
#define	AGC_REF_LEVEL_MSK											0x3f //default: 0x19

/********** AGC thresholds **********************/
#define	REG_AGC_THRESH1												0x62
#define	AGC_THRESH1_STEP1_MSK									0x1f //default: 0x0c
#define	AGC_THRESH1_STEP1_SHIFT								0

#define	REG_AGC_THRESH2												0x63
#define	AGC_THRESH2_STEP2_MSK									0xf0 //default: 0x04
#define	AGC_THRESH2_STEP2_SHIFT								4
#define	AGC_THRESH2_STEP3_MSK									0x0f //default: 0x0b
#define	AGC_THRESH2_STEP3_SHIFT								0

#define	REG_AGC_THRESH3												0x64
#define	AGC_THRESH3_STEP4_MSK									0xf0 //default: 0x0c
#define	AGC_THRESH3_STEP4_SHIFT								4
#define	AGC_THRESH3_STEP5_MSK									0x0f //default: 0x0c
#define	AGC_THRESH3_STEP5_SHIFT								0

/********** PLL BW ******************************/
#define	REG_PLL																0x70
#define	PLL_BW_75															0x00
#define	PLL_BW_150														0x40
#define	PLL_BW_225														0x80
#define	PLL_BW_300														0xc0
#define	PLL_RESERVED_BITS											0x10

//////////////////////////////////////////////////
/************** DIO mapping *********************/
//////////////////////////////////////////////////

/********** DIO mapping 1 ***********************/
#define	REG_DIOMAPPING1												0x40 //TBD
#define	DIOMAPPING1_DIO0_MSK									0xc0
#define	DIOMAPPING1_DIO1_MSK									0x30
#define	DIOMAPPING1_DIO2_MSK									0x0c
#define	DIOMAPPING1_DIO3_MSK									0x03

//LoRa mode
#define	DIO0_RXDONE														0x00
#define	DIO0_TXDONE														0x40
#define	DIO0_CADDONE													0x80

#define	DIO1_RXTO															0x00
#define	DIO1_FHSS_CHG_CH											0x10
#define	DIO1_CADDET														0x20

#define	DIO2_FHSS_CHG_CH0											0x00
#define	DIO2_FHSS_CHG_CH1											0x04
#define	DIO2_FHSS_CHG_CH2											0x08

#define	DIO3_CADDONE													0x00
#define	DIO3_VALID_HEADER											0x01
#define	DIO3_CRCERROR													0x02					

//continuous mode
#define	DIO0_SYNC_TXRDY												0x00
#define	DIO0_RSSI_PREDET											0x40
#define	DIO0_RXRDY_TXRDY											0x80

#define	DIO1_DCLK															0x00
#define	DIO1_RSSI_PREDET											0x10

#define	DIO2_DATA0														0x04
#define	DIO2_DATA1														0x04
#define	DIO2_DATA2														0x08
#define	DIO2_DATA3														0x0c

#define	DIO3_TO																0x00
#define	DIO3_RSSI_PREDET											0x01
#define	DIO3_TEMPCH_LOWBATT										0x03

//packet mode
#define	DIO0_PAYREADY_PKTSENT									0x00
#define	DIO0_CRCOK														0x40
#define	DIO0_TEMPCH_LOWBATT										0xc0

#define	DIO1_FIFOLEVEL												0x00
#define	DIO1_FE																0x10
#define	DIO1_FF																0x20

#define	DIO2_FF																0x00
#define	DIO2_RXRDY														0x04
#define	DIO2_FF_TO_FF													0x08
#define	DIO2_FF_SYNC_FF												0x0c

#define	DIO3_FE																0x00
#define	DIO3_TXRDY														0x01
#define	DIO3_FE_0															0x01
#define	DIO3_FE_1															0x02

/********** DIO mapping 2 ***********************/
#define	REG_DIOMAPPING2												0x04 //TBD
#define	DIOMAPPING2_DIO4_MSK									0xc0
#define	DIOMAPPING2_DIO5_MSK									0x30

#define	FSK_DIOMAPPING2_RSSIPRE_RSSI					0x00
#define	FSK_DIOMAPPING2_RSSIPRE_PRE						0x01

//LoRa mode
#define	DIO4_CADDET														0x00
#define	DIO4_PLLLOCK0													0x40
#define	DIO4_PLLLOCK1													0x80

#define	DIO5_MODERDY_LR												0x00
#define	DIO5_CLKOUT0													0x10
#define	DIO5_CLKOUT1													0x20

//continuous mode
#define	DIO4_MODERDY													0xc0
#define	DIO5_RSSI_PREDET											0x20

//packet mode
#define	DIO4_TEMPCH_LOWBATT										0x00
#define	DIO4_PLLLOCK													0x40
#define	DIO4_TO																0x80
#define	DIO4_RSSI_PREDET											0xc0

#define	DIO5_CLKOUT														0x00
#define	DIO5_PLLLOCK													0x10
#define	DIO5_DATA															0x20
#define	DIO5_MODERDY													0x30

//////////////////////////////////////////////////
/**************LORA specific*********************/
//////////////////////////////////////////////////

/********** LORA FIFO address pointer******/
#define REG_LR_FIFOADDR_PTR										0x0d //SPI interface address pointer in FIFO data buffer

/***** LORA TX FIFO base address **********/
#define REG_LR_TXFIFO_BASE										0x0e

/***** LORA RX FIFO base address **********/
#define REG_LR_RXFIFO_BASE										0x0f

/***** LORA RX FIFO current address *******/
#define REG_LR_RXFIFO_CURRADDR								0x10

/***** LORA IRQ flags mask ****************/
#define REG_LR_IRQFLAGS_MASK									0x11
#define LR_IRQFLAGS_RX_TO											0x80
#define LR_IRQFLAGS_RX_DONE										0x40
#define LR_IRQFLAGS_CRC_ERROR									0x20
#define LR_IRQFLAGS_HEADER_VALID							0x10
#define LR_IRQFLAGS_TX_DONE										0x08
#define LR_IRQFLAGS_CAD_DONE									0x04
#define LR_IRQFLAGS_FHSS_CHANGECH							0x02
#define LR_IRQFLAGS_CAD_DET										0x01

/***** LORA IRQ flags ****************/
#define REG_LR_IRQFLAGS												0x12
//bit fields same as in REG_LR_IRQFLAGS_MASK

/***** LORA RX Nb bytes *******************/
#define REG_LR_RXBYTES_NB											0x13 //Number of payload bytes of latest packet received

/***** LORA RX header counter MSB *********/
#define REG_LR_RXHEAD_CNT_MSB									0x14

/***** LORA RX header counter LSB *********/
#define REG_LR_RXHEAD_CNT_LSB									0x15

/***** LORA RX packet counter MSB**********/
#define REG_LR_RXPKT_CNT_MSB									0x16

/***** LORA RX packet counter LSB**********/
#define REG_LR_RXPKT_CNT_LSB									0x17

/***** LORA modem status ******************/
#define REG_LR_MODEM_STATUS										0x18
#define LR_MODEM_STATUS_CR_MSK								0xe0
#define LR_MODEM_STATUS_CR_SHIFT							5
#define LR_MODEM_STATUS_MODEM_CLEAR						0x10
#define LR_MODEM_STATUS_HEAD_INFO_VALID				0x08
#define LR_MODEM_STATUS_RX_ONGOING						0x04
#define LR_MODEM_STATUS_SIG_SYNC							0x02
#define LR_MODEM_STATUS_SIG_DET								0x01

/***** LORA packet SNR ********************/
#define REG_LR_PKT_SNR												0x19 //Estimation of SNR on last packet received.In two’s compliment format mutiplied by 4.

/***** LORA packet RSSI *******************/
#define REG_LR_PKT_RSSI												0x1a 
//RSSI of the latest packet received (dBm):
//RSSI[dBm] = -157 + Rssi (using HF output port, SNR >= 0) or
//RSSI[dBm] = -164 + Rssi (using LF output port, SNR >= 0)

/***** LORA RSSI value ********************/
#define REG_LR_RSSI														0x1b
//Current RSSI value (dBm) RSSI[dBm] = -157 + Rssi (using HF output port) or
//RSSI[dBm] = -164 + Rssi (using LF output port)

/***** LORA hop channel *******************/
#define REG_LR_HOPCHANNEL											0x1c
#define LR_HOPCHANNEL_PLLTO										0x80 //PLL failed to lock while attempting a TX/RX/CAD operation 1 - PLL did not lock, 0 - PLL did lock
#define LR_HOPCHANNEL_CRCON										0x40 
#define LR_HOPCHANNEL_FHSSCH_MSK							0x3f //Current value of frequency hopping channel in use

/***** LORA modem config 1 ****************/
#define REG_LR_MODEMCONFIG1										0x1d
#define LR_BW_7P8															0x00
#define LR_BW_10P4														0x10
#define LR_BW_15P6														0x20
#define LR_BW_20P8														0x30
#define LR_BW_31P25														0x40
#define LR_BW_41P7														0x50
#define LR_BW_62P5														0x60
#define LR_BW_125															0x70
#define LR_BW_250															0x80
#define LR_BW_500															0x90
#define LR_CR_4_5															0x02
#define LR_CR_4_6															0x04
#define LR_CR_4_7															0x06
#define LR_CR_4_8															0x08
#define LR_IMPL_HEADER												0x01
#define LR_EXPL_HEADER												0x00

/***** LORA modem config 2 ****************/
#define REG_LR_MODEMCONFIG2										0x1e
#define LR_SF6																0x60
#define LR_SF7																0x70
#define LR_SF8																0x80
#define LR_SF9																0x90
#define LR_SF10																0xa0
#define LR_SF11																0xb0
#define LR_SF12																0xc0
#define LR_TXCONT															0x08
#define LR_CRCOFF															0x00
#define LR_CRCON															0x04
#define LR_SYMBTO_MSB_MSK											0x03 //LORA symbol timeout LSB 

/***** LORA symbol timeout LSB ************/
#define REG_LR_SYMTO_LSB											0x1f

/***** LORA preamble MSB ******************/
#define REG_LR_PRE_MSB												0x20

/***** LORA preamble LSB ******************/
#define REG_LR_PRE_LSB												0x21

/***** LORA payload lenght ****************/
#define REG_LR_PAYLEN													0x22

/***** LORA max. payload lenght ***********/
#define REG_LR_MAXPAYLEN											0x23

/***** LORA hop period ********************/
#define REG_LR_HOP_PERIOD											0x24

/***** LORA FIFO RX byte address **********/
#define REG_LR_RXBYTE_PTR											0x25

/***** LORA modem config 3 ****************/
#define REG_LR_MODEMCONFIG3										0x26
#define LR_MODEMCONFIG3_LDROPT								0x08
#define LR_MODEMCONFIG3_AUTOAGC								0x04

/***** LORA ppm correction ****************/
#define REG_LR_PPMCORR												0x27

/***** LORA FEI MSB ***********************/
#define REG_LR_FEI_MSB												0x28
#define LR_FEI_MSB_MSK												0x0f

/***** LORA FEI middle ***********************/
#define REG_LR_FEI_MID												0x29

/***** LORA FEI LSB ***********************/
#define REG_LR_FEI_LSB												0x2a

/***** LORA wideband RSSI *****************/
#define REG_LR_RSSI_WB												0x2c //Wideband RSSI measurement used to locally generate a random number

/***** LORA IFFREQ 1 **********************/
#define REG_LR_IFFREQ1												0x2f //See errata note

/***** LORA IFFREQ 2 **********************/
#define REG_LR_IFFREQ2												0x30 //See errata note

/***** LORA detection optimize **************/
#define REG_LR_DETOPT													0x31
#define LR_DETOPT_AUTOIF_ON										0x80 //Should be set to 0x0 after each reset (POR on manual). See errata note for more information
#define LR_DETOPT_DETOPT_MSK									0x07 //LoRa Detection Optimize. 0x03 - SF7 to SF12, 0x05 - SF6

/***** LORA invert IQ 1 *********************/
#define REG_LR_INVERTIQ1											0x33
#define LR_INVERTIQ1_INVERTRX									0x40
#define LR_INVERTIQ1_INVERTTX									0x01

/***** LORA high BW optimize 1 ****************/
#define REG_LR_HIGHBW_OPT1										0x36 //Optimization for 500 kHz bandwidth. See errata note.

/***** LORA detection threshold ***************/
#define REG_LR_DET_THR												0x37 //LoRa detection threshold. 0x0A - SF7 to SF12, 0x0C - SF6

/*LORA 0x38 reserved. Probably LoRa sync MSB*****/

/***** LORA sync word ***************************/
#define REG_LR_SYNC														0x39 //LoRa Sync Word. Value 0x34 is reserved for LoRaWAN networks

/***** LORA high BW optimize 2 ******************/
#define REG_LR_HIGHBW_OPT2										0x3a // Optimization for 500 kHz bandwidth. See errata note.

/***** LORA invert IQ 2 *************************/
#define REG_LR_INVERTIQ2											0x3b //Set to 0x19 for inverted IQ

//////////////////////////////////////////////////
/**************FSK specific**********************/
//////////////////////////////////////////////////

/************FSK bitrate***********/
#define	REG_FSK_BR_MSB												0x02
#define	REG_FSK_BR_LSB												0x03

/***********FSK deviation**********/
#define	REG_FSK_FDEV_MSB											0x04
#define	FSK_FDEV_MSB_MSK											0x3f
#define	REG_FSK_FDEV_LSB											0x05

/******** FSK RX config *******************/
#define	REG_FSK_RXCONFIG											0x0d
#define	FSK_RXCONFIG_RST_ON_COLL							0x80
#define	FSK_RXCONFIG_RST_RX_NO_PLL_LOCK				0x40
#define	FSK_RXCONFIG_RST_RX_PLL_LOCK					0x20
#define	FSK_RXCONFIG_AFC_AUTO_OFF							0
#define	FSK_RXCONFIG_AFC_AUTO_ON							0x10
#define	FSK_RXCONFIG_AGC_AUTO_OFF							0
#define	FSK_RXCONFIG_AGC_AUTO_ON							0x08
#define	FSK_RXCONFIG_RXTRIG_MSK								0x07
#define	FSK_RXCONFIG_RXTRIG_NONE							0
#define	FSK_RXCONFIG_RXTRIG_RSSI							0x01
#define	FSK_RXCONFIG_RXTRIG_PRE_DET						0x06
#define	FSK_RXCONFIG_RXTRIG_RSSI_PRE					0x07

/********** FSK RSSI config ***************/
#define	REG_FSK_RSSICONFIG										0x0e
#define	FSK_RSSICONFIG_OFFSET_MSK							0xf8
#define	FSK_RSSICONFIG_SMOOTH_MSK							0x07
#define	FSK_RSSICONFIG_SMOOTH0								0
#define	FSK_RSSICONFIG_SMOOTH4								0x01
#define	FSK_RSSICONFIG_SMOOTH8								0x02
#define	FSK_RSSICONFIG_SMOOTH16								0x03
#define	FSK_RSSICONFIG_SMOOTH32								0x04
#define	FSK_RSSICONFIG_SMOOTH64								0x05
#define	FSK_RSSICONFIG_SMOOTH128							0x06
#define	FSK_RSSICONFIG_SMOOTH256							0x07

/********** FSK RSSI collision ************/
#define	REG_FSK_RSSICOLLISION									0x0f

/********** FSK RSSI threshold ************/
#define	REG_FSK_RSSITHRESHOLD									0x10

/********** FSK RSSI value ****************/
#define	REG_FSK_RSSIVAL												0x11

/********** FSK RX BW *********************/
#define	REG_FSK_RXBW													0x12
#define	FSK_RXBW_MANT_MSK											0x18
#define	FSK_RXBW_EXP_MSK											0x07

/********** FSK AFC BW ********************/
#define	REG_FSK_AFCBW													0x13
#define	FSK_AFCBW_MANT_MSK										0x18
#define	FSK_AFCBW_EXP_MSK											0x07

// FSK BW
#define FSK_BW_2600														0x17
#define FSK_BW_3100														0x0f
#define FSK_BW_3900														0x07
#define FSK_BW_5200														0x16
#define FSK_BW_6300														0x0e
#define FSK_BW_7800														0x06
#define FSK_BW_10400													0x15
#define FSK_BW_12500													0x0d
#define FSK_BW_15600													0x05
#define FSK_BW_20800													0x14
#define FSK_BW_25000													0x0c
#define FSK_BW_31300													0x04
#define FSK_BW_41700													0x13
#define FSK_BW_50000													0x0b
#define FSK_BW_62500													0x03
#define FSK_BW_83333													0x12
#define FSK_BW_100000													0x0a
#define FSK_BW_125000													0x02
#define FSK_BW_166700													0x11
#define FSK_BW_200000													0x09
#define FSK_BW_250000													0x01
#define FSK_BW_300000													0x00 // Invalid Badwidth

/********** FSK OOK PEAK ******************/
#define	REG_FSK_OOK_PEAK											0x14
#define	FSK_OOK_PEAK_BITSYNC_OFF							0
#define	FSK_OOK_PEAK_BITSYNC_ON								0x20
#define	FSK_OOK_PEAK_THRTYPE_FIXED						0
#define	FSK_OOK_PEAK_THRTYPE_PEAK							0x08
#define	FSK_OOK_PEAK_THRTYPE_AVG							0x10
#define	FSK_OOK_PEAK_THRSTEP0P5								0
#define	FSK_OOK_PEAK_THRSTEP1P0								0x01
#define	FSK_OOK_PEAK_THRSTEP1P5								0x02
#define	FSK_OOK_PEAK_THRSTEP2P0								0x03
#define	FSK_OOK_PEAK_THRSTEP3P0								0x04
#define	FSK_OOK_PEAK_THRSTEP4P0								0x05
#define	FSK_OOK_PEAK_THRSTEP5P0								0x06
#define	FSK_OOK_PEAK_THRSTEP6P0								0x07

/********** FSK OOK fix *******************/
#define	FSK_OOK_FIX														0x15

/********** FSK OOK average ***************/
#define	REG_FSK_OOK_AVG												0x16
#define	FSK_OOK_AVG_PKTHR1										0x0
#define	FSK_OOK_AVG_PKTHR2										0x20
#define	FSK_OOK_AVG_PKTHR4										0x40
#define	FSK_OOK_AVG_PKTHR8										0x60
#define	FSK_OOK_AVG_PKTHR2IN1									0x80
#define	FSK_OOK_AVG_PKTHR4IN1									0xa0
#define	FSK_OOK_AVG_PKTHR8IN1									0xc0
#define	FSK_OOK_AVG_PKTHR16IN1								0xe0
#define	FSK_OOK_AVG_OFFSET0										0
#define	FSK_OOK_AVG_OFFSET2										0x04
#define	FSK_OOK_AVG_OFFSET4										0x08
#define	FSK_OOK_AVG_OFFSET6										0x0c
#define	FSK_OOK_AVG_THRFILT32PI								0
#define	FSK_OOK_AVG_THRFILT8PI								0x01
#define	FSK_OOK_AVG_THRFILT4PI								0x02
#define	FSK_OOK_AVG_THRFILT2PI								0x03

/********** FSK AFC-FEI *******************/
#define	REG_FSK_AFCFEI												0x1a
#define	FSK_AFCFEI_AGCSTART										0x10
#define	FSK_AFCFEI_AFCCLEAR										0x02
#define	FSK_AFCFEI_AFCAUTOCLR									0x01

/********** FSK AFC MSB *******************/
#define	REG_FSK_AFC_MSB												0x1b

/********** FSK AFC LSB *******************/
#define	REG_FSK_AFC_LSB												0x1c

/********** FSK FEI MSB********************/
#define	REG_FSK_FEI_MSB												0x1d

/********** FSK FEI LSB********************/
#define	REG_FSK_FEI_LSB												0x1e

/********** FSK preamble detection *********/
#define	REG_FSK_PREDET												0x1f
#define	FSK_PREDET_OFF												0
#define	FSK_PREDET_ON													0x80
#define	FSK_PREDET_SIZE1											0
#define	FSK_PREDET_SIZE2											0x20
#define	FSK_PREDET_SIZE3											0x40
#define	FSK_PREDET_TOL_MSK										0x1f

/********** FSK RSSI timeout **************/
#define	REG_FSK_RX_TO1												0x20 // TimeoutRxRssi

/********** FSK preamble timeout **********/
#define	REG_FSK_RX_TO2												0x21 //	TimeoutRxPreamble

/********** FSK sync timeout **************/
#define	REG_FSK_RX_TO3												0x22 //	TimeoutSignalSync

/********** FSK RX delay ******************/
#define	REG_FSK_RX_DELAY											0x23

/********** FSK oscillator settings *******/
#define	REG_FSK_OSC														0x24
#define	FSK_OSC_RC_CALSTART										0x08
#define	FSK_OSC_CLKOUT_DIV1										0
#define	FSK_OSC_CLKOUT_DIV2										0x01
#define	FSK_OSC_CLKOUT_DIV4										0x02
#define	FSK_OSC_CLKOUT_DIV8										0x03
#define	FSK_OSC_CLKOUT_DIV16									0x04
#define	FSK_OSC_CLKOUT_DIV32									0x05
#define	FSK_OSC_CLKOUT_RC											0x06
#define	FSK_OSC_CLKOUT_OFF										0x07

/********** FSK preamble MSB **************/
#define	REG_FSK_PRE_MSB												0x25

/********** FSK preamble LSB **************/
#define	REG_FSK_PRE_LSB												0x26

/********** FSK sync config ***************/
#define	REG_FSK_SYNCCONFIG										0x27
#define	FSK_SYNCCONFIG_AUTORST_OFF						0
#define	FSK_SYNCCONFIG_AUTORST_NOWAITPLL			0x40
#define	FSK_SYNCCONFIG_AUTORST_WAITPLL				0x80
#define	FSK_SYNCCONFIG_PRE_AA									0
#define	FSK_SYNCCONFIG_PRE_55									0x20
#define	FSK_SYNCCONFIG_SYNC_OFF								0
#define	FSK_SYNCCONFIG_SYNC_ON								0x10
#define	FSK_SYNCCONFIG_SYNCSIZE_MSK						0x07

/********** FSK sync value 1 **************/
#define	REG_FSK_SYNCVAL1											0x28

/********** FSK sync value 2 **************/
#define	REG_FSK_SYNCVAL2											0x29

/********** FSK sync value 3 **************/
#define	REG_FSK_SYNCVAL3											0x2a

/********** FSK sync value 4 **************/
#define	REG_FSK_SYNCVAL4											0x2b

/********** FSK sync value 5 **************/
#define	REG_FSK_SYNCVAL5											0x2c

/********** FSK sync value 6 **************/
#define	REG_FSK_SYNCVAL6											0x2d

/********** FSK sync value 7 **************/
#define	REG_FSK_SYNCVAL7											0x2e

/********** FSK sync value 8 **************/
#define	REG_FSK_SYNCVAL8											0x2f

/********** FSK packet config 1 ***********/
#define	REG_FSK_PKTCFG1												0x30
#define	FSK_PKTCFG1_FIX_LEN										0
#define	FSK_PKTCFG1_VAR_LEN										0x80
#define	FSK_PKTCFG1_DCFREE_NONE								0
#define	FSK_PKTCFG1_DCFREE_MANCH							0x20
#define	FSK_PKTCFG1_DCFREE_WHITE							0x40
#define	FSK_PKTCFG1_CRC_OFF										0
#define	FSK_PKTCFG1_CRC_ON										0x10
#define	FSK_PKTCFG1_CRC_AUTOCLR_ON						0
#define	FSK_PKTCFG1_CRC_AUTOCLR_OFF						0x08
#define	FSK_PKTCFG1_ADDRFILT_NONE							0
#define	FSK_PKTCFG1_ADDRFILT_ADDR							0x02
#define	FSK_PKTCFG1_ADDRFILT_ADDR_BR					0x04
#define	FSK_PKTCFG1_CRC_CCITT									0
#define	FSK_PKTCFG1_CRC_IBM										0x01

/********** FSK packet config 2 ***********/
#define	REG_FSK_PKTCFG2												0x31
#define	FSK_PKTCFG2_CONT											0
#define	FSK_PKTCFG2_PKT												0x40
#define	FSK_PKTCFG2_IOHOME_OFF								0
#define	FSK_PKTCFG2_IOHOME_ON									0x20
#define	FSK_PKTCFG2_IOHOMEPWRFRM							0x10 //reserved
#define	FSK_PKTCFG2_BEACON_OFF								0
#define	FSK_PKTCFG2_BEACON_ON									0x08
#define	FSK_PKTCFG2_PAYLEN_MSB_MSK						0x07 //3 bits

/********** FSK packet payload lenght *******/
#define	REG_FSK_PAYLEN												0x32

/********** FSK node address ****************/
#define	REG_FSK_NODE_ADDR											0x33

/********** FSK broadcast address ***********/
#define	REG_FSK_BROADCAST_ADDR								0x34

/********** FSK FIFO threshold **************/
#define	REG_FSK_FIFOTRESH											0x35
#define	FSK_FIFOTRESH_TXSTART_FIFOLEVEL				0
#define	FSK_FIFOTRESH_TXSTART_FIFONOTEMPTY		0x80
#define	FSK_FIFOTRESH_THRESHOLD_MSK						0x3f

/********** FSK sequencer config 1 **********/
#define	REG_FSK_SEQCONFIG1										0x36
#define	FSK_SEQCONFIG1_SEQSTART								0x80
#define	FSK_SEQCONFIG1_SEQSTOP								0x40
#define	FSK_SEQCONFIG1_STBY										0
#define	FSK_SEQCONFIG1_SLEEP									0x20
#define	FSK_SEQCONFIG1_FROMSTART_LP						0
#define	FSK_SEQCONFIG1_FROMSTART_RX						0x08
#define	FSK_SEQCONFIG1_FROMSTART_TX						0x10
#define	FSK_SEQCONFIG1_FROMSTART_TXONFIFO			0x18
#define	FSK_SEQCONFIG1_LP_SEQOFF							0
#define	FSK_SEQCONFIG1_LP_SEQIDLE							0x04
#define	FSK_SEQCONFIG1_FROMIDLE_TX						0
#define	FSK_SEQCONFIG1_FROMIDLE_RX						0x02
#define	FSK_SEQCONFIG1_FROMTX_LP							0
#define	FSK_SEQCONFIG1_FROMTX_RX							0x01

/********** FSK sequencer config 2 **********/
#define	REG_FSK_SEQCONFIG2										0x37
#define	FSK_SEQCONFIG2_FROMRX_PKTRCV_ONPL			0x20
#define	FSK_SEQCONFIG2_FROMRX_LP_ONPL					0x40
#define	FSK_SEQCONFIG2_FROMRX_RKTRCV_ONCRC		0x60
#define	FSK_SEQCONFIG2_FROMRX_SEQOFF_ONRSSI		0x80
#define	FSK_SEQCONFIG2_FROMRX_SEQOFF_ONSYNC		0xa0
#define	FSK_SEQCONFIG2_FROMRX_SEQOFF_ONPRE		0xc0
#define	FSK_SEQCONFIG2_FROMRXTO_RX						0x0
#define	FSK_SEQCONFIG2_FROMRXTO_TX						0x08
#define	FSK_SEQCONFIG2_FROMRXTO_LP						0x10
#define	FSK_SEQCONFIG2_FROMRXTO_SEQOFF				0x18
#define	FSK_SEQCONFIG2_FROMPKT_SEQOFF					0
#define	FSK_SEQCONFIG2_FROMPKT_TXONFIFO				0x01
#define	FSK_SEQCONFIG2_FROMPKT_LP							0x02
#define	FSK_SEQCONFIG2_FROMPKT_RXFS						0x03
#define	FSK_SEQCONFIG2_FROMPKT_RX							0x04

/********** FSK timer resolution ****************/
#define	reg_FSK_TIMERRESOL										0x38
#define	FSK_TIMERRESOL_TIM1_DIS								0
#define	FSK_TIMERRESOL_TIM1_64								0x04
#define	FSK_TIMERRESOL_TIM1_4096							0x08
#define	FSK_TIMERRESOL_TIM1_262144						0x0c
#define	FSK_TIMERRESOL_TIM2_DIS								0
#define	FSK_TIMERRESOL_TIM2_64								0x01
#define	FSK_TIMERRESOL_TIM2_4096							0x02
#define	FSK_TIMERRESOL_TIM2_262144						0x03

/********** FSK timer coefficients **************/
#define	FSK_TIM1_COEFF												0x39

/********** FSK timer coefficients **************/
#define	REG_FSK_TIM2_COEFF										0x3a

/********** FSK image calibration ***************/
#define	REG_FSK_IMGCAL												0x3b 
#define	FSK_IMGCAL_AUTOIMGCAL_OFF							0x00 //default
#define	FSK_IMGCAL_AUTOIMGCAL_ON							0x80
#define	FSK_IMGCAL_IMGCAL_START								0x40
#define	FSK_IMGCAL_IMGCALRUNNING							0x20
#define	FSK_IMGCAL_TEMPCHANGE									0x08
#define	FSK_IMGCAL_TEMPTHRESHOLD_5						0x00
#define	FSK_IMGCAL_TEMPTHRESHOLD_10						0x02 //default
#define	FSK_IMGCAL_TEMPTHRESHOLD_15						0x04
#define	FSK_IMGCAL_TEMPTHRESHOLD_20						0x06
#define	FSK_IMGCAL_TEMPMON_ON									0x00 
#define	FSK_IMGCAL_TEMPMON_OFF								0x01

/********** temperature *********************/
#define	REG_FSK_TEMP													0x3c //-1°C per Lsb

/********** battery status ******************/
#define	REG_FSK_LOWBATT												0x3d
#define	FSK_LOWBATT_DIS												0x00 //default
#define	FSK_LOWBATT_EN												0x08
#define	FSK_LOWBATT_1P695											0x00
#define	FSK_LOWBATT_1P764											0x01
#define	FSK_LOWBATT_1P835											0x02 //default
#define	FSK_LOWBATT_1P905											0x03
#define	FSK_LOWBATT_1P976											0x04
#define	FSK_LOWBATT_2P045											0x05
#define	FSK_LOWBATT_2P116											0x06
#define	FSK_LOWBATT_2P185											0x07

/********** FSK IRQ flags 1 *********************/
#define	REG_FSK_IRQFLAGS1											0x3e
#define	FSK_IRQFLAGS1_MODEREADY								0x80
#define	FSK_IRQFLAGS1_RXREADY									0x40
#define	FSK_IRQFLAGS1_TXREADY									0x20
#define	FSK_IRQFLAGS1_PLLLOCK									0x10
#define	FSK_IRQFLAGS1_RSSI										0x08
#define	FSK_IRQFLAGS1_TIMEOUT									0x04
#define	FSK_IRQFLAGS1_PREDET									0x02
#define	FSK_IRQFLAGS1_SYNCDET									0x01

/********** FSK IRQ flags 2 *********************/
#define	REG_FSK_IRQFLAGS2											0x3f
#define	FSK_IRQFLAGS2_FIFOFULL								0x80
#define	FSK_IRQFLAGS2_FIFOEMPTY								0x40
#define	FSK_IRQFLAGS2_FIFOLEVEL								0x20
#define	FSK_IRQFLAGS2_FIFOOVR									0x10
#define	FSK_IRQFLAGS2_PKTSENT									0x08
#define	FSK_IRQFLAGS2_PAYREADY								0x04
#define	FSK_IRQFLAGS2_CRCOK										0x02
#define	FSK_IRQFLAGS2_LOWBATT									0x01

/********** FSK PLL hopping *********************/
#define	REG_FSK_PLLHOP												0x44
#define	FSK_PLLHOP_FASTHOP_OFF								0x00
#define	FSK_PLLHOP_FASTHOP_ON									0x80

/********** FSK BR fractional part **************/
#define	REG_FSK_BR_FRAC												0x5d

#endif

