#include "lr112x_proc.h"
#include "lr112x.h"
#include "radio_proc.h"


//#define K_FREQ	1.0000051f //-6.8 ppm
//#define K_FREQ	1.0f


void LR112X_printerrors(void);
void LR112X_printstatus(void);
void LR112X_configcommon(void);

const LR112X_RssiCalibParams_t calib_0_600 =     {12,12,14, 0, 1, 3, 4, 4, 3, 6, 6, 6, 6, 6, 6, 6, 6, 0};
const LR112X_RssiCalibParams_t calib_600_2000 =  { 2, 2, 2, 3, 3, 4, 5, 4, 4, 6, 5, 5, 6, 6, 6, 7, 6, 0};
const LR112X_RssiCalibParams_t calib_2000_2700 = { 6, 7, 6, 4, 3, 4,14,12,14,12,12,12,12, 8, 8, 9, 9, 2030};

void LR112X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
		opmode = RADIO_OPMODE_SLEEP;
    LR112X_SetSleep(LR112X_SLEEP_POWERDOWN,false);
    break;

    case RADIO_OPMODE_STBYRC:
		opmode = RADIO_OPMODE_STBYRC;
    LR112X_SetStandby(0);
    break;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
    LR112X_SetStandby(1);
    break;

    case RADIO_OPMODE_FS:
		opmode = RADIO_OPMODE_FS;
    LR112X_SetFs();
    break;

    case RADIO_OPMODE_TX:
		opmode = RADIO_OPMODE_TX;
    LR112X_SetTx(0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
		opmode = RADIO_OPMODE_RX;
    LR112X_SetRx(0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    LR112X_SetTxCw();
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
		opmode = RADIO_OPMODE_TXSTREAMPRE;
    LR112X_SetTxInfinitePreamble();
    break;
  }
	//LR112X_printerrors(0);
}


void LR112X_RssiCal(uint32_t freq)
{
	if(freq <= 60000000) LR112X_SetRssiCalibration(&calib_0_600);
	else if((freq > 60000000) && (freq <= 200000000)) LR112X_SetRssiCalibration(&calib_600_2000);
	else LR112X_SetRssiCalibration(&calib_2000_2700);
}

void LR112X_printstatus(void)
{
	uint8_t stat1,stat2;
	LR112X_GetStatus(&stat1,&stat2);
	printf("Status=0x%02X,0x%02X\r\n",stat1>>1,(stat2 & 0x0f) >> 1); //mask reset source	`		
}

void LR112X_printerrors(void)
{
	uint16_t errors = LR112X_GetErrors();
	printf("Errors: 0x%04X\r\n",errors);
}

//GainOffset: Global offset added to the Gain Tune values. The offset is a 12-bit signed value, where 1lsb = 0.5dB.
//The power seen by the LR1121 analog front-end is affected by external components such as the matching network, or RF switches. An incorrect RSSI results in a sensitivity degradation in (G)FSK mode and an incorrect gain selection in LoRa and GFSK mode. An incorrect gain can result in a missed detection (packet loss) or decreased resistance to interference.
//By default, the chip is calibrated for the 868-915MHz band on the LR1121 EVK.

//below 600MHz 0  12 12 14 0 1 3 4 4 3 6 6 6 6 6 6 6 6
//from 600MHz to 2GHz 0 2  2  2  3  3  4  5  4  4  6  5  5  6  6  6  7  6
//above 2GHz 2030 6 7 6 4 3 4 14 12 14 12 12 12 12 8 8 9 9

// uint8_t gain[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 }; // NB: for the 2G4 path, the max gain is 16 - 17-20 can be ignored.
//    float power[] = { -35.0, -41.0, -45.0, -50.0, -53.5, -60.0, -65.0, -69.5, -75.0, -81.0, -82.5, -83.5, -84.0, -85.0, -86.0, -86.5, -87.0 };
//    /* Configure the chip at the system level */ 
//    lr11xx_system_reset( context );
//    lr11xx_system_set_reg_mode( context, reg_mode ); 
//    lr11xx_system_set_dio_as_rf_switch( context, rf_switch_cfg );
//    lr11xx_system_set_tcxo_mode( context, tune, timeout );  // Optional - only if there is a TCXO
//    lr11xx_system_clear_errors( context );
//    lr11xx_system_calibrate( context, 0x3F );  // 0x3F to enable all fields
//    /* Configure the chip at the modem level */ 
//    lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_GFSK ); 
//    lr11xx_radio_set_rf_freq( context, freq_in_hz );
//    lr11xx_system_calibrate_image_in_mhz( context, freq1_in_mhz, freq2_in_mhz ); 
//    lr11xx_radio_set_gfsk_mod_params( context, mod_params );  // Rx BW has to be set to
//LR11XX_RADIO_GFSK_BW_234300 - other modulation parameters can be anything 
//    lr11xx_radio_set_gfsk_pkt_params( context, pkt_params );  // Packet parameters can be
//anything
///* Configure the chip to be controlled manually */
//    lr11xx_regmem_write_regmem32_mask( context, 0x00F20214, 0x00080000, 0x00080000 );
//    lr11xx_regmem_write_regmem32_mask( context, 0x00F20230, 0x71110000, 0x71100000 );
//			
// lr11xx_radio_set_rssi_calibration( context, rssi_cal_table );  // All parrameters of rssi_cal_table set to 0
//    lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, 0xFFFFFF );
//    for( int i = 0; i++; i < 17 )  // 17 is the number of elements in gain array 
//    {
//        const uint8_t gain_step = MIN( gain[i], 13 );
//        const uint8_t lna_boost = ( gain > 13 ) ? gain - 13 : 0;
//        lr11xx_regmem_write_regmem32_mask( context, 0x00F20214, 0x00F00000, gain_step << 20 ); 
//        lr11xx_regmem_write_regmem32_mask( context, 0x00F3008C, 0x00070000, lna_boost << 16 );

///* Wait for 1 ms */
///* Insert here a control for your test equipment to generate a tone at RF frequency set to freq_in_hz with an output power set to power[i] dBm */
//        lr11xx_radio_get_rssi_inst( context, rssi_in_dbm );
///* Add a way to log (gain[i], power[i], rssi_in_dbm) triplet to be able to compute offset and
//tunes for the RSSI calibration */
//    }		
