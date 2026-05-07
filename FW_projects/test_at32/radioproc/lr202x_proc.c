#include "lr202x_proc.h"
#include "lr20xx.h"
#include "radio_proc.h"

void LR202x_printerrors(void);
void LR202x_printstatus(void);

//const lr11xx_radio_rssi_calibration_table_t calib_0_600 =     {12,12,14, 0, 1, 3, 4, 4, 3, 6, 6, 6, 6, 6, 6, 6, 6, 0};
//const lr11xx_radio_rssi_calibration_table_t calib_600_2000 =  { 2, 2, 2, 3, 3, 4, 5, 4, 4, 6, 5, 5, 6, 6, 6, 7, 6, 0};
//const lr11xx_radio_rssi_calibration_table_t calib_2000_2700 = { 6, 7, 6, 4, 3, 4,14,12,14,12,12,12,12, 8, 8, 9, 9, 2030};

void LR202x_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
		{
			opmode = RADIO_OPMODE_SLEEP;
			lr20xx_system_sleep_cfg_t sleepcfg;
			sleepcfg.is_clk_32k_enabled = false;
			sleepcfg.is_ram_retention_enabled = false;
			lr20xx_system_set_sleep_mode(NULL,&sleepcfg,0);
			break;
		}

    case RADIO_OPMODE_STBYRC:
		opmode = RADIO_OPMODE_STBYRC;
    lr20xx_system_set_standby_mode(NULL,LR20XX_SYSTEM_STANDBY_MODE_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
    lr20xx_system_set_standby_mode(NULL,LR20XX_SYSTEM_STANDBY_MODE_XOSC);
    break;

    case RADIO_OPMODE_FS:
		opmode = RADIO_OPMODE_FS;
    lr20xx_system_set_fs_mode(NULL);
    break;

    case RADIO_OPMODE_TX:
		opmode = RADIO_OPMODE_TX;
    lr20xx_radio_common_set_tx(NULL,0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
		opmode = RADIO_OPMODE_RX;
    lr20xx_radio_common_set_rx(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
		opmode = RADIO_OPMODE_TXSTREAMPRE;
    lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_INFINITE_PREAMBLE);
    break;
		
		//add PN9
  }
	//LR112X_printerrors(0);
}

void LR20xx_RssiCal(uint32_t freq)
{
//	if(freq <= 60000000) lr11xx_radio_set_rssi_calibration(NULL,&calib_0_600);
//	else if((freq > 60000000) && (freq <= 200000000)) lr11xx_radio_set_rssi_calibration(NULL,&calib_600_2000);
//	else lr11xx_radio_set_rssi_calibration(NULL,&calib_2000_2700);
}

void LR20xx_printstatus(void)
{
	lr20xx_system_stat1_t stat1;
	lr20xx_system_stat2_t stat2;
	lr20xx_system_irq_mask_t irqstatus;
	lr20xx_system_get_status(NULL,&stat1,&stat2,&irqstatus);

	printf("Status=0x%02X,0x%02X\r\n",stat1.command_status,stat2.chip_mode); //mask reset source	`		
}

void LR20xx_printerrors(void)
{
	uint16_t errors;
	lr20xx_system_get_errors(NULL,&errors);
	printf("Errors: 0x%04X\r\n",errors);
}

