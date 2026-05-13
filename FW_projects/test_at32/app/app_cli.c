#include <stdio.h>
#include "app_cli.h"
#include "bsp.h"
#include "radio_proc.h"
#include "wk_adc.h"
#include "flash.h"
#include "usb_app.h"

//#include "gui.h"
//#include "gpsparser.h"


CommandState_t state;
char ciBuffer[256];
//uint8_t printmode = 0;
//uint8_t txmode = 0;
int32_t check_param(int32_t param,const int32_t param_min,const int32_t param_max);
//General
void cli_reset(int argc, char **argv);
void cli_info(int argc, char **argv);
void cli_getver(int argc, char **argv);
void cli_getdevid(int argc, char **argv);
void cli_setdevid(int argc, char **argv);
//RF settings

void cli_getfreq(int argc, char **argv);
void cli_setfreq(int argc, char **argv);
void cli_getpower(int argc, char **argv);
void cli_setpower(int argc, char **argv);
void cli_getxotrim(int argc, char **argv);
void cli_setxotrim(int argc, char **argv);
//void cli_storectune(int argc, char **argv);

void cli_getRSSI(int argc, char **argv);

void cli_getstatus(int argc, char **argv);

//LoRa
void cli_getmodparams(int argc, char **argv);
void cli_setmodparams(int argc, char **argv);
void cli_getpacketparams(int argc, char **argv);
void cli_setpacketparams(int argc, char **argv);

//debug functions
void cli_readreg(int argc, char **argv);
void cli_writereg(int argc, char **argv);

void cli_pan_readreg(int argc, char **argv);
void cli_pan_writereg(int argc, char **argv);

void cli_initconfig(int argc, char **argv);
void cli_storeconfig(int argc, char **argv);

void cli_dumpregs(int argc, char **argv);

void cli_pan_dumpregs(int argc, char **argv);


//void cli_storeconfig(int argc, char **argv);

void cli_sendburst(int argc, char **argv);
void cli_stopburst (int argc, char **argv);
void cli_txstream(int argc, char **argv);

void cli_getstats(int argc, char **argv);
void cli_clearstats(int argc, char **argv);

//device dependent
void cli_getvbat(int argc, char **argv);
//special tests

void cli_getchip(int argc, char **argv);
void cli_radioinit(int argc, char **argv);

void cli_setopmode(int argc, char **argv);

//special tests
void cli_sweeptx(int argc, char **argv);
void cli_sweeprx(int argc, char **argv);


//void cli_setpaconfig(int argc, char **argv); //technological


CommandEntry_t commands[] =
  {
    //general
    COMMAND_ENTRY("RESET", "", cli_reset, "Device reset"),
    COMMAND_ENTRY("GET_UID", "", cli_info, "Get CPU ID"),
    COMMAND_ENTRY("GET_VERSION", "", cli_getver, "Get HW/FW version"),
    COMMAND_ENTRY("GET_DEVID", "", cli_getdevid, "Get device ID"),
    COMMAND_ENTRY("SET_DEVID", "w", cli_setdevid, "Get device ID"),
    //RF settings
    COMMAND_ENTRY("GET_FREQ", "", cli_getfreq, "Get frequency settings"),
    COMMAND_ENTRY("SET_FREQ", "w", cli_setfreq, "Set frequency settings"),

    COMMAND_ENTRY("GET_POWER", "", cli_getpower, ""),
    COMMAND_ENTRY("SET_POWER", "w", cli_setpower, ""),
    COMMAND_ENTRY("GET_XOTRIM", "", cli_getxotrim, ""),
    COMMAND_ENTRY("SET_XOTRIM", "w", cli_setxotrim, ""),
    //COMMAND_ENTRY("STORE_CTUNE", "", cli_storectune, ""),

    COMMAND_ENTRY("GET_RSSI", "", cli_getRSSI, ""),
    
    COMMAND_ENTRY("GET_STATUS", "", cli_getstatus, ""),

    //LoRa
    COMMAND_ENTRY("GET_MODPARAMS", "", cli_getmodparams, "Get LoRa modulation parameters"),
    COMMAND_ENTRY("SET_MODPARAMS", "wwww", cli_setmodparams, "Set LoRa modulation parameters"),
    COMMAND_ENTRY("GET_PACKETPARAMS", "", cli_getpacketparams, "Get LoRa packet parameters"),
    COMMAND_ENTRY("SET_PACKETPARAMS", "wwwwww", cli_setpacketparams, "Set LoRa packet parameters"),
                        
    //RF tests
    COMMAND_ENTRY("START_TX", "www", cli_sendburst, "Start packet burst"),
    COMMAND_ENTRY("STOP_TX", "", cli_stopburst, "Stop packet burst"),
    COMMAND_ENTRY("TX_STREAM", "v", cli_txstream, "Start/stop TX stream"),
		
		COMMAND_ENTRY("GET_STATS", "", cli_getstats, ""),
		COMMAND_ENTRY("CLR_STATS", "", cli_clearstats, ""),
    //System health - device dependent
    COMMAND_ENTRY("GET_VBAT", "", cli_getvbat, "Get ADC data"),

    COMMAND_ENTRY("READ_REG", "w", cli_readreg, ""),
    COMMAND_ENTRY("WRITE_REG", "ww", cli_writereg, ""),
    COMMAND_ENTRY("DUMP_REGS", "ww", cli_dumpregs, ""),
		
		COMMAND_ENTRY("PAN_READREG", "ww", cli_pan_readreg, ""),
    COMMAND_ENTRY("PAN_WRITEREG", "www", cli_pan_writereg, ""),
		COMMAND_ENTRY("PAN_DUMPREGS", "www", cli_pan_dumpregs, ""),
                        
    COMMAND_ENTRY("INIT_CONFIG", "ww", cli_initconfig, ""),
		COMMAND_ENTRY("STORE_CONFIG", "", cli_storeconfig, ""),
    //COMMAND_ENTRY("SET_PACONFIG", "ww", cli_setpaconfig, ""), //technological
		
		COMMAND_ENTRY("GET_CHIP", "", cli_getchip, ""),
		COMMAND_ENTRY("RADIO_INIT", "", cli_radioinit, ""),
		COMMAND_ENTRY("SET_OPMODE", "w", cli_setopmode, ""),
		
    COMMAND_ENTRY("SWEEP_TX", "wwwww", cli_sweeptx, "TX sweep"),
    COMMAND_ENTRY("SWEEP_RX", "wwwww", cli_sweeprx, "RX scan"),
		
		COMMAND_ENTRY(NULL, NULL, NULL, NULL)
  };


void cli_init(void)
{
  ciInitState(&state, ciBuffer, sizeof(ciBuffer), commands);
}

void cli_proc(void)
{
  char input = RETARGET_ReadChar();
  if(input != '\0' && input != 0xFF) ciProcessInput(&state, &input, 1);
}

void ciErrorCallback(char* command, CommandError_t error)
{
  if (error == CI_UNKNOWN_COMMAND) 
	{
    printf("INVALID COMMAND\r\n");
  } 
	else if (error == CI_MAX_ARGUMENTS) 
	{
    printf("TOO MANY ARGUMENTS\r\n");
  } 
	else if (error == CI_INVALID_ARGUMENTS) 
	{
    printf("INVALID ARGUMENT\r\n");
  }
}


//cli functions implementation
//General
void cli_reset(int argc, char **argv)
{
  printf("RESET: OK\r\n");
	bsp_reset_proc();
}

void cli_info(int argc, char **argv)
{
	printf("GET_UID: 0x%llX\r\n", bsp_get_uid());
	//printf("GET_UID: 0x%llX\r\n", *(uint64_t*)0x4002002c);
}

void cli_getver(int argc, char **argv)
{
  printf("GET_VERSION: HW=%d,FW=%d.%d\r\n", HW_VERSION, FW_VERSION, FW_REVISION);
}

void cli_getdevid(int argc, char **argv)
{
  printf("GET_DEVID: 0x%08X\r\n", radioconfig.id);
}

void cli_setdevid(int argc, char **argv)
{
  radioconfig.id = ciGetUnsigned(argv[1]);
  writeconfig();
  printf("SET_DEVID: 0x%08X\r\n", radioconfig.id);
}

void cli_getfreq(int argc, char **argv)
{
  printf("GET_FREQ: %u\r\n",radioconfig.freq / 1000);
}

void cli_setfreq(int argc, char **argv)
{
	uint8_t err = radio_set_freq(ciGetUnsigned(argv[1]));
	printf("SET_FREQ: ");
	if(err == RADIO_OK) 
	{
		radioconfig.freq = ciGetUnsigned(argv[1]) * 1000;
		printf("%u\r\n",radioconfig.freq / 1000);
	}
	else printerror(err); 
}

void cli_getpower(int argc, char **argv)
{
  printf("GET_POWER: %d dBm\r\n",radioconfig.txpower);
}

void cli_setpower(int argc, char **argv)
{
	//check power parameter here!
	int8_t dbm = ciGetSigned(argv[1]);
	uint8_t err = radio_set_power(dbm);
	printf("SET_POWER: ");
	if(err == RADIO_OK) 
	{
		radioconfig.txpower = dbm;
		printf("%d dBm\r\n",dbm);
	}
  else printerror(err);
}


void cli_getxotrim(int argc, char **argv)
{
  uint8_t trim;
	int8_t err = radio_getxotrim(&trim);
	printf("GET_XOTRIM: ");
	if(err == RADIO_OK) printf("%d\r\n",trim);
	else printerror(err);
}

void cli_setxotrim(int argc, char **argv)
{
  uint8_t trim = ciGetUnsigned(argv[1]);
	check_param(trim,0,94);
  int8_t err = radio_setxotrim(trim);
	printf("SET_XOTRIM: ");
	if(err == RADIO_OK) printf("%d\r\n",trim);
	else printerror(err);
}

void cli_getRSSI(int argc, char **argv)
{
  float rssi;
	int8_t err = radio_getrssi(&rssi);
  printf("GET_RSSI: ");
	if(err == RADIO_OK) printf("%.1f dBm\r\n",rssi);
	else printerror(err); 
}

void cli_getstatus(int argc, char **argv)
{
	uint8_t stat1,stat2;
	printf("GET_STATUS: ");
	int8_t err = radio_get_status(&stat1,&stat2);
	if(err == RADIO_OK) printf("0x%02X,0x%02X\r\n",stat1,stat2);
	else printerror(err); 
}

//LoRa

void cli_getmodparams(int argc, char **argv)
{
  uint16_t bw_khz;
	uint8_t sf;
	uint8_t cr;
	uint8_t ldropt;
	
	printf("GET_MODPARAMS: \r\nBW=%u\r\nSF=%d\r\n",radioconfig.bw,radioconfig.sf);
	printf("CR:");
	switch(radioconfig.cr)
	{
	case 0:
	printf("OFF\r\n");
	break;
	case 1:
	printf("4_5\r\n");
	break;
	case 2:
	printf("4_6\r\n");
	break;
	case 3:
	printf("4_7\r\n");
	break;
	case 4:
	printf("4_8\r\n");
	break;
	default:
	printf("INVALID\r\n");
	break;
	}
	printf("LDROPT:");
	if(radioconfig.ldropt == 1) printf("ON\r\n");
	else printf("OFF\r\n");
}

void cli_setmodparams(int argc, char **argv)
{
  uint16_t bw = ciGetUnsigned(argv[1]);
	uint8_t sf = ciGetUnsigned(argv[2]);
	uint8_t cr = ciGetUnsigned(argv[3]);
	uint8_t opt = ciGetUnsigned(argv[4]);
	check_param(bw,7,1600);
	check_param(sf,5,12);
	check_param(cr,0,4);
	check_param(opt,0,1);
	printf("SET_MODPARAMS: ");
	int8_t err = radio_setmodparams(bw,sf,cr,opt);
	if(err == RADIO_OK)
	{
    radioconfig.bw = bw;
		radioconfig.sf = sf;
		radioconfig.cr = cr;
		radioconfig.ldropt = opt;
		printf("\r\nBW=%u\r\nSF=%d\r\n",radioconfig.bw,radioconfig.sf);
    printf("CR:");
    switch(radioconfig.cr)
    {
      case 0:
      printf("OFF\r\n");
      break;
      case 1:
      printf("4_5\r\n");
      break;
      case 2:
      printf("4_6\r\n");
      break;
      case 3:
      printf("4_7\r\n");
      break;
      case 4:
      printf("4_8\r\n");
      break;
      default:
      printf("INVALID\r\n");
      break;
    }
    printf("LDROPT:");
    if(radioconfig.ldropt == true) printf("ON\r\n");
    else printf("OFF\r\n");
	}
	else printerror(err);
}

void cli_getpacketparams(int argc, char **argv)
{
	printf("GET_PACKETPARAMS:\r\nSYNC=0x%02X,PRE_LEN=%d\r\nPAY_LEN=%d\r\nHEADER:",radioconfig.sync,radioconfig.prelen,radioconfig.paylen);
  if(radioconfig.header == true) printf("IMPLICIT\r\n");
  else printf("EXPLICIT\r\n");
  printf("CRC:");
  if(radioconfig.crc == 1) printf("ON\r\n");
  else printf("OFF\r\n");
  printf("INVERTIQ:");
  if(radioconfig.invertiq == 1) printf("ON\r\n");
  else printf("OFF\r\n");
}

void cli_setpacketparams(int argc, char **argv)
{
  uint8_t sync;  
	uint16_t pre;
	uint8_t pay;
	uint8_t head;
	uint8_t crc;
	uint8_t inviq;
	sync = ciGetUnsigned(argv[1]) & 0xff;
	pre = ciGetUnsigned(argv[2]) & 0xffff;
	pay = ciGetUnsigned(argv[3]);
	head = ciGetUnsigned(argv[4]);
	check_param(head,0,1);
	check_param(crc,0,1);
	check_param(inviq,0,1);
	int8_t err = radio_setpktparams(sync,pre,pay,head,crc,inviq);
	printf("SET_PACKETPARAMS: ");
	if(err == RADIO_OK) 
	{
		radioconfig.sync = sync;
		radioconfig.prelen = pre;
		radioconfig.paylen = pay;
		radioconfig.header = head;
		radioconfig.crc = crc;
		radioconfig.invertiq = inviq;
		printf("\r\nSYNC=0x%04X,PRE_LEN=%d\r\nPAY_LEN=%d\r\nHEADER:",radioconfig.sync,radioconfig.prelen,radioconfig.paylen);
    if(radioconfig.header == 1) printf("IMPLICIT\r\n");
    else printf("EXPLICIT\r\n");
    printf("CRC:");
    if(radioconfig.crc == 1) printf("ON\r\n");
    else printf("OFF\r\n");
    printf("INVERTIQ:");
    if(radioconfig.invertiq == true) printf("ON\r\n");
    else printf("OFF\r\n");
	}
	else printerror(err);
}

//System tests
void cli_sendburst(int argc, char **argv)
{
  txpacketcount = ciGetUnsigned(argv[1]);
  if(txpacketcount == 0) txpacketcount = 1;
  inter_packet_delay = ciGetUnsigned(argv[2]);
  slave_id = ciGetUnsigned(argv[3]);
  if(inter_packet_delay < 100) inter_packet_delay = 100;
  radio_startburst();
  printf("START_TX: %d,%d\r\n",txpacketcount,inter_packet_delay);
}

void cli_stopburst (int argc, char **argv)
{
  if(master)
  {
    master = false;
    tx_request = false;
    printf("STOP_TX: OK\r\n");
  }
  else printf("STOP_TX: ERROR\r\n");
}

void cli_txstream(int argc, char **argv)
{
  printf("TX_STREAM: ");
	int8_t err = radio_stream(ciGetUnsigned(argv[1]));
	if(err == RADIO_OK)
	{
		switch(txmode)
		{
			case 0:
			default:
			printf("OFF\r\n");
			break;

			case 1:
      printf("CW\r\n");
      break;

			case 2:
      printf("PREAMBLE\r\n");
      break;
		}
	}
	else printerror(err);
}

//Device dependent commands
void cli_getvbat(int argc, char **argv)
{
  printf("GET_VBAT: %.2f\r\n",Vbatt);
}

void cli_readreg(int argc, char **argv)
{
  uint32_t val;
	uint32_t reg = ciGetUnsigned(argv[1]);
	printf("READ_REG: ");
	int8_t err = radio_readreg(reg,&val);
	if(err == RADIO_OK) printf("0x%X,0x%X\r\n",reg,val);
  else printerror(err);
}

void cli_writereg(int argc, char **argv)
{
  uint32_t reg = ciGetUnsigned(argv[1]);
  uint32_t val = ciGetUnsigned(argv[2]);
	printf("WRITE_REG: ");
	int8_t err = radio_writereg(reg,val);
  if(err == RADIO_OK) printf("0x%X,0x%X\r\n",reg,val);
	else printerror(err);
}

void cli_dumpregs(int argc, char **argv)
{
  uint32_t reg_l,reg_h;
  uint32_t i;
	uint32_t val;
  reg_l = ciGetUnsigned(argv[1]);
  reg_h = ciGetUnsigned(argv[2]);
  printf("DUMP_REGS:\r\n"); 
  for(i = reg_l; i <= reg_h; i++)
  {
    int8_t err = radio_readreg(i,&val);
		if(err == RADIO_OK) printf("0x%X,0x%X\r\n",i,val);
		else
		{
			printerror(err);
			break;
		}
  }
}

void cli_initconfig(int argc, char **argv)
{
  printf("INIT_CONFIG: ");
	int8_t err = radio_initconfig(ciGetUnsigned(argv[1]),ciGetUnsigned(argv[2]));
  if(err == RADIO_OK) printf("OK\r\n");
  else printerror(err);
}

void cli_storeconfig(int argc, char **argv)
{
	printf("STORE_CONFIG: ");
	int32_t err = writeconfig();
	if(err == RADIO_OK) printf("OK\r\n");
	else printerror((int8_t)err); 
}

//void cli_setpaconfig(int argc, char **argv)
//{
//	uint8_t dutycycle;
//	uint8_t hpmax;
//	
//	dutycycle = ciGetUnsigned(argv[1]);
//	if(dutycycle > 7) dutycycle = 7;
//	if(hpmax > 7) hpmax = 7;
//	hpmax = ciGetUnsigned(argv[2]);
//	SX126X_SetPaConfig(dutycycle,hpmax,false);
//	printf("SET_PACONFIG: %d,%d\r\n",dutycycle,hpmax);
//}

void cli_getstats(int argc, char **argv)
{
	rxstats_t rxstats;
	printf("GET_STATS: ");
	int8_t err = radio_getstats(&rxstats);
	if(err == RADIO_OK) printf("PKT=%d,FERR=%d,HERR=%d\r\n",rxstats.pkt_received,rxstats.crc_error,rxstats.header_error);
	else printerror(err);
}

void cli_clearstats(int argc, char **argv)
{
	printf("CLR_STATS: ");
	int8_t err = radio_clearstats();
	if(err == RADIO_OK) printf("OK\r\n");
	else printerror(err);
}

void cli_getchip(int argc, char **argv)
{
	uint8_t hw,use_case,fw_major,fw_minor; 
	printf("GET_CHIP: ");
	int8_t err = radio_get_chip_version(&hw,&use_case,&fw_major,&fw_minor);
	if(err == RADIO_OK) printf("%d,%d,%d,%d\r\n\r\n",hw,use_case,fw_major,fw_minor);
	else printerror(err);
}

void cli_setopmode(int argc, char **argv)
{
	uint8_t op_mode = ciGetUnsigned(argv[1]); 
	printf("SET_OPMODE ");
	int8_t err = radio_setopmode(opmode);
	if(err == RADIO_OK) printf("%d\r\n\r\n",op_mode);
	else printerror(err);
}

void printerror(int8_t error)
{
	//printf("ERROR: ");
	switch(error)
	{
		case INVALID_CHIP:
		printf("INVALID CHIP\r\n");
		break;
		case RADIO_COMM_FAIL:
		printf("NOT RESPONDING\r\n");
		break;
		case RADIO_BUSY:
		printf("BUSY\r\n");
		break;
		case FEATURE_NOT_SUPPORTED:
		printf("NOT SUPPORTED\r\n");
		break;
		case RADIO_INVALID_MODE:
		printf("INVALID MODE\r\n");
		break;
		case RADIO_INVALID_PARAMETER:
		printf("INVALID PARAMETER\r\n");
		break;
		case RADIO_TODO:
		printf("TODO\r\n");
		break;
		default:
		printf("UNKNOWN\r\n");
	}
}  

void cli_radioinit(int argc, char **argv)
{
	printf("RADIO_INIT: ");
	int8_t err = radio_init();
	if(err == RADIO_OK) printf("OK\r\n");
	else printerror(err);
}

void cli_sweeptx(int argc, char **argv)
{
	uint32_t startfreq = ciGetUnsigned(argv[1]);
	uint32_t stopfreq = ciGetUnsigned(argv[2]);
	uint32_t step = ciGetUnsigned(argv[3]);
	uint32_t us = ciGetUnsigned(argv[4]);
	uint8_t stream = ciGetUnsigned(argv[5]);
	printf("SWEEP_TX: ");
	int8_t err = radio_txsweep(startfreq,stopfreq,step,us,stream);
	if(err == RADIO_OK)
	{
		//check result, on or off
		if(txmode == 0) 
		{
			txled_off();
			printf("STOP\r\n");
		}
		else 
		{
			txled_on();
			printf("START\r\n");
		}
	}
	else printerror(err);
}

void cli_sweeprx(int argc, char **argv)
{
	uint32_t startfreq = ciGetUnsigned(argv[1]);
	uint32_t stopfreq = ciGetUnsigned(argv[2]);
	uint32_t step = ciGetUnsigned(argv[3]);
	uint32_t ms = ciGetUnsigned(argv[4]);
	float rssitr = (float)ciGetSigned(argv[5]);
	printf("SWEEP_RX: ");
	int8_t err = radio_rxscan(startfreq,stopfreq,step,ms,rssitr);
	if(err == RADIO_OK)
	{
		//check result, on or off
		if(sweeprx == false) 
		{
			rxled_off();
			printf("STOP\r\n");
		}
		else 
		{
			rxled_on();
			printf("START\r\n");
		}
	}
	else printerror(err);
}

void cli_pan_readreg(int argc, char **argv)
{
	uint8_t page = ciGetUnsigned(argv[1]);
	if(page > 3) page = 3;
	uint8_t reg = ciGetUnsigned(argv[2]);
	uint8_t val;
	printf("PAN_READREG: ");
	if(radioconfig.chip != 3029) printerror(INVALID_CHIP);
	else 
	{
		val = PAN_ReadPageReg(page,reg);
		printf("%d,0x%02X,0x%02X\r\n",page,reg,val);
	}		
	
}

void cli_pan_writereg(int argc, char **argv)
{
	uint8_t page = ciGetUnsigned(argv[1]);
	if(page > 3) page = 3;
	uint8_t reg = ciGetUnsigned(argv[2]);
	uint8_t val = ciGetUnsigned(argv[3]);
	printf("PAN_WRITEREG: ");
	if(radioconfig.chip != 3029) printerror(INVALID_CHIP);
	else
	{
		PAN_WritePageReg(page,reg,val);
		printf("%d,0x%02X,0x%02X\r\n",page,reg,val);
	}
}

void cli_pan_dumpregs(int argc, char **argv)
{
	uint8_t i;
	uint8_t page = ciGetUnsigned(argv[1]);
	if(page > 3) page = 3;
	uint8_t startreg = ciGetUnsigned(argv[2]);
	uint8_t stopreg = ciGetUnsigned(argv[3]);
	if(startreg > 0x7e) startreg = 0x7e;
	if(stopreg > 0x7f) stopreg = 0x7f;
	if(stopreg <= startreg) stopreg = startreg + 1;
	printf("PAN_DUMPREGS: ");
	if(radioconfig.chip != 3029) printerror(INVALID_CHIP);
	else
	{
		for(i = startreg; i < stopreg+1; i++) printf("\r\n%d,0x%02X,0x%02X",page,i,PAN_ReadPageReg(page,i));
		printf("\r\n");
	}
}

int32_t check_param(int32_t param,const int32_t param_min,const int32_t param_max)
{
	if(param <= param_min) param = param_min;
	if(param >= param_max) param = param_max;
	return param;
}



