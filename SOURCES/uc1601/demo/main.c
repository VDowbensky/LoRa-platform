/* Демонстрация графических функций для работы с индикаторами на драйвере
UC1601S.   Тестирование проводилась на индикторе RDX0154/120/077
HI-TECH Software PICC-18 v9.66/9.80
MPLAB v8.85
controller PIC16LF1936
Геннадий Чернов, Днепропетровск 2012
Gennady Chernov, Dnepropetrovsk 2012
проект ИЛЛИССИ
*/
#include <htc.h>			// для настройки под выбранный контроллер
//----------------------------------------------------
#pragma jis					// допускает конфигурирование не латинских символов в обработчике строк
							// для обработки русских символов
//----------------------------------------------------
#include <stdlib.h>			// для функции rand
#include "graphic.h"		// графическая библиотека


/*
//---------------------------------------------------------------------------
// конфигурирование контроллера
__CONFIG(1, FOSC_INTIO67 & FCMEN_OFF & IESO_OFF);
//				|				|			+---Отключен режим переключения Генератора
//				|				+--Отказоустойчивый Монитор Генератора отключен
//				+--Выбор Генератора, Внутренний генератор, функция порта на RA6 и RA7
//
__CONFIG(2L, PWRT_ON & BOREN_ON & BORV_27);
//				|			|		+---Уровень напряжения срабатывания сброс при понижении питания 2.7 В
//				|			+---Сброс по понижению питания
//				+---Задежка запуска процессора при подаче питания (включена)
//
__CONFIG(2H, WDTEN_ON & WDTPS_32768);
//				|			+---1:1024 задежка страбатывания сторожевого таймера.	
//				+---сторожевой таймер включен
//
__CONFIG(3, CCP2MX_PORTBE & PBADEN_OFF & LPT1OSC_OFF & HFOFST_OFF & MCLRE_ON);
//					|			|				|			|			+---MCLR включен
//					|			|				|			+---HF-INTOSC Быстрый Запуск - отключен	
//					|			|				+---Режим малой мощности Генератора Timer1 Отключен, T1 работает в стандартном режиме мощности.		
//					|			+---Режим работы порта PORTB после сброса, выводы PORTB<4:0> конфигурированы на цифровой ввод		
//					+---Выход CCP2 подключен к выводу RB3.		
__CONFIG(4, STVREN_OFF	& LVP_OFF & XINST_OFF & DEBUG_ON);
//					|			|		|			+---Режим фонового отладчика отключен.
//					|			|		+---Расширенный набор команд отключен  
//					|			+---Низковольтное программирование выключенно.
//					+---Сброс при переполнении стека отключен.
//
__CONFIG(5, CP0_OFF & CP1_OFF & CP2_OFF & CP3_OFF & CPB_OFF & CPD_OFF);
//				|		|		|			|		|		+---защита EEPROM данных активирована
//				|		|		|			|		+---защита блока загрузки активирована
//				|		|		|			+---защита кода блока 3 активирована
//				|		|		+---защита кода блока 2 активирована
//				|		+---защита кода блока 1 активирована
//				+---защита кода блока 0 активирована
//
__CONFIG(6, WRT0_OFF & WRT1_OFF & WRT2_OFF & WRT3_OFF & WRTB_OFF & WRTC_OFF & WRTD_OFF);
//				|		|			|		|			|			|		+---Защита записи в EEPROM память
//				|		|			|		|			|			+---Защита записи в регистр конфирураций
//				|		|			|		|			+---Защита записи в блок загрузки
//				|		|			|		+---защита записи в блок 3
//				|		|			+---защита записи в блок 2
//				|		+---защита записи в блок 1
//				+---защита записи в блок 0  
__CONFIG(7, EBTR0_OFF &  EBTR1_OFF &  EBTR2_OFF &  EBTR3_OFF &  EBTRB_OFF);
//				|			|			|				|			|
//				|			|			|				|			+---Защита от табличного чтения блок загрузки
//				|			|			|				+---Защита от табличного чтения блока памяти 3
//				|			|			+---Защита от табличного чтения блока памяти 2
//				|			+---Защита от табличного чтения блока памяти 1
//				+---Защита от табличного чтения блока памяти 0
//------------------------------------------------------------------------------------------------------- 
*/
//---------------------------------------------------------------------------

// конфигурирование контроллера

__CONFIG(
 FOSC_HS & 		// INTOSC oscillator: I/O function on CLKIN pin
// FOSC_INTOSC & 
 WDTE_ON	&		// WDT enabled
 PWRTE_ON	&		// PWRT enabled
 MCLRE_ON	&		// MCLR/VPP pin function is digital input
 CP_ON		&		// Program memory code protection is enabled
 CPD_ON		&		// Data memory code protection is enabled
 BOREN_ON	&		// Brown-out Reset enabled
 CLKOUTEN_OFF	&	// CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
 IESO_OFF	&		// Internal/External Switchover mode is disabled
 FCMEN_OFF);		// Fail-Safe Clock Monitor is disabled

__CONFIG(
 WRT_ALL 	& 		// 000h to 7FFh write protected, no addresses may be modified by EECON control
 PLLEN_ON 	&		// 4x PLL enabled
 STVREN_OFF & 		// Stack Overflow or Underflow will not cause a Reset 
 BORV_HI 	&		// Brown-out Reset Voltage (VBOR) set to 2.7V
 LVP_OFF);			// High-voltage on MCLR/VPP must be used for programming
//------------------------------------------------------------------------------------------------------- 
//---------------------------------------------------------------------------

#define PODSC LATC5		//управление подсветкой дисплея
#define _XTAL_FREQ 32000000 // Наша тактовая 32 Mhz

// прототип
// инициализация контроллера
void init(void);

/*
const char *s_list[] = 
			{
			"[*][Меню][Режим][Стоп]",			// вызвать s_list[0]
			"S-2: ESSENDON ",		// вызвать s_list[1]
			" S-3: FOOTSCRAY ",		// вызвать s_list[2]
			"S-4: JOLEY ",
			"S-5: Привет ",
			"S-6: ALTONA ",
			"S-7: MELBOURNE ",		// вызвать s_list[6]
			};
*/
int  a,m,n;

static bit NAP, mig; // бит для функции заливки


void main(void)
{
		CLRWDT(); // сброс сторожевого таймера +++++++++++++++++++++- 
		init();
		WDTCON=0b00100101;
		PODSC=1;// включить подсветку дисплея
		clear_LCD(0);//очистка дисплея 



		height=1;		
		width=1;
		__delay_ms(100);// ожидание
		StringS_LCD ("UC1601",-1,6);
		width=2;
		__delay_ms(100);// ожидание
		StringS_LCD ("UC1601",-1,6);
		width=3;
		__delay_ms(100);// ожидание
		StringS_LCD ("UC1601",-1,6);		
		width=4;
		__delay_ms(100);// ожидание
		StringS_LCD ("UC1601",-1,6);		
		__delay_ms(100);// ожидание
		StringS_LCD ("      ",-1,6);
		width=3;
		StringS_LCD ("UC1601",-1,6);				
		__delay_ms(100);// ожидание
		StringS_LCD ("      ",-1,6);		
		width=2;
		StringS_LCD ("UC1601",-1,6);
		__delay_ms(100);// ожидание
		StringS_LCD ("      ",-1,6);		
		width=1;
		StringS_LCD ("UC1601",-1,6);		
		height=0;
		StringS_LCD ("graphics library",-1,4);
		StringS_LCD ("V-2.0.1",-1,3);
/*

		height=0;		
		width=0;
		String_LCD (" 2345 ",-1,45);
		height=0;		
		width=1;
		String_LCD (" 2345 ",-1,37);
		width=2;
		String_LCD (" 2345 ",-1,29);
		width=3;
		String_LCD (" 2345 ",-1,21);		
		width=4;
		String_LCD (" 2345 ",-1,13);		
		width=5;
		String_LCD (" 2345 ",-1,5);

*/


		__delay_ms(5000);// ожидание		
		clear_LCD(2);//очистка дисплея 
		
		// экран-1	
		tipBor=8;
		rectangle( 5, 5, dispX-10, dispY-10);
		tipBor=0;
		lineType=1;
		line(11,7,11,40);
		line(7,11,40,11);
		line(124,52,80,52);
		line(120,56,120,25);		
		
		height=1; width=2;
		StringS_LCD ("Привет",-1,5);
		StringS_LCD ("Мир!",-1,3);							
		height=0; width=0;
		StringS_LCD (" MAGETEX ",8,8);		
		__delay_ms(5000);// ожидание		
		
		// экран-2	
		clear_LCD(0);	//очистка дисплея 
		StringS_LCD ("Типы штрифтов",-1,8);


		cursorG_LCD (0,33);
		height=2;
		width=4;
		type=1;
		SymbolS_LCD (0);
		SymbolS_LCD (4);
		width=2;
		type=0;
		SymbolS_LCD (':');
		width=4;
		type=1;
		SymbolS_LCD (1);
		SymbolS_LCD (7);		

		height=1;		
		width=0;
		type=0;		
		cursorS_LCD (100,5);
		SymbolS_LCD ('a');
		SymbolS_LCD ('m');		
		
		
		height=0;
		width=1;
		cursorS_LCD (2,3);
		StringpS_LCD("понедельник, 16/04/12");		
		height=1;

		cursorS_LCD (0,1);
		StringpS_LCD("hello Dolly");
		
		height=0;		
		width=0;
		StringS_LCD ("Звонок",87,2);
		StringS_LCD ("17-00",90,1);					
		__delay_ms(5000);

		// экран-3
		clear_LCD(1);	// заливка

		for (a=0;a<200;a++)	point (0,(rand()%dispX), (rand()%dispY));
		height=0;		
		width=2;
		inv=1;
		StringS_LCD (" Звездное ",-1,7);
		StringS_LCD (" небо ",-1,0);
		inv=0;
		__delay_ms(5000);// ожидание				
		
		clear_LCD(0);	//очистка дисплея 
		lineType=1;
		for (a=0;a<30;a++)	line (rand()%dispX,  rand()%dispY,  rand()%dispX, rand()%dispY);
		StringS_LCD ("Линии",-1,1);


		// ожидание
		__delay_ms(5000);				
		//очистка дисплея


		clear_LCD(0);

//1-тип угла,2-тип бордюра,3-толщина бордюра,4-тип заливки,5/6-координаты вернего левого угла, ширина и высота.			
			angleType=1;
			lineType=1;
			tipBor=2;
			fillType=0;
			rectangle(0, 0, dispX, dispY);	
			angleType=0;
			lineType=0;
			tipBor=0;
			fillType=3;
			rectangle(55, 10, 25, 25);
			fillType=4;
			rectangle(70, 20, 25, 25);
			fillType=7;
			rectangle(85, 30, 25, 25);			

		// ожидание
		__delay_ms(1000);	
			lineType=1;
			tipBor=1;
			fillType=1;
			rectangle(10, 10, 25, 11);
			fillType=2;
			rectangle(15, 15, 25, 11);
			fillType=3;
			rectangle(20, 20, 25, 11);
			fillType=4;
			rectangle(25, 25, 25, 11);
		
		// ожидание
		__delay_ms(1000);	
			angleType=1;
			lineType=1;
			tipBor=1;
			fillType=1;
			rectangle(55, 30, 25, 11);
			fillType=2;
			rectangle(50, 35, 25, 11);
			fillType=3;
			rectangle(45, 40, 25, 11);
			fillType=4;
			rectangle(40, 45, 25, 11);


		// ожидание
		__delay_ms(5000);				
		//очистка дисплея
		clear_LCD(0);
		
			lineType=4;		
		for (a=0;a<dispX;a++)	line (0,  dispY,  a, 0);		
		for (a=0;a<dispX;a++)	line (dispX,  0,  a, dispY);		
			angleType=0;
			lineType=1;
			tipBor=2;
			fillType=1;		
		rectangle(15, 18, 94, 29);
			angleType=1;		
			tipBor=3;
		rectangle(17, 20, 90, 25);
		height=1;
		width=2;
		StringS_LCD ("MAGETEX",25,4);
		
		__delay_ms(5000);			
		//очистка дисплея
		clear_LCD(0);
	width=0;
	height=0;
	StringS_LCD ("Система координат",-1,8);
		lineType=1;
		line(   0,  12,  dispX, 12);
		line( dispX,  12,  126, 10);
		line( dispX,  12,  126, 14);

		line(   5,   7,  5, dispY);
		line(   5,  dispY,  3, 58);
		line(   5,  dispY,  7, 58);

		height=0;
		width=0;
		StringS_LCD ("по Х [0-131]",-1,1);
		StringS_LCD ("по Y [0-63]",-1,2);


		cursorS_LCD (0,5);
		StringpS_LCD (" За начало вывода при-");
		cursorS_LCD (0,4);
		StringpS_LCD ("митивов, берется ниж-");
		cursorS_LCD (0,3);
		StringpS_LCD ("ний левый угол.");

		__delay_ms(5000);				
		//очистка дисплея
		clear_LCD(0);
		strip (0,1,3,10,55,40,6,30);		
		strip (0,1,3,80,55,40,6,80);

		strip (0,1,3,10,40,110,10,70);
		width=0;
		height=0;
		StringS_LCD (" Индикатор - 70%",-1,4);
		StringS_LCD ("Размеры и положение",-1,3);	
		StringS_LCD ("произвольные",-1,2);
		StringS_LCD ("Значение 0-100%",-1,1);

	

			

		__delay_ms(5000);				
		//очистка дисплея
		clear_LCD(2);

		n=0;
		NAP=1;


while (1)
	{
		CLRWDT(); // сброс сторожевого таймера +++++++++++++++++++++- 
// вывод бегущей строки			
		ticker_LCD("Демонстрация бегущей строки расположенной в ПЗУ программ.",22,0,3);
// вывод бегущей строки	из ОЗУ
		tickerB_LCD(10,30,1);
// мигание сообщения
		if(mig)	{inv=0; StringS_LCD (" Авария ",75,7); mig=0;}
		else	{inv=1; StringS_LCD (" Авария ",75,7); mig=1;}
		inv=0;
		m++;
		if (m>9) m=0;
		if(NAP)	n=n+3;
		else	n=n-3;
		if (n>=90)
		{
			 NAP=0;
			 ca[32]='1';// смена значения в бегущей строке
			 ca[33]='2';
			 ca[35]='0';
		}	  
		if (n<=0)
		{
			NAP=1;
		 	ca[32]='5';// смена значения в бегущей строке
			ca[33]='5';
			ca[35]='7';
		} 		
		strip (0,1,3,5,46,50,10,n);
	}

}//


// описание функции инициализации контроллера
void init(void)
{
/*
	// конфигурирование аналоговых выводов
	ANSELH = 0x00;			// все выводы цифровые	
	ANSEL = 0x00;			// все выводы цифровые

	// настройка порта А
	LATA	= 0;		 	// или можно PORTA=0;
	TRISA  = 0b00010000; 	// настройка порта: все выводы на выход	

	// настройка порта В
	LATB	= 0;		 	// или можно PORTB=0;	
	TRISB 	= 0b00000001; 	// настройка порта: 1=input, 0=output RBO=вход, остальные выход

	// настройка порта С
	LATC	= 0;		 	//
	TRISC 	= 0b00011000;  	//	
//настройка генератора
	OSCCON = 0b01110000;	// IRCF<6:4> 16 MHz (111-HFINTOSC выбор 16 Мгц), SCS<1:0> 00 = настройка на главный генератор, 
							// для возможности работы через умножитель.
	OSCTUNE = 0b01111110;	// PLL enabled for HFINTOSC, Oscillator module is running at the factory calibrated frequency.

	SLRCON = 0;				// стандартное время переключение портов.

//настройка i2c
	I2C_Open();				// Скорость просмотра включается для режима на 100 кГц				

	i2c_start(0x70,0,0);	// 0x70 адрес индикатора, команда, запись
	i2c_write(0b11100010);
	i2c_stop();

	int_LCD();				// инициализация LCD;

*/
	CLRWDT();	// сброс сторожевого таймера

// настройка генератора 8*4=32мГц 
	OSCCON=0b11110000;
//	 	     ||||||||
//			 ||||| ++-- SCS<1:0>	основной генератор (работа через PLL) 
//			 |++++----- IRCF<3:0>	частота 8 мГц 
//			 +--------- SPLLEN	умножитель 0-отключен, 1-включен

// настройка портов
	TRISA	= 0;		//0b00000001; // RA0 вход датчика температуры
	ANSELA	= 0;
	PORTA	= 0;
	LATA	= 0;
		
	TRISB	= 0;
	ANSELB	= 0;
	LATB	= 0;
	PORTB	= 0;
		
	TRISC	= 0;
	LATC	= 0;
	PORTC	= 0;

// настройка сторожевого таймера	
//	WDTCON = 0b00010111;


//настройка i2c
	I2C_Open (400);			// Скорость просмотра включается для режима на 400 кГц				
	int_LCD();				// инициализация LCD;


}
//
