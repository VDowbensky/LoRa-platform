#include "graphic.h"

// переменные глобальные extern
char cursorX, cursorY; // текущие положения курсоров.
char pos=0, posB=0;    // позиция бегущей строки

// знакогенератор
// знакогенератор WINDOWS
const char font[] = {

0x00, 0x00, 0x00, 0x00, 0x00,	// 0x20	пробел
0x00, 0x00, 0xF2, 0x00, 0x00,	// 0x21	! 
0x00, 0xE0, 0x00, 0xE0, 0x00,	// 0x22	" 
0x28, 0xFE, 0x28, 0xFE, 0x28,	// 0x23	# 
0x24, 0x54, 0xFE, 0x54, 0x48,	// 0x24	$ 
0xC6, 0xC8, 0x10, 0x26, 0xC6,	// 0x25	%
0x6C, 0x92, 0xAA, 0x44, 0x0A,	// 0x26	&
0x00, 0xA0, 0xC0, 0x00, 0x00,	// 0x27	'
0x38, 0x44, 0x82, 0x00, 0x00,	// 0x28	(				
0x00, 0x00, 0x82, 0x44,	0x38,	// 0x29	)			
0x28, 0x10, 0x7C, 0x10,	0x28,	// 0x2A	*			
0x10, 0x10, 0x7C, 0x10, 0x10,	// 0x2B	+				
0x00, 0x00, 0x0A, 0x0C, 0x00,	// 0x2C	,			
0x10, 0x10, 0x10, 0x10, 0x10,	// 0x2D	- 				
0x00, 0x00, 0x06, 0x06, 0x06,	// 0x2E	. 				
0x04, 0x08, 0x10, 0x20,	0x40,	// 0x2F	/		 

0x7C, 0x8A, 0x92, 0xA2, 0x7C,	// 0x30	0 
0x00, 0x42, 0xFE, 0x02, 0x00,	// 0x31	1
0x42, 0x86, 0x8A, 0x92, 0x62,	// 0x32	2
0x84, 0x82, 0xA2, 0xD2, 0x8C,	// 0x33	3
0x18, 0x28, 0x48, 0xFE, 0x08,	// 0x34	4
0xE4, 0xA2, 0xA2, 0xA2, 0x9C,	// 0x35	5
0x3C, 0x52, 0x92, 0x92, 0x0C,	// 0x36	6
0x80, 0x8E, 0x90, 0xA0, 0xC0,	// 0x37	7
0x6C, 0x92, 0x92, 0x92, 0x6C,	// 0x38	8
0x60, 0x92, 0x92, 0x94, 0x78,	// 0x39	9
0x00, 0x00, 0x6C, 0x6C, 0x00,	// 0x3A	:
0x00, 0x00, 0x6A, 0x6C, 0x00,	// 0x3B	//
0x10, 0x28, 0x44, 0x82, 0x00,	// 0x3C	<
0x28, 0x28, 0x28, 0x28, 0x28,	// 0x3D	=
0x00, 0x82, 0x44, 0x28, 0x10,	// 0x3E	>
0x40, 0x80, 0x8A, 0x90,	0x60,	// 0x3F	?		

0x4C, 0x92, 0x9C, 0x42, 0x3C,	// 0x40	@
0x7E, 0x88, 0x88, 0x88, 0x7E,	// 0x41	А 
0xFE, 0x92, 0x92, 0x92, 0x6C,	// 0x42	В
0x7C, 0x82, 0x82, 0x82, 0x44,	// 0x43	С
0xFE, 0x82, 0x82, 0x44, 0x38,	// 0x44	D
0xFE, 0x92, 0x92, 0x92, 0x82,	// 0x45	Е
0xFE, 0x90, 0x90, 0x90, 0x80,	// 0x46	F
0x7C, 0x82, 0x92, 0x92, 0x5E,	// 0x47	G
0xFE, 0x10, 0x10, 0x10, 0xFE,	// 0x48	Н			
0x00, 0x82, 0xFE, 0x82, 0x00,	// 0x49	I
0x04, 0x02, 0x82, 0xFC, 0x80,	// 0x4A	J
0xFE, 0x10, 0x28, 0x44, 0x82,	// 0x4B	К
0xFE, 0x02, 0x02, 0x02, 0x02,	// 0x4C	L
0xFE, 0x40, 0x20, 0x40, 0xFE,	// 0x4D	М
0xFE, 0x20, 0x10, 0x08, 0xFE,	// 0x4E	N
0x7C, 0x82, 0x82, 0x82, 0x7C,	// 0x4F	О			

0xFE, 0x90, 0x90, 0x90, 0x60,	// 0x50	Р
0x7C, 0x82, 0x8A, 0x84, 0x7A,	// 0x51	Q
0xFE, 0x90, 0x98, 0x94, 0x62,	// 0x52	R
0x62, 0x92, 0x92, 0x92, 0x8C,	// 0x53	S
0x80, 0x80, 0xFE, 0x80, 0x80,	// 0x54	Т	
0xFC, 0x02, 0x02, 0x02, 0xFC,	// 0x55	U
0xF8, 0x04, 0x02, 0x04, 0xF8,	// 0x56	V
0xFC, 0x02, 0x1C, 0x02, 0xFC,	// 0x57	W
0xC6, 0x28, 0x10, 0x28,	0xC6,	// 0x58	Х
0xE0, 0x10, 0x1E, 0x10, 0xE0,	// 0x59	Y
0x86, 0x8A, 0x92, 0xA2, 0xC2,	// 0x5A	Z
0x00, 0xFE, 0x82, 0x82, 0x00,	// 0x5B	[
0x18, 0x24, 0x7E, 0x24, 0x18,	// 0x5C	ф
0x00, 0x82, 0x82, 0xFE, 0x00,	// 0x5D	]
0x20, 0x40, 0x80, 0x40, 0x20,	// 0x5E	^
0x02, 0x02, 0x02, 0x02, 0x02,	// 0x5F	_			

0x00, 0x00, 0x80, 0x40, 0x00,	// 0x60	'
0x04, 0x2A, 0x2A, 0x2A, 0x1E,	// 0x61	а	
0xFE, 0x12, 0x22, 0x22, 0x1C,	// 0x62	b
0x1C, 0x22, 0x22, 0x22, 0x04,	// 0x63	с
0x1C, 0x22, 0x22, 0x12, 0xFE,	// 0x64	d
0x1C, 0x2A, 0x2A, 0x2A, 0x18,	// 0x65	е
0x10, 0x7E, 0x90, 0x80, 0x40,	// 0x66	f
0x10, 0x2A, 0x2A, 0x2A, 0x3C,	// 0x67	g
0xFE, 0x10, 0x20, 0x20, 0x1E,	// 0x68	h
0x00, 0x22, 0xBE, 0x02, 0x00,	// 0x69	i
0x04, 0x02, 0x22, 0xBC, 0x00,	// 0x6A	j
0x00, 0xFE, 0x08, 0x14, 0x22,	// 0x6B	k
0x00, 0x82, 0xFE, 0x02, 0x00,	// 0x6C	l
0x3E, 0x20, 0x18, 0x20, 0x1E,	// 0x6D	m
0x3E, 0x10, 0x20, 0x20, 0x1E,	// 0x6E	n

0x1C, 0x22, 0x22, 0x22,	0x1C,	// 0x6F	o		
0x3E, 0x28, 0x28, 0x28, 0x10,	// 0x70	р
0x10, 0x28, 0x28, 0x28, 0x3E,	// 0x71	q
0x3E, 0x10, 0x20, 0x20, 0x10,	// 0x72	r
0x12, 0x2A, 0x2A, 0x2A, 0x24,	// 0x73	s
0x20, 0xFC, 0x22, 0x02, 0x04,	// 0x74	t
0x3C, 0x02, 0x02, 0x04, 0x3E,	// 0x75	u
0x38, 0x04, 0x02, 0x04, 0x38,	// 0x76	v
0x3C, 0x02, 0x0C, 0x02, 0x3C,	// 0x77	w
0x22, 0x14, 0x08, 0x14, 0x22,	// 0x78	х
0x30, 0x0A, 0x0A, 0x0A, 0x30,	// 0x79	у
0x22, 0x26, 0x2A, 0x32, 0x22,	// 0x7A	z
0x00, 0x10, 0x6C, 0x82, 0x00,	// 0x7B	{
0x00, 0x00, 0xFE, 0x00, 0x00,	// 0x7C	|
0x00, 0x82, 0x6C, 0x10, 0x00,	// 0x7D	}
0x08, 0x10, 0x10, 0x08, 0x08,	// 0x7E	~ 			

0xFF, 0x80, 0x80, 0x80, 0x80,	// 0x7F символы псевдографики
0x80, 0x80, 0x80, 0x80, 0x80,	// 0x80
0x80, 0x80, 0xFF, 0x80, 0x80,	// 0x81			
0x80, 0x80, 0x80, 0x80, 0xFF,	// 0x82
0xFF, 0x00, 0x00, 0x00, 0x00,	// 0x83			
0x00, 0x00, 0xFF, 0x00, 0x00,	// 0x84		
0x00, 0x00, 0x00, 0x00, 0xFF,	// 0x85
0xFF, 0x10, 0x10, 0x10, 0x10,	// 0x86
0x10, 0x10, 0x10, 0x10, 0x10,	// 0x87
0x10, 0x10, 0xFF, 0x10, 0x10,	// 0x88	
0x10, 0x10, 0x10, 0x10, 0xFF,	// 0x89
0xFF, 0x10, 0x10, 0x10, 0x10,	// 0x8A
0x10, 0x10, 0x10, 0x10, 0x10,	// 0x8B
0x10, 0x10, 0xFF, 0x10, 0x10,	// 0x8C
0x10, 0x10, 0x10, 0x10, 0xFF,	// 0x8D
0xFF, 0x80, 0xBF, 0xA0, 0xA0,	// 0x8E
0xA0, 0xA0, 0xA0, 0xA0, 0xA0,	// 0x8F
0xA0, 0xBF, 0x80, 0xBF, 0xA0,	// 0x90
0xA0, 0xA0, 0xBF, 0x80, 0xFF,	// 0x91
0xFF, 0x00, 0xFF, 0x00, 0x00,	// 0x92
0x00, 0xFF, 0x00, 0xFF, 0x00,	// 0x93					
0x00, 0x00, 0xFF, 0x00, 0xFF,	// 0x94
0xFF, 0x00, 0xEF, 0x28, 0x28,	// 0x95
0x28, 0x28, 0x28, 0x28, 0x28,	// 0x96
0x28, 0xEF, 0x00, 0xEF, 0x28,	// 0x97
0x28, 0x28, 0xEF, 0x00, 0xFF,	// 0x98
0xFF, 0x01, 0xFD, 0x05, 0x05,	// 0x99
0x05, 0x05, 0x05, 0x05, 0x05,	// 0x9A				
0x05, 0xFD, 0x01, 0xFD,	0x05,	// 0x9B
0x05, 0x05, 0xFD, 0x01, 0xFF,	// 0x9C

0x00, 0x00, 0x3E, 0x22, 0x3E,	// 0x9D 0 маленькие цифры
0x00, 0x00, 0x00, 0x00, 0x3E,	// 0x9E 1
0x00, 0x00, 0x2E, 0x2A, 0x3A,	// 0x9F 2
0x00, 0x00, 0x2A, 0x2A, 0x3E,	// 0xA0 3
0x00, 0x00, 0x38, 0x08, 0x3E,	// 0xA1 4
0x00, 0x00, 0x3A, 0x2A, 0x2E,	// 0xA2 5
0x00, 0x00, 0x3E, 0x2A, 0x2E,	// 0xA3 6
0x00, 0x00, 0x20, 0x20, 0x3E,	// 0xA4 7
0x00, 0x00, 0x3E, 0x2A, 0x3E,	// 0xA5 8
0x82, 0xBA, 0xAA, 0x92, 0xBA,	// 0xA6 dli
0x8A, 0x8A, 0x82, 0xBA, 0x82,	// 0xA7
0x88, 0x54, 0x22, 0x88, 0x22,	// 0xA8 нагрев
0xAA, 0xAA, 0xAA, 0xAA, 0xAA,	// 0xA9 дымогенератор
0x10, 0x20, 0x10, 0x10, 0x20,	// 0xAA	~// spe *
0x20, 0x40, 0xFE, 0x40, 0x20,	// 0xAB	su * стрелка верх 
				

0x08, 0x04, 0xFE, 0x04, 0x08,	// 0xAC	sd * // вниз
0x10, 0x10, 0x54, 0x38, 0x10,	// 0xAD	sr * стрелка в право ->
0x10, 0x38, 0x54, 0x10, 0x10,	// 0xAE	sl * <- стрелка в влево			
0x00, 0x07, 0x08, 0x13, 0x24,	// 0xAF	верхний левый угол
0x28, 0x28, 0x28, 0x28, 0x28,	// 0xB0	горизонтальные линии верняя
0x28, 0x28, 0x13, 0x08, 0x07,	// 0xB1	верний правый угол
0x00, 0xFF, 0x00, 0xFF, 0x00,	// 0xB2	веритикальные левые линии
0x00, 0x00, 0xFF, 0x00, 0xFF,	// 0xB3	вертикальные правые линии
0x00, 0xE0, 0x10, 0xC8, 0x24,	// 0xB4	нижний левый угол
0x14, 0x24, 0xC8, 0x10, 0xE0,	// 0xB5	нижний правый угол
0x14, 0x14, 0x14, 0x14, 0x14,	// 0xB6	горизонтальные линии нижняя	
0x10, 0x38, 0x7C, 0xFE, 0x00,	// 0xB7	треугольник влево
0x00, 0xFE, 0x7C, 0x38, 0x10,	// 0xB8	треугольник вправо			
0x08, 0x78, 0xFC, 0x78, 0x08,	// 0xB9	kol * колокольчик
				

0x63, 0x25, 0x18, 0x18,	0xA4,	// 0xBA	вентилятор	
0x7E, 0x42, 0x42, 0x42, 0x7E,	// 0xBB	квадрат пустой
0x7E, 0x7E, 0x7E, 0x7E, 0x7E,	// 0xBC	квадрат полный
0x30, 0x0C, 0x30, 0x0C, 0x30,	// 0xBD	птичка
0x80, 0xFF, 0x01, 0x01, 0xFF,	// 0xBE	импульс			
0x60, 0x90, 0x90, 0x60, 0x00,	// 0xBF	градус *

0x7E, 0x88, 0x88, 0x88, 0x7E,	// 0xC0	А
0xFE, 0x92, 0x92, 0x92, 0x0C,	// Б
0xFE, 0x92, 0x92, 0x92, 0x6C,	// В
0xFE, 0x80, 0x80, 0x80, 0xC0,	// Г
0x07, 0x8A, 0xF2, 0x82, 0xFF,	// Д
0xFE, 0x92, 0x92, 0x92, 0x82,	// Е
0xEE, 0x10, 0xFE, 0x10, 0xEE,	// 0хС6 Ж
0x92, 0x92, 0x92, 0x92, 0x6C,	// З
0xFE, 0x08, 0x10, 0x20, 0xFE,	// И
0x3E, 0x84, 0x48, 0x90, 0x3E,	// Й
0xFE, 0x10, 0x28, 0x44, 0x82,	// К
0x04, 0x82, 0xFC, 0x80, 0xFE,	// Л
0xFE, 0x40, 0x20, 0x40, 0xFE,	// М
0xFE, 0x10, 0x10, 0x10, 0xFE,	// Н
0x7C, 0x82, 0x82, 0x82, 0x7C,	// О
0xFE, 0x80, 0x80, 0x80, 0xFE,	// П
0xFE, 0x90, 0x90, 0x90,	0x60,	// Р		
0x7C, 0x82, 0x82, 0x82,	0x44,	// С		
0x80, 0x80, 0xFE, 0x80, 0x80,	// Т			
0xE2, 0x14, 0x08, 0x10,	0xE0,	// У		
0x18, 0x24, 0xFE, 0x24, 0x18,	// ф	
0xC6, 0x28, 0x10, 0x28, 0xC6,	// Х			
				
0xFE, 0x02, 0x02, 0x02, 0xFF,	// Ц
0xE0, 0x10, 0x10, 0x10,	0xFE,	// Ч	
0xFE, 0x02, 0xFE, 0x02, 0xFE,	// Ш
0xFE, 0x02, 0xFE, 0x02, 0xFF,	// Щ			
0x80, 0xFE, 0x12, 0x12, 0x0C,	// Ъ
0xFE, 0x12, 0x0C, 0x00, 0xFE,	// Ы
0xFE, 0x12, 0x12, 0x12, 0x0C,	// Ь	
0x44, 0x82, 0x92, 0x92,	0x7C,	// Э
0xFE, 0x10, 0x7C, 0x82,	0x7C,	// Ю
0x62, 0x94, 0x98, 0x90, 0xFE,	// Я
0x04, 0x2A, 0x2A, 0x2A, 0x1E,	// а
0x3C, 0x52, 0x52, 0x92, 0x8C,	// б
0x3E, 0x2A, 0x2A, 0x14, 0x00,	// в
0x3E, 0x20, 0x20, 0x20, 0x30,	// г
				

0x07, 0x2A, 0x32, 0x22, 0x3F,	// д
0x1C, 0x2A, 0x2A, 0x2A, 0x18,	// е
0x36, 0x08, 0x3E, 0x08, 0x36,	// ж
0x22, 0x22, 0x2A, 0x2A, 0x14,	// з
0x3E, 0x04, 0x08, 0x10, 0x3E,	// и
0x1E, 0x42, 0x24, 0x48, 0x1E,	// й
0x3E, 0x08, 0x14, 0x22, 0x00,	// к
0x04, 0x22, 0x3C, 0x20, 0x3E,	// л
0x3E, 0x10, 0x08, 0x10, 0x3E,	// м
0x3E, 0x08, 0x08, 0x08, 0x3E,	// н
0x1C, 0x22, 0x22, 0x22, 0x1C,	// o
0x3E, 0x20, 0x20, 0x20, 0x3E,	// п
0x3E, 0x28, 0x28, 0x28, 0x10,	// р
0x1C, 0x22, 0x22, 0x22, 0x04,	// с

0x20, 0x20, 0x3E, 0x20, 0x20,	// т
0x30, 0x0A, 0x0A, 0x0A, 0x3C,	// у
0x38, 0x44, 0xFE, 0x44, 0x38,	// Ф
0x22, 0x14, 0x08, 0x14, 0x22,	// х
0x3E, 0x02, 0x02, 0x02, 0x3F,	// ц
0x30, 0x08, 0x08, 0x08, 0x3E,	// ч
0x3E, 0x02, 0x3E, 0x02, 0x3E,	// ш
0x3E, 0x02, 0x3E, 0x02, 0x3F,	// щ
0x20, 0x3E, 0x0A, 0x0A, 0x04,	// ъ
0x3E, 0x0A, 0x04, 0x00, 0x3E,	// ы
0x3E, 0x0A, 0x0A, 0x04, 0x00,	// ь
0x14, 0x22, 0x2A, 0x2A, 0x1C,	// э
0x3E, 0x08, 0x1C, 0x22, 0x1C,	// ю
0x10, 0x2A, 0x2C, 0x28, 0x3E,	// я

};

// параметр
//функции вывод символов/строк
// если при использовании функции необходим новый шрифт - измените параметр.
bit 
type, 		// тип выводимого  0-символ/1-число
inv;		// инверсия символа  0-нормальный/1-инверсный

char 
width,		// ширина символа  0-1-нормальный/2-7 ширина
height;		// высота символа  0-нормальный/1-двойная высота/>1-четырех кратная высота


// cod - символ или цифра
// type - тип (1-цифра или 0-символ)
// inv - 0-нет 1-да, инвертирование знакоместа
// width - ширина 0-стандартная 7-максимальная.
// height - высота 0-стандартная, 1-двойная, 2-четверная


 char 
 angleType, 	// [тип углов, прямоугольник, бар]0-прямые, 1-скругленные
 lineType, 	// [тип линии примитива]0-белая 1- сплошная, от 2 и более варианты
 tipBor, 	// [толщина бордюра] 0- нет (заливка все равно будет делать 1 пиксел отступа), 1,2,3 толщина 
 fillType; 	// [тип заливки]0-нет заливки,1-белая,2-черная, 3 и более варианты

// массив для сообщений храняшизся в озу, возможна инициализация
// программа корректирует данные, а функция выводит на дисплей сообщения
char ca[] = "Бегущая строка в ОЗУ данные --> 32.7С";
//char ca[20]; возможный вариант без инициализации, но с указанием размера



//
void int_LCD (void)
{
  uint8_t buf[5] = 
  {
    UC1601_SET_BR_9, // BIAS 6
    UC1601_SET_PM, // настройка Vbias
    120, // 124 
    UC1601_SET_LC20, // настройка типа разветки свеху в низ, и слево на право 120/77
    UC1601_SET_DC2_EN
  };
  
  UC1601CmdWrite(UC1601_SET_SC);
	delay_ms(10);			// 16 000 = 1мС - 10мС	
	i2c_start(0x70,0,0);	// 0x70 адрес индикатора, 0-команда, 0-запись
	i2c_write(0b11101011); 	// BIAS 6
	i2c_write(0b10000001);  // настройка Vbias
	i2c_write(120);			// 124
  UC1601CmdBurst(buf,5);
	clear_LCD(0); 			 // очистка дисплея
}



// Очистка дисплея
//type==0 очистка,
//type==1 черный,
//type==2 серое 50%.
void clear_LCD (char type)
{
  uint16_t i;

  uint8_t buf[1056] = 
  {
    UC1601_SET_PA,
    UC1601_SET_CA_LSB,
    UC1601_SET_CA_MSB
  };
    
  UC1601CmdBurst(buf,3);
  
  if (type == 0)	
  {
    for(i = 0; i < 1056; i++) buf[i] = 0;
  }
	else 	if(type == 1) 
  {
    for(i = 0; i < 1056; i++) buf[i] = 0xff;
  }
  else 	
  {
    for (i = 0; i < 528; i++)	
    {
      buf[2*i] = 0xaa;
      buf[2*i + 1] = 0x55;
    }
  }
	UC1601DataBurst(buf,1056);
}

// String_LCD - вывод на дисплей строк с указанной точки дисплея
void String_LCD(const char *str,char x,char y)// , char cстрока, ширина, высота, центрирование (22 символа в строке)
{
	int ptr = 0; 		// инициализировать указатель
	if (x & 0x80)	// тестировать на -1 авто центрирование
	{// вычисление начала вывода строки 
		if(width == 0) width++;				//width - не должен быть равен 0
		while(str[ptr] != 0) ptr++;		// возвращает длину строки
		ptr = (dispX - ((ptr*width)*5 + ptr + 1));	// 
		if (ptr < 0) ptr = 0;					// установка начала если строка превышает размер дисплея
		ptr = ptr/2;
		cursorG_LCD(ptr, y);
		ptr = 0;
	}
	else cursorG_LCD(x, y);	
	type = 0; //???
	while(str[ptr] != 0) Symbol_LCD(str[ptr++]);	// цикл передачи символов пока не достигнут ноль (конец строки) 	
}//

// Stringp_LCD вывод строки с текущей точки
void Stringp_LCD(const char *str)// , cтрока, инверсия, ширина, высота
{
	int ptr = 0; // инициализировать указатель
  
	type = 0; //???
	while(str[ptr] != 0) Symbol_LCD(str[ptr++]);	// цикл передачи символов пока не достигнут ноль (конец строки) 	
}
//


//----------------------------------------------------------------------------	
// прорисовка символа
// X - 0-dispY; Y - 0-64
// codv - символ или цифра
void Symbol_LCD (char codv)
{
#define bitset(var,bitno) ((var) |= 1<<(bitno)); 	// установить
unsigned char ZD,b,a,c,z,fon0,fon1,fon2,bufr,widthf,heightf;
int cod;
volatile unsigned long buf, fon, risun, mask, mask1, mask2;
	cod=codv;
	widthf=width+1;						// для использования диапазона от 0
	widthf=widthf & 0b00000111; 		// ограничит ширину до 7
	heightf=height & 0b00000001; 		// ограничит высоту до 1

// преобразование символ или число
	if (type) 
	{
		if (cod>9)cod=0x32; 	// если число больше 9 выводить звездочку
		else cod=(cod*5)+0x50;	// полученные данные цифра
	}	
	else cod=(cod-0x20)*5; 		// из символа координаты	

//  вычисление смещение для сдвига
		if ((cursorY/8)==0)ZD=cursorY%8;// 
		else ZD=(cursorY%8)+8;			//
// прорисовка символа
	for (b=0;b<5;b++) 			// 5-ширина символа в знакогенераторе
	{

// формирование символов двойной высоты
		if (heightf==0)// если height выводить символы двойной высоты		
		{
			if (inv)
			{
				bufr=~font[cod+b];
				risun=bufr;
			}	
			else risun= font[cod+b];// загрузка данных	//*********************************
			risun=risun<<ZD;
			if (inv) bitset(risun,ZD-1);  // установить бит ZD
		}	
		else//символы двойной высоты
		{
			if (inv) buf=~font[cod+b];	//*********************************
			    else buf= font[cod+b];
			mask=0b00000001;
			risun=0;
			for(z=0;z<8;z++)
			{
				risun=risun>>2;
				if (buf & mask)
				{
					risun |= 0b1100000000000000;
				}	
				mask=mask<<1;
			}
			risun=risun<<ZD;//+1 коррекция сдвига символов двойной высоты (одна строка снизу)
			if (inv)bitset(risun,ZD-1); // установить бит ZD
		}	

// формирование ширины символа
		for (a=1;a<widthf;a++)
		{
			//-----------------------------------------------------------
			if(cursorX>131) // прерывание если курсор вылез за размер 131
			{
				i2c_start(0x70,0,0); 	 	// 
				i2c_write(0b10001001);  	// движение по строкам вверх
				i2c_stop(); 			 	//
				return; 	 				//
			}
			//-----------------------------------------------------------
			cursorG_LCD (cursorX,cursorY);
// чтение изображения с индиктора
			i2c_start(0x70,0,0); 	 	// 
			i2c_write(0b10001011);  	// движение по строкам вниз
			i2c_stop();
			
			i2c_start(0x70,1,1);		// 0x70 адрес индикатора, данные, чтение
			i2c_read_ack(); 
			fon0=i2c_read_ack(); 			
			fon1=i2c_read_ack(); 		
			fon2=i2c_read_ack(); 
			fon=i2c_read_noack();
			i2c_stop();
			fon=fon<<8;			
			fon+=fon2;
			fon=fon<<8;
			fon+=fon1;
			fon=fon<<8;
			fon+=fon0;
			
// формирование маски для очистки
			 
			if (inv || cursorY==0)// если инверсия и самая первая строка
			{	
				if (heightf==0) 	 mask=0b11111111111111111111111100000000;
						    else mask=0b11111111111111110000000000000000;
			}
			else
			{			    
				if (heightf==0) 	 mask=0b11111111111111111111111100000000;
						    else mask=0b11111111111111110000000000000000;
			}	
					    
			
			// сдвиг маски взависимости от положения символа
			for	(c=0;c<ZD;c++)	
		{
			mask1=(mask << 1);
			mask2=(mask >> 31);
			mask=mask1 | mask2;
		//	mask=(mask << 1)|(mask >> 31); // вращение вправо
		}
			// наложение маски на фон
			fon=fon&mask; // стирание фона маской
			// наложение символа на фон				
			buf=risun|fon;

			// коррекция курсора
			cursorG_LCD (cursorX,cursorY);
			cursorX++;

			i2c_start(0x70,0,0); 	 	// 
			i2c_write(0b10001011);  	// движение по строкам в обратную сторону
			i2c_stop();
			
			i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
			i2c_write(buf);
			i2c_write(buf=buf>>8);
			i2c_write(buf=buf>>8);
			i2c_write(buf>>8);
			i2c_stop();	

		}
	}	

	cursorG_LCD (cursorX,cursorY); //коррекция курсора
// формирование разделительной линии
// чтение изображения с индиктора
	i2c_start(0x70,0,0); 	 	// 
	i2c_write(0b10001011);  	// движение по строкам В ПЕРЕД
	i2c_stop();
	
	i2c_start(0x70,1,1);		// 0x70 адрес индикатора, данные, чтение
	i2c_read_ack(); 
	fon0=i2c_read_ack(); 			
	fon1=i2c_read_ack(); 		
	fon2=i2c_read_ack(); 
	fon=i2c_read_noack();
	i2c_stop();
	fon=fon<<8;			
	fon+=fon2;
	fon=fon<<8;
	fon+=fon1;
	fon=fon<<8;
	fon+=fon0;


// формирование маски для очистки
	if (inv==0)
	{
		if (heightf==0) mask=0b11111111111111111111111100000000;
		else mask=0b11111111111111110000000000000000;
	// сдвиг маски взависимости от положения символа
	for	(c=0;c<ZD;c++)
		{
			mask1=(mask << 1);
			mask2=(mask >> 31);
			mask=mask1 | mask2;
		//	mask=(mask << 1)|(mask >> 31); // вращение вправо
		}
	fon=fon&mask;// наложение маски на фон
	}
	else
	{
		if (heightf==0) mask=0b00000000000000000000000011111111;
		else mask=0b00000000000000001111111111111111;
	// сдвиг маски взависимости от положения символа
	for	(c=0;c<ZD;c++)
		{
			mask1=(mask << 1);
			mask2=(mask >> 31);
			mask=mask1 | mask2;
		//	mask=(mask << 1)|(mask >> 31); // вращение вправо
		}	
	fon=fon|mask;// наложение маски на фон
	}
	buf=fon;

// коррекция курсора
	cursorG_LCD (cursorX,cursorY);
	cursorX++;

	i2c_start(0x70,0,0); 	 	// 
	i2c_write(0b10001011);  	// движение по строкам в обратную сторону
	i2c_stop();
	
	i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
	i2c_write(buf);
	i2c_write(buf=buf>>8);
	i2c_write(buf=buf>>8);
	i2c_write(buf>>8);
	i2c_stop();	

	i2c_start(0x70,0,0); 	 	// 
	i2c_write(0b10001001);  	// движение по строкам в обратную сторону
	i2c_stop();

}//



// StringS_LCD - вывод на дисплей строк с указанной строки дисплея
void StringS_LCD(const char *str,char X,char Y)
{
	int ptr = 0; 		// инициализировать указатель
	if (X&0b10000000)	// тестировать на -1 авто центрирование
	{// вычисление начала вывода строки 
		if(width==0)width++;				//width - не должен быть равен 0
		while (str[ptr] != 0) ptr++;		// возвращает длину строки
		ptr=(dispX-((ptr*width)*5+ptr+1));	// 
		if (ptr<0) ptr=0;					// установка начала если строка превышает размер дисплея
		ptr=ptr/2;
		cursorG_LCD (ptr, Y*8);
		ptr = 0;
	}
	else cursorG_LCD (X, Y*8);	
	type=0;
	while (str[ptr] != 0) SymbolS_LCD (str[ptr++]);	// цикл передачи символов пока не достигнут ноль (конец строки) 	
}//

// Stringp_LCD вывод строки с текущей точки
void StringpS_LCD(const char *str)
{
	int ptr = 0; // инициализировать указатель
	type=0;
	while (str[ptr] != 0) SymbolS_LCD (str[ptr++]);	// цикл передачи символов пока не достигнут ноль (конец строки) 	
}
//


// прорисовка символа
void SymbolS_LCD (char codv)
{
int cod;
unsigned char b,c,mask,buf,z,widthf,heightf;
volatile unsigned long risun;
	cod=codv;
	widthf=width+1;						// для использования диапазона от 0
	widthf=widthf & 0b00000111; 		// ограничит ширину до 7
	heightf=height & 0b00000011; 		// ограничит ширину до 1
// преобразование символ или число
	if (type) 
	{
		if (cod>9)cod=0x32; 	// если число больше 9 выводить звездочку
		else cod=(cod*5)+0x50;	// полученные данные цифра
	}	
	else cod=(cod-0x20)*5; 		// из символа координаты

// прорисовка символа
	for (b=0;b<5;b++) 			// 5-ширина символа в знакогенераторе
	{
	// формирование картинки для прорисовки
		buf=font[cod+b];		// загрузка данных
		if (inv)buf=~buf;

		if (heightf)
		{
			mask=0b10000000;
			risun=0;
			for(z=0;z<8;z++)
			{
				if (heightf>1)risun=risun<<4;
				else risun=risun<<2;
				if (buf & mask)
				{
					if (heightf>1)risun |= 0b0000000000000000000000000001111;
					else risun |= 0b0000000000000000000000000000011;
				}	
				mask=mask>>1;
			}
		}
		else
		{
			risun=buf;
		}	

// рисования одной вертикального столбца
		//	cursorG_LCD (cursorX,cursorY);
		c=1;
		do
		{
			if (heightf)
			{
		//	cursorG_LCD (cursorX,cursorY);
				i2c_start(0x70,0,0); 	 	// 
				i2c_write(0b10001011);  	// движение по строкам вверх
				i2c_stop();
				i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
				i2c_write(risun);
				i2c_write(risun>>8);
				if (heightf>1)
				{
					i2c_write(risun>>16);
					i2c_write(risun>>24);
				}	
				i2c_stop();
				cursorX++;
				cursorG_LCD (cursorX,cursorY);
			}
			else
			{
				i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
				i2c_write(risun); 		 	//
				i2c_stop(); 	 			//
				cursorX++; 	 				//
			}				
			//	cursorX++;
			//	cursorG_LCD (cursorX,cursorY);
			++c;
			//-----------------------------------------------------------
			if(cursorX>131) // прерывание если курсор вылез за размер 131
			{
				i2c_start(0x70,0,0); 	 	// 
				i2c_write(0b10001001);  	// движение по строкам вверх
				i2c_stop(); 			 	//
				return; 	 				//
			}
			//-----------------------------------------------------------
		}	
		while (c<widthf);
	}
// формирование разделительной линии
	if (heightf)
	{
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
		if (inv)
		{
			i2c_write(0xff);
			i2c_write(0xff);
			if (heightf>1)
			{ 
				i2c_write(0xff);
				i2c_write(0xff);
			}	
		}	
		else
		{
			i2c_write(0);
			i2c_write(0);
			if (heightf>1)
			{ 
				i2c_write(0);
				i2c_write(0);
			}	
		}	

		i2c_stop();
	}
	else
	{
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
		if (inv)i2c_write(0xff);
		else i2c_write(0);
		i2c_stop();
	}
	
// установка нормального режима
		i2c_start(0x70,0,0); 	 	// 0x70 адрес индикатора, команда, запись 
		i2c_write(0b10001001);  	// движение по строке слево направо
		i2c_stop();
		cursorX++;
		cursorG_LCD (cursorX,cursorY);				

}//	










				

//при вызове функции в окно бегущей строки выводиться 1 следующий символ.
//в цикле вывода информации должна быть одна строка
//строка выезжает в окно, провегает окно, и начинается заново
//[строка]
//[длина]
//[столбец начала]
//[строка 0-7] 
void ticker_LCD (const char *str,char dlinok,char start,char Y)
{
char a,b,X;
char dlinst=0; 					// длина строки в символах (6 столбцов)
// pos - позиция бегущей строки(внешняя переменная)
char colcim; 					// количество выводимых символов
int ptr=0,symb; 					// инициализировать указатель

// вычисление длины строки
	while (str[dlinst++] != 0); // длина строки

// вычисление выводимой длины строки за 1 раз
	if (dlinst>dlinok)//для строк больших размера окна вывода
	{
		if (pos<dlinok){ptr=0; colcim=pos; X=start+(dlinok-pos)*6;}
		else 
		if (pos<dlinst){ptr=pos-dlinok; colcim=dlinok; X=start;}
		else		
		{ptr=dlinst-dlinok+(pos-dlinst); colcim=dlinok-(pos-dlinst+1); X=start;}
	}
	else//для строк равных и меньше размера окна вывода
	{
		if  (pos<dlinst){ptr=0;	colcim=pos; X=start+(dlinok-pos)*6;}
		else 
		if  (pos<dlinok){ptr=0;	colcim=dlinst-1; X=start+(dlinok-pos)*6;}
		else  		
		{ptr=dlinst-dlinok+(pos-dlinst); colcim=dlinok-(pos-dlinst+1); X=start;}
	}		
// коррекция положениея начала вывода строки	
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();							//

	for (b=0;b<colcim;b++) // цикл выводимых символов
	{
		symb=(str[ptr+b]-0x20)*5; 	// пребразование кода символа в позицию для знакогенератора	
	// по байтное чтение рисунка символа и вывод на дисплей	
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись		
		for (a=0;a<5;a++) i2c_write(font[symb+a]);
		i2c_write(0);
		i2c_stop();
	}
	if (pos>=dlinst)
	{		
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись		
		for (a=0;a<5;a++) i2c_write(0);
		i2c_write(0);
		i2c_stop();
	}	
	pos++;
	if (dlinst==(pos-dlinok)) pos=0; // когда указатель дойдет до (pos-dlinok) установить в ноль	
}//


// при вызове функции в окно бегущей строки выводиться 1 следующий символ.
// в цикле вывода информации должна быть одна строка
// строка выезжает в окно, провегает окно, и начинается заново
// [строка][длина][столбец начала][строка 0-7] 
void tickerB_LCD (char dlinok,char start,char Y)
{
char a,b,X;
char dlinst=0; 					// длина строки в символах (6 столбцов)
// posB - позиция бегущей строки(внешняя переменная)
char colcim; 					// количество выводимых символов
int ptr=0,symb; 					// инициализировать указатель

// вычисление длины строки
	while (ca[dlinst++] != 0); // длина строки

// вычисление выводимой длины строки за 1 раз
	if (dlinst>dlinok)//для строк больших размера окна вывода
	{
		if (posB<dlinok){ptr=0; colcim=posB; X=start+(dlinok-posB)*6;}
		else 
		if (posB<dlinst){ptr=posB-dlinok; colcim=dlinok; X=start;}
		else		
		{ptr=dlinst-dlinok+(posB-dlinst); colcim=dlinok-(posB-dlinst+1); X=start;}
	}
	else//для строк равных и меньше размера окна вывода
	{
		if  (posB<dlinst){ptr=0;	colcim=posB; X=start+(dlinok-posB)*6;}
		else 
		if  (posB<dlinok){ptr=0;	colcim=dlinst-1; X=start+(dlinok-posB)*6;}
		else  		
		{ptr=dlinst-dlinok+(posB-dlinst); colcim=dlinok-(posB-dlinst+1); X=start;}
	}		
// коррекция положения начала вывода строки	
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();							//

	for (b=0;b<colcim;b++) // цикл выводимых символов
	{
		symb=(ca[ptr+b]-0x20)*5; 	// пребразование кода символа в позицию для знакогенератора	
	// по байтное чтение рисунка символа и вывод на дисплей	
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись		
		for (a=0;a<5;a++) i2c_write(font[symb+a]);
		i2c_write(0);
		i2c_stop();
	}
	if (posB>=dlinst)
	{		
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись		
		for (a=0;a<5;a++) i2c_write(0);
		i2c_write(0);
		i2c_stop();
	}	
	posB++;
	if (dlinst==(posB-dlinok)) posB=0; // когда указатель дойдет до (posB-dlinok) установить в ноль	
}//

//----------------------------------------------------------------------------	
// X - 0-dispY; Y - 0-64
// функция установки курсора
void cursorS_LCD (char X,char Y)
{
		Y=Y*8;
		cursorY=Y;							// запоминаем значение во внешних
		cursorX=X;							// переменных
		if ((Y/8)==0)Y=(Y/8);				// 
		else Y=(Y/8)-1;						//
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();							//
}			
//






//----------------------------------------------------------------------------	
// X - 0-dispY; Y - 0-64
// функция установки графического курсора
void cursorG_LCD (char X,char Y)
{
		cursorY=Y;							// запоминаем значение во внешних
		cursorX=X;							// переменных
		if ((Y/8)==0)Y=(Y/8);				// 
		else Y=(Y/8)-1;						// 
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();							//
}			
//
				
// рисование точки
//[тип]0-белая тоска,1-черная
//[координата по X] 0-131
//[координата по Y] 0-63
void point (char lineType, char X, char Y)
{
#define bitset(var,bitno) ((var) |= 1<<(bitno)); 	// установить
#define bitclr(var,bitno) ((var) &= ~(1<<(bitno)));	// сбросить
char fon,Ya;
// определение координат требуемого байта и установка курсора
		Ya=Y%8; 							// положение точки в байте.
		if ((Y/8)==0)Y=(Y/8);				// 
		else Y=(Y/8);						// 

// установка курсора		
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();		
//чтение байта с индикатора
		i2c_start(0x70,1,1);		// 0x70 адрес индикатора, данные, чтение
		i2c_read_ack(); 
		fon=i2c_read_noack();
		i2c_stop();
// модификация байта	
		if (lineType) {bitset(fon,Ya);}
		else {bitclr(fon,Ya);}
// коррекция курсора по X
		i2c_start(0x70,0,0);				// 0x70 адрес индикатора, команда, запись
	    i2c_write(0b10110000 | Y);			// установка страницы
	    i2c_write(X & 0b00001111);			// установка столбца
	    i2c_write((X >> 4) | 0b00010000);	//
		i2c_stop();
// запись модибицированного байта
		i2c_start(0x70,1,0);		// 0x70 адрес индикатора, данные, запись
		i2c_write(fon);
		i2c_stop();

}//	


// рисование линии алгоритм Брезенхема
// x0 y0 x1 y1 - кординаты линии
void line(char x0, char y0, char x1, char y1)
{
#define abs(a)	(((a)> 0) ? (a) : -(a))
     char step, t , vid;
     int deltax, deltay, error;
     char x, y;
     char ystep;

     step = (abs(y1-y0)>abs(x1-x0)); // определяем логику
     
     if (step)
     { // перестановка x и y
         t=x0; x0=y0; y0=t;
         t=x1; x1=y1; y1=t;
     }
     if (x0>x1) 
     {  // перестановка окончание
         t=x0; x0=x1; x1=t;
         t=y0; y0=y1; y1=t;
     }
     
     deltax = x1-x0;
     deltay = abs(y1-y0);
     error = 0;
     y = y0;
     
     if (y0<y1) ystep = 1; else ystep = -1;
     for (x=x0;x<x1+1;x++)
     {
	     if (lineType==0)vid=0;
	     	else if (lineType==1)vid=1;
		     	else vid =x&lineType-1; // формирование типа заливки
         if (step) point(vid,y,x); else point(vid,x,y);
         error += deltay;
         if ((error<<1)>=deltax)
         {
             y += ystep;
             error -= deltax;
         } // условие 
     } // цикл
} // линия 

//---------------------------------------------------------------------------------
// рисование прямоугольника (есть ограничение на задание координат - задание кординат вехний левый угол + ширина и высота)
//5-[x0][y0] - коорината нижнего левого угла
//6-[ширина] - ширина прямоугольника
//7-[высота] - высота прямоугольника
//---------------------------------------------------------------------------------
void rectangle (char x0, char y0, char sh, char vs)
{
#define abs(a)	(((a)> 0) ? (a) : -(a))
char a,b,x1,y1;
// для фунции заливки
char fill=0;
char step, t, vid;
int deltax, deltay, error;
char x, y, poi;
char ystep;
char  zx0,zy0,zy1,zx1;
static bit RIS; // бит для функции заливки
     
		x1=x0+sh;
		y1=y0+vs;		
		if (tipBor)
		{ // для толщины линии >0
			if (angleType==0) // для прямых углов
			{
				b=0;
				for (a=0;a<tipBor;a++)
				{
					line(x0+b, y0+a,  x1-b, y0+a);	
					line(x1-a, y0+b,  x1-a, y1-b);	
					line(x1-b, y1-a,  x0+b, y1-a);	
					line(x0+a, y1-b,  x0+a, y0+b);
					b++;
				}
				y0=y0+a-1;
				y1=y1-a+1;
				x0=x0+a-1;
				x1=x1-a+1;			
			}
			else 		// для закругленных
			{	
				b=0;
				for (a=0;a<tipBor;a++)
				{
					line(x0+4+b, y0+a,    x1-4-b, y0+a);	
					line(x1-a,   y0+4+b,  x1-a,   y1-4-b);	
					line(x1-4-b, y1-a,    x0+4+b, y1-a);	
					line(x0+a,   y1-4-b,  x0+a,   y0+4+b);
	
					point (lineType, x0+1+b, y0+2+b);	
					point (lineType, x0+1+b, y0+3+b);			
					point (lineType, x0+2+b, y0+1+b);	
					point (lineType, x0+3+b, y0+1+b);			
		
					point (lineType, x1-1-b, y0+2+b);	
					point (lineType, x1-1-b, y0+3+b);			
					point (lineType, x1-2-b, y0+1+b);	
					point (lineType, x1-3-b, y0+1+b);
		
					point (lineType, x1-1-b, y1-2-b);	
					point (lineType, x1-1-b, y1-3-b);
					point (lineType, x1-3-b, y1-1-b);	
					point (lineType, x1-2-b, y1-1-b);
					
					point (lineType, x0+1+b, y1-2-b);	
					point (lineType, x0+1+b, y1-3-b);			
					point (lineType, x0+2+b, y1-1-b);	
					point (lineType, x0+3+b, y1-1-b);
	
					b++;
				}
				y0=y0+a-1;
				y1=y1-a+1;
				x0=x0+a-1;
				x1=x1-a+1;
			}
		}		
		// заливка
		if (fillType>0)
		{
			for (a=1;a<(x1-x0);a++) // ширина заливки
			{
				zy0=y0+1;		// данные необходимо перед каждым входом 
				zy1=y1-1;		// обновлять
				zx1=zx0=x0+a;	//
		         // коррекция координат для заливки скругленных углов
		         if (angleType)
		         {
			          if (a>3) poi=sh-a;	// ширина заливки - указатель заливки 
				      else poi= a;
				      switch (poi)
				      {
					      case 1:
				      		zy0=zy0+3;
				      		zy1=zy1-3;	
				      		break; 
				      	  case 2:
				      	  case 3:
				   		  	zy0=zy0+1;
				   		  	zy1=zy1-1;
							break;
				      }			
			     }	
				// заливка
			     step = (abs(zy1-zy0)>abs(zx1-zx0)); // определяем логику
			     if (step)
			     { // перестановка x и y
			         t=zx0; zx0=zy0; zy0=t;
			         t=zx1; zx1=zy1; zy1=t;
			     }
			     if (zx0>zx1) 
			     {  // перестановка окончание
			         t=zx0; zx0=zx1; zx1=t;
			         t=zy0; zy0=zy1; zy1=t;
			     }
			     deltax = zx1-zx0;
			     deltay = abs(zy1-zy0);
			     error = 0;
			     y = zy0;
			     if (zy0<zy1) ystep = 1; else ystep = -1;
			     for (x=zx0;x<zx1+1;x++)
			     {   // тип заливки
				     if (fillType==1)vid=0;
				     else if (fillType==2)vid=1;
					     	else if (RIS)vid =(x&1); // формирование типа заливки
						     		else vid =!(x&1);
			         if (step) point(vid,y,x); else point(vid,x,y);
			         error += deltay;
			         if ((error<<1)>=deltax)
			         {
			             y += ystep;
			             error -= deltax;
			         }
			     } // 
				// формирование рисунка заливки
				if (fillType>2)
				{
					fill++;	
					if (fill==fillType-2){RIS=!RIS;fill=0;}
				}
			}
		}// заливка
}//	
				

//---------------------------------------------------------------------------------
// полоса загрузки индикатор линейный (есть ограничение на задание координат - задание кординат вехний левый угол + ширина и высота)
//angleType-[тип углов]0-прямые, 1-скругленные
//lineType-[тип линии]0-белая 1- сплошная, от 2 и более варианты
//fillType-[тип заливки]0-белая,1-черная, 2 и более варианты
//[x0][y0] - коорината вернего левого угла
//sh-[ширина] - ширина прямоугольника
//vs-[высота] - высота прямоугольника
//[vol] - уровень 0-100%
//---------------------------------------------------------------------------------
void strip ( char angleType,char lineType,char fillType, char x0, char y0, char sh, char vs, char vol)
{
#define abs(a)	(((a)> 0) ? (a) : -(a))
char a,x1,y1,width;
// для фунции заливки
char fill=0;
int step, t, vid;
int deltax, deltay, error;
int x, y, poi;
int ystep;
int zx0,zy0,zy1,zx1;
int dbar;
static bit RIS; // бит для функции заливки
     
		dbar=sh*vol/100; // вычисление длины бара
		
		x1=x0+sh;
		y1=y0+vs;		

			if (angleType==0) // для прямых углов
			{
					line(x0, y0,  x1, y0);	
					line(x1, y0,  x1, y1);	
					line(x1, y1,  x0, y1);	
					line(x0, y1,  x0, y0);
			}
			else 		// для закругленных
			{	
					line(x0+4, y0,    x1-4, y0);	
					line(x1,   y0+4,  x1,   y1-4);	
					line(x1-4, y1,    x0+4, y1);	
					line(x0,   y1-4,  x0,   y0+4);
	
					point (lineType, x0+1, y0+2);	
					point (lineType, x0+1, y0+3);			
					point (lineType, x0+2, y0+1);	
					point (lineType, x0+3, y0+1);			
		
					point (lineType, x1-1, y0+2);	
					point (lineType, x1-1, y0+3);			
					point (lineType, x1-2, y0+1);	
					point (lineType, x1-3, y0+1);
		
					point (lineType, x1-1, y1-2);	
					point (lineType, x1-1, y1-3);
					point (lineType, x1-3, y1-1);	
					point (lineType, x1-2, y1-1);
					
					point (lineType, x0+1, y1-2);	
					point (lineType, x0+1, y1-3);			
					point (lineType, x0+2, y1-1);	
					point (lineType, x0+3, y1-1);
			}
		
		// заливка
		width=x1-x0;
		RIS=0; // синхронизация заливки
		if (fillType>0)
		{
			for (a=1;a<(width);a++) // ширина заливки
	//		for (a=1;a<(dbar);a++) // ширина заливки
				{
				zy0=y0+1;		// данные необходимо перед каждым входом 
				zy1=y1-1;		// обновлять
				zx1=zx0=x0+a;	//
		         // коррекция координат для заливки скругленных углов
		         if (angleType)
		         {
			          if (a>3) poi=sh-a;	// ширина заливки - указатель заливки 
				      else poi= a;
				      switch (poi)
				      {
					      case 1:
				      		zy0=zy0+3;
				      		zy1=zy1-3;	
				      		break; 
				      	  case 2:
				      	  case 3:
				   		  	zy0=zy0+1;
				   		  	zy1=zy1-1;
							break;
				      }			
			     }	
				// заливка
			     step = (abs(zy1-zy0)>abs(zx1-zx0)); // определяем логику
			     if (step)
			     { // перестановка x и y
			         t=zx0; zx0=zy0; zy0=t;
			         t=zx1; zx1=zy1; zy1=t;
			     }
			     if (zx0>zx1) 
			     {  // перестановка окончание
			         t=zx0; zx0=zx1; zx1=t;
			         t=zy0; zy0=zy1; zy1=t;
			     }
			     deltax = zx1-zx0;
			     deltay = abs(zy1-zy0);
			     error = 0;
			     y = zy0;
			     if (zy0<zy1) ystep = 1; else ystep = -1;
			     for (x=zx0;x<zx1+1;x++)
			     {   // тип заливки
				     if (a<dbar)
				     {
					     if (fillType==1)vid=0;
					     else if (fillType==2)vid=1;
						     	else if (RIS)vid =(x&1); // формирование типа заливки
							     		else vid =!(x&1);
			      	 }
			     	  else vid=0;   
			        
			         if (step) point(vid,y,x); else point(vid,x,y);
			         error += deltay;
			         if ((error<<1)>=deltax)
			         {
			             y += ystep;
			             error -= deltax;
			         }
			     } // 
				// формирование рисунка заливки
				if (fillType>2)
				{
					fill++;	
					if (fill==fillType-2){RIS=!RIS;fill=0;}
				}
			}
		}// заливка
}//	
				





