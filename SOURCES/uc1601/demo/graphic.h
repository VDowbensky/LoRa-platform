/* Графический модуль для работы с графическими индикаторами на драйвере
UC1601S. Тестирование проводилась на индикторе RDX0154/120/077
HI-TECH Software PICC-18 v9.66/9.80
MPLAB v8.85
Геннадий Чернов, Днепропетровск 2012
Gennady Chernov, Dnepropetrovsk 2012
версия V2.0.1
проект ИЛЛИССИ
*/


// выбор дисплея, нужный раскоментировать
#define LCD154 // для работы с 154 индикатором
//#define LCD077
//#define LCD120
//
#ifdef LCD154 //132-64
#define dispX 131
#define dispY 63
#endif//

#ifdef LCD120//64-32
#define dispX 63
#define dispY 31
#endif//

#ifdef LCD077//128-64
#define dispX 127
#define dispY 63
#endif//

// для работы с I2C
void I2C_Open (unsigned int FCLOCK);// инициализация, значение частоты шины в килогерцах (100, 300, 400)
void i2c_idle (void); // провекра на готовность I2C к работе
void i2c_stop (void); // формирование стоп
char i2c_start (char adres, char C_D, char R_W); // адрес устройства и управление младшими битами
char i2c_restart (char adres, char C_D, char R_W);
char i2c_write (char data); //запись байта
char i2c_read_ack (void);	//чтение с подтверждением
char i2c_read_noack (void);	//чтение без подтверждения

// для работы с индикатором
void int_LCD (void); // инициализация
void clear_LCD (char type); // очиска всего дисплея
// установка курсора
void cursorG_LCD (char X,char Y);
void cursorS_LCD (char X,char Y);

// вывод символа или числа
// вывод символа с любой точки экрана
void Symbol_LCD (char codv);
// вывод строк
void String_LCD(const char *str,char X,char Y);//X == -1 центрирование, (если вы задаете значение -1 выполняется автоматическое выравнивание строки по центру экрана, строки длиной больше ширины экрана по центру не выравниваются).
void Stringp_LCD(const char *str);


// вывод символа или числа
// выводит символы с привязкой к строкам индикатора
void SymbolS_LCD (char codv);
void StringS_LCD (const char *str,char X,char Y);
void StringpS_LCD(const char *str);


// бегущая строка
void ticker_LCD (const char *str,char start,char ends,char Y); //с ПЗУ
void tickerB_LCD (char dlinok,char start,char Y);// с озу


// для работы с индикатором графические примитивы
void point (char type, char X, char Y); 
void line(char x0, char y0, char x1, char y1);
void rectangle (char x0, char y0, char sh, char vs);
void strip (char angleType,char lineType,char fillType, char x0, char y0, char sh, char vs, char vol);


