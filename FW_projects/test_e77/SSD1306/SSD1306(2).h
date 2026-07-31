#ifdef   reset
extern sbit RST;
#endif



#ifdef  i2c_soft
#define I2C__Start Soft_I2C_Start
#define I2C__Write  Soft_I2C_Write
#define I2C__Stop   Soft_I2C_Stop
#define  I2C__Init()  Soft_I2C_Init()

extern sbit Soft_I2C_Scl;
extern sbit Soft_I2C_Sda;
extern sbit Soft_I2C_Scl_Direction;
extern sbit Soft_I2C_Sda_Direction;


#else
#define I2C__Start    I2C1_Start
#define I2C__Write    I2C1_Wr
#define I2C__Stop     I2C1_Stop
#define  I2C__Init()  I2C1_Init(400000)

#endif





#define SSD1306_I2C_Address                        0X7A //0x78    //Arduino 3C=78*2

#define DAT                                       0x60
#define CMD                                       0x00



#define Set_Lower_Column_Start_Address_CMD        0x00 
#define Set_Higher_Column_Start_Address_CMD       0x10
#define Set_Memory_Addressing_Mode_CMD            0x20
#define Set_Column_Address_CMD                    0x21
#define Set_Page_Address_CMD                      0x22

#define ACTIVATE_SCROLL                           0x2F
#define DEACTIVATE_SCROLL                         0x2E
#define SET_VERTICAL_SCROLL_AREA                  0xA3
#define RIGHT_HORIZONTAL_SCROLL                   0x26
#define LEFT_HORIZONTAL_SCROLL                    0x27
#define VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL      0x29
#define VERTICAL_AND_LEFT_HORIZONTAL_SCROLL      0x2A


#define Set_Display_Start_Line_CMD                0x40
#define Set_Contrast_Control_CMD                  0x81
#define Set_Charge_Pump_CMD                       0x8D
#define Set_Segment_Remap_CMD                     0xA0
#define Set_Entire_Display_ON_CMD                 0xA4
#define Set_Normal_or_Inverse_Display_CMD         0xA6
#define Set_Multiplex_Ratio_CMD                   0xA8
#define Set_Display_ON_or_OFF_CMD                 0xAE
#define Set_Page_Start_Address_CMD                0xB0
#define Set_COM_Output_Scan_Direction_CMD         0xC0
#define Set_Display_Offset_CMD                    0xD3
#define Set_Display_Clock_CMD                     0xD5
#define Set_Pre_charge_Period_CMD                 0xD9
#define Set_Common_HW_Config_CMD                  0xDA
#define Set_VCOMH_Level_CMD                       0xDB
#define Set_NOP_CMD                               0xE3

#define Horizontal_Addressing_Mode                0x00
#define Vertical_Addressing_Mode                  0x01
#define Page_Addressing_Mode                      0x02

#define Disable_Charge_Pump                       0x00
#define Enable_Charge_Pump                        0x04
                                                                             
#define Column_Address_0_Mapped_to_SEG0           0x00
#define Column_Address_0_Mapped_to_SEG127         0x01

#define Normal_Display                            0x00
#define Entire_Display_ON                         0x01
                                                    
#define Non_Inverted_Display                      0x00
#define Inverted_Display                          0x01
                                               
#define Display_OFF                               0x00
#define Display_ON                                0x01

#define Scan_from_COM0_to_63                      0x00
#define Scan_from_COM63_to_0                      0x08

 //tamaño de la pantalla
#define x_size                                    128
#define x_max                                     x_size
#define x_min                                     0
#define y_size                                    64
#define y_max                                     8
#define y_min                                     0

#define ON                                        1
#define OFF                                       0


#define YES                                       1
#define NO                                        0


#define ROUND                                     1
#define SQUARE                                    0

#define buffer_size                               1024//(x_max * y_max)



unsigned char buffer[buffer_size];


void OLED_init();
void OLED_reset_sequence();
void OLED_write(unsigned char value, unsigned char type);
void OLED_gotoxy(unsigned char x_pos, unsigned char y_pos);
void OLED_fill(unsigned char bmp_data);
void OLED_clear_screen();
void OLED_clear_buffer();
void OLED_cursor(unsigned char x_pos, unsigned char y_pos);
void OLED_draw_bitmap(unsigned char xb, unsigned char yb, unsigned char xe, unsigned char ye, unsigned char *bmp_img);
void OLED_print_char(unsigned char x_pos, unsigned char y_pos, unsigned char ch,const unsigned char *font_t);
void OLED_print_text(unsigned char x_pos, unsigned char y_pos, unsigned char *ch,const unsigned char *font_t);
void OLED_print_short(unsigned char x_pos, unsigned char y_pos, signed int value,const unsigned char *font_t);
void Draw_Pixel(unsigned char x_pos, unsigned char y_pos, unsigned char colour);
void Draw_Line(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char colour);
void Draw_Rectangle(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char fill, unsigned char colour, unsigned char type);
void Draw_Circle(signed int xc, signed int yc, signed int radius, unsigned char fill, unsigned char colour);
void  OLED_scroll_der(unsigned char inicio, unsigned char fin);
void  OLED_scroll_IZQ(unsigned char inicio, unsigned char fin);
void  OLED_scroll_diag_der(unsigned char inicio, unsigned char fin);
void  OLED_scroll_diag_izq(unsigned char inicio, unsigned char fin);
void  OLED_stop_scroll();

