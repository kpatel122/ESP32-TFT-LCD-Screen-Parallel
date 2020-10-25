/***********************************************************************************
*This program is a demo of clearing screen to display red,green,blue.
*This demo was made for LCD modules with 8bit or 16bit data port.
*This program don't need to rely on any libraries and Can run directly.

* File                : simple_test.ino
* Hardware Environment: Arduino Mega2560
* Build Environment   : Arduino

*Set the pins to the correct ones for your development shield or breakout board.
*This demo use the BREAKOUT BOARD only and use these 16 data lines to the LCD,
//pin usage as follow:
//             CS  CD  WR  RD  RST  D0  D1  D2  D3  D4  D5  D6  D7  D8  D9  D10  D11  D12  D13  D14  D15 
//Arduino Mega 40  38  39  43  41   37  36  35  34  33  32  31  30  22  23  24   25   26   27   28   29
//             TP_IRQ  MOSI  MISO  TP_CS  EX_CLK
//Arduino Mega   44    51     50    53      52

//when using the BREAKOUT BOARD only and using these 8 data lines to the LCD,
//pin usage as follow:
//             CS  CD  WR  RD  RST  D0  D1  D2  D3  D4  D5  D6  D7  D8  D9  D10  D11  D12  D13  D14  D15 
//Arduino Mega 40  38  39  43  41   37  36  35  34  33  32  31  30  /   /    /    /    /    /    /    /
//             TP_IRQ  MOSI  MISO  TP_CS  EX_CLK
//Arduino Mega   44    51     50    53      52

*Remember to set the pins to suit your display module!
*
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**********************************************************************************/
//#define LCD_MODE 1    //1-8bit,0-16bit
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

#define TFT_BLACK       0x0000      /*   0,   0,   0 */

// Generic commands used by TFT_eSPI.cpp
#define TFT_NOP     0x00
#define TFT_SWRST   0x01

#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C
#define TFT_RAMRD   0x2E

#define TFT_MADCTL  0x36
#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_RGB 0x00

int32_t  addr_row, addr_col;        

/*dont use pins 6-11 on esp32*/
/*do use pins <32 so writes can be done on single register */
#define LCD_RD   23
#define LCD_WR   22     
#define LCD_RS   21        
#define LCD_CS   19       
#define LCD_REST 18

#define LCD_D0 16 //RX2
#define LCD_D1 17 //TX2
#define LCD_D2 25
#define LCD_D3 26
#define LCD_D4 27
#define LCD_D5 14
#define LCD_D6 12
#define LCD_D7 13

/*dont use pins 6-11 on esp32*/

unsigned int pins[8];


/*eSPI Library*/

 // Bit masks for ESP32 parallel bus interface
  uint32_t xclr_mask, xdir_mask; // Port set/clear and direction control masks

           // Lookup table for ESP32 parallel bus interface uses 1kbyte RAM,
  uint32_t xset_mask[256]; // Makes Sprite rendering test 33% faster, for slower macro equivalent
                           // see commented out #define set_mask(C) within TFT_eSPI_ESP32.h

                           

// Create a bit set lookup table for data bus - wastes 1kbyte of RAM but speeds things up dramatically
  // can then use e.g. GPIO.out_w1ts = set_mask(0xFF); to set data bus to 0xFF
  #define CONSTRUCTOR_INIT_TFT_DATA_BUS            \
  for (int32_t c = 0; c<256; c++)                  \
  {                                                \
    xset_mask[c] = 0;                              \
    if ( c & 0x01 ) xset_mask[c] |= (1 << LCD_D0); \
    if ( c & 0x02 ) xset_mask[c] |= (1 << LCD_D1); \
    if ( c & 0x04 ) xset_mask[c] |= (1 << LCD_D2); \
    if ( c & 0x08 ) xset_mask[c] |= (1 << LCD_D3); \
    if ( c & 0x10 ) xset_mask[c] |= (1 << LCD_D4); \
    if ( c & 0x20 ) xset_mask[c] |= (1 << LCD_D5); \
    if ( c & 0x40 ) xset_mask[c] |= (1 << LCD_D6); \
    if ( c & 0x80 ) xset_mask[c] |= (1 << LCD_D7); \
  }                                                \

  // Mask for the 8 data bits to set pin directions
  #define dir_mask ((1 << LCD_D0) | (1 << LCD_D1) | (1 << LCD_D2) | (1 << LCD_D3) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7))

  // Data bits and the write line are cleared to 0 in one step
  #define clr_mask (dir_mask) //| (1 << LCD_WR))

  // A lookup table is used to set the different bit patterns, this uses 1kByte of RAM
  #define set_mask(C) xset_mask[C] // 63fps Sprite rendering test 33% faster, graphicstest only 1.8% faster than shifting in real time

  // Real-time shifting alternative to above to save 1KByte RAM, 47 fps Sprite rendering test
  /*#define set_mask(C) (((C)&0x80)>>7)<<TFT_D7 | (((C)&0x40)>>6)<<TFT_D6 | (((C)&0x20)>>5)<<TFT_D5 | (((C)&0x10)>>4)<<TFT_D4 | \
                        (((C)&0x08)>>3)<<TFT_D3 | (((C)&0x04)>>2)<<TFT_D2 | (((C)&0x02)>>1)<<TFT_D1 | (((C)&0x01)>>0)<<TFT_D0
  //*/

#define RS_C GPIO.out_w1tc = (1 << LCD_RS); GPIO.out_w1tc = (1 << LCD_RS)
#define RS_D GPIO.out_w1ts = (1 << LCD_RS); GPIO.out_w1ts = (1 << LCD_RS)

#define DC_C RS_C
#define DC_D RS_D

#define CS_L GPIO.out_w1tc = (1 << LCD_CS)
#define CS_H GPIO.out_w1ts = (1 << LCD_CS)


#define WR_L GPIO.out_w1tc = (1 << LCD_WR)
#define WR_H GPIO.out_w1ts = (1 << LCD_WR)



// Write 8 bits to TFT
#define tft_Write_8(C)  GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t)(C));WR_L; WR_H;  //WR_H

 #define tft_Write_16(C) GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((C) >> 8)); WR_L;WR_H; \
                           GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((C) >> 0)); WR_L;WR_H

 
  // Write two concatenated 16 bit values to TFT

  // Write two concatenated 16 bit values to TFT
  #define tft_Write_32C(C,D) GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((C) >> 8)); WR_L;WR_H; \
                             GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((C) >> 0)); WR_L;WR_H; \
                             GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((D) >> 8)); WR_L;WR_H; \
                             GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t) ((D) >> 0)); WR_L;WR_H


void setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  
  addr_row = 0xFFFF;
  addr_col = 0xFFFF;
  
  DC_C; tft_Write_8(TFT_CASET);
  DC_D; tft_Write_32C(x0, x1);
  DC_C; tft_Write_8(TFT_PASET);
  DC_D; tft_Write_32C(y0, y1);
  DC_C; tft_Write_8(TFT_RAMWR);
  DC_D;
  
}

/***************************************************************************************
** Function name:           writecommand
** Description:             Send an 8 bit command to the TFT
***************************************************************************************/
void writecommand(uint8_t c)
{
  DC_C;
  tft_Write_8(c);
  /*
  begin_tft_write();

  DC_C;

  tft_Write_8(c);

  DC_D;

  end_tft_write();
  */

}


/***************************************************************************************
** Function name:           writedata
** Description:             Send a 8 bit data value to the TFT
***************************************************************************************/
void writedata(uint8_t d)
{
  DC_D;
  tft_Write_8(d);
  /*
  begin_tft_write();

  DC_D;        // Play safe, but should already be in data mode

  tft_Write_8(d);

  CS_L;        // Allow more hold time for low VDI rail

  end_tft_write();
  */
}

void Lcd_Init(void)
{
  digitalWrite(LCD_REST,HIGH);
  delay(5); 
  digitalWrite(LCD_REST,LOW);
  delay(15);
  digitalWrite(LCD_REST,HIGH);
  delay(15);

  digitalWrite(LCD_CS,HIGH);
  digitalWrite(LCD_WR,HIGH);
  digitalWrite(LCD_CS,LOW);  //CS

   writecommand(0xF0);
  writedata(0xC3);
  writecommand(0xF0);
  writedata(0x96);
  writecommand(0x36);
  writedata(0x68);  
  writecommand(0x3A);
  writedata(0x05);  
  writecommand(0xB0);
  writedata(0x80);  
  writecommand(0xB6);
  writedata(0x20);
  writedata(0x02);  
  writecommand(0xB5);
  writedata(0x02);
  writedata(0x03);
  writedata(0x00);
  writedata(0x04);
  writecommand(0xB1);
  writedata(0x80);  
  writedata(0x10);  
  writecommand(0xB4);
  writedata(0x00);
  writecommand(0xB7);
  writedata(0xC6);
  writecommand(0xC5);
  writedata(0x24);
  writecommand(0xE4);
  writedata(0x31);
  writecommand(0xE8);
  writedata(0x40);
  writedata(0x8A);
  writedata(0x00);
  writedata(0x00);
  writedata(0x29);
  writedata(0x19);
  writedata(0xA5);
  writedata(0x33);
  writecommand(0xC2);
  writecommand(0xA7);
  
  writecommand(0xE0);
  writedata(0xF0);
  writedata(0x09);
  writedata(0x13);
  writedata(0x12);
  writedata(0x12);
  writedata(0x2B);
  writedata(0x3C);
  writedata(0x44);
  writedata(0x4B);
  writedata(0x1B);
  writedata(0x18);
  writedata(0x17);
  writedata(0x1D);
  writedata(0x21);

  writecommand(0XE1);
  writedata(0xF0);
  writedata(0x09);
  writedata(0x13);
  writedata(0x0C);
  writedata(0x0D);
  writedata(0x27);
  writedata(0x3B);
  writedata(0x44);
  writedata(0x4D);
  writedata(0x0B);
  writedata(0x17);
  writedata(0x17);
  writedata(0x1D);
  writedata(0x21);

  writecommand(0X36);
  writedata(0x08);
  writecommand(0xF0);
  writedata(0xC3);
  writecommand(0xF0);
  writedata(0x69);
  writecommand(0X13);
  writecommand(0X11);
  writecommand(0X29);
}

void initPins()
{
  pins[0] = LCD_D0;//32; //DB0
  pins[1] = LCD_D1;//33; //DB1
  pins[2] = LCD_D2;//25; //DB2
  pins[3] = LCD_D3;//26; //DB3
  pins[4] = LCD_D4;//27; //DB4
  pins[5] = LCD_D5;//14; //DB5
  pins[6] = LCD_D6;//12; //DB6
  pins[7] = LCD_D7;//13; //DB7
}

inline void begin_tft_write(void){CS_L;}

/***************************************************************************************
** Function name:           end_tft_write (was called spi_end)
** Description:             End transaction for write and deselect TFT
***************************************************************************************/
inline void end_tft_write(void){CS_H;}

void pushBlock(uint16_t color, uint32_t len){
  if ( (color >> 8) == (color & 0x00FF) )
  { if (!len) return;
    tft_Write_16(color);
    while (--len) {WR_L; WR_H; WR_L; WR_H;}

  }
  else while (len--) {tft_Write_16(color);}
}

void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  /*
  if (_vpOoB) return;

  x+= _xDatum;
  y+= _yDatum;

  // Clipping
  if ((x >= _vpW) || (y >= _vpH)) return;

  if (x < _vpX) { w += x - _vpX; x = _vpX; }
  if (y < _vpY) { h += y - _vpY; y = _vpY; }

  if ((x + w) > _vpW) w = _vpW - x;
  if ((y + h) > _vpH) h = _vpH - y;

  if ((w < 1) || (h < 1)) return;
  */

  //Serial.print(" _xDatum=");Serial.print( _xDatum);Serial.print(", _yDatum=");Serial.print( _yDatum);
  //Serial.print(", _xWidth=");Serial.print(_xWidth);Serial.print(", _yHeight=");Serial.println(_yHeight);

  //Serial.print(" _vpX=");Serial.print( _vpX);Serial.print(", _vpY=");Serial.print( _vpY);
  //Serial.print(", _vpW=");Serial.print(_vpW);Serial.print(", _vpH=");Serial.println(_vpH);

  //Serial.print(" x=");Serial.print( y);Serial.print(", y=");Serial.print( y);
  //Serial.print(", w=");Serial.print(w);Serial.print(", h=");Serial.println(h);

  
  begin_tft_write();
  //Address_set(x, y, x + w - 1, y + h - 1);
  setWindow(x, y, x + w - 1, y + h - 1);

  DC_D; 
  pushBlock(color, w * h);
  end_tft_write();
}

void setup()
{

  initPins();
  CONSTRUCTOR_INIT_TFT_DATA_BUS
 
  for(int x=0;x<8;x++)
  {
    pinMode(pins[x],OUTPUT);
  }
  
  pinMode(LCD_RS,OUTPUT);
  pinMode(LCD_WR,OUTPUT);
  pinMode(LCD_CS,OUTPUT);
  pinMode(LCD_REST,OUTPUT);
  pinMode(LCD_RD,OUTPUT);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_REST, HIGH);
  digitalWrite(LCD_RD, HIGH);
 
  Lcd_Init();

}

void loop()
{  
  fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, 0xf800);
  fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, 0x07E0);
  fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, 0x001F);
}
