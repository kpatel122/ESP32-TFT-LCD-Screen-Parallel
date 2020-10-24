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

// Write 8 bits to TFT
#define tft_Write_8(C)  GPIO.out_w1tc = clr_mask; GPIO.out_w1ts = set_mask((uint8_t)(C));//WR_H



#define RS_C GPIO.out_w1tc = (1 << LCD_RS); GPIO.out_w1tc = (1 << LCD_RS)
#define RS_D GPIO.out_w1ts = (1 << LCD_RS); GPIO.out_w1ts = (1 << LCD_RS)

#define CS_L GPIO.out_w1tc = (1 << LCD_CS)
#define CS_H GPIO.out_w1ts = (1 << LCD_CS)


#define WR_L GPIO.out_w1tc = (1 << LCD_WR)
#define WR_H GPIO.out_w1ts = (1 << LCD_WR)

//#define WR_L GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR); GPIO.out_w1tc = (1 << LCD_WR)
//#define WR_H GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR); GPIO.out_w1ts = (1 << LCD_WR)
 
#define Lcd_Writ_Bus(d)  tft_Write_8(d) WR_L; WR_H;  

/* use this if you are getting a white screen
inline void Lcd_Writ_Bus(unsigned int d)
{
    tft_Write_8(d);
    WR_L;  
    WR_H;
}
*/

void Lcd_Write_Com(unsigned int VH)  
{   
  RS_C;
  Lcd_Writ_Bus(VH);
}

void Lcd_Write_Data(unsigned int VH)
{
  RS_D;
  Lcd_Writ_Bus(VH);
}

void Lcd_Write_Com_Data(unsigned int com,unsigned int dat)
{
  Lcd_Write_Com(com);
  Lcd_Write_Data(dat);
}

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
  Lcd_Write_Com(0x2a);
  Lcd_Write_Data(x1>>8);
  Lcd_Write_Data(x1);
  Lcd_Write_Data(x2>>8);
  Lcd_Write_Data(x2);
  Lcd_Write_Com(0x2b);
  Lcd_Write_Data(y1>>8);
  Lcd_Write_Data(y1);
  Lcd_Write_Data(y2>>8);
  Lcd_Write_Data(y2);
  Lcd_Write_Com(0x2c);               
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



    Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0xC3);
  Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0x96);
  Lcd_Write_Com(0x36);
  Lcd_Write_Data(0x68);  
  Lcd_Write_Com(0x3A);
  Lcd_Write_Data(0x05);  
  Lcd_Write_Com(0xB0);
  Lcd_Write_Data(0x80);  
  Lcd_Write_Com(0xB6);
  Lcd_Write_Data(0x20);
  Lcd_Write_Data(0x02);  
  Lcd_Write_Com(0xB5);
  Lcd_Write_Data(0x02);
  Lcd_Write_Data(0x03);
  Lcd_Write_Data(0x00);
  Lcd_Write_Data(0x04);
  Lcd_Write_Com(0xB1);
  Lcd_Write_Data(0x80);  
  Lcd_Write_Data(0x10);  
  Lcd_Write_Com(0xB4);
  Lcd_Write_Data(0x00);
  Lcd_Write_Com(0xB7);
  Lcd_Write_Data(0xC6);
  Lcd_Write_Com(0xC5);
  Lcd_Write_Data(0x24);
  Lcd_Write_Com(0xE4);
  Lcd_Write_Data(0x31);
  Lcd_Write_Com(0xE8);
  Lcd_Write_Data(0x40);
  Lcd_Write_Data(0x8A);
  Lcd_Write_Data(0x00);
  Lcd_Write_Data(0x00);
  Lcd_Write_Data(0x29);
  Lcd_Write_Data(0x19);
  Lcd_Write_Data(0xA5);
  Lcd_Write_Data(0x33);
  Lcd_Write_Com(0xC2);
  Lcd_Write_Com(0xA7);
  
  Lcd_Write_Com(0xE0);
  Lcd_Write_Data(0xF0);
  Lcd_Write_Data(0x09);
  Lcd_Write_Data(0x13);
  Lcd_Write_Data(0x12);
  Lcd_Write_Data(0x12);
  Lcd_Write_Data(0x2B);
  Lcd_Write_Data(0x3C);
  Lcd_Write_Data(0x44);
  Lcd_Write_Data(0x4B);
  Lcd_Write_Data(0x1B);
  Lcd_Write_Data(0x18);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x1D);
  Lcd_Write_Data(0x21);

  Lcd_Write_Com(0XE1);
  Lcd_Write_Data(0xF0);
  Lcd_Write_Data(0x09);
  Lcd_Write_Data(0x13);
  Lcd_Write_Data(0x0C);
  Lcd_Write_Data(0x0D);
  Lcd_Write_Data(0x27);
  Lcd_Write_Data(0x3B);
  Lcd_Write_Data(0x44);
  Lcd_Write_Data(0x4D);
  Lcd_Write_Data(0x0B);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x1D);
  Lcd_Write_Data(0x21);

  Lcd_Write_Com(0X36);
  Lcd_Write_Data(0x08);
  Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0xC3);
  Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0x69);
  Lcd_Write_Com(0X13);
  Lcd_Write_Com(0X11);
  Lcd_Write_Com(0X29);
}

void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{ 

  unsigned int i,j;
  Lcd_Write_Com(0x02c); //write_memory_start

  RS_D;
  CS_L;
  
  l=l+x;
  Address_set(x,y,l,y);
  j=l*2;
  for(i=1;i<=j;i++)
  {
    Lcd_Write_Data(c);
  }
  CS_H;
  
}

void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{ 
  unsigned int i,j;
  Lcd_Write_Com(0x02c); //write_memory_start

  RS_D;
  CS_L;
  
  l=l+y;
  Address_set(x,y,x,l);
  j=l*2;
  for(i=1;i<=j;i++)
  { 
    Lcd_Write_Data(c);
  }
  CS_H;

}

void Rect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  H_line(x  , y  , w, c);
  H_line(x  , y+h, w, c);
  V_line(x  , y  , h, c);
  V_line(x+w, y  , h, c);
}

void Rectf(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  unsigned int i;
  for(i=0;i<h;i++)
  {
    H_line(x  , y  , w, c);
    H_line(x  , y+i, w, c);
  }
}
int RGB(int r,int g,int b)
{return r << 16 | g << 8 | b;
}
void LCD_Clear(unsigned int j)                   
{ 
  unsigned int i,m;
 Address_set(0,0,320,480);
  CS_L;
  
  for(i=0;i<320;i++)
    for(m=0;m<480;m++)
    {
      Lcd_Write_Data(j>>8);
      Lcd_Write_Data(j);
    }

  CS_H;   
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
 //LCD_Clear(0xf800);
}

void loop()
{  
  
   LCD_Clear(0xf800);
   LCD_Clear(0x07E0);
   LCD_Clear(0x001F);

   
   /*
  LCD_Clear(0xf800);
    
  for(int i=0;i<1000;i++)
  {
    Rect(random(300),random(300),random(300),random(300),random(65535)); // rectangle at x, y, with, hight, color
  }
  */
  

}
