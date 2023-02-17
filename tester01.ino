/*********************************************
 * Autor: Carlos Escobar
 * Asistencia técnica: Carlos Maidana
 * Feb-2023
 *********************************************/

#include <TimerOne.h>

//const int A[] = {37,36,35,34,33,32,31,30,22,23,24,25,26,27,28,29};
//const int D[] = {49,48,47,46,45,44,43,42};
#define WR 41
#define RD 39
#define M1 38
#define IORQ 40
#define MREQ 51
//#define RFSH 50
#define RFSH 8
#define CLOCK 52

#define CHARSET_BASE 7103

const String colorName[] = {
  "TRANSPARENT", 
  "BLACK", 
  "MEDIUM GREEN", 
  "LIGHT GREEN", 
  "DARK BLUE", 
  "LIGHT BLUE", 
  "DARK RED", 
  "CYAN", 
  "MEDIUM RED", 
  "LIGHT RED", 
  "DARK YELLOW", 
  "LIGHT YELLOW", 
  "DARK GREEN", 
  "MAGENTA", 
  "GRAY", 
  "WHITE"
}; 

void setup() {
  Serial.begin(115200);
  
  pinMode(M1, OUTPUT);
  pinMode(IORQ, OUTPUT);
  pinMode(RD, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(MREQ, OUTPUT);
  pinMode(RFSH, OUTPUT);
  pinMode(CLOCK, INPUT);

  digitalWrite(M1, HIGH); //importante para I/O
  digitalWrite(IORQ, HIGH);
  digitalWrite(RD, HIGH);
  digitalWrite(WR, HIGH);
  digitalWrite(MREQ, HIGH);
  digitalWrite(RFSH, HIGH);
  
  DDRC = 0xff; //A0~A7 output
  DDRA = 0xff; //A8~A15 output

  PPI_init();

  Timer1.initialize(25); //Initialize timer 30 us
  Timer1.attachInterrupt(refreshFunc,25);

}

inline void PPI_init()
{
  out(0xAB, 0x82); // Bit 7=1, modo por default, puerto A output, puerto B input y puerto C output.
  out(0xAA, 0x50); // RELE OFF, CAPS LED OFF
  out(0xA8, 0x50); // ROM en slot 0 pag 1 y 2, RAM en slot 1 pag 3 y 4.
}

inline void M1_low()
{
  PORTD &= B01111111; //38 = PD7
}

inline void M1_high()
{
  PORTD |= B10000000; //38 = PD7
}

inline void RFSH_low()
{
  //PORTB &= B11110111; //50 = PB3
  PORTH &= ~(1<<PH5); //8 = PH5
}

inline void RFSH_high()
{
  //PORTB |= B00001000; //50 = PB3
  PORTH |= (1<<PH5); //8 = PH5
}

inline void MREQ_low()
{
  PORTB &= B11111011; //51 = PB2
}

inline void MREQ_high()
{
  PORTB |= B00000100; //51 = PB2
}

inline void RD_low()
{
  PORTG &= B11111011; //39 = PG2
}

inline void RD_high()
{
  PORTG |= B00000100; //39 = PG2
}

inline void WR_low()
{
  //PORTG &= B11111110; //41 = PG0
  PORTG &= ~(1<<PG0); //41 = PG0
}

inline void WR_high()
{
  //PORTG |= B00000001; //41 = PG0
  PORTG |= (1<<PG0); //41 = PG0
}

inline void IORQ_low()
{
  //PORTG &= B11111101; //40 = PG1
  PORTG &= ~(1<<PG1); //40 = PG1
}

inline void IORQ_high()
{
  //PORTG |= B00000010; //40 = PG1
  PORTG |= (1<<PG1); //40 = PG1
}

void refreshFunc(void)
{
  static byte R=0;
  PORTC = (byte)(R & 0x7f); //A0~A7
  PORTA = 0; //A8~A15

  //M1_high();
  //WR_high();
  //RD_high();
  //IORQ_high();

  //while ((PINB & (1<<PB1)));
  
  RFSH_low();
  MREQ_low();
  R++;
  MREQ_high();
  RFSH_high();
}

void printTest(void)
{
  Serial.println("pasa");
}

String hex(byte x)
{
  const char nibble[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  return String(nibble[x>>4]) + String(nibble[ x & 0xf]);
}

inline byte memRead(uint16_t address)
{
  cli(); 
  //Timer1.stop();
  DDRL = 0x00; //D0~D7 input
  PORTC = (byte)(address & 0xff); //A0~A7
  PORTA = (byte)(address >> 8); //A8~A15

  M1_low();
  asm("nop \n nop \n");
  MREQ_low();
  RD_low();
  //delayMicroseconds(10); //anda con 2, pero pongo mas para mayor seguridad
  asm("nop \n nop \n nop \n nop \n nop"); //5 nops = 300ns aprox
  byte data = PINL;
  
  RD_high(); //digitalWrite(RD, HIGH);
  MREQ_high(); //digitalWrite(MREQ, HIGH);
  M1_high();
  sei(); 
  //Timer1.resume();
  return data;
}

inline void memWrite(uint16_t address, byte data)
{
  cli(); 
  //Timer1.stop();
  PORTC = (byte)(address & 0xff); //A0~A7
  PORTA = (byte)(address >> 8); //A8~A15

  DDRL = 0xff; //D0~D7 output
  PORTL = data;

  RFSH_high();
  M1_low();
  asm("nop\n nop\n nop\n nop\n nop\n");
  MREQ_low(); 
  asm("nop\n nop\n nop\n nop\n nop\n");
  WR_low();
  
  //asm("nop \n");
  asm("nop\n nop\n nop\n nop\n nop\n");
  
  WR_high();
  MREQ_high();
  M1_high();
  
  DDRL = 0x00;
  sei();
  //Timer1.resume();
}

inline byte inp(byte port)
{
  cli(); 
  //Timer1.stop();
  DDRL = 0x00; //D0~D7 input
  PORTC = port; //A0~A7
  PORTA = 0; //A8~A15

  IORQ_low(); //digitalWrite(IORQ, LOW);
  RD_low(); //digitalWrite(RD, LOW);

  byte data = PINL;

  RD_high(); //digitalWrite(RD, HIGH);
  IORQ_high(); //digitalWrite(IORQ, HIGH);

  sei(); 
  //Timer1.resume();
  return data;
}

inline void out(byte port, byte data)
{
  cli(); 
  //Timer1.stop();
  PORTC = port; //A0~A7
  PORTA = 0; //A8~A15
  DDRL = 0xff; //D0~D7 output
  PORTL = data; //

  IORQ_low(); //digitalWrite(IORQ, LOW);
  WR_low(); //digitalWrite(WR, LOW);
  //delayMicroseconds(1);
  WR_high(); //digitalWrite(WR, HIGH);
  IORQ_high(); //digitalWrite(IORQ, HIGH);

  DDRL = 0x00; //D0~D7 input
  sei(); 
  //Timer1.resume();
}

void Test_PSG_1()
{
  out(0xa0, 7);
  out(0xa1, 0xB8); //b1011 1000 ; apaga ruido y deja bit 7 en 1 que es el default para MSX
  for (uint16_t xx=0; xx<0x280; xx++)
  {
    out(0xa0, 0);
    out(0xa1, xx & 0xff);
    out(0xa0, 1);
    out(0xa1, xx >> 8);
    out(0xa0, 8);
    out(0xa1, 8);
    delay(1);
  }
  out(0xa0, 8);
  out(0xa1, 0);
}

static byte _vdp_reg[8] = {0,0,0,0,0,0,0,0};
void VDP_set(byte reg, byte value)
{
  reg = reg & 0x7; //solo registros de 0 a 7, TMS9918A
  out(0x99, value); // paso el valor 
  out(0x99, 0x80 | reg); // a continuacion el registro destino
  _vdp_reg[reg] = value; //guardo el valor para el get
}

byte VDP_get(byte reg)
{
  reg = reg & 0x7;
  return _vdp_reg[reg];
}

void VRAM_write(uint16_t addr, byte value)
{
  static uint16_t nextAddr = 0xffff;
  addr = addr & 0x3fff; //enmascaro para solo 16K de vram

  if (addr != nextAddr) // si no es la sig posicion de memoria la seteo
  {
    out(0x99, addr & 0xff); //parte baja
    out(0x99,0x40 | (addr>>8) ); //parte alta
  }
  out(0x98, value);

  nextAddr = addr + 1;
}

byte VRAM_read(uint16_t addr)
{
  static uint16_t nextAddr = 0xffff;
  addr = addr & 0x3fff; //enmascaro para solo 16K de vram

  if (addr != nextAddr) // si no es la sig posicion de memoria la seteo
  {
    out(0x99, addr & 0xff); //parte baja
    out(0x99, /*0x40 |*/ (addr>>8) ); //parte alta
  }
  nextAddr = addr + 1;

  return inp(0x98);
}

void Test_VDP_1()
{
  Serial.println("TEST SCREEN BACKGROUND COLOR\n");
  for (byte i=1; i<16; i++)
  {
    Serial.print(String(i) + "-" + colorName[i]);
    VDP_set(7, i); // cambia el color del fondo de pantalla
    delay(2000);
  } 
}

void Test_VDP_2(char c)
{
  VDP_set(0, 0);
  VDP_set(1, 0xf0); //11010000 text 1 mode
  //out(0x99,0xC8); //11001000 multicolor mode
  VDP_set(7, 0xF4); //COLOR 15,4
  
  for (uint16_t i=0; i<0x4000; i++)
  {
    static byte c=0;
    VRAM_write(i, c);
    //delayMicroseconds(500);
  } 
  delay(1000);
}

void Test_VDP_3()
{
  Serial.println(VRAM_read(0));
  VRAM_write(0,123);
  Serial.println(VRAM_read(0));
}

void screen0() //SCREEN 0
{
  inp(0x99);
  VDP_set(0, 0); //
  VDP_set(1, 0xf0); // text 1 mode
  //VDP_set(1, 0xC8);  //11001000 multicolor mode
  VDP_set(2, 0);
  VDP_set(3, 0);
  VDP_set(4, 0x01); //tabla de patrones en &H800
  VDP_set(5, 0);
  VDP_set(6, 0);
  VDP_set(7, 0xF5); //COLOR 15,4
  
  for (uint16_t x=0; x<(40*24); x++) 
  {
    VRAM_write(x, (byte)x);
  }
}

void Test_ROM()
{
  for (uint16_t i=0; i<2048; i++)
  {
    Serial.print(memRead(i));
    Serial.print(" ");
    if (i%60 == 0 ) Serial.println();
  }
}

void CharSet(uint16_t base)
{
  for (uint16_t i=0; i<2048; i++)
  {
    byte c = memRead(i+base);
    VRAM_write(0x800 + i, c);
  }
}

void Test_VDP_6()
{
  screen0();
  for (uint16_t x=0; x<40*24; x++)
    VRAM_write(x,0);
  CharSet(CHARSET_BASE);

  for (byte color=0; color<16; color++)
  {
    Serial.println(colorName[color&0xf]); 
    VDP_set(7, ((color & 0xf)==0xf ? 0x10 :0xf0) | color & 0xf);
    for (uint16_t xy = 0; xy<20; xy++)
    {
      VRAM_write(xy, colorName[color&0xf][xy]); 
    }
    Test_PSG_1();
    delay(1000);
  }
}

void loop0()
{
  /*
  static byte slot=0;
  out(0xa8, slot);
  if (slot == 0) slot=0x55;
  else if (slot == 0x55) slot = 0xaa;
  else if (slot == 0xaa) slot = 0xff;
  else if (slot == 0xff) slot = 0x00;
  Serial.println("slot "+hex(slot));
  */

  uint16_t address = random(32)+0x7ff0;
  byte xx=memRead(address);
  xx++;
  memWrite(address, xx);
  byte c = memRead(address);
  if (c!=xx) 
  {
    Serial.println(hex(address>>8)+hex(address&0xff)+":"+hex(c)); 
  }
  else
  {
    Serial.println(hex(address>>8)+hex(address&0xff)+": ok"); 
  }
  delay(1000);
}

static bool flag=true;
void loop01() 
{
  if (flag)
  {
    screen0();
    out(0xa8, B01010000);
    for (uint16_t x=0; x<2048; x++)
    {
      byte b = memRead(CHARSET_BASE+x);
      memWrite(0x8000+x, b);
      memWrite(0xC000+x, b);
    }
    flag = false;
  }
  //CharSet(CHARSET_BASE);
  //static uint16_t aaa=0x8000; aaa+=8;
  memWrite(0x8000, 0x55); 
  memWrite(0x8001, 0xaa); 
  memWrite(0x8002, 0x55); 
  memWrite(0x8003, 0xaa); 
  memWrite(0x8004, 0x55); 
  memWrite(0x8005, 0xaa); 
  memWrite(0x8006, 0x55); 
  memWrite(0x8007, 0xaa); 
  memWrite(0x8000, 0xaa); 
  memWrite(0xc001, 0x55); 
  memWrite(0xc002, 0xaa); 
  memWrite(0xc003, 0x55); 
  memWrite(0xc004, 0xaa); 
  memWrite(0xc005, 0x55); 
  memWrite(0xc006, 0xaa); 
  memWrite(0xc007, 0x55); 
  CharSet(0x8000);
 // delay(1000);
  CharSet(0xc000);
  //delay(1000);
}

void loop() 
{
  Test_VDP_6();
}
