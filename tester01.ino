/*********************************************
 * Autor: Carlos Escobar
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
#define RFSH 50

volatile uint32_t rf=0;
volatile byte R=0;

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
  Timer1.attachInterrupt(refreshFunc);
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
  PORTB &= B11110111; //50 = PB3
}

inline void RFSH_high()
{
  PORTB |= B00001000; //50 = PB3
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
  PORTC = R & 0x7f; //A0~A7
  PORTA = 0; //A8~A15

  RFSH_low(); //digitalWrite(RFSH, LOW);
  MREQ_low(); //digitalWrite(MREQ, LOW);
  R++;
  rf++;
  asm("nop \n");
  MREQ_high(); //digitalWrite(MREQ, HIGH);
  RFSH_high(); //digitalWrite(RFSH, HIGH);
}

void printTest(void)
{
  Serial.println("pasa");
}

void PPI_init()
{
  out(0xAB, 0x82); // Bit 7=1, modo por default, puerto A output, puerto B input y puerto C output.
  out(0xA8, 0x50); // ROM en slot 0 pag 1 y 2, RAM en slot 1 pag 3 y 4.
}

String hex(byte x)
{
  const char nibble[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  return String(nibble[x>>4]) + String(nibble[ x & 0xf]);
}

inline byte memRead(uint16_t address)
{
  cli(); //Timer1.stop();
  PORTC = address & 0xff; //A0~A7
  PORTA = address >> 8; //A8~A15

  DDRL = 0x00; //D0~D7 input

  //M1_low();
  MREQ_low(); //digitalWrite(MREQ, LOW);
  RD_low(); //digitalWrite(RD, LOW);
  delayMicroseconds(10); //anda con 2, pero pongo mas para mayor seguridad

  byte data = PINL;
  
  RD_high(); //digitalWrite(RD, HIGH);
  MREQ_high(); //digitalWrite(MREQ, HIGH);
  //M1_high();
  sei(); //Timer1.resume();
  return data;
}

inline byte memWrite(uint16_t address, byte data)
{
  cli(); //Timer1.stop();
  PORTC = address & 0xff; //A0~A7
  PORTA = address >> 8; //A8~A15

  DDRL = 0xff; //D0~D7 output
  PORTL = data;

  MREQ_low(); //digitalWrite(MREQ, LOW);
  delayMicroseconds(5);
  WR_low(); //digitalWrite(WR, LOW);
  delayMicroseconds(5);
  WR_high(); //digitalWrite(WR, HIGH);
  delayMicroseconds(5);
  MREQ_high(); //digitalWrite(MREQ, HIGH);
  
  DDRL = 0x00;
  sei(); //Timer1.resume();
  return data;
}

inline byte inp(byte port)
{
  cli(); //Timer1.stop();
  DDRL = 0x00; //D0~D7 input
  PORTC = port; //A0~A7
  PORTA = 0; //A8~A15

  IORQ_low(); //digitalWrite(IORQ, LOW);
  RD_low(); //digitalWrite(RD, LOW);

  byte data = PINL;

  RD_high(); //digitalWrite(RD, HIGH);
  IORQ_high(); //digitalWrite(IORQ, HIGH);

  sei(); //Timer1.resume();
  return data;
}

inline void out(byte port, byte data)
{
  cli(); //Timer1.stop();
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
  sei(); //Timer1.resume();
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

void Test_VDP_4() //SCREEN 0
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

void CharSet()
{
  for (uint16_t i=0; i<2048; i++)
  {
    byte c = memRead(7103 + i);
    VRAM_write(0x800 + i, c);
  }
}

void loop() 
{
  static bool flag=true;
  uint32_t ini=millis();
  if (flag)
  {  
    Serial.println("copiando ROM a RAM");
    for (uint16_t x=0; x<16384; x++)
    {
      byte c = memRead(x);
      memWrite(x+32768, 0x55);
      //delay(1);
    }
    flag=false;
  }
  Serial.println("comprobar...");
  uint16_t err=0;
  for (uint16_t x=0; x<16384; x++)
  {
    byte c = memRead(x);
    byte d = memRead(x+32768);
    if (d!=0x55) err++;
    //delay(1);
  }  
  Serial.println(err);
  
  
  //static byte C = 0x80;
  Test_PSG_1();
  Test_VDP_4();
  CharSet();
  delay(1000);
  //Test_VDP_3();
  //delay(1000);
  //Test_ROM();
  //out(0xAA, C&0x7f);
  //delay(1000);
  //out(0xAA, C|0x80);
  //delay(1000);
  //Test_VDP_1();
  //Serial.println("RF="+String(rf));
  Serial.println("us="+String((millis()-ini)*1000/rf)+" R="+String(R));
  rf=0;
}
