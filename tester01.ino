//const int A[] = {37,36,35,34,33,32,31,30,22,23,24,25,26,27,28,29};
//const int D[] = {49,48,47,46,45,44,43,42};
#define WR 41
#define RD 39
#define M1 38
#define IORQ 40
#define MREQ 51

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

  digitalWrite(M1, HIGH); //importante para I/O
  digitalWrite(IORQ, HIGH);
  digitalWrite(RD, HIGH);
  digitalWrite(WR, HIGH);
  digitalWrite(MREQ, HIGH);
  
  DDRC = 0xff; //A0~A7 output
  DDRA = 0xff; //A8~A15 output

  PPI_init();
}

void PPI_init()
{
  out(0xAB, 0x82); // Bit 7=1, modo por default, puerto A output, puerto B input y puerto C output.
  //out(0xA8, 0x00); //
}

const char nibble[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
String hex(byte x)
{
  return String(nibble[x>>4]) + String(nibble[ x & 0xf]);
}

byte memRead(uint16_t address)
{
  PORTC = address & 0xff; //A0~A7
  PORTA = address >> 8; //A8~A15

  DDRL = 0x00; //D0~D7 input

  digitalWrite(MREQ, LOW);
  digitalWrite(RD, LOW);
  //delayMicroseconds(50);

  byte data = PINL;
  
  digitalWrite(RD, HIGH);
  //delayMicroseconds(10);
  digitalWrite(MREQ, HIGH);
  return data;
}

byte inp(byte port)
{
  DDRL = 0x00; //D0~D7 input
  PORTC = port; //A0~A7
  PORTA = 0; //A8~A15

  digitalWrite(WR, HIGH); //por las dudas
  digitalWrite(IORQ, LOW);
  //delayMicroseconds(1);
  digitalWrite(RD, LOW);
  //delayMicroseconds(1);

  byte data = PINL;
  
  digitalWrite(RD, HIGH);
  //delayMicroseconds(10);
  digitalWrite(IORQ, HIGH);

  return data;
}

void out(byte port, byte data)
{
  PORTC = port; //A0~A7
  PORTA = 0; //A8~A15
  DDRL = 0xff; //D0~D7 output
  PORTL = data; //

  digitalWrite(RD, HIGH); //por las dudas
  digitalWrite(IORQ, LOW);
  digitalWrite(WR, LOW);
  //delayMicroseconds(1);
  digitalWrite(WR, HIGH);
  //delayMicroseconds(1);
  digitalWrite(IORQ, HIGH);

  DDRL = 0x00; //D0~D7 input
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
    //delay(1);
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

void loop() {
  //static byte C = 0x80;
  Test_PSG_1();
  Test_VDP_4();
  CharSet();
  delay(1000);
  Test_VDP_3();
  delay(1000);
  //Test_ROM();
  //out(0xAA, C&0x7f);
  //delay(1000);
  //out(0xAA, C|0x80);
  //delay(1000);
  //Test_VDP_1();
}
