/*
Arduino OBD reader (OBD protocol KW1281,  Audi A4 B5, Golf IV etc.)

wiring:
D2 --- OBD level shifter input (RX) (e.g. LM339)
D3 --- OBD level shifter output (TX) (e.g. LM339)
A5 --- Arduino 20x4 LCD display SCL
A4 --- Arduino 20x4 LCD display SDA                                                                            
*/

// https://www.blafusel.de/obd/obd2_kw1281.html -- KWP1281 info
// http://grauonline.de/wordpress/?p=74 -- Alexander's car diagnostic
// https://github.com/OneB1t/ardKWP1281 -- repository with this fork of Alexander's car diagnostic

#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "NewSoftwareSerial.h"
#include "Adafruit_ST7735.h"

#define pinKLineRX 2
#define pinKLineTX 3

#define pinLED 13
#define pin 9

#define pinButton 4
#define pinButton2 5
#define pinButton3 6

#define ADR_Engine 0x01
#define ADR_Gears  0x02
#define ADR_ABS_Brakes 0x03
#define ADR_Airbag 0x15
#define ADR_Dashboard 0x17
#define ADR_Immobilizer 0x25
#define ADR_Central_locking 0x35

#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8

NewSoftwareSerial obd(pinKLineRX, pinKLineTX, false); // RX, TX, inverse logic
Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

uint8_t currAddr = 0; // aktualne ctena adresa
uint8_t blockCounter = 0; //
uint8_t errorTimeout = 0;
uint8_t errorData = 0;
bool connected = false;
bool skipnextread = false;
bool disabled = false;

uint8_t currPage = 1;
uint8_t unitAddress = 1;
unsigned long start, finished, elapsed;

int movepage = 0;
int coolantTemp = 0; // teplota chladici kapaliny
int oilTemp = 0; // teplota oleje
int oilPressure = 0; // tlak oleje
float turboBoostSpec = 0; // pozadovany tlak turba
float turboBoostAct = 0; // aktualni tlak turba
float MAFSpec = 0; // pozadovana vaha vzduchu
float MAFAct = 0; // aktualni vaha vzduchu
float injectedQuantityAct = 100; // aktualni vstrikovane mnozstvi paliva
float injectedQuantitySpec = 100; // pozadovane mnozstvi paliva
float actConsumption = 0; // spotreba
float engineLoad = 0; // zatez motoru
int   engineSpeed = 0; // otacky -- working
float throttleValve = 0; // seslapnuti plynu
float supplyVoltage = 0; // napeti v palubni siti
float vehicleSpeed = 0; // rychlost vozidla
float fuelConsumption = 0; // aktualni spotreba paliva

int debug1 = 0;
int debug2 = 0;
int debug3 = 0;
int debug4 = 0;

float stabCyl1 = 0;
float stabCyl2 = 0;
float stabCyl3 = 0;
float stabCyl4 = 0;

int frontLeftWheel = 0;
int frontRightWheel = 0;
int rearLeftWheel = 0;
int rearRightWheel = 0;

float intakeAirTemp = 0; // teplota v sani
float fuelTemperature = 0;
float coolantTemperature = 0;



String floatToString(float v){
  String res; 
  char buf[16];      
  dtostrf(v,4, 2, buf); 
  res=String(buf);
  return res;
}

void disconnect(){
  connected = false;
  currAddr = 0;
}

void lcdPrint(int x, int y, String s, int width){
  lcd.setCursor(x * 10 ,y* 20);
  String empty = "";
  for(int i = 0;i < width;i++)
  {
    empty.concat(" ");
  }
  lcd.print(empty);
  lcd.setCursor(x * 10 ,y* 20);
  lcd.print(s);
}


void lcdPrint(int x, int y, String s){
  lcd.setCursor(x * 10,y * 20);
  lcd.print(s);
}

void obdWrite(uint8_t data){
  obd.write(data);
}

uint8_t obdRead(){
  unsigned long timeout = millis() + 1500;  
  while (!obd.available()){
    if (millis() >= timeout) {
      disconnect();      
      errorTimeout++;
      return 0;
    }
  }
  uint8_t data = obd.read();
  return data;
}

// 5Bd, 7O1
void send5baud(uint8_t data){
  // // 1 start bit, 7 data bits, 1 parity, 1 stop bit
  #define bitcount 10
  byte bits[bitcount];
  byte even=1;
  byte bit;
  for (int i=0; i < bitcount; i++){
    bit=0;
    if (i == 0)  bit = 0;
      else if (i == 8) bit = even; // computes parity bit
      else if (i == 9) bit = 1;
      else {
        bit = (byte) ((data & (1 << (i-1))) != 0);
        even = even ^ bit;
      }
    bits[i]=bit;
  }
  // now send bit stream    
  for (int i=0; i < bitcount+1; i++){
    if (i != 0){
      // wait 200 ms (=5 baud), adjusted by latency correction
      delay(200);
      if (i == bitcount) break;
    }
    if (bits[i] == 1){ 
      // high
      digitalWrite(pinKLineTX, HIGH);
    } else {
      // low
      digitalWrite(pinKLineTX, LOW);
    }
  }
  obd.flush();
}


bool KWP5BaudInit(uint8_t addr){
  send5baud(addr);
  return true;
}


bool KWPSendBlock(char *s, int size){
  for (int i=0; i < size; i++){
    uint8_t data = s[i];    
    obdWrite(data);
    if (i < size-1){
      uint8_t complement = obdRead();        
      if (complement != (data ^ 0xFF)){
        disconnect();
        errorData++;
        return false;
      }
    }
  }
  blockCounter++;
  return true;
}

// count: if zero given, first received byte contains block length
// 4800, 9600 oder 10400 Baud, 8N1
bool KWPReceiveBlock(char s[], int maxsize, int &size){  
  bool ackeachbyte = false;
  uint8_t data = 0;
  int recvcount = 0;
  if (size == 0) 
    ackeachbyte = true;

  unsigned long timeout = millis() + 1000;   // this is important while switching pages to correctly reconnect to unit
  while ((recvcount == 0) || (recvcount != size)) {
    while (obd.available()){      
      data = obdRead();
      s[recvcount] = data;    
      recvcount++;      
      if ((size == 0) && (recvcount == 1)) {
        size = data + 1;
        if (size > maxsize) {
          //Serial.println("ERROR: invalid maxsize");
          return false;
        }  
      }
      if ((ackeachbyte) && (recvcount == 2)) {
        if (data != blockCounter){
          //Serial.println(F("ERROR: invalid blockCounter"));
          disconnect();
          errorData++;
          return false;
        }
      }
      if ( ((!ackeachbyte) && (recvcount == size)) ||  ((ackeachbyte) && (recvcount < size)) ){
        obdWrite(data ^ 0xFF);  // send complement ack        
      }
      timeout = millis() + 1000;        
    } 
    if (millis() >= timeout){
      disconnect();
      errorTimeout++;
      return false;
    }
  }
  blockCounter++;
  return true;
}

bool KWPSendAckBlock(){
  char buf[32];  
  sprintf(buf, "\x03%c\x09\x03", blockCounter);  
  return (KWPSendBlock(buf, 4));
}

bool connect(uint8_t addr, int baudrate){            
  lcd.fillScreen(ST7735_BLACK);
  blockCounter = 0;  
  currAddr = 0;
  obd.begin(baudrate);       
  KWP5BaudInit(addr);
  // answer: 0x55, 0x01, 0x8A          
  char s[3];
  //lcdPrint(0,1, F("PRIJEM DAT"), 20);
  int size = 3;
  if (!KWPReceiveBlock(s, 3, size)) return false;
  if (    (((uint8_t)s[0]) != 0x55) 
     ||   (((uint8_t)s[1]) != 0x01) 
     ||   (((uint8_t)s[2]) != 0x8A)   ){
    lcdPrint(0,1, F("NEKOMUNIKUJE"), 20);
    connected = false;
    errorData++;
    return false;
  }
  currAddr = addr;
  connected = true;  
  if (!readConnectBlocks()) return false;
  return true;
}
  
bool readConnectBlocks(){  
  // read connect blocks
  lcdPrint(0,0, F("PRIPOJENO"));
  String info;  
  while (true){
    int size = 0;
    char s[64];
    if (!(KWPReceiveBlock(s, 64, size))) return false;
    if (size == 0) return false;
    if (s[2] == '\x09') break; 
    if (s[2] != '\xF6') {
      //Serial.println(F("ERROR: unexpected answer"));
      disconnect();
      errorData++;
      return false;
    }
    String text = String(s);
    info += text.substring(3, size-2);
    if (!KWPSendAckBlock()) return false;
  }
  lcd.fillScreen(ST7735_BLACK);  
  return true;
}

bool readSensors(int group){
  char s[64];
  sprintf(s, "\x04%c\x29%c\x03", blockCounter, group);
  if (!KWPSendBlock(s, 5)) return false;
  int size = 0;
  KWPReceiveBlock(s, 64, size);
  if (s[2] != '\xe7') {
    disconnect();
    errorData++;
    return false;
  }
  int count = (size-4) / 3;
  for (int idx=0; idx < count; idx++)
  {
      byte k=s[3 + idx*3];
      byte a=s[3 + idx*3+1];
      byte b=s[3 + idx*3+2];
      String n;
      float v = 0;
      String t = "";
      String units = "";
      char buf[32];    
      switch (k){
        case 1:  v=0.2*a*b;             units=F("rpm"); break;
        case 2:  v=a*0.002*b;           units=F("%%"); break;
        case 3:  v=0.002*a*b;           units=F("Deg"); break;
        case 4:  v=abs(b-127)*0.01*a;   units=F("ATDC"); break;
        case 5:  v=a*(b-100)*0.1;       units=F("°C");break;
        case 6:  v=0.001*a*b;           units=F("V");break;
        case 7:  v=0.01*a*b;            units=F("km/h");break;
        case 8:  v=0.1*a*b;             units=F(" ");break;
        case 9:  v=(b-127)*0.02*a;      units=F("Deg");break;
        case 10: if (b == 0) t=F("COLD"); else t=F("WARM");break;
        case 11: v=0.0001*a*(b-128)+1;  units = F(" ");break;
        case 12: v=0.001*a*b;           units =F("Ohm");break;
        case 13: v=(b-127)*0.001*a;     units =F("mm");break;
        case 14: v=0.005*a*b;           units=F("bar");break;
        case 15: v=0.01*a*b;            units=F("ms");break;
        case 18: v=0.04*a*b;            units=F("mbar");break;
        case 19: v=a*b*0.01;            units=F("l");break;
        case 20: v=a*(b-128)/128;       units=F("%%");break;
        case 21: v=0.001*a*b;           units=F("V");break;
        case 22: v=0.001*a*b;           units=F("ms");break;
        case 23: v=b/256*a;             units=F("%%");break;
        case 24: v=0.001*a*b;           units=F("A");break;
        case 25: v=(b*1.421)+(a/182);   units=F("g/s");break;
        case 26: v=float(b-a);          units=F("C");break;
        case 27: v=abs(b-128)*0.01*a;   units=F("°");break;
        case 28: v=float(b-a);          units=F(" ");break;
        case 30: v=b/12*a;              units=F("Deg k/w");break;
        case 31: v=b/2560*a;            units=F("°C");break;
        case 33: v=100*b/a;             units=F("%%");break;
        case 34: v=(b-128)*0.01*a;      units=F("kW");break;
        case 35: v=0.01*a*b;            units=F("l/h");break;
        case 36: v=((unsigned long)a)*2560+((unsigned long)b)*10;  units=F("km");break;
        case 37: v=b; break; // oil pressure ?!
        case 38: v=(b-128)*0.001*a;        units=F("Deg k/w"); break;
        case 39: v=(b/4)*a*0.01;                units=F("mg/h"); break;
        case 40: v=b*0.1+(25.5*a)-400;     units=F("A"); break;
        case 41: v=b+a*255;                units=F("Ah"); break;
        case 42: v=b*0.1+(25.5*a)-400;     units=F("Kw"); break;
        case 43: v=b*0.1+(25.5*a);         units=F("V"); break;
        case 44: sprintf(buf, "%2d:%2d", a,b); t=String(buf); break;
        case 45: v=0.1*a*b/100;            units=F(" "); break;
        case 46: v=(a*b-3200)*0.0027;      units=F("Deg k/w"); break;
        case 47: v=(b-128)*a;              units=F("ms"); break;
        case 48: v=b+a*255;                units=F(" "); break;
        case 49: v=(b/4)*a*0.1;            units=F("mg/h"); break;
        case 50: v=(b-128)/(0.01*a);       units=F("mbar"); break;
        case 51: v=((b-128)/255)*a;        units=F("mg/h"); break;
        case 52: v=b*0.02*a-a;             units=F("Nm"); break;
        case 53: v=(b-128)*1.4222+0.006*a;  units=F("g/s"); break;
        case 54: v=a*256+b;                units=F("count"); break;
        case 55: v=a*b/200;                units=F("s"); break;
        case 56: v=a*256+b;                units=F("WSC"); break;
        case 57: v=a*256+b+65536;          units=F("WSC"); break;
        case 59: v=(a*256+b)/32768;        units=F("g/s"); break;
        case 60: v=(a*256+b)*0.01;         units=F("sec"); break;
        case 62: v=0.256*a*b;              units=F("S"); break;
        case 64: v=float(a+b);             units=F("Ohm"); break;
        case 65: v=0.01*a*(b-127);         units=F("mm"); break;
        case 66: v=(a*b)/511.12;          units=F("V"); break;
        case 67: v=(640*a)+b*2.5;         units=F("Deg"); break;
        case 68: v=(256*a+b)/7.365;       units=F("deg/s");break;
        case 69: v=(256*a +b)*0.3254;     units=F("Bar");break;
        case 70: v=(256*a +b)*0.192;      units=F("m/s^2");break;
        default: sprintf(buf, "%2x, %2x      ", a, b); break;
      }
      switch (currAddr){
        case ADR_Engine: 
          switch(group)
          {
              case 3: 
                 switch (idx)
                 {
                    case 0: engineSpeed = v; break;
                    case 1: MAFSpec=v; break;
                    case 2: MAFAct =v; break;
                 }              
              break;
              case 7:
                 switch (idx)
                 {
                    case 0: fuelTemperature = v; break;
                    case 2: intakeAirTemp =v; break;
                    case 3: coolantTemperature = v; break;
                 }              
              break;

            case 11: 
                switch (idx)
                {
                  case 0: engineSpeed = v; break;
                  case 1: turboBoostSpec = v; break;
                  case 2: turboBoostAct = v; break;
                }              
                break; 
            case 12: // get supply voltage
            switch(idx)
            {
              case 2: supplyVoltage = v; break; 
            }
            break;
            case 13: 
                switch (idx) // STABILIZACE VOLNOBEHU
                {
                  case 0: stabCyl1 = v; break;
                  case 1: stabCyl2 = v; break;
                  case 2: stabCyl3 = v; break;
                  case 3: stabCyl4 = v; break;
                }              
                break;       
            case 15: 
                 switch (idx)
                 {
                    case 0: engineSpeed = v; break;
                    case 1: injectedQuantityAct = v; break;
                    case 2: fuelConsumption = v; break;
                    case 3: injectedQuantitySpec = v; break;
                 }              
            break;
          }
          break;
        case ADR_Dashboard: 
          switch (group)
          { 
            case 1:  
              switch (idx)
              {
                case 0: vehicleSpeed = v; break;
                case 1: engineSpeed = v; break;
                case 2: oilPressure = v; break;
              }
              break;
            case 50:
              switch (idx)
              {
                case 1: engineSpeed = v; break;
                case 2: oilTemp = v; break;
                case 3: coolantTemp = v; break;
              }
              break;
          }
          break;
      }
    }
  return true;
}

void updateDisplay(){
    switch (currPage){
        case 1: // TLAK TURBA - SPECIFIKOVANY A REALNY
          lcdPrint(0,1, String(F("TURBO SPEC:")));
          lcdPrint(0,2,String(turboBoostSpec) + String(F(" mBar")));        
          lcdPrint(0,3, String(F("TURBO AKT:")));                                      
          lcdPrint(0,4,String(turboBoostAct)  + String(F(" mBar")));
        break;
        case 2: // MAF
         lcdPrint(0,1, String(F("MAF SPEC:")));   
          lcdPrint(0,2,String(MAFSpec)  + String(F(" mg/h")));
          lcdPrint(0,3, String(F("MAF AKT:")));   
          lcdPrint(0,4,String(MAFAct)  + String(F(" mg/h")));
        break;   
        case 3: // OTACKY - VSTRIKOVANE PALIVO - SPOTREBA
          lcdPrint(0,0,String(F("OTACKY  ")));
          lcdPrint(9,0,String(engineSpeed));     
          lcdPrint(0,1,String(F("AKT VST ")));
          lcdPrint(9,1, String(injectedQuantityAct));  
          lcdPrint(0,2,String(F("REQ VST ")));
          lcdPrint(9,2, String(injectedQuantitySpec));     
          lcdPrint(0,3,String(F("SPOTREB ")));
          lcdPrint(9,3, String(fuelConsumption));   
          lcdPrint(0,4,String(F("VYKONNM ")));
          lcdPrint(9,4, String((injectedQuantityAct * 0.0462 * engineSpeed / 2) / 60));  
          lcdPrint(0,5,String(F("NAPETI  ")));
          lcdPrint(9,5, String(supplyVoltage));
        break;
        case 4: // TEPLOTY 
          lcdPrint(0,1,String(F("NAFTA ")) + String(fuelTemperature) + "C");   
          lcdPrint(0,2,String(F("SANI  ")) + String(intakeAirTemp) + "C");   
          lcdPrint(0,3,String(F("MOTOR ")) + String(coolantTemperature) + "C");  
        break;
    }    
    movepage++;
    if(movepage == 15)
    {
      movepage = 0;
      currPage++;
      lcd.fillScreen(ST7735_BLACK);
      if(currPage > 4)
        currPage  = 0;
    }
}

void setup(){      
  lcd.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
  lcd.setRotation(3);
  lcd.setTextSize(2);
  lcd.setTextColor(0xF000,ST7735_BLACK);
  pinMode(pinKLineTX, OUTPUT);  
  digitalWrite(pinKLineTX, HIGH);  
  
  pinMode(pinButton, INPUT);  
}


void loop(){    
start = millis();   
  if(!disabled)
  {
    if(!skipnextread)
    {
        switch (currPage){   
          case 1:
            if (currAddr != ADR_Engine) {
              connect(ADR_Engine, 9600);
            } else {
              readSensors(11);
            }   
          break;   
          case 2:
            if (currAddr != ADR_Engine) {
              connect(ADR_Engine, 9600);
            } else {
              readSensors(3);
            }   
          break; 
          case 3:
            if (currAddr != ADR_Engine){       
              connect(ADR_Engine, 9600);
            } else  {
              readSensors(15);
              readSensors(12);
            }      
          break;
          case 4:
            if (currAddr != ADR_Engine){       
              connect(ADR_Engine, 9600);
            } else  {
              readSensors(7);
            }  
          break;
        }    
  
    finished=millis();  
    elapsed=finished-start;
    updateDisplay(); 
    }
    skipnextread = false;
  }   
}
