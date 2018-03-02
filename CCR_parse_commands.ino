#include <FastLED.h>
#include <DS3231_Simple.h>
DS3231_Simple Clock;
#include <SimpleDHT.h>
#include <EtherCard.h>
#include <IPAddress.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

// LCD setup
const int rs = 3, en = 4, d4 = 8, d5 = 7, d6 = 6, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// keypad setup
#define keypadIn A1

// RGB setup 
#define NUM_LEDS 2
#define DATA_PIN 9
CRGB leds[NUM_LEDS];

// ethernet interface ip, gateway, and mac addresses
static byte myip[] = { 10, 0, 6, 153 };
static byte gwip[] = { 126, 10, 220, 254 };
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
byte Ethernet::buffer[100]; // tcp/ip send and receive buffer

// Temp/Hum setup
int pinDHT22 = 2;     // Temp/Hum sensor setup at pin 2
SimpleDHT22 dht22;

// token values
#define tEOL 0x07F   //end of line
#define tERR 0x7E    //error
#define tGR 0x01     //green
#define tRE tGR + 1  //red
#define tON tRE + 1  //on
#define tSL tON + 1  //slow
#define tME tSL + 1  //medium
#define tFA tME + 1  //fast
#define tLE tFA + 1  //led
#define tMO tLE + 1  //motor
#define tAD tMO + 1  //add
#define tBL tAD + 1  //blink
#define tVE tBL + 1  //version
#define tOF tVE + 1  //off
#define tST tOF + 1  //status
#define tTE tST + 1  //temperature
#define tHU tTE + 1  //humidity
#define tQE tHU + 1  //query
#define tHI tQE + 1  //high
#define tLO tHI + 1  //low
#define tNU tLO + 1  //nuke
#define tPR tNU + 1  //print
#define tTI tPR + 1  //check time
#define tBR tTI + 1  //brightness
#define tRGB tBR + 1 //RGB
#define tSK tRGB + 1 //blue
#define tWH tSK + 1  //white

// led values
#define REDGRN 0b10011001
#define GRNRED 0b01100110
#define GRN 0b01010101
#define RED 0b10101010
#define GRNF 0b01110111
#define REDF 0b10111011
#define OFF 0b11111111

// keypad Values
#define BWIN    20
#define BLEFT   BWIN
#define BPLUS   152 //142
#define BMINUS  327
#define BRIGHT  504
#define BSEL    741
#define KLEFT   1
#define KPLUS   2
#define KMINUS  3
#define KRIGHT  4
#define KSEL    5
#define BMAX    1023-BWIN

byte toks[] = {
  'o', 'n', tON,
  'o', 'f', tOF, 
  'f', 0, tOF, 
  's', 'l', tSL, 
  'm', 'e', tME, 
  'f', 'a', tFA, 
  'l', 'e', tLE, 
  'm', 'o', tMO, 
  'a', 'd', tAD, 
  'b', 'l', tBL, 
  'r', 0, tRE, 
  'g', 0, tGR,
  'v', 'e', tVE,
  's', 't', tST,
  't', 'e', tTE,
  'h', 'u', tHU,
  'h', 'i', tHI,
  'l', 'o', tLO,
  '?', 0, tQE,
  'n', 'u', tNU,
  'p', 'r', tPR,
  't', 'i', tTI,
  'b', 'r', tBR,
  'r', 'g', tRGB,
  'b', 0, tSK,
  'w', 0, tWH,
  0 
};

char c;                       // input from Serial
char buf[32];                 // aggregated user input
char chunk[9];                // individual word in input
bool keyState = false;        // is button being pressed
bool exec = true;             // flag that allows execution of commands if no errors
int reading = 0;              // Reads analog pin A1
int keyPressed;               // 1-5 value representing a physical key
int err;                      // DHT22
int bufSize = 0;              // size counter of original input
int chunkSize = 0;            // size counter of individual words
int tSize = 0;                // size counter of tokbuf
int chunkNum;                 // converted inputted numbers 
int isWord = 0;               // 0 = undefined, 1 = word, 2 = number, 3 = error
int interval = 1000;           // timing of led blink

int queryDelay = 15000;     // 15000 = 15 seconds
int writeDelay = 900000;    // 900000 = 15 minutes
//int writeDelay = 10000;       // faster temp/hum write delay for development
//int queryDelay = 1000;        // faster temp/hum write delay for development

int humIdx = 0;               //starting index in EEPROM
int tempIdx = 100;            //starting index in EEPROM
int bestIdx;                  // index of high or low value during search
int tHead = 100;              // pointer to oldest temp value in EEPROM
int tTail = 100;              // tail that tracks newest temp addition to EEPROM
int hHead = 0;                // pointer to oldest hum value in EEPROM
int hTail = 0;                // tail that tracks newest hum addition to EEPROM    
int headTime = 196;           // 196: Year, 197: Month, 198: Day, 199: Hour, 200: Minute
static byte vec = 0;           // vector used in blinker()
static byte chvec = 0;         // vector used in recordStats()
byte sx, ex;                   // start and ending indexes of each word
byte colArr[5];               // user defined sequence of colors
byte tokbuf[15];               // array of tokens made from input
byte rotary = OFF;             // byte being rotated and read constantly, off by default
byte kpad = 0;                 // keypad vector
float curHum;                 // updated every 15 seconds
float curTemp;                // updated every 15 seconds  
unsigned long debounceTime = 0;                              

void printTok(byte tok){    // TODO: get this done dynamically
  if (tok == tEOL){
    Serial.print("<EOL>");
    Serial.println(tok);
    Serial.println(" ");
  } else if (tok == tERR){
    Serial.print("<ERR>");
    Serial.println(tok);
  } else if (tok == tOF){
    Serial.print("<OFF>");
    Serial.println(tok);
  } else if (tok == tGR){
    Serial.print("<GRE>");
    Serial.println(tok);
  } else if (tok == tSL){
    Serial.print("<SLO>");
    Serial.println(tok);
  } else if (tok == tME){
    Serial.print("<MED>");
    Serial.println(tok);
  } else if (tok == tFA){
    Serial.print("<FAS>");
    Serial.println(tok);
  } else if (tok == tLE){
    Serial.print("<LED>");
    Serial.println(tok);
  } else if (tok == tMO){
    Serial.print("<MOT>");
    Serial.println(tok);
  } else if (tok == tBL){
    Serial.print("<BLI>");
    Serial.println(tok);
  } else if (tok == tVE){
    Serial.print("<VER>");
    Serial.println(tok);
  } else if (tok == tON){
    Serial.print("<ON>");
    Serial.println(tok);
  } else if (tok == tHI){
    Serial.print("<HIG>");
    Serial.println(tok);
  } else if (tok == tRE){
    Serial.print("<RED>");
    Serial.println(tok);
  } else if (tok == tST){
    Serial.print("<STA>");
    Serial.println(tok);
  } else if (tok == tTE){
    Serial.print("<TEM>");
    Serial.println(tok);
  } else if (tok == tHU){
    Serial.print("<HUM>");
    Serial.println(tok);
  } else if (tok == tQE){
    Serial.print("<?>");
    Serial.println(tok);
  } else if (tok == tLO){
    Serial.print("<LOW>");
    Serial.println(tok);
  } else if (tok == tNU){
    Serial.print("<EEPROM Cleared>");
    Serial.println(tok);
  } else if (tok == tPR){
    Serial.print("<PRI>");
    Serial.println(tok);
  } else if (tok == tTI){
    Serial.print("<TIM>");
    Serial.println(tok);
  } else if (tok == tRGB){
    Serial.print("<RGB>");
    Serial.println(tok);
  } else if (tok >= 128){
    Serial.print("<");
    Serial.print(tok);
    Serial.print(">");
    Serial.println(tok - 128);
  }
}

// turn array of led colors into a byte
void translate(char arr[]) {
  rotary = OFF;
  for (int u = 0; u < 4; u++){      
      if (arr[u] == tRE){ // 10
        rotary = rotary ^ (1 << 6);
      } else if(arr[u] == tGR){ //01
        rotary = rotary ^ (1 << 7);
      } // else arr[u] == off, keep 11
      rotl(2);
  }
}

// rotate a byte howMany bits left
byte rotl(int howMany) { 
    rotary = (rotary >> howMany) | (rotary << (8 - howMany));
}

//constantly reading and rotating rotary byte and powering led with rotary's contents
void blinker() {
  static unsigned long int dly;
  switch(vec){
    case 0:
      rotary & (1 << 7) ? digitalWrite(3, HIGH) : digitalWrite(3, LOW);
      rotary & (1 << 6) ? digitalWrite(4, HIGH) : digitalWrite(4, LOW);
      rotl(2);
      dly = millis();
      vec++;
      break;
    case 1:   
      if ((millis() - dly) > interval) {
        dly = millis();
        vec++;
      }
      break;
    case 2:
      if((millis() - dly) > interval) {
        vec = 0;
      }
      break;
    default:
      vec = 0;
      break;
 }
}

// print unrecognized words
void invalidCmd(char *cmd) {
  Serial.print("Unrecognized Command: ");
  Serial.println(cmd);
  tokbuf[tSize] = tERR; // 126
  tSize++; 
  tokbuf[tSize] = tEOL; 
}

// turn word into byte
void getToken(char sent[], byte sx, byte ex){
  char cmd[10];
  int i;
  int j = 0; 

  for(i = sx; i <= ex;  i++) {  // get command substring from buf
       cmd[i - sx] = sent[i];
     }
  cmd[i - sx] = '\0';
  while(toks[j] != 0){  // search TOKS for match
    if(toks[j] == cmd[0] && toks[j + 1] == cmd[1]) { 
      tokbuf[tSize] = toks[j + 2];
      tSize++;
      tokbuf[tSize] = tEOL;
      return 0;
    } else {
      j += 3;
    }        
  } 
  invalidCmd(cmd); // word not found in toks 
}

// clearChunk chucks chunk
void clearChunk(void) {
    memset(chunk, '\0', sizeof(chunk));
    isWord = 0;
    chunkSize = 0;
}  

void lightOne(int r, int g, int b, int which) {
  which -= 128;
  leds[which] = CRGB(r, g, b);
  FastLED.show();
}

void lightAll(int r, int g, int b) {
  for(int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB(r, g, b);
    FastLED.show();
  }
}

void rgbCycle(int num){
  for(int i = 0; i < 255; i++){
    FastLED.showColor(CHSV(i++, 255, 255)); 
    delay(10);
  }
}

// converts chunk to int if number, token if string
void parseCmd(char buf[]){
  lcd.clear();
  lcd.print(buf);
  for (int i = 0; i <= bufSize; i++) {
    Serial.print(buf[i]);
    if ((buf[i] >= 65 && buf[i] <= 90) || (buf[i] >= 97 && buf[i] <= 122) || buf[i] == 63){ // letter
      if (isWord == 1 || isWord == 0) {
        isWord = 1;
        buf[i] = tolower(buf[i]);
        chunk[chunkSize] = buf[i];
        chunkSize++;
      } else { 
        isWord = 3;
        chunk[chunkSize] = buf[i];
        chunkSize++;
      }
    } else if (buf[i] >= 48 && buf[i] <= 57) { // number
        if (isWord == 2 || isWord == 0){
          isWord = 2;
          chunk[chunkSize] = buf[i];
          chunkSize++;
        } else {
          chunk[chunkSize] = buf[i];
          chunkSize++;
        }
    } else if (buf[i] == 32 || buf[i] == 10 || buf[i] == 13 || i == bufSize){ // space or return
        if ((chunkSize > 0) && (isWord == 1)) { // completed word
          getToken(buf, i - chunkSize, i - 1);
          clearChunk();
        } else if((chunkSize > 0) && (isWord == 2)) { // completed number, convert to int and check size
          chunk[chunkSize++] = '\0';
          chunkNum = 0;
          for(int k = 0; k < strlen(chunk); k++){
            chunkNum = 10 * chunkNum + (chunk[k] - '0');
          }
          if(chunkNum > 127 || chunkNum < 0) {
            Serial.print("Number too Large: "); // if number > 127, complain and ignore
            Serial.println(chunk);
          } else {
            chunkNum += 128;
            tokbuf[tSize] = chunkNum;
            tSize++;
          }
          clearChunk();
        } else if((chunkSize > 0) && (isWord == 3)) { // illegal combination
          chunk[chunkSize++] = '\0';
          Serial.print("PARSE ERROR: ");
          Serial.println(chunk);
          clearChunk();
        }
     } 
   }
 }

// accepts array of tokens and takes action based on its contents
  void executeCmd(byte *tBuff) {
    if((tBuff[0] == tVE) && (tBuff[2] == tEOL)) {
        Serial.println(" ");
        Serial.println("Version 1.4.0");
    } else if ((tBuff[0] == tLE) || (tBuff[1] == tLE)) {
      vec = 0;  // reset blinker cycle
      if (((tBuff[1] == tGR) || (tBuff[0] == tGR)) && ((tBuff[2] == tON) && (tBuff[3] == tEOL))) {
          rotary = GRN;
          lightAll(0, 255, 0);
      } else if((tBuff[1] == tON) && (tBuff[2] == tEOL)){
          rotary = GRN;
          lightAll(0, 255, 0);
      } else if(((tBuff[1] == tRE) || (tBuff[0] == tRE)) && (tBuff[2] == tON) && (tBuff[3] == tEOL)){
          rotary = RED; 
          lightAll(255, 0, 0);
      } else if(((tBuff[1] == tRE) || (tBuff[1] == tGR) || (tBuff[1] == tOF)) && ((tBuff[2] == tRE) || (tBuff[2] == tGR) || (tBuff[2] == tOF)) && ((tBuff[3] == tRE) || (tBuff[3] == tGR) || (tBuff[3] == tOF)) && ((tBuff[4] == tRE) || (tBuff[4] == tGR) || (tBuff[4] == tOF)) && (tBuff[5] == tBL) && (tBuff[6] == tEOL)) {
          colArr[0] = tBuff[1];
          colArr[1] = tBuff[2];
          colArr[2] = tBuff[3];
          colArr[3] = tBuff[4];
          translate(colArr);
      } else if((tBuff[1] == tBL) && (tBuff[2] == tEOL)) {
          rotary = GRNF; // green blink
      } else if(((tBuff[1] == tGR) || (tBuff[0] == tGR)) && (tBuff[2] == tBL) && (tBuff[3] == tEOL)) {
          rotary = GRNF; // green blink
      } else if(((tBuff[1] == tRE) || (tBuff[0] == tRE)) && (tBuff[2] == tBL) && (tBuff[3] == tEOL)) {
          rotary = REDF; // red blink
      } else if((tBuff[1] == tRE) && (tBuff[2] == tGR) && (tBuff[3] == tBL) && (tBuff[4] == tEOL)) {
          rotary = REDGRN; // red green blink
      } else if((tBuff[1] == tGR) && (tBuff[2] == tRE) && (tBuff[3] == tBL) && (tBuff[4] == tEOL)) {
          rotary = GRNRED; // green red blink 
      } else if((tBuff[1] == tRE) && (tBuff[2] == tGR) && (tBuff[3] == tST)&& (tBuff[4] == tEOL)){
          rotary = REDGRN; // red green blink
      } else if ((tBuff[1] == tGR) && (tBuff[2] == tRE) && (tBuff[3] == tST)&& (tBuff[4] == tEOL)){
          rotary = GRNRED; // green red blink
      } else if ((tBuff[1] == tST)&& (tBuff[2] == tEOL)){
          rotary = GRNRED; // green red blink
      } else if ((tBuff[1] == tFA) && (tBuff[2] == tEOL)){
          interval = 100; // set interval to 1/10 second
      } else if ((tBuff[1] == tME) && (tBuff[2] == tEOL)){
          interval = 1000; // set interval to 1 second
      } else if ((tBuff[1] == tSL) && (tBuff[4] == tEOL)){
          interval = 2000; // set interval to 2 seconds
      } else if ((tBuff[1] == tOF) && (tBuff[2] == tEOL)){
          rotary = OFF;
          lightAll(0, 0, 0);
      } else if ((tBuff[1] == tRGB)) {
        if ((tBuff[2] == tBR) && (tBuff[3] == tHI) && (tBuff[4] == tEOL)) {
          FastLED.setBrightness( 255 );
          FastLED.show();
        } else if ((tBuff[2] == tBR) && (tBuff[3] == tME) && (tBuff[4] == tEOL)) {
          FastLED.setBrightness( 128 );
          FastLED.show();
        } else if ((tBuff[2] == tBR) && (tBuff[3] == tLO) && (tBuff[4] == tEOL)) {
          FastLED.setBrightness( 30 );
          FastLED.show();
        } else if ((tBuff[2] >= 128) && (tBuff[3] == tOF)) {
          lightOne(0, 0, 0, tBuff[2]);
        } else if ((tBuff[2] >= 128) && (tBuff[3] == tRE)) {
          lightOne(255, 0, 0, tBuff[2]);
        } else if ((tBuff[2] >= 128) && (tBuff[3] == tGR)) {
          lightOne(0, 255, 0, tBuff[2]);
        } else if ((tBuff[2] >= 128) && (tBuff[3] == tSK)) {
          lightOne(0, 0, 255, tBuff[2]);
        } else if ((tBuff[2] >= 128) && (tBuff[3] == tWH)) {
          lightOne(255, 255, 255, tBuff[2]);
        } else if (tBuff[2] == tOF){
          lightAll(0, 0, 0);
        } else if (tBuff[2] == tRE){
          lightAll(255, 0, 0);
        } else if (tBuff[2] == tGR){
          lightAll(0, 255, 0);
        } else if (tBuff[2] == tSK){
          lightAll(0, 0, 255);
        } else if (tBuff[2] == tWH){
          lightAll(255, 255, 255);
        } else {
          for(int i = 0; i < NUM_LEDS; i++){
            rgbCycle(i);
          }
        }
      }
    } else if (tBuff[0] == tTE) { // temp
       if (tBuff[1] == tQE) {
        Serial.println(" ");
        Serial.print("Current Temperature: ");
        Serial.print(curTemp);
        Serial.println(" F");
       } else if (tBuff[1] == tHI) { // temp high
          Serial.println(" ");
          Serial.print("Highest temperature recorded in last 24 hours: ");
          Serial.print(readStats(tTE, tHI));
          Serial.println(" F");
          findTime(tTE);
       } else if (tBuff[1] == tLO) { // temp low
          Serial.println(" ");
          Serial.print("Lowest temperature recorded in last 24 hours: ");
          Serial.print(readStats(tTE, tLO));
          Serial.println(" F");
          findTime(tTE);
       } else if (tBuff[1] == tPR) {
          printData(tTE);
       }
    } else if (tBuff[0] == tHU) { //humidity
      if (tBuff[1] == tQE) {
        Serial.println(" ");
        Serial.print("Current Humidity: ");
        Serial.print(curHum);
        Serial.println("%");
       } else if (tBuff[1] == tHI) { //humidity high
          Serial.println(" ");
          Serial.print("Highest humidity recorded in last 24 hours: ");
          Serial.print(readStats(tHU, tHI));
          Serial.println("%");
          findTime(tHU);
       } else if (tBuff[1] == tLO) { //humidity low
          Serial.println(" ");
          Serial.print("Lowest humidity recorded in last 24 hours: ");
          Serial.print(readStats(tHU, tLO));
          Serial.println("%");
          findTime(tHU);
       }  else if (tBuff[1] == tPR) {
          Serial.println(" ");
          printData(tHU);
       }
    } else if (tBuff[0] == tNU) { // erase EEPROM 
        nuke();
    } else if (tBuff[0] == tTI) { // check time setting
      Serial.print("The time has been set to: ");
      Clock.printTo(Serial);
      Serial.println();
      storeTime();
    } else {
      Serial.println(" ");
      Serial.println("Command Incomplete");
    }
    
  }

void parseExecute(char inBuf[]){
  parseCmd(inBuf);
  if (exec){ // if no errors 
    executeCmd(tokbuf); 
  }
  Serial.println("");
  for(int z = 0; z <= tSize; z++) { // print tokbuf
        printTok(tokbuf[z]);
  }
            
  tSize = 0;
  memset(tokbuf, '\0', sizeof(tokbuf));
}

/********************************** Temp and Humidity Sensor **************************************/
// empty out EEPROM, reset heads and tails, take new reading
void nuke(){
  for( int k = 0; k < 196; k++) {
          EEPROM.write(k, 0);
  }
  hTail = 0;
  tTail = 100;
  hHead = 0;
  tTail = 100;
  chvec = 0;
  storeTime();
}

// write timestamp to EEPROM
void storeTime(){
    DateTime theTime = Clock.read();
    EEPROM.write(headTime, theTime.Year);
    EEPROM.write(headTime + 1, theTime.Month);
    EEPROM.write(headTime + 2, theTime.Day);
    EEPROM.write(headTime + 3, theTime.Hour);
    EEPROM.write(headTime + 4, theTime.Minute);
  
//  for(int i = 0; i < 5; i++){ // print timestamp upon write
//    int yargh = EEPROM.read(headTime + i);
//    Serial.println(yargh);
//  }
}

// reads temp or hum to find highest or lowest value
int readStats(byte humTemp, byte hiLo) {
  int index = (humTemp == tTE) ? 100 : 0;
  float best = EEPROM.read(index);
  float current;
  
  for( int y = 0; y <= 95; y++) {
      current = EEPROM.read(index);
      if((current != 255) && (current != 0)) { // if there is a value there to read
        if(hiLo == tHI) { // best = highest value found
          if((current > best) || (current == best)) {
            best = current;
            bestIdx = index;
          }
        } else if(hiLo == tLO) {  // best = lowest value found
          if((current < best) || (current == best)) {
            best = current;
            bestIdx = index;
          }
        }
      }
      index++;
  }
  return best;
}

// increment tail, head and loop back to 0 when appropriate
void adjPtrs(){
      if (hTail >= 95){
        hTail = 0;
      } else {
        hTail++;
      }
      if (tTail >= 195){
        tTail = 100;
      } else {
        tTail++;
      }
      if (hTail == hHead){
        hHead++;
      }
      if (tTail == tHead){
        tHead++;
        storeTime();
      }
      if (hHead >= 95){
        hHead = 0;
      }
      if (tHead >= 195){
        tHead = 100;
        storeTime();
      }
}

void printData(byte stat) { // print values from tail to head
  int start = (stat == tTE) ? tTail : hTail;
  int head = (stat == tTE) ? tHead : hHead;
  int limit = (stat == tTE) ? 100 : 0;
  start -= 1;
  int index = start;
  float current;

  while(index != (head - 1)) {
    if(index < limit) {
      index = (stat == tTE) ? 195 : 95;
    }
    current = EEPROM.read(index);
    Serial.print(index);
    Serial.print(": ");
    Serial.println(current);
    index--;
  }
  if(tHead != 100) {
    current = EEPROM.read(index);
    Serial.print(index);
    Serial.print(": ");
    Serial.println(current);
  }
}

void recordStats(float hum, float temp){
  static unsigned long int dly;
  int timeCheck;
  int hum2 = (int)hum;
  int temp2 = (int)temp;
  switch(chvec){
    case 0:
      EEPROM.write(hTail, hum2);
      EEPROM.write(tTail, temp2);
      Serial.print(hum2);
      Serial.print(" stored at ");
      Serial.println(hTail);
      Serial.print(temp2);
      Serial.print(" stored at ");
      Serial.println(tTail);
      Serial.print("hum and temp head ");
      Serial.print(hHead);
      Serial.print(" ");
      Serial.println(tHead);
      
      timeCheck = EEPROM.read(headTime);
      if (timeCheck == 0) { // if no timestamp in EEPROM
        storeTime();
      }
      adjPtrs();
      dly = millis();
      chvec++;
      break;
    case 1:   
      if ((millis() - dly) > writeDelay) {
        chvec++;
      }
      break;
    case 2:
      if((millis() - dly) > writeDelay) {
        chvec = 0;
      }
      break;
    default:
      chvec = 0;
      break;
 }
}

void takeReading(){
  static byte vect = 0;
  static unsigned long int dely;
  switch(vect){
    case 0:
      dht22.read2(pinDHT22, &curTemp, &curHum, NULL);
      curTemp = (1.8 * curTemp) + 32; // convert from nonsense units
      dely = millis();
      vect++;
      break;
    case 1:   
      if ((millis() - dely) > queryDelay) {
        dely = millis();
        vect++;
      }
      break;
    case 2:
      if((millis() - dely) > queryDelay) {
        vect = 0;
      }
      break;
    default:
      vect = 0;
      break;
 }
}

// calculates timestamp of highest or lowest value stored in EEPROM
void findTime(byte stat){
  int eIdx = bestIdx;
  int count = 0;
  int header = (stat == tTE) ? tHead : hHead;
  int limiter = (stat == tTE) ? 100 : 0;
  int elapseM;
  int minutes = 15; 
  int hYear = EEPROM.read(headTime);
  int hMonth = EEPROM.read(headTime + 1);
  int hDay = EEPROM.read(headTime + 2);
  int hHour = EEPROM.read(headTime + 3);
  int hMinute = EEPROM.read(headTime + 4); 
  
  while (eIdx != header) {
    if(eIdx <= limiter) { // loop to top at 0 or 100
      eIdx = limiter + 95;
    }
    count++;
    eIdx--;
  }
//  Serial.println("");
//  Serial.print("hi/lo value index: ");
//  Serial.println(bestIdx);
//  Serial.print("indexes between head and hi/low value: ");
//  Serial.println(count);
  while (count > 0) {
    hMinute += 15;
    if (hMinute > 60) {
      hMinute -= 60;
      hHour += 1;
    }
    if (hHour > 24) {
      hHour = 0;
      hDay ++;
    }
    count--;
  }
  Serial.print("Recorded at ");
  Serial.print(hHour);
  Serial.print(":");
  Serial.print(hMinute);
  Serial.print(" on ");
  Serial.print(hMonth);
  Serial.print("/");
  Serial.print(hDay);
  Serial.print("/");
  Serial.println(hYear);
}

/******************************************* Ethernet *****************************************/

//callback that prints received packets to the serial port
void udpSerialPrint(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, const char *data, uint16_t len){
  IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);
  
//  Serial.print("source port: ");
//  Serial.println(src_port);
  Serial.print("Source ip: ");
  ether.printIp(src_ip);
  Serial.println("");
//  Serial.print("data: ");
//  Serial.println(data);
//  Serial.println("");

  strcpy(buf, data);
  bufSize = strlen(buf);
  parseExecute(buf);
  bufSize = 0;
  memset(buf, '\0', sizeof(buf));
}

/*************Keypad **********************/

void lcdOut(int key) {
  switch(key) {
    case 1:
      lcd.clear();
      lcd.print("Left");
      break;
    case 2:
      lcd.clear();
      lcd.print("Up");
      break;
    case 3:
      lcd.clear();
      lcd.print("Down");
      break;
    case 4:
      lcd.clear();
      lcd.print("Right");
      break;
    case 5:
      lcd.clear();
      lcd.print("Select");
      break;
  }
}

void keypad() {
  switch (kpad) {
    case 0:
      keyState = 0;
      kpad++;
      break;
    case 1:
      reading = analogRead(keypadIn);      // read analog value
      keyPressed = keyConversion(reading);  // convert reading into key pressed value
      lcdOut(keyPressed);
      kpad++;
      break;
    case 2:
      if (checkDebounce()) {            // Check debounce time
        if (keyPressed != 0 && !keyState) {
          keyState = true;
        } else if (keyPressed == 0) {
          keyState = false;
        }
      }
      kpad++;
      break;
    default:
      kpad = 0;           // Reset
      break;
  }
}

// convert key inputs
int keyConversion(int key) {
  if (key > BMAX) return 0;
  if (key >= BLEFT - BWIN && key <= BLEFT + BWIN) return KLEFT;
  if (key >= BPLUS - BWIN && key <= BPLUS + BWIN) return KPLUS;
  if (key >= BMINUS - BWIN && key <= BMINUS + BWIN) return KMINUS;
  if (key >= BRIGHT - BWIN && key <= BRIGHT + BWIN) return KRIGHT;
  if (key >= BSEL - BWIN && key <= BSEL + BWIN) return KSEL;
}

// ensure one input for each keypress
boolean checkDebounce() {
  // Check if current time and previous time is greater than debounce interval
  const byte debounceInterval = 100;         // Debounce Interval
  if (millis() >= debounceTime) {
    debounceTime += debounceInterval;
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(  128 );   // default to medium brightness


  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
    Serial.println( "Failed to access Ethernet controller");
  }
  ether.staticSetup(myip, gwip);
  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);  
//  ether.printIp("DNS: ", ether.dnsip); 
  Serial.println("");

  //register udpSerialPrint() to port 2001
  ether.udpServerListenOnPort(&udpSerialPrint, 2001);

  //register udpSerialPrint() to port 19.
  ether.udpServerListenOnPort(&udpSerialPrint, 19);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("I'm alive!"); 

  Clock.begin();
  // uncomment next 8 lines to set time (24hr format)
//  DateTime MyTimestamp;
//  MyTimestamp.Day    = 5;
//  MyTimestamp.Month  = 10;
//  MyTimestamp.Year   = 17; 
//  MyTimestamp.Hour   = 15;
//  MyTimestamp.Minute = 10;
//  MyTimestamp.Second = 33;
//  Clock.write(MyTimestamp);

  
}

void loop() { 
  /************************************* Ethernet ******************************************/
  //this must be called for ethercard functions to work.
//  ether.packetLoop(ether.packetReceive());
  /*************************** Temperature and Humidity Readings ****************************/
  takeReading(); // record current temp/hum
  recordStats(curHum, curTemp); // write temp/hum to EEPROM
  /************************ Tokenizing and Executing Commands From Serial *******************/
  blinker(); // blink/status
  keypad();  // keypad input
  if(Serial.available()) { // aggregate into buffer letter by letter until enter is pressed
    c = Serial.read();
    buf[bufSize] = c; 
    bufSize++;
    if (c == 10 || c == 13){ // command complete
      Serial.println("");
      buf[bufSize] = '\0';
      parseExecute(buf);
      bufSize = 0;
      memset(buf, '\0', sizeof(buf));
    } 
  }
}
