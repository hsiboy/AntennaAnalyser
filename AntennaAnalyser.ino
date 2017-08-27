/*
*  RF Analyzer
*  
*    LogAmp AD8307
*    DDS AD9850
*    2.2" or 2.4" TFT display with ILI9341 driver
*    
*    Based on work by JA2NKD Ryuu Matsuura
*    All rights reserved.
*
*/


#include <SPI.h>         // supplied via arduino IDE.
#include <Ucglib.h>      // https://github.com/olikraus/ucglib - v1.4.0
#include <Rotary.h>      // https://github.com/brianlow/Rotary - v1.1
#include <EEPROM.h>      // supplied via arduino IDE. 

/*
* TFT display uses hardware SPI interface.
* Hardware SPI pins are specific to the Arduino board type and cannot be remapped to alternate pins.
*
* SCLK connects to SPI Clock. On Arduino Uno/Duemilanove/328-based, thats Digital 13. On Mega's, its Digital 52 and on Leonardo/Due its ICSP-3
* MISO connects to SPI MISO. On Arduino Uno/Duemilanove/328-based, thats Digital 12. On Mega's, its Digital 50 and on Leonardo/Due its ICSP-1
* MOSI connects to SPI MOSI. On Arduino Uno/Duemilanove/328-based, thats Digital 11. On Mega's, its Digital 51 and on Leonardo/Due its ICSP-4
* 
* CS connects to SPI Chip Select pin. We'll be using Digital 10.
* DC connects to SPI Data/Command select pin. We'll be using Digital 9.
* RST connects to the display reset pin. We'll be using Digital 8.
*
* Note: display is probably 3.3v Arduino Uno is 5v!
*/

const byte __CS  = 10;
const byte __DC  = 9;
const byte __RST = 8;

// AD9850 DDS
const byte DDS_RST   = 4;
const byte DDS_DATA  = 5;
const byte DDS_FQ_UD = 6;
const byte DDS_W_CLK = 7;

// set some constants.
const unsigned long TWO_E = 4294967295L;   // 2e32 AD9850,9851

// commands
const byte DDS_CMD  = B00000000;   // AD9851 B00000001
const byte AD8307   = A5;
unsigned long DDS_CLK = 125000000L;   // AD9851 180000000

long DDS_freq = 0;
long DDSFreqMin = 0;
long DDSfMinOld = 0;
long DDSFreqMax = 55000000;
long DDSfMaxOld = 55000000;
float DDSfPit = 0;

/* 
* Buttons (user interface)
*   Buttons (SPST non latching) will connect between pin and GND.
*   Arduino pullups will be enabled in setup().
*/
 
const byte swMenu  = A0;
const byte swEnter = A1;
const byte swRight = A3;
const byte swLeft  = A2;

// inititialize display driver
Ucglib_ILI9341_18x240x320_HWSPI ucg(__DC, __CS, __RST);

/*
*  Rotary Encoder:
*    encoder pin A to Arduino pin D2
*    encoder pin B to Arduino pin D3
*    encoder ground pin to GND
*/

Rotary r = Rotary(2, 3);

boolean fCountFlag = false;
int ADOut = 0;
int ADOutMax = 0;
int ADOutMin = 1023;
long ADOutMaxFreq = 0;
long ADOutMinFreq = 0;

int ADMaxI = 0;
int ADMinI = 0;

int xOld = 10;
int yOld = 230;

float dBm = 0;
float dBmOld = 100;

int menuIdx = 1;

long freqStep = 10000000;
int freqStepFlag = 8;
long sigGenFreq = 10000000;
long sigGenFreqOld = 56000000;
int configFlag = 0;
int mode = 0;
long offsetPix = 0;
long offsetPixOld = 0;
int romCheck = 0;
float AD_a = 0.09;
float AD_b = -86.06;
float SWR = 1;
float rLoss = 1;

void setup() {

  pinMode(DDS_RST, OUTPUT);
  pinMode(DDS_DATA, OUTPUT);
  pinMode(DDS_FQ_UD, OUTPUT);
  pinMode(DDS_W_CLK, OUTPUT);
  // enable pullup resistors for buttons
  pinMode(swMenu, INPUT_PULLUP);
  pinMode(swEnter, INPUT_PULLUP);
  pinMode(swRight, INPUT_PULLUP);
  pinMode(swLeft, INPUT_PULLUP);
  analogReference(EXTERNAL);

// if unit is switched on with "enter" button pressed, clear the eeprom to defaults.
  if ( digitalRead(swEnter) == LOW) {
    eepromInitWrite();
  }
  // check contents of eeprom, and recover if unexpected result.
  romCheck = EEPROM.read(0x001);
  if (romCheck != 104 ) {
    eepromInitWrite();
  }

  else {
    EEPROM.get(0x010, DDS_CLK);
    EEPROM.get(0x100, AD_a);
    EEPROM.get(0x120, AD_b);
    EEPROM.get(0x070, sigGenFreq);
  }

  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  //  ucg.setRotate270();
  ucg.setRotate90();

// Pin Change Interrupt Control Register
  PCICR |= (1 << PCIE2);
// Pin Change Mask Register 2  
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
// turn on interrupts
  sei();
  
  DDS_init();
  menu1();
  menu2();
}

void loop(void) {
  if (digitalRead(swMenu) == LOW) {
    menuIdx = menuIdx + 1;
    if (menuIdx >= 8) {
      menuIdx = 1;
    }
    menu2();
    while (digitalRead(swMenu) == LOW) {}
  }
  if (digitalRead(swEnter) == LOW) {
    switch (menuIdx) {
      case 1:
        signalg();
        break;
      case 2:
        ucg.clearScreen();
        powermeter();
        break;
      case 3:
        ucg.clearScreen();
        mode = 3;
        FRM();
        break;
      case 4:
        ucg.clearScreen();
        mode = 4;
        FRM();
        break;
      case 5:
        ucg.clearScreen();
        DDSajust();
        break;
      case 6:
        ucg.clearScreen();
        ADcheck();
        break;
    }
  }
}

// ------------------ Encoder Interrupt --------------------------
ISR(PCINT2_vect) {
  float minfprint = 0;
  float maxfprint = 0;
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) {
      if (mode == 1) {
        sigGenFreq = sigGenFreq + freqStep;
      }
      if ((mode == 3) or (mode == 4)) {
        ucg.setColor(0, 0, 0);
        ucg.drawBox(165, 172, 140, 30);
        ucg.setColor(0, 240, 240);
        if (configFlag == 1) {
          DDSFreqMin = DDSFreqMin + freqStep;
          DDSFreqMin = constrain(DDSFreqMin, 0, 55000000);
          ucg.setPrintPos(180, 195);
          minfprint = (float)DDSFreqMin / 1000;
          ucg.print(minfprint, 3);
        }
        if (configFlag == 2) {
          DDSFreqMax = DDSFreqMax + freqStep;
          DDSFreqMax = constrain(DDSFreqMax, 0, 55000000);
          ucg.setPrintPos(180, 195);
          maxfprint = (float)DDSFreqMax / 1000;
          ucg.print(maxfprint, 3);
        }
        if (configFlag == 3) {
          offsetPix = offsetPix + freqStep;
          offsetPix = constrain(offsetPix, -30, 30);
          ucg.setPrintPos(180, 195);
          ucg.print(offsetPix);
        }
      }
      if (mode == 5) {
        DDS_CLK = DDS_CLK + freqStep;
        DDS_CLK = constrain(DDS_CLK, 40000000, 126000000);
        ucg.setColor(0, 0, 0);
        ucg.drawBox(165, 172, 140, 30);
        ucg.setColor(0, 240, 240);
        ucg.setPrintPos(180, 195);
        ucg.print(DDS_CLK);
        DDS_freqset(10000000);
        EEPROM.put(0x10, DDS_CLK);
      }
    }

    else {
      if (mode == 1) {
        sigGenFreq = sigGenFreq - freqStep;
      }
      if ((mode == 3) or (mode == 4)) {
        ucg.setColor(0, 0, 0);
        ucg.drawBox(165, 172, 140, 30);
        ucg.setColor(0, 240, 240);
        if (configFlag == 1) {
          DDSFreqMin = DDSFreqMin - freqStep;
          DDSFreqMin = constrain(DDSFreqMin, 0, 55000000);
          ucg.setPrintPos(180, 195);
          minfprint = (float)DDSFreqMin / 1000;
          ucg.print(minfprint, 3);
        }
        if (configFlag == 2) {
          DDSFreqMax = DDSFreqMax - freqStep;
          DDSFreqMax = constrain(DDSFreqMax, 0, 55000000);
          ucg.setPrintPos(180, 195);
          maxfprint = (float)DDSFreqMax / 1000;
          ucg.print(maxfprint, 3);
        }
        if (configFlag == 3) {
          offsetPix = offsetPix - freqStep;
          offsetPix = constrain(offsetPix, -30, 30);
          ucg.setPrintPos(180, 195);
          ucg.print(offsetPix);
        }
      }
      if (mode == 5) {
        DDS_CLK = DDS_CLK - freqStep;
        DDS_CLK = constrain(DDS_CLK, 40000000, 180000000);
        ucg.setColor(0, 0, 0);
        ucg.drawBox(165, 172, 140, 30);
        ucg.setColor(0, 240, 240);
        ucg.setPrintPos(180, 195);
        ucg.print(DDS_CLK);
        DDS_freqset(10000000);
        EEPROM.put(0x010, DDS_CLK);
      }
    }
  }
  if (mode == 1) {
    sigGenFreq = constrain(sigGenFreq, 0, 55000000);
  }
}

// --------------------- Main Menu -------------------------------
void menu1() {
  ucg.clearScreen();
  DDS_freqset(0);
  mode = 0;
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(125, 35);
  ucg.print("[ MENU ]");
  ucg.setColor(255, 255, 255);
  ucg.setPrintPos(10, 70);
  ucg.print("1. Signal Generator");
  ucg.setPrintPos(10, 95);
  ucg.print("2. Power Meter");
  ucg.setPrintPos(10, 120);
  ucg.print("3. Frequency Responce");
  ucg.setPrintPos(10, 145);
  ucg.print("4. Antenna Analyzer");
  ucg.setPrintPos(10, 170);
  ucg.print("5. DDS Adjustment");
  ucg.setPrintPos(10, 195);
  ucg.print("6. AD8307 Adjustment");
  ucg.setPrintPos(30, 235);
  ucg.setFont(ucg_font_7x14B_tr);
  ucg.setColor(0, 240, 240);
  ucg.print("RF Analyzer Ver 1.0");
}

// -------------------- Main Menu sub ----------------------------
void menu2() {
  ucg.setFont(ucg_font_fub17_tr);
  switch (menuIdx) {
    case 1:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 195);
      ucg.print("6. AD8307 Adjustment");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 70);
      ucg.print("1. Signal Generator");
      break;
    case 2:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 70);
      ucg.print("1. Signal Generator");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 95);
      ucg.print("2. Power Meter");
      break;
    case 3:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 95);
      ucg.print("2. Power Meter");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 120);
      ucg.print("3. Frequency Responce");
      break;
    case 4:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 120);
      ucg.print("3. Frequency Responce");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 145);
      ucg.print("4. Antenna Analyzer");
      break;
    case 5:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 145);
      ucg.print("4. Antenna Analyzer");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 170);
      ucg.print("5. DDS Adjustment");
      break;
    case 6:
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(10, 170);
      ucg.print("5. DDS Adjustment");
      ucg.setColor(255, 0, 0);
      ucg.setPrintPos(10, 195);
      ucg.print("6. AD8307 Adjustment");
      break;

  }
}

// ------------------- Signal Generator --------------------------
void signalg() {
  sigGenFreqOld = 56000000;
  long counter = 0;
  dBmOld = 100;
  mode = 1;
  EEPROM.get(0x070, sigGenFreq);
  EEPROM.get(0x080, freqStepFlag);
  //  freqStepFlag=7;
  ucg.clearScreen();
  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(10, 30);
  ucg.print("Signal Generator");
  ucg.setPrintPos(10, 155);
  ucg.print("Power meter");
  DDS_freqset(sigGenFreq);
  step();
  while (digitalRead(swMenu) == HIGH) {
    if (sigGenFreq != sigGenFreqOld) {
      DDS_freqset(sigGenFreq);
      ucg.setColor(0, 0, 0);
      ucg.drawBox(47, 60, 232, 40);
      sigGenFreqOld = sigGenFreq;
      ucg.setFont(ucg_font_fub35_tn);
      ucg.setColor(255, 100, 100);
      int xpos = 50;
      if (sigGenFreq >= 0) {
        xpos = 243;
      }
      if (sigGenFreq >= 10) {
        xpos = 215;
      }
      if (sigGenFreq >= 100) {
        xpos = 187;
      }
      if (sigGenFreq >= 1000) {
        xpos = 159;
      }
      if (sigGenFreq >= 10000) {
        xpos = 131;
      }
      if (sigGenFreq >= 100000) {
        xpos = 103;
      }
      if (sigGenFreq >= 1000000) {
        xpos = 75;
      }
      if (sigGenFreq >= 10000000) {
        xpos = 47;
      }
      ucg.setPrintPos(xpos, 100);
      ucg.print(sigGenFreq);
      ucg.setPrintPos(280, 100);
      ucg.setColor(255, 255, 255);
      ucg.setFont(ucg_font_fub17_tr);
      ucg.print("Hz");
      ucg.setFont(ucg_font_7x14B_tr);
      ucg.setColor(255, 255, 255) ;
      ucg.setPrintPos(95, 115);
      ucg.print("M           K");
    }
    if (digitalRead(swLeft) == LOW) {
      freqStepFlag = freqStepFlag - 1;
      if (freqStepFlag < 1) {
        freqStepFlag = 1;
      }
      step();
      EEPROM.put(0x080, freqStepFlag);
      while (digitalRead(swLeft) == LOW) {
        delay(10);
      }
    }
    if (digitalRead(swRight) == LOW) {
      freqStepFlag = freqStepFlag + 1;
      if (freqStepFlag > 8) {
        freqStepFlag = 8;
      }
      step();
      EEPROM.put(0x080, freqStepFlag);
      while (digitalRead(swRight) == LOW) {
        delay(10);
      }
    }
    if (++counter == 100000) {
      ADOut = analogRead(AD8307);
      dBmconv();
      if (dBm != dBmOld) {
        ucg.setFont(ucg_font_fub17_tr);
        ucg.setColor(0, 0, 0);
        ucg.setPrintPos(120, 180);
        ucg.print(dBmOld, 1);
        ucg.setColor(0, 240, 240);
        ucg.setPrintPos(120, 180);
        ucg.print(dBm, 1);
        ucg.setPrintPos(180, 180);
        ucg.setColor(255, 255, 255);
        ucg.print("dBm");
        dBmOld = dBm;

      }
      counter = 0;
    }
  }
  //  DDS_freqset(0);
  EEPROM.put(0x070, sigGenFreq);
  dBmOld = 100;
  menu1();
  menu2();
}

// ----------------- Frequency Step change -----------------------
void step() {
  ucg.setColor(0, 0, 0);
  ucg.drawBox(220, 205, 85, 30);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(100, 100, 100);
  ucg.setPrintPos(140, 227);
  ucg.print("STEP [            ]");
  switch (freqStepFlag) {
    case 1:
      freqStep = 1;
      ucg.setPrintPos(220, 230);
      if (configFlag == 3) {
        ucg.print("   1dB");
      }
      else {
        ucg.print("    1Hz");
      }
      break;
    case 2:
      freqStep = 10;
      ucg.setPrintPos(220, 230);
      if (configFlag == 3) {
        ucg.print("  10dB");
      }
      else {
        ucg.print("  10Hz");
      }
      break;
    case 3:
      if (configFlag == 3) {
        freqStepFlag = 2;
        freqStep = 10;
        ucg.setPrintPos(220, 230);
        ucg.print("  10dB");
      }
      else {
        freqStep = 100;
        ucg.setPrintPos(220, 230);
        ucg.print(" 100Hz");
      }
      break;
    case 4:
      freqStep = 1000;
      ucg.setPrintPos(220, 230);
      ucg.print("  1kHz");
      break;
    case 5:
      freqStep = 10000;
      ucg.setPrintPos(220, 230);
      ucg.print(" 10kHz");
      break;
    case 6:
      freqStep = 100000;
      ucg.setPrintPos(220, 230);
      ucg.print("100kHz");
      break;
    case 7:
      freqStep = 1000000;
      ucg.setPrintPos(220, 230);
      ucg.print("  1MHz");
      break;
    case 8:
      freqStep = 10000000;
      ucg.setPrintPos(220, 230);
      ucg.print(" 10MHz");
      break;
  }
}

// --------------------- Power meter -----------------------------
void powermeter() {
  mode = 2;
  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(10, 30);
  ucg.print("Power meter");
  while (digitalRead(swMenu) == HIGH) {
    ADOut = analogRead(AD8307);
    dBmconv();
    if (dBm != dBmOld) {
      ucg.setFont(ucg_font_fub35_tn);
      ucg.setColor(0, 0, 0);
      if (dBmOld < 0) {
        ucg.setPrintPos(50, 100);
      }
      else {
        ucg.setPrintPos(72, 100);
      }
      ucg.print(dBmOld, 1);
      ucg.setColor(255, 100, 100);
      if (dBm < 0) {
        ucg.setPrintPos(50, 100);
      }
      else {
        ucg.setPrintPos(72, 100);
      }
      ucg.print(dBm, 1);
      ucg.setPrintPos(200, 100);
      ucg.setColor(255, 255, 255);
      ucg.setFont(ucg_font_fub17_tr);
      ucg.print("dBm");
      float watt = pow(10, dBm / 10);
      ucg.setColor(0, 0, 0);
      ucg.drawBox(150, 120, 100, 120);
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(150, 150);
      ucg.print(watt, 5);
      ucg.setPrintPos(250, 150);
      ucg.print("mW");
      float volt = sqrt(watt * 50 / 1000);
      ucg.setPrintPos(150, 180);
      ucg.print(volt);
      ucg.setPrintPos(250, 180);
      ucg.print("Vrms");
      float dbuv = dBm + 106.99;
      ucg.setPrintPos(150, 210);
      ucg.print(dbuv);
      ucg.setPrintPos(250, 210);
      ucg.print("dBuV");
    }
    dBmOld = dBm;
    delay(1000);
  }
  dBmOld = 100;
  menu1();
  menu2();
}


// -------------- Frequency Response Analyzer ---------------------
void FRM() {
  // mode=3;
  float maxfprint = 0;
  float minfprint = 0;
  ADOutMax = 0;
  ADOutMin = 1023;
  fraconfig();
  configFlag = 1;
  Casoldraw();
  noInterrupts();
  if (DDSFreqMin == 0) {
    DDSfPit = (DDSFreqMax / 300);
  }
  else {
    DDSfPit = ((DDSFreqMax - DDSFreqMin) / 300);
  }
  while (digitalRead(swMenu) == HIGH) {
    ucg.setPrintPos(12, 20);
    ucg.setFont(ucg_font_7x14B_tr);
    ucg.setColor(255, 0, 0);
    minfprint = (float)DDSFreqMin / 1000;
    ucg.print(minfprint, 3);
    ucg.setPrintPos(250, 20);
    maxfprint = (float)DDSFreqMax / 1000;
    ucg.print(maxfprint, 3);
    ucg.setColor(255, 255, 255);
    ucg.setPrintPos(120, 20);
    ucg.print("10dB/  ref=");
    ucg.print(offsetPix);
    xOld = 10; yOld = 235;
    ucg.setColor(255, 255, 255);
    ucg.setPrintPos(15, 232);
    float span = (float)(DDSFreqMax - DDSFreqMin) / 1000;
    ucg.print("SPAN=");
    ucg.print(span, 3);
    ucg.print("kHz");
    for (int i = 1; i <= 300; i++) {
      DDS_freq = DDSFreqMin + (DDSfPit * i); //33.33;
      DDS_freqset(DDS_freq);
      delay(1);
      ADOut = analogRead(AD8307);
      dBmconv();
      int y = 26 - (dBm * 3) + (offsetPix * 3);
      y = constrain(y, 25, 235);
      ucg.setColor(0, 255, 0);
      ucg.drawLine(xOld, yOld, (xOld + 1), y);
      xOld = xOld + 1;
      yOld = y;
      if (ADOut > ADOutMax) {
        ADOutMax = ADOut;
        ADOutMaxFreq = DDS_freq;
        ADMaxI = i;
      }
      if (ADOut < ADOutMin) {
        ADOutMin = ADOut;
        ADOutMinFreq = DDS_freq;
        ADMinI = i;
      }
    }
    if (!fCountFlag) {
      FRMf();
    }

    if (digitalRead(swRight) == LOW) {
      ucg.setColor(0, 0, 0);
      ucg.drawBox(10, 25, 310, 240);
      Casoldraw();
      fCountFlag = false;
      while (digitalRead(swRight) == LOW) {}
    }
  }
  interrupts();
  //  DDS_freqset(0);
  menu1();
  menu2();
  fCountFlag = false;
}

// ---------------- Frequency response sub1 ----------------------
void FRMf() {
  float admaxf = 0;
  float adminf = 0;

  if (mode == 3) {
    ucg.setColor(0, 255, 255);
    ucg.drawVLine((10 + ADMaxI), 25, 210);
    ucg.setColor(250, 120, 30);
    ucg.drawVLine((10 + ADMinI), 25, 210);
    if ((DDSFreqMax - DDSFreqMin) <= 300000) {
      int ADmaxj = 0;
      int ADminj = 0;
      ucg.setColor (255, 255, 255);
      ucg.setFont(ucg_font_7x14B_tr);
      ucg.setPrintPos(170, 37);
      ucg.print("Now Measuring");
      long DDS_freq = ADOutMaxFreq - 1000;
      if (DDS_freq <= DDSFreqMin) {
        DDS_freq = DDSFreqMin;
      }
      for (long j = 0; j <= 2000; j++) {
        DDS_freq = DDS_freq + j;
        if (DDS_freq > DDSFreqMax) {
          DDS_freq = DDSFreqMax;
        }
        DDS_freqset(DDS_freq);
        ADOut = analogRead(AD8307);
        if (ADOut > ADOutMax) {
          ADOutMax = ADOut;
          ADOutMaxFreq = DDS_freq;
        }
      }
      DDS_freq = ADOutMinFreq - 1000;
      if (DDS_freq < DDSFreqMin) {
        DDS_freq = DDSFreqMin;
      }
      for (long j = 0; j <= 2000; j++) {
        DDS_freq = DDS_freq + j;
        if (DDS_freq > DDSFreqMax) {
          DDS_freq = DDSFreqMax;
        }
        DDS_freqset(DDS_freq);
        ADOut = analogRead(AD8307);
        if (ADOut < ADOutMin) {
          ADOutMin = ADOut;
          ADOutMinFreq = DDS_freq;
        }
      }
      ucg.setColor (0, 0, 0);
      ucg.setFont(ucg_font_7x14B_tr);
      ucg.setPrintPos(170, 37);
      ucg.print("Now Measuring");
      ucg.setColor (250, 120, 30);
      ucg.setFont(ucg_font_7x14B_tr);
      ucg.setPrintPos(155, 37);
      ADOut = ADOutMax;
      dBmconv();
      ucg.setColor(0, 255, 255);
      ucg.print(dBm);
      ucg.print("dBm ");
      admaxf = (float)ADOutMaxFreq / 1000;
      ucg.print(admaxf, 3);
      ucg.print("kHz");
      ucg.setColor(250, 120, 30);
      ucg.setPrintPos(155, 52);
      ADOut = ADOutMin;
      dBmconv();
      ucg.print(dBm);
      ucg.print("dBm ");
      adminf = (float)ADOutMinFreq / 1000;
      ucg.print(adminf, 3);
      ucg.print("kHz");
      ucg.setColor(255, 255, 255);
      ucg.setPrintPos(155, 67);
      ucg.print("Delta f=");
      long deltf = fabs(ADOutMinFreq - ADOutMaxFreq);
      ucg.print(deltf);
      ucg.print("Hz");
      fCountFlag = true;
      ADOutMin = 1023;
      ADOutMax = 0;
    }
  }
  if (mode == 4) {
    ucg.setColor(250, 120, 30);
    ucg.drawVLine((10 + ADMinI), 25, 210);
    float dBmax, dBmin;
    ucg.setFont(ucg_font_7x14B_tr);
    ucg.setPrintPos(155, 37);
    ucg.setColor(0, 255, 255);
    ADOut = ADOutMax;
    dBmconv();
    ucg.print(dBm);
    dBmax = dBm;
    ucg.print("dBm  ");
    admaxf = (float)ADOutMaxFreq / 1000;
    ucg.print(admaxf, 3);
    ucg.print("kHz");
    ucg.setPrintPos(155, 52);
    ucg.setColor(250, 120, 30);
    ADOut = ADOutMin;
    dBmconv();
    ucg.print(dBm);
    dBmin = dBm;
    ucg.print("dBm  ");
    adminf = (float)ADOutMinFreq / 1000;
    ucg.print(adminf, 3);
    ucg.print("kHz");
    float rho = 0;
    rho = dBmax - dBmin;
    rho = rho / 20;
    rho = pow(10, rho);
    SWR = -1 * ((1 + rho) / (1 - rho));
    ucg.setColor(255, 255, 255);
    ucg.setPrintPos(155, 67);
    ucg.print("SWR = ");
    ucg.print(SWR);
    fCountFlag = true;
    ADOutMin = 1023;
    ADOutMax = 0;
  }
}

// --------------- Frequency Response sub2 -----------------------
void fraconfig() {

  noInterrupts();
  float maxfprint = 0;
  float minfprint = 0;
  ucg.clearScreen();
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(20, 25);
  configFlag = 1;
  if (mode == 3) {
    ucg.print("Frequency Analyzer Config");
    EEPROM.get(0x040, DDSFreqMin);
    EEPROM.get(0x050, DDSFreqMax);
    EEPROM.get(0x060, offsetPix);
    freqStepFlag = EEPROM.read(0x85);
  }
  if (mode == 4) {
    ucg.print("Antenna Analyzer Config");
    EEPROM.get(0x045, DDSFreqMin);
    EEPROM.get(0x055, DDSFreqMax);
    EEPROM.get(0x065, offsetPix);
    freqStepFlag = EEPROM.read(0x90);
  }
  ucg.setColor(255, 255, 255);
  ucg.setPrintPos(10, 75);
  ucg.print("1.Start ");
  ucg.setPrintPos(150, 75);
  minfprint = (float)DDSFreqMin / 1000;
  ucg.print(minfprint, 3);
  ucg.print("kHz");
  ucg.setPrintPos(10, 100);
  ucg.print("2.Stop ");
  ucg.setPrintPos(150, 100);
  maxfprint = (float)DDSFreqMax / 1000;
  ucg.print(maxfprint, 3);
  ucg.print("kHz");
  ucg.setPrintPos(10, 125);
  ucg.print("3.Ref set");
  ucg.setPrintPos(150, 125);
  ucg.print(offsetPix);
  while (digitalRead(swMenu) == HIGH) {
    if (digitalRead(swLeft) == LOW) {
      configFlag = configFlag - 1;
      while (digitalRead(swLeft) == LOW) {}
    }
    if (digitalRead(swRight) == LOW) {
      configFlag = configFlag + 1;
      while (digitalRead(swRight) == LOW) {}
    }
    configFlag = constrain(configFlag, 1, 3);
    switch (configFlag) {
      case 1:
        ucg.setPrintPos(10, 160);
        ucg.setColor(255, 255, 255);
        ucg.print("[");
        ucg.setColor(255, 0, 0);
        ucg.print(" 1");
        ucg.setColor(255, 255, 255);
        ucg.print(" 2 3 ]");
        break;
      case 2:
        ucg.setPrintPos(10, 160);
        ucg.setColor(255, 255, 255);
        ucg.print("[ 1");
        ucg.setColor(255, 0, 0);
        ucg.print(" 2");
        ucg.setColor (255, 255, 255);
        ucg.print(" 3 ]");
        break;
      case 3:
        ucg.setPrintPos(10, 160);
        ucg.setColor(255, 255, 255);
        ucg.print("[ 1 2");
        ucg.setColor(255, 0, 0);
        ucg.print(" 3");
        ucg.setColor(255, 255, 255);
        ucg.print(" ]");
        break;
    }
    if (digitalRead(swEnter) == LOW) {
      fraconfigin();
    }
  }
  ucg.clearScreen();
  interrupts();
}

// ----------------- Frequency Response sub3 ---------------------
void fraconfigin() {
  float minfprint = 0;
  float maxfprint = 0;
  interrupts();
  ucg.setColor(0, 240, 240);
  ucg.drawFrame(160, 170, 150, 35);
  switch (configFlag) {
    case 1:
      ucg.setPrintPos(140, 160);
      ucg.print("Start ");
      ucg.setPrintPos(180, 195);
      minfprint = (float)DDSFreqMin / 1000;
      ucg.print(minfprint, 3);
      step();
      break;
    case 2:
      ucg.setPrintPos(140, 160);
      ucg.print("Stop ");
      ucg.setPrintPos(180, 195);
      maxfprint = (float)DDSFreqMax / 1000;
      ucg.print(maxfprint, 3);
      step();
      break;
    case 3:
      ucg.setPrintPos(140, 160);
      ucg.print("Ref set ");
      ucg.setPrintPos(180, 195);
      ucg.print(offsetPix);
      freqStepFlag = 1;
      step();
      break;
  }
  while (digitalRead(swEnter) == HIGH) {
    if (digitalRead(swLeft) == LOW) {
      freqStepFlag = freqStepFlag - 1;
      if (freqStepFlag < 1) {
        freqStepFlag = 1;
      }
      while (digitalRead(swLeft) == LOW) {}
      step();
      ucg.setColor(0, 240, 240);
    }
    if (digitalRead(swRight) == LOW) {
      freqStepFlag = freqStepFlag + 1;
      if (freqStepFlag > 8) {
        freqStepFlag = 8;
      }
      while (digitalRead(swRight) == LOW) {}
      step();
      ucg.setColor(0, 240, 240);
    }
    if (configFlag != 3) {
      if (mode == 3) {
        EEPROM.write(0x085, freqStepFlag);
      }
      if (mode == 4) {
        EEPROM.write(0x090, freqStepFlag);
      }
    }
  }
  if (mode == 3) {
    freqStepFlag = EEPROM.read(0x085);
  }
  if (mode == 4) {
    freqStepFlag = EEPROM.read(0x090);
  }
  if (DDSFreqMax < DDSFreqMin) {
    DDSFreqMax = DDSFreqMin;
  }
  if (mode == 3) {
    EEPROM.put(0x040, DDSFreqMin);
    EEPROM.put(0x050, DDSFreqMax);
    EEPROM.put(0x060, offsetPix);
  }
  if (mode == 4) {
    EEPROM.put(0x045, DDSFreqMin);
    EEPROM.put(0x055, DDSFreqMax);
    EEPROM.put(0x065, offsetPix);
  }
  ucg.setColor(0, 0, 0);
  ucg.drawBox(135, 140, 185, 100);
  ucg.drawBox(150, 50, 170, 100);
  ucg.setColor(255, 255, 255);
  ucg.setPrintPos(150, 75);
  minfprint = (float)DDSFreqMin / 1000;
  ucg.print(minfprint, 3);
  ucg.print("kHz");
  ucg.setPrintPos(150, 100);
  maxfprint = (float)DDSFreqMax / 1000;
  ucg.print(maxfprint, 3);
  ucg.print("kHz");
  ucg.setPrintPos(150, 125);
  ucg.print(offsetPix);
  ucg.print("dBm");
  noInterrupts();
}

// --------------------- DDS Initialize --------------------------

void DDS_init() {
  digitalWrite(DDS_RST, HIGH);
  delay(1);
  digitalWrite(DDS_RST, LOW);
  digitalWrite(DDS_W_CLK, HIGH);
  digitalWrite(DDS_W_CLK, LOW);
  digitalWrite(DDS_FQ_UD, HIGH);
  digitalWrite(DDS_FQ_UD, LOW);
}

// ------------------ DDS Frequency set --------------------------

void DDS_freqset(double frequency) {
  unsigned long wrk = frequency * TWO_E / DDS_CLK;
  Serial.println(wrk);
  digitalWrite(DDS_FQ_UD, HIGH);
  digitalWrite(DDS_FQ_UD, LOW);
  shiftOut(DDS_DATA , DDS_W_CLK, LSBFIRST, wrk);
  shiftOut(DDS_DATA , DDS_W_CLK, LSBFIRST, (wrk >> 8));
  shiftOut(DDS_DATA , DDS_W_CLK, LSBFIRST, (wrk >> 16));
  shiftOut(DDS_DATA , DDS_W_CLK, LSBFIRST, (wrk >> 24));
  shiftOut(DDS_DATA , DDS_W_CLK, LSBFIRST, DDS_CMD);
  digitalWrite(DDS_FQ_UD, HIGH);
  digitalWrite(DDS_FQ_UD, LOW);
}

// -------------------- FRA Frame draw ---------------------------
void Casoldraw() {
  ucg.setColor(100, 100, 100);
  for (int i = 0; i <= 10; i++) {
    ucg.drawVLine(10 + (i * 30), 25, 210);
  }
  for (int i = 0; i <= 7; i++) {
    ucg.drawHLine(10, (235 - (i * 30)), 300);
  }
}

// --------------------- dB Converter ----------------------------
void dBmconv() {
  dBm = (ADOut * AD_a) + AD_b; //    dBm=(ADOut*0.09)-80.91;
}

// ------------------- AD8307 Ajustment --------------------------
void ADcheck() {
  mode = 6;
  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(10, 30);
  ucg.print("AD8307 Adjustment");
  float x1, x2;
  ucg.setFont(ucg_font_7x14B_tr);
  ucg.setPrintPos(20, 60);
  ucg.print("OLD AD_a =");
  ucg.print(AD_a);
  ucg.print("  OLD Ad_b =");
  ucg.print(AD_b);
  ucg.setPrintPos(20, 80);
  ucg.print("Input 0dBm");
  ucg.setPrintPos(20, 100);
  ucg.print("10seconds later Push [ENTER]");
  while (digitalRead(swEnter) == HIGH) {}
  x1 = analogRead(AD8307);
  ucg.setPrintPos(20, 120);
  ucg.print("AD8307Read X1=");
  ucg.print(x1);
  ucg.setPrintPos(20, 140);
  ucg.print("Input -50dBm");
  ucg.setPrintPos(20, 160);
  ucg.print("10seconds after Push {[ENTER]");
  while (digitalRead(swEnter) == HIGH) {}
  x2 = analogRead(AD8307);
  ucg.setPrintPos(20, 180);
  ucg.print("AD8307Read X2=");
  ucg.print(x2);
  AD_a = 50 / (x1 - x2);
  AD_b = 0 - (AD_a * x1);
  ucg.setPrintPos(20, 200);
  ucg.print("AD_a =");
  ucg.print(AD_a);
  ucg.print("  Ad_b =");
  ucg.print(AD_b);
  EEPROM.put(0x100, AD_a);
  EEPROM.put(0x120, AD_b);
  ucg.setPrintPos(20, 220);
  ucg.print("Push [MENU] back to TOP");
  while (digitalRead(swMenu) == HIGH) {}
  menu1();
  menu2();
}

// ------------------ DDS CLOCK Ajustment ------------------------
void DDSajust() {
  mode = 5;
  freqStep = 3;
  EEPROM.get(0x010, DDS_CLK);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(0, 240, 240);
  ucg.setPrintPos(180, 195);
  ucg.print(DDS_CLK);
  DDS_freqset(10000000);
  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setPrintPos(10, 30);
  ucg.print("DDS CLOCK Adjustment");
  ucg.setFont(ucg_font_7x14B_tr);
  ucg.setPrintPos(10, 70);
  ucg.print("OUTPUT Frequency of DDS is 10.000000MHz");
  ucg.setPrintPos(10, 85);
  ucg.print("Connect to frequency Counter");
  ucg.setPrintPos(10, 100);
  ucg.print("Adjust a frequency of DDS");
  ucg.setPrintPos(10, 115);
  ucg.print("      as a counter will be 10.000000MHz");
  ucg.setColor(0, 240, 240);
  ucg.drawFrame(160, 170, 150, 35);
  freqStepFlag = 3;
  step();
  while (digitalRead(swMenu) == HIGH) {
    if (digitalRead(swLeft) == LOW) {
      freqStepFlag = freqStepFlag - 1;
      if (freqStepFlag < 1) {
        freqStepFlag = 1;
      }
      step();
      ucg.setColor(0, 240, 240);
    }
    while (digitalRead(swLeft) == LOW) {}
    if (digitalRead(swRight) == LOW) {
      freqStepFlag = freqStepFlag + 1;
      if (freqStepFlag > 8) {
        freqStepFlag = 8;
      }
      step();
      ucg.setColor(0, 240, 240);
    }
    while (digitalRead(swRight) == LOW) {}
  }
  while (digitalRead(swMenu) == LOW) {};
  menu1();
  menu2();
}

void eepromInitWrite() {
  EEPROM.write(0x001, 104);
  EEPROM.put(0x010, 125000000);
  EEPROM.put(0x100, 0.09);
  EEPROM.put(0x120, -86.06);
  EEPROM.put(0x040, 0);
  EEPROM.put(0x050, 55000000);
  EEPROM.put(0x060, 0);
  EEPROM.put(0x045, 0);
  EEPROM.put(0x055, 55000000);
  EEPROM.put(0x065, 0);
  EEPROM.put(0x070, 10000000);
  EEPROM.write(0x080, 7);
  EEPROM.write(0x085, 7);
  EEPROM.write(0x090, 7);
}
