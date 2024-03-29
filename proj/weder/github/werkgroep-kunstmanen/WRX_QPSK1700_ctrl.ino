/*******************************************************************
 * RCSId: $Id$
 *
 * WRX170_QPSK tuner setting and display drive 
 * Project: HRPT/QPSK receiver
 * Author: Ben Schellekens, Rob Alblas
 *    wekrgroep Kunstmanen (working group Satellites)
 *    www.kunstmanen.net
 *
 * History: 
 * $Log$
 *
 * Release 3.3: - Corrected UV916, high-band setting
 *              - added some comments, some re-arrangements
 *
 * Release 3.2: - Corrected VCO/RF level update
 * 
 * Release 3.1: - Added standard LCD-lib support (3-1-2023)
 *              - Added UV916 support
 *
 * Release 3.0: - Added UV916 support (2-8-2022)
 *
 * Release 2.1: - Added setting via UART (alternative manual via rotary switch)
 *
 * Release 1.35: - Only support of UV1316;
 *               - No APT-satellites
 *               24-11-2017
 *
 * Release 0.34: - Support of UV1316
 *
 *******************************************************************/
/*******************************************************************
 * Copyright (C) 2018 Ben Schellekens, Rob Alblas 
 *   wekrgroep Kunstmanen (working group Satellites)
 *   www.kunstmanen.net
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
 * 02111-1307, USA.
 *******************************************************************/
/*******************************************************************
 * Description:
 * Tune frequency for the QPSK-receiver
 *
 * With a 12 position rotary switch and a resistor laddder an
 * AD-converter will select the frequency for the tuner.
 * Information is displayed on the I2C LCD-display.
 * From version 2.0 on tunersetting can also be done via the UART interface:
 * baudrate: 38400 
 * byte  1: sync
 * bytes 2,3: 16-bits, frequency*10; byte 2=MSByte
 * byte  4: modulation; 'P'=PSK, 'Q'=QPSK
 * byte  5-15: string (satellite name)
 * byte 16 : CRC
 *
 *  * Configuration//
 *    MCU : Arduino / Duemilanove / ATMega328
 *    Environment : Arduino 1.8.2
 *  * Connections
 *    - Tunerswitch AN1
 *    - RFlevel AN0
 *    - VCO / tuning voltage AN3
 *    - Modulation type HRPT / QPSK = 1 / 0 DIGITAL 3
 *******************************************************************/
// next zip-file for NewliquidCrystal doesn't exist anymore...
//https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip

#define VERSION "Version: 3.3"

/*******************************************************************
 *******************************************************************
 * START SETTINGS
 *******************************************************************
 *******************************************************************/

/*******************************************************************
 * LCD display settings
 *******************************************************************/
// Choose standard LiquidCrystal or NewliquidCrystal
#define USE_NEWLCDLIB false

// address display, could be 0x3f or 0x27
#define DISPL_ADDR 0x3f
//#define DISPL_ADDR 0x27
#define DISPLAY_NRCHARS 16
#define DISPLAY_NRLINES 2

/*******************************************************************
 * Downconverter settings
 * Choice done by input pin TYPEDOWNC.
 * Default: pin=1, downconverter A
 *******************************************************************/
#define TYPEDC_A 1                      // default downconverter, pin P_TYPEDOWNC=1
#define LO_DWNCONV_A 10000              // local oscillator, in MHz x 10

#define TYPEDC_B 0                      // alternate downconverter, pin P_TYPEDOWNC=0
#define LO_DWNCONV_B 15570              // local oscillator, in MHz x 10

#define FREQIF       365L               // in MHz x 10

/*******************************************************************
 * Tuner parameters, note ASP pin!
 * Choice done by input pin TYPETUNER.
 * Default: pin=1, tuner UV1316
 *******************************************************************/
#define I2CADDR_ASP0       0x60         // tunerpin ASP=GND,  = 0xC0>>1
#define I2CADDR_ASP2       0x61         // tunerpin ASP=open, = 0xC2>>1

//  Tuner parameters for UV1316
#define TYPE_UV1316        1            // default tuner, pin P_TYPETUNER=1
//#define FREQSTEP_UV1316    62.500     // 62.5 kHz
#define FREQSTEP_UV1316    50           // 50.0 kHz (UV1316 only)
#define BAND_UV1316_A      0x04         // high band
#define BAND_UV1316_B      0x01         // low band
#define UV1316_I2CADDR     I2CADDR_ASP0

#if FREQSTEP_UV1316==50
  #define UV1316_CTRLBYTE 0x80
#else
  #define UV1316_CTRLBYTE 0x86
#endif

// Tuner parameters for UV916
#define TYPE_UV916         0            // alternate tuner, pin P_TYPETUNER=0
#define FREQSTEP_UV916     62.500       // 62.5 kHz
#define BAND_UV916_A       0x30         // high band
#define BAND_UV916_B       0x60         // low band
#define UV916_I2CADDR      I2CADDR_ASP2
#define UV916_CTRLBYTE     0x8E

/*******************************************************************
 *******************************************************************
 * END SETTINGS
 *******************************************************************
 *******************************************************************/

// enable UART TX for debugging settings
#define MONITOR_TX 0

/*******************************************************************
 *******************************************************************
 * Start Pinning
 *******************************************************************
 *******************************************************************/
// Pinning code ATmega328: nr=Arduino numbering for Uno
// Digital:
// nr	naam	pin 
// 0	PD0	2	RXD
// 1	PD1	3	TXD
// 2	PD2	4
// 3	PD3	5
// 4	PD4	6
// 5	PD5	11
// 6	PD6	12
// 7	PD7	13
// 8	PB0	14
// 9	PB1	15
// 10	PB2	16
// 11	PB3	17
// 12	PB4	18
// 13	PB5	19
// 14	PC0	23
// 15	PC1	24
// 16	PC2	25
// 17	PC3	26

// Alternative numbering Analog (analogRead() only!): (Not for pinMode()!)
// nr	naam	pin 
// 0	PC0	23
// 1	PC1	24
// 2	PC2	25
// 3	PC3	26

// Define analog inputs
#define P_RFlevel      0    // PC0
#define P_TUNINGSWITCH 1    // PC1
#define P_VCO          3    // PC3

// Define digital in/outputs
#define P_QPSKpuls     5    // PD5
#define P_MODtype      6    // PD6
#define P_TYPEDOWNC   11    // PB3
#define P_TYPETUNER   12    // PB4

/*******************************************************************
 *******************************************************************
 * End Pinning
 *******************************************************************
 *******************************************************************/
// No changes beyond this point needed!

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if USE_NEWLCDLIB
// Use NewliquidCrystal
//                    addr      , en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(DISPL_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //PCF8574A
#else
// Use LiquidCrystal
//                    addr      
LiquidCrystal_I2C lcd(DISPL_ADDR, DISPLAY_NRCHARS,DISPLAY_NRLINES);
#endif

// for VCO/RF read-out
const int nrSamples = 10;
int RFlevelsamples[nrSamples];
int VCOsamples[nrSamples];

// for tuner-settings
unsigned int tuneradress, controlbyte,band;
unsigned int TuneFreq, DownConvFreq;

// for LCD display
String freqmessage, tuner;
char freqmessage_auto[17];

// for external frequency settings via RX input ATmega
boolean mode_psk_auto;
unsigned int TuneFreq_auto=0;

int TypeDC;
int TypeTuner;

void setup()
{
  int  f_int,f_frac;
  Wire.begin();
  // Set UART for external tuner setting
  Serial.begin(38400,SERIAL_8N1);

  pinMode(P_TYPEDOWNC, INPUT_PULLUP);
  pinMode(P_TYPETUNER, INPUT_PULLUP);
  pinMode(P_MODtype, OUTPUT);
  pinMode(P_QPSKpuls, OUTPUT);

#if !USE_NEWLCDLIB
  lcd.init();
#endif
  lcd.begin(DISPLAY_NRCHARS,DISPLAY_NRLINES);
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
           //1234567890123456   
  lcd.print("QPSK-ontvanger");
  lcd.setCursor(0, 1);
  lcd.print(VERSION);

  delay(2000); // wait to show version number

  // Get downconverter and tuner choice from inpu pins
  TypeDC    = digitalRead(P_TYPEDOWNC);   // 1: LO=TYPEDC_A, 0: LO=TYPEDC_B
  TypeTuner = digitalRead(P_TYPETUNER);   // 1: UV1316 , 0: UV916

  for (int j = 0; j < nrSamples; j++)
  {
    RFlevelsamples[j] = 0;
  }

  // define codes to send to tuner
  if (TypeTuner == TYPE_UV1316)
  {    
    tuneradress = UV1316_I2CADDR;       // I2C address 
    controlbyte = UV1316_CTRLBYTE;      // 50 or 62.5 freq. step
    if (TypeDC == TYPEDC_A)
      band=BAND_UV1316_A;
    else
      band=BAND_UV1316_B;
  }
  else                                  // TYPE_UV916
  {
    tuneradress = UV916_I2CADDR;        // I2C address 
    controlbyte = UV916_CTRLBYTE;       // always 62.5 freq. step
    if (TypeDC == TYPEDC_A)
      band=BAND_UV916_A;
    else
      band=BAND_UV916_B;
  }

  // define downconverter LO
  if (TypeDC == TYPEDC_A)
  {
    DownConvFreq = LO_DWNCONV_A;        // Frequency * 10
    #if MONITOR_TX
      Serial.println("Downconverter LO: 1557");
    #endif
  }
  else
  {
    DownConvFreq = LO_DWNCONV_B;        // Frequency * 10
    #if MONITOR_TX
      Serial.println("Downconverter LO: 1000");
    #endif
  }

  // show settings shortly
  lcd.setCursor(0, 0); // row 1
  if (TypeTuner==TYPE_UV1316)
  {
             //1234567890123456   
    lcd.print("UV1316          ");
  }
  else  // TYPE_UV916
  {
             //1234567890123456   
    lcd.print("UV916           ");
  }

  f_int=DownConvFreq/10;
  f_frac=DownConvFreq-(f_int*10);
  lcd.setCursor(0, 1); // row 2
  lcd.print(String("DC LO: ")+String(f_int)+String(".")+String(f_frac));

  delay(3000); // time to show above info

  // show VCO, RF levels at row 2
  lcd.setCursor(0, 1);
  lcd.print("VCO      RF     ");
}

void resetpuls()
{
  digitalWrite(P_QPSKpuls, HIGH);
  delay(200);
  digitalWrite(P_QPSKpuls, LOW);
}

void send2uv()
{
  unsigned int lsb, msb;
  unsigned int UVFreq;
  unsigned long nrSteps;
  unsigned int stepsize_10MHz;
  
  if (TypeTuner==TYPE_UV1316)
    stepsize_10MHz=1000/FREQSTEP_UV1316; // UV1316
  else
    stepsize_10MHz=1000/FREQSTEP_UV916; // UV916

  if (TuneFreq > 2000)                  // Downconverter is used
    UVFreq = TuneFreq - DownConvFreq;
  else                                  // no downconverter
    UVFreq = TuneFreq;

  nrSteps = (UVFreq + FREQIF) * stepsize_10MHz;
  nrSteps = nrSteps / 10;               // Because all frequencies x 10

  msb = nrSteps / 256;
  lsb = nrSteps % 256;

  Wire.beginTransmission((byte)tuneradress); // Adres 
  Wire.write(msb);  // Delermsb
  Wire.write(lsb);  // Delerlsb
  Wire.write(controlbyte); // Chargepump
  Wire.write(band); // Band
  Wire.endTransmission();

  #if MONITOR_TX
    Serial.print("Tunerfrequency: ");
    Serial.println(TuneFreq);
    Serial.print("nrSteps: ");
    Serial.println(nrSteps);
    Serial.print("msb: ");
    Serial.println(msb, HEX);
    Serial.print("lsb: ");
    Serial.println(lsb, HEX);
    Serial.print("address: ");
    Serial.println(tuneradress, HEX);
    Serial.print("controlbyte: ");
    Serial.println(controlbyte, HEX);
    Serial.print("band: ");
    Serial.println(band, HEX);
    
  #endif
  delay(100);

  // Show frequency and satellite on display
  lcd.setCursor(0, 0);
  lcd.print(freqmessage);
}

#define STARTBYTE 0x53
#define NRBYTES 16
unsigned char get_data(unsigned char *data)
{
  static int bytecnt=NRBYTES;
  unsigned char idat;
  char ready=0;
  if (Serial.available())
  {
    idat=Serial.read();
    if ((idat==STARTBYTE) && (bytecnt>=NRBYTES)) bytecnt=0;
    if (bytecnt<NRBYTES)
    {
      data[bytecnt]=idat;
      bytecnt++;
      if (bytecnt>=NRBYTES) ready=1;
    }     
  }
  return ready;
}

void handle_switch()
{
  static int nr_crcerr;
  static int TuneSwitchOld;
  int TuneSwitchNew;
  unsigned char sdata[20];
  boolean mode_psk;
  // From the AD-converter we receive 12 valules:
  // 0, 91, 185, 278, 372, 464, 557, 651, 744, 837, 930, 1023.
  TuneSwitchNew = (analogRead(P_TUNINGSWITCH) + 46) / 93 + 1;
  // Check if there is a change in the position of the tuning switch.
  if (TuneSwitchNew != TuneSwitchOld)
  {
    mode_psk=LOW;
    switch (TuneSwitchNew)
    {
      case 5:
        if (!TuneFreq_auto)               // set if no UART data received yet
        {
          mode_psk_auto=HIGH;
          strcpy(freqmessage_auto,"1698.0 MHz BP   "); // 2839 steps
          TuneFreq_auto = 16980;
        }
        mode_psk=mode_psk_auto;
        freqmessage=freqmessage_auto;
        TuneFreq=TuneFreq_auto;
      break;
      case 6:                             // TEST; no downconverter asumed
        freqmessage = "150.00 MHz TEST "; // 2983 steps
        TuneFreq = 1500;
      break;
      case 7:
        mode_psk=HIGH;
        freqmessage = "1698.0 MHz HRPT "; // 2839 steps
        TuneFreq = 16980;
      break;

      case 8:
        mode_psk=HIGH;
        freqmessage = "1700.0 MHz MHRPT"; // 2871 steps
        TuneFreq = 17000;
      break;

      case 9:
        freqmessage = "1701.3 MHz QPSK "; // 2871 steps
        TuneFreq = 17013;
      break;

      case 10:
        freqmessage = "1702.5 MHz HRPT "; // 2911 steps
        TuneFreq = 17025;
      break;

      case 11:
        freqmessage = "1704.5 MHz QPSK "; // 2943 steps
        TuneFreq = 17045;
      break;

      case 12:
        mode_psk=HIGH;
        freqmessage = "1707.0 MHz HRPT "; // 2983 steps
        TuneFreq = 17070;
      break;

      default: // 1...4 
        mode_psk=HIGH;
        freqmessage = "1698.0 MHz HRPT "; // 2839 steps
        TuneFreq = 16980;
       break;
    }
    digitalWrite(P_MODtype, mode_psk);

    send2uv();
    resetpuls();
    TuneSwitchOld = TuneSwitchNew;
  }

  if (get_data(sdata))      // always search for data...
  {
    if (TuneSwitchNew==5)   // but only used in auto-mode
    {
      unsigned char crc=0;
      unsigned char *xsdata;
      int dTuneFreq;

      // Check: calculate CRC
      for (int i=0; i<NRBYTES; i++) crc+=sdata[i];

      if (crc==0)
      { // OK
        TuneFreq_auto=sdata[1]*256+sdata[2];
        dTuneFreq=TuneFreq_auto-(TuneFreq_auto/10)*10;
        if (sdata[3] == 'P') mode_psk_auto=HIGH; else mode_psk_auto=LOW;
        snprintf(freqmessage_auto,17,"%4d.%d MHz %s %2d",TuneFreq_auto/10,dTuneFreq,(sdata[3]=='P'? "BP" : "QP"),nr_crcerr);

        freqmessage=freqmessage_auto;
        TuneFreq=TuneFreq_auto;
        mode_psk=mode_psk_auto;

        digitalWrite(P_MODtype, mode_psk); 

        nr_crcerr=0;
        send2uv();
        resetpuls();
        sdata[15]=0;
        xsdata=(sdata+4); // name
        lcd.setCursor(0, 1);
        lcd.print((char *)xsdata);
        delay(500);
      }
      else
      { // Error; ignore!
        nr_crcerr++;
      }
    }
  }
}

// VCO xxxx RF yyyy 
void handle_rflevel()
{
  int RFlevelTot;
  static int i;
  static int RFlevelAvgOld, RFlevelAvgNew;
  static char str[17];
  static String strRFlevel;
  static String strVCO;

  int VCOTot;
  static int VCOAvgOld, VCOAvgNew;

  // Fill arrays with samples to calculate averages
  RFlevelsamples[i] = analogRead(P_RFlevel);
  VCOsamples[i] = analogRead(P_VCO);
  delay(20);
  i++;
  if (i >= nrSamples)
  {
    i = 0;
    
    // Calculate averages
    RFlevelTot = 0;
    VCOTot = 0;
    
    for (int j = 0; j < nrSamples; j++)
    {
      RFlevelTot = RFlevelTot + RFlevelsamples[j];
      VCOTot = VCOTot + VCOsamples[j];
    }
    
    RFlevelAvgNew = RFlevelTot / nrSamples;
    VCOAvgNew = VCOTot / nrSamples;
  }
  // If there is a change in the RFlevel-value print this value on row 2 of the LCD-display.
  // due to the small size of the Arduino calculation with longs is not possible.
  if (RFlevelAvgNew != RFlevelAvgOld)
  {
    strRFlevel = String(RFlevelAvgNew / 10);
    strRFlevel.concat(".");
    strRFlevel.concat(RFlevelAvgNew % 10);

    RFlevelAvgOld = RFlevelAvgNew;
  }

  // Calculate VCO-level with two decimals
  if (VCOAvgNew != VCOAvgOld)
  {
    long centivolts = ((VCOAvgNew * 500L ) / 1023);
    int dispvalue;
    dispvalue = centivolts / 100;
    strVCO = String(centivolts / 100);
    strVCO.concat(".");
    sprintf(str,"%02d",centivolts % 100);
    strVCO.concat(str);

    VCOAvgOld = VCOAvgNew;
  }
  snprintf(str,17,"VCO %-4s RF %-4s",strVCO.c_str(),strRFlevel.c_str());
  lcd.setCursor(0, 1);
  lcd.print(str);
}


void loop()
{
  handle_switch();
  if (!Serial.available())   // only handle vco/rf levels if no serial data available
    handle_rflevel();
}
