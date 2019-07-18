
//Time & Date functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

#define sensorTime 50

#define alpha_I 0.98

// kalman variables MOT
float varMotTemp = 702;  // variance determined using excel and reading samples of raw sensor data
float varProcess = .01;
float Pc = 0.0;
float G = 0.0;
float P = 8.0;
float Xp = 0.0;
float Zp = 0.0;
float tempMOTK = 80.0;

// kalman variables ESC
float varESCTemp = 20;  // variance determined using excel and reading samples of raw sensor data
float varESCProcess = .01;
float PcE = 0.0;
float GE = 0.0;
float PE = 8.0;
float XpE = 0.0;
float ZpE = 0.0;
float tempESCK = 80.0;

int temp0;
int temp1;
int temp2;
int I_Raw;

float  tempAmb;
float  tempESC;
float  tempMOT;
float I_Conv;
float I_Filt;

int count = 0;

unsigned long oldSensorTime = 0;
unsigned long oldSerialTime = 0;

int serialTXTime = 1000;

void setup(void)
{
  Serial.begin(57600);
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  Serial.println("Date & Time, Ambient, ESC, ESC Kal, Motor, Motor Kal, Current, Current Filter");
}

void loop(void)
{
  if (millis() - oldSensorTime >= sensorTime){

    temp0 = analogRead(A0);
    temp1 = analogRead(A1);
    temp2 = analogRead(A2);

    I_Raw = analogRead(A11);

    //  float  tempAmb=map(temp0,395,493,51,168);
    tempAmb = temp0 * 1.1915 - 419.27;
    tempESC = temp2 * 1.1915 - 419.27;
    tempMOT = temp1 * 1.1915 - 419.27;

    //Current Conversion
    I_Conv = (I_Raw/28.5);

    // kalman process
    Pc = P + varProcess;
    G = Pc / (Pc + varMotTemp);  // kalman gain
    P = (1 - G) * Pc;
    Xp = tempMOTK;
    Zp = Xp;
    tempMOTK = G * (tempMOT - Zp) + Xp; // the kalman estimate of the sensor voltage

    // kalman process ESC
    PcE = PE + varESCProcess;
    GE = PcE / (PcE + varESCTemp);  // kalman gain
    PE = (1 - GE) * PcE;
    XpE = tempESCK;
    ZpE = XpE;
    tempESCK = GE * (tempESC - ZpE) + XpE; // the kalman estimate of the sensor voltage

    //Complimentary Filter for Current
    //alpha_I is the filter constant
    I_Filt = (I_Filt * alpha_I) + (I_Conv * (1 - alpha_I));

    oldSensorTime = millis();
  }

  //Conditional statement to adjust serial transmit time based on temperatures and currents
  if (tempAmb >= 130 || tempESCK >= 150 || tempMOTK >= 300 || I_Filt >= 25){
    serialTXTime  = 1000;
  } else {
    serialTXTime = 30000;
  }
  
  if (millis() - oldSerialTime >= serialTXTime){
      DateTime now = rtc.now();
    
      //Print Time
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print('/');
      Serial.print(now.year(), DEC);
      Serial.print(' ');
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      int zerom = now.minute();
      if (zerom < 10) Serial.print("0");
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      int zeros = now.second();
      if (zeros < 10) Serial.print("0");
      Serial.print(now.second(), DEC);
      Serial.print(", ");

      //Print Data
      Serial.print(tempAmb);
      Serial.print(", ");
      Serial.print(tempESC);
      Serial.print(", ");
      Serial.print(tempESCK);
      Serial.print(", ");
      Serial.print(tempMOT);
      Serial.print(", ");
      Serial.print(tempMOTK);
      
      Serial.print(", ");
      Serial.print(I_Conv);
      Serial.print(", ");
      Serial.println(I_Filt);

      oldSerialTime = millis();
  }
}
