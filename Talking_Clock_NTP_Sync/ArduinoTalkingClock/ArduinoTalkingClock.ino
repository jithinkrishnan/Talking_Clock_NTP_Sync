/************************************************************************
*   Arduino Talking Clock With NTP Time Sync
*   
*   File:   ArduinoTalkingClock.ino
*   Author:  Jithin Krishnan.K
*       Rev. 1.1 : 13/10/2020 :  07:13 PM
* 
* This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* Email: jithinkrishnan.k@gmail.com
*   
************************************************************************/
#include <Wire.h>
#include "DS3231.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include "RotaryEncoder.h"
#include "SD.h"
#include "TMRpcm.h"
#include "SPI.h"

#define SD_ChipSelectPin 10
#define buttonPin 7
#define AmpON 2
#define SPK_OUT 9
#define WIFI_BOARD 5
#define LED_RED 8 
#define VOL_MAX 5
#define VOL_MIN 2            
#define doggieTickle() resetTime = millis();
#define TIMEOUTPERIOD 15000
#define HANDLE_TAGS

DS3231 clk;
TMRpcm tmrpcm;
RTCDateTime dt;
RotaryEncoder encoder(A2, A3);

char i, hr;
char toggle = 0;
int vol;
unsigned long resetTime = 0;
volatile byte state = LOW;
void(* resetFunc) (void) = 0;


void setup()
{
   
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);
    tmrpcm.speakerPin = SPK_OUT;
    pinMode(buttonPin, INPUT);              
    pinMode(AmpON, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(WIFI_BOARD, OUTPUT);  

    /* NTP Time Sync */
    digitalWrite(WIFI_BOARD, HIGH);
    digitalWrite(LED_RED, HIGH);
    delay(60000);
    digitalWrite(WIFI_BOARD, LOW);
    digitalWrite(LED_RED, LOW); 
               
    //Serial.begin(115200);
    clk.begin();
    //WatchdogSetup();
    // Manual (YYYY, MM, DD, HH, II, SS                  
    //clock.setDateTime(2019, 1, 3, 8, 54, 54);                      
    if(!SD.begin(SD_ChipSelectPin)) {
      //Serial.println("SD fail");
      return;
    }
    vol = EEPROM.read (0); //int
    tmrpcm.setVolume(vol);
    tmrpcm.quality(1);           
}


void loop()
  {
   dt = clk.getDateTime();

   if (dt.hour == 12 && dt.minute == 15 && !dt.second) { //  Time sync with NTP server @ 12:15:00 PM
       digitalWrite(WIFI_BOARD, HIGH);
       digitalWrite(LED_RED, HIGH);
       delay(60000);
       digitalWrite(WIFI_BOARD, LOW);
       digitalWrite(LED_RED, LOW);
   } 
    
   delay(10);
   if(dt.hour > 12)
      hr = dt.hour - 12;
   else
      hr = dt.hour;
      if (hr == 0)
          hr = 12;
   SetVolume();   
   if (!dt.minute && !dt.second) {
        AmpPowerUp();
        tmrpcm.play("ring.wav");
        while(tmrpcm.isPlaying());
       
        for(i = 0; i < hr; i++)
          {
            tmrpcm.play("ding.wav");
            while(tmrpcm.isPlaying());
          }
          AmpPowerDown();
      }

      if (dt.minute == 30 && !dt.second) {
          AmpPowerUp();
          tmrpcm.play("ring.wav");
          while(tmrpcm.isPlaying());     
          tmrpcm.play("ding.wav");
          while(tmrpcm.isPlaying());
          AmpPowerDown(); 
        }
   if (!digitalRead(buttonPin)) {
      AmpPowerUp();
      TexttoSpeech(dt.hour);
      while(tmrpcm.isPlaying());
      tmrpcm.play("h.wav");
      while(tmrpcm.isPlaying());
      TexttoSpeech(dt.minute);
      while(tmrpcm.isPlaying());
      tmrpcm.play("m.wav");
      while(tmrpcm.isPlaying());
      TexttoSpeech(dt.second);
      while(tmrpcm.isPlaying());
      tmrpcm.play("s.wav");
      while(tmrpcm.isPlaying());
      DayOfWeek(dt.dayOfWeek);
      while(tmrpcm.isPlaying());
      TexttoSpeech(dt.day);
      while(tmrpcm.isPlaying());
      SpeechMonth(dt.month);
      while(tmrpcm.isPlaying());
      tmrpcm.play("2K.wav");
      while(tmrpcm.isPlaying());
      TexttoSpeech(dt.year%2000);
      while(tmrpcm.isPlaying());
      AmpPowerDown();
   }  else {
      //clock.forceConversion();
      //Serial.print("Temperature: ");
      //Serial.println(clock.readTemperature());  
   }
}

void DayOfWeek(char d) 
{
  switch(d) {
    case 1:
      tmrpcm.play("mon.wav");
      while(tmrpcm.isPlaying());
      break;
    case 2:
      tmrpcm.play("tue.wav");
      while(tmrpcm.isPlaying());
      break;
    case 3:
      tmrpcm.play("wed.wav");
      while(tmrpcm.isPlaying());
      break;
    case 4:
      tmrpcm.play("thu.wav");
      while(tmrpcm.isPlaying());
      break;
    case 5:
      tmrpcm.play("fri.wav");
      while(tmrpcm.isPlaying());
      break;
    case 6:
      tmrpcm.play("sat.wav");
      while(tmrpcm.isPlaying());
      break;
    case 7:
      tmrpcm.play("sun.wav");
      while(tmrpcm.isPlaying());
      break;
  }
}
void SpeechMonth(char digit)
{
  switch(digit) {
    case 1:
      tmrpcm.play("jan.wav");
      while(tmrpcm.isPlaying());
      break;
    case 2:
      tmrpcm.play("feb.wav");
      while(tmrpcm.isPlaying());
      break;
    case 3:
      tmrpcm.play("mar.wav");
      while(tmrpcm.isPlaying());
      break;
    case 4:
      tmrpcm.play("apr.wav");
      while(tmrpcm.isPlaying());
      break;
    case 5:
      tmrpcm.play("may.wav");
      while(tmrpcm.isPlaying());
      break;
    case 6:
      tmrpcm.play("jun.wav");
      while(tmrpcm.isPlaying());
      break;
    case 7:
      tmrpcm.play("jul.wav");
      while(tmrpcm.isPlaying());
      break;
    case 8:
      tmrpcm.play("aug.wav");
      while(tmrpcm.isPlaying());
      break;
    case 9:
      tmrpcm.play("sep.wav");
      while(tmrpcm.isPlaying());
      break;
    case 10:
      tmrpcm.play("oct.wav");
      while(tmrpcm.isPlaying());
      break;
    case 11:
      tmrpcm.play("nov.wav");
      while(tmrpcm.isPlaying());
      break;
    case 12:
      tmrpcm.play("dec.wav");
      while(tmrpcm.isPlaying());
      break;         
  }
}
void TexttoSpeech(char digit)
{
  switch(digit) {
    case 0:
      tmrpcm.play("0.wav");
      while(tmrpcm.isPlaying());
      break;
    case 1:
      tmrpcm.play("1.wav");
      while(tmrpcm.isPlaying());
      break;
    case 2:
      tmrpcm.play("2.wav");
      while(tmrpcm.isPlaying());
      break;
    case 3:
      tmrpcm.play("3.wav");
      while(tmrpcm.isPlaying());
      break;
    case 4:
      tmrpcm.play("4.wav");
      while(tmrpcm.isPlaying());
      break;
    case 5:
      tmrpcm.play("5.wav");
      while(tmrpcm.isPlaying());
      break;
    case 6:
      tmrpcm.play("6.wav");
      while(tmrpcm.isPlaying());
      break;
    case 7:
      tmrpcm.play("7.wav");
      while(tmrpcm.isPlaying());
      break; 
    case 8:
      tmrpcm.play("8.wav");
      while(tmrpcm.isPlaying());
      break;
    case 9:
      tmrpcm.play("9.wav");
      while(tmrpcm.isPlaying());
      break;
    case 10:
      tmrpcm.play("10.wav");
      while(tmrpcm.isPlaying());
      break;
    case 11:
      tmrpcm.play("11.wav");
      while(tmrpcm.isPlaying());
      break;
    case 12:
      tmrpcm.play("12.wav");
      while(tmrpcm.isPlaying());
      break;
    case 13:
      tmrpcm.play("13.wav");
      while(tmrpcm.isPlaying());
      break;
    case 14:
      tmrpcm.play("14.wav");
      while(tmrpcm.isPlaying());
      break;
    case 15:
      tmrpcm.play("15.wav");
      while(tmrpcm.isPlaying());
      break;
    case 16:
      tmrpcm.play("16.wav");
      while(tmrpcm.isPlaying());
      break;
    case 17:
      tmrpcm.play("17.wav");
      while(tmrpcm.isPlaying());
      break;
    case 18:
      tmrpcm.play("18.wav");
      while(tmrpcm.isPlaying());
      break;
    case 19:
      tmrpcm.play("19.wav");
      while(tmrpcm.isPlaying());
      break;
    case 20:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      break;
    case 21:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("1.wav");
      while(tmrpcm.isPlaying());
      break;
    case 22:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("2.wav");
      while(tmrpcm.isPlaying());
      break;
    case 23:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("3.wav");
      while(tmrpcm.isPlaying());
      break;
    case 24:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("4.wav");
      while(tmrpcm.isPlaying());
      break;
     case 25:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("5.wav");
      while(tmrpcm.isPlaying());
      break;
     case 26:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("6.wav");
      while(tmrpcm.isPlaying());
      break;
     case 27:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("7.wav");
      while(tmrpcm.isPlaying());
      break;
     case 28:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("8.wav");
      while(tmrpcm.isPlaying());
      break;
    case 29:
      tmrpcm.play("20.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("9.wav");
      while(tmrpcm.isPlaying());
      break;
    case 30:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      break;
    case 31:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("1.wav");
      while(tmrpcm.isPlaying());
      break;
    case 32:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("2.wav");
      while(tmrpcm.isPlaying());
      break;
    case 33:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("3.wav");
      while(tmrpcm.isPlaying());
      break; 
    case 34:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("4.wav");
      while(tmrpcm.isPlaying());
      break;
    case 35:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("5.wav");
      while(tmrpcm.isPlaying());
      break;
    case 36:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("6.wav");
      while(tmrpcm.isPlaying());
      break;
    case 37:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("7.wav");
      while(tmrpcm.isPlaying());
      break;
    case 38:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("8.wav");
      while(tmrpcm.isPlaying());
      break;
    case 39:
      tmrpcm.play("30.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("9.wav");
      while(tmrpcm.isPlaying());
      break;
    case 40:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      break;
    case 41:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("1.wav");
      while(tmrpcm.isPlaying());
      break;
    case 42:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("2.wav");
      while(tmrpcm.isPlaying());
      break;
    case 43:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("3.wav");
      while(tmrpcm.isPlaying());
      break; 
    case 44:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("4.wav");
      while(tmrpcm.isPlaying());
      break;
    case 45:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("5.wav");
      while(tmrpcm.isPlaying());
      break;
    case 46:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("6.wav");
      while(tmrpcm.isPlaying());
      break;      
    case 47:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("7.wav");
      while(tmrpcm.isPlaying());
      break;
    case 48:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("8.wav");
      while(tmrpcm.isPlaying());
      break;
    case 49:
      tmrpcm.play("40.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("9.wav");
      while(tmrpcm.isPlaying());
      break;
    case 50:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      break;
    case 51:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("1.wav");
      while(tmrpcm.isPlaying());
      break;
    case 52:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("2.wav");
      while(tmrpcm.isPlaying());
      break;  
   case 53:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("3.wav");
      while(tmrpcm.isPlaying());
      break;
    case 54:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("4.wav");
      while(tmrpcm.isPlaying());
      break;  
    case 55:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("5.wav");
      while(tmrpcm.isPlaying());
      break;
    case 56:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("6.wav");
      while(tmrpcm.isPlaying());
      break;  
    case 57:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("7.wav");
      while(tmrpcm.isPlaying());
      break;
    case 58:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("8.wav");
      while(tmrpcm.isPlaying());
      break;
    case 59:
      tmrpcm.play("50.wav");
      while(tmrpcm.isPlaying());
      tmrpcm.play("9.wav");
      while(tmrpcm.isPlaying());
      break;                                       
   }
}

void WatchdogSetup()
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  MCUSR &= ~(1<<WDRF);  // because the data sheet said to
  /*
  WDTCSR configuration:
  WDIE = 1 :Interrupt Enable
  WDE = 1  :Reset Enable - I won't be using this on the 2560
  WDP3 = 0 :For 1000ms Time-out
  WDP2 = 1 :bit pattern is 
  WDP1 = 1 :0110  change this for a different
  WDP0 = 0 :timeout period.
  */
  // Enter Watchdog Configuration mode:
  WDTCSR = (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings: interrupte enable, 0110 for timer
  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
  sei();
}

ISR(WDT_vect) // Watchdog timer interrupt.
{ 
  if(millis() - resetTime > TIMEOUTPERIOD){
      resetFunc();     // This will call location zero and cause a reboot.
  } 
}

ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

void AmpPowerUp() 
{
  digitalWrite(AmpON, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(500);
}

void AmpPowerDown() 
{
  delay(500);
  digitalWrite(AmpON, LOW);
  digitalWrite(LED_RED, LOW);
  
}

void SetVolume () {
    static int dir = 0;
    
    dir = encoder.getDirection();
    
    if(dir == -1) {
       vol = vol + 1;
       if(vol > VOL_MAX)
          vol = VOL_MAX;
          EEPROM.write (0, vol);
          tmrpcm.setVolume(vol);
          Serial.print("Vol = ");
          Serial.println(vol);  
        } else if (dir == 1) {
          vol = vol - 1;
          if (vol < VOL_MIN)
            vol = VOL_MIN;
            Serial.print("Vol = ");
            Serial.println(vol);
            EEPROM.write (0, vol);
            tmrpcm.setVolume(vol);    
      }    
}
