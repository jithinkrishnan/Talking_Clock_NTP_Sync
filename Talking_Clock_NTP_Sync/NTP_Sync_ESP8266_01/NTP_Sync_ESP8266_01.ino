/************************************************************************
*   ESP8266 NTP - DS3231 RTC Time sync
*   File:   NTP_Sync_ESP8266_01.ino
*   Author:  Jithin Krishnan.K
*       Rev. 1.0 : 10/08/2020 :  09:10 PM
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
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <Time.h>
#include <TimeLib.h>
#include "DS3231.h"

#define SOFT_SDA  0			
#define SOFT_SCL  2

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

DS3231 clk;
RTCDateTime dt, ist_dt;

char ssid[] = "********";    // SSID
char pass[] = "********";;   // Wifi Password
unsigned int localPort = 2390;

IPAddress timeServerIP; 
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];

char do_once = 0;
//uint8_t deg_sym[8]  = {0x6,0x9,0x9,0x6,0x0,0,0,0};

WiFiUDP udp;

void setup()
{
  Serial.begin(115200);
  clk.begin(SOFT_SDA, SOFT_SCL); // In ESP8266-01, SDA=0, SCL=2   
  Serial.print("Wifi Connecting");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  }
  
  Serial.print("WiFi Connected!");
  udp.begin(localPort);
}

void loop()
{

  if (!do_once) {
    WiFi.hostByName(ntpServerName, timeServerIP);
    sendNTPpacket(timeServerIP);
    delay(1000);

    int cb = udp.parsePacket();
  
    if (!cb) {
    } else {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    epoch += 19800UL; // GMT+5:30
        
    ist_dt.hour   = hour(epoch);
    ist_dt.minute = minute(epoch);
    ist_dt.second = second(epoch);
    ist_dt.year   = year(epoch);
    ist_dt.month  = month(epoch);
    ist_dt.day    = day(epoch);
    
    dt = clk.getDateTime(); 
    
    if((ist_dt.year != dt.year || ist_dt.month != dt.month || ist_dt.day != dt.day ||
        ist_dt.hour != dt.hour) || (ist_dt.minute != dt.minute) || (ist_dt.second != dt.second))  {
        clk.setDateTime(ist_dt.year, ist_dt.month, ist_dt.day, ist_dt.hour, ist_dt.minute, ist_dt.second);
    }
   }
   do_once = -1;
  }
}


unsigned long sendNTPpacket(IPAddress& address) {
   memset(packetBuffer, 0, NTP_PACKET_SIZE);
   packetBuffer[0] = 0b11100011;   // LI, Version, Mode
   packetBuffer[1] = 0;     // Stratum, or type of clock
   packetBuffer[2] = 6;     // Polling Interval
   packetBuffer[3] = 0xEC;  // Peer Clock Precision
   packetBuffer[12]  = 49;
   packetBuffer[13]  = 0x4E;
   packetBuffer[14]  = 49;
   packetBuffer[15]  = 52;
   udp.beginPacket(address, 123); //NTP requests are to port 123
   udp.write(packetBuffer, NTP_PACKET_SIZE);
   udp.endPacket();
}
