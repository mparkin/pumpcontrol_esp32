
/*
 * HelTec Automation(TM) WIFI_Kit_32 factory test code, witch includ
 * follow functions:
 *
 * - Basic OLED function test;
 *
 * - Basic serial port test(in baud rate 115200);
 *
 * - LED blink test;
 *
 * - WIFI join and scan test;
 *
 * - Timer test and some other Arduino basic functions.
 *
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.cn
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include "Arduino.h"
#include "heltec.h"
#include "oled/OLEDDisplayUi.h"
#include "images.h"
#include "WiFi.h"
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <TimeLib.h>
#include <Time.h>
#include <analogWrite.h>

#define SPEED 26
#define DEMO_DURATION 3000
typedef void (*Demo)(void);
const int timeZone = -7;  // Pacific Daylight Time (USA)

extern Heltec_ESP32 Heltec;
OLEDDisplayUi ui( Heltec.display );

#ifndef STASSID
#define STASSID "940dm2"
#define STAPSK  "thisisnewrouter"
#endif

unsigned int localPort = 2390;      // local port to listen for UDP packets
char dateTime[10];
/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
WiFiMulti wifiMulti;

//how many clients should be able to telnet to this ESP32
#define MAX_SRV_CLIENTS 1
WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
bool wifiConnected = false;

char timeStamp[]= "00:00:00";

void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(128, 0, String(millis()));
}



void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
//  display->drawXbm(x, y, BT_width, BT_height, BT_bits);
  if(wifiConnected)display->drawXbm(x+BT_width, y, WIFI_width, WIFI_height, WIFI_bits);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x+BT_width+WIFI_width+1, y,dateTime);
  display->drawXbm(x + 108, y, BAT_width, BAT_height, BAT_bits);
  display->drawXbm(x + 34, y + 14, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
}

void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
//  display->drawXbm(x, y, BT_width, BT_height, BT_bits);
    if(wifiConnected)display->drawXbm(x+BT_width, y, WIFI_width, WIFI_height, WIFI_bits);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(x+BT_width+WIFI_width+1, y,dateTime); 
    display->drawXbm(x + 108, y, BAT_width, BAT_height, BAT_bits);
    display->drawXbm(x + 34, y + 12, LoRa_Logo_width, LoRa_Logo_height, LoRa_Logo_bits);
}

void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->drawXbm(x, y + 5, HelTec_LOGO_width, HelTec_LOGO_height, HelTec_LOGO_bits);
}

void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x, y,dateTime);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x, y + 25, "Idle");
  display->drawString(x, y + 35, "Waiting for input");
}

FrameCallback frames[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4 };

int frameCount = 4;

bool WIFISetUp(void)
{
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(STASSID,STAPSK);
  delay(100);

  byte count = 0;
  while(WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Heltec.display -> drawString(0, 0, "Connecting...");
    Heltec.display -> display();
  }

  Heltec.display -> clear();
  if(WiFi.status() == WL_CONNECTED)
  {
    Heltec.display -> drawString(0, 0, "Connecting...OK.");
    wifiConnected = true;
    Heltec.display -> display();
    delay(500);
  }
  else
  {
    Heltec.display -> clear();
    Heltec.display -> drawString(0, 0, "Connecting...Failed");
    Heltec.display -> display();
    while(1);
  }
  Heltec.display -> drawString(0, 10, "WIFI Setup done");
  Heltec.display -> display();
  delay(500);
  return WiFi.status();
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

 time_t getNtpTime(){
   //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  } else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    unsigned long secsSince1900;
    // convert four bytes starting at location 40 to a long integer
    secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
    secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
    secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
    secsSince1900 |= (unsigned long)packetBuffer[43];
    return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
  }
  return 0;
}

char *digitalClockDisplay()
{
  time_t t = now();
  // digital clock display of the time

  sprintf(dateTime,"%02hhu:%02hhu:%02hhu",hour(),minute(),second());
  //Serial.print(dateTime);
  //Serial.println();
  return dateTime;
}

void setup()
{
	pinMode(LED,OUTPUT);
	digitalWrite(LED,HIGH);
  pinMode(5, OUTPUT); 
  analogWrite(SPEED,0);
	Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);

  ui.setTargetFPS(90);

  // Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(BOTTOM);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Initialising the UI will init the display too.
  ui.init();

  Heltec.display->flipScreenVertically();
  
	if(WIFISetUp()){
     udp.begin(localPort);
     setSyncProvider(getNtpTime);
     setSyncInterval(300);
     server.begin();
     server.setNoDelay(true);
	}
 
}

void loop()
{
	int remainingTimeBudget = ui.update();
  if (remainingTimeBudget > 0) {
    digitalClockDisplay();
    delay(remainingTimeBudget); 
  }
}
