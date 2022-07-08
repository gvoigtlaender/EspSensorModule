/* Copyright 2018 Georg Voigtlaender gvoigtlaender@googlemail.com */

#include <string>
#include <vector>
#include <fstream>

// const int VERSION = 2001;
const char VERSION_STRING[] = "2.0.0.3";

#include "./config.h"
/* config.h
  #define USE_DISPLAY 1               // [enable/disable display]
  #define DTH_TYPE DHT22              // [DHT11 / DHT22]
  #define DTH_PIN 12                  // [DIO pin]
  #define LEDPIN 13                   // [Status LED]
  #define ESP_STATUS 2                // [Attiny status feedback]
  #define HAS_NTP FALSE               // [not tested anymore
  #define WLAN_SSID "<WLAN>"          // [wifi ssid]
  #define WLAN_PASSWD "<PASSWD>"      // [wifi pass]
  #define mqtt_server "<MQTT_SERVER>" // [mqtt server ip]
*/

//! Wifi includes
#include <Arduino.h>
#include <ESP8266WiFi.h>


#include <Wire.h>

#include <PubSubClient.h>
#if USE_DISPLAY == 1
# include <U8x8lib.h>
# include <SPI.h>
#endif

#if HAS_NTP == 1
//! from various tutorials
# include <WiFiUdp.h>
# include <TimeLib.h>  // by Paul Stoffregen
# include <Timezone.h>  // by Jack Christensen
#endif


#include "CFilter.hpp"

// ADC_MODE(ADC_VCC);

#include <Adafruit_Sensor.h>
#include <DHT.h>


bool bDthDone = false;
bool bMqttDone = false;


class CControl {
 public:
  CControl()
  :m_nState(0)
  , m_uiTime(millis())
  , m_sInstanceName("") {
  }
  explicit CControl(const char* sInstance)
  :m_nState(0)
  , m_uiTime(millis())
  , m_sInstanceName(sInstance) {
    CControl::Log("Instance %s", sInstance);
    ms_Instances.push_back(this);
    // control(true);
  }

  // cppchecdk-suppress unusedFunction
  virtual void control(bool bForce) {
    // CControl::Log("%s->control(), time=%du\n",
    //  m_sInstanceName.c_str(), m_uiTime);
  }
  static void Log(const char *pcMessage, ...) {
    char czDebBuf[255] = {0};
    va_list arg_ptr;

    va_start(arg_ptr, pcMessage);
    vsprintf(czDebBuf, pcMessage, arg_ptr);
    va_end(arg_ptr);

    Serial.printf("%lu: \t%s\n", millis(), czDebBuf);
  }

  void _log(const char *pcMessage, ...) {
    char czDebBuf[255] = {0};
    va_list arg_ptr;

    va_start(arg_ptr, pcMessage);
    vsprintf(czDebBuf, pcMessage, arg_ptr);
    va_end(arg_ptr);

    Serial.printf("%lu: \t%s\t%s\n", millis(),
     m_sInstanceName.c_str(), czDebBuf);
  }

 protected:
  int m_nState;
  uint32_t m_uiTime;
  std::string m_sInstanceName;

 public:
  static std::vector<CControl*> ms_Instances;
  static void control() {
    for ( unsigned int n = 0; n < ms_Instances.size(); n++ )
      ms_Instances[n]->control(false);
  }
};
// static
std::vector<CControl*> CControl::ms_Instances;


class CControl_Wifi : CControl {
 public:
  CControl_Wifi() : CControl("Wifi") {}
  virtual void control(bool bForce);
  static bool ms_bDone;
  static uint32_t ms_uiProcessTime;
};
// static
bool CControl_Wifi::ms_bDone = false;
// static
uint32_t CControl_Wifi::ms_uiProcessTime = 0;


class CControl_Mqtt : CControl {
 public:
  CControl_Mqtt() : CControl("Mqtt")
  , m_clientName("") {}
  virtual void control(bool bForce);
  String m_clientName;
  static bool ms_bDone;
  static uint32_t ms_uiProcessTime;
};
// static
bool CControl_Mqtt::ms_bDone = false;
// static
uint32_t CControl_Mqtt::ms_uiProcessTime = 0;


class CControl_Sensors : CControl {
 public:
  CControl_Sensors() : CControl("Sensors") {}
  virtual void control(bool bForce);
  static bool ms_bDone;
  static uint32_t ms_uiProcessTime;
};
// static
bool CControl_Sensors::ms_bDone = false;
// static
uint32_t CControl_Sensors::ms_uiProcessTime = 0;


#if USE_DISPLAY == 1
class CControl_Display : CControl {
 public:
  CControl_Display() : CControl("Display") {
    m_uiTime += 1500;
  }
  virtual void control(bool bForce);
  static bool ms_bDone;
  static uint32_t ms_uiProcessTime;
  static bool ms_bTrigger;
};
// static
bool CControl_Display::ms_bDone = false;
// static
bool CControl_Display::ms_bTrigger = false;
// static
uint32_t CControl_Display::ms_uiProcessTime = 0;
#endif




class CEspSensorModule {
 public:
CEspSensorModule()
: m_bSingleCycleMode(false)
, m_uiLedPin(0)
, m_uiESP_STATUS(0)
, m_uiDhtPin(0)
, m_uiDhtType(DHT22)
, m_uiDisplayResetPin(0)
, m_sWifiSSID("")
, m_sWifiPassword("")
, m_pDth(NULL)
#ifdef U8X8_HAVE_HW_SPI
, m_pDisplay(NULL)
#endif
#if HAS_NTP == 1
, m_pUdp(NULL)
#endif
{
}
  // setup instance
  void SetupLed(uint8 uiLedPin, uint8 uiESP_STATUS);
  void SetupDht(uint8 uiDhtPin, uint8 uiDhtType);
  #ifdef U8X8_HAVE_HW_SPI
  void SetupDisplay(uint8 uiDisplayResetPin);
  #endif
  void SetupWifi(const std::string& sWifiSSID,
    const std::string& sWifiPassword);
#if HAS_NTP
  void UpdateTime();
#endif

  // fwk::setup
  void Setup();
  // fwk::loop
  void Loop();

  // protected:
  bool  m_bSingleCycleMode;
  uint8 m_uiLedPin;
  uint8 m_uiESP_STATUS;
  uint8 m_uiDhtPin;
  uint8 m_uiDhtType;
  uint8 m_uiDisplayResetPin;
  std::string m_sWifiSSID;
  std::string m_sWifiPassword;
#if HAS_NTP
  std::vector<std::string> m_sNtpServers;
#endif
  DHT*  m_pDth;
#ifdef U8X8_HAVE_HW_SPI
  U8X8* m_pDisplay;
#endif
#if HAS_NTP == 1
  WiFiUDP* m_pUdp;
#endif

 private:
#if HAS_NTP == 1
  bool getNtpTime(const char* ntpServerName) {
    Serial.print(F("NTP request..."));
    if (timeStatus() == timeSet) {
      Serial.println(F("not necessary"));
      return true;
    }

    IPAddress ntpServerIP;  // NTP server's ip address

    // discard any previously received packets
    while (m_pUdp->parsePacket() > 0) {
    }
    Serial.println(F("Transmit NTP Request"));
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    Serial.print(ntpServerName);
    Serial.print(": ");
    Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = m_pUdp->parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        Serial.println(F("Receive NTP Response"));
        // read packet into the buffer
        m_pUdp->read(packetBuffer, NTP_PACKET_SIZE);
        uint32 secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 = (uint8)packetBuffer[40] << 24;
        secsSince1900 |= (uint8)packetBuffer[41] << 16;
        secsSince1900 |= (uint8)packetBuffer[42] << 8;
        secsSince1900 |= (uint8)packetBuffer[43];
        setTime(secsSince1900 - 2208988800UL);
        // setTime(23, 55, 0, 30, 3, 2016); //simulate time for test
        return true;
      }
    }
    Serial.println(F("FATAL ERROR : No NTP Response."));
    return false;  // return 0 if unable to get the time
  }
  // send an NTP request to the time server at the given address
  void sendNTPpacket(const IPAddress &address) {
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:

    // NTP requests are to port 123
    m_pUdp->beginPacket(address, 123);
    m_pUdp->write(packetBuffer, NTP_PACKET_SIZE);
    m_pUdp->endPacket();
  }
#endif
};

void CEspSensorModule::SetupLed(uint8 uiLedPin, uint8 uiESP_STATUS) {
  CControl::Log("CEspSensorModule::SetupLed(%u, %u)", uiLedPin, uiESP_STATUS);
  m_uiLedPin = uiLedPin;
  pinMode(m_uiLedPin, OUTPUT);     // Initialize the relay pin as an output
  m_uiESP_STATUS = uiESP_STATUS;
  pinMode(m_uiESP_STATUS, OUTPUT);     // Initialize the relay pin as an output
  if (m_uiLedPin != 0) {
    digitalWrite(m_uiLedPin, LOW);
  }
  if (m_uiESP_STATUS != 0) {
    digitalWrite(m_uiESP_STATUS, LOW);
  }
  CControl::Log("done");
}

void CEspSensorModule::SetupDht(uint8 uiDhtPin, uint8 uiDhtType) {
  CControl::Log("CEspSensorModule::SetupDht(%u, %u)", uiDhtPin, uiDhtType);
  m_uiDhtPin = uiDhtPin;
  m_uiDhtType = uiDhtType;
  if (m_uiDhtPin != 0) {
    m_pDth = new DHT(m_uiDhtPin, m_uiDhtType);
    m_pDth->begin();
  }
  CControl::Log("done");
}

#ifdef U8X8_HAVE_HW_SPI
void CEspSensorModule::SetupDisplay(uint8 uiDisplayResetPin) {
  CControl::Log("CEspSensorModule::SetupDisplay(%u)", uiDisplayResetPin);
  m_uiDisplayResetPin = uiDisplayResetPin;
  m_pDisplay = new U8X8_SSD1306_128X32_UNIVISION_HW_I2C(m_uiDisplayResetPin);
  CControl::Log("done");
}
#endif

static int nErrorCounter = 0;
void CEspSensorModule::SetupWifi(const std::string& sWifiSSID,
  const std::string& sWifiPassword) {
  CControl::Log("CEspSensorModule::SetupWifi(%s, ***)", sWifiSSID.c_str());
  m_sWifiSSID = sWifiSSID;
  m_sWifiPassword = sWifiPassword;

  /*
  g_Wifi.control(true);

  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(sWifiSSID.c_str());



  WiFi.begin(sWifiSSID.c_str(), sWifiPassword.c_str());

  while (WiFi.status() != WL_CONNECTED && ++nErrorCounter < 50) {
    delay(200);
    // Serial.print(".");
    // updateDisplay();
    g_Display.control();
  }
  */
  CControl::Log("done");
}

#if HAS_NTP == 1
void CEspSensorModule::UpdateTime() {
  m_pUdp = new WiFiUDP();
  m_pUdp->begin(123);

  m_sNtpServers.push_back("ntp1.t-online.de");
  m_sNtpServers.push_back("time.nist.gov");

  int n = 0;
  Serial.print("UpdateTime ");
  while ( true ) {
    if (getNtpTime(m_sNtpServers[n++ % m_sNtpServers.size()].c_str())) {
      // bNtpDone = true;
      m_pUdp->stop();
      Serial.printf(" done %lu\n", millis());
      break;
    } else {
      Serial.print(".");
      delay(100);
    }
  }
}
#endif


// fwk::setup
void CEspSensorModule::Setup() {
}
// fwk::loop
void CEspSensorModule::Loop() {
}


CEspSensorModule EspSensorModule;

void blink(int nTimes, int nDelay) {
  CControl::Log("blink(%d, %d)", nTimes, nDelay);
  for ( int n = 0; n < nTimes; n++ ) {
    digitalWrite(LEDPIN, HIGH);   // turn the LED on
    delay(nDelay);                       // wait for a second
    digitalWrite(LEDPIN, LOW);    // turn the LED off
    if ( n < (nTimes-1) )
      delay(nDelay);                       // wait for a second
  }
  CControl::Log("done");
}


void SetupEspSensorModule() {
  EspSensorModule.SetupLed(LEDPIN, ESP_STATUS);

  blink(2, 10);

  // delay(1000);

  // blink(3, 100);

#ifdef U8X8_HAVE_HW_SPI
  EspSensorModule.SetupDisplay(U8X8_PIN_NONE);
  // updateDisplay();
#endif
  EspSensorModule.SetupDht(DTH_PIN, DTH_TYPE);
  EspSensorModule.SetupWifi(WLAN_SSID, WLAN_PASSWD);

#if HAS_NTP == 1
  EspSensorModule.UpdateTime();
#endif
  EspSensorModule.Setup();
  CControl::Log("SetupEspSensorModule() done");
}

#if HAS_NTP == 1
// NTP time is in the first 48 bytes of message
const int NTP_PACKET_SIZE = 48;
// buffer to hold incoming & outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];

// Timezone
// Central European Time (Frankfurt, Paris)
// Central European Summer Time
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };
// Central European Standard Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 60 };
Timezone CE(CEST, CET);
// pointer to the time change rule, use to get the TZ abbrev
TimeChangeRule *tcr;
time_t utc, local;
#endif


// #define mqtt_user "user"
// #define mqtt_password "password"

#define humidity_topic "sensor/humidity"
#define temperature_celsius_topic "sensor/temperature_celsius"
// #define temperature_fahrenheit_topic "sensor/temperature_fahrenheit"
// const char* inTopic = "led/in";
const char* supply_topic = "supply/voltate";
const char* milliseconds_topic = "process/milliseconds";
bool relayState = LOW;

WiFiClient espClient;
PubSubClient client(espClient);



CFilter<double> Voltage(10);
CFilter<int> Voltage_dig(10);
CFilter<double> Temperature(10);
CFilter<double> Humidity(10);

/*
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print(static_cast<char>(payload[i]));
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if (static_cast<char>(payload[0]) == '0') {
    // Turn the LED on (Note that LOW is the voltage level
    digitalWrite(LEDPIN, LOW);
    Serial.println("LEDPIN -> LOW");
    relayState = LOW;
  } else if (static_cast<char>(payload[0]) == '1') {
    // Turn the LED off by making the voltage HIGH
    digitalWrite(LEDPIN, HIGH);
    Serial.println("LEDPIN -> HIGH");
    relayState = HIGH;
  } else if (static_cast<char>(payload[0]) == '2') {
    relayState = !relayState;
    // Turn the LED off by making the voltage HIGH
    digitalWrite(LEDPIN, relayState);
    Serial.print("LEDPIN -> switched to ");
    Serial.println(relayState);
  }
}
*/

uint32 _time, _timeStart;
void setup() {
  _timeStart = millis();
  // delay(200);
  Serial.begin(74880);
  // delay(1000);
  Serial.println();
  CControl::Log("setup version: %s", VERSION_STRING);
  // delay(1000);
  SetupEspSensorModule();

  client.setServer(mqtt_server, 1883);
  // client.setCallback(callback);
  // digitalWrite(LEDPIN, relayState);

  new CControl_Wifi();
  new CControl_Mqtt();
  new CControl_Sensors();
#ifdef U8X8_HAVE_HW_SPI
  new CControl_Display();
#endif

  // blink(2, 10);

  _time = millis();
  nErrorCounter = 0;
  CControl::Log("::Setup() done");
}

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}


/*
void reconnect() {
  if ( WiFi.status() != WL_CONNECTED )
    return;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Generate client name based on MAC address and last 8 bits of msec counter
    String clientName;
    clientName += "esp8266-";
    uint8_t mac[6];
    WiFi.macAddress(mac);
    clientName += macToStr(mac);
    clientName += "-";
    clientName += String(micros() & 0xff, 16);
    Serial.print("Connecting to ");
    Serial.print(mqtt_server);
    Serial.print(" as ");
    Serial.println(clientName);


    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    if (client.connect(clientName.c_str())) {
      // Serial.println("connected");
      // client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}
*/
// const int c_nValues = 3;
int nLoop = 0;
float hic = 0.0;
float hum = 0.0;
float tmp = 0.0;
int nVoltage = 0;
bool bDTH11_OK = false;
int n2S = 0;
#if HAS_NTP == 1
bool bNtpDone = false;
#endif
const double dVoltageDiv = 944 / 5.145;  // 94.1954;
// cppcheckc-suppress unusedFunction
void loop() {
  EspSensorModule.Loop();
  nLoop++;

  // Serial.printf("loop %lu\n", millis());

  CControl::control();

  static bool bSwitched = false;
  if (CControl_Mqtt::ms_bDone && !bSwitched) {
    // digitalWrite(LEDPIN, HIGH);
    bSwitched = true;

    uint32 delta = millis() - _timeStart;
    CControl::Log("Programm complete, tool %lu [ms] ", delta);
    digitalWrite(ESP_STATUS, HIGH);

    blink(2, 10);
    delay(100);
    Serial.end();

    client.disconnect();
    WiFi.disconnect(true);

    delay(100);
    // updateDisplay();
    // g_Display.control(true);

#if defined U8X8_HAVE_HW_SPI
    // EspSensorModule.m_pDisplay->clearDisplay();
#endif

    delay(1000000);
    // ESP.deepSleep(0);
  }
  delay(0);
}

#include <Time.h>

String F2S(float f) {
  char szTmp[8];
  snprintf(szTmp, sizeof(szTmp), "%.1f", f);
  return String(szTmp);
}

String Date2S() {
  char szTmp[16];
  int nStep = n2S % 4;
  switch ( nStep ) {
    case 0:
#if HAS_NTP == 1
      if ( bNtpDone )
        snprintf(szTmp, sizeof(szTmp),
          "%02d.%02d.%02d", day(), month(), year());
      else
        snprintf(szTmp, sizeof(szTmp), " ntp not sync ");
      break;
#endif

    case 2:
#if HAS_NTP == 1
      if ( bNtpDone ) {
        snprintf(szTmp, sizeof(szTmp),
          "%02d:%02d:%02d", hour(), minute(), second());
      } else {
        snprintf(szTmp, sizeof(szTmp), " ntp not sync ");
      }
      break;
#endif

    case 1:
    case 3:
    default:
      snprintf(szTmp, sizeof(szTmp), "S: %dd %.2fV",
        Voltage_dig.m_OutputValue, Voltage.m_OutputValue);
      break;
  }

  return String(szTmp);
}

void CControl_Mqtt::control(bool bForce /*= false*/) {
  CControl::control(bForce);
  if ( this->m_uiTime > millis() && !bForce )
    return;

  this->m_uiTime += 5;
  // Serial.print(m_sInstanceName.c_str());
  // Serial.println(".control()");

  enum {
      eStart = 0,
      eWaitForWifi,
      eSetup,
      eConnect,
      ePublish,
      eDone
    };

  switch ( this->m_nState ) {
    case eStart:
      CControl_Display::ms_bTrigger = true;
      _log("W4Wifi");
      this->m_nState = eWaitForWifi;

    case eWaitForWifi:
      // wait for WiFi
      if ( !CControl_Wifi::ms_bDone || WiFi.status() != WL_CONNECTED ) {
        CControl_Display::ms_bTrigger = true;
        break;
      }

      // Generate client name based on MAC address and last 8 bits of msec cnt
      CControl_Display::ms_uiProcessTime = millis();
      this->m_clientName = "";
      this->m_clientName += "esp8266-";
      uint8_t mac[6];
      WiFi.macAddress(mac);
      this->m_clientName += macToStr(mac);
      this->m_clientName += "-";
      this->m_clientName += String(micros() & 0xff, 16);

      _log("Connecting to %s as %s",
        mqtt_server, this->m_clientName.c_str());

      CControl_Display::ms_bTrigger = true;
      this->m_nState = eSetup;
      break;

    case eSetup:

      client.connect(this->m_clientName.c_str());
      this->m_nState = eConnect;

    case eConnect:
      if ( client.connected() ) {
        CControl_Display::ms_bTrigger = true;
        _log("connected");
        this->m_nState = ePublish;
        break;
      }
      _log("connecting");
      break;

    case ePublish:
      client.loop();
      if ( CControl_Sensors::ms_bDone ) {
        CControl_Display::ms_bTrigger = true;
        _log("Sensors::Done -> Mqtt->publish()");
        client.publish("espsm1/Module/Version", VERSION_STRING, true);
        client.publish("espsm1/Sensor/Temperature",
          String(Temperature.m_OutputValue).c_str(), true);
        client.publish("espsm1/Sensor/Humidity",
          String(Humidity.m_OutputValue).c_str(), true);
        client.publish("espsm1/Module/Voltage",
          String(Voltage.m_OutputValue, 2).c_str(), true);
        client.publish("espsm1/Module/Voltage_dig",
          String(Voltage_dig.m_OutputValue).c_str(), true);
        client.publish("espsm1/Module/milliseconds",
          String(millis() - _timeStart).c_str(), true);
        client.publish("espsm1/Module/milliseconds_wifi",
          String(CControl_Wifi::ms_uiProcessTime).c_str(), true);
        client.publish("espsm1/Module/milliseconds_sensor",
          String(CControl_Sensors::ms_uiProcessTime).c_str(), true);
        CControl_Mqtt::ms_bDone = true;
        this->m_nState = eDone;
        CControl_Display::ms_uiProcessTime = millis()
          - CControl_Display::ms_uiProcessTime;
      }
      break;

    case eDone:
      break;
  }
}

void CControl_Wifi::control(bool bForce /*= false*/) {
  CControl::control(bForce);
  if ( this->m_uiTime > millis() && !bForce )
    return;

  this->m_uiTime += 5;
  // Serial.print(m_sInstanceName.c_str());
  // Serial.println(".control()");

  switch ( this->m_nState ) {
    case 0:
      CControl_Wifi::ms_uiProcessTime = millis();
      CControl_Display::ms_bTrigger = true;
      // We start by connecting to a WiFi network
      _log("Connecting to %s",
        EspSensorModule.m_sWifiSSID.c_str());
      {
#if WIFI_STATIC == 1
        IPAddress oIP; oIP.fromString(WIFI_STATIC_IP);
        IPAddress oGW; oGW.fromString(WIFI_STATIC_GW);
        IPAddress oSN; oSN.fromString(WIFI_STATIC_SN);
        WiFi.config(oIP, oGW, oSN);
#endif
        WiFi.mode(WIFI_AP_STA);

        WiFi.persistent(false);
        WiFi.begin(EspSensorModule.m_sWifiSSID.c_str(),
          EspSensorModule.m_sWifiPassword.c_str());
      }
      this->m_nState = 1;
      break;

    case 1:
      if ( WiFi.status() == WL_CONNECTED ) {
        CControl_Display::ms_bTrigger = true;
        CControl_Wifi::ms_uiProcessTime = millis()
         - CControl_Wifi::ms_uiProcessTime;
        _log("connected");
        this->m_nState = 2;
        break;
      }

      break;

    case 2:
      CControl_Wifi::ms_bDone = true;
      break;
  }
}

const unsigned int c_SensorLoops = 2;
void CControl_Sensors::control(bool bForce /*= false*/) {
  CControl::control(bForce);
  if ( this->m_uiTime > millis() && !bForce )
    return;

  this->m_uiTime += 20;
  CControl_Display::ms_bTrigger = true;
  // Serial.print(m_sInstanceName.c_str());
  // Serial.println(".control()");

  switch ( this->m_nState ) {
    case 0:
      CControl_Sensors::ms_uiProcessTime = millis();
      this->m_nState = 1;

    case 1:
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 secs 'old' (its a very slow sensor)
      hum = EspSensorModule.m_pDth->readHumidity();
      // Read temperature as Celsius (the default)
      tmp = EspSensorModule.m_pDth->readTemperature();

      if ( isnan(hum) || isnan(tmp) ) {
        _log("Failed to read from DHT sensor!");
      } else {
        Temperature.Filter(tmp);
        Humidity.Filter(hum);
        nVoltage = analogRead(A0);
        Voltage_dig.Filter(nVoltage);
        Voltage.Filter(nVoltage/dVoltageDiv);
        bDTH11_OK = true;

        _log("Temp=%.1fC, Cnt=%d", tmp, Temperature.m_nSize);
        if ( Temperature.m_nSize >= c_SensorLoops ) {
          CControl_Sensors::ms_bDone = true;
          this->m_nState = 2;
          CControl_Sensors::ms_uiProcessTime = millis()
           - CControl_Sensors::ms_uiProcessTime;
        }
        this->m_uiTime += 1000;
      }

      break;

    case 2:
      break;
  }
}

String sLines[4];
void CControl_Display::control(bool bForce /*= false*/) {
  CControl::control(bForce);

    enum  {
      eStart = 0,
      eSetup,
      eCycle
      };

  if ( this->m_uiTime > millis() && !bForce
    && (!CControl_Display::ms_bTrigger || this->m_nState < eCycle) )
    return;

  switch ( this->m_nState ) {
    case eStart:
      _log("start");
      this->m_nState = eSetup;

    case eSetup:
      EspSensorModule.m_pDisplay->begin();
      EspSensorModule.m_pDisplay->setPowerSave(0);
      EspSensorModule.m_pDisplay->setFont(u8x8_font_chroma48medium8_r);
      EspSensorModule.m_pDisplay->setFont(u8x8_font_5x8_r);
      _log("started");
      this->m_nState = eCycle;

    case eCycle:
    CControl_Display::ms_bTrigger = false;
    this->m_uiTime += 3000;
    // Serial.print(m_sInstanceName.c_str());
    // Serial.println(".control()");

    String sTmp[4];
    //! IP
    sTmp[0] = String("IP:") + String((WiFi.status() == WL_CONNECTED)
    ? WiFi.localIP().toString() : "connecting");

    //! MQTT & LED
    sTmp[1] = String("M:") + String((client.connected())
      ? String("1") : String("0"));
    sTmp[1] += String(" LED:") + String((relayState == LOW) ? "0" : "1");

    //! Temp & Humidity
    if ( bDTH11_OK ) {
      sTmp[2] = String("T:") + F2S(tmp) + String("°C");
      sTmp[2] += String(" H:") + F2S(hum) + String("p");
    }  else {
      sTmp[2] = String("DTH11 FAILURE");
    }

    //! Temp & Humidity
    sTmp[3] = Date2S();


    for ( unsigned int n = 0; n < 4; n++ ) {
      if ( sTmp[n] != sLines[n] ) {
        sLines[n] = sTmp[n];
        EspSensorModule.m_pDisplay->clearLine(n);
        EspSensorModule.m_pDisplay->drawUTF8(0, n, sTmp[n].c_str());
        if ( !CControl_Sensors::ms_bDone )
          _log("\t%d: %s", n, sTmp[n].c_str());
      }
    }
    EspSensorModule.m_pDisplay->setContrast(n2S % 256);
    break;
  }
}
