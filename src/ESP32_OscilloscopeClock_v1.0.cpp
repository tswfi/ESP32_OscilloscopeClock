/******************************************************************************

  ESP32 Oscilloscope Clock
  using internal DACs, with WiFi and ntp sync.

  Based on great work by: Mauro Pintus , Milano 2018/05/25
  GitHub Repository
  https://github.com/maurohh/ESP32_OscilloscopeClock

  This modified version by: Tatu Wikman
  https://github.com/tswfi/ESP32_OscilloscopeClock

  How to use it:
  Build and upload to esp32, connect dac pins to oscilloscope on xy mode and
  follow the instructions to get wifi and timezone adjusted
******************************************************************************/
#include "Arduino.h"

// for saving settings in spiffs
#include "FS.h"
#include "SPIFFS.h"
#include "ArduinoJson.h"

//
#include "driver/dac.h"
#include "soc/rtc.h"
#include "soc/sens_reg.h"
#include "simpleDSTadjust.h"
#include "WiFi.h"

// data to show
#include "DataTable.h"

// for wifi manager
#include "DNSServer.h"
#include "WebServer.h"
#include "WiFiManager.h"

// timezone and dst rules
int utc_offset = 0;
struct dstRule StartRule = {"PDT", Second, Sun, Mar, 2, 3600}; // Pacific Daylight time = UTC/GMT -7 hours
struct dstRule EndRule = {"PST", First, Sun, Nov, 1, 0};       // Pacific Standard time = UTC/GMT -8 hour
simpleDSTadjust dstAdjusted(StartRule, EndRule);

WiFiManager wifiManager;
bool shouldSaveConfig = false;

// change for different NTP (time servers)
#define NTP_SERVERS "pool.ntp.org"

// August 1st, 2018
#define NTP_MIN_VALID_EPOCH 1533081600

//Variables
int lastx, lasty;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long ntp_refresh_interval = 10 * 60 * 1000; // in milliseconds, 10 minutes
bool ntpsynchappened = false;

// Draw a dot in X Y
inline void Dot(int x, int y)
{
  if (lastx != x)
  {
    lastx = x;
    dac_output_voltage(DAC_CHANNEL_1, x);
  }
  if (lasty != y)
  {
    lasty = y;
    dac_output_voltage(DAC_CHANNEL_2, y);
  }
}

// Bresenham's Algorithm implementation optimized
// also known as a DDA - digital differential analyzer
void Line(byte x1, byte y1, byte x2, byte y2)
{
  int n = 2;
  int acc;
  bool first_point = true;
  // for speed, there are 8 DDA's, one for each octant
  if (y1 < y2)
  { // quadrant 1 or 2
    byte dy = y2 - y1;
    if (x1 < x2)
    { // quadrant 1
      byte dx = x2 - x1;
      if (dx > dy)
      { // < 45
        acc = (dx >> 1);
        for (; x1 <= x2; x1++)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dy;
          if (acc < 0)
          {
            y1++;
            acc += dx;
          }
        }
      }
      else
      { // > 45
        acc = dy >> 1;
        for (; y1 <= y2; y1++)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dx;
          if (acc < 0)
          {
            x1++;
            acc += dy;
          }
        }
      }
    }
    else
    { // quadrant 2
      byte dx = x1 - x2;
      if (dx > dy)
      { // < 45
        acc = dx >> 1;
        for (; x1 >= x2; x1--)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dy;
          if (acc < 0)
          {
            y1++;
            acc += dx;
          }
        }
      }
      else
      { // > 45
        acc = dy >> 1;
        for (; y1 <= y2; y1++)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dx;
          if (acc < 0)
          {
            x1--;
            acc += dy;
          }
        }
      }
    }
  }
  else
  { // quadrant 3 or 4
    byte dy = y1 - y2;
    if (x1 < x2)
    { // quadrant 4
      byte dx = x2 - x1;
      if (dx > dy)
      { // < 45
        acc = dx >> 1;
        for (; x1 <= x2; x1++)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dy;
          if (acc < 0)
          {
            y1--;
            acc += dx;
          }
        }
      }
      else
      { // > 45
        acc = dy >> 1;
        for (; y1 >= y2; y1--)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dx;
          if (acc < 0)
          {
            x1++;
            acc += dy;
          }
        }
      }
    }
    else
    { // quadrant 3
      byte dx = x1 - x2;
      if (dx > dy)
      { // < 45
        acc = dx >> 1;
        for (; x1 >= x2; x1--)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dy;
          if (acc < 0)
          {
            y1--;
            acc += dx;
          }
        }
      }
      else
      { // > 45
        acc = dy >> 1;
        for (; y1 >= y2; y1--)
        {
          Dot(x1, y1);
          if (first_point)
          {
            delayMicroseconds(n);
            first_point = false;
          }
          acc -= dx;
          if (acc < 0)
          {
            x1--;
            acc += dy;
          }
        }
      }
    }
  }
}

void PlotTable(byte *SubTable, int SubTableSize, int skip, int opt, int offset)
{
  int i = offset;
  while (i < SubTableSize)
  {
    if (SubTable[i + 2] == skip)
    {
      i = i + 3;
      if (opt == 1)
        if (SubTable[i] == skip)
          i++;
    }
    Line(SubTable[i], SubTable[i + 1], SubTable[i + 2], SubTable[i + 3]);
    if (opt == 2)
    {
      Line(SubTable[i + 2], SubTable[i + 3], SubTable[i], SubTable[i + 1]);
    }
    i = i + 2;
    if (SubTable[i + 2] == 0xFF)
      break;
  }
}

// time_t getNtpTime()
void getNtpTime()
{
  time_t now;

  int i = 0;
  configTime(utc_offset * 3600, 0, NTP_SERVERS);
  while ((now = time(nullptr)) < NTP_MIN_VALID_EPOCH)
  {
    delay(500);
    i++;
    if (i > 60)
      break;
  }

  ntpsynchappened = true;
}

// callback coming back from config mode
void configModeCallback(WiFiManager *myWiFiManager)
{
  shouldSaveConfig = true;
}

void writeConfig(String json)
{
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("could not write the default config file, freezing");
    while (1)
      ;
  }
  configFile.println(json);
  Serial.println("Wrote default config");
  configFile.close();
}

void readConfig()
{
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile)
  {
    Serial.println("could not read the config file, freezing");
    while (1)
      ;
  }
  size_t size = configFile.size();
  std::unique_ptr<char[]> buf(new char[size]);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, configFile.readString());
  if (error)
  {
    Serial.println("Failed to deserialize config file, freezing");
    while (1)
      ;
  }
  // read our utc offset from the doc
  utc_offset = doc["utc_offset"];
  Serial.print("Read utc_offset from config: ");
  Serial.println(utc_offset);
  configFile.close();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\nESP32 Oscilloscope Clock v1.0");
  Serial.println("Mauro Pintus 2018\nwww.mauroh.com");
  Serial.println("Extended by Tatu Wikman");

  if (!SPIFFS.begin(true))
  {
    Serial.println("Failed to start or format SPIFFS, freezing");
    while (1)
      ;
  }

  Serial.println("mounted SPIFFS");
  if (!SPIFFS.exists("/config.json"))
  {
    writeConfig("{\"utcOffset\": 0}");
  }
  // read the config from SPIFFS
  readConfig();

  Serial.println("Starting WifiManager");

  // register our callback
  wifiManager.setAPCallback(configModeCallback);

  // timezone paramater for wifimanager
  char buf[8];
  itoa(utc_offset, buf, 10);
  WiFiManagerParameter timezoneparameter("timezone", "Timezone: +2, -8, 0", buf, 3);
  wifiManager.addParameter(&timezoneparameter);

  // connect to network or start ap to get the network information
  wifiManager.autoConnect();

  if(shouldSaveConfig) {
    char new_utc_offset[15];
    strcpy(new_utc_offset, timezoneparameter.getValue());
    String newconfig = "{\"utcOffset\": ";
    newconfig.concat(new_utc_offset);
    newconfig.concat("}");
  }

  // start dac
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

  // fetch time on startup
  getNtpTime();

  previousMillis = currentMillis;
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= ntp_refresh_interval)
  {
    previousMillis = currentMillis;
    getNtpTime();
  }

  char *dstAbbrev;
  time_t now = dstAdjusted.time(&dstAbbrev);
  struct tm *timeinfo = localtime(&now);

  if (ntpsynchappened)
  {
    Serial.println();
    Serial.printf("NTP Sync time now: %d:%d:%d\n", timeinfo->tm_hour % 12, timeinfo->tm_min, timeinfo->tm_sec);
    ntpsynchappened = false;
  }

  int h = timeinfo->tm_hour % 12;
  int m = timeinfo->tm_min;
  int s = timeinfo->tm_sec;

  PlotTable(DialData, sizeof(DialData), 0x00, 1, 0);                        //2 to back trace
  PlotTable(DialDigits12, sizeof(DialDigits12), 0x00, 1, 0);                //2 to back trace
  PlotTable(HrPtrData, sizeof(HrPtrData), 0xFF, 0, (9 * (5 * h + m / 12))); // 9*h
  PlotTable(MinPtrData, sizeof(MinPtrData), 0xFF, 0, 9 * m);                // 9*m
  PlotTable(SecPtrData, sizeof(SecPtrData), 0xFF, 0, 5 * s);                // 5*s
}
