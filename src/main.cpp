/* Application to use the SH_ESP32 as a NMEA2000 device providing barometric pressure 
   Addition hardware: Bosch PMB280, Adafruit SSD1306 (optional) 
   Author: Aswin Bouwmeester
   Version: 1.0
   */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <Wire.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <Preferences.h>

// SDA and SCL pins on SH-ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// Update period
#define PressureUpdatePeriod 4000

// Calibration file
#define CAL_FILE "/calibration"


TwoWire *i2c;
Adafruit_SSD1306 *display;
Adafruit_GFX *graph;
Adafruit_BMP280 *bmp;
tNMEA2000 &NMEA2000 = *(new tNMEA2000_esp32());
AsyncWebServer server(80);
Preferences preferences;

double pressure;
double temperature;
double pres_offset = 0;
double temp_offset = 0;
IPAddress localIP;

/* Shows a short message on the OLED and optionally pauses the system for a given amount of time for the message to be read. 
   Messages are also sent to the Serial output. */
void showMessage(char const *text, int t = 0, int line = 0)
{
  display->clearDisplay();
  display->setCursor(0, 16 * line);

  display->printf(text);
  display->display();
  Serial.println(text);
  delay(t);
}

/* Shows a short line of text on the OLED and duplicates the message to the Serial output. */
void showLine(char const *text, int line = 0)
{
  display->setCursor(0, 16 * line);
  display->printf(text);
  display->display();
  Serial.println(text);
}

void showLine(String text, int line = 0)
{
  display->setCursor(0, 16 * line);
  display->print(text);
  display->display();
  Serial.println(text);
}

/* Initializes the OLDE */
bool SSD1306_init()
{
  // initialize the display
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);

  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    showMessage("SSD1306 error", 1000);
    return false;
  }
  else
  {
    delay(100);
    display->setRotation(2);
    display->clearDisplay();
    display->setTextSize(2);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    showMessage("SSD1306 ready", 0);
    return true;
  }
}

/* Initializes the B|MP280 pressure sensor */

bool BMP280_init()
{
  bmp = new Adafruit_BMP280(i2c);
  if (!bmp->begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
  {
    showMessage("BMP error", 1000);
    return false;
  }
  else
  {
    showMessage("BMP ready", 0);
    return true;
  }
}

/* Initializes NMEA200 */
bool NMEA2000_init()
{
  // Set Product information
  NMEA2000.SetProductInformation("00000001",   // Manufacturer's Model serial code
                                 100,          // Manufacturer's product code
                                 "Sensor hub", // Manufacturer's Model ID
                                 "1.0",        // Manufacturer's Software version code
                                 "1.0"         // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(ESP.getEfuseMac(), // Unique number. Use e.g. Serial number.
                                130,               // Device function=Pressure. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,                // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040               // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  const unsigned long TransmitMessages[] PROGMEM = {130310L, 130311L, 130314L, 130315L, 0};
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  if (NMEA2000.Open())
  {
    showMessage("NMEA ready", 0);
    return true;
  }
  else
  {
    showMessage("NMEA error", 1000);
    return false;
  }
}

/* Slows down the ESP to save energy */
void setPower()
{
  setCpuFrequencyMhz(80);
}

/* Sends the actual pressure over the NMEA2000 Bus every 4 seconds (Interval can be set using PressureUpdatePeriod).  
   Temperature is also sent in PGN130311 as inside temperature. */
void sendPressure()
{
  static unsigned long PressureUpdated = 0;
  tN2kMsg N2kMsg;
  double pressure_calibrated;
  double temperature_calibrated;

  if (PressureUpdated + PressureUpdatePeriod < millis())
  {
    PressureUpdated = millis();
    pressure = bmp->readPressure();
    temperature = bmp->readTemperature();
    pressure_calibrated = pressure + pres_offset;
    temperature_calibrated = temperature + temp_offset;

    // 130310
    SetN2kPGN130310(N2kMsg, 1, N2kDoubleNA, N2kDoubleNA, pressure_calibrated);
    NMEA2000.SendMsg(N2kMsg);

    // 130311
    SetN2kPGN130311(N2kMsg, 1, N2kts_InsideTemperature, CToKelvin(temperature_calibrated), N2khs_Undef, N2kDoubleNA, pressure_calibrated);
    NMEA2000.SendMsg(N2kMsg);

    // 130314
    SetN2kPGN130314(N2kMsg, 1, 1, tN2kPressureSource::N2kps_Atmospheric, pressure_calibrated);
    NMEA2000.SendMsg(N2kMsg);

    // 130315
    SetN2kPGN130315(N2kMsg, 1, 1, tN2kPressureSource::N2kps_Atmospheric, pressure_calibrated);
    NMEA2000.SendMsg(N2kMsg);
  }
}


bool Wifi_init_station()
{
  if (!preferences.isKey("ssid_station")) return false;
  if (preferences.getString("ssid_station").isEmpty()) return false;
  WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
  if (WiFi.begin(preferences.getString("ssid_station").c_str(), preferences.getString("pw_station").c_str()))
  {
    long start= millis();
    while (!WiFi.isConnected() && millis() < start + 3000);
    if (WiFi.isConnected()) {
    showMessage(WiFi.localIP().toString().c_str(), 2000);
    localIP = WiFi.localIP();
    showMessage("WiFi station", 0);
    return true;
    }
    else return false;
  }
  return false;
}

/* Initializes a Wifi access point */
bool Wifi_init_access()
{
  WiFi.mode(wifi_mode_t::WIFI_MODE_AP);
  if (WiFi.softAP(preferences.getString("ssid_ap").c_str(), preferences.getString("pw_ap").c_str()))
  {
    IPAddress local(192, 168, 1, 1), subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local, local, subnet);
    showMessage("WiFi ap", 0);
    localIP = WiFi.softAPIP();
    return true;
  }
  return false;
}

bool Wifi_init()
{
  if (!Wifi_init_station())
  {
    if (!Wifi_init_access())
    {
      showMessage("WiFi error", 0);
      return false;
    };
  }
  return true;
}

/* Handle request for non existing pages */
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

/* Replaces placeholders in index.html page */
String processor(const String &var)
{
  char text[12];
  if (var == "PRES_RAW")
  {
    sprintf(text, "%7.2f", PascalTomBar(pressure));
    return text;
  }
  if (var == "PRES_OFFSET")
  {
    sprintf(text, "%7.2f", PascalTomBar(pres_offset));
    return text;
  }
  if (var == "PRES_CAL")
  {
    sprintf(text, "%7.2f", PascalTomBar(pressure + pres_offset));
    return text;
  }
  if (var == "TEMP_RAW")
  {
    sprintf(text, "%2.2f", temperature);
    return text;
  }
  if (var == "TEMP_OFFSET")
  {
    sprintf(text, "%2.2f", temp_offset);
    return text;
  }
  if (var == "TEMP_CAL")
  {
    sprintf(text, "%2.2f", temperature + temp_offset);
    return text;
  }

  return String();
}

String processorPreferences(const String &var)
{
  if (preferences.isKey(var.c_str())) {
    if (preferences.getType(var.c_str()) == PreferenceType::PT_STR) {
      return preferences.getString(var.c_str());
    }
  }
  /*
  if (var == "SSID_STATION")
  {
    return preferences.getString("ssid_station", " ");
  }
  if (var == "PW_STATION")
  {
    return preferences.getString("pw_station", " ");
  }
  if (var == "SSID_AP")
  {
    return preferences.getString("ssid_ap", "sensorhub");
  }
  if (var == "PW_AP")
  {
    return preferences.getString("pw_ap", "1234567890");
  }
*/
  return String();
}

/* Initializes the web server */
bool Webserver_init()
{

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    showMessage("Request");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route for root / web page
  server.on("/wificonfig.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    showMessage("Request");
    request->send(SPIFFS, "/wificonfig.html", String(), false, processorPreferences);
  });


  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to submit pressure calibration
  server.on("/cal_pres", HTTP_GET, [](AsyncWebServerRequest *request) {
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("pressure"))
    {
      if (!request->getParam("pressure")->value().isEmpty())
      {
        pres_offset = mBarToPascal(request->getParam("pressure")->value().toDouble()) - bmp->readPressure();
        preferences.putDouble("pressure", pres_offset);
      }
    }
    request->redirect("/");
  });

  // Route to submit temperature calibration
  server.on("/cal_temp", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("temperature"))
    {
      if (!request->getParam("temperature")->value().isEmpty())
      {
        temp_offset = request->getParam("temperature")->value().toDouble() - bmp->readTemperature();
        preferences.putDouble("temperature", temp_offset);
      }
    }
    request->redirect("/");
  });

    // Route to submit wifi configuration
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
    for (int i =0; i< request->params();i++)
     {
       AsyncWebParameter* param = request->getParam(i);
      preferences.putString(param->name().c_str(), param->value().c_str());
    }
    request->redirect("/wificonfig.html");
  });


  server.onNotFound(notFound);
  server.begin();
  return true;
}

// starts the SPIFFS filesysem used to store web pages
bool SPIFFS_init()
{
  if (!SPIFFS.begin(true, "/data"))
  {
    showMessage("SPIFFS error", 1000);
    return false;
  }
  else
  {
    showMessage("SPIFFS init", 0);
    return true;
  }
}

// Starts the preferences storage used for calibration and set default values
bool preferences_init()
{
  preferences.begin("sensorhub", false);
  pres_offset = preferences.getDouble("pressure", 0);
  temp_offset = preferences.getDouble("temperature", 0);
  if (!preferences.isKey("ssid_ap")) preferences.putString("ssid_ap","sensorhub");
  if (!preferences.isKey("pw_ap")) preferences.putString("pw_ap","1234567890");
  return true;
}

// Shows pressure, temp and IP
void updateDisplay()
{
  static unsigned long DisplayUpdated = 0;
  if (DisplayUpdated + PressureUpdatePeriod < millis())
  {
    DisplayUpdated = millis();
    double pressure_calibrated = pressure + pres_offset;
    double temperature_calibrated = temperature + temp_offset;

    display->clearDisplay();
    char text[12];
    sprintf(text, "%7.2f mB", PascalTomBar(pressure_calibrated));
    showLine(text, 0);
    sprintf(text, "%3.2f C", temperature_calibrated);
    showLine(text, 1);
    showLine(localIP.toString(), 2);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Board id:");
  Serial.println(ESP.getEfuseMac());
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  SSD1306_init();
  BMP280_init();
  NMEA2000_init();
  SPIFFS_init();
  preferences_init();
  Wifi_init();
  Webserver_init();
  setPower();
}

void loop()
{
  sendPressure();
  updateDisplay();
  NMEA2000.ParseMessages();
}
