/**
* Connect a TFT-display (ST7789 driver, SPI interface) and BME280 sensor (I2C interface) with
* a Heltec Wireless Stick (Lite) V3 device.
*
* The device comes with an SX1262 LoRa module, mounted on an ESP32-S3 development board.
* The LoRa module is NOT used in this sketch !
*
* For Heltec Wireless Stick V3 only:
* Don't forget to change the Board in Arduino: Heltec Wireless Stick (V3) or - in Tools menu:
* Tools - Board - esp32 - Heltec Wireless Stick (V3)
* 
* For Heltec Wireless Stick Lite V3 only:
* Don't forget to change the Board in Arduino: Heltec Wireless Stick Lite(V3) or - in Tools menu:
* Tools - Board - esp32 - Heltec Wireless Stick Lite(V3)
*
* This is working for both deivces: Wireless Stick V3 and Wireless Stick Lite V3
*/

/*
Version Management
11.01.2026 V02 Tutorial version
10.01.2026 V01 Initial programming
*/

#define HELTEC_WIRELESS_STICK_V3
//#define HELTEC_WIRELESS_STICK_LITE_V3

#ifdef HELTEC_WIRELESS_STICK_V3
#include "Heltec_Wireless_Stick_V3_Hardware_Settings.h"
const char* PROGRAM_VERSION = "Heltec WS V3 ST7789 BME280 V02";
const char* PROGRAM_VERSION_1 = "  Heltec WS V3";
#endif

#ifdef HELTEC_WIRELESS_STICK_LITE_V3
#include "Heltec_Wireless_Stick_Lite_V3_Hardware_Settings.h"
const char* PROGRAM_VERSION = "Heltec WSL V3 ST7789 BME280 V02";
const char* PROGRAM_VERSION_1 = "  Heltec WSL V3";
#endif

#include <Arduino.h>
#include "FONT_MONOSPACE_9.h"

#if defined(IS_64_32_OLED)
#include <Wire.h>
#include "SSD1306.h"  // https://github.com/ThingPulse/esp8266-oled-ssd1306
// The small display gives 3 rows with 12 characters each
SSD1306Wire odisplay(OLED_I2C_ADDRESS, OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN, GEOMETRY_64_32);
#define DEFINED_64_32_OLED
#endif

String odisplay1 = "";
String odisplay2 = "";
String odisplay3 = "";

#ifdef IS_TFT_ST7789
// ------------------------------------------------------------------
// TFT display ST7789 2.0' 240 * 320 RGB
#include <SPI.h>
#include <Adafruit_GFX.h>     // Core graphics library, https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789, https://github.com/adafruit/Adafruit-ST7735-Library
#include <Fonts/FreeMono12pt7b.h>
SPIClass spi(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);  // Hardware SPI, fast
#endif

#define COLOR_SKYBLUE 0x867D  // 135, 206, 235

String display1 = "";
String display2 = "";
String display3 = "";
String display4 = "";
String display5 = "";
String display6, display7, display8, display9, display10, display11, display12, display13, display14, display15, display16;

// display messages
const char* PROGRAM_VERSION_2 = "ST7789 BME280 V02";
const char* DIVIDER = "-----------------";
const char* SENSOR_1 = "  BME280 data";
const char* CHIP_TEMP_1 = "  Internal Temp.";
const char* PRESS_BUTTON_1 = "  Press BOOT btn";
const char* PRESS_BUTTON_2 = "  for an update";

// -----------------------------------------------------------------------
// BME280 sensor

#ifdef IS_BME280
#include <Adafruit_Sensor.h>  // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C
#endif

float temperature = -99;
float humidity = -99;
float pressure = -99;
float altitude = -99;

void getBme280Values() {
#ifdef IS_BME280
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
#else
  // dummy values
  temperature = 29.12;
  humidity = 70.23;
  pressure = 1234.5;
#endif
}

void printBme280Values() {
  Serial.print(F("Temperature: "));
  Serial.print(temperature, 1);
  Serial.print(F("c, Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%, Pressure: "));
  Serial.print(pressure, 0);
  Serial.print(F("hPa, Altitude: "));
  Serial.print(altitude, 1);
  Serial.println(F("m"));
  Serial.flush();
}

// -----------------------------------------------------------------------
// PRG/Boot button
// #define BOOT_BUTTON_PIN 0 // see settings or hardware settings
boolean isBootButtonPressed = false;
uint8_t modeCounter = 0;  // just a counter

void IRAM_ATTR bootButtonPressed() {
  modeCounter++;
  isBootButtonPressed = true;
  // deactivate the interrupt to avoid bouncing
  detachInterrupt(BOOT_BUTTON_PIN);
}

// -----------------------------------------------------------------------
// Internal chip temperature

#if ESP_ARDUINO_VERSION_MAJOR >= 3
#include "driver/temperature_sensor.h"
#else
#include "driver/temp_sensor.h"
#endif
float internalTemperature = 0.0;

/**
 * @brief Measures esp32 chip temperature
 * 
 * @return float with temperature in degrees celsius.
*/
//float heltec_temperature() {
float readEsp32S3InternalTemperature() {
  float result = 0;
  // If temperature for given n below this value,
  // then this is the best measurement we have.
  int cutoffs[5] = { -30, -10, 80, 100, 2500 };
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  int range_start[] = { -40, -30, -10, 20, 50 };
  int range_end[] = { 20, 50, 80, 100, 125 };
  temperature_sensor_handle_t temp_handle = NULL;
  for (int n = 0; n < 5; n++) {
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(range_start[n], range_end[n]);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &result));
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_handle));
    if (result <= cutoffs[n]) break;
  }
#else
  // We start with the coldest range, because those temps get spoiled
  // the quickest by heat of processor waking up.
  temp_sensor_dac_offset_t offsets[5] = {
    TSENS_DAC_L4,  // (-40°C ~  20°C, err <3°C)
    TSENS_DAC_L3,  // (-30°C ~  50°C, err <2°C)
    TSENS_DAC_L2,  // (-10°C ~  80°C, err <1°C)
    TSENS_DAC_L1,  // ( 20°C ~ 100°C, err <2°C)
    TSENS_DAC_L0   // ( 50°C ~ 125°C, err <3°C)
  };
  for (int n = 0; n < 5; n++) {
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = offsets[n];
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    temp_sensor_read_celsius(&result);
    temp_sensor_stop();
    if (result <= cutoffs[n]) break;
  }
#endif
  return result;
}

void loop() {

  // the BOOT button was pressed
  if (isBootButtonPressed) {
    isBootButtonPressed = false;
    setDisplayData();
    displayColoredData();
    odisplayData();
    ledFlash(1, 125);
    // activate the interrupt again
    attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
  }
}

void ledFlash(uint16_t flashes, uint16_t delaymS) {
  // run only if a LED is connected
  if (LED_PIN >= 0) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
      digitalWrite(LED_PIN, HIGH);
      delay(delaymS);
      digitalWrite(LED_PIN, LOW);
      delay(delaymS);
    }
  }
}

void setDisplayBacklight(bool isDisplayOn) {
  if (isDisplayOn) {
    digitalWrite(TFT_BL, HIGH);
    Serial.println(F("The TFT display backlight is ON"));
  } else {
    digitalWrite(TFT_BL, LOW);
    Serial.println(F("The TFT display backlight is OFF"));
  }
}

void initDisplayBacklight() {
  if (TFT_BL >= 0) {
    pinMode(TFT_BL, OUTPUT);
  }
}

void displayColoredData() {
#if defined(IS_TFT_ST7789)
  const uint8_t DISTANCE = 20;
  const uint8_t OFFSET = 16;
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setFont(&FreeMono12pt7b);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(0, OFFSET);
  tft.print(display1);
  tft.setCursor(0, OFFSET + 1 * DISTANCE);
  tft.print(display2);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 2 * DISTANCE);
  tft.print(display3);
  tft.setCursor(0, OFFSET + 3 * DISTANCE);
  tft.print(display4);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 4 * DISTANCE);
  tft.print(display5);
  tft.setTextColor(COLOR_SKYBLUE, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 5 * DISTANCE);
  tft.print(display6);
  tft.setCursor(0, OFFSET + 6 * DISTANCE);
  tft.print(display7);
  tft.setCursor(0, OFFSET + 7 * DISTANCE);
  tft.print(display8);
  tft.setCursor(0, OFFSET + 8 * DISTANCE);
  tft.print(display9);
  tft.setCursor(0, OFFSET + 9 * DISTANCE);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(display10);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 10 * DISTANCE);
  tft.print(display11);
  tft.setCursor(0, OFFSET + 11 * DISTANCE);
  tft.print(display12);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 12 * DISTANCE);
  tft.print(display13);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 13 * DISTANCE);
  tft.print(display14);
  tft.setCursor(0, OFFSET + 14 * DISTANCE);
  tft.print(display15);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 15 * DISTANCE);
  tft.print(display16);
#endif
}

void odisplayData() {
#if defined(IS_64_32_OLED)
  odisplay.clear();
  odisplay.setColor(WHITE);
  odisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  odisplay.setFont(Monospaced_plain_9);
  odisplay.drawString(0, 0, odisplay1);
  odisplay.drawString(0, 10, odisplay2);
  odisplay.drawString(0, 20, odisplay3);
  odisplay.display();
#endif
}

void setDisplayData() {
  char buf[40];
  display1 = PROGRAM_VERSION_1;
  display2 = PROGRAM_VERSION_2;
  sprintf(buf, "IDF: %s", ESP.getSdkVersion());
  display3 = buf;
  sprintf(buf, "Arduino: %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  display4 = buf;
  display5 = DIVIDER;
  display6 = SENSOR_1;
  getBme280Values();
  printBme280Values();
  sprintf(buf, "Temp: %2.2f C", temperature);
  display7 = buf;
  sprintf(buf, "Hum:  %2.2f\%%", humidity);
  display8 = buf;
  sprintf(buf, "B.Press: %4.0f hg", pressure);
  display9 = buf;
  display10 = DIVIDER;
  internalTemperature = readEsp32S3InternalTemperature();
  display11 = CHIP_TEMP_1;
  sprintf(buf, "ITemp: %2.2f C", internalTemperature);
  display12 = buf;
  display13 = DIVIDER;
  display14 = PRESS_BUTTON_1;
  display15 = PRESS_BUTTON_2;
  display16 = DIVIDER;

  // data for oled display
  odisplay1 = "T:" + String(temperature) + " C";
  odisplay2 = "H:" + String(humidity) + " %";
  odisplay3 = "IT: " + String(internalTemperature) + " C";
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(PROGRAM_VERSION);

  // if we have a power control for devices put it on
#ifdef IS_VEXT_CONTROL
  setVextControl(true);
  // setVextControl(false); // if uncommented, the internal OLED display on the Heltec WS V3 board will not work
#endif

  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);  // setup pin as output for indicator LED
    ledFlash(2, 125);          // two quick LED flashes to indicate program start
  }
  delay(1000);

  // init TFT display
#ifdef IS_TFT_ST7789
  initDisplayBacklight();
  setDisplayBacklight(true);
  Serial.println("Init TFT ST7789");
  spi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.init(240, 320);            // Init ST7789 320x240
  tft.fillScreen(ST77XX_BLACK);  // fill screen with black = empty
  tft.setTextWrap(false);        // no automated line wrapping
#ifdef DISPLAY_ORIENTATION_FLIPPED
  tft.setRotation(0);  // Portrait 0 degrees right rotated
#else
  tft.setRotation(2);               // Portrait 180 degrees right rotated
#endif
  Serial.println(F("Display ST7789 init done"));
  delay(500);
#endif

  // setup display
#if defined(IS_64_32_OLED)
  if (OLED_I2C_RST_PIN >= 0) {
    pinMode(OLED_I2C_RST_PIN, OUTPUT);
    digitalWrite(OLED_I2C_RST_PIN, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(OLED_I2C_RST_PIN, HIGH);
    delay(50);
    Serial.println("Heltec OLED display init by RST pin done");
  }
  //clearDisplayData();
  odisplay.init();
  delay(500);
#ifdef DISPLAY_ORIENTATION_FLIPPED
  // do nothing
#else
  odisplay.flipScreenVertically();  // Landscape 90 degrees right rotated
#endif
  Serial.println(F("OLED display init is done"));
  delay(500);
#endif

  // init BME280 sensor
#ifdef IS_BME280
#ifndef IS_SECOND_I2C_BUS
  bool bme_status = bme.begin(BME280_I2C_ADDRESS);  // address either 0x76 or 0x77
  Serial.printf("Init BME280 on System I2C pins SDA: %d SCL: %d\n", SDA, SCL);
#else
  bool wireStatus = Wire1.begin(BME280_I2C_SDA_PIN, BME280_I2C_SCL_PIN);
  Serial.printf("Init Wire1/BME280 on Second I2C pins SDA: %d SCL: %d\n", BME280_I2C_SDA_PIN, BME280_I2C_SCL_PIN);
  if (!wireStatus) {
    Serial.println(F("Wire1 failed to init"));
    display3 = "Wire FAILURE";
    display4 = "System is halting";
    displayData();
    while (1)
      ;
  } else {
    display3 = "BME280/Wire1 ACTIVE";
    displayData();
    Serial.println(F("Wire1 initialized"));
  }
  bool bme_status = bme.begin(BME280_I2C_ADDRESS, &Wire1);  //address either 0x76 or 0x77
#endif
  if (!bme_status) {
    ledFlash(100, 15);  // long very fast speed flash indicates BME280 device error
    display4 = "BME280 not found";
    display5 = "System is halting";
    displayData();
    Serial.println(F("No valid BME280 found"));
    while (1)
      ;
  } else {
    display4 = "BME280 ACTIVE";
    displayData();
    Serial.println(F("BME280 found"));
  }
  // the preferred way to get correct indoor temperatures
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X16,  // temperature
                  Adafruit_BME280::SAMPLING_X1,   // pressure
                  Adafruit_BME280::SAMPLING_X1,   // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
#endif

  // init the mode select button
  pinMode(BOOT_BUTTON_PIN, INPUT);
  attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);

  setDisplayData();
  displayColoredData();
  odisplayData();
}

void displayData() {
#if defined(IS_TFT_ST7789)
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setFont(&FreeMono12pt7b);

  const uint8_t distance = 20;
  const uint8_t offset = 16;
  tft.setCursor(0, offset);
  tft.print(display1);
  tft.setCursor(0, offset + 1 * distance);
  tft.print(display2);
  tft.setCursor(0, offset + 2 * distance);
  tft.print(display3);
  tft.setCursor(0, offset + 3 * distance);
  tft.print(display4);
  tft.setCursor(0, offset + 4 * distance);
  tft.print(display5);
  tft.setCursor(0, offset + 5 * distance);
  tft.print(display6);
  tft.setCursor(0, offset + 6 * distance);
  tft.print(display7);
  tft.setCursor(0, offset + 7 * distance);
  tft.print(display8);
  tft.setCursor(0, offset + 8 * distance);
  tft.print(display9);
  tft.setCursor(0, offset + 9 * distance);
  tft.print(display10);
  tft.setCursor(0, offset + 10 * distance);
  tft.print(display11);
  tft.setCursor(0, offset + 11 * distance);
  tft.print(display12);
  tft.setCursor(0, offset + 12 * distance);
  tft.print(display13);
  tft.setCursor(0, offset + 13 * distance);
  tft.print(display14);
  tft.setCursor(0, offset + 14 * distance);
  tft.print(display15);
  tft.setCursor(0, offset + 15 * distance);
  tft.print(display16);
#endif
}

void clearDisplayData() {
  display1 = "";
  display2 = "";
  display3 = "";
  display4 = "";
  display5 = "";
  display6 = "";
  display7 = "";
  display8 = "";
  display9 = "";
  display10 = "";
  display11 = "";
  display12 = "";
  display13 = "";
  display14 = "";
  display15 = "";
  display16 = "";
}

void setVextControl(boolean trueIsOn) {
#ifdef IS_VEXT_CONTROL
  if (trueIsOn) {
    pinMode(VEXT_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(VEXT_POWER_CONTROL_PIN, LOW);
  } else {
    // pulled up, no need to drive it
    pinMode(VEXT_POWER_CONTROL_PIN, INPUT);
  }
#endif
}
