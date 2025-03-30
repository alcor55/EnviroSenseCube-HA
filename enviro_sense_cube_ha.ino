/*
 * EnviroSenseCube HA v1.0 - Environment monitor HomeAssistant edition by Vincenzo Rivelli.
 *
 * Features:
 * - Temperature, Relative humidity, Barometric pressure && Air Quality.
 * - Low power optimization && battery monitor.
 * - Data refresh threshold to save energy.
 * - E-Paper Display.
 * - Home Assistant update over WiFi.
 * 
 * Hardware:
 * - Adafruit Huzzah ESP8266 ( 3.3volt logic ).
 * - Adafruit PowerBoost 500 Charger.
 * - LiPo battery 1800mAh ( EEMB Cell ).
 * - Adafruit BME688 ( I2C address: Default 0x77, 0x76 with closed bridge ).
 * - Adafruit LC709203F LiPoly/LiIon Fuel Gauge. ( I2C address fixed: 0x0B ). Need the same power as the logic level of your microcontroller ( 3.3v ).
 * - Adafruit On-Off Power Button ADA1683 ( Power Switch ).
 * - Aceinnolab Universal E-Paper driver board for 24-pin SPI.
 * - GoodDisplay GDEW0097T50 E-Paper Mini 0.97inc 184x88 24pin SPI. Min.FULL upd.: 180sec(3min.), Min.PARTIAL upd.: 5sec. https://www.good-display.com/product/434.html https://www.good-display.com/news/80.html#precaution
 * - Acrilic Case.
 *
 * GPIO Pin map: #0, #2, #4, #5, #12, #13, #14, #15, #16.
 * - HUZZAH GPIO #16 Pin             -> HUZZAH GPIO Rst. Pin. ( For the DeepSleep awake ).
 * - HUZZAH TX Pin                   -> UART Cable WHITE.
 * - HUZZAH RX Pin                   -> UART Cable GREEN.
 * - HUZZAH V+ Pin                   -> UART Cable RED.
 * - HUZZAH GND Pin                  -> UART Cable BLACK.
 * - HUZZAH GPIO #13 Pin             -> E-Paper driver SPI SDI.
 * - HUZZAH GPIO #14 Pin             -> E-Paper driver SPI CLK.
 * - HUZZAH GPIO #2 Pin              -> E-Paper driver SPI CS.
 * - HUZZAH GPIO #15 Pin             -> E-Paper driver SPI DC.
 * - HUZZAH GPIO #0 Pin              -> E-Paper driver SPI RST.
 * - HUZZAH GPIO #12 Pin             -> E-Paper driver SPI BUSY.
 * - HUZZAH GPIO #4 Pin (i2c SDA)    -> BME688 SDI Pin    -> LC709203F SDA Pin (JST-SH Blue).
 * - HUZZAH GPIO #5 Pin (i2c SCL)    -> BME688 SCK Pin    -> LC709203F SCL Pin (JST-SH Yellow).
 * 
 * Power management:
 * - Charging time: 6 hours and 15 minutes. (Yellow LED (near pins): Charging; Green LED (near screw): Charge complete). 
 * - Autonomy: 11 days 4 hours and 40 minutes. ( SLEEP_TIME = 5m ).
 *
 * TODO:
 * - Graphics isssue.
 * - Battery percent isssue.
 * - Ad-Hoc wifi for configs.
 *
*/



// OTA.
#include <ArduinoOTA.h>

// WiFi.
#include <ESP8266WiFi.h>

// RTC Memory.
#include <EEPROM.h>

// For ThingSpeak.
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// i2c
#include <Wire.h>

// Sensors.
#include <Adafruit_Sensor.h>
#include "Adafruit_LC709203F.h"
#include "Adafruit_BME680.h"

// Utilities.
#include <math.h>

// E-Paper Display.
#include "GDEW0097T50_lib.h"
#include "epaper_img.h"
#include <Adafruit_GFX.h>

// Credentials & C. file.
#include "credentials.h"



// Debug.
bool DEBUG = false;

bool FIRST_RUN = false;

// For ThingSpeak.
WiFiClient client;
HTTPClient http;

// Deep Sleep time. 3 min (180e6) is the minimum SLEEP_TIME. Because is the min. update time of the EPD display.
//const unsigned long SLEEP_TIME = 300e6; // 5 minutes.
const unsigned long SLEEP_TIME = 240e6; // 4 minutes.
//const unsigned long SLEEP_TIME = 180e6; // 3 minutes.

// I2C Pin.
const uint8_t SDA_PIN = 4;
const uint8_t SCL_PIN = 5;

// On board Leds.
const uint8_t LED_RED_PIN = 0;
//const uint8_t LED_BLUE_PIN = 2;

// Sensor BME688.
Adafruit_BME680 bme;
float temperature, humidity, pressure, gas_resistance;

// Sensor LC709203F.
Adafruit_LC709203F lc;
//float battery_voltage;
uint8_t battery_avg;
const uint8_t battery_history_values = 8;

// Air Quality calc.
uint8_t air_quality;
const int     gas_resistance_min = 10000; // Bad air quality.
const int     gas_resistance_max = 70000; // Good air quality.
const uint8_t best_humidity = 50; // Optimal relative humidity %.

// Sensors compensations.
const float TEMPERATURE_OFFSET = -2.3; // C.
const float HUMIDITY_OFFSET    =    0; // %.
const float PRESSURE_OFFSET    =    0; // hPA.

// Sensors threshold.
const float TEMPERATURE_THRESHOLD = 1; // C.
const float HUMIDITY_THRESHOLD    = 2; // %.
const float PRESSURE_THRESHOLD    = 2; // hPA.
const float AIR_Q_THRESHOLD       = 5; // Arbitrary 1/100.

// E-Paper Display Pins. SPI Default pin (SPI.cpp for ESP8266 Arduino) SCK 14, MISO 12,MOSI 13.
const uint8_t BUSY_PIN = 12; // As Busy ( Unused Default MISO pin ).
const uint8_t RST_PIN  =  0; // Reset.
const uint8_t DC_PIN   = 15; // Data/Command.
const uint8_t CS_PIN   =  2; // SPI Chip Select (Slave Select) SS.
//const uint8_t CLK_PIN  = 14; // SPI Clock SPI SCK (Already declared in SPI.ccp for the ESP8266).
//const uint8_t SDI_PIN = 13; // SPI Serial Data In, Master Out Slave In (MOSI). (Already declared in SPI.ccp for the ESP8266).
//const uint8_t MISO_PIN = 12; // SPI Master In Slave Out (MISO). Unused. (Already declared in SPI.ccp for the ESP8266).

// E-Paper Display Sizes.
const uint8_t DISPLAY_W = 184;
const uint8_t DISPLAY_H = 88;

// Canvas.
GFXcanvas1 epd_canvas( DISPLAY_W, DISPLAY_H );
uint8_t* epd_byte_array;

// RTC Memory data struct.
const uint32_t MAGIC_NUMBER = 0xDEADBEEF; // Random unique identifier (only to check and init the struct).
const uint8_t history_values = 12; // One value per chart bar. (12 pixels width).
struct
{
  uint32_t magic_number;
  uint8_t  awake_count;
  float temp_history[history_values];
  float hum_history[history_values];
  float press_history[history_values];
  float air_q_history[history_values];
  float battery_history[battery_history_values];
} support_data;



//-----------------------
void write_support_data()
{
  if ( DEBUG ) Serial.println( "| Writing support_data in RTC memory." );

  // Manage the awake counter. 1 to 4 cyclically.
  support_data.awake_count = ( support_data.awake_count < 4 ) ? support_data.awake_count + 1 : 1;

  // Data write in the RTC memory.
  system_rtc_mem_write( 64, &support_data, sizeof( support_data ) );
}



//--------------------
void get_stored_data()
{
  if ( DEBUG ) Serial.print( "| Reading support_data in RTC memory." );

  system_rtc_mem_read( 64, &support_data, sizeof( support_data ) );

  if ( support_data.magic_number != MAGIC_NUMBER )
  {
    if ( DEBUG ) Serial.print( " ( First run, init stuffs )." );

    FIRST_RUN = true;
    // First boot, initialize struct.
    support_data.magic_number = MAGIC_NUMBER;
    support_data.awake_count = 1;

    // Initialize temperature history with zeros.
    for ( uint8_t i = 0; i < history_values; i++ )
    {
      support_data.temp_history[i] = 0;
      support_data.hum_history[i] = 0;
      support_data.press_history[i] = 0;
      support_data.air_q_history[i] = 0;
    }
  }
  else
    FIRST_RUN = false;
  
  if ( DEBUG ) Serial.println( " ( awake_count: " + String( support_data.awake_count ) + " )." );
}



//--------------------
void connect_to_wifi()
{
  WiFi.forceSleepWake();
  WiFi.persistent( false );
  WiFi.mode( WIFI_STA );
  WiFi.config( WiFi_IP, WiFi_GATEWAY, WiFi_SUBNET, WiFi_GATEWAY );
  WiFi.begin( WiFi_SSID, WiFi_PWD );

  if ( DEBUG ) Serial.print( "| WiFi connecting" );

  while ( WiFi.status() != WL_CONNECTED )
  {
    if ( DEBUG ) Serial.print( "." );
    delay( 500 );
  }
  if ( DEBUG ) Serial.println( " connected to " + String( WiFi_SSID ) + " ( IP: " + WiFi.localIP().toString() + " )." );
}



//--------------------
void switch_off_wifi()
{
  if ( DEBUG ) Serial.println("| WiFi disconnecting.");

  WiFi.disconnect(true);
  delay(100);

  // Switch OFF esp8266 WiFi.
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
}



//------------
void run_ota()
{
  Serial.println("| FIRST RUN: Waiting for OTA updates for 30 sec...");

  connect_to_wifi();
  pinMode( LED_RED_PIN, OUTPUT );

  // Config OTA credentials.
  ArduinoOTA.setHostname( WiFi_HOST );
  ArduinoOTA.setPassword( OTA_PWD );

  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started.");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();

  // Wait OTA update for 30sec.
  unsigned long startTime = millis();
  bool ledState = false;
  while (millis() - startTime < 30000)
  {
    ArduinoOTA.handle();

    // 1sec LED blink.
    if ( ( millis() - startTime ) % 1000 < 10 )
    {
      ledState = !ledState;
      digitalWrite(LED_RED_PIN, ledState);
    }
    delay(10);
  }
  // Turn LED off.
  digitalWrite(LED_RED_PIN, LOW);
  switch_off_wifi();

  Serial.println("| Modalità OTA terminata, proseguo con l'esecuzione normale.");
}



//-------------------------------------------
float round_float( float number, bool to_05 )
{
  if ( to_05 == true )
    return 0.5 * round( 2.0 * number ); // Round with 0.5 step.
  else
    return round( number ); // Round to a int.
}



//--------------------------
void update_home_assistant()
{
  if ( DEBUG ) Serial.print("| Home Assistant updating via webhook.");

  // // Compose Home Assistant webhook URL.
  String haWebhook = String("http://") + HOME_ASSISTANT_IP + String(":8123/api/webhook/envirocube_update_") + HOME_ASSISTANT_KEY;
  
  // Init HTTP connection.
  http.begin(client, haWebhook);
  http.addHeader("Content-Type", "application/json");

  // Prepara il payload JSON con i dati del sensore
  StaticJsonDocument<200> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["pressure"] = pressure;
  doc["gas_resistance"] = gas_resistance;
  doc["air_quality"] = air_quality;
  doc["battery"] = battery_avg;
  
  String payload;
  serializeJson(doc, payload);

  if ( DEBUG ) Serial.println(haWebhook);

  // Invia la richiesta POST
  int httpCode = http.POST(payload);
  if (httpCode == HTTP_CODE_OK)
  {
    if ( DEBUG ) Serial.println("| Home Assistant update: OK.");
  }
  else
    if ( DEBUG ) Serial.println("| Home Assistant update: ERROR " + String(httpCode));
  
  // Close connection.
  http.end();
}



//------------------------
void init_bme688_sensor()
{
  if ( DEBUG ) Serial.print( "| Initializing BME688 sensor." );

  if ( !bme.begin() )
  {
    Serial.println(" Error, not found!");
    while (1);
  }
  else
    Serial.println(" Ok.");

  // Set up oversampling and filter initialization.
  bme.setTemperatureOversampling(BME680_OS_4X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_8X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms // The sensor takes ~30-mins to fully stabilise.
}



//-----------------------
void read_bme688_sensor()
{
  if ( !bme.performReading() ) // This perform a "Force Mode" automatically.
  {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Raw data + compensation.
  temperature        = bme.temperature + TEMPERATURE_OFFSET; // Centigrade.
  humidity           = bme.humidity + HUMIDITY_OFFSET; // % RH.
  float raw_pressure = bme.pressure + PRESSURE_OFFSET; // Pa
  pressure           = raw_pressure / 100.0F; // Pascal (Pa) -> hectoPascal (hPa).
  gas_resistance     = bme.gas_resistance; // KOhms

  if ( DEBUG ) Serial.println( "| Reading BME688 ( Temp: " + String( temperature ) + " C - Press: " + String( pressure ) + " Pa - Hum: " + String( humidity ) + " % - Hum: " + String( gas_resistance ) + " KOHm )." );
}



//--------------------------
void calculate_air_quality()
{
  const int8_t max_score = 100;
  const float weight_humidity = 25;
  const float weight_gas = 75;

  // Calc humidity score.
  int deviation = abs(humidity - best_humidity);
  int humidity_score = weight_humidity - (deviation * weight_humidity / best_humidity);
  if (humidity_score < 0)
    humidity_score = 0;

  // Calc gas score.
  int gas_score = (gas_resistance - gas_resistance_min) * weight_gas / (gas_resistance_max - gas_resistance_min);
  if (gas_score < 0)
    gas_score = 0;
  else if (gas_score > weight_gas)
    gas_score = weight_gas;

  // Calc air quality index.
  air_quality = humidity_score + gas_score;

  if ( DEBUG ) Serial.println( "| Calc. Air Quality. ( Hum. Score: " + String( humidity_score ) + " - Gas Score: " + String( gas_score ) + " - Air Quality: " + String( air_quality ) + " )." );
}



//-------------------------------
void update_temperature_history()
{
  if ( DEBUG ) Serial.println( "| Updating sensors history." );

  // Shift temperatures to the left.
  for ( uint8_t i = 0; i < history_values-1; i++ )
  {
    support_data.temp_history[i] = support_data.temp_history[i + 1];
    support_data.hum_history[i] = support_data.hum_history[i + 1];
    support_data.press_history[i] = support_data.press_history[i + 1];
    support_data.air_q_history[i] = support_data.air_q_history[i + 1];
  }

  // Add new temperature at the end.
  support_data.temp_history[history_values-1] = round_float( temperature, true );
  support_data.hum_history[history_values-1] = round_float( humidity, true );
  support_data.press_history[history_values-1] = round_float( pressure, true );
  support_data.air_q_history[history_values-1] = round_float( air_quality, true );
}



//-------------------------------
bool threshold_exceeded()
{
  if ( DEBUG ) Serial.print( "| Check if values exceed the thresholds. " );

  if ( FIRST_RUN == true )
  {
    Serial.println( "FIRST_RUN." );
    return true;
  }

  const float temp_diff = abs( support_data.temp_history[history_values-1] - support_data.temp_history[history_values-2] );
  const float hum_diff = abs( support_data.hum_history[history_values-1] - support_data.hum_history[history_values-2] );
  const float press_diff = abs( support_data.press_history[history_values-1] - support_data.press_history[history_values-2] );
  const float air_q_diff = abs( support_data.air_q_history[history_values-1] - support_data.air_q_history[history_values-2] );

  if ( DEBUG ) Serial.printf(" (temp_diff: %.2f, hum_diff: %.2f, press_diff: %.2f, air_q_diff: %.2f).\n", temp_diff, hum_diff, press_diff, air_q_diff);

  return (temp_diff > TEMPERATURE_THRESHOLD ||
          hum_diff > HUMIDITY_THRESHOLD || 
          press_diff > PRESSURE_THRESHOLD || 
          air_q_diff > AIR_Q_THRESHOLD );
}



//---------------------------
void init_lc709203f_sensor()
{
  if ( DEBUG ) Serial.print( "| Initializing LC709203F sensor." );
  
  if ( !lc.begin() )
  {
    if ( DEBUG ) Serial.println(" Error, not found!");
    while (1) delay(10);
  }
  else
    if ( DEBUG ) Serial.print( " Ok (V.0x"); Serial.print( lc.getICversion(), HEX ); Serial.println( ").");

  //lc.setPackSize( LC709203F_APA_2000MAH ); // Approx.
  lc.setPackAPA( 0x2A ); // APA Type 1-3, 42 -> 0x2A Figure.13 LC709203F Datasheet.
  lc.setThermistorB(3950); // Internal thermistor.
  lc.setAlarmVoltage( 0 ); // Need the 'INT' pin connected. Set '3.8' or '0' to disable the alarm.
  lc.setPowerMode( LC709203F_POWER_SLEEP ); // Set sleep mode.
}



//--------------------------
void read_lc709203f_sensor()
{
  if ( DEBUG ) Serial.print( "| Reading LC709203F" );

  // Read sensor.
  //delay(2000); // Don't read too often. (unused with esp deepsleep).
  lc.setPowerMode( LC709203F_POWER_OPERATE ); // Set operative mode.
  float actual_batt_percent = lc.cellPercent();
  //raw_batt_voltage = lc.cellVoltage();
  lc.setPowerMode( LC709203F_POWER_SLEEP ); // Set sleep mode.

  if ( FIRST_RUN == true )
  {
    // Initialize battery history with the first read.
    for ( uint8_t i = 0; i < battery_history_values; i++ )
      support_data.battery_history[i] = actual_batt_percent;
  }
  else
  {
    // Shift readings to the left.
    for ( uint8_t i = 0; i < battery_history_values-1; i++ )
      support_data.battery_history[i] = support_data.battery_history[i + 1];
  }
  support_data.battery_history[battery_history_values-1] = actual_batt_percent; // Update history last value.

  // Calc the battery moving average filter. The sensor reads a lot of outlier (probably caused by the bme688 burn-in), when reading occured during load peaks.
  uint16_t history_sum = 0;
  for ( uint8_t i = 0; i < battery_history_values; i++ )
    history_sum += support_data.battery_history[i];
  
  battery_avg = round_float( history_sum / battery_history_values, true );

  if ( DEBUG ) Serial.println( " Actual: " + String( actual_batt_percent ) + "%  Floating AVG: " + String( battery_avg ) + "% )." );
}



//-------------------
void draw_display_canvas()
{
  // Min. FULL update: 180sec (3min.) | Min. PARTIAL update: 5sec.
  if ( DEBUG ) Serial.println( "| Drawing Canvas." );

  // Prepare Canvas
  epd_canvas.setTextColor( 0 ); // 0:Black 1:White.
  epd_canvas.setFont();
  epd_canvas.setTextSize(5);
  epd_canvas.fillScreen( 1 ); // 0:Black 1:White. // Set canvas background.

  // Draw Temperature label.
  int16_t temperature_val = round_float( temperature, false );
  const uint8_t temp_label_xy[2] = { 26, 3 }; // X, Y. (top-right corner).
  epd_canvas.setCursor( temp_label_xy[0], temp_label_xy[1] );
  epd_canvas.print( temperature_val );

  // Draw Humidity label.
  int16_t humidity_val = round_float( humidity, false );
  const uint8_t hum_label_xy[2] = { 26, 50 }; // X, Y. (top-right corner).
  epd_canvas.setCursor( hum_label_xy[0], hum_label_xy[1] );
  epd_canvas.print( humidity_val );

  // Draw Pressure label.
  int16_t pressure_val = round_float( pressure, false );
  int16_t start_pressure_val = pressure_val % 100; // First 1-2 digit of pressure_val.
  int16_t end_pressure_val = pressure_val / 100; // Last 2 digit of pressure_val.
  const uint8_t press_label_xy[4] = { 116, 3, 95, 19 }; // 1st. 2 digits X, Y, 2nd. digits X, Y. (top-right corner).
  epd_canvas.setCursor( press_label_xy[0], press_label_xy[1] );
  epd_canvas.print( start_pressure_val < 10 ? "0" + String(start_pressure_val) : String(start_pressure_val) );
  epd_canvas.setTextSize(1);
  epd_canvas.setCursor( press_label_xy[2], press_label_xy[3] );
  epd_canvas.print( end_pressure_val < 10 ? "0" + String(end_pressure_val) : String(end_pressure_val) );  
  epd_canvas.setTextSize(5);

  // Draw Air Quality label.
  int16_t air_quality_val = round_float( air_quality, false );
  const uint8_t air_q_label_xy[2] = { 116, 50 }; // X, Y. (top-right corner).
  epd_canvas.setCursor( air_q_label_xy[0], air_q_label_xy[1] );
  epd_canvas.print( air_quality_val );

  // Draw Charts.  (X and Y at top-right corner).
  const uint8_t temp_chart_xyhw[4]  = {   5, 28, 9, 12 }; // X, Y, H, W.
  draw_canvas_pixel_chart( support_data.temp_history,  sizeof(support_data.temp_history)/sizeof(support_data.temp_history[0]), temp_chart_xyhw );
  const uint8_t hum_chart_xyhw[4]   = {   5, 73, 9, 12 }; // X, Y, H, W.
  draw_canvas_pixel_chart( support_data.hum_history,   sizeof(support_data.hum_history)/sizeof(support_data.hum_history[0]), hum_chart_xyhw );
  const uint8_t press_chart_xyhw[4] = {  95, 28, 9, 12 }; // X, Y, H, W.
  draw_canvas_pixel_chart( support_data.press_history, sizeof(support_data.press_history)/sizeof(support_data.press_history[0]), press_chart_xyhw );
  const uint8_t air_q_chart_xyhw[4] = {  95, 73, 9, 12 }; // X, Y, H, W.
  draw_canvas_pixel_chart( support_data.air_q_history, sizeof(support_data.air_q_history)/sizeof(support_data.air_q_history[0]), air_q_chart_xyhw );

  // Draw Battery.
  const uint8_t batt_chart_xyhw[4]  = { 181, 1, 87, 4 }; // X, Y, H, W.
  int16_t bar_height = ( battery_avg * batt_chart_xyhw[2] ) / 100;
  epd_canvas.fillRect( batt_chart_xyhw[0], batt_chart_xyhw[2]-bar_height+1, batt_chart_xyhw[3], bar_height, 0 ); // Black bar. (Used energy). 0:Black 1:White.
  
  // Convert the GFXcanvas1 in a byte array following the display spec. See the GoodDisplay GDEW0097T50 manual.
  canvas_to_byte_array( epd_canvas );
}



//-------------------------------------------------------------------------------------------
void draw_canvas_pixel_chart( float values[], uint8_t values_count, const uint8_t xyhw[] )
{
  // Coordinates.
  int16_t x = xyhw[0];
  int16_t y = xyhw[1];
  int16_t h = xyhw[2];
  int16_t w = xyhw[3];
  const int16_t min_h = 1; // min pix height
  const int16_t pixel_range = h - min_h;
  
  // Values.
  int16_t value_max = *std::max_element(values, values + values_count);
  int16_t value_min = *std::min_element(values, values + values_count);
  int16_t values_range = value_max - value_min;
  if (values_range == 0)
    values_range = 1;

  // Draw.
  int16_t cycle = 0;
  for (int8_t i = w-1; i >= 0; --i)
  {
    ++cycle;
    int16_t index = values_count-cycle;
    int16_t value = index >= 0 ? values[index] : 1;
    int16_t bar_height = min_h + ( ( value - value_min ) * pixel_range ) / values_range;

    epd_canvas.drawLine( x+i, y+h-1, x+i, y+h-bar_height, 0 ); // Black bar. ( x0, y0, x1, y1, color)
  }
}



//-------------------------------------------------
// Convert the GFXcanvas1 in a byte array following the display spec. See the GoodDisplay GDEW0097T50 manual. (Mimic Image2Display.exe)
void canvas_to_byte_array(const GFXcanvas1& canvas)
{
  // Invert horizontal.
  for (uint8_t y = 0; y < DISPLAY_H; y++) {
    for (uint8_t x = 0; x < DISPLAY_W / 2; x++) {
      uint8_t temp = epd_canvas.getPixel(DISPLAY_W - x - 1, y);  
      epd_canvas.drawPixel(DISPLAY_W - x - 1, y, epd_canvas.getPixel(x, y));  
      epd_canvas.drawPixel(x, y, temp);  
    }
  }

  size_t byteArraySize = (DISPLAY_W * DISPLAY_H) / 8; // Calculate the size of the necessary byte array.
  epd_byte_array = (uint8_t*)malloc(byteArraySize); // Allocate memory for the byte array.

  if (epd_byte_array == nullptr)
    if ( DEBUG ) Serial.println( "| Bytes conversion memory Error." );

  memset( epd_byte_array, 0, byteArraySize ); // Initialize the array with byte at zero.
  uint8_t byteIndex = 0; // Initialize the index of the byte array.

  // Iterate through each pixel of the canvas.
  for (uint8_t x = 0; x < DISPLAY_W; x++)
  {
    for (uint8_t y = 0; y < DISPLAY_H; y++)
    {
      int bytePos = (x * DISPLAY_H + y) / 8; // Calculate the index of the byte corresponding to the pixel.
      int bitPos = 7 - (y % 8); // Calculate the index of the bit within the byte.
      uint8_t pixelColor = epd_canvas.getPixel(x, y); // Read the color of the pixel from the canvas.
      epd_byte_array[bytePos] |= (pixelColor ? (1 << bitPos) : 0); // Set the corresponding bit in the byte of the array.
    }
  }
}



//-----------------
void init_display()
{ // Min. FULL update: 180sec (3min.) | Min. PARTIAL update: 5sec.
  if ( DEBUG ) Serial.println( "| Initializing E-Paper display." );

  // Set pins for SPI.
  pinMode( RST_PIN, OUTPUT );
  pinMode( DC_PIN, OUTPUT);  
  pinMode( CS_PIN, OUTPUT);  

  // Start SPI.
  SPI.beginTransaction( SPISettings( 10000000, MSBFIRST, SPI_MODE0 ) ); 
  SPI.begin();

  // Force unused SPI_MISO pin as SPI_BUSY.
  pinMode( BUSY_PIN, INPUT );
}



//-----------------
void clear_display()
{ // Min. FULL update: 180sec (3min.) | Min. PARTIAL update: 5sec.
  if ( DEBUG ) Serial.println( "| Cleaning E-Paper. ( Every 4 update_display )." );

  // Clear display.
  EPD_Init();
  EPD_WhiteScreen_White();
  EPD_DeepSleep();
  delay(2000); // DeepSleep + delay, Otherwise it will reduce the lifespan of the screen.
}



//-------------------
void update_display()
{ // Min. FULL update: 180sec (3min.) | Min. PARTIAL update: 5sec.

  // Init ePaper.
  init_display();

  // Clear the display every 4 update_display.
  if ( support_data.awake_count == 1 )
    clear_display();

  if ( DEBUG ) Serial.println( "| Updating E-Paper." );
  
  // Perform a partial update.
  EPD_Init();
  EPD_SetRAMValue_BaseMap( display_background ); // Set background function, otherwise it will cause unstable display during partial refresh.
  EPD_Dis_Part( 0, 0, epd_byte_array, DISPLAY_W, DISPLAY_H ); // Update the display (Partial).
  EPD_DeepSleep();
  delay(2000); // DeepSleep + delay, Otherwise it will reduce the lifespan of the screen.

  // Free epd_byte_array malloc.
  free( epd_byte_array );
}



//----------
void setup()
{
  // Init serial console.
  if ( DEBUG )
  {
    Serial.begin( 115200 );
    Serial.setTimeout( 2000 );
    while( !Serial ){}  // Wait for serial to initialize.
    delay( 100 );
    Serial.println( "\n\n\n| Initializing Serial console." );
  }

  // Retrive stuff from previous run. 
  get_stored_data();

  // Run OTA mode on the first run.
  if ( FIRST_RUN == true )
    run_ota();

  // Init i2c.
  Wire.begin( SDA_PIN, SCL_PIN );
  
  // Init sensor.
  init_bme688_sensor();
  init_lc709203f_sensor();
}



//---------
void loop()
{
  // Read sensors.
  read_bme688_sensor();
  read_lc709203f_sensor();

  // Sensors postprocessing.
  calculate_air_quality();
  update_temperature_history();

  // Check if the environment value
  if ( threshold_exceeded() )
  {
    // Update Thingspeak.
    connect_to_wifi();
    update_home_assistant();
    switch_off_wifi();

    // Update Display.
    draw_display_canvas();
    update_display();
  }

  // Store current run stuff.
  write_support_data();

  // ESP8266 deep-sleep mode.
  if ( DEBUG ) Serial.println( "| Entering into deep-sleep for " + String( ( SLEEP_TIME / 1000000 ) / 60 ) + " min.\n" );
  ESP.deepSleep( SLEEP_TIME, WAKE_RF_DISABLED ); // WAKE_RF_DISABLED to keep the WiFi radio disabled when it wakes up.
}



/*
    That's all folks!
*/
