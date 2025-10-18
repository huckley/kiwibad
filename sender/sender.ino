#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HT_lCMEN2R13EFC1.h"
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <HardwareSerial.h>
#include "HT_TinyGPS++.h"
#include "HT_E0213A367.h"
#include "driver/board-config.h"
#include "Monospaced_bold_50.h"
#include "esp_bt.h"                 // ESP32 Bluetooth control (power saving)
#include "esp_wifi.h"               // ESP32 WiFi control (power saving)

ScreenDisplay *epaper_display;
#include "battery.h"

#include <time.h>
#include "../lora-settings.h"
#include "../secret.h"

// ===============================
// ONE WIRE Configuration
// ===============================

#define ONE_WIRE_BUS 17
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define MAX_SENSORS 2
DeviceAddress sensorAddresses[MAX_SENSORS];
int deviceCount = 0;
float airTempC = 0;
float waterTempC = 0;

DeviceAddress air_sensor_addr   = { 0x28, 0xCE, 0xC8, 0x37, 0x00, 0x00, 0x00, 0x40 };
DeviceAddress water_sensor_addr = { 0x28, 0xD7, 0xB9, 0xB3, 0x00, 0x00, 0x00, 0xFA };

// ===============================
// LoWAN Configuration
// ===============================
/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC8 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x12, 0xD5, 0x0E, 0xEF, 0x28, 0xB3, 0x28, 0xFC, 0x0E, 0xCB, 0x13, 0x80, 0x1A, 0x1A, 0xA8, 0xB6 };
/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000*4*2;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

#define LED  45
#define PIN_EINK_SCLK 4
#define PIN_EINK_DC   2
#define PIN_EINK_CS   5
#define PIN_EINK_RES  3
#define PIN_EINK_MOSI 6

#define SLEEP_TIME 60              // Sleep time in secounds
#define GPS_RX 44
#define GPS_TX 43
#define GPS_BAUD 9600
#define GPS_TIMEOUT_MS 180000                 // Max wait for GPS (3 min)
#define TZ_INFO "CET-1CEST,M3.5.0/2,M10.5.0/3" // Timezone string
TinyGPSPlus GPS;

RTC_DATA_ATTR int64_t epoch_base = 0;              // Full 64-bit epoch time
RTC_DATA_ATTR bool send_on_lora = true;

static void prepareTxFrame(uint8_t port, float airTemp,float waterTemp, float batteryVoltage) {
  int16_t airTempInt = (int16_t)(airTemp * 100);        // 2 bytes
  int16_t waterTempInt = (int16_t)(waterTemp * 100);    // 2 bytes
  uint16_t battVoltInt = (uint16_t)(batteryVoltage * 1000); // 2 bytes

  appDataSize = 6; // 2 + 2 + 2 = 6 bytes

  appData[0] = (airTempInt >> 8) & 0xFF;
  appData[1] = airTempInt & 0xFF;

  appData[2] = (waterTempInt >> 8) & 0xFF;
  appData[3] = waterTempInt & 0xFF;

  appData[4] = (battVoltInt >> 8) & 0xFF;
  appData[5] = battVoltInt & 0xFF;
}


void VextON(void) {
  pinMode(18,OUTPUT);
  digitalWrite(18, HIGH);
  pinMode(46,OUTPUT);
  digitalWrite(46, HIGH);
}

void VextOFF(void) {
  pinMode(18,OUTPUT);
  digitalWrite(18, LOW);
  pinMode(46,OUTPUT);
  digitalWrite(46, LOW);
}

void enterDeepSleepForSecounds(uint32_t secounds) {
  VextOFF ();

  // Prepare peripherals
  Radio.Sleep();         // Put LoRa radio to sleep
  SPI.end();             // End SPI to save power

  // Set other unused pins to analog to minimize leakage
  pinMode(14, ANALOG);
  pinMode(12, ANALOG);
  pinMode(13, ANALOG);
  pinMode(9, ANALOG);
  pinMode(11, ANALOG);
  pinMode(10, ANALOG);

  struct timeval now;
  gettimeofday(&now, nullptr);         // get current system time
  epoch_base = (int64_t)now.tv_sec;    // save seconds part to RTC memory
  Serial.printf("Saved time to RTC: %lld\n", epoch_base);
  Serial.printf("going to sleep for %u sec...\n", secounds);
  // Enable wake-up timer
  Serial.flush();
  //send_on_lora = !send_on_lora;
  esp_sleep_enable_timer_wakeup((uint64_t)secounds  * 1000000ULL);
  // Enter deep sleep
  esp_deep_sleep_start();
}

void init_display () {
  pinMode(PIN_EINK_SCLK, OUTPUT); 
  pinMode(PIN_EINK_DC, OUTPUT); 
  pinMode(PIN_EINK_CS, OUTPUT);
  pinMode(PIN_EINK_RES, OUTPUT);
    
  //rest e-ink
  digitalWrite(PIN_EINK_RES, LOW);
  delay(20);
  digitalWrite(PIN_EINK_RES, HIGH);
  delay(20);

  digitalWrite(PIN_EINK_DC, LOW);
  digitalWrite(PIN_EINK_CS, LOW);

  // write cmd
  uint8_t cmd = 0x2F;
  pinMode(PIN_EINK_MOSI, OUTPUT);  
  digitalWrite(PIN_EINK_SCLK, LOW);
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_EINK_MOSI, (cmd & 0x80) ? HIGH : LOW);
    cmd <<= 1;
    digitalWrite(PIN_EINK_SCLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_EINK_SCLK, LOW);
    delayMicroseconds(1);
  }
  delay(10);

  digitalWrite(PIN_EINK_DC, HIGH);
  pinMode(PIN_EINK_MOSI, INPUT_PULLUP); 

  // read chip ID
  uint8_t chipId = 0;
  for (int8_t b = 7; b >= 0; b--) {
    digitalWrite(PIN_EINK_SCLK, LOW);  
    delayMicroseconds(1);
    digitalWrite(PIN_EINK_SCLK, HIGH);
    delayMicroseconds(1);
    if (digitalRead(PIN_EINK_MOSI)) chipId |= (1 << b);  
  }
  digitalWrite(PIN_EINK_CS, HIGH);
  if((chipId &0x03) !=0x01) {
    epaper_display = new HT_ICMEN2R13EFC1(3, 2, 5, 1, 4, 6, -1, 6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
  } else {
    epaper_display = new HT_E0213A367(3, 2, 5, 1, 4, 6, -1, 6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
  }
  epaper_display->init();
  epaper_display->setFont(ArialMT_Plain_10);
  epaper_display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void Navigation_bar(float airtempC) {
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%H:%M:%S  %d.%m.%Y", &timeinfo);
  epaper_display->setTextAlignment(TEXT_ALIGN_LEFT);
  epaper_display->setFont(ArialMT_Plain_10);
  epaper_display->drawLine(0, 15, 250, 15);
  String line_one = "Luft: " + String(airtempC, 2) + "°C | " + buffer;
  Serial.println(line_one);
  epaper_display->drawString(0, 0, line_one);
  battery();
}

void update_display() {
  epaper_display->update(BLACK_BUFFER);
  epaper_display->display();
}

void flash_led(){
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);  // LED on for 200 ms
  digitalWrite(LED, LOW);
}

void synctime_from_gps(){
  Serial.println("🛰️  Syncing time from GPS...");
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  unsigned long start = millis();
  bool synced = false;

  while (millis() - start < GPS_TIMEOUT_MS) {
    while (Serial1.available() > 0) {

      GPS.encode(Serial1.read());
        if (GPS.time.isUpdated() && GPS.date.isUpdated() && GPS.time.isValid() && GPS.date.isValid() ) {
          struct tm t;
          t.tm_year = GPS.date.year() - 1900;
          t.tm_mon  = GPS.date.month() - 1;
          t.tm_mday = GPS.date.day();
          t.tm_hour = GPS.time.hour();
          t.tm_min  = GPS.time.minute();
          t.tm_sec  = GPS.time.second();
          t.tm_isdst = -1;

				Serial.printf("%02d.%02d.%04d %02d:%02d:%02d.%02d",GPS.date.day(),GPS.date.month(),GPS.date.year(),GPS.time.hour(),GPS.time.minute(),GPS.time.second(),GPS.time.centisecond());
        Serial.print("LAT: ");
        Serial.print(GPS.location.lat(), 6);
        Serial.print(", LON: ");
        Serial.print(GPS.location.lng(), 6);
        Serial.println();
        
          time_t gpsEpoch = mktime(&t);
          if (gpsEpoch > 0) {
            struct timeval tv = { .tv_sec = gpsEpoch };
            settimeofday(&tv, nullptr);
            epoch_base = (int64_t)gpsEpoch;
            Serial.println("✅ GPS time synced:");
            synced = true;
            break;
          }
        }
    }
    if (synced) break;
  }
  if (!synced) {
      Serial.println(GPS.charsProcessed());
      Serial.println("⚠️ GPS sync failed. Continuing with estimated time.");
  }
}

void setup() {
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  setCpuFrequencyMhz(80);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
 
 // Set timezone
  setenv("TZ", TZ_INFO, 1);
  tzset();

  Serial.begin(115200);
  VextON();
  
  if (epoch_base > 0) {
    time_t restored = (time_t)(epoch_base + SLEEP_TIME); // Add sleep duration
    struct timeval now = { .tv_sec = restored };
    settimeofday(&now, nullptr);
    Serial.println("✅ Restored time from RTC:");
  } else {
    Serial.println("❌ No RTC time available.");
    struct timeval now { .tv_sec = epoch_base }; 
    settimeofday(&now, nullptr);
   }

  // Disable Bluetooth and WiFi completely
  btStop();
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_wifi_deinit();

  flash_led();
  Serial.println("init >>> ");
  init_display();

  char buffer[32];
  struct tm timeinfo;
  strftime(buffer, sizeof(buffer), "%H:%M:%S  %d.%m.%Y", &timeinfo);
  Serial.println(buffer);
  if (timeinfo.tm_min < 5) {
    synctime_from_gps();
  }
  strftime(buffer, sizeof(buffer), "%H:%M:%S  %d.%m.%Y", &timeinfo);
  Serial.println(buffer);

  sensors.begin();

  deviceCount = sensors.getDeviceCount();
  Serial.print("Gefundene Sensoren: ");
  Serial.println(deviceCount);
  for (int i = 0; i < deviceCount && i < MAX_SENSORS; i++) {
    if (sensors.getAddress(sensorAddresses[i], i)) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" Adresse: ");
      printAddress(sensorAddresses[i]);
      sensors.setResolution(sensorAddresses[i], TEMPERATURE_PRECISION);
    } else {
      Serial.println("Sensor " + String(i) + " hat keine gültige Adresse.");
    }
  }
  sensors.requestTemperatures();
  airTempC = sensors.getTempC(air_sensor_addr);
  waterTempC = sensors.getTempC(water_sensor_addr);
  Serial.print("Send mode: ");
  Serial.println(send_on_lora ? "LoRa (raw)" : "LoRaWAN (TTN)");
  if (send_on_lora) {  
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0,
                    LORA_IQ_INVERSION_ON, 3000); // 3000 ms timeout
  }
  epaper_display->clear();
  Navigation_bar(airTempC);
  epaper_display->setTextAlignment(TEXT_ALIGN_CENTER);
  epaper_display->setFont(Monospaced_bold_50);
  String tempStr = String(waterTempC, 0) + "°C";
  Serial.println(tempStr);
  epaper_display->drawString(125, 40, tempStr);
  update_display();
  sendLoRaWithTempsAndBattery(airTempC, waterTempC, batteryVoltage);
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

void sendLoRaWithTempsAndBattery(float airTemp,float waterTemp, float batteryVoltage) {
  // Format sensor addresses as hex
  char airAddrStr[17];
  char waterAddrStr[17];

  snprintf(airAddrStr, sizeof(airAddrStr),
    "%02X%02X%02X%02X%02X%02X%02X%02X",
    air_sensor_addr[0], air_sensor_addr[1], air_sensor_addr[2], air_sensor_addr[3],
    air_sensor_addr[4], air_sensor_addr[5], air_sensor_addr[6], air_sensor_addr[7]);

  snprintf(waterAddrStr, sizeof(waterAddrStr),
    "%02X%02X%02X%02X%02X%02X%02X%02X",
    water_sensor_addr[0], water_sensor_addr[1], water_sensor_addr[2], water_sensor_addr[3],
    water_sensor_addr[4], water_sensor_addr[5], water_sensor_addr[6], water_sensor_addr[7]);

  // Prepare LoRa payload
  snprintf(packet, BUFFER_SIZE,
    "BAT:%.2fV|T1:%.1fC[%s]|T2:%.1fC[%s]",
    batteryVoltage, airTemp, airAddrStr, waterTemp, waterAddrStr);
 
  char sigHex[17];  // 8 bytes * 2 + null terminator
  if (!computeHMACSignature(packet, sigHex, sizeof(sigHex))) {
    Serial.println("HMAC calculation failed");
    return;
  }

  strncat(packet, "|SIG:", BUFFER_SIZE - strlen(packet) - 1);
  strncat(packet, sigHex, BUFFER_SIZE - strlen(packet) - 1);

  Serial.println("Sending LoRa message:");
  Serial.println(packet);
  // Send packet
  if (send_on_lora) {
    Radio.Send((uint8_t *)packet, strlen(packet));
  }
}

void printDevEUI() {
  extern uint8_t devEui[8];  // Declared in the Heltec library
  Serial.print("Generated DevEUI: ");
  for (int i = 0; i < 8; i++) {
    if (devEui[i] < 0x10) Serial.print("0"); // Leading zero for single-digit values
    Serial.print(devEui[i], HEX);
  }
  Serial.println();
}

void loop() {
  if (send_on_lora) {
    Radio.IrqProcess();
  } else {
    switch (deviceState) {
      case DEVICE_STATE_INIT: {
#if(LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        printDevEUI();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        //both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
      case DEVICE_STATE_JOIN: {
          Serial.println("=== DEVICE_STATE_JOIN: starting OTAA join");
          LoRaWAN.join();
          Serial.println("LoRaWAN.join() called");
          break;
      }
      case DEVICE_STATE_SEND: {
          prepareTxFrame(appPort,airTempC,waterTempC,batteryVoltage);
          LoRaWAN.send();
          deviceState = DEVICE_STATE_CYCLE;
          break;
      }
      case DEVICE_STATE_CYCLE: {
        enterDeepSleepForSecounds(SLEEP_TIME);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
      case DEVICE_STATE_SLEEP:{
        LoRaWAN.sleep(loraWanClass);
        break;
      }
      default:{
        deviceState = DEVICE_STATE_INIT;
        break;
      }
    }
  }
}

void OnJoinRejected(void) {
  Serial.println("OTAA Join rejected!");
  // Retry or handle join failure
  deviceState = DEVICE_STATE_CYCLE; // Try join again or reset
}

void OnJoinAccepted(void) {
  Serial.println("OTAA Join accepted!");
  deviceState = DEVICE_STATE_SEND; // Move to send state
}
void OnTxDone(void) {
  Serial.println("txdone");
  enterDeepSleepForSecounds(SLEEP_TIME);
}

void OnTxTimeout(void) {
  Serial.println("txtimeout");
  enterDeepSleepForSecounds(SLEEP_TIME);
}