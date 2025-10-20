#include <time.h>
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
#include "../secret.h"
#include "../lora.h"

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
uint32_t joinStartTime = 0;

#define LED  45
#define PIN_EINK_SCLK 4
#define PIN_EINK_DC   2
#define PIN_EINK_CS   5
#define PIN_EINK_RES  3
#define PIN_EINK_MOSI 6

#define SLEEP_TIME 300              // Sleep time in secounds
#define GPS_RX 43  // GPS TX -> ESP32 RX pin (GPIO 16)
#define GPS_TX 44  // GPS RX -> ESP32 TX pin (GPIO 17)
#define GPS_BAUD 9600
#define GPS_TIMEOUT_MS 180000  // Max wait for GPS (3 min)
#define TZ_INFO "CET-1CEST,M3.5.0/2,M10.5.0/3"  // Timezone string
TinyGPSPlus GPS;

RTC_DATA_ATTR int64_t epoch_base = 0;  // Full 64-bit epoch time
RTC_DATA_ATTR bool send_on_lora = true;

void VextON(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);
}

void VextOFF(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(46, OUTPUT);
  digitalWrite(46, LOW);
}

void enterDeepSleepForSecounds(uint32_t secounds) {
  VextOFF();

  // Prepare peripherals
  Radio.Sleep();         // Put LoRa radio to sleep
  SPI.end();             // End SPI to save power

  // Set other unused pins to analog to minimize leakage
  pinMode(RADIO_DIO_1, ANALOG);
  pinMode(RADIO_RESET, ANALOG);
  pinMode(RADIO_BUSY, ANALOG);
  pinMode(LORA_CLK, ANALOG);
  pinMode(LORA_MISO, ANALOG);
  pinMode(LORA_MOSI, ANALOG);

  struct timeval now;
  gettimeofday(&now, nullptr);         // get current system time
  epoch_base = (int64_t)now.tv_sec;    // save seconds part to RTC memory
  Serial.printf("Saved time to RTC: %lld\n", epoch_base);
  Serial.printf("going to sleep for %u sec...\n", secounds);
  // Enable wake-up timer
  Serial.flush();
  send_on_lora = !send_on_lora;
  esp_sleep_enable_timer_wakeup((uint64_t)secounds  * 1000000ULL);
  // Enter deep sleep
  esp_deep_sleep_start();
}

void init_display() {
  pinMode(PIN_EINK_SCLK, OUTPUT);
  pinMode(PIN_EINK_DC, OUTPUT);
  pinMode(PIN_EINK_CS, OUTPUT);
  pinMode(PIN_EINK_RES, OUTPUT);

  // rest e-ink
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
  if ((chipId &0x03) !=0x01) {
    epaper_display = new HT_ICMEN2R13EFC1(3, 2, 5, 1, 4, 6, -1, 6000000);  // rst,dc,cs,busy,sck,mosi,miso,frequency
  } else {
    epaper_display = new HT_E0213A367(3, 2, 5, 1, 4, 6, -1, 6000000);  // rst,dc,cs,busy,sck,mosi,miso,frequency
  }
  epaper_display->init();
  epaper_display->setFont(ArialMT_Plain_10);
  epaper_display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void Navigation_bar(float airtempC) {
  setenv("TZ", TZ_INFO, 1);
  tzset();
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

void flash_led() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);  // LED on for 200 ms
  digitalWrite(LED, LOW);
}

void synctime_from_gps() {
  Serial.println("🛰️  Syncing time from GPS...");
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  uint32_t start = millis();
  bool synced = false;

  while (millis() - start < GPS_TIMEOUT_MS) {
    while (Serial1.available() > 0) {
      GPS.encode(Serial1.read());
        if ( GPS.time.isUpdated() && GPS.date.isUpdated() && GPS.time.isValid() && GPS.date.isValid() ) {
         struct tm t;
          t.tm_year = GPS.date.year() - 1900;
          t.tm_mon  = GPS.date.month() - 1;
          t.tm_mday = GPS.date.day();
          t.tm_hour = GPS.time.hour();
          t.tm_min  = GPS.time.minute();
          t.tm_sec  = GPS.time.second();
          t.tm_isdst = -1;

          time_t gpsEpoch = mktime(&t);
          if (gpsEpoch > 1760817666) {
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

void restore_time_from_rtc(uint32_t start) {
  struct timeval now;
  Serial.printf("Saved time from RTC: %lld\n", epoch_base);
  if (epoch_base > 0) {
    time_t restored = (time_t)(epoch_base + SLEEP_TIME + (start / 1000));  // Adjust for sleep duration
    now.tv_sec = restored;
    now.tv_usec = 0;
    Serial.print("✅ Restored time from RTC: ");
  } else {
    // fallback if no valid epoch_base
    time_t fallback_time = (time_t)(start / 1000);
    now.tv_sec = fallback_time;
    now.tv_usec = 0;
    Serial.print("❌ No valid RTC time. Using uptime-based time: ");
  }
  settimeofday(&now, nullptr);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%d.%m.%Y %H:%M:%S", &timeinfo);
  Serial.println(buffer);
  if (timeinfo.tm_min < 5 || (timeinfo.tm_year + 1900) < 2025) {
    synctime_from_gps();
  }
  setenv("TZ", TZ_INFO, 1);
  tzset();
}

void setup() {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  setCpuFrequencyMhz(80);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  uint32_t start = millis();
  Serial.begin(115200);
  VextON();

  // Disable Bluetooth and WiFi completely
  btStop();
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_wifi_deinit();

  flash_led();
  Serial.println("init >>> ");
  init_display();
#if(LORAWAN_DEVEUI_AUTO)
  LoRaWAN.generateDeveuiByChipID();
#endif
  restore_time_from_rtc(start);
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
                    LORA_IQ_INVERSION_ON, 3000);  // 3000 ms timeout
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
  String devAddrHex = toHexString(deviceAddress, 8);
  Serial.println(devAddrHex);
}

String toHexString(const uint8_t* data, size_t len) {
  String result = "";
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) result += "0";
    result += String(data[i], HEX);
  }
  return result;
}

void loop() {
  if (send_on_lora) {
    Radio.IrqProcess();
  } else {
    switch (deviceState) {
      case DEVICE_STATE_INIT: {
        Serial.println("LoraWAN init");
        LoRaWAN.init(loraWanClass, loraWanRegion);
        // both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
      case DEVICE_STATE_JOIN: {
          Serial.println("=== DEVICE_STATE_JOIN: starting OTAA join");
          joinStartTime = millis();
          LoRaWAN.join();
          Serial.println("LoRaWAN.join() called");
          break;
      }
      case DEVICE_STATE_SEND: {
          Serial.println("LoraWAN send");
          prepareTxFrame(appPort, airTempC, waterTempC, batteryVoltage);
          LoRaWAN.send();
          deviceState = DEVICE_STATE_CYCLE;
          break;
      }
      case DEVICE_STATE_CYCLE: {
        Serial.println("LoraWAN cycle");
        enterDeepSleepForSecounds(SLEEP_TIME);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
      case DEVICE_STATE_SLEEP: {
        LoRaWAN.sleep(loraWanClass);
        if (millis() - joinStartTime >= JOIN_TIMEOUT) {
          enterDeepSleepForSecounds(SLEEP_TIME);
        }
        break;
      }
      default: {
        Serial.println("LoraWAN init");
        deviceState = DEVICE_STATE_INIT;
        break;
      }
    }
  }
}

void OnTxDone(void) {
  Serial.println("txdone");
  enterDeepSleepForSecounds(SLEEP_TIME);
}

void OnTxTimeout(void) {
  Serial.println("txtimeout");
  enterDeepSleepForSecounds(SLEEP_TIME);
}
