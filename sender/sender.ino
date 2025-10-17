#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HT_lCMEN2R13EFC1.h"
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <HardwareSerial.h>
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

DeviceAddress air_sensor_addr   = { 0x28, 0xCE, 0xC8, 0x37, 0x00, 0x00, 0x00, 0x40 };
DeviceAddress water_sensor_addr = { 0x28, 0xD7, 0xB9, 0xB3, 0x00, 0x00, 0x00, 0xFA };

// ===============================
// LoRa Configuration
// ===============================

#define SLEEP_TIME 60              // Sleep time in secounds

#define LED  45
#define BAUD 9600
#define PIN_EINK_SCLK 4
#define PIN_EINK_DC   2
#define PIN_EINK_CS   5
#define PIN_EINK_RES  3
#define PIN_EINK_MOSI 6

RTC_DATA_ATTR int64_t epoch_base = 0;              // Full 64-bit epoch time
RTC_DATA_ATTR uint32_t last_awake_duration_ms = 0; // Awake time before last sleep
RTC_DATA_ATTR bool send_on_lora = true;

// LoraWAN
uint8_t appPort = 2;

unsigned long current_awake_start = 0;

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

  last_awake_duration_ms = millis() - current_awake_start;
  Serial.printf("Awake for %.2f sec, going to sleep for %u sec...\n",
                last_awake_duration_ms / 1000.0, secounds);
  // Enable wake-up timer
  Serial.flush();
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

void setup() {
  current_awake_start = millis();
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  setCpuFrequencyMhz(80);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
 
  //BERLIN
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);

  if (epoch_base == 0) {
    } else {
    // Woke from deep sleep - calculate new time
    int64_t updated_time = epoch_base
                         + SLEEP_TIME            // time spent asleep
                         + last_awake_duration_ms / 1000; // time spent awake last cycle

    struct timeval tv;
    tv.tv_sec = (time_t)(updated_time);  // Truncated to 32-bit if needed
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);

    epoch_base = updated_time; // update for next sleep
  }
  tzset();
  // Disable Bluetooth and WiFi completely
  btStop();
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_wifi_deinit();

  Serial.begin(115200);
  VextON();
  flash_led();
  Serial.println("init >>> ");
  init_display();
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
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0,
                    LORA_IQ_INVERSION_ON, 3000); // 3000 ms timeout
  sensors.requestTemperatures();
  float airtempC = sensors.getTempC(air_sensor_addr);
  float watertempC = sensors.getTempC(water_sensor_addr);
  epaper_display->clear();
  Navigation_bar(airtempC);
  epaper_display->setTextAlignment(TEXT_ALIGN_CENTER);
  epaper_display->setFont(Monospaced_bold_50);
  String tempStr = String(watertempC, 0) + "°C";
  Serial.println(tempStr);
  epaper_display->drawString(125, 40, tempStr);
  update_display();
  sendLoRaWithTempsAndBattery(airtempC, watertempC, batteryVoltage);
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
  Radio.Send((uint8_t *)packet, strlen(packet));
}

void loop() {
  if (send_on_lora) {
    Radio.IrqProcess();
  } else {
  /*  
  switch (deviceState) {
    case DEVICE_STATE_INIT: {
      LoRaWAN.generateDeveuiByChipID();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      //both set join DR and DR when ADR off
      LoRaWAN.setDefaultDR(3);
      break;
    }
    case DEVICE_STATE_JOIN {
        LoRaWAN.join();
        break;
    }
    case DEVICE_STATE_SEND: {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
    }
    case DEVICE_STATE_CYCLE: {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
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
  */
  }
}


void OnTxDone(void) {
  // Put radio to sleep to save power
  Serial.println("txdone");
  Radio.Sleep();
  enterDeepSleepForSecounds(SLEEP_TIME);
}

void OnTxTimeout(void) {
  Serial.println("txtimeout");
  Radio.Sleep();
  enterDeepSleepForSecounds(60);
}