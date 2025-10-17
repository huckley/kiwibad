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
#include "battery.h"
#include "esp_bt.h"                 // ESP32 Bluetooth control (power saving)
#include "esp_wifi.h"               // ESP32 WiFi control (power saving)
#include <time.h>

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

#define RF_FREQUENCY 865000000      // Frequency for EU868 band (Hz)
#define TX_OUTPUT_POWER 5           // Transmission power in dBm
#define LORA_BANDWIDTH 0            // 125 kHz bandwidth
#define LORA_SPREADING_FACTOR 7     // SF7 for fast, short-range comms
#define LORA_CODINGRATE 1           // 4/5 error correction
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 80              // Max buffer for outgoing message
#define SLEEP_TIME 60              // Sleep time in secounds
char txpacket[BUFFER_SIZE];         // Outgoing message buffer
// LoRa event structure
static RadioEvents_t RadioEvents;

// Function prototypes for LoRa TX events
void OnTxDone(void);
void OnTxTimeout(void);


#define Resolution 0.000244140625 
#define battary_in 3.3
#define coefficient 5.30624798087485
float batteryVoltage = 0;
#define LED  45
#define BAUD 9600
#define PIN_EINK_SCLK 4
#define PIN_EINK_DC   2
#define PIN_EINK_CS   5
#define PIN_EINK_RES  3
#define PIN_EINK_MOSI 6

RTC_DATA_ATTR int64_t epoch_base = 0;              // Full 64-bit epoch time
RTC_DATA_ATTR uint32_t last_awake_duration_ms = 0; // Awake time before last sleep
unsigned long current_awake_start = 0;

ScreenDisplay *display;

void VextON(void)
{
  pinMode(18,OUTPUT);
  digitalWrite(18, HIGH);
  pinMode(46,OUTPUT);
  digitalWrite(46, HIGH);
}

void VextOFF(void) //Vext default OFF
{
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
    display = new HT_ICMEN2R13EFC1(3, 2, 5, 1, 4, 6, -1, 6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
  } else {
    display = new HT_E0213A367(3, 2, 5, 1, 4, 6, -1, 6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
  }
  display->init();
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void Navigation_bar() {
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  float airtempC = sensors.getTempC(air_sensor_addr);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%H:%M:%S  %d.%m.%Y", &timeinfo);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawLine(0, 15, 250, 15);
  String line_one = "Luft: " + String(airtempC, 2) + "°C | " + buffer;
  Serial.println(line_one);
  display->drawString(0, 0, line_one);
  battery();
}

void battery() {   
  int rawADC = analogRead(7);
  batteryVoltage = rawADC * Resolution * battary_in * coefficient;//battary/4096*3.3*coefficient
  // Battery percentage
  float batteryMin = 3.0;
  float batteryMax = 4.2;
  float batteryPercent = ((batteryVoltage - batteryMin) / (batteryMax - batteryMin)) * 100.0;
  batteryPercent = constrain(batteryPercent, 0, 100);
  Serial.printf("Raw ADC: %d | Voltage: %.3f | %.0f%%\n", rawADC, batteryVoltage, batteryPercent);
  int level = batteryPercent / 12.5;  // 0–8 range

  switch (level) {
    case 0: display->drawXbm(230, 0, battery_w, battery_h, battery0); break;
    case 1: display->drawXbm(230, 0, battery_w, battery_h, battery1); break;
    case 2: display->drawXbm(230, 0, battery_w, battery_h, battery2); break;
    case 3: display->drawXbm(230, 0, battery_w, battery_h, battery3); break;
    case 4: display->drawXbm(230, 0, battery_w, battery_h, battery4); break;
    case 5: display->drawXbm(230, 0, battery_w, battery_h, battery5); break;
    case 6: display->drawXbm(230, 0, battery_w, battery_h, battery6); break;
    default:
      display->drawXbm(230, 0, battery_w, battery_h, batteryfull);
      break;
  }  
}

void update_display() {
  display->update(BLACK_BUFFER);
  display->display();
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
  display->clear();
  Navigation_bar();

  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Monospaced_bold_50);
  float water_tempC = sensors.getTempC(water_sensor_addr);
  String tempStr = String(water_tempC, 0) + "°C";
  Serial.println(tempStr);
  display->drawString(125, 40, tempStr);
  update_display();
  sendLoRaWithTempsAndBattery();
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

void sendLoRaWithTempsAndBattery() {
  float airTemp = sensors.getTempC(air_sensor_addr);
  float waterTemp = sensors.getTempC(water_sensor_addr);

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
  snprintf(txpacket, BUFFER_SIZE,
    "BAT:%.2fV|T1:%.1fC[%s]|T2:%.1fC[%s]",
    batteryVoltage, airTemp, airAddrStr, waterTemp, waterAddrStr);

  Serial.println("Sending LoRa message:");
  Serial.println(txpacket);
  // Send packet
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
}

void loop() {
  Radio.IrqProcess( );
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