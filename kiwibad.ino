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

#define ONE_WIRE_BUS 17 // Pin an dem der DS18B20 hängt
#define TEMPERATURE_PRECISION 9

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define MAX_SENSORS 10
DeviceAddress sensorAddresses[MAX_SENSORS];
int deviceCount = 0;

#define Resolution 0.000244140625 
#define battary_in 3.3
#define coefficient 5.30624798087485
#define LED  45
#define BAUD 9600
#define PIN_EINK_SCLK 4
#define PIN_EINK_DC   2
#define PIN_EINK_CS   5
#define PIN_EINK_RES  3
#define PIN_EINK_MOSI 6
DeviceAddress air_sensor_addr   = { 0x28, 0xCE, 0xC8, 0x37, 0x00, 0x00, 0x00, 0x40 };
DeviceAddress water_sensor_addr = { 0x28, 0xD7, 0xB9, 0xB3, 0x00, 0x00, 0x00, 0xFA };

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
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
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
  float batteryVoltage = rawADC * Resolution * battary_in * coefficient;//battary/4096*3.3*coefficient
  // Battery percentage
  float batteryMin = 3.0;
  float batteryMax = 4.2;
  float batteryPercent = ((batteryVoltage - batteryMin) / (batteryMax - batteryMin)) * 100.0;
  batteryPercent = constrain(batteryPercent, 0, 100);
  Serial.printf("Raw ADC: %d | Voltage: %.3f | %.0f%%\n", rawADC, batteryVoltage, batteryPercent);
  float battery_one = 12.5;
  if (batteryPercent < battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery0);
  }
  else if (batteryPercent < 2 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery1);
  }
  else if (batteryPercent < 3 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery2);
  }
  else if (batteryPercent < 4 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery3);
  }
  else if (batteryPercent < 5 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery4);
  }
  else if (batteryPercent < 6 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery5);
  }
  else if (batteryPercent < 7 * battery_one) {
    display->drawXbm(230, 0, battery_w, battery_h, battery6);
  }
  else {
    display->drawXbm(230, 0, battery_w, battery_h, batteryfull);
  }
}

void update_display() {
  display->update(BLACK_BUFFER);
  display->display();
}

void setup() {
  Serial.begin(115200);
  VextON();

  Serial.println("init >>> ");
  init_display();
  display->clear();  
  display->drawString(0, 0, "init >>> ");
  update_display();
  sensors.begin();

  deviceCount = sensors.getDeviceCount();
  Serial.print("Gefundene Sensoren: ");
  Serial.println(deviceCount);
  display->drawString(0, 10, "Gefundene Sensoren: ");
  display->drawString(110, 10, String(deviceCount));

  for (int i = 0; i < deviceCount && i < MAX_SENSORS; i++) {
    if (sensors.getAddress(sensorAddresses[i], i)) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" Adresse: ");
      printAddress(sensorAddresses[i]);

      sensors.setResolution(sensorAddresses[i], TEMPERATURE_PRECISION);

      char addressStr[25];
      sprintf(addressStr, "%02X%02X%02X%02X%02X%02X%02X%02X",
        sensorAddresses[i][0], sensorAddresses[i][1],
        sensorAddresses[i][2], sensorAddresses[i][3],
        sensorAddresses[i][4], sensorAddresses[i][5],
        sensorAddresses[i][6], sensorAddresses[i][7]);

      int y = 20 + (i * 10);
      display->drawString(0, y, "Sensor " + String(i));
      display->drawString(110, y, addressStr);
    } else {
      Serial.println("Sensor " + String(i) + " hat keine gültige Adresse.");
    }
  }
  update_display();
  delay(2000);
}

void loop() {

  sensors.requestTemperatures();
  display->clear();
  Navigation_bar();

  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Monospaced_bold_50);
  float water_tempC = sensors.getTempC(water_sensor_addr);
  String tempStr = String(water_tempC, 0) + "°C";
  Serial.println(tempStr);
  display->drawString(125, 40, tempStr);
  delay(5000);
  update_display();
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
