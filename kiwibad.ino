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


#define ONE_WIRE_BUS 38 // Pin an dem der DS18B20 hängt

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress sensorAddress;

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
  //display->screenRotate(DIRECTION);
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
}


void Navigation_bar() {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawLine(0, 15, 250, 15);
  display->drawString(0, 0, "Luft: -17°C 11.10.25 20:11");
  battery();
}

void battery()
{   
    int rawADC = analogRead(7);
    float batteryVoltage = rawADC * Resolution * battary_in * coefficient;//battary/4096*3.3*coefficient
    // Battery percentage
    float batteryMin = 3.0;
    float batteryMax = 4.2;
    float batteryPercent = ((batteryVoltage - batteryMin) / (batteryMax - batteryMin)) * 100.0;
    batteryPercent = constrain(batteryPercent, 0, 100);
    Serial.printf("Raw ADC: %d | Voltage: %.3f | %.0f%%\n", rawADC, batteryVoltage, batteryPercent);
    String msg = "Raw ADC: " + String(rawADC);
      msg += " | Voltage: " + String(batteryVoltage, 3);
      msg += " | " + String(batteryPercent, 0) + "%";  
    display->drawString(0, 100, msg);

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

  Serial.println("DS18B20 Test");
  init_display();
  display->clear();
  display->drawString(9, 0, "init >>> ");
  update_display();
  sensors.begin();

  int deviceCount = sensors.getDeviceCount();
  Serial.print("Gefundene Sensoren: ");
  display->drawString(0, 20, "Gefundene Sensoren: ");
  Serial.println(deviceCount);
  display->drawString(150, 20, String(deviceCount));

  if (!sensors.getAddress(sensorAddress, 0)) {
    Serial.println("Kein Sensor gefunden!");
    display->drawString(0, 40, "Kein Sensor gefunden!");
    update_display();
    return;
  }

  Serial.print("Sensor-Adresse: ");
  display->drawString(0, 40, "Sensor-Adresse: ");
  printAddress(sensorAddress);
  char addressStr[25];
  sprintf(addressStr, "%02X%02X%02X%02X%02X%02X%02X%02X",
    sensorAddress[0], sensorAddress[1], sensorAddress[2], sensorAddress[3],
    sensorAddress[4], sensorAddress[5], sensorAddress[6], sensorAddress[7]);  
  display->drawString(150, 40, addressStr);
  sensors.setResolution(sensorAddress, 12);
  Serial.print("Auflösung: ");
  display->drawString(0, 60, "Auflösung: ");
  Serial.println(sensors.getResolution(sensorAddress));
  display->drawString(150, 60, String(sensors.getResolution(sensorAddress)));
  update_display();
}

void loop() {
  sensors.requestTemperatures();

  float tempC = sensors.getTempC(sensorAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Fehler: Sensor nicht verbunden!");
  } else {
    Serial.print("Temperatur: ");
    Serial.print(tempC);
    Serial.println(" °C");
  }
  display->clear();

  delay(2000);
  Navigation_bar();

  display->setTextAlignment(TEXT_ALIGN_LEFT);
//  display->setTextAlignment(TEXT_ALIGN_CENTER);
//  x = width / 2;
//  y = height / 2+1;
  display->setFont(Monospaced_bold_50);
  display->drawString(0, 15, "-17°C");
  update_display();
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
