#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <time.h>

#include "../secret.h"
#include "../lora.h"
#include "esp_bt.h"                 // ESP32 Bluetooth control (power saving)
#include "esp_wifi.h"               // ESP32 WiFi control (power saving)
#include "../sensors.h"
// ==== OLED ====
SSD1306Wire  oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#include "battery.h"

#define buttonPin 0
volatile bool buttonPressed = false;
unsigned long lastActivityTime = 0;
bool displayOn = true;
const unsigned long DISPLAY_TIMEOUT = 5 * 60 * 1000; // 5 Minuten in ms
volatile unsigned long lastButtonTime = 0; // Zeitstempel für Entprellung
const unsigned long DEBOUNCE_DELAY = 250;  // 250ms Sperrzeit

// ISR für den Button
void IRAM_ATTR handleButton() {
  unsigned long currentTime = millis();
  // Nur akzeptieren, wenn der letzte Klick länger als DEBOUNCE_DELAY her ist
  if (currentTime - lastButtonTime > DEBOUNCE_DELAY) {
    buttonPressed = true;
    lastButtonTime = currentTime;
  }
}

typedef enum {
  LOWPOWER,
  STATE_RX
} States_t;

States_t state;

struct Measurement {
  uint8_t address1;
  uint8_t address2;
  uint8_t hour;
  uint8_t minute;
  float temperature1;
  float temperature2;
  float batteryVoltage; 
};

const int MAX_ENTRIES = 24;
Measurement rrdBuffer[MAX_ENTRIES];
int head = 0;
int displayhead = 0;

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void setup() {
  Serial.begin(115200);
 
  // Disable Bluetooth and WiFi completely
  btStop();
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_wifi_deinit();

  // Board and Display Init
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButton, FALLING);
  lastActivityTime = millis(); // Timer starten

  VextON();
  oled_display.init();
  oled_display.setContrast(255);
  oled_display.screenRotate(ANGLE_0_DEGREE);
  oled_display.setFont(ArialMT_Plain_10);
  for (int i = 0; i < MAX_ENTRIES; i++){
    storeData(0, 0,  0, i, NAN, NAN, NAN);
  }
  Serial.println(url);
  Serial.println("LoRa RX Listening...");
  oled_display.drawString(0, 0, "RX Listening... ");
  battery();
  oled_display.drawString(0, 53, url);
  oled_display.display();
 
  // LoRa Init
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  state = STATE_RX;
}

void loop() {
  if (buttonPressed) {
    buttonPressed = false;
    lastActivityTime = millis(); // Timer resetten
    
    if (!displayOn) {
      VextON();
      oled_display.init();
      displayOn = true;
    } else {
      // Rückwärts durch den Buffer blättern
      displayhead = (displayhead - 1 + MAX_ENTRIES) % MAX_ENTRIES;
    }
    updateDisplay(displayhead);
  }

  // 2. Timeout Check (Display aus nach 5 Min)
 /*
  if (displayOn && (millis() - lastActivityTime > DISPLAY_TIMEOUT)) {
    oled_display.clear();
    oled_display.display();
    // Vext ausschalten spart am Heltec Board am meisten Strom
    VextOFF(); 
    displayOn = false;
  }
*/
  switch (state) {
    case STATE_RX:
      Radio.Rx(0);  // Listen for packets
      state = LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess();  // Check LoRa events
      break;
    default:
      break;
  }
}

// === OLED Display Update ===
void updateDisplay(int myhead) {
  float airTempC = NAN;
  float waterTempC = NAN;
  String h = (rrdBuffer[myhead].hour < 10) ? "0" + String(rrdBuffer[myhead].hour) : String(rrdBuffer[myhead].hour);
  String m = (rrdBuffer[myhead].minute < 10) ? "0" + String(rrdBuffer[myhead].minute) : String(rrdBuffer[myhead].minute);
  oled_display.clear();
  oled_display.setFont(ArialMT_Plain_10);
  oled_display.drawString(0, 0, "Time: " + h + ":" + m + "|" + rrdBuffer[myhead].address1 + rrdBuffer[myhead].address2);
  battery();
  oled_display.setFont(ArialMT_Plain_24);
  for (int j = 0; j < (sizeof(air_sensor_addr) / sizeof(air_sensor_addr[0])); j++) {
    if ( rrdBuffer[myhead].address1 == getshortaddr(air_sensor_addr[j]) )  {
      airTempC = rrdBuffer[myhead].temperature1;
    }
    if ( rrdBuffer[myhead].address2 == getshortaddr(air_sensor_addr[j]) )  {
      airTempC = rrdBuffer[myhead].temperature2;
    }
  }
  for (int j = 0; j < (sizeof(water_sensor_addr) / sizeof(water_sensor_addr[0])); j++) {
    if ( rrdBuffer[myhead].address1 == getshortaddr(water_sensor_addr[j]) )  {
      waterTempC = rrdBuffer[myhead].temperature1;
    }
    if ( rrdBuffer[myhead].address2 == getshortaddr(water_sensor_addr[j]) )  {
      waterTempC = rrdBuffer[myhead].temperature2;
    }
  }
  oled_display.drawString(0, 10, "L: " + String(airTempC, 1) + "°C");
  oled_display.drawString(0, 34, "W: " + String(waterTempC, 1) + "°C");
  oled_display.display();
}

void storeData(uint8_t addr1, uint8_t addr2, uint8_t hour, uint8_t minute, float temp1, float temp2, float batV) {
  rrdBuffer[head].hour = hour;
  rrdBuffer[head].minute = minute;
  rrdBuffer[head].address1 = addr1;
  rrdBuffer[head].address2 = addr2;
  rrdBuffer[head].temperature1 = temp1;
  rrdBuffer[head].temperature2 = temp2;
  rrdBuffer[head].batteryVoltage = batV;
  displayhead = head;
  head = (head + 1) % MAX_ENTRIES;
}

// === LoRa Packet Received ===
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();

  // Validate size
  if (size != PAYLOAD_LEN) {
    Serial.println("Invalid payload length");
    state = STATE_RX;
    return;
  }

  // Copy payload to local buffer if needed
  uint8_t packet[PAYLOAD_LEN]; 
  memcpy(packet, payload, size);

  // Optional: debug raw payload
  Serial.print("Raw payload: ");
  for (int i = 0; i < size; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();

  // === VERIFY HMAC ===
  if (!verifyHMACSignatureBinary(packet)) {
    Serial.println("Invalid signature - ignoring packet");
    state = STATE_RX;
    return;
  }

  // === PARSE PAYLOAD ===
  uint8_t sensorID1 = packet[0];
  uint8_t sensorID2 = packet[1];

  uint8_t hour = packet[2];
  uint8_t minute = packet[3];

  uint16_t batteryRaw = (packet[4] << 8) | packet[5];
  float batteryVoltage = batteryRaw / 1000.0f;
  
  uint16_t temp1Raw = (packet[6] << 8) | packet[7];
  float temp1 = (temp1Raw == 0xFFFF) ? NAN : (temp1Raw / 10.0f) - 40.0f;
  uint16_t temp2Raw = (packet[8] << 8) | packet[9];
  float temp2 = (temp2Raw == 0xFFFF) ? NAN : (temp2Raw / 10.0f) - 40.0f;

  storeData(sensorID1, sensorID2, hour, minute, temp1, temp2, batteryVoltage); 
  // === OUTPUT ===
  Serial.println("---- LoRa Packet Received ----");
  Serial.printf("SensorID: %02X:%02X\n", sensorID1, sensorID2);
  Serial.printf("Time   : %02d:%02d\n", hour, minute);
  Serial.printf("Battery: %.3f V\n", batteryVoltage);
  Serial.printf("Temp1  : %s\n", isnan(temp1) ? "-127" : String(temp1, 1).c_str());
  Serial.printf("Temp2  : %s\n", isnan(temp2) ? "-127" : String(temp2, 1).c_str());
  if (!displayOn) {
    VextON();
    oled_display.init();
    displayOn = true;
  }
  lastActivityTime = millis();
  updateDisplay(displayhead);  
  
  state = STATE_RX;
}
