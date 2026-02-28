#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <time.h>

#include "../secret.h"
#include "../lora.h"
#include "esp_bt.h"                 // ESP32 Bluetooth control (power saving)
#include "esp_wifi.h"               // ESP32 WiFi control (power saving)

#define buttonPin  0
#define PIN_VBAT 1          // ADC-Pin für Batterie
#define PIN_ADC_CTRL 37

typedef enum {
  LOWPOWER,
  STATE_RX
} States_t;

States_t state;

// ==== OLED ====
SSD1306Wire  oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // LOW = ON
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
  VextON();
  analogReadResolution(12);
  // Set pin 37 as an output pin (used for ADC control):
  pinMode(PIN_ADC_CTRL, OUTPUT);
  // Set pin 37 to HIGH (enable ADC control):
  digitalWrite(PIN_ADC_CTRL, HIGH);

  float voltage = getBatteryVoltage();
  oled_display.init();
  oled_display.setContrast(255);
  oled_display.screenRotate(ANGLE_0_DEGREE);
  oled_display.setFont(ArialMT_Plain_10);
  Serial.println("LoRa RX Listening...");
  oled_display.drawString(0, 0, "RX Listening... |Bat:" + String(voltage, 2));
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

float getBatteryVoltage() {
  int analogVolts = analogReadMilliVolts(PIN_VBAT);
  return (float)analogVolts * 490 / 100000;
}

// === OLED Display Update ===
void updateDisplay(float battery, float tempAir, float tempWater, uint8_t hour , uint8_t minute) {
  float voltage = getBatteryVoltage();
  oled_display.clear();
  oled_display.setFont(ArialMT_Plain_10);
  oled_display.drawString(0, 0, "Time: " + String(hour) + ":" + String(minute) + "|B: " + String(voltage, 2) +"|B: " + String(battery, 2) + " V");
  oled_display.setFont(ArialMT_Plain_24);
  oled_display.drawString(0, 10, "L: " + String(tempAir, 1) + "°C");
  oled_display.drawString(0, 34, "W: " + String(tempWater, 1) + "°C");
  oled_display.display();
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
  uint8_t packet[PAYLOAD_LEN];  // or use global if required
  memcpy(packet, payload, size);

  // Optional: only null-terminate for printing as string (only if it's known to be string)
  // Not needed here since it's binary payload
  // packet[size] = '\0';

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

  // 1. DevEUI
  char devEuiStr[17];
  for (int i = 0; i < 8; i++) {
    sprintf(&devEuiStr[i * 2], "%02X", packet[i]);
  }
  devEuiStr[16] = '\0';

  // 2. Time
  uint8_t hour = packet[8];
  uint8_t minute = packet[9];

  // 3. Battery Voltage (millivolts → float volts)
  uint16_t batteryRaw = (packet[10] << 8) | packet[11];
  float batteryVoltage = batteryRaw / 1000.0f;

  // 4. Air Temp
  uint16_t airRaw = (packet[12] << 8) | packet[13];
  float airTemp = (airRaw == 0xFFFF) ? NAN : (airRaw / 10.0f) - 40.0f;

  // 5. Water Temp
  uint16_t waterRaw = (packet[14] << 8) | packet[15];
  float waterTemp = (waterRaw == 0xFFFF) ? NAN : (waterRaw / 10.0f) - 40.0f;

  // === OUTPUT ===
  Serial.println("---- LoRa Packet Received ----");
  Serial.print("DevEUI: "); Serial.println(devEuiStr);
  Serial.printf("Time   : %02d:%02d\n", hour, minute);
  Serial.printf("Battery: %.3f V\n", batteryVoltage);
  Serial.printf("Air    : %s\n", isnan(airTemp) ? "-127" : String(airTemp, 1).c_str());
  Serial.printf("Water  : %s\n", isnan(waterTemp) ? "-127" : String(waterTemp, 1).c_str());

  // 7. Update display (make sure variable names match!)
  updateDisplay(batteryVoltage, airTemp, waterTemp, hour, minute);

  // 8. Resume RX
  state = STATE_RX;
}
