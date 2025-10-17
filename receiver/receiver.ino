#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <WiFiManager.h>            // Wi-Fi auto config
#include <time.h>

// WiFiManager fallback AP name
#define AP_NAME "LoRaNode-Setup"
#define AP_PASSWORD "configureme"

#include "../lora-settings.h"
#include "../secret.h"

typedef enum {
  LOWPOWER,
  STATE_RX
} States_t;

States_t state;

// ==== OLED ====
SSD1306Wire  oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // LOW = ON
}

void apModeCallback(WiFiManager *wm) {
  Serial.println("Entered AP Mode");
  oled_display.clear();
  oled_display.setFont(ArialMT_Plain_10);
  oled_display.drawString(0, 0, "Config Portal");
  oled_display.drawString(0, 14, AP_NAME);
  oled_display.drawString(0, 28, "Connect to AP");
  oled_display.drawString(0, 42, "Pass: " AP_PASSWORD);
  oled_display.display();
}

void setup() {
  Serial.begin(115200);
  // Start WiFiManager
  WiFiManager wm;
  wm.setAPCallback(apModeCallback);
  WiFiManagerParameter custom_ntp_server("server", "ntp server", "pool.ntp.org", 40);
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  wm.addParameter(&custom_ntp_server);
  wm.setConfigPortalBlocking(true);
  if (!wm.autoConnect(AP_NAME, AP_PASSWORD)) {
    Serial.println("Failed to connect to WiFi");
  } else {
    Serial.println("WiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  
    // Sync time
    const char* ntp_server = custom_ntp_server.getValue();
    if (strlen(ntp_server) == 0) {
      ntp_server = "pool.ntp.org";  // fallback
    }
    configTime(0, 0, ntp_server);
    unsigned long startAttempt = millis();
    const unsigned long timeout = 10000;  // 10 seconds

    while (time(nullptr) < 100000) {
      if (millis() - startAttempt > timeout) {
        Serial.println("\n⏱️ NTP sync timeout!");
        break;
      }
      delay(500);
      Serial.print(".");
    } 
    Serial.println();
  }
  
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();

  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char timeStr[30];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S %d.%m.%Y", &timeinfo);

  Serial.print("Current time: ");
  Serial.println(timeStr);


  // Board and Display Init
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  VextON();
  oled_display.init();
  oled_display.setContrast(255);
  oled_display.flipScreenVertically();
  oled_display.setFont(ArialMT_Plain_10);
  Serial.println("LoRa RX Listening...");
  oled_display.drawString(0, 0, "LoRa RX Listening...");
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
      Radio.Rx(0); // Listen for packets
      state = LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess(); // Check LoRa events
      break;
    default:
      break;
  }
}

// === LoRa Packet Received ===
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(packet, payload, size);
  packet[size] = '\0';
  Radio.Sleep();

  Serial.printf("Received: \"%s\"\n", packet);

  if (!verifyHMACSignature(packet)) {
    Serial.println("Invalid signature - ignoring packet");
    state = STATE_RX;
    return;
  }

  String message = String(packet);

  float battery = parseValue(message, "BAT:", "V");
  float tempAir = parseValue(message, "T1:", "C");
  float tempWater = parseValue(message, "T2:", "C");

  Serial.printf("Battery: %.2f V, Air: %.1f °C, Water: %.1f °C\n", battery, tempAir, tempWater);

  // Display update
  updateDisplay(battery, tempAir, tempWater);

  state = STATE_RX; // Resume listening
}

// === Parse values like BAT:3.81V, T1:20.1C[...], T2:15.8C[...] ===
float parseValue(String payload, String prefix, String stopChar) {
  int start = payload.indexOf(prefix);
  if (start == -1) return NAN;
  start += prefix.length();
  int end = payload.indexOf(stopChar, start);
  if (end == -1) return NAN;
  return payload.substring(start, end).toFloat();
}

// === OLED Display Update ===
void updateDisplay(float battery, float tempAir, float tempWater) {
  oled_display.clear();
  oled_display.setFont(ArialMT_Plain_10);

  char timeStr[20];
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", 
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  oled_display.drawString(0, 0, "LoRa Data:");
  oled_display.drawString(0, 12, "Air: " + String(tempAir, 1) + " °C");
  oled_display.drawString(0, 24, "Water: " + String(tempWater, 1) + " °C");
  oled_display.drawString(0, 36, "Battery: " + String(battery, 2) + " V");
  oled_display.drawString(0, 50, "Time: " + String(timeStr));
  oled_display.display();
}