#define RF_FREQUENCY 865000000      // Frequency for EU868 band (Hz)
#define TX_OUTPUT_POWER 14           // Transmission power in dBm
#define LORA_BANDWIDTH 0            // 125 kHz bandwidth
#define LORA_SPREADING_FACTOR 7     // SF7 for fast, short-range comms
#define LORA_CODINGRATE 1           // 4/5 error correction
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false


// ===============================
// LoWAN Configuration
// ===============================
/* ABP para*/
uint8_t nwkSKey[] = { };
uint8_t appSKey[] = { };
uint32_t devAddr;
/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
#define LORAMAC_DEFAULT_RX2_FREQUENCY 869525000
#define LORAMAC_DEFAULT_RX2_DR        DR_3

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
#define JOIN_TIMEOUT 300000  // Timeout for OTAA join in milliseconds (300 seconds)

// LoRa event structure
uint8_t payload[PAYLOAD_LEN];
static RadioEvents_t RadioEvents;

// Function prototypes for LoRa TX events
void OnTxDone(void);
void OnTxTimeout(void);

uint16_t encodeTemp(float temp) {
  if (temp <= -126.0f) return 0xFFFF;  // Error code (use 65535)
  return constrain((int16_t)((temp + 40.0f) * 10.0f), 0, 1250);
}

static void prepareTxFrame(uint8_t port, float airTemp, float waterTemp, float batteryVoltage) {
  uint16_t airTempInt = encodeTemp(airTemp);
  uint16_t waterTempInt = encodeTemp(waterTemp);
  uint16_t battVoltInt = (uint16_t)(batteryVoltage * 1000);  // 2 bytes

  appDataSize = 6;  // 2 + 2 + 2 = 6 bytes

  appData[0] = (battVoltInt >> 8) & 0xFF;
  appData[1] = battVoltInt & 0xFF;

  appData[2] = (airTempInt >> 8) & 0xFF;
  appData[3] = airTempInt & 0xFF;

  appData[4] = (waterTempInt >> 8) & 0xFF;
  appData[5] = waterTempInt & 0xFF;
}


void sendLoRaWithTempsAndBattery(float airTemp, float waterTemp, float batteryVoltage) {
  uint16_t airTempInt = encodeTemp(airTemp);
  uint16_t waterTempInt = encodeTemp(waterTemp);
  uint16_t battVoltInt = (uint16_t)(batteryVoltage * 1000);  // 2 bytes
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  // 1. devEUI
  memcpy(payload, devEui, 8);
  payload[8] = timeinfo->tm_hour;
  payload[9] = timeinfo->tm_min;

  payload[10] = (battVoltInt >> 8) & 0xFF;
  payload[11] = battVoltInt & 0xFF;

  payload[12] = (airTempInt >> 8) & 0xFF;
  payload[13] = airTempInt & 0xFF;

  payload[14] = (waterTempInt >> 8) & 0xFF;
  payload[15] = waterTempInt & 0xFF;

  // 5. HMAC signature
  uint8_t signature[8];
  if (!computeHMACSignatureBinary(payload, 16, signature, 8)) {
    Serial.println("HMAC failed");
    return;
  }

  memcpy(payload + 16, signature, 8);  // Append sig
  Serial.print("Payload: ");
  for (int i = 0; i < sizeof(payload); i++) {
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  // 6. Send
  Radio.Send(payload, sizeof(payload));
}

