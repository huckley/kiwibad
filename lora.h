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

void sendLoRaWithTempsAndBattery( uint8_t* sensorIDs, float* temps, float batteryVoltage) {
  uint16_t TempInt[2]; 
  TempInt[0] = encodeTemp(temps[0]);
  TempInt[1] = encodeTemp(temps[1]);
  uint16_t battVoltInt = (uint16_t)(batteryVoltage * 1000);  // 2 bytes
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  payload[0] = sensorIDs[0];
  payload[1] = sensorIDs[1];
  payload[2] = timeinfo->tm_hour;
  payload[3] = timeinfo->tm_min;

  payload[4] = (battVoltInt >> 8) & 0xFF;
  payload[5] = battVoltInt & 0xFF;

  payload[6] = (TempInt[0] >> 8) & 0xFF;
  payload[7] = TempInt[0] & 0xFF;

  payload[8] = (TempInt[1] >> 8) & 0xFF;
  payload[9] = TempInt[1] & 0xFF;

  // 5. HMAC signature
  uint8_t signature[SIG_LEN];
  if (!computeHMACSignatureBinary(payload, MSG_LEN, signature, SIG_LEN)) {
    Serial.println("HMAC failed");
    return;
  }

  memcpy(payload + MSG_LEN, signature, SIG_LEN);  // Append sig
  Serial.print("Payload: ");
  for (int i = 0; i < sizeof(payload); i++) {
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  // 6. Send
  Radio.Send(payload, sizeof(payload));
}