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
uint8_t payload[21];  // 13 bytes of data + 8-byte HMAC
static RadioEvents_t RadioEvents;

// Function prototypes for LoRa TX events
void OnTxDone(void);
void OnTxTimeout(void);

static void prepareTxFrame(uint8_t port, float airTemp, float waterTemp, float batteryVoltage) {
  int16_t airTempInt = (int16_t)(airTemp * 100);  // 2 bytes
  int16_t waterTempInt = (int16_t)(waterTemp * 100);  // 2 bytes
  uint16_t battVoltInt = (uint16_t)(batteryVoltage * 1000);  // 2 bytes

  appDataSize = 6;  // 2 + 2 + 2 = 6 bytes

  appData[0] = (airTempInt >> 8) & 0xFF;
  appData[1] = airTempInt & 0xFF;

  appData[2] = (waterTempInt >> 8) & 0xFF;
  appData[3] = waterTempInt & 0xFF;

  appData[4] = (battVoltInt >> 8) & 0xFF;
  appData[5] = battVoltInt & 0xFF;
}


void sendLoRaWithTempsAndBattery(float airTemp, float waterTemp, float batteryVoltage) {
  extern uint8_t devEui[8];
 
  // 1. devEUI
  memcpy(payload, devEui, 8);

  // 2. battery (scaled 0-5V to 0–255)
  payload[8] = constrain(batteryVoltage * 51.0f, 0, 255);

  // 3. airTemp and waterTemp (-40 to +85)
  payload[9] = constrain((int)(airTemp + 40), 0, 255);
  payload[10] = constrain((int)(waterTemp + 40), 0, 255);

  // 4. time
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  payload[11] = timeinfo->tm_hour;
  payload[12] = timeinfo->tm_min;

  // 5. HMAC signature
  uint8_t signature[8];
  if (!computeHMACSignatureBinary(payload, 13, signature, sizeof(signature))) {
    Serial.println("HMAC failed");
    return;
  }

  memcpy(payload + 13, signature, 8);  // Append sig
  Serial.println("payload");
  // 6. Send
  Radio.Send(payload, sizeof(payload));
}
