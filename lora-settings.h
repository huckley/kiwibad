#define RF_FREQUENCY 865000000      // Frequency for EU868 band (Hz)
#define TX_OUTPUT_POWER 5           // Transmission power in dBm
#define LORA_BANDWIDTH 0            // 125 kHz bandwidth
#define LORA_SPREADING_FACTOR 7     // SF7 for fast, short-range comms
#define LORA_CODINGRATE 1           // 4/5 error correction
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 128              // Max buffer for outgoing message
// LoRa event structure
static RadioEvents_t RadioEvents;
char packet[BUFFER_SIZE];         //message buffer

// Function prototypes for LoRa TX events
void OnTxDone(void);
void OnTxTimeout(void);