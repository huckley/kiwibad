#include "mbedtls/md.h"

#define HMAC_KEY "MySuperSecretKey123"  // Use a strong key
#define PAYLOAD_LEN 18

/* OTAA para*/
uint8_t devEui[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78 };  // example
uint8_t appEui[] = { 0xD7, 0xF9, 0x79, 0x7A, 0x39, 0x42, 0xAD, 0x79 };
uint8_t appKey[] = { 0xA7, 0x1B, 0xF6, 0x04, 0xB2, 0xB3, 0x50, 0xCA, 0x14, 0x21, 0xDF, 0xB9, 0xD1, 0xEA, 0x62, 0x4C };

bool computeHMACSignatureBinary(const uint8_t* message, size_t messageLen, uint8_t* outSig, size_t outSigLen) {
  if (outSigLen < 8) return false;  // Need at least 8 bytes for truncated signature

  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!info) return false;

  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, info, 1) != 0) {
    mbedtls_md_free(&ctx);
    return false;
  }

  if (mbedtls_md_hmac_starts(&ctx, (const unsigned char*)HMAC_KEY, (sizeof(HMAC_KEY) - 1)  ) != 0 ||
      mbedtls_md_hmac_update(&ctx, message, messageLen) != 0) {
    mbedtls_md_free(&ctx);
    return false;
  }

  uint8_t hmac[32];  // Full SHA-256 HMAC output
  if (mbedtls_md_hmac_finish(&ctx, hmac) != 0) {
    mbedtls_md_free(&ctx);
    return false;
  }

  mbedtls_md_free(&ctx);

  // Copy the first 8 bytes (64 bits) of the HMAC to output buffer
  memcpy(outSig, hmac, outSigLen);
  Serial.print("hmac: ");
  for (int i = 0; i < outSigLen; i++) {
    Serial.print(hmac[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  return true;
}

bool verifyHMACSignatureBinary(const uint8_t* payload) {
  const uint8_t* message = payload;
  const uint8_t* receivedSig = payload + 16;

  Serial.print(" receivedSig ");
  for (int i = 0; i < 8; i++) {
    Serial.print(receivedSig[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  uint8_t expectedSig[8];
  if (!computeHMACSignatureBinary(message, 16, expectedSig, 8)) {
    return false;
  }

  bool match = (memcmp(receivedSig, expectedSig, 8) == 0);

  return match;
}
