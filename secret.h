#include "mbedtls/md.h"

#define HMAC_KEY "MySuperSecretKey123"  // Use a strong key
#define HMAC_KEY_LEN (sizeof(HMAC_KEY) - 1)  // exclude null terminator

bool computeHMACSignature(const char* message, char* sigHex, size_t sigHexSize) {
  if (sigHexSize < 17) return false;  // Need at least 16 hex chars + null

  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!info) return false;

  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, info, 1) != 0) return false;

  if (mbedtls_md_hmac_starts(&ctx, (const unsigned char*)HMAC_KEY, HMAC_KEY_LEN) != 0) return false;
  if (mbedtls_md_hmac_update(&ctx, (const unsigned char*)message, strlen(message)) != 0) return false;

  uint8_t hmac[32];
  if (mbedtls_md_hmac_finish(&ctx, hmac) != 0) return false;

  mbedtls_md_free(&ctx);

  // Convert first 8 bytes to hex
  for (int i = 0; i < 8; i++) {
    sprintf(&sigHex[i * 2], "%02X", hmac[i]);
  }
  sigHex[16] = '\0'; // null-terminate

  return true;
}

bool verifyHMACSignature(const char* payload) {
  // Find "|SIG:" in payload
  const char* sigPtr = strstr(payload, "|SIG:");
  if (!sigPtr) {
    Serial.println("No signature found");
    return false;
  }

  // Extract received signature hex string (16 chars)
  char receivedSig[17];
  strncpy(receivedSig, sigPtr + 5, 16);
  receivedSig[16] = '\0';

  // Extract data before signature
  int dataLen = sigPtr - payload;
  char dataToVerify[dataLen + 1];
  strncpy(dataToVerify, payload, dataLen);
  dataToVerify[dataLen] = '\0';

  // Compute HMAC signature for extracted data
  char computedSig[17];
  if (!computeHMACSignature(dataToVerify, computedSig, sizeof(computedSig))) {
    Serial.println("Failed to compute HMAC");
    return false;
  }

  // Compare receivedSig and computedSig case-insensitive
  for (int i = 0; i < 16; i++) {
    if (toupper(receivedSig[i]) != toupper(computedSig[i])) {
      Serial.println("Signature mismatch");
      return false;
    }
  }

  Serial.println("Signature verified!");
  return true;
}