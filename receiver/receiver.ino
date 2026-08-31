#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <time.h>
#include "../secret.h"
#include "../lora.h"
#include "esp_wifi.h"     // ESP32 WiFi control (power saving)
#include "../sensors.h"

// ==== phyphox BLE (nativ, ohne phyphoxBLE-Bibliothek) ====
// Zwei GATT-Services:
//  1) Der offizielle phyphox-Service (UUID cddf0001-...). Wird beworben,
//     damit das Geraet im Bluetooth-Scan von phyphox direkt "weiss"
//     (=unterstuetzt) erscheint. Ueber die Characteristics 0002/0003
//     liefert der Receiver die komplette Experiment-Konfiguration
//     (siehe PHYPHOX_XML unten) automatisch an die App - kein manuelles
//     Laden einer .phyphox-Datei, kein QR-Code noetig.
//     Protokoll: https://phyphox.org/wiki/index.php/Bluetooth_Low_Energy
//     (Abschnitt "Sending phyphox-files from a device")
//  2) Ein eigener Service fuer die eigentlichen Messwerte (6b6a0001-...),
//     auf den die oben gelieferte Konfiguration verweist.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BLE_DEVICE_NAME "Kiwibad"

#define PHYPHOX_SERVICE_UUID     "cddf0001-30f7-4671-8b43-5e40ba53514a"
#define PHYPHOX_EXPERIMENT_UUID  "cddf0002-30f7-4671-8b43-5e40ba53514a" // liefert die Konfiguration
#define PHYPHOX_CONTROL_UUID     "cddf0003-30f7-4671-8b43-5e40ba53514a" // App schreibt 1 = "sende jetzt"

#define KIWIBAD_DATA_SERVICE_UUID "6b6a0001-1c7c-4d69-9d2b-3a2e6f9a1000"
#define KIWIBAD_DATA_CHAR_UUID    "6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000"

// Identisch zu Kiwibad.phyphox im Repo (dort nur als lesbares Backup/Referenz -
// diese eingebettete Kopie ist die, die tatsaechlich an die App geschickt wird).
static const char PHYPHOX_XML[] =
R"PHYPHOXFILE(<phyphox version="1.7">
    <title>Kiwibad Temperatur</title>
    <category>Kiwibad</category>
    <color>ff8000</color>
    <description>
        Luft- und Wassertemperatur (je 2 Sensoren) sowie Batteriespannung aus
        dem Kiwibad, per LoRa empfangen. Der Receiver liefert dieses
        Experiment automatisch per Bluetooth mit - kein manuelles Laden
        einer Datei noetig, einfach im Bluetooth-Scan antippen.
        Zeigt zuerst die gespeicherte Historie (bis zu 24 Messungen), danach
        neue Messungen live.
        X-Achse: Uhrzeit in Dezimalstunden (z.B. 8.5 = 08:30 Uhr).
        Ein Sensor, der noch nie gemeldet wurde, liefert -127°C (dieselbe
        Fehlerkonvention wie beim DS18B20-Sensor selbst) - diese Punkte
        werden in den Graphen per Filter entfernt (echte Luecke statt
        Ausreisser).
    </description>

    <data-containers>
        <container size="500">recordTimeHours</container>
        <container size="500">air1Temp</container>
        <container size="500">air2Temp</container>
        <container size="500">water1Temp</container>
        <container size="500">water2Temp</container>
        <container size="500">batteryMilliVolts</container>
        <container size="500">air1TempF</container>
        <container size="500">air1TimeF</container>
        <container size="500">air2TempF</container>
        <container size="500">air2TimeF</container>
        <container size="500">water1TempF</container>
        <container size="500">water1TimeF</container>
        <container size="500">water2TempF</container>
        <container size="500">water2TimeF</container>
    </data-containers>

    <input>
        <bluetooth name="Kiwibad" uuid="cddf0001-30f7-4671-8b43-5e40ba53514a" id="kiwibad" autoConnect="true" mode="notification" subscribeOnStart="false">
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="float32LittleEndian" offset="0" size="4">recordTimeHours</output>
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="int8" offset="4" size="1">air1Temp</output>
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="int8" offset="5" size="1">air2Temp</output>
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="int8" offset="6" size="1">water1Temp</output>
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="int8" offset="7" size="1">water2Temp</output>
            <output char="6b6a0002-1c7c-4d69-9d2b-3a2e6f9a1000" conversion="uInt16LittleEndian" offset="8" size="2">batteryMilliVolts</output>
        </bluetooth>
    </input>

    <analysis>
        <rangefilter>
            <input clear="false">air1Temp</input>
            <input as="min" type="value">-50</input>
            <input as="max" type="value">100</input>
            <input clear="false">recordTimeHours</input>
            <output>air1TempF</output>
            <output>air1TimeF</output>
        </rangefilter>
        <rangefilter>
            <input clear="false">air2Temp</input>
            <input as="min" type="value">-50</input>
            <input as="max" type="value">100</input>
            <input clear="false">recordTimeHours</input>
            <output>air2TempF</output>
            <output>air2TimeF</output>
        </rangefilter>
        <rangefilter>
            <input clear="false">water1Temp</input>
            <input as="min" type="value">-50</input>
            <input as="max" type="value">100</input>
            <input clear="false">recordTimeHours</input>
            <output>water1TempF</output>
            <output>water1TimeF</output>
        </rangefilter>
        <rangefilter>
            <input clear="false">water2Temp</input>
            <input as="min" type="value">-50</input>
            <input as="max" type="value">100</input>
            <input clear="false">recordTimeHours</input>
            <output>water2TempF</output>
            <output>water2TimeF</output>
        </rangefilter>
    </analysis>

    <views>
        <view label="Temperaturverlauf">
            <value label="Uhrzeit" unit="h" precision="2">
                <input>recordTimeHours</input>
            </value>
            <value label="Luft 1" unit="°C" precision="0" color="ff8000">
                <input>air1Temp</input>
            </value>
            <value label="Luft 2" unit="°C" precision="0" color="ffc080">
                <input>air2Temp</input>
            </value>
            <value label="Wasser 1" unit="°C" precision="0" color="0080ff">
                <input>water1Temp</input>
            </value>
            <value label="Wasser 2" unit="°C" precision="0" color="80c0ff">
                <input>water2Temp</input>
            </value>
            <value label="Batteriespannung" unit="mV" precision="0">
                <input>batteryMilliVolts</input>
            </value>

            <separator height="1" />

            <graph label="Lufttemperatur" labelX="Uhrzeit (h)" labelY="Temperatur" unitY="°C" partialUpdate="true" style="dots">
                <input axis="x" color="ff8000">air1TimeF</input>
                <input axis="y" color="ff8000">air1TempF</input>
                <input axis="x" color="ffc080">air2TimeF</input>
                <input axis="y" color="ffc080">air2TempF</input>
            </graph>

            <graph label="Wassertemperatur" labelX="Uhrzeit (h)" labelY="Temperatur" unitY="°C" partialUpdate="true" style="dots">
                <input axis="x" color="0080ff">water1TimeF</input>
                <input axis="y" color="0080ff">water1TempF</input>
                <input axis="x" color="80c0ff">water2TimeF</input>
                <input axis="y" color="80c0ff">water2TempF</input>
            </graph>

            <graph label="Batteriespannung" labelX="Uhrzeit (h)" labelY="Spannung" unitY="mV" partialUpdate="true" style="dots">
                <input axis="x">recordTimeHours</input>
                <input axis="y">batteryMilliVolts</input>
            </graph>
        </view>
    </views>

    <export>
        <set name="Kiwibad Messwerte (roh, inkl. -127 = kein Wert)">
            <data name="Uhrzeit (Dezimalstunden)">recordTimeHours</data>
            <data name="Lufttemperatur 1 (°C)">air1Temp</data>
            <data name="Lufttemperatur 2 (°C)">air2Temp</data>
            <data name="Wassertemperatur 1 (°C)">water1Temp</data>
            <data name="Wassertemperatur 2 (°C)">water2Temp</data>
            <data name="Batteriespannung (mV)">batteryMilliVolts</data>
        </set>
        <set name="Kiwibad Messwerte (gefiltert)">
            <data name="Luft 1 - Uhrzeit">air1TimeF</data>
            <data name="Luft 1 - Temperatur (°C)">air1TempF</data>
            <data name="Luft 2 - Uhrzeit">air2TimeF</data>
            <data name="Luft 2 - Temperatur (°C)">air2TempF</data>
            <data name="Wasser 1 - Uhrzeit">water1TimeF</data>
            <data name="Wasser 1 - Temperatur (°C)">water1TempF</data>
            <data name="Wasser 2 - Uhrzeit">water2TimeF</data>
            <data name="Wasser 2 - Temperatur (°C)">water2TempF</data>
        </set>
    </export>
</phyphox>
)PHYPHOXFILE";

BLEServer *pBleServer = nullptr;
BLECharacteristic *pPhyphoxExperimentChar = nullptr; // 0002: liefert PHYPHOX_XML
BLECharacteristic *pPhyphoxControlChar = nullptr;     // 0003: App schreibt 1 = "sende jetzt"
BLECharacteristic *pKiwibadDataChar = nullptr;        // eigene Messwert-Characteristic
volatile bool bleClientConnected = false;

static BLE2902 phyphoxExperimentCCCD;
static BLE2902 kiwibadDataCCCD;

// === Nicht-blockierendes History-Replay (statt delay(30) in einer Schleife) ===
// tickHistoryReplay() wird aus loop() aufgerufen und schickt pro Aufruf
// hoechstens EINEN Eintrag - der naechste folgt erst wieder, wenn seit dem
// letzten Versand mindestens HISTORY_SEND_INTERVAL_MS vergangen sind.
const unsigned long HISTORY_SEND_INTERVAL_MS = 30;
bool historyReplayActive = false;
int historyReplayIndex = 0;
float historyFillAir[MAX_SENSORS];
float historyFillWater[MAX_SENSORS];
unsigned long historyLastSendTime = 0;

// Fallback, falls der CCCD-Subscribe-Callback aus irgendeinem Grund nicht
// zuverlaessig feuert (variiert je nach ESP32-BLE-Bibliotheksversion):
// solange verbunden, wird die Historie zusaetzlich in einem festen Intervall
// automatisch erneut geschickt - dann klappt es spaetestens danach von selbst,
// ganz ohne Tastendruck.
const unsigned long AUTO_HISTORY_RESEND_INTERVAL_MS = 5000;
unsigned long lastAutoHistorySend = 0;

// === Nicht-blockierende Uebertragung der phyphox-Experiment-Datei ===
const unsigned long FILE_CHUNK_INTERVAL_MS = 20;
const size_t FILE_CHUNK_SIZE = 18; // sicher unter der Standard-BLE-MTU von 20 Byte
volatile bool fileTransferRequested = false; // wird aus der BLE-Callback gesetzt
bool fileTransferActive = false;
bool fileTransferHeaderSent = false;
size_t fileTransferSent = 0;
unsigned long fileTransferLastChunkTime = 0;

// ==== OLED ====
SSD1306Wire oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#include "battery.h"

#define buttonPin 0
volatile bool buttonPressed = false;
unsigned long lastActivityTime = 0;
bool displayOn = true;
const unsigned long DISPLAY_TIMEOUT = 5 * 60 * 1000; // 5 Minuten in ms
volatile unsigned long lastButtonTime = 0; // Zeitstempel für Entprellung
const unsigned long DEBOUNCE_DELAY = 250; // 250ms Sperrzeit

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

// === Sensor-Rohwerte den einzelnen physischen Sensoren zuordnen ===
// sensors.h definiert MAX_SENSORS (=2) Luft- UND 2 Wassersensoren mit fester
// Adresse. Ein LoRa-Paket trägt immer nur zwei Messwerte (addr1/temp1,
// addr2/temp2), aber über mehrere Pakete hinweg können unterschiedliche
// Sensoren gemeldet werden. Deshalb hier NICHT auf ein gemeinsames
// "airTemp"/"waterTemp" zusammenfassen, sondern jedem
// air_sensor_addr[i] / water_sensor_addr[i] seinen eigenen Kanal geben.
// Nicht in diesem Paket enthaltene Sensoren bleiben NAN.
void classifySensors(uint8_t addr1, uint8_t addr2, float temp1, float temp2,
                      float airTemps[MAX_SENSORS], float waterTemps[MAX_SENSORS]) {
  for (int i = 0; i < MAX_SENSORS; i++) {
    airTemps[i] = NAN;
    waterTemps[i] = NAN;
  }
  for (int i = 0; i < MAX_SENSORS; i++) {
    uint8_t airAddr = getshortaddr(air_sensor_addr[i]);
    if (addr1 == airAddr) airTemps[i] = temp1;
    if (addr2 == airAddr) airTemps[i] = temp2;

    uint8_t waterAddr = getshortaddr(water_sensor_addr[i]);
    if (addr1 == waterAddr) waterTemps[i] = temp1;
    if (addr2 == waterAddr) waterTemps[i] = temp2;
  }
}

// Ersten nicht-NAN Wert aus einem Array zurückgeben (fürs kleine OLED-Display,
// das nur Platz für einen einzelnen "L:"/"W:"-Wert hat)
float firstValid(float values[MAX_SENSORS]) {
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!isnan(values[i])) return values[i];
  }
  return NAN;
}

// === Einen Messpunkt an phyphox schicken ===
// Ein Paket, 10 Byte, little endian (siehe Kiwibad.phyphox):
//   Byte 0-3: Uhrzeit in Dezimalstunden (float32)
//   Byte 4  : Lufttemperatur 1, ganze °C (int8) - oder -127, wenn dieser
//             Sensor noch nie gemeldet wurde (dieselbe Fehlerkonvention wie
//             beim DS18B20 selbst, siehe OnRxDone weiter unten)
//   Byte 5  : Lufttemperatur 2, ganze °C (int8, -127 = kein Wert)
//   Byte 6  : Wassertemperatur 1, ganze °C (int8, -127 = kein Wert)
//   Byte 7  : Wassertemperatur 2, ganze °C (int8, -127 = kein Wert)
//   Byte 8-9: Batteriespannung in mV (uInt16)
const int8_t PHYPHOX_NO_VALUE = -127;

void sendMeasurementToPhyphox(uint16_t recordTime, float airTemps[MAX_SENSORS],
                               float waterTemps[MAX_SENSORS], float batV) {
  if (!bleClientConnected || pKiwibadDataChar == nullptr) {
    Serial.println("phyphox-Messwert NICHT gesendet: kein BLE-Client verbunden");
    return;
  }

  float recordTimeHoursF = (float)recordTime / 60.0f;

  int8_t airVals[MAX_SENSORS];
  int8_t waterVals[MAX_SENSORS];
  for (int s = 0; s < MAX_SENSORS; s++) {
    airVals[s]   = isnan(airTemps[s])   ? PHYPHOX_NO_VALUE : (int8_t)lroundf(airTemps[s]);
    waterVals[s] = isnan(waterTemps[s]) ? PHYPHOX_NO_VALUE : (int8_t)lroundf(waterTemps[s]);
  }
  uint16_t battMv = (uint16_t)lroundf(batV * 1000.0f);

  uint8_t packet[10];
  memcpy(&packet[0], &recordTimeHoursF, sizeof(float));
  packet[4] = (uint8_t)airVals[0];
  packet[5] = (uint8_t)airVals[1];
  packet[6] = (uint8_t)waterVals[0];
  packet[7] = (uint8_t)waterVals[1];
  packet[8] = battMv & 0xFF;
  packet[9] = (battMv >> 8) & 0xFF;

  pKiwibadDataChar->setValue(packet, sizeof(packet));
  pKiwibadDataChar->notify();

  Serial.printf("phyphox-Messwert gesendet: t=%.2f h, Luft1=%d Luft2=%d Wasser1=%d Wasser2=%d C, Batt=%u mV\n",
                recordTimeHoursF, airVals[0], airVals[1], waterVals[0], waterVals[1], battMv);
}

// === Kompletten Ringpuffer (älteste zuerst) an phyphox schicken ===
// Ein LoRa-Paket liefert immer nur 2 der 4 Sensoren - deshalb wird hier beim
// Durchlaufen fortgeschrieben (letzter bekannter Wert bleibt stehen, statt
// bei jedem Eintrag auf 2 von 4 Linien eine Lücke zu erzeugen).
//
// Nicht-blockierend: startHistoryReplay() setzt nur den Startzustand,
// tickHistoryReplay() (aus loop() aufgerufen) schickt pro Aufruf höchstens
// einen Eintrag und pausiert dazwischen per millis()-Vergleich statt delay().
void startHistoryReplay() {
  for (int s = 0; s < MAX_SENSORS; s++) {
    historyFillAir[s] = NAN;
    historyFillWater[s] = NAN;
  }
  historyReplayIndex = 0;
  historyReplayActive = true;
  historyLastSendTime = 0; // naechster tick darf sofort senden
}

void tickHistoryReplay() {
  if (!historyReplayActive) return;
  if (!bleClientConnected) {
    historyReplayActive = false;
    return;
  }
  if (millis() - historyLastSendTime < HISTORY_SEND_INTERVAL_MS) return;

  // Noch nie beschriebene Slots (Startzustand) direkt überspringen, ohne
  // dafür extra zu warten (es wird ja nichts gesendet)
  while (historyReplayIndex < MAX_ENTRIES) {
    int idx = (head + historyReplayIndex) % MAX_ENTRIES; // head zeigt auf den ältesten Eintrag
    if (!(isnan(rrdBuffer[idx].temperature1) && isnan(rrdBuffer[idx].temperature2))) break;
    historyReplayIndex++;
  }

  if (historyReplayIndex >= MAX_ENTRIES) {
    historyReplayActive = false;
    return;
  }

  int idx = (head + historyReplayIndex) % MAX_ENTRIES;
  float airTemps[MAX_SENSORS], waterTemps[MAX_SENSORS];
  classifySensors(rrdBuffer[idx].address1, rrdBuffer[idx].address2,
                   rrdBuffer[idx].temperature1, rrdBuffer[idx].temperature2,
                   airTemps, waterTemps);

  for (int s = 0; s < MAX_SENSORS; s++) {
    if (!isnan(airTemps[s])) historyFillAir[s] = airTemps[s];
    if (!isnan(waterTemps[s])) historyFillWater[s] = waterTemps[s];
  }

  uint16_t recordTime = (uint16_t)rrdBuffer[idx].hour * 60 + rrdBuffer[idx].minute;
  sendMeasurementToPhyphox(recordTime, historyFillAir, historyFillWater, rrdBuffer[idx].batteryVoltage);

  historyReplayIndex++;
  historyLastSendTime = millis();
}

// === CRC32 (zlib-Polynom), fürs phyphox-Uebertragungsprotokoll benoetigt ===
uint32_t crc32_calc(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

// === Liefert PHYPHOX_XML ueber die phyphox-Experiment-Characteristic (0002) ===
// Protokoll: erstes Paket = "phyphox" (7 Byte) + Groesse (4 Byte, big endian)
// + CRC32 (4 Byte, big endian). Danach folgt der Dateiinhalt in Chunks.
//
// Nicht-blockierend: requestPhyphoxFileTransfer() (aus der BLE-Callback)
// setzt nur ein Flag, tickPhyphoxFileTransfer() (aus loop()) schickt pro
// Aufruf höchstens ein Paket (Header oder einen Chunk) und wartet zwischen
// den Paketen per millis()-Vergleich statt delay().
void requestPhyphoxFileTransfer() {
  fileTransferRequested = true;
}

void tickPhyphoxFileTransfer() {
  if (fileTransferRequested) {
    fileTransferRequested = false;
    fileTransferActive = true;
    fileTransferHeaderSent = false;
    fileTransferSent = 0;
    fileTransferLastChunkTime = 0; // naechster tick darf sofort senden
  }

  if (!fileTransferActive || pPhyphoxExperimentChar == nullptr) return;
  if (millis() - fileTransferLastChunkTime < FILE_CHUNK_INTERVAL_MS) return;

  size_t fileLen = strlen(PHYPHOX_XML);

  if (!fileTransferHeaderSent) {
    uint32_t crc = crc32_calc((const uint8_t *)PHYPHOX_XML, fileLen);
    uint8_t header[15];
    memcpy(header, "phyphox", 7);
    header[7]  = (fileLen >> 24) & 0xFF;
    header[8]  = (fileLen >> 16) & 0xFF;
    header[9]  = (fileLen >> 8) & 0xFF;
    header[10] = fileLen & 0xFF;
    header[11] = (crc >> 24) & 0xFF;
    header[12] = (crc >> 16) & 0xFF;
    header[13] = (crc >> 8) & 0xFF;
    header[14] = crc & 0xFF;

    pPhyphoxExperimentChar->setValue(header, sizeof(header));
    pPhyphoxExperimentChar->notify();
    fileTransferHeaderSent = true;
    fileTransferLastChunkTime = millis();
    Serial.printf("phyphox-Datei: Uebertragung gestartet (%u Byte, CRC32 %08X)\n", (unsigned)fileLen, crc);
    return;
  }

  size_t remaining = fileLen - fileTransferSent;
  size_t n = (remaining < FILE_CHUNK_SIZE) ? remaining : FILE_CHUNK_SIZE;
  pPhyphoxExperimentChar->setValue((uint8_t *)(PHYPHOX_XML + fileTransferSent), n);
  pPhyphoxExperimentChar->notify();
  fileTransferSent += n;
  fileTransferLastChunkTime = millis();

  if (fileTransferSent >= fileLen) {
    fileTransferActive = false;
    Serial.println("phyphox-Datei: Uebertragung abgeschlossen");
  }
}

// App schreibt 1 auf die Control-Characteristic (0003), um den Transfer der
// Experiment-Datei anzustossen (Standardweg laut phyphox-Doku fuer Geraete,
// die nicht auf eine Notify-Subscription reagieren koennen).
// getData()/getLength() statt getValue() (std::string) - keine String-Klasse.
class PhyphoxControlCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (data != nullptr && len > 0 && data[0] == 1) {
      requestPhyphoxFileTransfer();
    }
  }
};

// Erkennt, wann phyphox sich fuer die Mess-Characteristic (6b6a0002)
// anmeldet (Notifications aktiviert) - das ist der zuverlaessige Zeitpunkt
// fuer den History-Versand, NICHT eine geratene feste Wartezeit nach dem
// Connect: phyphox laedt zu dem Zeitpunkt oft noch die Konfigurationsdatei
// herunter und hat die Mess-Characteristic noch gar nicht entdeckt/abonniert.
class KiwibadDataCCCDCallbacks: public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor *descriptor) override {
    if (kiwibadDataCCCD.getNotifications()) {
      startHistoryReplay();
    }
  }
};

// onConnect setzt nur das Verbindungs-Flag (kein delay()). Der eigentliche
// History-Versand wird ueber KiwibadDataCCCDCallbacks ausgeloest, sobald
// phyphox wirklich zuhoert - siehe oben.
class KiwibadBleCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
  }
  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    server->getAdvertising()->start(); // Advertising nach Disconnect neu starten
  }
};

// Statisch statt "new ...Callbacks()" - keine Heap-Allokation.
static PhyphoxControlCallbacks phyphoxControlCallbacks;
static KiwibadBleCallbacks kiwibadBleCallbacks;
static KiwibadDataCCCDCallbacks kiwibadDataCCCDCallbacks;

void setupPhyphoxBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  pBleServer = BLEDevice::createServer();
  pBleServer->setCallbacks(&kiwibadBleCallbacks);

  // --- phyphox-Service: liefert die Experiment-Konfiguration automatisch ---
  BLEService *pPhyphoxService = pBleServer->createService(PHYPHOX_SERVICE_UUID);

  pPhyphoxExperimentChar = pPhyphoxService->createCharacteristic(
                              PHYPHOX_EXPERIMENT_UUID,
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
  pPhyphoxExperimentChar->addDescriptor(&phyphoxExperimentCCCD);

  pPhyphoxControlChar = pPhyphoxService->createCharacteristic(
                           PHYPHOX_CONTROL_UUID,
                           BLECharacteristic::PROPERTY_WRITE
                         );
  pPhyphoxControlChar->setCallbacks(&phyphoxControlCallbacks);

  pPhyphoxService->start();

  // --- eigener Service fuer die Messwerte ---
  BLEService *pDataService = pBleServer->createService(KIWIBAD_DATA_SERVICE_UUID);
  pKiwibadDataChar = pDataService->createCharacteristic(
                        KIWIBAD_DATA_CHAR_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pKiwibadDataChar->addDescriptor(&kiwibadDataCCCD);
  kiwibadDataCCCD.setCallbacks(&kiwibadDataCCCDCallbacks);
  pDataService->start();

  // Nur den phyphox-Service bewerben - dadurch erscheint das Geraet im
  // phyphox-Bluetooth-Scan "weiss" (=direkt unterstuetzt), genau wie ein
  // Geraet mit eingebauter phyphox-Unterstuetzung. Der Datenservice wird
  // nach dem Verbinden ganz normal per GATT-Discovery gefunden, braucht
  // also keine eigene Werbung.
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(PHYPHOX_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("phyphox-BLE gestartet, Geraetename: " BLE_DEVICE_NAME);
}

void setup() {
  Serial.begin(115200);

  // WiFi wird nicht gebraucht -> spart Strom.
  // Bluetooth bleibt an, das brauchen wir jetzt für phyphox!
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

  for (int i = 0; i < MAX_ENTRIES; i++) {
    storeData(0, 0, 0, i, NAN, NAN, NAN);
  }

  setupPhyphoxBle();

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
  // Fallback-Trigger: alle paar Sekunden automatisch erneut senden, solange
  // verbunden - unabhaengig davon, ob der CCCD-Subscribe-Callback zuverlaessig
  // gefeuert hat. Kein delay(): reiner millis()-Vergleich.
  if (bleClientConnected && !historyReplayActive &&
      (millis() - lastAutoHistorySend >= AUTO_HISTORY_RESEND_INTERVAL_MS)) {
    startHistoryReplay();
    lastAutoHistorySend = millis();
  }

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
    startHistoryReplay(); // Taste dient auch als manuelles "jetzt an phyphox senden"
  }

  // Nicht-blockierende BLE-Zustandsautomaten - schicken pro Aufruf höchstens
  // ein Paket und kehren sofort zurück, kein delay() irgendwo im Ablauf.
  tickHistoryReplay();
  tickPhyphoxFileTransfer();

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

// === OLED Display Update ===
void updateDisplay(int myhead) {
  char lineBuf[48];
  snprintf(lineBuf, sizeof(lineBuf), "Time: %02u:%02u|%u %u",
           rrdBuffer[myhead].hour, rrdBuffer[myhead].minute,
           rrdBuffer[myhead].address1, rrdBuffer[myhead].address2);

  oled_display.clear();
  oled_display.setFont(ArialMT_Plain_10);
  oled_display.drawString(0, 0, lineBuf);
  battery();

  oled_display.setFont(ArialMT_Plain_24);

  float airTemps[MAX_SENSORS], waterTemps[MAX_SENSORS];
  classifySensors(rrdBuffer[myhead].address1, rrdBuffer[myhead].address2,
                   rrdBuffer[myhead].temperature1, rrdBuffer[myhead].temperature2,
                   airTemps, waterTemps);
  // Kleines Display hat nur Platz für je eine Zeile -> ersten gemeldeten
  // Sensor der Kategorie zeigen (die vollen 4 Kanäle sieht man in phyphox)
  float airTempC = firstValid(airTemps);
  float waterTempC = firstValid(waterTemps);

  char airBuf[16];
  char waterBuf[16];
  if (isnan(airTempC)) {
    snprintf(airBuf, sizeof(airBuf), "L: --°C");
  } else {
    snprintf(airBuf, sizeof(airBuf), "L: %.1f°C", airTempC);
  }
  if (isnan(waterTempC)) {
    snprintf(waterBuf, sizeof(waterBuf), "W: --°C");
  } else {
    snprintf(waterBuf, sizeof(waterBuf), "W: %.1f°C", waterTempC);
  }

  oled_display.drawString(0, 10, airBuf);
  oled_display.drawString(0, 34, waterBuf);
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

  // === Neue Historie an phyphox schicken (inkl. dieser neuen Messung) ===
  startHistoryReplay();

  // === OUTPUT ===
  Serial.println("---- LoRa Packet Received ----");
  Serial.printf("SensorID: %02X:%02X\n", sensorID1, sensorID2);
  Serial.printf("Time : %02d:%02d\n", hour, minute);
  Serial.printf("Battery: %.3f V\n", batteryVoltage);
  if (isnan(temp1)) {
    Serial.println("Temp1 : -127");
  } else {
    Serial.printf("Temp1 : %.1f\n", temp1);
  }
  if (isnan(temp2)) {
    Serial.println("Temp2 : -127");
  } else {
    Serial.printf("Temp2 : %.1f\n", temp2);
  }

  if (!displayOn) {
    VextON();
    oled_display.init();
    displayOn = true;
  }
  lastActivityTime = millis();
  updateDisplay(displayhead);
  state = STATE_RX;
}
