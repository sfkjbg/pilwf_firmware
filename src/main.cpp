#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HX711.h>
#include <cmath>

// ================= UUIDs (match Flutter app) =================
static NimBLEUUID SERVICE_UUID("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b201");
static NimBLEUUID NOTIFY_UUID ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b202");
static NimBLEUUID CTRL_UUID   ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b203"); // write commands: TARE / ZERO
static NimBLEUUID CFG_UUID    ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b204"); // optional: set SLOT

// ================= HX711 wiring =================
// Set these to your actual pins
// static const int HX_DOUT = 4;
// static const int HX_SCK  = 5;
static const int HX_DOUT = 32;
static const int HX_SCK  = 33;

// Calibration factor:
// - You MUST tune this for your load cell.
// - Start with 1.0, then adjust until grams read correctly.
static float g_calFactor = 1.0f;

static HX711 g_scale;

// ================= Device identity =================
static char g_devName[32] = "Hive";
static NimBLEAdvertising* g_adv = nullptr;

// Slot id (what the device represents). You can change in-app later.
static uint8_t g_slotId = 1;

// ================= BLE objects =================
static NimBLEServer* g_server = nullptr;
static NimBLEService* g_service = nullptr;
static NimBLECharacteristic* g_notifyCh = nullptr;
static NimBLECharacteristic* g_ctrlCh   = nullptr;
static NimBLECharacteristic* g_cfgCh    = nullptr;

static bool g_hasClient = false;
static uint8_t g_seq = 0;

// ================= Scale state =================
// Base is the "baseline" value used by the app (NOT the HX711 offset)
static float g_baseG = 0.0f;
static float g_lastWeightG = 0.0f;

// Flags bitfield expected by the app
// bit0 TAKEN, bit1 REMOVED, bit2 UNEXPECTED, bit3 STABLE
static uint8_t g_flags = (1 << 3);

// eventType is optional for your app right now
static uint8_t g_eventType = 0;

// How long to “watch” readings for ZERO/TARE behavior
static const uint32_t ZERO_TARE_WINDOW_MS = 2500;

// ================= Helpers =================
static inline uint16_t u16_from_x10(float g) {
  int v = (int)lroundf(g * 10.0f);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

static inline int16_t i16_from_mg_delta(float deltaG) {
  // App interprets deltaMg as signed int16, and prints deltaMg/1000.0 as grams
  long v = lroundf(deltaG * 1000.0f);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

static void buildPacket(uint8_t out[12], float weightG, float baseG) {
  // Format must match Flutter SlotPacket.parse()
  // [0..1]=0xCA,0xFE
  // [2]=slotId
  // [3]=flags
  // [4..5]=deltaMg (int16 LE)
  // [6..7]=weightX10 (uint16 LE)
  // [8..9]=baseX10 (uint16 LE)
  // [10]=eventType
  // [11]=seq

  const float deltaG = weightG - baseG;
  const int16_t deltaMg = i16_from_mg_delta(deltaG);
  const uint16_t w10 = u16_from_x10(weightG);
  const uint16_t b10 = u16_from_x10(baseG);

  out[0] = 0xCA;
  out[1] = 0xFE;
  out[2] = g_slotId;
  out[3] = g_flags;

  out[4] = (uint8_t)(deltaMg & 0xFF);
  out[5] = (uint8_t)((deltaMg >> 8) & 0xFF);

  out[6] = (uint8_t)(w10 & 0xFF);
  out[7] = (uint8_t)((w10 >> 8) & 0xFF);

  out[8] = (uint8_t)(b10 & 0xFF);
  out[9] = (uint8_t)((b10 >> 8) & 0xFF);

  out[10] = g_eventType;
  out[11] = g_seq++;
}

// Average RAW ADC readings for a window (used by ZERO)
static long sampleAvgRaw(uint32_t windowMs) {
  const uint32_t t0 = millis();
  long sum = 0;
  uint32_t n = 0;

  while (millis() - t0 < windowMs) {
    if (g_scale.is_ready()) {
      sum += g_scale.read(); // RAW
      n++;
    }
    delay(10);
  }

  return (n > 0) ? (sum / (long)n) : 0;
}

// Average calibrated units for a window (used by TARE baseline)
static float sampleAvgUnits(uint32_t windowMs) {
  const uint32_t t0 = millis();
  double sum = 0.0;
  uint32_t n = 0;

  while (millis() - t0 < windowMs) {
    if (g_scale.is_ready()) {
      sum += (double)g_scale.get_units(1); // "grams" if calFactor correct
      n++;
    }
    delay(10);
  }

  return (n > 0) ? (float)(sum / (double)n) : 0.0f;
}

// ================= Advertising helper =================
static void startAdvertising() {
  g_adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setName(g_devName);
  advData.addServiceUUID(SERVICE_UUID);
  g_adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(g_devName);
  g_adv->setScanResponseData(scanResp);

  g_adv->start();

  Serial.print("BLE advertising started as: ");
  Serial.println(g_devName);
}

// ================= Server callbacks =================
// NimBLE-Arduino 2.x uses NimBLEConnInfo.
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    (void)pServer;
    (void)connInfo;
    g_hasClient = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    g_hasClient = false;
    Serial.print("BLE client disconnected, reason=");
    Serial.println(reason);

    if (g_adv) {
      g_adv->start();
      Serial.println("BLE advertising restarted");
    }
  }
};

// ================= Characteristic callbacks =================
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
public:
  // Provide both overloads (avoid fragile 'override' signature issues across NimBLE-Arduino versions)
  void onWrite(NimBLECharacteristic* c) { handleWrite(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) { (void)connInfo; handleWrite(c); }

private:
  void handleWrite(NimBLECharacteristic* c) {
    const std::string v = c->getValue();
    if (v.empty()) return;

    Serial.print("CTRL write: ");
    Serial.println(v.c_str());

    // ✅ Desired behavior:
    // - ZERO: wait/average 2.5s, then set HX711 offset so reading becomes ~0
    // - TARE: average 2.5s, then set baseline to that average (does NOT change HX711 offset)

    if (v == "ZERO") {
      Serial.println("ZERO: averaging RAW for 2.5s, then setting HX711 offset so weight reads ~0");

      long avgRaw = sampleAvgRaw(ZERO_TARE_WINDOW_MS);

      // Most HX711 libs support set_offset(). If yours doesn't compile, tell me the error
      // and I'll adapt to that library's API.
      g_scale.set_offset(avgRaw);

      // After zeroing, baseline should be 0
      g_baseG = 0.0f;
      g_eventType = 2;

      Serial.print("ZERO done. avgRaw=");
      Serial.println(avgRaw);

    } else if (v == "TARE") {
      Serial.println("TARE: averaging UNITS for 2.5s, then setting baseline to that average");

      float avgG = sampleAvgUnits(ZERO_TARE_WINDOW_MS);
      g_baseG = avgG;
      g_eventType = 1;

      Serial.print("TARE done. baseline(g)=");
      Serial.println(g_baseG, 3);

    } else if (v.rfind("CAL=", 0) == 0) {
      const float cf = atof(v.c_str() + 4);
      if (isfinite(cf) && fabsf(cf) > 0.00001f) {
        g_calFactor = cf;
        g_scale.set_scale(g_calFactor);
        Serial.print("Calibration factor set to: ");
        Serial.println(g_calFactor, 6);
        g_eventType = 3;
      }
    }
  }
};

class CfgCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) { handleWrite(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) { (void)connInfo; handleWrite(c); }

private:
  void handleWrite(NimBLECharacteristic* c) {
    const std::string v = c->getValue();
    if (v.empty()) return;

    Serial.print("CFG write: ");
    Serial.println(v.c_str());

    // Accept: "SLOT=3"
    if (v.rfind("SLOT=", 0) == 0) {
      const int sid = atoi(v.c_str() + 5);
      if (sid > 0 && sid < 256) {
        g_slotId = (uint8_t)sid;
        Serial.print("SlotId set to: ");
        Serial.println(g_slotId);
      }
    }
  }
};

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HIVE ESP32 BOOT ===");

  // Unique device name from MAC so multiple units show in scan list
  const uint64_t chipId = ESP.getEfuseMac();
  snprintf(g_devName, sizeof(g_devName), "Hive_%02X%02X%02X",
           (uint8_t)(chipId >> 16),
           (uint8_t)(chipId >> 8),
           (uint8_t)(chipId));

  // Init HX711
  g_scale.begin(HX_DOUT, HX_SCK);
  g_scale.set_scale(g_calFactor);

  // Do an initial zero-ish using library tare to start sane
  g_scale.tare(15);
  g_baseG = 0.0f;

  Serial.println("HX711 ready (initial tare done)");

  // Init BLE
  NimBLEDevice::init(g_devName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, false);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  g_service = g_server->createService(SERVICE_UUID);

  // Notify characteristic (read + notify)
  g_notifyCh = g_service->createCharacteristic(
    NOTIFY_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  // Control characteristic (write)
  g_ctrlCh = g_service->createCharacteristic(
    CTRL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  g_ctrlCh->setCallbacks(new CtrlCallbacks());

  // Config characteristic (write + read)
  g_cfgCh = g_service->createCharacteristic(
    CFG_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  g_cfgCh->setCallbacks(new CfgCallbacks());

  g_service->start();

  startAdvertising();
}

// ================= Loop =================
void loop() {
  static uint32_t lastMs = 0;
  static uint32_t lastDbgMs = 0;
  const uint32_t now = millis();

  // ~5Hz notify updates
  if (now - lastMs >= 200) {
    lastMs = now;

    float w = g_lastWeightG;

    if (g_scale.is_ready()) {
      w = (float)g_scale.get_units(5);
      g_lastWeightG = w;
    }

    // Mark stable if current weight is close to baseline
    const float deltaAbs = fabsf(w - g_baseG);
    if (deltaAbs < 0.05f) {
      g_flags = (1 << 3); // stable
    } else {
      g_flags = (1 << 2); // unexpected change
    }

    // Send packet if connected
    if (g_notifyCh && g_hasClient) {
      uint8_t pkt[12];
      buildPacket(pkt, w, g_baseG);
      g_notifyCh->setValue(pkt, sizeof(pkt));
      g_notifyCh->notify();
      g_eventType = 0; // clear one-shot
    }

    // Serial weight line (so you can confirm it's alive)
    Serial.print("SERIAL_WEIGHT g=");
    Serial.print(w, 3);
    Serial.print("  baseline=");
    Serial.print(g_baseG, 3);
    Serial.print("  delta=");
    Serial.print((w - g_baseG), 3);
    Serial.print("  flags=");
    Serial.print((int)g_flags, BIN);
    Serial.print("  slot=");
    Serial.println(g_slotId);
  }

  // Occasional HX711 deeper debug
  if (now - lastDbgMs >= 1200) {
    lastDbgMs = now;

    bool rdy = g_scale.is_ready();
    long raw = 0;
    if (rdy) raw = g_scale.read();

    Serial.print("HX711 dbg ready=");
    Serial.print(rdy ? "YES" : "NO");
    Serial.print(" raw=");
    Serial.print(raw);
    Serial.print(" units=");
    Serial.print(g_lastWeightG, 3);
    Serial.print("  baseline=");
    Serial.print(g_baseG, 3);
    Serial.print("  delta=");
    Serial.print((g_lastWeightG - g_baseG), 3);
    Serial.print("  flags=");
    Serial.print((int)g_flags, BIN);
    Serial.print("  hasClient=");
    Serial.println(g_hasClient ? "YES" : "NO");
  }

  delay(5);
}