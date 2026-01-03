#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HX711.h>
#include <cmath>

// =============================================================
// HIVE / PILWF firmware (ESP32 + HX711)
//
// Packet format (12 bytes) matches Flutter ble_slot_page.dart:
//
//   0:0xCA 1:0xFE 2:slotId 3:flags
//   4-5: deltaMg (i16, signed)   = (weightG - baseG) * 1000
//   6-7: weightX10 (u16)         = weightG * 10
//   8-9: baseX10 (u16)           = baseG * 10
//   10: eventType (u8)           = command/status events
//   11: seq (u8)
//
// IMPORTANT: Flutter UI expects these flag bits:
//   bit0 TAKEN      (firmware does NOT auto-set yet)
//   bit1 REMOVED    (firmware does NOT auto-set yet)
//   bit2 UNEXPECTED (true when not stable / settling)
//   bit3 STABLE     (true when stable)
//
// For ZERO/TARE progress we use eventType only.
// =============================================================

// ================= UUIDs (match Flutter app) =================
static NimBLEUUID SERVICE_UUID("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b201");
static NimBLEUUID NOTIFY_UUID ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b202");
static NimBLEUUID CTRL_UUID   ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b203"); // write: "ZERO" / "TARE" / "CAL=..."
static NimBLEUUID CFG_UUID    ("7d2a0a5d-7c7a-4e8b-8cb4-2f2cf6b5b204"); // write: "SLOT=#"

// ================= HX711 wiring =================
// Remembered pins for ESP32: HX_DOUT=32, HX_SCK=33
static const int HX_DOUT = 32;
static const int HX_SCK  = 33;

// ================= Calibration factor =================
// Update via CTRL write: CAL=-87415.835938
static float g_calFactor = -87415.835938f;
static HX711 g_scale;

// ================= Identity =================
static char g_devName[32] = "Hive";
static NimBLEAdvertising* g_adv = nullptr;
static uint8_t g_slotId = 1;

// ================= BLE objects =================
static NimBLEServer* g_server = nullptr;
static NimBLEService* g_service = nullptr;
static NimBLECharacteristic* g_notifyCh = nullptr;
static NimBLECharacteristic* g_ctrlCh   = nullptr;
static NimBLECharacteristic* g_cfgCh    = nullptr;

static bool g_hasClient = false;
static uint8_t g_seq = 0;

// ================= Rates & filtering =================
static const uint32_t SAMPLE_HZ        = 40;     // internal reads
static const uint32_t BLE_HZ           = 10;     // notifications
static const float    EMA_ALPHA        = 0.12f;  // 0..1 lower=smoother

// Stability (tune these after real behavior)
static const float    STABLE_BAND_G    = 0.05f;  // +/- 0.05g
static const uint32_t STABLE_HOLD_MS   = 1200;

// Robust mean window for g_reportG
static const uint32_t AVG_WINDOW_MS    = 2000;
static const uint32_t AVG_MIN_SAMPLES  = 20;

// ZERO/TARE capture window
static const uint32_t PENDING_MS       = 2500;

// ================= Flags (byte 3) =================
static const uint8_t FLAG_TAKEN        = (1 << 0);
static const uint8_t FLAG_REMOVED      = (1 << 1);
static const uint8_t FLAG_UNEXPECTED   = (1 << 2);
static const uint8_t FLAG_STABLE       = (1 << 3);

// ================= Events (byte 10) =================
static const uint8_t EVT_NONE          = 0;
static const uint8_t EVT_TARE_DONE     = 1;
static const uint8_t EVT_ZERO_DONE     = 2;
static const uint8_t EVT_CAL_SET       = 3;
static const uint8_t EVT_ZERO_PENDING  = 10;
static const uint8_t EVT_TARE_PENDING  = 11;
static const uint8_t EVT_CMD_FAILED    = 99;

// ================= Scale state =================
static float g_baseG   = 0.0f;
static float g_rawG    = 0.0f;
static float g_emaG    = 0.0f;
static float g_reportG = 0.0f;

static uint8_t g_flags = 0;
static uint8_t g_eventType = EVT_NONE;

static bool g_isStable = false;
static uint32_t g_lastStableChangeMs = 0;

// ================= Ring buffer (trimmed mean) =================
struct Ring {
  static const int N = 180;
  float v[N];
  uint32_t t[N];
  int head = 0;
  int count = 0;

  void push(float val, uint32_t ms) {
    v[head] = val;
    t[head] = ms;
    head = (head + 1) % N;
    if (count < N) count++;
  }

  bool trimmedMean(uint32_t nowMs, uint32_t windowMs, float& outMean, int& outUsed) {
    float tmp[N];
    int n = 0;

    const uint32_t cutoff = nowMs - windowMs;
    for (int i = 0; i < count; i++) {
      int idx = head - 1 - i;
      if (idx < 0) idx += N;
      if (t[idx] < cutoff) break;
      tmp[n++] = v[idx];
    }
    if (n < 5) return false;

    // insertion sort (small n)
    for (int i = 1; i < n; i++) {
      float key = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = key;
    }

    int trim = (int)lroundf(n * 0.10f);
    int start = trim;
    int end   = n - trim;
    if (end <= start) { start = 0; end = n; }

    float sum = 0;
    int used = 0;
    for (int i = start; i < end; i++) { sum += tmp[i]; used++; }
    if (used <= 0) return false;

    outMean = sum / used;
    outUsed = used;
    return true;
  }
};

static Ring g_ring;

// ================= Helpers =================
static inline uint16_t u16_from_x10(float g) {
  int v = (int)lroundf(g * 10.0f);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

static inline int16_t i16_from_mg_delta(float deltaG) {
  long v = lroundf(deltaG * 1000.0f);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

static void buildPacket(uint8_t out[12]) {
  const float deltaG = g_reportG - g_baseG;
  const int16_t deltaMg = i16_from_mg_delta(deltaG);
  const uint16_t w10 = u16_from_x10(g_reportG);
  const uint16_t b10 = u16_from_x10(g_baseG);

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

// ================= Advertising =================
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

  Serial.print("BLE advertising as: ");
  Serial.println(g_devName);
}

// ================= Server callbacks =================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_hasClient = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    g_hasClient = false;
    Serial.print("BLE client disconnected, reason=");
    Serial.println(reason);
    if (g_adv) {
      g_adv->start();
      Serial.println("BLE advertising restarted");
    }
  }
};

// ================= ZERO/TARE pending =================
static bool g_pendingZero = false;
static bool g_pendingTare = false;
static uint32_t g_pendingStartMs = 0;

static void beginPendingZero() {
  g_pendingZero = true;
  g_pendingTare = false;
  g_pendingStartMs = millis();
  g_eventType = EVT_ZERO_PENDING;
  Serial.println("Pending ZERO (2.5s collect)...");
}

static void beginPendingTare() {
  g_pendingTare = true;
  g_pendingZero = false;
  g_pendingStartMs = millis();
  g_eventType = EVT_TARE_PENDING;
  Serial.println("Pending TARE (2.5s collect)...");
}

static void finishPendingIfReady() {
  if (!g_pendingZero && !g_pendingTare) return;

  const uint32_t now = millis();
  if (now - g_pendingStartMs < PENDING_MS) return;

  float mean = 0;
  int used = 0;
  bool ok = g_ring.trimmedMean(now, PENDING_MS, mean, used);

  if (ok && used >= 10) {
    if (g_pendingZero) {
      g_scale.tare(15);
      g_baseG = 0.0f;
      g_eventType = EVT_ZERO_DONE;
      Serial.println("ZERO done: tare() + base=0");
    } else if (g_pendingTare) {
      g_baseG = mean;
      g_eventType = EVT_TARE_DONE;
      Serial.print("TARE done: base=");
      Serial.println(g_baseG, 4);
    }
  } else {
    g_eventType = EVT_CMD_FAILED;
    Serial.println("Command failed: not enough samples");
  }

  g_pendingZero = false;
  g_pendingTare = false;
}

// ================= Characteristic callbacks =================
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) override { handleWrite(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override { handleWrite(c); }

private:
  void handleWrite(NimBLECharacteristic* c) {
    const std::string v = c->getValue();
    if (v.empty()) return;

    Serial.print("CTRL write: ");
    Serial.println(v.c_str());

    if (v == "ZERO") {
      beginPendingZero();
    } else if (v == "TARE") {
      beginPendingTare();
    } else if (v.rfind("CAL=", 0) == 0) {
      const float cf = atof(v.c_str() + 4);
      if (isfinite(cf) && fabsf(cf) > 0.00001f) {
        g_calFactor = cf;
        g_scale.set_scale(g_calFactor);
        g_eventType = EVT_CAL_SET;
        Serial.print("Calibration set: ");
        Serial.println(g_calFactor, 6);
      }
    }
  }
};

class CfgCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) override { handleWrite(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override { handleWrite(c); }

private:
  void handleWrite(NimBLECharacteristic* c) {
    const std::string v = c->getValue();
    if (v.empty()) return;

    Serial.print("CFG write: ");
    Serial.println(v.c_str());

    if (v.rfind("SLOT=", 0) == 0) {
      const int sid = atoi(v.c_str() + 5);
      if (sid > 0 && sid < 256) {
        g_slotId = (uint8_t)sid;
        Serial.print("SlotId set: ");
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

  const uint64_t chipId = ESP.getEfuseMac();
  snprintf(g_devName, sizeof(g_devName), "Hive_%02X%02X%02X",
           (uint8_t)(chipId >> 16),
           (uint8_t)(chipId >> 8),
           (uint8_t)(chipId));

  // HX711
  g_scale.begin(HX_DOUT, HX_SCK);
  g_scale.set_scale(g_calFactor);
  g_scale.tare(20);
  g_baseG = 0.0f;

  Serial.print("HX711 ready. calFactor=");
  Serial.println(g_calFactor, 6);

  // BLE
  NimBLEDevice::init(g_devName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  g_service = g_server->createService(SERVICE_UUID);

  g_notifyCh = g_service->createCharacteristic(
    NOTIFY_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  g_ctrlCh = g_service->createCharacteristic(
    CTRL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  g_ctrlCh->setCallbacks(new CtrlCallbacks());

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
  const uint32_t now = millis();

  // 1) Sample HX711 at SAMPLE_HZ
  static uint32_t lastSampleMs = 0;
  const uint32_t samplePeriodMs = 1000 / SAMPLE_HZ;

  if (now - lastSampleMs >= samplePeriodMs) {
    lastSampleMs = now;

    if (g_scale.is_ready()) {
      const float u = (float)g_scale.get_units(2); // grams
      g_rawG = u;

      g_ring.push(g_rawG, now);

      if (g_emaG == 0.0f) g_emaG = g_rawG;
      g_emaG = (EMA_ALPHA * g_rawG) + ((1.0f - EMA_ALPHA) * g_emaG);
    }
  }

  // 2) Reported weight = trimmed mean over AVG_WINDOW_MS (fallback to EMA)
  float mean = 0;
  int used = 0;
  const bool ok = g_ring.trimmedMean(now, AVG_WINDOW_MS, mean, used);
  if (ok && used >= (int)AVG_MIN_SAMPLES) g_reportG = mean;
  else g_reportG = g_emaG;

  // 3) Stable detection vs baseline
  const float deltaAbs = fabsf(g_reportG - g_baseG);
  const bool within = (deltaAbs <= STABLE_BAND_G);

  if (within) {
    if (!g_isStable) {
      if (g_lastStableChangeMs == 0) g_lastStableChangeMs = now;
      if (now - g_lastStableChangeMs >= STABLE_HOLD_MS) g_isStable = true;
    }
  } else {
    g_isStable = false;
    g_lastStableChangeMs = now;
  }

  // Only set stable/settling bits. Do NOT set TAKEN/REMOVED automatically here.
  g_flags &= (uint8_t)~(FLAG_UNEXPECTED | FLAG_STABLE);
  if (g_isStable) g_flags |= FLAG_STABLE;
  else            g_flags |= FLAG_UNEXPECTED;

  // 4) Finish pending ZERO/TARE
  finishPendingIfReady();

  // 5) Notify at BLE_HZ
  static uint32_t lastBleMs = 0;
  const uint32_t blePeriodMs = 1000 / BLE_HZ;

  if (now - lastBleMs >= blePeriodMs) {
    lastBleMs = now;

    if (g_notifyCh && g_hasClient) {
      uint8_t pkt[12];
      buildPacket(pkt);
      g_notifyCh->setValue(pkt, sizeof(pkt));
      g_notifyCh->notify();

      // Clear one-shot events after announce; keep pending repeating until done.
      if (g_eventType != EVT_ZERO_PENDING && g_eventType != EVT_TARE_PENDING) {
        g_eventType = EVT_NONE;
      }
    }

    // Debug
    Serial.print("raw=");  Serial.print(g_rawG, 3);
    Serial.print(" ema="); Serial.print(g_emaG, 3);
    Serial.print(" rep="); Serial.print(g_reportG, 3);
    Serial.print(" base=");Serial.print(g_baseG, 3);
    Serial.print(" d=");   Serial.print(g_reportG - g_baseG, 3);
    Serial.print(" stable="); Serial.print(g_isStable ? "Y" : "N");
    Serial.print(" flags=");  Serial.print(g_flags, BIN);
    Serial.print(" evt=");    Serial.print((int)g_eventType);
    Serial.print(" slot=");   Serial.println((int)g_slotId);
  }

  delay(2);
}