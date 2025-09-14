/**
 * Forest Guard - Node Agent (NA)  — ESP32‑S3
 * -----------------------------------------------------------
 * HW:
 *  - ESP32‑S3
 *  - Environmental sensor (DFRobot SEN050X family)
 *  - GNSS (DFRobot GNSS I2C – optional)
 *  - Analog smoke sensor on pin 5
 *  - Microphone (I2S) for Edge Impulse model on Core 1
 *  - RGB LED (WS2812/NeoPixel) on pin 6
 *  - Waveshare RP2040 LoRa module (Mestastics firmware) on UART1
 *
 * PROTOCOL (strictly between # and *; ignore all other garbage on the line):
 *  Registration:
 *    ->  NA: #<NODE_ID>*
 *    <-  GA: #<NODE_ID>+OK*
 *  Periodic telemetry (every 5 s once registered):
 *    ->  NA: #E,<NODE_ID>,<tempC:0.1>,<humidity:0.1>,<uv:0.01>,<lux:0>,<pressure:0.1>,<altitude:0.1>*
 *    ->  NA: #L,<NODE_ID>,<latDeg:0.000001>,<lonDeg:0.000001>*     (only when GNSS satsUsed > 3)
 *         If GNSS hardware is not present, NA will send the initial set location instead.
 *  Events (repeat every 10 s until CLEAR is received):
 *    ->  NA: #F+<id0_100>,<NODE_ID>,<smoke>,<YYYY>/<MM>/<DD>,<hh>:<mm>:<ss>*   or  #F+<id0_100>,<NODE_ID>,<smoke>,NT*
 *    ->  NA: #G+<id0_100>,<NODE_ID>,<score:0.00>,<YYYY>/<MM>/<DD>,<hh>:<mm>:<ss>*  or  #G+<id0_100>,<NODE_ID>,<score>,NT*
 *    <-  GA: #<NODE_ID>+C*   (clear current event; NA must unlatch and allow new events)
 *
 * LED semantics (non‑blocking “breathing” effect):
 *  - Blue breath      : boot + during registration / LoRa activity
 *  - Green breath     : briefly after each telemetry send (ENV/LOC)
 *  - Red breath       : while an event (FIRE/GUN) is latched
 *
 * Notes:
 *  - Edge Impulse and smoke monitoring are enabled ONLY AFTER successful node registration.
 *  - All timings are non‑blocking — no delay() in the hot path.
 */

#define EIDSP_QUANTIZE_FILTERBANK 0

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <Forest_Guard_Gunshot_Detection__inferencing.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------------------
// User Config
// -------------------------------
#define NODE_ID            "01"

// If GNSS hardware is connected set true, else false (optional module)
static const bool GNSS_AVAILABLE = true;

// Fallback / initial location (used when GNSS_AVAILABLE == false)
static const float INITIAL_LAT   = 17.6783352f;
static const float INITIAL_LON   = 77.6058542f;

// LoRa UART (Waveshare RP2040 LoRa w/ Mestastics)
HardwareSerial LORA(1);
#define LORA_TX_PIN 17
#define LORA_RX_PIN 16
#define LORA_BAUD   115200

// NeoPixel
#define NUM_LEDS   1
#define LED_PIN    6
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
uint8_t ledBrightness = 255;

// Smoke sensor
#define SMOKE_PIN 5  // GPIO 5 analog
static const int SMOKE_SAMPLES   = 4;
static const int SMOKE_THRESHOLD = 1000;
static const int SMOKE_HYST      = 60;

// Cadence
static const uint32_t ENV_MS       = 10000UL;
static const uint32_t REG_MS       = 10000UL;
static const uint32_t EVENT_MS     = 10000UL;     // repeat while latched
static const uint32_t GNSS_TRY_MS  = 30000UL ;      // read sats/LL every 5 s

// Gunshot
static const float    GUN_TRIGGER  = 0.92f;       // EI score threshold
static const float    SMOOTH_ALPHA = 0.6f;        // EMA

// I2S / audio
const i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int   SAMPLE_RATE   = EI_CLASSIFIER_FREQUENCY;
constexpr bool  MIC_ON_RIGHT  = false;

#ifndef I2S_BCK
  #define I2S_BCK  D11
#endif
#ifndef I2S_WS
  #define I2S_WS   D10
#endif
#ifndef I2S_SD
  #define I2S_SD   D9
#endif

// DFRobot sensors
#include "DFRobot_EnvironmentalSensor.h"
#include "DFRobot_GNSS.h"
DFRobot_EnvironmentalSensor environment(SEN050X_DEFAULT_DEVICE_ADDRESS, &Wire);
DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);

// -------------------------------
// State
// -------------------------------
volatile bool  registered = false;
volatile bool  fireLatched = false;
volatile bool  gunLatched  = false;
volatile float lastGunScore = 0.0f;
volatile int   lastSmokeAvg = 0;

enum EventType : uint8_t { EVT_NONE, EVT_FIRE, EVT_GUN };
volatile EventType currentEvt = EVT_NONE;
volatile int   currentEvtId   = -1; // 0..100
portMUX_TYPE   mux = portMUX_INITIALIZER_UNLOCKED;

// GNSS
volatile bool gnssUp = false;
volatile uint8_t satsUsed = 0;

// timers
uint32_t tLastEnv = 0;
uint32_t tLastEvent = 0;
uint32_t tLastReg = 0;
uint32_t tLastGnss = 0;
uint32_t tLastLed = 0;

// LED helpers
inline void ledShow(uint8_t r, uint8_t g, uint8_t b) { led.setPixelColor(0, led.Color(r,g,b)); led.show(); }

void ledBreath(uint32_t now, uint8_t r, uint8_t g, uint8_t b, uint16_t periodMs = 1500) {
  uint16_t t = now % periodMs;
  uint8_t bri = (t < periodMs/2) ? (t * 255) / (periodMs/2) : ((periodMs - t) * 255) / (periodMs/2);
  led.setPixelColor(0, led.Color((r*bri)/255, (g*bri)/255, (b*bri)/255));
  led.show();
}

uint32_t greenPulseUntil = 0;

// -------------------------------
// LoRa helpers (robust frame extraction)
// -------------------------------
String inBuf;

bool readLoraFrame(String &out) {
  while (LORA.available()) {
    char c = (char)LORA.read();
    if (c == '#') { inBuf = "#"; }
    else if (inBuf.length()) {
      inBuf += c;
      if (c == '*') {
        out = inBuf;
        inBuf = "";
        return true;
      }
      if (inBuf.length() > 128) inBuf.remove(0, 64); // keep it bounded
    }
  }
  return false;
}

inline void loraPrint(const char *s) { LORA.print(s); /*Serial.print("[LoRa->] "); Serial.println(s); */}
inline void loraPrintln(const char *s) { LORA.println(s);  /*Serial.print("[LoRa->] "); Serial.println(s); */}

// -------------------------------
// Telemetry senders
// -------------------------------
void sendEnv() {
  float tempC    = environment.getTemperature(TEMP_C);
  float humidity = environment.getHumidity();
  float uv       = environment.getUltravioletIntensity(eS12SD);
  float lux      = environment.getLuminousIntensity();
  float pressure = environment.getAtmospherePressure(HPA);
  float altitude = environment.getElevation();

  static char buf[128];
  snprintf(buf, sizeof(buf), "#E,%s,%.1f,%.1f,%.2f,%.0f,%.1f,%.1f*",
           NODE_ID, tempC, humidity, uv, lux, pressure, altitude);
  loraPrintln(buf);
  greenPulseUntil = millis() + 1200;
}

void sendLoc(float latDeg, float lonDeg) {
  static char buf[96];
  snprintf(buf, sizeof(buf), "#L,%s,%.6f,%.6f*", NODE_ID, latDeg, lonDeg);
  loraPrintln(buf);
  greenPulseUntil = millis() + 1200;
}

void sendFireEvent() {
  static char buf[128];
  int smoke = lastSmokeAvg;
  int id;
  portENTER_CRITICAL(&mux);
  id = currentEvtId;
  portEXIT_CRITICAL(&mux);

  if (gnssUp && satsUsed > 3) {
    sTim_t utc  = gnss.getUTC();
    sTim_t date = gnss.getDate();
    if (utc.hour || utc.minute || utc.second) {
      snprintf(buf, sizeof(buf), "#F+%d,%s,%d,%04d/%02d/%02d,%02d:%02d:%02d*",
               id, NODE_ID, smoke, date.year, date.month, date.date,
               utc.hour, utc.minute, utc.second);
      loraPrintln(buf); return;
    }
  }
  snprintf(buf, sizeof(buf), "#F+%d,%s,%d,NT*", id, NODE_ID, smoke);
  loraPrintln(buf);
}

void sendGunEvent() {
  static char buf[128];
  float score;
  int id;
  portENTER_CRITICAL(&mux);
  score = lastGunScore; id = currentEvtId;
  portEXIT_CRITICAL(&mux);

  if (gnssUp && satsUsed > 3) {
    sTim_t utc  = gnss.getUTC();
    sTim_t date = gnss.getDate();
    if (utc.hour || utc.minute || utc.second) {
      snprintf(buf, sizeof(buf), "#G+%d,%s,%.2f,%04d/%02d/%02d,%02d:%02d:%02d*",
               id, NODE_ID, score, date.year, date.month, date.date,
               utc.hour, utc.minute, utc.second);
      loraPrintln(buf); return;
    }
  }
  snprintf(buf, sizeof(buf), "#G+%d,%s,%.2f,NT*", id, NODE_ID, score);
  loraPrintln(buf);
}

// -------------------------------
// Sensors
// -------------------------------
bool tryInitEnv() { for (int i=0;i<5;i++){ if (environment.begin() == 0) return true; delay(200);} return false; }

bool tryInitGnss() {
  if (!GNSS_AVAILABLE) return false;
  for (uint8_t i=0;i<10;i++) {
    if (gnss.begin()) { gnss.enablePower(); gnss.setGnss(eGPS_BeiDou_GLONASS); gnss.setRgbOn(); return true; }
    delay(300);
  }
  return false;
}

bool getGnssLatLon(double &lat, double &lon) {
  sLonLat_t la = gnss.getLat();
  sLonLat_t lo = gnss.getLon();
  lat = la.latitudeDegree; lon = lo.lonitudeDegree;
  if (lat <= -90.0 || lat >= 90.0)   return false;
  if (lon <= -180.0 || lon >= 180.0) return false;
  return true;
}

int readSmokeAvg() {
  long sum = 0;
  for (int i=0;i<SMOKE_SAMPLES;i++){ sum += analogRead(SMOKE_PIN); delayMicroseconds(400); }
  return (int)(sum / SMOKE_SAMPLES);
}

bool smokeTriggered(int avg) {
  static bool above=false;
  if (!above && avg >= SMOKE_THRESHOLD) above = true;
  else if (above && avg <= (SMOKE_THRESHOLD - SMOKE_HYST)) above = false;
  return above;
}

// -------------------------------
// I2S init/deinit (Edge Impulse)
// -------------------------------
static int16_t *g_win = nullptr;
static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!g_win) return 0;
  numpy::int16_to_float(&g_win[offset], out_ptr, length);
  return 0;
}

static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (int)sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
#if ESP_IDF_VERSION_MAJOR >= 4
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
#else
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
#endif
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, nullptr) != ESP_OK) return ESP_FAIL;
  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) return ESP_FAIL;
  if (i2s_zero_dma_buffer(I2S_PORT) != ESP_OK) return ESP_FAIL;
  return ESP_OK;
}

static int i2s_deinit(void) { i2s_driver_uninstall(I2S_PORT); return 0; }

// -------------------------------
// Core 1: Edge Impulse loop
// -------------------------------
static void taskAudio(void *arg) {
  (void)arg;

  const uint32_t n_samples = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  int16_t *win = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!win) { /*Serial.println("ERR: audio buffer alloc");*/ vTaskDelete(nullptr); }
  g_win = win;
  uint32_t idx = 0;

  if (i2s_init(SAMPLE_RATE) != ESP_OK) { /*Serial.println("ERR: i2s_init");*/ vTaskDelete(nullptr); }

  bool smoothInit = false;
  float smooth[EI_CLASSIFIER_LABEL_COUNT] = {0};
  int gunLabel = -1;

  while (true) {
    size_t bytes_read = 0;
    esp_err_t r = i2s_read(I2S_PORT, &win[idx], (n_samples - idx) * sizeof(int16_t), &bytes_read, portMAX_DELAY);
    if (r != ESP_OK || bytes_read == 0) continue;

    idx += bytes_read / sizeof(int16_t);
    if (idx < n_samples) continue;
    idx = 0;

    if (!registered) continue; // do not evaluate before registration

    signal_t signal;
    signal.total_length = n_samples;
    signal.get_data = &ei_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR e = run_classifier(&signal, &result, false);
    if (e != EI_IMPULSE_OK) continue;

    if (gunLabel < 0) {
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        if (strcmp(result.classification[ix].label, "gun") == 0) { gunLabel = ix; break; }
    }

    float score = (gunLabel >= 0) ? result.classification[gunLabel].value : 0.0f;

    if (!smoothInit) { for (size_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++) smooth[i]=result.classification[i].value; smoothInit=true; }
    else { score = SMOOTH_ALPHA * score + (1.0f - SMOOTH_ALPHA) * lastGunScore; }

    portENTER_CRITICAL(&mux);
    lastGunScore = score;
    bool allowNewEvent = (!gunLatched && !fireLatched && currentEvt == EVT_NONE);
    if (allowNewEvent && score >= GUN_TRIGGER) {
      gunLatched = true;
      currentEvt  = EVT_GUN;
      currentEvtId = (int)(esp_random() % 101);
      // Serial.printf("🔫 Gunshot detected — score=%.2f, eventId=%d\n", score, currentEvtId);
    }
    portEXIT_CRITICAL(&mux);
  }
}

// -------------------------------
// Core 0: Node task
// -------------------------------
static void taskNode(void *arg) {
  (void)arg;

  Wire.begin();
  led.begin(); led.setBrightness(ledBrightness); ledShow(0,0,0);

  // Serial.println("== Forest Guard NA boot ==");
  LORA.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  // Serial.printf("LoRa UART on %d/%d @ %d\n", LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);

  // if (!tryInitEnv()) Serial.println("! Env sensor init FAILED"); else Serial.println("✓ Env sensor init OK");
  gnssUp = tryInitGnss();
  // Serial.printf("GNSS available: %s\n", gnssUp ? "YES" : "NO");

  // Immediately show initial location (fallback if no GNSS)
  if (!gnssUp && GNSS_AVAILABLE == false) {
    sendLoc(INITIAL_LAT, INITIAL_LON);
  }

  tLastReg = millis() - REG_MS; // send first registration right away

  // kick ENV after registration completes
  tLastEnv = millis();

  // Main loop
  while (true) {
    uint32_t now = millis();

    // LED
    if (now - tLastLed >= 20) {
      tLastLed = now;
      if (!registered)      ledBreath(now, 0, 0, 255);           // blue while waiting for GA
      else if (currentEvt != EVT_NONE) ledBreath(now, 255, 0, 0); // red while event
      else if (greenPulseUntil > now)  ledBreath(now, 0, 255, 0); // green pulse after data send
      else                             ledShow(0,0,0);
    }

    // Parse inbound LoRa frames
    String frame;
    while (readLoraFrame(frame)) {
      // keep only inside # ... *
      int hash = frame.indexOf('#');
      int star = frame.lastIndexOf('*');
      if (hash < 0 || star <= hash) continue;
      String body = frame.substring(hash + 1, star);
      body.trim();
      // Serial.printf("[LoRa<-] %s\n", body.c_str());

      // ACKS from Gateway
      if (body.equals(String(NODE_ID) + "+OK")) {
        registered = true;
        // Serial.println("✓ Registration ACK received. Sensors & EI enabled.");
        greenPulseUntil = now + 1500;
      }
      else if (body.equals(String(NODE_ID) + "+C")) {
        // Clear current event
        portENTER_CRITICAL(&mux);
        fireLatched = false; gunLatched = false;
        currentEvt = EVT_NONE; currentEvtId = -1;
        portEXIT_CRITICAL(&mux);
        // Serial.println("✓ CLEAR received from GA — events reset.");
        greenPulseUntil = now + 1500;
      }
    }

    // Registration heartbeat
    if (!registered && (now - tLastReg >= REG_MS)) {
      tLastReg = now;
      char buf[24]; snprintf(buf, sizeof(buf), "#%s*", NODE_ID);
      loraPrintln(buf);
    }

    if (registered) {
      // Periodic ENV + (maybe) LOC
      if (now - tLastEnv >= ENV_MS) {
        tLastEnv = now;
        sendEnv();

        // Location rules:
        //  - if GNSS chip up AND satsUsed > 3 -> send GNSS loc
        //  - else if GNSS not present -> send initial loc (so GA has something)
        if (gnssUp && (now - tLastGnss >= GNSS_TRY_MS)) {
          tLastGnss = now;
          satsUsed = gnss.getNumSatUsed();
          double lat, lon;
          if (satsUsed > 3 && getGnssLatLon(lat, lon)) {
            sendLoc((float)lat, (float)lon);
          }
        } else if (!gnssUp && GNSS_AVAILABLE == false) {
          sendLoc(INITIAL_LAT, INITIAL_LON);
        }
      }

      // Fire detection (simple analog threshold with hysteresis)
      int avg = readSmokeAvg();
      lastSmokeAvg = avg;
      bool trig = smokeTriggered(avg);

      if (trig && !fireLatched && currentEvt == EVT_NONE && !gunLatched) {
        portENTER_CRITICAL(&mux);
        fireLatched   = true;
        currentEvt    = EVT_FIRE;
        currentEvtId  = (int)(esp_random() % 101);
        portEXIT_CRITICAL(&mux);
        // Serial.printf("🔥 Fire detected — avg=%d, eventId=%d\n", avg, currentEvtId);
      }

      // Event cadence (repeat every 10 s)
      if (currentEvt != EVT_NONE && (now - tLastEvent >= EVENT_MS)) {
        tLastEvent = now;
        if (currentEvt == EVT_FIRE) sendFireEvent(); else sendGunEvent();
      }
    }

    vTaskDelay(1);
  }
}

// -------------------------------
// Arduino entry points
// -------------------------------
void setup() {
  // Serial.begin(115200);
  // delay(100);
  // Serial.println("\n[Forest Guard NA] ESP32-S3 booting...");
  pinMode(SMOKE_PIN, INPUT);

  xTaskCreatePinnedToCore(taskNode,  "node_task",  8192, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(taskAudio, "audio_task", 8192, nullptr, 11, nullptr, 1);
}

void loop() { vTaskDelay(1000); }

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
  #error "Invalid Edge Impulse model for microphone sensor."
#endif
