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
static const uint32_t REG_MS       = 5000UL;
static const uint32_t EVENT_MS     = 5000UL;     // repeat while latched
static const uint32_t GNSS_TRY_MS  = 15000UL;     // try GNSS periodically

// Gunshot
static const float    GUN_TRIGGER  = 0.90f;       // EI score threshold
static const float    SMOOTH_ALPHA = 0.60f;       // EMA for scores

// I2S / audio
const i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int   SAMPLE_RATE   = EI_CLASSIFIER_FREQUENCY;
constexpr bool  MIC_ON_RIGHT  = false;

constexpr int DMA_BUF_COUNT = 8;      // 6–12 typical
constexpr int DMA_BUF_LEN   = 256;    // frames per DMA buffer (32-bit frames)

// Convert 32-bit I2S frames to 16-bit PCM using right shift
// Adjust shift (8–16) based on your mic gain to avoid clipping / noise floor
constexpr uint8_t AUDIO_SHIFT = 11;   // same as your working example

#define I2S_BCK  D11
#define I2S_WS   D10
#define I2S_SD   D12

// DFRobot sensors
#include "DFRobot_EnvironmentalSensor.h"
#include "DFRobot_GNSS.h"
DFRobot_EnvironmentalSensor environment(SEN050X_DEFAULT_DEVICE_ADDRESS, &Wire);
DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);

// -------------------------------
/* State */
// -------------------------------
volatile bool  registered = false;
volatile bool  fireLatched = false;
volatile bool  gunLatched  = false;
volatile float lastGunScore = 0.0f;   // now stores *smoothed* Gun score
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

inline void loraPrint(const char *s) { LORA.print(s); }
inline void loraPrintln(const char *s) { LORA.println(s); }

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
static int16_t *g_win = nullptr;                 // EI window (int16 mono)
static const size_t I2S_READ_CHUNK_FRAMES = 256; // DMA read granularity (32-bit frames)
static uint32_t     i2s_raw[I2S_READ_CHUNK_FRAMES];

static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!g_win) return 0;
  numpy::int16_to_float(&g_win[offset], out_ptr, length);
  return 0;
}

static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (int)sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = MIC_ON_RIGHT ? I2S_CHANNEL_FMT_ONLY_RIGHT : I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
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

// Local helper to convert 32-bit I2S frame to int16 with gain shift
static inline int16_t s32_to_s16(int32_t s32) {
  return (int16_t)(s32 >> AUDIO_SHIFT);
}

// -------------------------------
// Core 1: Edge Impulse loop  (fixed capture + decision)
// -------------------------------
static void taskAudio(void *arg) {
  (void)arg;

  // Allocate EI window (int16 mono)
  const uint32_t n_samples = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  int16_t *win = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!win) { vTaskDelete(nullptr); }
  g_win = win;

  // Init I2S
  if (i2s_init(SAMPLE_RATE) != ESP_OK) { vTaskDelete(nullptr); }

  // Smoothing across all labels
  bool  smoothInit = false;
  float smooth[EI_CLASSIFIER_LABEL_COUNT] = {0};
  int   gunLabel = -1;

  // DMA read buffer: read 32-bit frames, then convert to int16 with right-shift
  static const size_t I2S_READ_CHUNK_FRAMES = 256;
  uint32_t i2s_raw[I2S_READ_CHUNK_FRAMES];

  uint32_t idx = 0;
  size_t bytes_read = 0;

  while (true) {
    // Fill the EI window from 32-bit I2S frames
    while (idx < n_samples) {
      esp_err_t r = i2s_read(I2S_PORT, (void*)i2s_raw, sizeof(i2s_raw), &bytes_read, portMAX_DELAY);
      if (r != ESP_OK || bytes_read == 0) continue;

      size_t frames = bytes_read / sizeof(uint32_t);
      for (size_t i = 0; i < frames && idx < n_samples; ++i) {
        int32_t s32 = (int32_t)i2s_raw[i];
        win[idx++]  = s32_to_s16(s32);   // uses AUDIO_SHIFT internally
      }
    }

    // Full window captured; reset for the next one immediately
    idx = 0;

    // Do not run EI until the node has registered
    if (!registered) {
      // Drop the window and keep capturing
      continue;
    }

    // Wrap buffer for EI
    signal_t signal;
    signal.total_length = n_samples;
    signal.get_data     = &ei_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR e = run_classifier(&signal, &result, false);
    if (e != EI_IMPULSE_OK) {
      // If classification fails, just continue with next window
      continue;
    }

    // Resolve the "Gun" label index once
    if (gunLabel < 0) {
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (strcmp(result.classification[ix].label, "Gun") == 0) { gunLabel = ix; break; }
      }
    }

    // Initialize or update exponential smoothing over all labels
    if (!smoothInit) {
      for (size_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++) {
        smooth[i] = result.classification[i].value;
      }
      smoothInit = true;
    } else {
      for (size_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++) {
        smooth[i] = SMOOTH_ALPHA * result.classification[i].value
                  + (1.0f - SMOOTH_ALPHA) * smooth[i];
      }
    }

    // Top-1 (smoothed)
    size_t best_ix = 0; float best_val = smooth[0];
    for (size_t i=1;i<EI_CLASSIFIER_LABEL_COUNT;i++) {
      if (smooth[i] > best_val) { best_val = smooth[i]; best_ix = i; }
    }

    // Smoothed "Gun" score
    float gun_smoothed = (gunLabel >= 0) ? smooth[gunLabel] : 0.0f;

    // Share score & possibly latch event
    portENTER_CRITICAL(&mux);
    lastGunScore = gun_smoothed;  // report the smoothed value upstream

    bool allowNewEvent = (!gunLatched && !fireLatched && currentEvt == EVT_NONE);
    bool isGunTop      = (gunLabel >= 0) && (best_ix == (size_t)gunLabel);

    if (allowNewEvent && isGunTop && gun_smoothed >= GUN_TRIGGER) {
      gunLatched   = true;
      currentEvt   = EVT_GUN;
      currentEvtId = (int)(esp_random() % 101);
      // LED breath switches to red via main loop while event is latched
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

  LORA.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  // Initialize sensors
  (void)tryInitEnv();
  gnssUp = tryInitGnss();

  // Immediately show initial location (fallback if no GNSS)
  if (!gnssUp && GNSS_AVAILABLE == false) {
    sendLoc(INITIAL_LAT, INITIAL_LON);
  }

  tLastReg = millis() - REG_MS; // send first registration right away
  tLastEnv = millis();

  // Main loop
  while (true) {
    uint32_t now = millis();

    // LED
    if (now - tLastLed >= 20) {
      tLastLed = now;
      if (!registered)                  ledBreath(now, 0, 0, 255); // blue while waiting for GA
      else if (currentEvt != EVT_NONE)  ledBreath(now, 255, 0, 0); // red while event
      else if (greenPulseUntil > now)   ledBreath(now, 0, 255, 0); // green pulse after data send
      else                              ledShow(0,0,0);
    }

    // Parse inbound LoRa frames
    String frame;
    while (readLoraFrame(frame)) {
      int hash = frame.indexOf('#');
      int star = frame.lastIndexOf('*');
      if (hash < 0 || star <= hash) continue;
      String body = frame.substring(hash + 1, star);
      body.trim();

      // ACKS from Gateway
      if (body.equals(String(NODE_ID) + "+OK")) {
        registered = true;
        greenPulseUntil = now + 1500;
      }
      else if (body.equals(String(NODE_ID) + "+C")) {
        // Clear current event
        portENTER_CRITICAL(&mux);
        fireLatched = false; gunLatched = false;
        currentEvt = EVT_NONE; currentEvtId = -1;
        portEXIT_CRITICAL(&mux);
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
      }

      // Event cadence (repeat every 10 s)
      if (currentEvt != EVT_NONE && (now - tLastEvent >= EVENT_MS)) {
        tLastEvent = now;
        if (currentEvt == EVT_FIRE) sendFireEvent();
        else                        sendGunEvent();
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
  // delay(500);
  pinMode(SMOKE_PIN, INPUT);
  xTaskCreatePinnedToCore(taskNode,  "node_task",  8192, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(taskAudio, "audio_task", 8192, nullptr, 11, nullptr, 1);
}

void loop() { vTaskDelay(1000); }

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
  #error "Invalid Edge Impulse model for microphone sensor."
#endif
