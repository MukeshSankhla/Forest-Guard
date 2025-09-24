/*
   Project   : Forest Guard
   Author    : Mukesh Sankhla
   Website   : www.makerbrains.com

   Description:
   This sketch streams raw audio data from the I²S microphone 
   over the serial port. The data can then be captured by a 
   Flask-based PC tool and uploaded to Edge Impulse using the 
   Edge Impulse API for model training.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <FastLED.h>

// ====== CONFIG ======
const i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int SAMPLE_RATE   = 16000;     // Hz
constexpr int SECONDS_TO_GRAB = 10;      // 10 seconds, change if needed
constexpr bool MIC_ON_RIGHT = false;     // set true if your mic is wired to RIGHT

// Replace these with your actual ESP32 GPIO numbers
#define I2S_BCK  D11
#define I2S_WS   D10
#define I2S_SD   D12

#define LED_DATA_PIN 6
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// I2S DMA settings – larger buffers avoid dropouts
constexpr int DMA_BUF_COUNT = 8;         // 6–12 is typical
constexpr int DMA_BUF_LEN   = 256;       // samples per DMA buffer (32‑bit frames)

// Bytes we expect to send to the host as 16‑bit PCM
constexpr size_t TOTAL_SAMPLES = SAMPLE_RATE * SECONDS_TO_GRAB;
constexpr size_t BYTES_TO_SEND = TOTAL_SAMPLES * sizeof(int16_t);

static inline int16_t s32_to_s16(int32_t s32) {
  // For most I2S MEMS mics (24‑bit left‑justified in 32‑bit frame),
  // shifting by ~11 keeps good dynamic range without clipping.
  // You can adjust between 8–16 depending on mic gain.
  return (int16_t)(s32 >> 11);
}


void setup() {
  Serial.begin(921600); // fast binary pipe
  // Optional: wait for terminal
  // while(!Serial) { delay(10); }

  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(1000);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
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
    .data_out_num = -1,     // not used
    .data_in_num = I2S_SD
  };

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, nullptr) != ESP_OK) {
    Serial.println("ERR i2s_driver_install");
    while (true) { delay(1000); }
  }
  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("ERR i2s_set_pin");
    while (true) { delay(1000); }
  }
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("READY");

  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}

void capture_and_send_10s() {
  // Announce exact payload size so the host can read binary safely
  Serial.print("START_AUDIO ");
  Serial.println((unsigned)BYTES_TO_SEND);

  constexpr size_t READ_CHUNK_BYTES = 1024;        // read 32‑bit frames from I2S
  alignas(4) uint8_t i2s_raw[READ_CHUNK_BYTES];
  size_t samples_sent = 0;

  while (samples_sent < TOTAL_SAMPLES) {
    size_t bytes_read = 0;
    // Blocking read from I2S
    esp_err_t r = i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &bytes_read, portMAX_DELAY);
    if (r != ESP_OK || bytes_read == 0) continue;

    // Convert 32‑bit I2S sample frames to 16‑bit PCM on the fly
    size_t frames = bytes_read / 4; // 32‑bit per frame
    for (size_t i = 0; i < frames && samples_sent < TOTAL_SAMPLES; ++i) {
      int32_t s32;
      memcpy(&s32, &i2s_raw[i * 4], 4);
      int16_t s16 = s32_to_s16(s32);
      Serial.write((uint8_t*)&s16, sizeof(s16));
      samples_sent++;
    }
  }

  Serial.println();
  Serial.println("END_AUDIO");
}

void loop() {
  // Simple command interface
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CAPTURE_AUDIO") {
      leds[0] = CRGB::Green;
      FastLED.show();

      capture_and_send_10s();

      leds[0] = CRGB::Black;
      FastLED.show();
    }
  }
}