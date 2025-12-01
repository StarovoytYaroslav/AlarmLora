#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Mak_Ans_aj24-project-1_inferencing.h>
#include <TinyGPS++.h>
#include <driver/i2s.h>

// --------------------------------------------------------------------------
// 🔴 TODO: INCLUDE YOUR EDGE IMPULSE LIBRARY HERE
// --------------------------------------------------------------------------
// Example: #include <My_Audio_Project_inferencing.h>
// #include <Your_Project_Name_inferencing.h>

// If you haven't imported the library yet, uncomment these defines to make it compile
// so you can test the hardware (Mocking the AI library)
#ifndef _EDGE_IMPULSE_RUN_CLASSIFIER_H_
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT 16000
typedef struct
{
  int value;
} signal_t;
typedef struct
{
  float value;
  char *label;
} ei_impulse_result_classification_t;
typedef struct
{
  ei_impulse_result_classification_t *classification;
} ei_impulse_result_t;
void run_classifier_init() {}
void run_classifier(signal_t *s, ei_impulse_result_t *r, bool b) {}
#endif

// ==================================================
// 2. GLOBAL OBJECTS & SHARED DATA
// ==================================================
SemaphoreHandle_t aiMutex;
String labelNoise = "Silence";
String labelSteps = "Steps";
float conf[2] = {0.0};



#define NODE_ID 2
#define GPS_BAUD 9600

SPIClass loraSPI(1);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

// Audio Configuration
#define ADC_FS 8000 // Sample rate
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLES_PER_READ 1024
#define EI_FS (EI_CLASSIFIER_FREQUENCY) // очікує 1000 Гц
#define TRESHOLD 0.75
_Static_assert(ADC_FS % EI_FS == 0, "ADC_FS must be an integer multiple of EI_FS");
static const uint32_t DECIM = (ADC_FS / EI_FS); // 8 або 16

// Inference Struct
typedef struct
{
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

inference_t inference;
volatile bool record_status = false;
int32_t sampleBuffer[I2S_SAMPLES_PER_READ]; // 32-bit buffer for I2S read

// ==================================================
// 3. AUDIO HELPER FUNCTIONS
// ==================================================

typedef struct
{
  float b0, b1, b2, a1, a2;
  float z1, z2; // DF2T
} biquad_t;

static biquad_t lpf1, lpf2;

static inline float biquad_process(biquad_t *s, float x)
{
  float y = s->b0 * x + s->z1;
  s->z1 = s->b1 * x - s->a1 * y + s->z2;
  s->z2 = s->b2 * x - s->a2 * y;
  return y;
}

// Placeholder LPF init (Add your DSP logic here if needed)
void lpf_init_for_fs(uint32_t fs)
{
  // 2-й порядок LPF, fc ≈ 400 Гц, Q ≈ 0.707; каскадуємо двічі
  if (fs == 8000)
  {
    // розраховано за RBJ cookbook
    lpf1 = (biquad_t){.b0 = 0.020083365558f, .b1 = 0.040166731116f, .b2 = 0.020083365558f, .a1 = -1.561018075331f, .a2 = 0.641351537564f, .z1 = 0, .z2 = 0};
  }
  else
  { // 16000
    lpf1 = (biquad_t){.b0 = 0.005542717209f, .b1 = 0.011085434419f, .b2 = 0.005542717209f, .a1 = -1.778631777527f, .a2 = 0.800802646365f, .z1 = 0, .z2 = 0};
  }
  lpf2 = lpf1; // той самий бікуад вдруге → 4-й порядок
}

static uint32_t decim_phase = 0;

static inline int16_t float_to_i16_clipped(float v)
{
  v *= 32767.0f;
  if (v > 32767.0f)
    v = 32767.0f;
  if (v < -32768.0f)
    v = -32768.0f;
  return (int16_t)v;
}

// Decimator: Converts 32-bit I2S data to 16-bit for Edge Impulse
static void decimator_feed(const int32_t *in, size_t n_in_samples)
{
  for (size_t i = 0; i < n_in_samples; i++)
  {
    // PCM1808: ефективні 24 біти у 32-біт слові → зсуваємо
    float x = (float)(in[i] >> 8) * (1.0f / 8388608.0f);

    // 4-го порядку LPF
    float y = biquad_process(&lpf2, biquad_process(&lpf1, x));

    if ((decim_phase++ % DECIM) == 0)
    {
      if (inference.buf_count < inference.n_samples)
      {
        inference.buffer[inference.buf_count++] = float_to_i16_clipped(y);
      }
      if (inference.buf_count >= inference.n_samples)
      {
        inference.buf_count = 0;
        inference.buf_ready = 1;
      }
    }
  }
}

static int i2s_init(uint32_t sampling_rate)
{
  i2s_config_t i2s_cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = sampling_rate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = true,
      .tx_desc_auto_clear = false,
      .fixed_mclk = sampling_rate * 256};

  i2s_pin_config_t pin_cfg = {
      .mck_io_num = I2S_SCK_PIN,
      .bck_io_num = I2S_BCK_PIN,
      .ws_io_num = I2S_LRC_PIN,
      .data_out_num = -1,
      .data_in_num = I2S_DIN_PIN};

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_cfg, 0, NULL);
  if (err != ESP_OK)
    return err;

  err = i2s_set_pin(I2S_PORT, &pin_cfg);
  if (err != ESP_OK)
    return err;

  Serial.println("I2S started. Expect ~2.048 MHz on GPIO1 (MCLK).");

  return 0;
}

static int i2s_deinit(void)
{
  i2s_driver_uninstall(I2S_PORT);
  return 0;
}

// Task that runs ONLY during recording
static void capture_samples(void *arg)
{
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {

    /* read data at once from i2s */
    esp_err_t ok = i2s_read(I2S_PORT, sampleBuffer, i2s_bytes_to_read, &bytes_read, portMAX_DELAY);
    if (ok != ESP_OK || bytes_read == 0) {
      ei_printf("Error in I2S read : %d\n", (int)bytes_read);
      continue;
    }
    if (bytes_read < (size_t)i2s_bytes_to_read) {
      ei_printf("Partial I2S read\n");
    }

    // скільки 32-біт слів отримали
    size_t n = bytes_read / sizeof(int32_t);
    decimator_feed(sampleBuffer, n);
    //stream_raw_pcm_to_serial(sampleBuffer, n);
  }
  vTaskDelete(NULL);
}

// Edge Impulse Glue
static bool microphone_inference_start(uint32_t n_samples)
{
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
  if (inference.buffer == NULL)
    return false;

  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  lpf_init_for_fs(ADC_FS);

  if (i2s_init(ADC_FS) != 0)
  {
    Serial.println("Failed to start I2S!");
    return false;
  }

  ei_sleep(100);

  record_status = true;
  const uint32_t bytes_per_read = I2S_SAMPLES_PER_READ * sizeof(int32_t);

  // Launch the capture task
  xTaskCreate(capture_samples, "Capture", 4096, (void *)bytes_per_read, 10, NULL);
  return true;
}

static bool microphone_inference_record(void)
{
  while (inference.buf_ready == 0)
  {
    vTaskDelay(10);
  }
  return true;
}

static void microphone_inference_end(void)
{
  record_status = false;
  vTaskDelay(50); // Give capture task time to finish
  i2s_deinit();
  free(inference.buffer);
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  // Convert int16 buffer to float for AI
  for (size_t i = 0; i < length; i++)
  {
    out_ptr[i] = (float)inference.buffer[offset + i];
  }
  return 0;
}

// ==================================================
// 4. TASK AI (CORE 0)
// ==================================================
void taskAI(void *pvParameters)
{
  Serial.println("[AI] Task Started on Core 0");
  microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT);
  run_classifier_init();

  while (true)
  {
      microphone_inference_record();

      // 2. Run Inference
      signal_t signal;
      signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
      signal.get_data = &microphone_audio_signal_get_data;
      ei_impulse_result_t result = {0};

      // run_classifier blocks here for processing time
      run_classifier(&signal, &result, false);
      int best = 0;
      float best_val = result.classification[0].value;

      // print the predictions
      ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
      ei_printf(": \n");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
      {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        conf[ix] = result.classification[ix].value;
        ei_printf("\n");
        if (result.classification[ix].value > best_val)
        {
          best_val = result.classification[ix].value;
          best = ix;
        }
      }

      if (result.classification[best].value >= TRESHOLD)
      {
        // set_status_led(best);
      }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("    anomaly score: ");
      ei_printf_float(result.anomaly);
      ei_printf("\n");
#endif
      // 3. Find top result
      // String bestLabel = "Unknown";
      // float bestConf = 0.0;

      // // (Assuming you have classification results structure)
      // // for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      // //    if (result.classification[ix].value > bestConf) {
      // //        bestConf = result.classification[ix].value;
      // //        bestLabel = result.classification[ix].label;
      // //    }
      // // }

      // // Mock result if library not linked:
      // bestLabel = "Noise";
      // bestConf = 0.85;

      // // 4. Update Shared Data safely
      // if (xSemaphoreTake(aiMutex, 100))
      // {
      //   sharedAiLabel = bestLabel;
      //   sharedAiConf = bestConf;
      //   xSemaphoreGive(aiMutex);
      // }

    vTaskDelay(100);
  }
}

// ==================================================
// 5. TASK RADIO (CORE 1)
// ==================================================
void taskRadio(void *pvParameters)
{
  Serial.println("[Radio] Task Started on Core 1");

  // GPS Init
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // LoRa Init
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);
  int state = radio.begin(868.0, 125.0, 9, 8, 0x12, 22);
  if (state != RADIOLIB_ERR_NONE)
    Serial.printf("Radio Fail: %d\n", state);

  while (true)
  {
    // 1. GPS Feed
    while (GPS_Serial.available())
      gps.encode(GPS_Serial.read());

    // 2. Check Radio
    String str;
    state = radio.receive(str);

    if (state == RADIOLIB_ERR_NONE)
    {
      int separator = str.indexOf(':');
      int targetID = str.substring(0, separator).toInt();

      if (targetID == NODE_ID)
      {
        Serial.println("Ping Received! Gathering Data...");

        // Get GPS
        String latStr = "0.0", lonStr = "0.0", fix = "0";
        if (gps.location.isValid())
        {
          latStr = String(gps.location.lat(), 6);
          lonStr = String(gps.location.lng(), 6);
          fix = "1";
        }

        // Get AI Data
        String aiResult = labelNoise + ": " + String(conf[0], 2) + "; " + labelSteps + ": " + String(conf[1], 2) + ";";
        // if (xSemaphoreTake(aiMutex, 50))
        // {
        //   aiResult = sharedAiLabel + "(" + String(sharedAiConf, 2) + ")";
        //   xSemaphoreGive(aiMutex);
        // }

        // Reply: "1:Lat,Lon,Fix,AIResult"
        String reply = "1:" + latStr + "," + lonStr + "," + fix + "," + aiResult;
        digitalWrite(LED_PIN, HIGH); // LED On
        radio.transmit(reply);
        digitalWrite(LED_PIN, LOW); // LED On
        Serial.println("Sent: " + reply);
      }
    }
    vTaskDelay(10);
  }
}

// ==================================================
// SETUP
// ==================================================
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  aiMutex = xSemaphoreCreateMutex();

  // Core 0 for AI (High Calc), Core 1 for Radio (High Timing)
  xTaskCreatePinnedToCore(taskAI, "AI_Task", 16384, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskRadio, "Radio_Task", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelay(1000); // Main loop does nothing
}