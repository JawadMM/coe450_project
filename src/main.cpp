#include <Arduino.h>
#include "esp_camera.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <TJpg_Decoder.h>

// Camera pins (adjust based on your ESP32 board)
#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

// Callback function declaration
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

// Input buffer for the model
static float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Buffer to hold the resized image
uint16_t resized_image[EI_CLASSIFIER_INPUT_WIDTH  * EI_CLASSIFIER_INPUT_HEIGHT];

// Callback function for JPEG decoder to read into a temporary buffer
bool jpeg_to_resized_buffer(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  static uint16_t decoded_image[600 * 480]; // Buffer for the original 600x480 image

  // Copy decoded blocks into the temporary image buffer
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      if ((x + i) < 600 && (y + j) < 480) {
        decoded_image[(y + j) * 600 + (x + i)] = bitmap[j * w + i];
      }
    }
  }

  return true;
}

// Function to resize the decoded image to 96x96
void resize_image(uint16_t *src, uint16_t *dst, int src_width, int src_height, int dst_width, int dst_height) {
  float x_ratio = (float)src_width / dst_width;
  float y_ratio = (float)src_height / dst_height;

  for (int y = 0; y < dst_height; y++) {
    for (int x = 0; x < dst_width; x++) {
      int src_x = (int)(x * x_ratio);
      int src_y = (int)(y * y_ratio);
      dst[y * dst_width + x] = src[src_y * src_width + src_x];
    }
  }
}



void setup() {
  Serial.begin(115200);


  // Initialize JPEG decoder
  TJpgDec.setJpgScale(1);          // No scaling
  TJpgDec.setSwapBytes(true);      // Swap bytes for compatibility
  TJpgDec.setCallback(jpeg_to_resized_buffer);

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  // config.frame_size = FRAMESIZE_96X96;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }
  Serial.println("Camera initialized");
}

void loop() {
   delay(1000);
  // Capture a frame from the camera
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Preprocess the frame data into the input buffer
  for (int y = 0; y < EI_CLASSIFIER_INPUT_HEIGHT; y++) {
    for (int x = 0; x < EI_CLASSIFIER_INPUT_WIDTH; x++) {
      // Extract pixel from RGB565 format
      uint16_t pixel = ((uint16_t *)fb->buf)[y * EI_CLASSIFIER_INPUT_WIDTH + x];
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;

      // Convert to grayscale and normalize
      float grayscale = (r * 0.299 + g * 0.587 + b * 0.114) / 31.0;
      input_buf[y * EI_CLASSIFIER_INPUT_WIDTH + x] = grayscale;
    }
  }

  // Release the frame buffer
  esp_camera_fb_return(fb);

  // Set up the signal object
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_signal_data;

  // Run the classifier
  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  if (res != EI_IMPULSE_OK) {
    Serial.printf("Classifier error: %d\n", res);
    return;
  }

  // Print the classification results
  Serial.println("Predictions:");
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("  %s: %.5f\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
  }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  Serial.printf("Anomaly prediction: %.3f\n", result.anomaly);
#endif
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

 
}

// Callback: Fill buffer with model input
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, input_buf + offset, length * sizeof(float));
  return EIDSP_OK;
}
