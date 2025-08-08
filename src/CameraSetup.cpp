// CameraSetup.cpp
#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "CameraSetup.h"
#include "base64.h"

bool cameraReady = false;  // definition of the extern

void setupCamera() {
   camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
      config.frame_size = FRAMESIZE_QVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    } else {
      config.frame_size = FRAMESIZE_QQVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
  }

  cameraReady = (esp_camera_init(&config) == ESP_OK);
  if (cameraReady) {
    Serial.println("✅ Camera initialized successfully.");
  } else {
    Serial.println("❌ Camera initialization failed.");
  }
}

// return a fresh fb pointer (or nullptr on failure)
camera_fb_t* captureImagePreview() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Capture failed");
    return nullptr;
  }
  // (optional) log base64
  String b64 = base64::encode(fb->buf, fb->len);
  Serial.println(b64);
  return fb;
}
