// STEP 6c: gleich wie 6b aber OHNE WRITE_PERI_REG (brownout-disable)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

static SPIClass imuSPI(HSPI);
ICM20948_WE myIMU(&imuSPI, 43, 2, 3, 4, true);
SparkFun_VL53L5CX myImager;
I2SClass I2S;

static camera_config_t cfg = {
  .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM, .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM, .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM, .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM, .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
  .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA,
  .jpeg_quality = 30, .fb_count = 2,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_LATEST
};

void setup() {
  // KEIN WRITE_PERI_REG hier!
  Serial.begin(115200);
  delay(1500);
  Serial.println("STEP6c: kein brownout-disable in setup");
  Serial.flush();

  Wire.begin();

  if (SD.begin(21, SPI, 40000000))
    Serial.printf("  SD: OK (%llu MB)\n", SD.cardSize() / (1024ULL * 1024ULL));
  else
    Serial.println("  SD: FAIL");

  Serial.println("STEP6c done");
}

void loop() {
  Serial.println("alive");
  delay(2000);
}
