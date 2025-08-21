#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ==== WiFi config ====
const char* ssid = "123";
const char* password = "30attenb";

// ==== Flask API endpoint ====
const char* serverName = "http://192.168.4.117:5000/upload"; // replace with your PC IP

// ==== OLED config ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA_PIN  13  // safe pins, not used by camera
#define OLED_SCL_PIN  15
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==== Freenove WROVER CAM pins ====
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

void setup() {
  Serial.begin(115200);

  // Init OLED on safe pins
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }
  display.clearDisplay();
  display.display();

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // Camera config
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565; // RGB565 for easy OLED conversion
  config.frame_size = FRAMESIZE_QQVGA;    // 160x120
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    return;
  }

  delay(2000);

  // Capture frame
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Camera capture failed");
    return;
  }

  // Display on OLED (128x64)
  display.clearDisplay();
  for(int y=0; y<SCREEN_HEIGHT; y++){
    for(int x=0; x<SCREEN_WIDTH; x++){
      int srcX = x * fb->width / SCREEN_WIDTH;
      int srcY = y * fb->height / SCREEN_HEIGHT;
      uint16_t color = ((uint16_t*)fb->buf)[srcY*fb->width + srcX];
      uint8_t gray = ((color>>11 & 0x1F)*8 + (color>>5 & 0x3F)*4 + (color & 0x1F)*8)/3;
      if(gray>128) display.drawPixel(x, y, SSD1306_WHITE);
      else display.drawPixel(x, y, SSD1306_BLACK);
    }
  }
  display.display();

  // Optional: send to Flask server
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "image/jpeg");
    int httpResponseCode = http.POST(fb->buf, fb->len);
    Serial.printf("POST code: %d\n", httpResponseCode);
    http.end();
  }

  // Free frame buffer
  esp_camera_fb_return(fb);
}

void loop() {
  // Capture frame
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Camera capture failed");
    delay(500);
    return;
  }

  // Display on OLED
  display.clearDisplay();
  for(int y=0; y<SCREEN_HEIGHT; y++){
    for(int x=0; x<SCREEN_WIDTH; x++){
      int srcX = x * fb->width / SCREEN_WIDTH;
      int srcY = y * fb->height / SCREEN_HEIGHT;
      uint16_t color = ((uint16_t*)fb->buf)[srcY*fb->width + srcX];
      uint8_t gray = ((color>>11 & 0x1F)*8 + (color>>5 & 0x3F)*4 + (color & 0x1F)*8)/3;
      if(gray>128) display.drawPixel(x, y, SSD1306_WHITE);
      else display.drawPixel(x, y, SSD1306_BLACK);
    }
  }
  display.display();

  // Free frame buffer
  esp_camera_fb_return(fb);

  delay(200); // small delay for refresh (~5 FPS)
}

