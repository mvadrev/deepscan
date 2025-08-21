#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ==== WiFi config ====
const char* ssid = "123";
const char* password = "30attenb";
const char* serverName = "http://192.168.4.117:5000/upload"; // Flask endpoint

// ==== OLED config ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA_PIN  13
#define OLED_SCL_PIN  15
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==== Camera pins ====
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

// ==== Button ====
#define BUTTON_PIN 14
bool buttonPressedLast = false;

void setup() {
  Serial.begin(115200);

  // OLED init
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("SSD1306 allocation failed"); 
    while(1); 
  }

  // Boot splash
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,20);
  display.println("Deepscript.ai");
  display.display();
  delay(2000);

  // WiFi connect
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED){ 
    delay(500); Serial.print("."); 
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

  // Camera config (JPEG)
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
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
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format   = PIXFORMAT_JPEG;   // JPEG for Flask
  config.frame_size     = FRAMESIZE_QVGA;   // 320x240
  config.jpeg_quality   = 12;               // 0-63 (lower = better)
  config.fb_count       = 1;                // minimal memory

  if(esp_camera_init(&config) != ESP_OK){
    Serial.println("Camera init failed!");
    display.clearDisplay();
    display.setCursor(0,20);
    display.println("Camera Error!");
    display.display();
    while(1){ delay(1000); }
  }

  // Button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // READY screen
  display.clearDisplay();
  display.setCursor(0,20);
  display.println("READY");
  display.display();
}

void loop() {
  // Detect single button press
  bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if(pressed && !buttonPressedLast){
    Serial.println("Button pressed: capturing image...");
    delay(50); // small pause before capture

    // Capture JPEG frame
    camera_fb_t* fb = esp_camera_fb_get();
    if(!fb){
      Serial.println("Failed to capture image!");
    } else {
      Serial.printf("Captured %d bytes\n", fb->len);

      // POST to Flask
      if(WiFi.status() == WL_CONNECTED){
        HTTPClient http;
        http.begin(serverName);
        http.addHeader("Content-Type","image/jpeg");
        int code = http.POST(fb->buf, fb->len);
        Serial.printf("POST code: %d\n", code);
        http.end();
      }

      esp_camera_fb_return(fb);
    }
  }
  buttonPressedLast = pressed;

  delay(50); // debounce
}
