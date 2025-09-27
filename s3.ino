#include "esp_camera.h"
#include <lvgl.h>
#include <demos/lv_demos.h>
#include <Arduino_GFX_Library.h>
#include "bsp_cst816.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define PWDN_GPIO_NUM 17
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 21
#define SIOC_GPIO_NUM 16
#define Y9_GPIO_NUM 2
#define Y8_GPIO_NUM 7
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 11
#define Y4_GPIO_NUM 15
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 12
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 4
#define PCLK_GPIO_NUM 9

#define EXAMPLE_PIN_NUM_LCD_SCLK 39
#define EXAMPLE_PIN_NUM_LCD_MOSI 38
#define EXAMPLE_PIN_NUM_LCD_MISO 40
#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45
#define EXAMPLE_PIN_NUM_LCD_BL 1
#define EXAMPLE_PIN_NUM_TP_SDA 48
#define EXAMPLE_PIN_NUM_TP_SCL 47

#define LEDC_FREQ 5000
#define LEDC_TIMER_10_BIT 10

#define EXAMPLE_LCD_ROTATION 0
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  EXAMPLE_PIN_NUM_LCD_DC, EXAMPLE_PIN_NUM_LCD_CS,
  EXAMPLE_PIN_NUM_LCD_SCLK, EXAMPLE_PIN_NUM_LCD_MOSI, EXAMPLE_PIN_NUM_LCD_MISO);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, EXAMPLE_PIN_NUM_LCD_RST, EXAMPLE_LCD_ROTATION, true,
  EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf;
lv_disp_drv_t disp_drv;

static SemaphoreHandle_t lvgl_api_mux = NULL;

lv_obj_t *img_camera;
lv_obj_t *splash_label;
lv_obj_t *label_pill_count;
lv_img_dsc_t snapshot_img;
bool snapshot_taken = false;
bool screen_touched = false;

enum AppState {
  SPLASH,
  PREVIEW,
  SNAPSHOT
};

AppState app_state = SPLASH;
unsigned long splash_start_time = 0;

bool lvgl_lock(int timeout_ms) {
  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void) {
  xSemaphoreGiveRecursive(lvgl_api_mux);
}

#if LV_USE_LOG != 0
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
  lv_disp_flush_ready(disp_drv);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  uint16_t touchpad_x;
  uint16_t touchpad_y;
  bsp_touch_read();
  if (bsp_touch_get_coordinates(&touchpad_x, &touchpad_y)) {
    data->point.x = touchpad_x;
    data->point.y = touchpad_y;
    data->state = LV_INDEV_STATE_PRESSED;
    screen_touched = true;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void lvgl_camera_ui_init(lv_obj_t *parent) {
  img_camera = lv_img_create(parent);
  lv_obj_align(img_camera, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_pos(img_camera, -1, 0);
  lv_obj_set_scroll_dir(parent, LV_DIR_NONE);
  lv_obj_set_style_pad_top(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(img_camera, 0, LV_PART_MAIN);

  label_pill_count = lv_label_create(parent);
  lv_obj_set_style_text_color(label_pill_count, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_font(label_pill_count, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(label_pill_count, LV_ALIGN_TOP_MID, 0, 10);
  lv_label_set_text(label_pill_count, "");
}

void post_image_and_show_count() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return;
  }

  HTTPClient http;
  http.begin("http://192.168.4.117:5000/upload");
  http.addHeader("Content-Type", "application/octet-stream");

  int httpResponseCode = http.POST((uint8_t *)snapshot_img.data, snapshot_img.data_size);


  if (httpResponseCode == HTTP_CODE_OK) {
    String payload = http.getString();

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      if (doc.containsKey("pill_count")) {
        int pill_count = doc["pill_count"];
        if (lvgl_lock(-1)) {
          char buf[32];
          snprintf(buf, sizeof(buf), "Pills: %d", pill_count);
          lv_label_set_text(label_pill_count, buf);
          lvgl_unlock();
        }
      }
    } else {
      Serial.println("JSON parse error");
    }
  } else {
    Serial.printf("HTTP POST failed, error: %d\n", httpResponseCode);
  }
  http.end();
}

static void task(void *param) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_1;
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
  config.frame_size = FRAMESIZE_HVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    vTaskDelete(NULL);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);

  camera_fb_t *pic;
  lv_img_dsc_t img_dsc;
  img_dsc.header.always_zero = 0;
  img_dsc.header.w = 480;
  img_dsc.header.h = 320;
  img_dsc.data_size = 320 * 480 * 2;
  img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
  img_dsc.data = NULL;

  unsigned long upload_time = 0;

  while (1) {
    if (app_state == SPLASH) {
      if (millis() - splash_start_time >= 2000) {
        if (lvgl_lock(-1)) {
          lv_obj_del(splash_label);
          lvgl_camera_ui_init(lv_scr_act());
          lvgl_unlock();
        }
        app_state = PREVIEW;
      }
    } else if (app_state == PREVIEW) {
      pic = esp_camera_fb_get();
      if (pic && !snapshot_taken) {
        img_dsc.data = pic->buf;
        if (lvgl_lock(-1)) {
          lv_img_set_src(img_camera, &img_dsc);
          lvgl_unlock();
        }
        if (screen_touched) {
          snapshot_taken = true;
          screen_touched = false;
          snapshot_img = img_dsc;
          snapshot_img.data = (uint8_t *)heap_caps_malloc(img_dsc.data_size, MALLOC_CAP_SPIRAM);
          if (snapshot_img.data) {
            memcpy((void *)snapshot_img.data, (const void *)pic->buf, img_dsc.data_size);
          }
          app_state = SNAPSHOT;
          upload_time = 0;
        }
        esp_camera_fb_return(pic);
      }
    } else if (app_state == SNAPSHOT) {
      if (lvgl_lock(-1)) {
        lv_img_set_src(img_camera, &snapshot_img);
        lvgl_unlock();
      }
      if (upload_time == 0) {
        upload_time = millis();
      } else if (millis() - upload_time > 2000) {
        post_image_and_show_count();
        upload_time = ULONG_MAX;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
  Serial.println("Arduino_GFX LVGL_Arduino_v8 example ");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

  WiFi.begin("123", "30attenb");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef EXAMPLE_PIN_NUM_LCD_BL
  ledcAttach(EXAMPLE_PIN_NUM_LCD_BL, LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcWrite(EXAMPLE_PIN_NUM_LCD_BL, (1 << LEDC_TIMER_10_BIT) / 100 * 80);
#endif

  Wire.begin(EXAMPLE_PIN_NUM_TP_SDA, EXAMPLE_PIN_NUM_TP_SCL);
  bsp_touch_init(&Wire, gfx->getRotation(), gfx->width(), gfx->height());
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();
  bufSize = screenWidth * screenHeight;

  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }

  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.direct_mode = true;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    splash_label = lv_label_create(lv_scr_act());
    lv_label_set_text(splash_label, "deepscript.ai");
    lv_obj_set_style_text_font(splash_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(splash_label, LV_ALIGN_CENTER, 0, 0);
    splash_start_time = millis();
  }

  Serial.println("Setup done");
  xTaskCreatePinnedToCore(
    task,
    "lvgl_app_task",
    1024 * 10,
    NULL,
    1,
    NULL,
    0);
}

void loop() {
  if (lvgl_lock(-1)) {
    lv_timer_handler();
    lvgl_unlock();
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}
