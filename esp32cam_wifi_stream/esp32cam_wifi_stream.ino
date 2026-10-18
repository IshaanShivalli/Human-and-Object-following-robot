#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"
#include "board_config.h"
#include "wifi_secrets.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#define MDNS_NAME "esp32cam"

#define ARD_BAUD    57600
#define ArduinoSerial Serial

#define DEBUG_MODE 0
#if DEBUG_MODE
  #define SERIAL_BAUD     115200
  #define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define SERIAL_BAUD     ARD_BAUD
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

static esp_err_t motor_handler(httpd_req_t* req) {
  char query[32], cmd[16];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "cmd", cmd, sizeof(cmd)) == ESP_OK && cmd[0]) {
      ArduinoSerial.print(cmd);
      ArduinoSerial.print('\n');
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "ok");
}

static esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char hdr[128];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    size_t hl = snprintf(hdr, sizeof(hdr),
                         "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
    esp_err_t res = httpd_resp_send_chunk(req, hdr, hl);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return ESP_OK;
}

void startServers() {
  httpd_config_t ctl = HTTPD_DEFAULT_CONFIG();
  ctl.server_port      = 80;
  ctl.max_uri_handlers = 4;
  httpd_handle_t ctlServer = NULL;
  if (httpd_start(&ctlServer, &ctl) == ESP_OK) {
    httpd_uri_t motorUri = { "/motor", HTTP_GET, motor_handler, NULL };
    httpd_register_uri_handler(ctlServer, &motorUri);
  }

  httpd_config_t st = HTTPD_DEFAULT_CONFIG();
  st.server_port        = 81;
  st.ctrl_port          = 32769;
  st.recv_wait_timeout  = 10;
  st.send_wait_timeout  = 10;
  st.stack_size         = 8192;
  httpd_uri_t streamUri = { "/stream", HTTP_GET, stream_handler, NULL };
  httpd_handle_t streamServer = NULL;
  if (httpd_start(&streamServer, &st) == ESP_OK) httpd_register_uri_handler(streamServer, &streamUri);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  ArduinoSerial.begin(SERIAL_BAUD);
  DBG_PRINTLN("Arduino UART bridge ready on GPIO1(TX)/GPIO3(RX)");
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz    = 16000000;
  c.pixel_format    = PIXFORMAT_JPEG;
  c.frame_size      = FRAMESIZE_QVGA;
  c.jpeg_quality    = 18;
  c.fb_count        = 3;
  c.fb_location     = CAMERA_FB_IN_PSRAM;
  c.grab_mode       = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    DBG_PRINTLN("Camera init failed - check power and ribbon cable");
    delay(3000); ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  if (FLIP_180) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
  s->set_aec2(s, 1);
  s->set_gainceiling(s, GAINCEILING_8X);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_17dBm);
  DBG_PRINT("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(400); DBG_PRINT("."); }
  DBG_PRINTLN();

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("http", "tcp", 81);
    DBG_PRINTLN("mDNS started: http://" MDNS_NAME ".local:81/stream");
  } else {
    DBG_PRINTLN("mDNS failed – use IP: http://" + WiFi.localIP().toString() + ":81/stream");
  }

  startServers();
  DBG_PRINT("IP fallback : http://");
  DBG_PRINT(WiFi.localIP());
  DBG_PRINTLN(":81/stream   (battery mode: flash OFF)");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_PRINTLN("WiFi lost - reconnecting...");
    MDNS.end();
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int i = 0; i < 25 && WiFi.status() != WL_CONNECTED; i++) delay(400);
    if (WiFi.status() == WL_CONNECTED) {
      DBG_PRINT("Reconnected: "); DBG_PRINTLN(WiFi.localIP());

      if (MDNS.begin(MDNS_NAME)) {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("http", "tcp", 81);
      }
    }
  }
  delay(500);
}