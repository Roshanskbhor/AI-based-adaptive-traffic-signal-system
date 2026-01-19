// ESP32-CAM code - send JPEG snapshot to FastAPI, stream on :81
// Wiring required:
// Controller GPIO32 -> CAM GPIO12 (BIT0)
// Controller GPIO33 -> CAM GPIO13 (BIT1)
// Controller GPIO4  -> CAM GPIO14 (TRIGGER)

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_http_server.h"

// -------- WiFi --------
const char* ssid = "RRR";
const char* password = "20050522";

// -------- FastAPI --------
const char* FASTAPI_HOST = "192.168.137.181";
const int FASTAPI_PORT = 8000;

// -------- GPIO pins (CAM-side) --------
#define TRIGGER_PIN   14  // from controller (GPIO4)
#define BIT0_PIN      12  // controller GPIO32 -> CAM GPIO12 (BIT0)
#define BIT1_PIN      13  // controller GPIO33 -> CAM GPIO13 (BIT1)

// -------- Camera pins (AI-Thinker layout) --------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y2_GPIO_NUM        5
#define Y3_GPIO_NUM       18
#define Y4_GPIO_NUM       19
#define Y5_GPIO_NUM       21
#define Y6_GPIO_NUM       36
#define Y7_GPIO_NUM       39
#define Y8_GPIO_NUM       34
#define Y9_GPIO_NUM       35
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();

void setupCamera()
{
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
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // HIGH RESOLUTION
  config.frame_size = FRAMESIZE_SVGA; // 800x600
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println(" Camera init failed!");
    while (true) delay(1000);
  }
}

// decode bits: b1 b0 -> lane (00=1, 01=2, 10=3)
int detectLaneFromBits()
{
  int b0 = digitalRead(BIT0_PIN); // BIT0
  int b1 = digitalRead(BIT1_PIN); // BIT1

  if (b1 == 0 && b0 == 0) return 1;
  if (b1 == 0 && b0 == 1) return 2;
  if (b1 == 1 && b0 == 0) return 3;
  return 1;
}

void sendSnapshot(int lane)
{
  delay(80); // avoid FB conflict with stream and allow bits to settle

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(" Snapshot FAILED (fb NULL)");
    return;
  }

  String url = "http://" + String(FASTAPI_HOST) + ":" +
               String(FASTAPI_PORT) + "/upload/" + String(lane);

  Serial.println("➡ Uploading to " + url);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "image/jpeg");

  int code = http.POST(fb->buf, fb->len);
  Serial.printf("HTTP %d\n", code);
  if (code > 0) {
    String resp = http.getString();
    Serial.println("Response:");
    Serial.println(resp);
  } else {
    Serial.printf("HTTP error: %d\n", code);
  }

  http.end();
  esp_camera_fb_return(fb);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\nBooting ESP32-CAM...");

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nWiFi OK");
  Serial.println(WiFi.localIP());

  // TRIGGER pin is input (controller pulses this)
  pinMode(TRIGGER_PIN, INPUT);

  // Lane bits: stable LOW by default – use pull-down (so 00 -> lane1)
  pinMode(BIT0_PIN, INPUT_PULLDOWN);
  pinMode(BIT1_PIN, INPUT_PULLDOWN);

  setupCamera();
  startCameraServer();
  Serial.println(" Stream ready on port 81!");
}

void loop()
{
  // Trigger logic: respond once per HIGH pulse
  if (digitalRead(TRIGGER_PIN) == HIGH) {
    delay(40); // debounce
    if (digitalRead(TRIGGER_PIN) == HIGH) {
      // read lane bits immediately after settle
      int lane = detectLaneFromBits();
      Serial.printf("📸 Trigger → Lane %d\n", lane);

      sendSnapshot(lane);

      // wait until trigger goes low to avoid double-capture
      while (digitalRead(TRIGGER_PIN) == HIGH) delay(10);
      delay(150); // short debounce
    }
  }
}

// streaming
typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;

    httpd_resp_sendstr_chunk(req, "--frame\r\n");
    httpd_resp_sendstr_chunk(req, "Content-Type: image/jpeg\r\n\r\n");
    httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    httpd_resp_sendstr_chunk(req, "\r\n");
    esp_camera_fb_return(fb);
  }
  return ESP_OK;
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_handle_t httpd = NULL;
  httpd_uri_t uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(httpd, &uri);
    Serial.println("Stream server started.");
  } else {
    Serial.println("Failed to start stream server");
  }
}