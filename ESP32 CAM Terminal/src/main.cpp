#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include "esp_camera.h"
#include "Config.h"
#include "ArduinoJson-v6.11.1.h"  // 你上传的版本 (6.11.1)

// Async server
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>

// ---------- WiFi ----------
AsyncWebServer server(80);
static const char* AP_SSID = "ESP32-CAM";
static const char* AP_PASS = "12345678";

// ---------- UART to UNO ----------
static const uint32_t UART_BAUDRATE = 9600;   // 和 UNO 一致
// UART_RX_PIN / UART_TX_PIN 来自 Config.h

// ---------- Camera ----------
static const framesize_t STREAM_FRAME = FRAMESIZE_VGA;  // 640x480
static const int JPEG_QUALITY = 8;                      // 1(最好)~63(最差)
static const int FB_COUNT = 2;                          // 双 buffer

// ---------- MJPEG ----------
static const char* BOUNDARY = "mjpeg-boundary-0123456789";

// 维护当前连接的 MJPEG 客户端
struct MjpegClient {
  AsyncClient* c;
  bool headerSent;
  MjpegClient(AsyncClient* client=nullptr): c(client), headerSent(false) {}
};

static std::vector<MjpegClient> g_clients;
static SemaphoreHandle_t g_clients_mutex;

// 最新一帧 JPEG 缓存（由采集任务更新，MJPEG 广播复用）
static uint8_t* g_last_jpg = nullptr;
static size_t   g_last_jpg_len = 0;
static SemaphoreHandle_t g_jpg_mutex;

// 采集帧率控制
static const int CAPTURE_INTERVAL_MS = 50; // ~20 fps 目标
static TaskHandle_t g_cam_task = nullptr;

// ---------- Last command state (/status) ----------
static String   g_last_motion = "Stop";
static int      g_last_speed  = 0;
static uint32_t g_last_cmd_ms = 0;

// 统一构造并发送 { "M": "...", "v": <int> } 给 UNO
static inline void send_to_uno(const char* motion, int speed) {
  StaticJsonDocument<100> doc;
  doc["M"] = motion;        // Forward/Backward/Left/Right/Stop
  doc["v"] = speed;         // 0..255
  serializeJson(doc, Serial1);
  Serial1.print('\n');      // 友好分隔（UNO 接收按 {...} 抓取，\n不影响）
}

// // ------- json_get_string 放在文件前面（工具函数） -------
// static bool json_get_string(const String& s, const char* key, String& out) {
//   // 找到 "key"
//   String kq = String("\"") + key + "\"";
//   int k = s.indexOf(kq);
//   if (k < 0) return false;
//   // 冒号
//   int colon = s.indexOf(':', k + kq.length());
//   if (colon < 0) return false;
//   // 第一个引号（值的起始引号）
//   int q1 = s.indexOf('\"', colon + 1);
//   if (q1 < 0) return false;
//   // 第二个引号（值的结束引号）
//   int q2 = s.indexOf('\"', q1 + 1);
//   if (q2 < 0) return false;
//   out = s.substring(q1 + 1, q2);
//   out.trim();
//   return true;
// }

// static bool json_get_int(const String& s, const char* key, int& out) {
//   String kq = String("\"") + key + "\"";
//   int k = s.indexOf(kq);
//   if (k < 0) return false;
//   int colon = s.indexOf(':', k + kq.length());
//   if (colon < 0) return false;
//   // 取到逗号或大括号结束
//   int end = s.indexOf(',', colon + 1);
//   if (end < 0) end = s.indexOf('}', colon + 1);
//   if (end < 0) end = s.length();
//   String num = s.substring(colon + 1, end);
//   num.trim();
//   if (num.length() == 0) return false;
//   out = num.toInt();  // 非数字会返回 0
//   return true;
// }

// ---------- Utils ----------
static const char* map_motion(const String& m) {
  if (m == "Forward")  return "Forward";
  if (m == "Backward") return "Backward";
  if (m == "Left")     return "Left";
  if (m == "Right")    return "Right";
  if (m == "stop_it" || m == "Stop") return "Stop";
  return "Unknown";
}

// 安全复制最新 JPEG（用于 /jpg 快照或调试）
static bool copy_last_jpg(std::vector<uint8_t>& out) {
  bool ok = false;
  if (xSemaphoreTake(g_jpg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (g_last_jpg && g_last_jpg_len > 0) {
      out.resize(g_last_jpg_len);
      memcpy(out.data(), g_last_jpg, g_last_jpg_len);
      ok = true;
    }
    xSemaphoreGive(g_jpg_mutex);
  }
  return ok;
}

// 发送一帧给单个 MJPEG 客户端（无阻塞尝试）
static void mjpeg_send_frame_to(AsyncClient* client, const uint8_t* jpg, size_t len) {
  if (!client || !client->connected()) return;
  if (!jpg || len == 0) return;

  // 边界 + 头
  char hdr[128];
  int hdrlen = snprintf(hdr, sizeof(hdr),
                        "--%s\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: %u\r\n\r\n",
                        BOUNDARY, (unsigned)len);

  if (client->space() >= (size_t)hdrlen + len + 2) {
    client->add(hdr, hdrlen);
    client->add((const char*)jpg, len);
    client->add("\r\n", 2);
    client->send();
  }
}

// 广播一帧给所有 MJPEG 客户端
static void mjpeg_broadcast(const uint8_t* jpg, size_t len) {
  if (xSemaphoreTake(g_clients_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

  // 清理断开客户端
  for (size_t i = 0; i < g_clients.size(); ) {
    if (!g_clients[i].c || !g_clients[i].c->connected()) {
      if (g_clients[i].c) {
        g_clients[i].c->close(true);
        delete g_clients[i].c;
      }
      g_clients.erase(g_clients.begin() + i);
    } else {
      ++i;
    }
  }

  for (auto& mc : g_clients) {
    if (!mc.c || !mc.c->connected()) continue;

    // 首次：发送 HTTP 头
    if (!mc.headerSent) {
      char head[256];
      int n = snprintf(head, sizeof(head),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--%s\r\n"
        "Connection: close\r\n"
        "\r\n", BOUNDARY);
      mc.c->add(head, n);
      mc.c->send();
      mc.headerSent = true;
    }

    // 发一帧
    mjpeg_send_frame_to(mc.c, jpg, len);
  }

  xSemaphoreGive(g_clients_mutex);
}

// 相机采集任务：抓帧 -> 更新 g_last_jpg -> 广播给所有 /mjpeg 客户端
static void cameraTask(void* arg) {
  uint32_t last = 0;
  for (;;) {
    uint32_t now = millis();
    if (now - last < CAPTURE_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    last = now;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // 默认 JPEG 直出
    if (fb->format == PIXFORMAT_JPEG) {
      // 更新缓存
      if (xSemaphoreTake(g_jpg_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_last_jpg) { free(g_last_jpg); g_last_jpg = nullptr; g_last_jpg_len = 0; }
        g_last_jpg = (uint8_t*)malloc(fb->len);
        if (g_last_jpg) {
          memcpy(g_last_jpg, fb->buf, fb->len);
          g_last_jpg_len = fb->len;
        }
        xSemaphoreGive(g_jpg_mutex);
      }

      // 广播
      mjpeg_broadcast(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
  }
}

// ---------- HTTP: root ----------
static void handle_root(AsyncWebServerRequest* request) {
  String html;
  html  = "<html><body><h3>ESP32 Async MJPEG (PC-side AprilTag)</h3>";
  html += "<p><a href='/mjpeg'>/mjpeg</a> (async video stream)</p>";
  html += "<p><a href='/jpg'>/jpg</a> (single snapshot)</p>";
  html += "<p><a href='/status'>/status</a> (last command JSON)</p>";
  html += "<p>POST control to <code>/cmd</code>, e.g. <code>{\"M\":\"Left\",\"v\":90}</code></p>";
  html += "</body></html>";
  request->send(200, "text/html", html);
}

// ---------- HTTP: /status ----------
static void handle_status(AsyncWebServerRequest* request) {
  String j = "{";
  j += "\"motion\":\"" + g_last_motion + "\",";
  j += "\"speed\":" + String(g_last_speed) + ",";
  j += "\"ts_ms\":" + String(g_last_cmd_ms);
  j += "}";
  request->send(200, "application/json", j);
}

// ---------- HTTP: /jpg ----------
static void handle_jpg(AsyncWebServerRequest* request) {
  std::vector<uint8_t> jpg;
  if (copy_last_jpg(jpg)) {
    AsyncResponseStream* res = request->beginResponseStream("image/jpeg");
    res->write(jpg.data(), jpg.size());
    request->send(res);
    return;
  }
  // 否则现抓一帧（同上）
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { request->send(503, "text/plain", "camera busy"); return; }
  AsyncResponseStream* res = request->beginResponseStream("image/jpeg");
  res->write((const uint8_t*)fb->buf, fb->len);
  request->send(res);
  esp_camera_fb_return(fb);
}

// ---------- HTTP: /cmd (POST body, ArduinoJson版，线程安全 per-request 缓存) ----------
static void on_cmd_body(AsyncWebServerRequest* request,
                        uint8_t* data, size_t len, size_t index, size_t total) {
  // 为每个请求挂一个 String* 作缓存，避免不同请求之间串包
  String* body = reinterpret_cast<String*>(request->_tempObject);
  if (index == 0) {
    if (!body) {
      body = new String();
      request->_tempObject = body;
    }
    body->reserve(total + 4);
    body->clear();
  }
  body->concat(reinterpret_cast<const char*>(data), len);

  // 整个 POST 收齐
  if (index + len == total) {
    // 容错：有些客户端可能漏掉右大括号
    if (!body->endsWith("}")) body->concat("}");

    // 1) 解析 JSON（接收 "M"/"m" 与 "v"/"V"）
    StaticJsonDocument<100> docIn;
    DeserializationError err = deserializeJson(docIn, *body);
    if (err) {
      Serial.printf("[cmd] JSON parse error: %s  raw=%s\n",
                    err.c_str(), body->c_str());
      request->send(400, "application/json", "{\"ok\":false,\"err\":\"bad json\"}");
      return;
    }

    // 提取并归一化
    String mVal;
    if (docIn.containsKey("M"))      mVal = (const char*)docIn["M"];
    else if (docIn.containsKey("m")) mVal = (const char*)docIn["m"];
    else                             mVal = "Unknown";

    int vVal = -1;
    if (docIn.containsKey("v"))      vVal = docIn["v"].as<int>();
    else if (docIn.containsKey("V")) vVal = docIn["V"].as<int>();
    vVal = constrain(vVal < 0 ? 0 : vVal, 0, 255);

    // 统一动作名（保持和 UNO 端映射一致）
    const char* motionNorm = map_motion(mVal);  // 你的工具函数：Unknown/Stop/Forward/...

    // 2) 用 ArduinoJson 重新“干净”序列化并发给 UNO
    send_to_uno(motionNorm, vVal);

    // 3) 更新 /status 与日志
    g_last_motion = motionNorm;
    g_last_speed  = vVal;
    g_last_cmd_ms = millis();
    Serial.printf("[cmd] motion=%s  v=%d\n", g_last_motion.c_str(), g_last_speed);

    // 4) 回复客户端
    request->send(200, "application/json", "{\"ok\":true}");

    // 释放本请求缓存
    delete body;
    request->_tempObject = nullptr;
  }
}

// ---------- Camera init ----------
static bool init_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = CAM_PIN_D0;  config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2;  config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4;  config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6;  config.pin_d7 = CAM_PIN_D7;

  config.pin_xclk   = CAM_PIN_XCLK;
  config.pin_pclk   = CAM_PIN_PCLK;
  config.pin_vsync  = CAM_PIN_VSYNC;
  config.pin_href   = CAM_PIN_HREF;

  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;

  config.pin_pwdn  = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;

  config.xclk_freq_hz = CAM_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;  // 关键：JPEG直出
  config.frame_size   = STREAM_FRAME;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count     = FB_COUNT;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[camera] init failed 0x%x\n", err);
    return false;
  }

  // 建议只做垂直翻转；水平镜像在 PC 端通过 --hflip 再镜回去
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    // s->set_hmirror(s, 0);
  }
  return true;
}

// ---------- HTTP server start ----------
static void startHttp() {
  // 首页
  server.on("/", HTTP_GET, handle_root);

  // 状态
  server.on("/status", HTTP_GET, handle_status);

  // 快照
  server.on("/jpg", HTTP_GET, handle_jpg);

  // 异步 MJPEG：注册路由并“接管”客户端
  server.on("/mjpeg", HTTP_GET, [](AsyncWebServerRequest* request){
    // 拿到底层 TCP 客户端并加入列表
    AsyncClient* client = request->client();
    client->setNoDelay(true);

    // 断开回调：从列表移除
    client->onDisconnect([](void* arg, AsyncClient* c){
      if (xSemaphoreTake(g_clients_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (size_t i = 0; i < g_clients.size(); ++i) {
          if (g_clients[i].c == c) {
            c->close(true);
            delete c;
            g_clients.erase(g_clients.begin()+i);
            break;
          }
        }
        xSemaphoreGive(g_clients_mutex);
      } else {
        c->close(true);
        delete c;
      }
    }, nullptr);

    // 添加到客户端列表（稍后由 cameraTask 周期推送帧）
    if (xSemaphoreTake(g_clients_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_clients.emplace_back(new AsyncClient(*client)); // clone client
      xSemaphoreGive(g_clients_mutex);
    }

    // 这里不 request->send()，我们直接用 AsyncClient 写头+数据（由广播函数写入）
    // 立即结束 handler（异步继续）
  });

  // 控制（POST）
  server.on("/cmd", HTTP_POST,
    // onRequest
    [](AsyncWebServerRequest* request){
      // 若 body 已在 onBody 处理并发送，通常不会走到这里；保险返回 OK
      if (!request->_tempObject) request->send(200, "application/json", "{\"ok\":true}");
    },
    // onUpload (unused)
    NULL,
    // onBody
    on_cmd_body
  );

  // GET /cmd 提示
  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest* request){
    request->send(200, "text/plain", "POST JSON here, e.g. {\"M\":\"Left\",\"v\":90}");
  });

  server.begin();
  Serial.println("[http] Async server started on port 80");
}

// ---------- Arduino setup/loop ----------
void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println("[boot] ESP32 Async MJPEG (PC-side AprilTag)");

  // UART to UNO
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Camera
  if (!init_camera()) {
    Serial.println("[camera] init failed. Check pins.");
  }

  // 互斥量
  g_clients_mutex = xSemaphoreCreateMutex();
  g_jpg_mutex     = xSemaphoreCreateMutex();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[wifi] AP %s  SSID=\"%s\"  PASS=\"%s\"\n", ok ? "OK" : "FAIL", AP_SSID, AP_PASS);
  Serial.printf("[wifi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // HTTP
  startHttp();

  // Camera acquisition + broadcast tasks (run on the secondary core)
  xTaskCreatePinnedToCore(cameraTask, "camTask", 4096, nullptr, 2, &g_cam_task, 1);

  // Send a Stop signal to UNO after power on
  send_to_uno("Stop", 0);
  g_last_motion = "Stop";
  g_last_speed  = 0;
  g_last_cmd_ms = millis();
  Serial.println("Stop (initial)");
}

void loop() {
  // AsyncWebServer 无需在 loop 里反复 handle
  vTaskDelay(pdMS_TO_TICKS(100));
}

// #include <ArduinoJson-v6.11.1.h>
// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_task_wdt.h>
// #include "esp_camera.h"
// #include "Config.h"
// #include "ArduinoJson-v6.11.1.h"  // 你上传的版本 (6.11.1)

// // Async server
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <vector>

// void setup() {
//   Serial2.begin(9600, SERIAL_8N1, 3, 40); // UART2 初始化
// }

// void loop() {
//   StaticJsonDocument<100> doc;
//   doc["cmd"] = "left";
//   serializeJson(doc, Serial2);  // 序列化输出到 UART
//   delay(1000);
// }
