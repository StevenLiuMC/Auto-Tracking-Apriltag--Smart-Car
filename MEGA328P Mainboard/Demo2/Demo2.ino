#include <Arduino.h>
#include "DeviceDriverSet_xxx0.h"
#include "ArduinoJson-v6.11.1.h"

/* -------- 外部对象 -------- */
extern DeviceDriverSet_Motor      AppMotor;
extern DeviceDriverSet_ULTRASONIC AppULTRASONIC;

/* -------- 可调参数 -------- */
constexpr uint16_t kStopThresholdCm = 30;   // 障碍阈值
constexpr uint8_t  kForwardSpeed     = 50; // 前进速度
constexpr uint8_t  kTurnSpeed        = 50; // 转向速度
constexpr uint16_t kLoopIntervalMs   = 40;  // 主循环周期

/* -------- 指令定义 -------- */
enum CmdType { CMD_NONE, CMD_FORWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP };

struct CmdState {
  CmdType   last = CMD_NONE;
  uint32_t  ts   = 0;
} g_cmd;

/* -------- 模式 -------- */
enum Mode { MODE_IDLE, MODE_AUTO, MODE_MANUAL };
Mode g_mode = MODE_IDLE;         // 初始静止
CmdType g_manualAction = CMD_STOP; // 手动模式下保持的动作

/* -------- 电机动作（与官方Left/Right映射一致） -------- */
static inline void goForward(uint8_t speed) {
  AppMotor.DeviceDriverSet_Motor_control(
    /*A*/direction_just, speed,
    /*B*/direction_just, speed,
    /*en*/control_enable);
}
static inline void stopCar() {
  AppMotor.DeviceDriverSet_Motor_control(
    direction_void, 0, direction_void, 0, control_enable);
}
static inline void turnLeft(uint8_t speed) {
  // Left: A forward, B back
  AppMotor.DeviceDriverSet_Motor_control(
    direction_just, speed, direction_back, speed, control_enable);
}
static inline void turnRight(uint8_t speed) {
  // Right: A back, B forward
  AppMotor.DeviceDriverSet_Motor_control(
    direction_back, speed, direction_just, speed, control_enable);
}

/* -------- 串口接收 JSON/文本 指令（立刻更新状态机） -------- */
static bool tryReadCommandFromSerial(CmdState &st) {
  if (!Serial.available()) return false;

  StaticJsonDocument<160> doc;
  DeserializationError err = deserializeJson(doc, Serial);
  if (!err) {
    const char* s = nullptr;
    if (doc.containsKey("M")) s = doc["M"];
    else if (doc.containsKey("cmd")) s = doc["cmd"];
    if (s) {
      String ss(s); ss.trim(); ss.toLowerCase();
      CmdType c = CMD_NONE;
      if (ss == "forward") c = CMD_FORWARD;
      else if (ss == "left") c = CMD_LEFT;
      else if (ss == "right") c = CMD_RIGHT;
      else if (ss == "stop") c = CMD_STOP;

      if (c != CMD_NONE) {
        st.last = c;
        st.ts = millis();
        Serial.print(F("[CMD] ")); Serial.println(ss);

        // —— 立刻执行并切换模式 ——
        switch (c) {
          case CMD_FORWARD:
            g_mode = MODE_AUTO;          // 进入自动巡航
            break;
          case CMD_LEFT:
            g_mode = MODE_MANUAL;        // 进入手动
            g_manualAction = CMD_LEFT;   // 立刻左转，并保持
            turnLeft(kTurnSpeed);
            break;
          case CMD_RIGHT:
            g_mode = MODE_MANUAL;
            g_manualAction = CMD_RIGHT;
            turnRight(kTurnSpeed);
            break;
          case CMD_STOP:
            g_mode = MODE_MANUAL;
            g_manualAction = CMD_STOP;
            stopCar();
            break;
          default: break;
        }
        return true;
      }
    }
    return false;
  }

  // 文本兜底
  String line = Serial.readStringUntil('\n'); line.trim(); line.toLowerCase();
  if (line.length()) {
    CmdType c = CMD_NONE;
    if (line == "forward") c = CMD_FORWARD;
    else if (line == "left") c = CMD_LEFT;
    else if (line == "right") c = CMD_RIGHT;
    else if (line == "stop") c = CMD_STOP;

    if (c != CMD_NONE) {
      st.last = c;
      st.ts = millis();
      Serial.print(F("[CMD-TXT] ")); Serial.println(line);

      switch (c) {
        case CMD_FORWARD: g_mode = MODE_AUTO; break;
        case CMD_LEFT:    g_mode = MODE_MANUAL; g_manualAction = CMD_LEFT;  turnLeft(kTurnSpeed);  break;
        case CMD_RIGHT:   g_mode = MODE_MANUAL; g_manualAction = CMD_RIGHT; turnRight(kTurnSpeed); break;
        case CMD_STOP:    g_mode = MODE_MANUAL; g_manualAction = CMD_STOP;  stopCar();             break;
        default: break;
      }
      return true;
    }
  }
  return false;
}

/* -------- 初始化 -------- */
void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();

  Serial.println(F("Combo v4: Idle on power-up; Left/Right/Stop act immediately; Forward enters AUTO"));
  stopCar();                 // 上电静止
  g_mode = MODE_IDLE;
  g_manualAction = CMD_STOP;
}

/* -------- 主循环 -------- */
void loop() {
  static uint32_t lastRun = 0;
  const uint32_t now = millis();
  if (now - lastRun < kLoopIntervalMs) return;
  lastRun = now;

  // 先读取并立即处理可能的指令（会直接改变模式/动作）
  tryReadCommandFromSerial(g_cmd);

  // 根据当前模式运行
  if (g_mode == MODE_AUTO) {
    uint16_t distance_cm = 0;
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance_cm);

    if (distance_cm == 0 || distance_cm <= kStopThresholdCm) {
      stopCar();
      // 仍然处于 AUTO，只是被安全停车
      // 等下一次距离>阈值自动继续前进
      // 注意：收到 Stop/Left/Right 会立即切换到手动
      Serial.print(F("AUTO: ")); Serial.print(distance_cm); Serial.println(F(" cm -> STOP(safety)"));
    } else {
      goForward(kForwardSpeed);
      Serial.print(F("AUTO: ")); Serial.print(distance_cm); Serial.println(F(" cm -> Forward"));
    }
  } else if (g_mode == MODE_MANUAL) {
    // 手动模式：持续执行上一次动作，直到新指令改变它
    switch (g_manualAction) {
      case CMD_LEFT:  turnLeft(kTurnSpeed);  break;
      case CMD_RIGHT: turnRight(kTurnSpeed); break;
      case CMD_STOP:  stopCar();             break;
      default:        stopCar();             break;
    }
  } else { // MODE_IDLE
    stopCar();
  }
}
