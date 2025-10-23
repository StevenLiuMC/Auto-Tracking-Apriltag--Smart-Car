#pragma once

// Wi-Fi
// #define WIFI_SSID      "OUR PARADISE"
// #define WIFI_PASSWORD  "o2oivs3EcdHgo_CydCRj"

// UART (ESP32 <-> UNO)
// 统一 9600 波特率（与你 UNO 保持一致）
#define UART_BAUD   9600
// 根据你的接线：ESP32 的 RX 接 UNO TX0，ESP32 的 TX 接 UNO RX0
// 示例（ESP32-S3）：RX=GPIO18, TX=GPIO17
#define UART_RX_PIN 3
#define UART_TX_PIN 40

// Camera: OV3660 on ESP32S3-EYE 
// 对应你给出的 CAMERA_MODEL_ESP32S3_EYE 引脚映射
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5

// Y2..Y9 → D0..D7
#define CAM_PIN_D0     11  // Y2
#define CAM_PIN_D1      9  // Y3
#define CAM_PIN_D2      8  // Y4
#define CAM_PIN_D3     10  // Y5
#define CAM_PIN_D4     12  // Y6
#define CAM_PIN_D5     18  // Y7
#define CAM_PIN_D6     17  // Y8
#define CAM_PIN_D7     16  // Y9

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK   13

// 摄像头像素/帧率参数（先低分辨率稳一下，再往上加）
#define CAM_XCLK_HZ       20000000   // 不稳可降到 10000000
#define CAM_FRAME_SIZE    FRAMESIZE_VGA // 初期用 QVGA；稳定后换 VGA/SVGA
#define CAM_JPEG_QUALITY  7
#define CAM_FB_COUNT      2