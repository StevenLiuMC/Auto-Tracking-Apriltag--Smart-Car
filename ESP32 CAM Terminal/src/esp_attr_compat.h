#pragma once

// 对 ESP32 平台，尽量包含官方属性定义（若可用）
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
  #include <esp_attr.h>
#endif

// 如果还是没定义，就当成空宏，避免语法错误
#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

#ifndef DRAM_ATTR
  #define DRAM_ATTR
#endif
