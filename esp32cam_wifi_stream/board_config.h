#pragma once
// Board + rover settings for the AI Thinker ESP32-CAM

// ---- Rover settings ----
#define FLIP_180           true    // true = rotate picture 180 deg. If true, do NOT also run the Pi with --flip
#define FLASH_PIN          4       // AI Thinker onboard white LED
#define FLASH_DEFAULT_ON   false    // LED is ON after boot. Can be switched at runtime: http://<ip>/flash?on=0 or 1
#define FLASH_LEVEL        40      // LED brightness 0-255 (40 = about 15%, easy on the power supply). Runtime: http://<ip>/flash?level=80

// ---- Automatic flash (runs on the ESP32 itself) ----
#define FLASH_DARK_LEVEL    60     // picture brightness (0-255) below this  -> LED turns ON
#define FLASH_BRIGHT_LEVEL  130    // picture brightness above this (LED on) -> LED turns OFF
#define FLASH_DWELL_MS      3000   // LED stays in a state at least this long (prevents flicker)
#define BRIGHTNESS_EVERY_N  30     // measure brightness every N streamed frames (higher = less CPU on battery)
#define FLASH_FAILSAFE_MS   30000  // auto mode: LED goes off if nobody has been streaming this long

// ---- AI Thinker camera pin map ----
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22