// ============================================================
//  NTP Clock v4.0 — ESP32-C3-WROOM-02
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <DNSServer.h>

// ============================================================
// РАЗДЕЛ 1: КОНФИГУРАЦИЯ
// ============================================================

#define CONFIG_SCREEN_TIME_MS           10000  // экран "Время"                        (мс)
#define CONFIG_SCREEN_TEMP_MS            3000  // экран "Температура"                  (мс)
#define CONFIG_SCREEN_HUM_MS             3000  // экран "Влажность"                    (мс)
#define CONFIG_SENSOR_POLL_MS           10000  // период опроса AHT21B                 (мс)
#define CONFIG_AHT_MEASURE_MS              85  // ожидание готовности AHT21B           (мс)
#define CONFIG_AHT_REINIT_MS             5000  // повтор инициализации AHT21B          (мс)
#define CONFIG_AHT_TEMP_OFFSET           -4.0f // сдвиг температуры калибровка         (°C)
#define CONFIG_AHT_FAIL_MAX                 3  // ошибок подряд до переинициализации
#define CONFIG_HUM_THRESHOLD               75  // порог влажности для LED              (%)
#define CONFIG_BRIGHTNESS_DEFAULT           1  // яркость по умолчанию               (1..5)
#define CONFIG_SETTINGS_TIMEOUT_MS      15000  // таймаут настройки без нажатий       (мс)
#define CONFIG_LONG_PRESS_MS             1000  // длина длинного нажатия               (мс)
#define CONFIG_DEBOUNCE_MS                 50  // антидребезг кнопок                   (мс)
#define CONFIG_BLINK_MS                   500  // период мигания цифр                  (мс)
#define CONFIG_TRIPLE_WINDOW_MS           800  // окно тройного нажатия MINUS          (мс)
#define CONFIG_LED_BLINK_ON_MS            150  // вспышка LED при старте               (мс)
#define CONFIG_LED_BLINK_OFF_MS           150  // пауза между вспышками LED            (мс)
#define CONFIG_LED_BLINK_COUNT              3  // количество вспышек при старте
#define CONFIG_RENDER_INTERVAL_MS          10  // интервал обновления дисплея          (мс)
#define CONFIG_I2C_HW_FREQ            50000UL  // частота HW I2C для TM1650            (Гц)
#define CONFIG_I2C_SW_DELAY_US              5  // полупериод SW I2C для AHT21B        (мкс)
#define CONFIG_SW_SCL_TIMEOUT_US         1000  // таймаут clock stretching            (мкс)
#define CONFIG_TM_FAIL_MAX                  5  // ошибок TM1650 до переинициализации
#define CONFIG_WDT_TIMEOUT_S                8  // таймаут watchdog                      (с)

#define CONFIG_NTP_SYNC_INTERVAL_H         12  // автосинхронизация каждые N часов
#define CONFIG_WIFI_CONNECT_TIMEOUT_MS  15000  // таймаут подключения к Wi-Fi         (мс)
#define CONFIG_WIFI_AP_TIMEOUT_MS      120000  // таймаут точки доступа без активности(мс)
#define CONFIG_WIFI_RESULT_MS            2000  // время показа done/Err на дисплее    (мс)
#define CONFIG_NTP_TIMEOUT_MS           10000  // таймаут ожидания NTP ответа         (мс)
#define CONFIG_NTP_SERVER_1     "pool.ntp.org"        // NTP сервер 1
#define CONFIG_NTP_SERVER_2     "time.google.com"     // NTP сервер 2
#define CONFIG_NTP_SERVER_3     "time.cloudflare.com" // NTP сервер 3
#define CONFIG_NTP_TIMEZONE     "MSK-3"               // часовой пояс (POSIX формат, UTC+3)
#define CONFIG_AP_SSID          "NTP-Clock"           // SSID точки доступа
#define CONFIG_AP_PASSWORD      "12345678"            // пароль точки доступа (мин. 8 символов)
#define CONFIG_NVS_NAMESPACE    "ntpclock"            // пространство имён NVS
#define CONFIG_NVS_KEY_SSID     "ssid"                // ключ NVS для SSID
#define CONFIG_NVS_KEY_PASS     "pass"                // ключ NVS для пароля
#define CONFIG_AP_TRIGGER_MS     2000                 // удержание + и − для запуска AP  (мс)

// ============================================================
// РАЗДЕЛ 2: ПИНЫ
// ============================================================

#define PIN_SDA_TM      0
#define PIN_SCL_TM      1
#define PIN_SDA_AHT     4
#define PIN_SCL_AHT     5
#define PIN_BTN_SELECT  8
#define PIN_BTN_PLUS    2
#define PIN_BTN_MINUS   9
#define PIN_LED        10

// ============================================================
// РАЗДЕЛ 3: АДРЕСА И КОНСТАНТЫ TM1650
// ============================================================

#define TM1650_CTRL_ADDR    0x24
#define TM1650_DIG1_ADDR    0x34
#define TM1650_DIG2_ADDR    0x35
#define TM1650_DIG3_ADDR    0x36
#define TM1650_DIG4_ADDR    0x37
#define AHT21_ADDR          0x38
#define TM1650_DISPLAY_ON   0x01
#define TM1650_DISPLAY_OFF  0x00

// ============================================================
// РАЗДЕЛ 4: АППАРАТНЫЙ I2C — TM1650
// ============================================================

TwoWire I2C_TM = TwoWire(0);

// ============================================================
// РАЗДЕЛ 5: ПРОГРАММНЫЙ I2C — AHT21B
// ============================================================

static inline void sw_sda_high() { pinMode(PIN_SDA_AHT, INPUT);               }
static inline void sw_sda_low()  { pinMode(PIN_SDA_AHT, OUTPUT);
                                   digitalWrite(PIN_SDA_AHT, LOW);             }
static inline void sw_scl_low()  { pinMode(PIN_SCL_AHT, OUTPUT);
                                   digitalWrite(PIN_SCL_AHT, LOW);             }
static inline bool sw_sda_read() { return (bool)digitalRead(PIN_SDA_AHT);     }
static inline void sw_delay()    { delayMicroseconds(CONFIG_I2C_SW_DELAY_US); }

static bool sw_scl_wait_high() {
  pinMode(PIN_SCL_AHT, INPUT);
  unsigned long start = micros();
  while (!digitalRead(PIN_SCL_AHT)) {
    if ((unsigned long)(micros() - start) >= CONFIG_SW_SCL_TIMEOUT_US)
      return false;
  }
  return true;
}

static void sw_bus_recovery() {
  sw_sda_high();
  for (uint8_t i = 0; i < 9; i++) {
    if (!sw_scl_wait_high()) break;
    sw_delay();
    sw_scl_low(); sw_delay();
  }
  sw_scl_low();       sw_delay();
  sw_sda_low();       sw_delay();
  sw_scl_wait_high(); sw_delay();
  sw_sda_high();      sw_delay();
}

static void sw_start() {
  sw_sda_high(); sw_delay();
  if (!sw_scl_wait_high()) { sw_bus_recovery(); return; }
  sw_delay();
  sw_sda_low();  sw_delay();
  sw_scl_low();  sw_delay();
}

static void sw_stop() {
  sw_sda_low();  sw_delay();
  if (!sw_scl_wait_high()) { sw_bus_recovery(); return; }
  sw_delay();
  sw_sda_high(); sw_delay();
}

static bool sw_write_byte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    if (data & (1 << i)) sw_sda_high();
    else                  sw_sda_low();
    sw_delay();
    if (!sw_scl_wait_high()) return false;
    sw_delay();
    sw_scl_low(); sw_delay();
  }
  sw_sda_high(); sw_delay();
  if (!sw_scl_wait_high()) return false;
  sw_delay();
  bool ack = !sw_sda_read();
  sw_scl_low(); sw_delay();
  return ack;
}

static bool sw_read_byte(bool sendAck, uint8_t &outData) {
  outData = 0;
  sw_sda_high();
  for (int i = 7; i >= 0; i--) {
    sw_delay();
    if (!sw_scl_wait_high()) return false;
    sw_delay();
    if (sw_sda_read()) outData |= (1 << i);
    sw_scl_low();
  }
  if (sendAck) sw_sda_low();
  else          sw_sda_high();
  sw_delay();
  if (!sw_scl_wait_high()) return false;
  sw_delay();
  sw_scl_low(); sw_delay();
  sw_sda_high();
  return true;
}

// ============================================================
// РАЗДЕЛ 6: ШРИФТ 7-СЕГМЕНТНОГО ИНДИКАТОРА
// ============================================================

static const uint8_t FONT_DIGIT[10] = {
  0x3F, 0x06, 0x5B, 0x4F,
  0x66, 0x6D, 0x7D, 0x07,
  0x7F, 0x6F
};

#define SEG_BLANK   0x00
#define SEG_MINUS   0x40
#define SEG_DEGREE  0x63
#define SEG_C_CHAR  0x39
#define SEG_H_CHAR  0x76
#define SEG_B_LOWER 0x7C
#define SEG_R_LOWER 0x50
#define SEG_DP      0x80
#define SEG_N_LOWER 0x54
#define SEG_O_LOWER 0x5C
#define SEG_T_LOWER 0x78
#define SEG_P_UPPER 0x73
#define SEG_A_UPPER 0x77
#define SEG_D_LOWER 0x5E
#define SEG_E_UPPER 0x79

// ============================================================
// РАЗДЕЛ 7: СОСТОЯНИЯ
// ============================================================

enum Screen  : uint8_t { SCR_TIME = 0, SCR_TEMP = 1, SCR_HUM = 2 };
enum Setting : uint8_t { SET_NONE, SET_HOURS, SET_MINUTES, SET_BRIGHT };

enum WifiState : uint8_t {
  WS_IDLE,
  WS_CONNECTING,
  WS_NTP_SYNCING,
  WS_AP_ACTIVE,
  WS_AP_CONNECTING,
  WS_DONE,
  WS_ERROR
};

static const unsigned long SCREEN_DURATIONS[3] = {
  CONFIG_SCREEN_TIME_MS,
  CONFIG_SCREEN_TEMP_MS,
  CONFIG_SCREEN_HUM_MS
};

// ============================================================
// РАЗДЕЛ 8: ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================

static uint8_t       g_hours           = 0;
static uint8_t       g_minutes         = 0;
static uint8_t       g_seconds         = 0;
static unsigned long g_lastSecTick     = 0;

static float         g_tempC           = 0.0f;
static uint8_t       g_humidity        = 0;
static bool          g_sensorDataValid = false;
static unsigned long g_lastAhtPoll     = 0;
static bool          g_ahtReady        = false;
static bool          g_ahtMeasuring    = false;
static unsigned long g_ahtTriggerTime  = 0;
static unsigned long g_lastAhtInit     = 0;
static uint8_t       g_ahtFailCount    = 0;

static uint8_t       g_tmFailCount     = 0;

static Screen        g_screen          = SCR_TIME;
static unsigned long g_screenTimer     = 0;

static Setting       g_setting         = SET_NONE;
static uint8_t       g_editHours       = 0;
static uint8_t       g_editMinutes     = 0;
static uint8_t       g_editBright      = CONFIG_BRIGHTNESS_DEFAULT;
static unsigned long g_settingTimer    = 0;

static uint8_t       g_brightness      = CONFIG_BRIGHTNESS_DEFAULT;
static uint8_t       g_dispBuf[4]      = {0, 0, 0, 0};
static unsigned long g_lastRender      = 0;

static bool          g_blinkState      = true;
static unsigned long g_lastBlink       = 0;

static bool          g_sleepMode       = false;

static uint8_t       g_minusCount      = 0;
static unsigned long g_minusFirstTime  = 0;

static WifiState     g_wifiState       = WS_IDLE;
static unsigned long g_wifiTimer       = 0;
static unsigned long g_lastNtpSync     = 0;
static bool          g_ntpEverSynced   = false;
static bool          g_apTriggerArmed  = false;
static unsigned long g_apTriggerStart  = 0;
static char          g_pendingSsid[64] = {0};
static char          g_pendingPass[64] = {0};

static Preferences   g_prefs;
static WebServer     g_server(80);
static DNSServer     g_dnsServer;

// ============================================================
// РАЗДЕЛ 9: TM1650
// ============================================================

static uint8_t tm_write(uint8_t addr, uint8_t data) {
  I2C_TM.beginTransmission(addr);
  I2C_TM.write(data);
  return I2C_TM.endTransmission();
}

static void tm_reinit() {
  tm_write(TM1650_CTRL_ADDR,
    (uint8_t)((constrain(g_brightness, 1, 5) << 4) | TM1650_DISPLAY_ON));
}

static void tm_setBrightness(uint8_t level, bool on) {
  level = constrain(level, 1, 5);
  uint8_t ctrl = (uint8_t)((level << 4) | (on ? TM1650_DISPLAY_ON : TM1650_DISPLAY_OFF));
  tm_write(TM1650_CTRL_ADDR, ctrl);
}

static void tm_sendBuffer(const uint8_t buf[4]) {
  uint8_t e1 = tm_write(TM1650_DIG1_ADDR, buf[0]);
  uint8_t e2 = tm_write(TM1650_DIG2_ADDR, buf[1]);
  uint8_t e3 = tm_write(TM1650_DIG3_ADDR, buf[2]);
  uint8_t e4 = tm_write(TM1650_DIG4_ADDR, buf[3]);
  if (e1 || e2 || e3 || e4) {
    if (++g_tmFailCount >= CONFIG_TM_FAIL_MAX) { g_tmFailCount = 0; tm_reinit(); }
  } else {
    g_tmFailCount = 0;
  }
}

static void tm_clear() {
  const uint8_t blank[4] = {0, 0, 0, 0};
  tm_sendBuffer(blank);
}

static void tm_off() { tm_setBrightness(g_brightness, false); tm_clear(); }
static void tm_on()  { tm_setBrightness(g_brightness, true);              }

// ============================================================
// РАЗДЕЛ 10: AHT21B
// ============================================================

static uint8_t crc8_aht(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static bool aht_init() {
  sw_start();
  if (!sw_write_byte((uint8_t)(AHT21_ADDR << 1)) ||
      !sw_write_byte(0xBE) || !sw_write_byte(0x08) || !sw_write_byte(0x00)) {
    sw_bus_recovery(); return false;
  }
  sw_stop(); return true;
}

static bool aht_trigger() {
  sw_start();
  if (!sw_write_byte((uint8_t)(AHT21_ADDR << 1)) ||
      !sw_write_byte(0xAC) || !sw_write_byte(0x33) || !sw_write_byte(0x00)) {
    sw_bus_recovery(); return false;
  }
  sw_stop(); return true;
}

static bool aht_collect(float &outTemp, uint8_t &outHum) {
  sw_start();
  if (!sw_write_byte((uint8_t)((AHT21_ADDR << 1) | 0x01))) {
    sw_bus_recovery(); return false;
  }
  uint8_t d[7];
  for (uint8_t i = 0; i < 7; i++) {
    if (!sw_read_byte(i < 6, d[i])) { sw_bus_recovery(); return false; }
  }
  sw_stop();
  if (d[0] & 0x80)            return false;
  if (crc8_aht(d, 6) != d[6]) return false;
  uint32_t rawH = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | ((uint32_t)(d[3] >> 4));
  uint32_t rawT = (((uint32_t)(d[3] & 0x0F)) << 16) | ((uint32_t)d[4] << 8) | ((uint32_t)d[5]);
  outTemp = ((float)rawT / 1048576.0f) * 200.0f - 50.0f + CONFIG_AHT_TEMP_OFFSET;
  float h = ((float)rawH / 1048576.0f) * 100.0f;
  outHum  = (uint8_t)constrain((int)(h + 0.5f), 0, 100);
  return true;
}

// ============================================================
// РАЗДЕЛ 11: ПОСТРОЕНИЕ БУФЕРА ДИСПЛЕЯ
// ============================================================

static void buildTime(uint8_t h, uint8_t m, bool colonOn) {
  g_dispBuf[0] = FONT_DIGIT[h / 10];
  g_dispBuf[1] = FONT_DIGIT[h % 10];
  g_dispBuf[2] = FONT_DIGIT[m / 10] | (colonOn ? SEG_DP : 0);
  g_dispBuf[3] = FONT_DIGIT[m % 10] | (colonOn ? SEG_DP : 0);
}

static void buildTemp(float t) {
  int temp = (int)(t >= 0.0f ? t + 0.5f : t - 0.5f);
  temp = constrain(temp, -99, 99);
  if (temp >= 0) {
    if (temp >= 10) {
      g_dispBuf[0] = FONT_DIGIT[temp / 10]; g_dispBuf[1] = FONT_DIGIT[temp % 10];
      g_dispBuf[2] = SEG_DEGREE;            g_dispBuf[3] = SEG_C_CHAR;
    } else {
      g_dispBuf[0] = SEG_BLANK;             g_dispBuf[1] = FONT_DIGIT[temp];
      g_dispBuf[2] = SEG_DEGREE;            g_dispBuf[3] = SEG_C_CHAR;
    }
  } else {
    int a = -temp;
    if (a < 10) {
      g_dispBuf[0] = SEG_MINUS;             g_dispBuf[1] = FONT_DIGIT[a];
      g_dispBuf[2] = SEG_DEGREE;            g_dispBuf[3] = SEG_C_CHAR;
    } else {
      g_dispBuf[0] = SEG_MINUS;             g_dispBuf[1] = FONT_DIGIT[a / 10];
      g_dispBuf[2] = FONT_DIGIT[a % 10];   g_dispBuf[3] = SEG_DEGREE;
    }
  }
}

static void buildHum(uint8_t h) {
  if (h >= 100) {
    g_dispBuf[0] = SEG_H_CHAR;   g_dispBuf[1] = FONT_DIGIT[1];
    g_dispBuf[2] = FONT_DIGIT[0]; g_dispBuf[3] = FONT_DIGIT[0];
  } else {
    g_dispBuf[0] = SEG_H_CHAR;   g_dispBuf[1] = SEG_BLANK;
    g_dispBuf[2] = FONT_DIGIT[h / 10]; g_dispBuf[3] = FONT_DIGIT[h % 10];
  }
}

static void buildDashes() {
  g_dispBuf[0] = g_dispBuf[1] = g_dispBuf[2] = g_dispBuf[3] = SEG_MINUS;
}

static void buildBrightness(uint8_t level, bool digitOn) {
  g_dispBuf[0] = SEG_B_LOWER; g_dispBuf[1] = SEG_R_LOWER;
  g_dispBuf[2] = SEG_BLANK;   g_dispBuf[3] = digitOn ? FONT_DIGIT[level] : SEG_BLANK;
}

// "conn"
static void buildConn() {
  g_dispBuf[0] = SEG_C_CHAR;   g_dispBuf[1] = SEG_O_LOWER;
  g_dispBuf[2] = SEG_N_LOWER;  g_dispBuf[3] = SEG_N_LOWER;
}

// "ntp " (мигает)
static void buildNtp(bool on) {
  if (on) {
    g_dispBuf[0] = SEG_N_LOWER;  g_dispBuf[1] = SEG_T_LOWER;
    g_dispBuf[2] = SEG_P_UPPER;  g_dispBuf[3] = SEG_BLANK;
  } else {
    g_dispBuf[0] = g_dispBuf[1] = g_dispBuf[2] = g_dispBuf[3] = SEG_BLANK;
  }
}

// "AP  " (мигает)
static void buildAP(bool on) {
  if (on) {
    g_dispBuf[0] = SEG_A_UPPER;  g_dispBuf[1] = SEG_P_UPPER;
    g_dispBuf[2] = SEG_BLANK;    g_dispBuf[3] = SEG_BLANK;
  } else {
    g_dispBuf[0] = g_dispBuf[1] = g_dispBuf[2] = g_dispBuf[3] = SEG_BLANK;
  }
}

// "donE"
static void buildDone() {
  g_dispBuf[0] = SEG_D_LOWER;  g_dispBuf[1] = SEG_O_LOWER;
  g_dispBuf[2] = SEG_N_LOWER;  g_dispBuf[3] = SEG_E_UPPER;
}

// "Err "
static void buildErr() {
  g_dispBuf[0] = SEG_E_UPPER;  g_dispBuf[1] = SEG_R_LOWER;
  g_dispBuf[2] = SEG_R_LOWER;  g_dispBuf[3] = SEG_BLANK;
}

// ============================================================
// РАЗДЕЛ 12: РЕНДЕР
// ============================================================

static void render(unsigned long now) {
  if (now - g_lastBlink >= CONFIG_BLINK_MS) {
    g_lastBlink  = now;
    g_blinkState = !g_blinkState;
  }

  bool colonOn = (now - g_lastSecTick) < 500;

  switch (g_wifiState) {
    case WS_CONNECTING:
    case WS_AP_CONNECTING:
      buildConn();
      break;
    case WS_NTP_SYNCING:
      buildNtp(g_blinkState);
      break;
    case WS_AP_ACTIVE:
      buildAP(g_blinkState);
      break;
    case WS_DONE:
      buildDone();
      break;
    case WS_ERROR:
      buildErr();
      break;
    default:
      if (g_setting != SET_NONE) {
        if (g_setting == SET_BRIGHT) {
          buildBrightness(g_editBright, g_blinkState);
        } else {
          buildTime(g_editHours, g_editMinutes, colonOn);
          if (!g_blinkState) {
            switch (g_setting) {
              case SET_HOURS:
                g_dispBuf[0] = SEG_BLANK; g_dispBuf[1] = SEG_BLANK;
                break;
              case SET_MINUTES:
                g_dispBuf[2] = colonOn ? SEG_DP : SEG_BLANK;
                g_dispBuf[3] = colonOn ? SEG_DP : SEG_BLANK;
                break;
              default: break;
            }
          }
        }
      } else {
        switch (g_screen) {
          case SCR_TIME: buildTime(g_hours, g_minutes, colonOn); break;
          case SCR_TEMP: g_sensorDataValid ? buildTemp(g_tempC)    : buildDashes(); break;
          case SCR_HUM:  g_sensorDataValid ? buildHum(g_humidity)  : buildDashes(); break;
        }
      }
      break;
  }

  tm_sendBuffer(g_dispBuf);
}

// ============================================================
// РАЗДЕЛ 13: LED
// ============================================================

static void ledStartupBlink() {
  for (uint8_t i = 0; i < CONFIG_LED_BLINK_COUNT; i++) {
    digitalWrite(PIN_LED, HIGH); delay(CONFIG_LED_BLINK_ON_MS);
    digitalWrite(PIN_LED, LOW);
    if (i < CONFIG_LED_BLINK_COUNT - 1) delay(CONFIG_LED_BLINK_OFF_MS);
  }
}

static void updateLed() {
  bool state = (!g_sleepMode && g_sensorDataValid &&
                g_humidity > CONFIG_HUM_THRESHOLD);
  static bool lastState = false;
  if (state != lastState) {
    lastState = state;
    digitalWrite(PIN_LED, state ? HIGH : LOW);
  }
}

// ============================================================
// РАЗДЕЛ 14: ТИХИЙ РЕЖИМ
// ============================================================

static void enterSleep() {
  g_sleepMode    = true;
  g_ahtMeasuring = false;
  tm_off();
  digitalWrite(PIN_LED, LOW);
}

static void exitSleep(unsigned long now) {
  g_sleepMode    = false;
  g_ahtMeasuring = false;
  g_lastAhtPoll  = now - CONFIG_SENSOR_POLL_MS;
  g_screen       = SCR_TIME;
  g_screenTimer  = now;
  tm_on();
}

// ============================================================
// РАЗДЕЛ 15: КНОПКИ
// ============================================================

struct Button {
  uint8_t       pin;
  bool          lastRead;
  bool          stableState;
  unsigned long debounceTime;
  unsigned long pressStart;
  bool          longFired;
};

static Button btnSelect = { PIN_BTN_SELECT, true, true, 0, 0, false };
static Button btnPlus   = { PIN_BTN_PLUS,   true, true, 0, 0, false };
static Button btnMinus  = { PIN_BTN_MINUS,  true, true, 0, 0, false };

static uint8_t buttonUpdate(Button &b, unsigned long now) {
  bool    reading = (bool)digitalRead(b.pin);
  uint8_t event   = 0;
  if (reading != b.lastRead) b.debounceTime = now;
  b.lastRead = reading;
  if ((now - b.debounceTime) > CONFIG_DEBOUNCE_MS) {
    bool prev     = b.stableState;
    b.stableState = reading;
    if (prev && !b.stableState) { b.pressStart = now; b.longFired = false; }
    if (!b.stableState && !b.longFired &&
        (now - b.pressStart) >= CONFIG_LONG_PRESS_MS) {
      b.longFired = true; event = 2;
    }
    if (!prev && b.stableState && !b.longFired) event = 1;
  }
  return event;
}

// ============================================================
// РАЗДЕЛ 16: NVS — ХРАНЕНИЕ CREDENTIALS
// ============================================================

static bool nvs_loadCredentials(char *ssid, char *pass, size_t maxLen) {
  g_prefs.begin(CONFIG_NVS_NAMESPACE, true);
  String s = g_prefs.getString(CONFIG_NVS_KEY_SSID, "");
  String p = g_prefs.getString(CONFIG_NVS_KEY_PASS, "");
  g_prefs.end();
  if (s.length() == 0) return false;
  strncpy(ssid, s.c_str(), maxLen - 1);
  strncpy(pass, p.c_str(), maxLen - 1);
  return true;
}

static void nvs_saveCredentials(const char *ssid, const char *pass) {
  g_prefs.begin(CONFIG_NVS_NAMESPACE, false);
  g_prefs.putString(CONFIG_NVS_KEY_SSID, ssid);
  g_prefs.putString(CONFIG_NVS_KEY_PASS, pass);
  g_prefs.end();
}

static void nvs_clearCredentials() {
  g_prefs.begin(CONFIG_NVS_NAMESPACE, false);
  g_prefs.remove(CONFIG_NVS_KEY_SSID);
  g_prefs.remove(CONFIG_NVS_KEY_PASS);
  g_prefs.end();
}

// ============================================================
// РАЗДЕЛ 17: WIFI / NTP
// ============================================================

static void wifi_applyTime(unsigned long ts) {
  struct timeval tv;
  tv.tv_sec  = (time_t)ts;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  setenv("TZ", CONFIG_NTP_TIMEZONE, 1);
  tzset();
  struct tm ti;
  if (getLocalTime(&ti)) {
    g_hours   = (uint8_t)ti.tm_hour;
    g_minutes = (uint8_t)ti.tm_min;
    g_seconds = (uint8_t)ti.tm_sec;
    g_lastSecTick = millis();
  }
}

static void wifi_applyNtp() {
  struct tm ti;
  if (getLocalTime(&ti, 0)) {
    g_hours   = (uint8_t)ti.tm_hour;
    g_minutes = (uint8_t)ti.tm_min;
    g_seconds = (uint8_t)ti.tm_sec;
    g_lastSecTick = millis();
  }
}

static void wifi_stopAll() {
  g_dnsServer.stop();
  g_server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

static const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>NTP Clock</title>
<style>
*{box-sizing:border-box}
body{font-family:Arial,sans-serif;max-width:420px;margin:0 auto;padding:16px;background:#f0f0f0}
h1{text-align:center;color:#333;font-size:22px;margin-bottom:4px}
.card{background:#fff;border-radius:8px;padding:14px;margin:10px 0;box-shadow:0 2px 4px rgba(0,0,0,.1)}
.card h3{margin:0 0 10px;color:#555;font-size:14px;text-transform:uppercase;letter-spacing:.5px}
input{width:100%;padding:10px;margin:4px 0 8px;border:1px solid #ddd;border-radius:4px;font-size:14px;outline:none}
input:focus{border-color:#2196F3}
button{width:100%;padding:12px;border:none;border-radius:4px;color:#fff;font-size:15px;cursor:pointer;font-weight:bold}
.blue{background:#2196F3}.blue:hover{background:#1976D2}
.red{background:#f44336}.red:hover{background:#d32f2f}
#st{text-align:center;min-height:28px;padding:6px;font-weight:bold;font-size:15px;border-radius:4px;margin-bottom:4px}
.ok{background:#e8f5e9;color:#2e7d32}
.er{background:#ffebee;color:#c62828}
.wt{background:#fff8e1;color:#f57f17}
</style>
</head>
<body>
<h1>&#128336; NTP Clock</h1>
<div id='st'></div>
<div class='card'>
<h3>1 — Подключить к Wi-Fi</h3>
<input type='text' id='ss' placeholder='Имя сети (SSID)' autocomplete='off' autocorrect='off' spellcheck='false'>
<input type='password' id='pw' placeholder='Пароль сети'>
<button class='blue' onclick='saveWifi()'>Сохранить и синхронизировать</button>
</div>
<div class='card'>
<h3>2 — Установить время вручную</h3>
<button class='blue' onclick='syncTime()'>Синхронизировать с браузером</button>
</div>
<div class='card'>
<h3>3 — Сбросить Wi-Fi</h3>
<button class='red' onclick='clearWifi()'>Сбросить настройки и установить время</button>
</div>
<script>
var el=document.getElementById('st');
function show(m,c){el.innerHTML=m;el.className=c}
function ts(){return Math.floor(Date.now()/1000)}
function get(url){
  show('Отправка...','wt');
  fetch(url).then(function(r){return r.text()})
  .then(function(t){show(t,'ok')})
  .catch(function(){show('Ошибка связи','er')});
}
function saveWifi(){
  var s=document.getElementById('ss').value.trim();
  if(!s){show('Введите SSID','er');return}
  var p=document.getElementById('pw').value;
  get('/save?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)+'&ts='+ts());
}
function syncTime(){get('/synctime?ts='+ts())}
function clearWifi(){
  if(!confirm('Сбросить Wi-Fi настройки?'))return;
  get('/clearwifi?ts='+ts());
}
</script>
</body>
</html>
)rawliteral";

static void wifi_handleRoot() {
  g_server.send_P(200, "text/html", HTML_PAGE);
}

static void wifi_handleSave() {
  if (!g_server.hasArg("ssid") || !g_server.hasArg("ts")) {
    g_server.send(400, "text/plain", "Bad request");
    return;
  }
  String ssid = g_server.arg("ssid");
  String pass = g_server.arg("pass");
  unsigned long ts = (unsigned long)g_server.arg("ts").toInt();

  nvs_saveCredentials(ssid.c_str(), pass.c_str());
  if (ts > 1000000000UL) wifi_applyTime(ts);

  strncpy(g_pendingSsid, ssid.c_str(), sizeof(g_pendingSsid) - 1);
  strncpy(g_pendingPass, pass.c_str(), sizeof(g_pendingPass) - 1);

  g_server.send(200, "text/plain",
    "Сохранено! Подключаюсь к " + ssid + "...");

  delay(200);
  wifi_stopAll();
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_pendingSsid, g_pendingPass);
  g_wifiState = WS_AP_CONNECTING;
  g_wifiTimer = millis();
}

static void wifi_handleSyncTime() {
  if (!g_server.hasArg("ts")) {
    g_server.send(400, "text/plain", "Bad request");
    return;
  }
  unsigned long ts = (unsigned long)g_server.arg("ts").toInt();
  if (ts > 1000000000UL) {
    wifi_applyTime(ts);
    g_server.send(200, "text/plain", "Время установлено!");
  } else {
    g_server.send(400, "text/plain", "Неверная метка времени");
    return;
  }
  delay(200);
  wifi_stopAll();
  g_wifiState = WS_DONE;
  g_wifiTimer = millis();
}

static void wifi_handleClearWifi() {
  if (!g_server.hasArg("ts")) {
    g_server.send(400, "text/plain", "Bad request");
    return;
  }
  unsigned long ts = (unsigned long)g_server.arg("ts").toInt();
  nvs_clearCredentials();
  if (ts > 1000000000UL) {
    wifi_applyTime(ts);
    g_server.send(200, "text/plain", "Wi-Fi сброшен. Время установлено!");
  } else {
    g_server.send(200, "text/plain", "Wi-Fi сброшен.");
  }
  delay(200);
  wifi_stopAll();
  g_wifiState = WS_DONE;
  g_wifiTimer = millis();
}

static void wifi_startConnect(const char *ssid, const char *pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  g_wifiState = WS_CONNECTING;
  g_wifiTimer = millis();
}

static void wifi_handleCaptivePortalRedirect() {
  g_server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
  g_server.send(302, "text/plain", "");
}

static void wifi_startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
  delay(500);

  g_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  g_dnsServer.setTTL(0);
  g_dnsServer.start(53, "*", WiFi.softAPIP());

  g_server.on("/",          HTTP_GET, wifi_handleRoot);
  g_server.on("/save",      HTTP_GET, wifi_handleSave);
  g_server.on("/synctime",  HTTP_GET, wifi_handleSyncTime);
  g_server.on("/clearwifi", HTTP_GET, wifi_handleClearWifi);

  // Android
  g_server.on("/generate_204",        HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/gen_204",             HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/mobile/status.php",   HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/generate204",         HTTP_GET, wifi_handleCaptivePortalRedirect);

  // iOS / macOS — 302 говорит системе "это captive portal" → сразу открывает WebSheet
  g_server.on("/hotspot-detect.html",       HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/library/test/success.html", HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/bag",                       HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/captive.apple.com",         HTTP_GET, wifi_handleCaptivePortalRedirect);

  // Windows
  g_server.on("/ncsi.txt",            HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/connecttest.txt",     HTTP_GET, wifi_handleCaptivePortalRedirect);
  g_server.on("/redirect",            HTTP_GET, wifi_handleCaptivePortalRedirect);

  // Всё остальное
  g_server.onNotFound([]() {
    wifi_handleCaptivePortalRedirect();
  });

  g_server.begin();
  g_wifiState = WS_AP_ACTIVE;
  g_wifiTimer = millis();
}

static void wifi_task(unsigned long now) {
  switch (g_wifiState) {

    case WS_CONNECTING:
    case WS_AP_CONNECTING: {
      if (WiFi.status() == WL_CONNECTED) {
        configTime(0, 0,
          CONFIG_NTP_SERVER_1,
          CONFIG_NTP_SERVER_2,
          CONFIG_NTP_SERVER_3);
        setenv("TZ", CONFIG_NTP_TIMEZONE, 1);
        tzset();
        g_wifiState = WS_NTP_SYNCING;
        g_wifiTimer = now;
      } else if (now - g_wifiTimer >= CONFIG_WIFI_CONNECT_TIMEOUT_MS) {
        wifi_stopAll();
        g_wifiState = WS_ERROR;
        g_wifiTimer = now;
      }
      break;
    }

    case WS_NTP_SYNCING: {
      struct tm ti;
      if (getLocalTime(&ti, 0)) {
        wifi_applyNtp();
        g_ntpEverSynced = true;
        g_lastNtpSync   = now;
        wifi_stopAll();
        g_wifiState = WS_DONE;
        g_wifiTimer = now;
      } else if (now - g_wifiTimer >= CONFIG_NTP_TIMEOUT_MS) {
        wifi_stopAll();
        g_wifiState = WS_ERROR;
        g_wifiTimer = now;
      }
      break;
    }

    case WS_AP_ACTIVE: {
      g_dnsServer.processNextRequest();
      g_server.handleClient();
      if (now - g_wifiTimer >= CONFIG_WIFI_AP_TIMEOUT_MS) {
        wifi_stopAll();
        g_wifiState = WS_DONE;
        g_wifiTimer = now;
      }
      break;
    }

    case WS_DONE:
    case WS_ERROR: {
      if (now - g_wifiTimer >= CONFIG_WIFI_RESULT_MS) {
        g_wifiState = WS_IDLE;
      }
      break;
    }

    case WS_IDLE: {
      if (!g_ntpEverSynced) break;
      unsigned long intervalMs =
        (unsigned long)CONFIG_NTP_SYNC_INTERVAL_H * 3600UL * 1000UL;
      if (now - g_lastNtpSync >= intervalMs) {
        char ssid[64] = {0};
        char pass[64] = {0};
        if (nvs_loadCredentials(ssid, pass, sizeof(ssid))) {
          wifi_startConnect(ssid, pass);
        } else {
          g_lastNtpSync = now;
        }
      }
      break;
    }
  }
}

// ============================================================
// РАЗДЕЛ 18: SETUP
// ============================================================

void setup() {
  esp_task_wdt_init(CONFIG_WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  pinMode(PIN_BTN_SELECT, INPUT);
  pinMode(PIN_BTN_PLUS,   INPUT);
  pinMode(PIN_BTN_MINUS,  INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  sw_sda_high(); pinMode(PIN_SCL_AHT, INPUT); delay(10);

  I2C_TM.begin(PIN_SDA_TM, PIN_SCL_TM, CONFIG_I2C_HW_FREQ);
  delay(50);

  tm_setBrightness(g_brightness, true);
  delay(10);
  tm_clear();
  delay(10);

  {
    const uint8_t t[4] = {
      FONT_DIGIT[0], FONT_DIGIT[0],
      (uint8_t)(FONT_DIGIT[0] | SEG_DP),
      (uint8_t)(FONT_DIGIT[0] | SEG_DP)
    };
    tm_sendBuffer(t);
  }
  delay(1000);
  tm_clear();

  delay(100);
  g_ahtReady    = aht_init();
  g_lastAhtInit = millis();
  delay(20);

  ledStartupBlink();

  g_hours = g_minutes = g_seconds = 0;
  unsigned long now = millis();
  g_lastSecTick = now;
  g_screenTimer = now;
  g_lastBlink   = now;
  g_lastRender  = now;
  g_lastNtpSync = now;

  if (g_ahtReady) {
    if (aht_trigger()) {
      delay(CONFIG_AHT_MEASURE_MS);
      if (aht_collect(g_tempC, g_humidity)) g_sensorDataValid = true;
    }
  }
  g_lastAhtPoll  = millis();
  g_ahtMeasuring = false;
  g_ahtFailCount = 0;
  g_tmFailCount  = 0;

  char ssid[64] = {0};
  char pass[64] = {0};
  if (nvs_loadCredentials(ssid, pass, sizeof(ssid))) {
    wifi_startConnect(ssid, pass);
  }

  esp_task_wdt_reset();
}

// ============================================================
// РАЗДЕЛ 19: ОСНОВНОЙ ЦИКЛ
// ============================================================

void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();

  while (now - g_lastSecTick >= 1000UL) {
    g_lastSecTick += 1000UL;
    if (++g_seconds >= 60) {
      g_seconds = 0;
      if (++g_minutes >= 60) {
        g_minutes = 0;
        if (++g_hours >= 24) g_hours = 0;
      }
    }
  }

  wifi_task(now);

  if (!g_sleepMode) {
    if (!g_ahtReady && (now - g_lastAhtInit >= CONFIG_AHT_REINIT_MS)) {
      g_lastAhtInit = now;
      g_ahtReady    = aht_init();
      if (g_ahtReady) {
        g_lastAhtPoll  = now - CONFIG_SENSOR_POLL_MS;
        g_ahtMeasuring = false;
        g_ahtFailCount = 0;
      }
    }
    if (g_ahtReady) {
      if (!g_ahtMeasuring && (now - g_lastAhtPoll >= CONFIG_SENSOR_POLL_MS)) {
        if (aht_trigger()) {
          g_ahtMeasuring = true; g_ahtTriggerTime = now;
        } else {
          g_lastAhtPoll = now;
        }
      }
      if (g_ahtMeasuring && (now - g_ahtTriggerTime >= CONFIG_AHT_MEASURE_MS)) {
        g_ahtMeasuring = false;
        if (aht_collect(g_tempC, g_humidity)) {
          g_lastAhtPoll = now; g_ahtFailCount = 0; g_sensorDataValid = true;
        } else {
          g_lastAhtPoll = now;
          if (++g_ahtFailCount >= CONFIG_AHT_FAIL_MAX) {
            g_ahtReady = false; g_ahtFailCount = 0;
            g_lastAhtInit = now; g_sensorDataValid = false;
          }
        }
      }
    }
  }

  updateLed();

  uint8_t evSel   = buttonUpdate(btnSelect, now);
  uint8_t evPlus  = buttonUpdate(btnPlus,   now);
  uint8_t evMinus = buttonUpdate(btnMinus,  now);

  if (g_wifiState != WS_IDLE &&
      g_wifiState != WS_DONE &&
      g_wifiState != WS_ERROR) {
    if (now - g_lastRender >= CONFIG_RENDER_INTERVAL_MS) {
      g_lastRender = now;
      render(now);
    }
    return;
  }

  if (g_sleepMode) {
    if (evSel || evPlus || evMinus) exitSleep(now);
    else return;
  }

  bool plusHeld  = !digitalRead(PIN_BTN_PLUS);
  bool minusHeld = !digitalRead(PIN_BTN_MINUS);
  if (plusHeld && minusHeld &&
      g_wifiState == WS_IDLE && g_setting == SET_NONE && !g_sleepMode) {
    if (!g_apTriggerArmed) {
      g_apTriggerArmed = true;
      g_apTriggerStart = now;
    } else if (now - g_apTriggerStart >= CONFIG_AP_TRIGGER_MS) {
      g_apTriggerArmed = false;
      wifi_startAP();
      if (now - g_lastRender >= CONFIG_RENDER_INTERVAL_MS) {
        g_lastRender = now;
        render(now);
      }
      return;
    }
  } else {
    g_apTriggerArmed = false;
  }

  if (g_setting == SET_NONE && evMinus == 1) {
    if (g_minusCount == 0 ||
        (now - g_minusFirstTime) > CONFIG_TRIPLE_WINDOW_MS) {
      g_minusCount = 1; g_minusFirstTime = now;
    } else {
      g_minusCount++;
      if (g_minusCount >= 3) {
        g_minusCount = 0; enterSleep(); return;
      }
    }
  }
  if (g_minusCount > 0 &&
      (now - g_minusFirstTime > CONFIG_TRIPLE_WINDOW_MS)) {
    g_minusCount = 0;
  }

  if (g_setting != SET_NONE) {
    if (now - g_settingTimer >= CONFIG_SETTINGS_TIMEOUT_MS) {
      g_setting = SET_NONE; g_minusCount = 0;
      g_screen  = SCR_TIME; g_screenTimer = now;
      tm_setBrightness(g_brightness, true);
      evSel = evPlus = evMinus = 0;
    }
    if (g_setting != SET_NONE) {
      if (evSel == 1) {
        g_settingTimer = now;
        switch (g_setting) {
          case SET_HOURS:   g_setting = SET_MINUTES; break;
          case SET_MINUTES: g_setting = SET_BRIGHT;  break;
          case SET_BRIGHT:  g_setting = SET_HOURS;   break;
          default: break;
        }
      }
      if (evSel == 2) {
        g_hours = g_editHours; g_minutes = g_editMinutes;
        g_seconds = 0; g_lastSecTick = now;
        g_brightness = g_editBright;
        tm_setBrightness(g_brightness, true);
        g_minusCount = 0; g_setting = SET_NONE;
        g_screen = SCR_TIME; g_screenTimer = now;
      }
      if (evPlus == 1) {
        g_settingTimer = now;
        switch (g_setting) {
          case SET_HOURS:   g_editHours   = (g_editHours   + 1) % 24; break;
          case SET_MINUTES: g_editMinutes = (g_editMinutes + 1) % 60; break;
          case SET_BRIGHT:
            g_editBright = (g_editBright % 5) + 1;
            tm_setBrightness(g_editBright, true); break;
          default: break;
        }
      }
      if (evMinus == 1) {
        g_settingTimer = now;
        switch (g_setting) {
          case SET_HOURS:   g_editHours   = (g_editHours   + 23) % 24; break;
          case SET_MINUTES: g_editMinutes = (g_editMinutes + 59) % 60; break;
          case SET_BRIGHT:
            g_editBright = (g_editBright == 1) ? 5 : g_editBright - 1;
            tm_setBrightness(g_editBright, true); break;
          default: break;
        }
      }
    }
  } else {
    if (evSel == 1) {
      g_screen = (Screen)((g_screen + 1) % 3); g_screenTimer = now;
    }
    if (evSel == 2) {
      g_minusCount = 0; g_setting = SET_HOURS;
      g_editHours = g_hours; g_editMinutes = g_minutes;
      g_editBright = g_brightness; g_settingTimer = now;
      g_screen = SCR_TIME;
      g_lastBlink = now - CONFIG_BLINK_MS / 2; g_blinkState = false;
    }
    if (now - g_screenTimer >= SCREEN_DURATIONS[g_screen]) {
      g_screen = (Screen)((g_screen + 1) % 3); g_screenTimer = now;
    }
  }

  if (now - g_lastRender >= CONFIG_RENDER_INTERVAL_MS) {
    g_lastRender = now;
    render(now);
  }
}