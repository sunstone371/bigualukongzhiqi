/* ============================================================================
 * 壁挂炉温控系统 —— 客厅端（温控器 / 显示端）固件
 * 目标板: ESP8266 Arduino (esp8266:esp8266:generic)
 *
 * 硬件接线:
 *  - OLED: SSD1306 128x64, 硬件 I2C (SDA=GPIO4/D2, SCL=GPIO5/D1, 地址 0x3C)
 *  - 温湿度: DHT11 数据脚 GPIO2/D4
 *    （如换 DHT22/AM2302，只需把下方 DHT_MODEL 一行的 DHT11 改为 DHT22，其它不动）
 *  - 按钮: SET=GPIO14/D5, UP=GPIO12/D6, DOWN=GPIO13/D7
 *    （INPUT_PULLUP，另一端接 GND，按下为低电平，去抖 30ms）
 *  - 注意: 不使用板载 LED（GPIO2 已被 DHT 占用）
 *
 * 行为:
 *  - 每 5 秒读一次 DHT，并通过 ESP-NOW 向广播地址发送遥测包(BK_TYPE_TELEM)
 *  - 接收炉端状态包(BK_TYPE_STATUS)，超过 45 秒未收到则界面显示 NO LINK
 *  - 主界面: 大字号室温 + 'C'、湿度 RH%、目标 Target、炉端 HEATING/IDLE/NO LINK
 *  - 设置界面: SET 进入/保存退出，UP/DOWN 步进 0.5℃（长按连发），范围 5.0~30.0，
 *    15 秒无操作自动保存退出；目标温度存 EEPROM（0xA5 标记 + uint16 setX10）
 *  - DHT 连续 3 次失败: 置 BK_FLAG_SENSOR_ERR，界面标 Err，保留最后一次有效值
 * ========================================================================== */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <math.h>

extern "C" {
  #include <espnow.h>
  #include <user_interface.h>
}

/* ============ 共享链路协议（与炉端逐字节一致，禁止改动任何字段顺序/类型/常量） ============ */
#define BK_MAGIC   0xB0C0A101UL
#define BK_LINK_ID 1
#define BK_TYPE_TELEM 1
#define BK_TYPE_STATUS 2
#define BK_FLAG_SENSOR_ERR 0x01
struct __attribute__((packed)) BoilerPacket {
  uint32_t magic; uint8_t devId; uint8_t type; uint8_t seq; uint8_t flags;
  uint16_t tempX10; uint16_t humX10; uint16_t setX10; uint8_t heatOn; uint16_t crc;
};
uint16_t bkCrc16(const uint8_t* p, size_t n) { uint16_t crc=0xFFFF; while(n--){crc^=(uint16_t)(*p++)<<8; for(uint8_t i=0;i<8;i++) crc=(crc&0x8000)?(crc<<1)^0x1021:(crc<<1);} return crc; }
/* CRC 约定: crc 字段 = bkCrc16(整包前 sizeof(BoilerPacket)-2 字节，即覆盖 crc 之前的全部字段) */
#define BK_PKT_LEN ((uint8_t)sizeof(BoilerPacket)) /* 17 字节 */
static_assert(sizeof(BoilerPacket) == 17, "BoilerPacket must be 17 bytes, same as boiler side");

/* ============================ 本机硬件配置 ============================ */
/* 炉端 SoftAP 的 SSID/密码（必须与炉端固件完全一致，用于锁定信道1） */
#define AP_SSID  "BoilerLink"
#define AP_PASS  "boilerlink2024" /* 不少于 8 位 */
#define DHT_PIN   2     /* DHT 数据脚 GPIO2(D4) */
#define DHT_MODEL DHT11 /* 换 DHT22/AM2302 只需把本行 DHT11 改为 DHT22（仅此一处） */
DHT dht(DHT_PIN, DHT_MODEL);

#define PIN_SET  14     /* SET  键 GPIO14(D5) */
#define PIN_UP   12     /* UP   键 GPIO12(D6) */
#define PIN_DOWN 13     /* DOWN 键 GPIO13(D7) */

/* OLED: 硬件 I2C（SDA=GPIO4/D2, SCL=GPIO5/D1），地址由开机 I2C 扫描自动确定
 * 若串口扫描到 0x3C/0x3D 但屏幕仍全黑，大概率是 1.3 寸 SH1106 模块：
 * 把下面的 OLED_SH1106 改为 1 后重新编译上传 */
#define OLED_SH1106 0
#if OLED_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif
static bool     oledReady = false;  /* OLED 是否已成功初始化（I2C 扫描到设备后置位） */
static uint32_t oledNextScanMs = 0; /* 未就绪时的下次重试扫描时刻 */

/* ESP-NOW 广播地址 */
static uint8_t BK_BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ============================ 参数常量 ============================ */
#define TICK_MS            5000UL  /* DHT 采样 + 遥测广播周期: 5 秒 */
#define DEBOUNCE_MS        30UL    /* 按键去抖 30ms */
#define REPEAT_DELAY_MS    500UL   /* 长按 500ms 后开始连发 */
#define REPEAT_RATE_MS     150UL   /* 连发间隔 150ms */
#define SETTING_TIMEOUT_MS 15000UL /* 设置界面 15 秒无操作自动保存退出 */
#define LINK_TIMEOUT_MS    45000UL /* 超过 45 秒未收到炉端状态包 => NO LINK */
#define DRAW_MS            200UL   /* 屏幕刷新周期 */

#define SET_MIN_X10     50   /* 目标温度下限 5.0℃ */
#define SET_MAX_X10     300  /* 目标温度上限 30.0℃ */
#define SET_DEFAULT_X10 200  /* EEPROM 数据异常时默认 20.0℃ */
#define SET_STEP_X10    5    /* UP/DOWN 步进 0.5℃ */

#define EE_SIZE      16   /* EEPROM.begin(16) */
#define EE_ADDR_MARK 0    /* 0xA5 有效标记字节 */
#define EE_ADDR_VAL  1    /* uint16 setX10: 低字节在地址 1，高字节在地址 2 */
#define EE_MARK      0xA5

/* ============================ 运行状态 ============================ */
static int      tempX10i   = 0;    /* 最近一次有效室温(0.1℃)，读取失败时保留 */
static int      humX10i    = 0;    /* 最近一次有效湿度(0.1%RH) */
static bool     haveSensor = false;/* 是否曾取得有效读数（否则界面显示 --.-） */
static uint8_t  dhtFailCnt = 0;    /* DHT 连续失败计数 */
static bool     sensorErr  = false;/* 连续 3 次失败 => 置位 BK_FLAG_SENSOR_ERR 并标 Err */

static uint16_t setX10      = SET_DEFAULT_X10; /* 已保存的目标温度 */
static uint16_t editX10     = SET_DEFAULT_X10; /* 设置界面正在编辑的值 */
static bool     inSettings  = false;           /* 是否处于设置界面 */
static uint32_t lastActMs   = 0;               /* 设置界面最近一次操作时间 */

static uint8_t  boilerHeatOn = 0;  /* 最近一次炉端状态包的 heatOn */
static uint32_t lastStatusMs = 0;  /* 最近一次收到状态包的时间（0 = 从未收到） */
static bool     linkWasUp    = false; /* 链路是否正常（用于沿触发打印丢失日志） */

static uint8_t  txSeq      = 0;    /* 遥测包序号，递增 */
static uint32_t lastTickMs = 0;    /* 上次 DHT 读取 + 遥测广播时刻 */
static uint32_t lastDrawMs = 0;    /* 上次刷屏时刻 */

/* 心跳与自愈（长时间运行后信道可能漂移 / ESP-NOW 发送可能静默失效，需自恢复） */
static uint32_t tLastChanFix    = 0; /* 上次重申信道1+心跳日志时刻（每 30 秒） */
static uint8_t  txFailCnt      = 0; /* esp_now_send 连续失败计数（重建触发器） */

/* ESP-NOW 接收缓冲：回调在系统上下文执行，只做 memcpy 到 volatile 缓冲 + 置 volatile 标志，
 * 校验(magic/devId/size/crc)与业务处理全部放在 loop() 里 */
static volatile bool    g_rxFlag = false;
static volatile uint8_t g_rxLen  = 0;
static volatile uint8_t g_rxBuf[BK_PKT_LEN];
static volatile uint8_t g_lastSendStatus = 0xFF; /* 发送回调记录的结果（0=成功，仅调试用） */

/* ====================== 按键（去抖 + 可选长按连发） ====================== */
#define BTN_NONE   0
#define BTN_CLICK  1  /* 稳定按下沿（单击） */
#define BTN_REPEAT 2  /* 长按连发触发一次 */

struct Btn {
  uint8_t  pin;
  bool     pressed;   /* 去抖后的稳定状态（true=按下） */
  bool     lastRaw;   /* 上次原始电平（true=松开高电平） */
  uint32_t tChange;   /* 原始电平最近一次翻转时刻（去抖计时） */
  uint32_t tPress;    /* 本次按下的起始时刻 */
  bool     repeating; /* 长按连发已启动 */
  uint32_t tRepeat;   /* 最近一次连发时刻 */

  void begin(uint8_t p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP); /* 另一端接 GND，按下为低电平 */
    pressed = false; lastRaw = true;
    tChange = 0; tPress = 0; repeating = false; tRepeat = 0;
  }

  /* repeat=true 时支持长按连发: 按住 500ms 后每 150ms 触发一次 BTN_REPEAT */
  uint8_t poll(bool repeat) {
    uint32_t now = millis();
    bool raw = (digitalRead(pin) == LOW);
    if (raw != lastRaw) { lastRaw = raw; tChange = now; } /* 电平翻转，重置去抖计时 */
    bool stable = ((now - tChange) >= DEBOUNCE_MS) ? raw : pressed;
    if (stable && !pressed) {           /* 稳定按下沿 => 单击 */
      pressed = true; tPress = now; repeating = false;
      return BTN_CLICK;
    }
    if (!stable && pressed) {           /* 稳定松开沿 */
      pressed = false; repeating = false;
    }
    if (repeat && pressed) {            /* 长按连发 */
      if (!repeating && (now - tPress) >= REPEAT_DELAY_MS) {
        repeating = true; tRepeat = now;
        return BTN_REPEAT;
      }
      if (repeating && (now - tRepeat) >= REPEAT_RATE_MS) {
        tRepeat = now;
        return BTN_REPEAT;
      }
    }
    return BTN_NONE;
  }
};

static Btn btnSet, btnUp, btnDown;

/* ============================ 小工具 ============================ */
/* 0.1 粒度整数转 "d.d" 字符串（兼容负值，如 -5 => "-0.5"，-123 => "-12.3"） */
static const char* x10str(int v, char* buf, size_t n) {
  int a = (v < 0) ? -v : v;
  if (v < 0 && a < 10) snprintf(buf, n, "-0.%d", a);
  else                 snprintf(buf, n, "%d.%d", v / 10, a % 10);
  return buf;
}

/* 目标温度钳位到 5.0~30.0℃ */
static int clampSet(int v) {
  return (v < SET_MIN_X10) ? SET_MIN_X10 : ((v > SET_MAX_X10) ? SET_MAX_X10 : v);
}

/* ====================== EEPROM：目标温度掉电保存 ====================== */
static void loadSetpoint() {
  EEPROM.begin(EE_SIZE);
  uint8_t  mark = EEPROM.read(EE_ADDR_MARK);
  uint16_t v    = (uint16_t)EEPROM.read(EE_ADDR_VAL) | ((uint16_t)EEPROM.read(EE_ADDR_VAL + 1) << 8);
  if (mark == EE_MARK && v >= SET_MIN_X10 && v <= SET_MAX_X10) {
    setX10 = v;
    Serial.printf("[EEPROM] 读到目标温度 %u.%u C\n", (unsigned)(setX10 / 10), (unsigned)(setX10 % 10));
  } else {
    setX10 = SET_DEFAULT_X10; /* 数据无效 => 默认 20.0℃ */
    Serial.println("[EEPROM] 数据无效，使用默认 20.0C");
  }
}

static void saveSetpoint() {
  EEPROM.write(EE_ADDR_MARK, EE_MARK);
  EEPROM.write(EE_ADDR_VAL, setX10 & 0xFF);
  EEPROM.write(EE_ADDR_VAL + 1, (setX10 >> 8) & 0xFF);
  EEPROM.commit(); /* 保存时立即提交 */
}

/* ============ ESP-NOW 收发回调（系统上下文，只做最小工作） ============ */
static void espNowRecvCb(uint8_t *mac, uint8_t *data, uint8_t len) {
  (void)mac;
  /* 只 memcpy 到 volatile 缓冲 + 置标志；长度/魔数/devId/CRC 校验与业务都在 loop() 处理 */
  if (len > BK_PKT_LEN) len = BK_PKT_LEN; /* 防越界：超长包截断（长度校验在 loop 做） */
  memcpy((void*)g_rxBuf, (const void*)data, len);
  g_rxLen = len;
  g_rxFlag = true;
}

static void espNowSendCb(uint8_t *mac, uint8_t status) {
  (void)mac;
  g_lastSendStatus = status; /* 仅记录结果供调试打印 */
}

/* ============ ESP-NOW 初始化（setup 与自愈重建共用） ============ */
static void espnowSetup() {
  WiFi.mode(WIFI_STA);
  /* 连接炉端 SoftAP：连接成功后信道跟随 AP 固定为 1，彻底避免空闲 STA 信道随机跳变 */
  WiFi.begin(AP_SSID, AP_PASS);
  wifi_set_channel(1);                /* 连接完成前先置 1，连上后自动同步为 AP 信道 */
  WiFi.setSleepMode(WIFI_NONE_SLEEP); /* 关闭 WiFi 休眠，保证 ESP-NOW 收发及时 */
  if (esp_now_init() != 0) {
    Serial.println("[ESPNOW] 初始化失败！");
  } else {
    Serial.println("[ESPNOW] 初始化成功");
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(BK_BROADCAST_MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0); /* 信道 1 广播 peer */
  esp_now_register_recv_cb(espNowRecvCb);
  esp_now_register_send_cb(espNowSendCb);
}

/* ==================== 接收包处理（loop 上下文） ==================== */
static void processRx() {
  if (!g_rxFlag) return;
  /* 先取快照再清标志，尽量降低与回调的竞争窗口 */
  BoilerPacket pkt;
  memcpy(&pkt, (const void*)g_rxBuf, BK_PKT_LEN);
  uint8_t len = g_rxLen;
  g_rxFlag = false;

  /* 逐项校验: size / magic / devId / crc */
  if (len != BK_PKT_LEN) { Serial.printf("[RX] 长度错误 %u，丢弃\n", (unsigned)len); return; }
  if (pkt.magic != BK_MAGIC) { Serial.println("[RX] magic 不符，丢弃"); return; }
  if (pkt.devId != BK_LINK_ID) { Serial.println("[RX] devId 不符，丢弃"); return; }
  if (bkCrc16((const uint8_t*)&pkt, BK_PKT_LEN - 2) != pkt.crc) { Serial.println("[RX] CRC 校验失败，丢弃"); return; }

  if (pkt.type == BK_TYPE_STATUS) {
    /* 炉端状态包: 记录 heatOn 与接收时间（供界面 45 秒超时判断） */
    bool firstOrChanged = (lastStatusMs == 0) || (pkt.heatOn != boilerHeatOn);
    boilerHeatOn = pkt.heatOn;
    lastStatusMs = millis();
    if (firstOrChanged) {
      Serial.printf("[RX] 炉端状态: %s (seq=%u)\n",
                    boilerHeatOn ? "HEATING" : "IDLE", (unsigned)pkt.seq);
    }
  }
  /* 其它类型（如炉端自身遥测的回显）直接忽略 */
}

/* ============ OLED 初始化：I2C 总线扫描诊断（开机一次 + 未就绪时每 2 秒重试） ============ */
static bool tryInitOled() {
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[OLED] I2C 扫描发现设备 0x%02X，开始初始化...\n", a);
      u8g2.setI2CAddress((uint8_t)(a << 1)); /* u8g2 用 8 位地址(左移1位) */
      u8g2.setBusClock(400000);              /* 400kHz：SSD1306 支持，降低刷屏阻塞，保证按键采样跟手 */
      u8g2.begin();
      oledReady = true;
      if (a == 0x3C || a == 0x3D) {
        Serial.println("[OLED] 已按扫描地址初始化。若屏幕仍全黑：把 OLED_SH1106 改为 1 重试(1.3寸屏常见)");
      } else {
        Serial.printf("[OLED] 注意：地址 0x%02X 不是常见 OLED 地址(0x3C/0x3D)，请确认模块型号\n", a);
      }
      return true;
    }
  }
  Serial.println("[OLED] I2C 扫描无设备! 请检查接线: VCC→3V3, GND→GND, SDA→D2(GPIO4), SCL→D1(GPIO5) 是否接错/接反/松动");
  return false;
}

/* ==================== DHT 读取（每 5 秒一次） ==================== */
static void readDht() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    dhtFailCnt++;
    if (dhtFailCnt >= 3) {
      if (!sensorErr) {
        sensorErr = true; /* 连续 3 次失败: 置 SENSOR_ERR 标志，界面标 Err，保留最后有效值 */
        Serial.println("[DHT] 连续 3 次读取失败，置传感器错误标志（保留最后一次有效值）");
      }
    } else {
      Serial.printf("[DHT] 读取失败（连续第 %u 次）\n", (unsigned)dhtFailCnt);
    }
    return;
  }
  tempX10i = (int)roundf(t * 10.0f); /* 有效读数: 更新并保留 */
  humX10i  = (int)roundf(h * 10.0f);
  haveSensor = true;
  dhtFailCnt = 0;
  if (sensorErr) { sensorErr = false; Serial.println("[DHT] 传感器恢复正常"); }
  char b1[8], b2[8];
  Serial.printf("[DHT] 室温 %sC  湿度 %s%%RH\n",
                x10str(tempX10i, b1, sizeof(b1)), x10str(humX10i, b2, sizeof(b2)));
}

/* ==================== 遥测广播（每 5 秒一次） ==================== */
static void sendTelemetry() {
  BoilerPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic   = BK_MAGIC;
  pkt.devId   = BK_LINK_ID;
  pkt.type    = BK_TYPE_TELEM;
  pkt.seq     = txSeq++;                    /* 序号递增 */
  pkt.flags   = (sensorErr || !haveSensor) ? BK_FLAG_SENSOR_ERR : 0x00; /* 未取得首个有效读数前也置错误标志，避免炉端把 0℃ 当真实低温开阀 */
  pkt.tempX10 = (uint16_t)tempX10i;         /* 失败时保留最后有效值 */
  pkt.humX10  = (uint16_t)humX10i;
  pkt.setX10  = setX10;                     /* 当前目标温度 */
  pkt.heatOn  = 0;                          /* 遥测包固定填 0 */
  pkt.crc     = bkCrc16((const uint8_t*)&pkt, BK_PKT_LEN - 2);

  int r = esp_now_send(BK_BROADCAST_MAC, (uint8_t*)&pkt, BK_PKT_LEN);
  if (r != 0) txFailCnt++; else txFailCnt = 0; /* 连续失败用于触发 ESP-NOW 重建 */
  char b1[8], b2[8], b3[8];
  Serial.printf("[TX] 遥测 seq=%u T=%s H=%s Set=%s err=%u ret=%d lastCb=%u\n",
                (unsigned)pkt.seq,
                haveSensor ? x10str(tempX10i, b1, sizeof(b1)) : "--.-",
                haveSensor ? x10str(humX10i, b2, sizeof(b2)) : "--.-",
                x10str((int)setX10, b3, sizeof(b3)),
                (unsigned)(sensorErr ? 1 : 0), r, (unsigned)g_lastSendStatus);
}

/* ============ 设置界面（SET 进入/保存退出，UP/DOWN 调节） ============ */
static void exitSettings(bool save) {
  if (save) {
    setX10 = (uint16_t)clampSet((int)editX10);
    saveSetpoint(); /* 写 EEPROM 并 commit */
    Serial.printf("[UI] 保存目标温度 %u.%u C\n", (unsigned)(setX10 / 10), (unsigned)(setX10 % 10));
  }
  inSettings = false;
}

static void handleUi(uint32_t now, uint8_t aSet, uint8_t aUp, uint8_t aDown) {
  /* 注意：now 必须用 loop 开头的同一时间戳，不能在这里重新取 millis()——
   * 否则 lastActMs 会晚于本轮超时检查用的 now，(now-lastActMs) 无符号下溢成巨大值，
   * 导致按 UP/DOWN 的瞬间被误判为“15 秒无操作”而立即退出设置（真 bug，已实测复现） */

  if (!inSettings) {
    if (aSet == BTN_CLICK) {           /* 主界面按 SET: 进入设置 */
      inSettings = true;
      editX10 = setX10;
      lastActMs = now;
      Serial.println("[UI] 进入设置界面");
    }
    return;                            /* 主界面下 UP/DOWN 无功能 */
  }

  if (aSet == BTN_CLICK) {             /* 设置界面按 SET: 保存并退出 */
    exitSettings(true);
    return;
  }
  if (aUp || aDown) {                  /* UP/DOWN（单击或长按连发）: ±0.5℃ */
    int step = aUp ? SET_STEP_X10 : -SET_STEP_X10;
    editX10 = (uint16_t)clampSet((int)editX10 + step);
    lastActMs = now;                   /* 任意操作刷新 15 秒自动退出计时 */
  }
}

/* ============================ OLED 界面 ============================ */
static void drawMainScreen() {
  char buf[20], b1[10];

  /* 顶行: 目标温度（左） + 湿度（右） */
  u8g2.setFont(u8g2_font_6x12_tr);
  snprintf(buf, sizeof(buf), "Target %s", x10str((int)setX10, b1, sizeof(b1)));
  u8g2.drawStr(0, 11, buf);
  if (haveSensor) snprintf(buf, sizeof(buf), "RH %s%%", x10str(humX10i, b1, sizeof(b1)));
  else            snprintf(buf, sizeof(buf), "RH --%%");
  u8g2.drawStr(128 - u8g2.getStrWidth(buf), 11, buf);

  /* 中部: 大字号室温 + 小字号 'C' */
  u8g2.setFont(u8g2_font_logisoso32_tn);
  if (haveSensor) x10str(tempX10i, buf, sizeof(buf));
  else            snprintf(buf, sizeof(buf), "--.-");
  u8g2.drawStr(0, 50, buf);
  int tw = u8g2.getStrWidth(buf); /* 宽度按当前大号字体计算 */
  u8g2.setFont(u8g2_font_9x15_tr);
  u8g2.drawStr(tw + 4, 50, "C");

  /* 底行: 炉端状态（左） + 传感器错误标志 Err（右） */
  const char* st;
  if (lastStatusMs == 0 || (millis() - lastStatusMs) > LINK_TIMEOUT_MS) st = "NO LINK";
  else st = boilerHeatOn ? "HEATING" : "IDLE";
  u8g2.drawStr(0, 63, st);
  if (sensorErr) u8g2.drawStr(128 - u8g2.getStrWidth("Err"), 63, "Err");
}

static void drawSettingsScreen() {
  char buf[20];

  /* 顶行: 标题 + 范围提示 */
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 11, "Set Target");
  const char* rng = "5.0-30.0";
  u8g2.drawStr(128 - u8g2.getStrWidth(rng), 11, rng);

  /* 中部: 大字号当前编辑值 + 小字号 'C' */
  u8g2.setFont(u8g2_font_logisoso32_tn);
  x10str((int)editX10, buf, sizeof(buf));
  u8g2.drawStr(0, 50, buf);
  int tw = u8g2.getStrWidth(buf);
  u8g2.setFont(u8g2_font_9x15_tr);
  u8g2.drawStr(tw + 4, 50, "C");

  /* 底行: 操作提示 */
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 63, "SET save  +0.5  15s auto");
}

static void drawScreen() {
  u8g2.clearBuffer();
  if (inSettings) drawSettingsScreen();
  else            drawMainScreen();
  u8g2.sendBuffer();
}

/* ============================ 初始化 ============================ */
void setup() {
  Serial.begin(115200);
  delay(200); /* 等串口稳定 */
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" 壁挂炉温控系统 - 客厅端 (ESP8266 + ESP-NOW)");
  Serial.println("==============================================");

  loadSetpoint(); /* 目标温度从 EEPROM 恢复 */

  btnSet.begin(PIN_SET);   /* 按键: INPUT_PULLUP，另一端接 GND */
  btnUp.begin(PIN_UP);
  btnDown.begin(PIN_DOWN);

  dht.begin();   /* 温湿度传感器 */
  Wire.begin();  /* I2C 总线: SDA=GPIO4/D2, SCL=GPIO5/D1（扫描与 OLED 共用） */
  Serial.println("[OLED] 开始 I2C 总线扫描...");
  oledReady = tryInitOled();
  oledNextScanMs = millis() + 2000;
  if (oledReady) drawScreen();  /* 先画一帧主界面 */

  /* ESP-NOW 初始化（与自愈重建共用同一函数） */
  espnowSetup();

  Serial.printf("[WiFi] 本机 MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("[提示] SET 进入/保存设置, UP/DOWN 调节目标温度(0.5C); 每 5s 广播遥测; 每 30s 心跳日志");

  /* 开机约 2 秒后先执行一轮 DHT 读取 + 遥测广播（DHT11 上电需约 1s 稳定） */
  lastTickMs = millis() - (TICK_MS - 2000UL);
  lastDrawMs = 0;
  tLastChanFix = millis();
}

/* ============================ 主循环 ============================ */
void loop() {
  uint32_t now = millis();

  processRx(); /* 处理接收到的包（校验 + 业务） */

  /* 按键扫描: SET 仅单击；UP/DOWN 支持长按连发（连发只在设置界面生效） */
  uint8_t aSet  = btnSet.poll(false);
  uint8_t aUp   = btnUp.poll(true);
  uint8_t aDown = btnDown.poll(true);
  handleUi(now, aSet, aUp, aDown);

  /* 按键事件串口日志（排查按键逻辑/硬件接触问题用） */
  if (aSet == BTN_CLICK)  Serial.println("[BTN] SET 单击");
  if (aUp == BTN_CLICK)   Serial.println("[BTN] UP 单击");
  if (aUp == BTN_REPEAT)  Serial.println("[BTN] UP 连发");
  if (aDown == BTN_CLICK) Serial.println("[BTN] DOWN 单击");
  if (aDown == BTN_REPEAT) Serial.println("[BTN] DOWN 连发");

  /* 周期任务: 每 5 秒读 DHT 并广播遥测包 */
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    readDht();
    sendTelemetry();
  }

  /* 设置界面 15 秒无操作: 自动保存退出（有符号比较，避免时间戳回绕/未来值下溢误判） */
  if (inSettings && (int32_t)(now - lastActMs) >= (int32_t)SETTING_TIMEOUT_MS) {
    Serial.println("[UI] 设置超时，自动保存退出");
    exitSettings(true);
  }

  /* 链路丢失检测（沿触发打印，避免刷屏） */
  bool linkUp = (lastStatusMs != 0) && (now - lastStatusMs) <= LINK_TIMEOUT_MS;
  if (!linkUp && linkWasUp) {
    Serial.println("[LINK] 超过 45 秒未收到炉端状态包");
  }
  linkWasUp = linkUp;

  /* 每 30 秒: 重申信道1（防信道漂移）+ 心跳日志 */
  if ((int32_t)(now - tLastChanFix) >= 30000) {
    tLastChanFix = now;
    wifi_set_channel(1);
    Serial.printf("[心跳] up=%us ch=%d wl=%d heap=%u txSeq=%u 炉端:%s oled:%s\n",
                  (unsigned)(now / 1000), (int)wifi_get_channel(), (int)WiFi.status(),
                  (unsigned)ESP.getFreeHeap(), (unsigned)txSeq, linkUp ? "已连" : "无",
                  oledReady ? "OK" : "无");
  }

  /* 连续 5 次发送失败 → 重建 ESP-NOW（防发送侧静默失效） */
  if (txFailCnt >= 5) {
    txFailCnt = 0;
    Serial.println("[自愈] 遥测连续发送失败，重建 ESP-NOW + 重申信道1...");
    esp_now_deinit();
    espnowSetup();
  }

  /* OLED 未就绪时每 2 秒重试 I2C 扫描（接线修复后无需重新上电） */
  if (!oledReady && (int32_t)(now - oledNextScanMs) >= 0) {
    oledNextScanMs = now + 2000;
    if (tryInitOled()) drawScreen();
  }

  /* 定期刷新 OLED */
  if (oledReady && now - lastDrawMs >= DRAW_MS) {
    lastDrawMs = now;
    drawScreen();
  }
}
