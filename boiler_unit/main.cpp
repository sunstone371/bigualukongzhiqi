/*
 * ============================================================================
 *  壁挂炉温控系统 —— 壁挂炉端（继电器执行器）固件
 *  目标板：ESP8266 (esp8266:esp8266:generic)，Arduino 框架，不使用任何第三方库
 *
 *  功能概述：
 *    1. ESP-NOW（信道1）接收客厅端 BK_TYPE_TELEM 遥测包（温度/湿度/设定值），
 *       按 ±0.5℃ 回差控制 GPIO5 继电器输出（吸合 = 呼叫壁挂炉供暖）。
 *    2. 遥测带传感器错误标志 → 立即断开；超过 90 秒无合法遥测 → 安全停机并告警。
 *    3. 防频繁启停：关闭→再次开启 ≥180 秒；每次最短运行 60 秒
 *       （传感器错误 / 链路超时例外，可立即关）。
 *    4. 每收到一包合法遥测，立即向发送者单播回复 BK_TYPE_STATUS 状态包
 *       （首次收到时动态 esp_now_add_peer 注册对端 MAC，信道1，COMBO 角色）。
 *    5. GPIO2 板载 LED 指示链路：正常 500ms 慢闪 / 超时 100ms 快闪。
 *
 *  接线：
 *    GPIO5 (D1) → 继电器模块 IN（默认低电平吸合，常见光耦继电器模块）
 *    GPIO2      → 板载指示 LED（低电平点亮）
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>

/* ESP-NOW 直接使用乐鑫 SDK 头文件（Arduino 核心自带，非第三方库） */
extern "C" {
#include <espnow.h>
#include <user_interface.h>
}

/* ================= 共享链路协议（与客厅端逐字节一致，禁止改动任何字段/顺序/常量） ================= */

#define BK_MAGIC   0xB0C0A101UL /* 协议魔数 */
#define BK_LINK_ID 1            /* 本链路编号（收发包均须校验/填写） */
#define BK_TYPE_TELEM  1        /* 包类型：客厅端 → 壁挂炉端 遥测包 */
#define BK_TYPE_STATUS 2        /* 包类型：壁挂炉端 → 客厅端 状态回包 */
#define BK_FLAG_SENSOR_ERR 0x01 /* 遥测标志位：传感器错误 */

/* 链路数据包（packed，共 17 字节；crc 覆盖前 15 字节） */
struct __attribute__((packed)) BoilerPacket {
  uint32_t magic;    /* 帧头魔数 BK_MAGIC */
  uint8_t  devId;    /* 链路编号 BK_LINK_ID */
  uint8_t  type;     /* 包类型 1=遥测 2=状态 */
  uint8_t  seq;      /* 序号（发送方自增） */
  uint8_t  flags;    /* 标志位（bit0=传感器错误） */
  uint16_t tempX10;  /* 温度 x10 */
  uint16_t humX10;   /* 湿度 x10 */
  uint16_t setX10;   /* 设定温度 x10 */
  uint8_t  heatOn;   /* 供暖输出状态 0/1（状态包有效） */
  uint16_t crc;      /* CRC16-CCITT，覆盖本字段之前的全部字节 */
};
static_assert(sizeof(BoilerPacket) == 17, "BoilerPacket 布局与客厅端不一致！");

/* CRC16-CCITT（多项式 0x1021，初值 0xFFFF）—— 与客厅端完全一致 */
uint16_t bkCrc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

/* ================= 本机配置 ================= */

#define RELAY_PIN        14    /* 继电器输出 GPIO14 (D5)。
                                 * 选用普通 IO（非 strap 引脚）：GPIO15 是 boot strap 且板载下拉，
                                 * 上电瞬间会误吸合；GPIO16 是 RTC 引脚无内部上拉/无中断。
                                 * 当前为推挽输出驱动（曾试开漏，实测无法根治 5V 模块微导通，已回滚）。
                                 * 若改接 D6/D7，把本行改为 12 / 13 即可 */
#define RELAY_ACTIVE_LOW 1    /* 1：低电平吸合（5V 光耦模块常见）；高电平触发模块改为 0 */
#define LED_PIN          2    /* 板载指示 LED GPIO2（低电平点亮）。
                                 注：esp8266:esp8266:generic 变体的 LED_BUILTIN 映射到 GPIO1(TX)，
                                 故此处显式指定 GPIO2，不直接使用 LED_BUILTIN */

/* 射频信道锁定方案：本机开 SoftAP（信道1），客厅端 STA 连接本 AP
 * 原因：ESP8266 空闲 STA（不连任何 AP）的 WiFi 信道会被 SDK 随机跳变(1/5/12/13…)，
 * 导致 ESP-NOW 两端信道不同步而掉线。AP 模式下的信道固定不跳。 */
#define AP_SSID  "BoilerLink"
#define AP_PASS  "boilerlink2024" /* 不少于 8 位 */

#define HYST_X10         5        /* 回差 ±0.5℃（x10 表示） */
#define LINK_TIMEOUT_MS  90000UL  /* 链路超时：90 秒未收到合法遥测即安全停机 */
#define MIN_OFF_MS       180000UL /* 防频繁启停：断开 → 再次吸合 最小间隔 180 秒 */
#define MIN_ON_MS        60000UL  /* 防频繁启停：每次吸合最短运行 60 秒 */
#define LED_BLINK_OK_MS  500      /* 链路正常：500ms 慢闪 */
#define LED_BLINK_ERR_MS 100      /* 链路超时：100ms 快闪 */

/* ================= 运行状态（全部在 loop() 上下文维护） ================= */

static bool     relayOn = false;        /* 继电器当前是否吸合 */
static uint32_t tLastOn = 0;            /* 上次吸合时刻（millis） */
static uint32_t tLastOff = 0;           /* 上次断开时刻（millis） */
static bool     everTurnedOff = false;  /* 上电以来是否发生过"断开"动作（未发生则首次允许立即吸合） */
static uint32_t tLastTelem = 0;         /* 最近一次收到合法遥测的时刻（millis） */
static bool     linkTimedOut = false;   /* 链路超时标志（兼作 LED 模式选择） */
static uint8_t  txSeq = 0;              /* 状态包发送序号（自增） */

/* LED 非阻塞闪烁 */
static uint32_t tLedToggle = 0;
static bool     ledLit = false;

/* 心跳与自愈（长时间运行后 ESP-NOW 可能失效/信道漂移，需自恢复） */
static uint32_t tLastHeartbeat = 0; /* 上次心跳日志时刻（每 10 秒一条，便于串口识别端口与观察状态） */
static uint32_t tLastRecover   = 0; /* 上次超时自愈动作时刻（每 30 秒重建 ESP-NOW） */

/* ---- 接收缓冲：ESP-NOW 回调运行在系统上下文，回调内只做 memcpy + 置标志，
 *      一切校验（magic/devId/size/crc）与业务逻辑都在 loop() 中处理 ---- */
static volatile bool     g_rxFlag = false;              /* 收到新包标志 */
static volatile uint8_t  g_rxMac[6];                    /* 发送者 MAC */
static volatile uint8_t  g_rxBuf[sizeof(BoilerPacket)]; /* 原始包数据 */
static volatile uint8_t  g_rxLen = 0;                   /* 收到的字节数 */

/* ================= ESP-NOW 接收回调（系统上下文，越快越好） ================= */
/* 约定：仅拷贝数据 + 置标志；禁止在此做串口打印 / 校验 / 控制动作 */
static void onEspNowRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (mac != NULL) {
    memcpy((void*)g_rxMac, mac, 6);
  }
  if (data != NULL && len <= sizeof(BoilerPacket)) {
    memcpy((void*)g_rxBuf, data, len);   /* 只拷贝，不解析 */
    g_rxLen = len;
  } else {
    g_rxLen = 0;                          /* 长度异常：置 0，交给 loop() 按长度错误丢弃 */
  }
  g_rxFlag = true;
}

/* ================= 继电器底层控制 ================= */

/* 写继电器物理电平（不记时间、不打日志，供初始化与 setRelay 使用）
 * 注：曾尝试开漏（OUTPUT_OPEN_DRAIN）驱动 5V 低触发模块实现“断开=高阻”，
 * 但高阻时模块 5V 上拉与 ESP 引脚 ESD 锋位二极管形成微电流回路，光耦仍微导通，
 * 纯软件无法彻底断开（已实测）。现回滚为普通推挽输出；模块侧 5V 电平适配
 * 由硬件方案解决（模块 HIGH 跳线 / 串联 1N4148 二极管 / NPN 三极管反相） */
static void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);   /* 低电平吸合 */
  } else {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);   /* 高电平吸合 */
  }
}

/* 带状态切换记录的继电器操作（仅在状态真正变化时调用） */
static void setRelay(bool on, const char* reason) {
  relayWrite(on);
  relayOn = on;
  uint32_t now = millis();
  if (on) {
    tLastOn = now;
  } else {
    tLastOff = now;
    everTurnedOff = true;
  }
  Serial.printf("[控制] 继电器%s | %s\n", on ? "吸合(呼叫供暖)" : "断开(停止供暖)", reason);
}

/* ================= 对端管理 ================= */

/* 首次收到某发送者的包时动态注册为 ESP-NOW 对端（信道1，COMBO 角色）。
 * 本 SDK 的 espnow.h 未定义 ESP_NOW_EXISTED 宏，改用 esp_now_is_peer_exist()
 * 预判：已存在则直接忽略（等效于"已存在返回 ESP_NOW_EXISTED 则忽略"）。 */
static void ensurePeer(const uint8_t mac[6]) {
  u8 m[6];
  memcpy(m, mac, 6);
  if (esp_now_is_peer_exist(m)) {
    return;  /* 已注册，忽略 */
  }
  int r = esp_now_add_peer(m, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  if (r == 0) {
    Serial.printf("[ESP-NOW] 已动态注册对端 %02X:%02X:%02X:%02X:%02X:%02X (信道1, 角色 COMBO)\n",
                  m[0], m[1], m[2], m[3], m[4], m[5]);
  } else {
    Serial.printf("[ESP-NOW] 注册对端失败 ret=%d（不影响本次控制，下次包会重试）\n", r);
  }
}

/* ================= 状态包回送 ================= */

/* 每收到一包遥测后立即向发送者单播回复状态包：
 * heatOn = 当前继电器实际状态；seq 自增；温/湿/设定回显最新遥测值便于客厅端核对 */
static void sendStatus(const uint8_t mac[6], const BoilerPacket* telem) {
  BoilerPacket pkt;
  pkt.magic   = BK_MAGIC;
  pkt.devId   = BK_LINK_ID;
  pkt.type    = BK_TYPE_STATUS;
  pkt.seq     = txSeq++;          /* 序号自增 */
  pkt.flags   = 0;                /* 壁挂炉端无本地传感器，无错误标志 */
  pkt.tempX10 = telem->tempX10;   /* 回显最新遥测值 */
  pkt.humX10  = telem->humX10;
  pkt.setX10  = telem->setX10;
  pkt.heatOn  = relayOn ? 1 : 0;  /* 当前继电器状态 */
  pkt.crc     = bkCrc16((const uint8_t*)&pkt, sizeof(pkt) - 2); /* CRC 覆盖 crc 字段之前 */

  int r = esp_now_send((u8*)mac, (u8*)&pkt, (int)sizeof(pkt));
  if (r != 0) {
    Serial.printf("[ESP-NOW] 状态包发送失败 ret=%d\n", r);
  }
}

/* ================= 控制决策（仅在收到合法遥测后调用） ================= */

static void applyControl(const BoilerPacket* pkt) {
  uint32_t now = millis();

  /* 1) 传感器错误 → 立即断开（不受最短运行时间限制） */
  if (pkt->flags & BK_FLAG_SENSOR_ERR) {
    if (relayOn) {
      setRelay(false, "传感器错误 → 立即断开(安全停机)");
    } else {
      Serial.println("[控制] 传感器错误(保持断开)");
    }
    return;
  }

  /* 2) 回差控制：≤设定-0.5℃ 开；≥设定+0.5℃ 关；中间保持
   *    温度按 int16 语义解释（与客厅端 x10str/遥测发送一致，兼容 DHT22 负温：
   *    如 -3.5℃ 以 uint16 传输为 65501，若按无符号解释会误判为超高温而拒开阀） */
  int32_t diff = (int32_t)(int16_t)pkt->tempX10 - (int32_t)pkt->setX10;

  if (diff <= -(int32_t)HYST_X10) {
    /* 温度低于设定 0.5℃ → 需要供暖 */
    if (!relayOn) {
      uint32_t offElapsed = everTurnedOff ? (uint32_t)(now - tLastOff) : (uint32_t)MIN_OFF_MS;
      if (offElapsed >= MIN_OFF_MS) {
        setRelay(true, "温度低于设定0.5℃ → 呼叫供暖");
      } else {
        Serial.printf("[控制] 需供暖但断开仅 %u 秒(防频繁启停<180秒) → 暂不吸合\n",
                      (unsigned)(offElapsed / 1000));
      }
    }
    /* 已吸合则继续供暖，无需动作 */
  } else if (diff >= (int32_t)HYST_X10) {
    /* 温度高于设定 0.5℃ → 停止供暖 */
    if (relayOn) {
      uint32_t onElapsed = (uint32_t)(now - tLastOn);
      if (onElapsed >= MIN_ON_MS) {
        setRelay(false, "温度高于设定0.5℃ → 停止呼叫");
      } else {
        Serial.printf("[控制] 需停止但运行仅 %u 秒(最短运行<60秒) → 继续运行\n",
                      (unsigned)(onElapsed / 1000));
      }
    }
    /* 已断开则保持 */
  } else {
    /* 温度在回差区间内 → 保持原状态 */
    Serial.printf("[控制] 温度在回差区间内 → 保持%s\n", relayOn ? "吸合(供暖中)" : "断开");
  }
}

/* ================= 链路超时安全停机 ================= */

static void checkLinkTimeout(uint32_t now) {
  if (!linkTimedOut && (uint32_t)(now - tLastTelem) > LINK_TIMEOUT_MS) {
    linkTimedOut = true;
    Serial.println("[告警] 超过90秒未收到合法遥测包 → 链路超时！");
    if (relayOn) {
      setRelay(false, "链路超时 → 立即断开(安全停机)");
    }
  }
}

/* ================= 板载 LED 链路指示（非阻塞闪烁） ================= */

static void updateLed(uint32_t now) {
  uint32_t period = linkTimedOut ? LED_BLINK_ERR_MS : LED_BLINK_OK_MS;
  if ((uint32_t)(now - tLedToggle) >= period) {
    tLedToggle = now;
    ledLit = !ledLit;
    digitalWrite(LED_PIN, ledLit ? LOW : HIGH); /* 低电平点亮 */
  }
}

/* ================= 接收包处理（loop 上下文：校验 + 业务） ================= */

static void handleRx() {
  g_rxFlag = false;

  /* 从 volatile 缓冲拷贝出本地副本，再做校验与解析 */
  uint8_t      mac[6];
  BoilerPacket pkt;
  uint8_t      len = g_rxLen;
  memcpy(mac, (const void*)g_rxMac, 6);
  memcpy(&pkt, (const void*)g_rxBuf, sizeof(pkt));

  /* ---- 合法性校验：长度 / 魔数 / 链路号 / CRC ---- */
  if (len != sizeof(BoilerPacket)) {
    Serial.printf("[丢弃] 包长度错误 len=%u (应为17)\n", (unsigned)len);
    return;
  }
  if (pkt.magic != BK_MAGIC) {
    Serial.printf("[丢弃] 魔数错误 0x%08X\n", (unsigned)pkt.magic);
    return;
  }
  if (pkt.devId != BK_LINK_ID) {
    Serial.printf("[丢弃] devId=%u 非本链路\n", (unsigned)pkt.devId);
    return;
  }
  uint16_t crcCalc = bkCrc16((const uint8_t*)&pkt, sizeof(pkt) - 2);
  if (crcCalc != pkt.crc) {
    Serial.printf("[丢弃] CRC 错误 收=0x%04X 算=0x%04X\n", pkt.crc, crcCalc);
    return;
  }
  if (pkt.type != BK_TYPE_TELEM) {
    Serial.printf("[忽略] type=%u 非遥测包\n", (unsigned)pkt.type);
    return;
  }

  /* ---- 合法遥测：刷新链路心跳 ---- */
  uint32_t now = millis();
  bool recovered = linkTimedOut;
  tLastTelem = now;
  linkTimedOut = false;

  /* ---- 遥测摘要日志 ---- */
  Serial.printf("[遥测] seq=%u | 温 %.1fC | 湿 %.1f%% | 设定 %.1fC | 标志0x%02X | 来自 %02X:%02X:%02X:%02X:%02X:%02X\n",
                (unsigned)pkt.seq, (double)((int16_t)pkt.tempX10) / 10.0, pkt.humX10 / 10.0, pkt.setX10 / 10.0,
                (unsigned)pkt.flags, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (recovered) {
    Serial.println("[恢复] 重新收到合法遥测，链路恢复");
  }

  /* ---- 动态注册/确认对端（首次收到时 add_peer，已存在则忽略） ---- */
  ensurePeer(mac);

  /* ---- 先做控制决策，再回状态包，保证 heatOn 反映决策后的实际状态 ---- */
  applyControl(&pkt);
  sendStatus(mac, &pkt);
}

/* ================= 初始化 ================= */

void setup() {
  /* 1) 最先执行：继电器置于断开态（安全第一，先于串口与无线初始化） */
  relayWrite(false);          /* 预置输出锁存为"断开"电平 */
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);          /* 输出使能后再次确保断开 */

  /* 板载 LED 初始熄灭（低电平点亮 → 输出 HIGH） */
  digitalWrite(LED_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  /* 2) 串口初始化与启动信息 */
  Serial.begin(115200);
  delay(200); /* 等待串口稳定 */
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" 壁挂炉温控系统 - 壁挂炉端(继电器执行器) v1.0");
  Serial.println("==============================================");
  Serial.printf("[启动] 本机 MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("[启动] 继电器: GPIO%d, %s\n",
                RELAY_PIN, RELAY_ACTIVE_LOW ? "低电平吸合" : "高电平吸合");
  Serial.println("[启动] 继电器初始状态: 断开");

  /* 3) WiFi + ESP-NOW 初始化：开 SoftAP 锁定信道1（信道不漂移） */
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, 1); /* 信道 1：AP 模式信道固定不漂移 */
  wifi_set_channel(1);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  Serial.printf("[启动] SoftAP \"%s\" 已启动（客厅端连上后两端信道锁定为 1）\n", AP_SSID);

  int r = esp_now_init();
  if (r != 0) {
    Serial.printf("[错误] esp_now_init 失败 ret=%d，3秒后重试...\n", r);
    delay(3000);
    r = esp_now_init();
    if (r != 0) {
      Serial.printf("[错误] esp_now_init 再次失败 ret=%d（继续运行，仅无法收发）\n", r);
    }
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.println("[启动] ESP-NOW 初始化完成 (信道1, 角色 COMBO)");
  Serial.println("[启动] 等待客厅端遥测包(90秒内未收到将安全停机告警)...");

  /* 4) 时间基准：上电起给 90 秒链路宽限期 */
  uint32_t now = millis();
  tLastTelem = now;
  tLedToggle = now;
  tLastHeartbeat = now;
  tLastRecover = now;
  Serial.println("[提示] 每 10 秒输出一条心跳日志；链路超时后每 30 秒自动重建 ESP-NOW");
}

/* ================= 主循环 ================= */

void loop() {
  uint32_t now = millis();

  /* 1) 处理接收到的 ESP-NOW 包（校验 + 控制决策 + 状态回包） */
  if (g_rxFlag) {
    handleRx();
    now = millis();
  }

  /* 2) 链路超时检测：超 90 秒无合法遥测 → 安全停机并告警 */
  checkLinkTimeout(now);

  /* 3) 板载 LED 链路状态指示（正常 500ms 慢闪 / 超时 100ms 快闪） */
  updateLed(now);

  /* 4) 心跳日志: 每 10 秒一条（uptime/信道/堆/距上次遥测/继电器），串口连上即有输出 */
  if ((uint32_t)(now - tLastHeartbeat) >= 10000UL) {
    tLastHeartbeat = now;
    Serial.printf("[心跳] up=%us ch=%d heap=%u 上次遥测=%us前 继电器=%s\n",
                  (unsigned)(now / 1000), (int)wifi_get_channel(), (unsigned)ESP.getFreeHeap(),
                  (unsigned)((now - tLastTelem) / 1000), relayOn ? "开" : "关");
  }

  /* 5) 链路超时自愈: 每 30 秒重建 ESP-NOW + 重申信道1；超 5 分钟重启整机彻底自愈
 *    （重启后继电器默认断开，收到客厅端遥测后自动恢复控制，安全无虞） */
  if (linkTimedOut) {
    if ((uint32_t)(now - tLastRecover) >= 30000UL) {
      tLastRecover = now;
      Serial.println("[自愈] 重建 ESP-NOW + 重申信道1...");
      esp_now_deinit();
      wifi_set_channel(1);
      if (esp_now_init() == 0) {
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
        esp_now_register_recv_cb(onEspNowRecv);
        Serial.println("[自愈] ESP-NOW 重建完成，等待遥测");
      } else {
        Serial.println("[自愈] ESP-NOW 重建失败，30秒后重试");
      }
    }
    if ((uint32_t)(now - tLastTelem) >= 180000UL) {
      Serial.println("[自愈] 链路超时超过 3 分钟，重启整机...");
      Serial.flush();
      delay(100);
      ESP.restart();
    }
  }
}
