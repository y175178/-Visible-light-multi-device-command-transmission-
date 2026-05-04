// ==================== VLC 接收端完整代码 ====================
// 编码方案：定长3bit短码（信源编码）+ CRC-8（信道编码）+ 曼彻斯特解码（物理层）
// 用法：
//   1) 修改 DEVICE_ROLE: 1=灯 2=舵机 3=风扇
//   2) 修改 txHost 为发射端实际IP
//   3) 每块接收端烧录一份，分别对应三个设备

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ==================== 硬件配置 ====================
const int signalPin     = A0;    // 光电信号输入（LM358放大后）
const int outPin        = D1;    // 继电器/驱动输出

// ==================== 通信参数 ====================
const int BIT_PERIOD_US = 1000;  // 1kbps，每bit周期1000us
int adcThreshold        = 13;    // ADC判决阈值（启动时手动校准）

// ==================== 设备角色 ====================
// 1=LIGHT  2=SERVO  3=FAN
const int DEVICE_ROLE = 2;

// ==================== WiFi配置 ====================
const char* ssid     = "打扫干净屋子再请客";
const char* password = "1751787761";
const char* txHost   = "192.168.54.52";   // 发射端IP，按实际修改
const int   txPort   = 5000;

// ==================== 帧类型标识 ====================
#define FRAME_TYPE_SHORT  0xBB   // 定长短码帧
#define FRAME_TYPE_ASCII  0xFF   // 原ASCII帧（兼容保留）
#define FRAME_END         0xFF   // 帧尾

// ==================== 设备状态 ====================
bool   deviceOn    = false;
String lastCommand = "";
String receivedData = "等待接收数据...";
unsigned long lastAckMs = 0;

// ==================== WiFi上传 ====================
bool wifiUploadPending = false;
bool wifiUploading     = false;
unsigned long lastUploadTime  = 0;
const unsigned long UPLOAD_COOLDOWN = 100;

// ==================== 调试开关 ====================
bool debugSignal    = false;
bool continuousDebug = false;

// ==================== Web服务器 ====================
ESP8266WebServer server(80);

// ============================================================
//  CRC-8 计算（多项式 0x07，与发射端完全一致）
// ============================================================
uint8_t crc8(uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// ============================================================
//  定长短码解码表
//  发射端码表：LIGHT_ON=1, LIGHT_OFF=2, SERVO_ON=3,
//              SERVO_OFF=4, FAN_ON=5,   FAN_OFF=6
// ============================================================
const char* shortDecode(uint8_t code) {
  switch (code) {
    case 1: return "LIGHT_ON";
    case 2: return "LIGHT_OFF";
    case 3: return "SERVO_ON";
    case 4: return "SERVO_OFF";
    case 5: return "FAN_ON";
    case 6: return "FAN_OFF";
    default: return "";
  }
}

// ============================================================
//  设备角色
// ============================================================
String roleName() {
  if (DEVICE_ROLE == 1) return "LIGHT";
  if (DEVICE_ROLE == 2) return "SERVO";
  return "FAN";
}

bool commandMatchRole(const String& cmd) {
  String role = roleName();
  return cmd == role + "_ON" || cmd == role + "_OFF";
}

// ============================================================
//  执行指令
// ============================================================
void applyCommand(const String& cmd) {
  if (!commandMatchRole(cmd)) return;

  deviceOn = cmd.endsWith("_ON");
  digitalWrite(outPin, deviceOn ? HIGH : LOW);
  lastCommand  = cmd;
  receivedData = cmd;
  lastAckMs    = millis();

  Serial.print("[执行] ");
  Serial.println(cmd);

  wifiUploadPending = true;
}

// ============================================================
//  曼彻斯特解码状态机
// ============================================================
enum RxState { WAIT_PREAMBLE, READ_LEN, READ_PAYLOAD, READ_END };
RxState rxState   = WAIT_PREAMBLE;

// 移位寄存器（检测前导码）
uint32_t shiftReg = 0;
const uint32_t PREAMBLE_MASK  = 0x01FFFFFF;  // 25bit掩码
const uint32_t PREAMBLE_VALUE = 0x01FFFFFF;  // 全1前导码

// 字节级接收缓冲
uint8_t rxByte     = 0;
int     rxBitCount = 0;

// ASCII帧兼容
int    expectedLen = 0;
String rxBuffer    = "";

// 定长短码帧状态
bool    shortMode    = false;
uint8_t shortCode    = 0;
uint8_t shortCrc     = 0;
bool    shortWaitCrc = false;  // true时下一字节是CRC

// ============================================================
//  复位接收状态机
// ============================================================
void resetRx() {
  rxState      = WAIT_PREAMBLE;
  shiftReg     = 0;
  rxByte       = 0;
  rxBitCount   = 0;
  expectedLen  = 0;
  rxBuffer     = "";
  shortMode    = false;
  shortCode    = 0;
  shortCrc     = 0;
  shortWaitCrc = false;
}

// ============================================================
//  字节级处理（帧类型识别 + ASCII帧兼容）
// ============================================================
void processByte(uint8_t b) {

  // ---------- 读取帧类型/长度字节 ----------
  if (rxState == READ_LEN) {

    if (b == FRAME_TYPE_SHORT) {
      // 定长短码帧
      shortMode    = true;
      shortCode    = 0;
      shortCrc     = 0;
      shortWaitCrc = false;
      rxState      = READ_PAYLOAD;
      Serial.println("[帧] 识别为定长短码帧");
      return;
    }

    // 原ASCII帧（向下兼容）
    shortMode   = false;
    expectedLen = (int)b;
    if (expectedLen <= 0 || expectedLen > 64) {
      Serial.println("[帧] 长度非法，丢弃");
      resetRx();
      return;
    }
    rxState = READ_PAYLOAD;
    return;
  }

  // ---------- 读取Payload ----------
  if (rxState == READ_PAYLOAD) {
    if (shortMode) {
      if (shortCode == 0) {
        // 第1个payload字节 = 指令码
        shortCode    = b;
        shortWaitCrc = true;
        Serial.print("[短码] 收到指令码: ");
        Serial.println(shortCode);
        return;
      }
      if (shortWaitCrc) {
        // 第2个payload字节 = CRC
        shortCrc     = b;
        shortWaitCrc = false;
        rxState      = READ_END;
        Serial.print("[CRC] 收到校验值: 0x");
        Serial.println(shortCrc, HEX);
        return;
      }
    } else {
      // ASCII帧
      rxBuffer += (char)b;
      if ((int)rxBuffer.length() >= expectedLen)
        rxState = READ_END;
    }
    return;
  }

  // ---------- 帧尾校验 ----------
  if (rxState == READ_END) {
    if (b == FRAME_END) {
      if (shortMode) {
        // ★ CRC校验
        uint8_t calcCrc = crc8(&shortCode, 1);
        if (calcCrc == shortCrc) {
          const char* cmdStr = shortDecode(shortCode);
          if (strlen(cmdStr) > 0) {
            Serial.print("[CRC OK] 解码成功: ");
            Serial.println(cmdStr);
            applyCommand(String(cmdStr));
          } else {
            Serial.print("[CRC OK] 码字无效: ");
            Serial.println(shortCode);
          }
        } else {
          Serial.print("[CRC FAIL] 期望: 0x");
          Serial.print(calcCrc, HEX);
          Serial.print("  收到: 0x");
          Serial.println(shortCrc, HEX);
        }
      } else {
        // ASCII帧直接执行（向下兼容）
        Serial.print("[ASCII帧] 执行: ");
        Serial.println(rxBuffer);
        applyCommand(rxBuffer);
      }
    } else {
      Serial.println("[帧] 帧尾错误，丢弃");
    }
    resetRx();
  }
}

// ============================================================
//  Web服务器：CORS头
// ============================================================
void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ============================================================
//  Web服务器：/ack 接口（App查询状态）
// ============================================================
void handleAck() {
  sendCorsHeaders();
  String json = "{\"ok\":true,\"device\":\"";
  json += roleName();
  json += "\",\"state\":";
  json += (deviceOn ? "1" : "0");
  json += ",\"lastCmd\":\"";
  json += lastCommand;
  json += "\",\"lastAckMs\":";
  json += String(lastAckMs);
  json += "}";
  server.send(200, "application/json", json);
}

// ============================================================
//  Web服务器：/ 状态页面
// ============================================================
void handleRoot() {
  sendCorsHeaders();
  String html = "<html><head><meta charset='utf-8'></head>";
  html += "<body style='text-align:center;margin-top:40px;font-family:sans-serif'>";
  html += "<h2>VLC接收端</h2>";
  html += "<h3>角色: " + roleName() + "</h3>";
  html += "<h3>状态: " + String(deviceOn ? "<span style='color:green'>ON</span>" : "<span style='color:red'>OFF</span>") + "</h3>";
  html += "<h3>最后指令: " + receivedData + "</h3>";
  html += "<p>ADC阈值: " + String(adcThreshold) + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ============================================================
//  WiFi上传：将设备状态回报给发射端 /upload
// ============================================================
void uploadStateToTx() {
  if (wifiUploading) return;
  if ((millis() - lastUploadTime) < UPLOAD_COOLDOWN) return;

  wifiUploading    = true;
  lastUploadTime   = millis();

  Serial.println("[WiFi] 唤醒，开始上传...");
  WiFi.forceSleepWake();
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 20000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] 已连接");
    WiFiClient client;
    if (client.connect(txHost, txPort)) {
      String path = "/upload?device=" + roleName() + "&state=" + (deviceOn ? "1" : "0");
      client.print("GET " + path + " HTTP/1.1\r\n");
      client.print("Host: ");
      client.print(txHost);
      client.print(":" + String(txPort) + "\r\n");
      client.print("Connection: close\r\n\r\n");
      delay(300);
      client.stop();
      Serial.println("[WiFi] 状态已上报: " + path);
    } else {
      Serial.println("[WiFi] 连接发射端失败");
    }
  } else {
    Serial.println("[WiFi] 连接SSID超时");
  }

  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  Serial.println("[WiFi] 已关闭");
  wifiUploading = false;
}

// ============================================================
//  确保WiFi完全关闭（避免干扰ADC采样）
// ============================================================
void ensureWiFiOff() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(50);
  Serial.println("[WiFi] 已关闭");
}

// ============================================================
//  A0信号自检（启动时）
// ============================================================
void autoCheckSignalPin() {
  Serial.println("\n========== A0 信号自检 ==========");
  const int sampleCount = 100;
  int minVal = 1023, maxVal = 0, sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    int val = analogRead(signalPin);
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
    sum += val;
    delay(10);
  }
  int avgVal = sum / sampleCount;
  Serial.print("最小="); Serial.print(minVal);
  Serial.print("  平均="); Serial.print(avgVal);
  Serial.print("  最大="); Serial.println(maxVal);

  if (maxVal - minVal < 50)
    Serial.println("⚠️ 信号变化太小，请检查LM358接线");
  else
    Serial.println("✅ 信号正常");
  Serial.println("=================================\n");
}

// ============================================================
//  手动设置ADC阈值
// ============================================================
void setThreshold() {
  Serial.println("请输入ADC判决阈值（0~1023），回车确认：");
  while (!Serial.available()) delay(100);
  String input = Serial.readStringUntil('\n');
  int val = input.toInt();
  if (val >= 0 && val <= 1023) {
    adcThreshold = val;
    Serial.print("阈值已设置为: ");
    Serial.println(adcThreshold);
  } else {
    Serial.println("输入无效，保持原阈值");
  }
}

// ============================================================
//  打印帮助菜单
// ============================================================
void printHelp() {
  Serial.println("\n========== 串口命令 ==========");
  Serial.println("  d1    开启信号调试（300ms打印一次）");
  Serial.println("  d2    开启连续打印（1ms打印一次）");
  Serial.println("  d0    关闭调试打印");
  Serial.println("  check 重新执行A0信号自检");
  Serial.println("  thr   重新设置ADC阈值");
  Serial.println("  info  显示当前状态");
  Serial.println("  help  显示本菜单");
  Serial.println("==============================\n");
}

// ============================================================
//  setup
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(100);

  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);

  ensureWiFiOff();

  // 启动Web服务器
  server.on("/", handleRoot);
  server.on("/ack", handleAck);
  server.begin();
  Serial.println("[Web] 服务器已启动（端口80）");

  Serial.println("========================");
  Serial.print  (" VLC接收端  ROLE=");
  Serial.println(roleName());
  Serial.println(" 编码：定长短码 + CRC-8 + 曼彻斯特");
  Serial.println("========================");

  delay(500);
  autoCheckSignalPin();
  setThreshold();
  printHelp();
}

// ============================================================
//  loop
// ============================================================
void loop() {
  // HTTP请求处理（非阻塞）
  server.handleClient();

  // WiFi上传（收到指令后异步执行）
  if (wifiUploadPending && !wifiUploading) {
    wifiUploadPending = false;
    uploadStateToTx();
  }

  // 串口命令处理
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if      (s == "d1")    { debugSignal = true;  continuousDebug = false; Serial.println("调试开（300ms）"); }
    else if (s == "d2")    { debugSignal = true;  continuousDebug = true;  Serial.println("调试开（连续）"); }
    else if (s == "d0")    { debugSignal = false; continuousDebug = false; Serial.println("调试关"); }
    else if (s == "check") { autoCheckSignalPin(); }
    else if (s == "thr")   { setThreshold(); }
    else if (s == "info")  {
      Serial.print("设备状态: "); Serial.println(deviceOn ? "ON" : "OFF");
      Serial.print("上次指令: "); Serial.println(lastCommand);
      Serial.print("ADC阈值: "); Serial.println(adcThreshold);
      Serial.print("A0当前值: "); Serial.println(analogRead(signalPin));
    }
    else if (s == "help")  { printHelp(); }
    else                   { Serial.println("未知命令，输入 help 查看帮助"); }
  }

  // 调试信号打印
  if (debugSignal) {
    if (continuousDebug) {
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 1) {
        lastPrint = millis();
        Serial.println(analogRead(signalPin));
      }
    } else {
      static unsigned long lastDbg = 0;
      if (millis() - lastDbg >= 300) {
        lastDbg = millis();
        Serial.print("A0="); Serial.print(analogRead(signalPin));
        Serial.print(" | "); Serial.println(deviceOn ? "ON" : "OFF");
      }
    }
  }

  // ============================================================
  //  曼彻斯特解码主循环（高精度1000us周期）
  // ============================================================
  static unsigned long lastSample = 0;
  unsigned long now = micros();
  if (now - lastSample < (unsigned long)BIT_PERIOD_US) return;
  lastSample = now;

  // 两次采样判断跳变方向
  int val1 = analogRead(signalPin);
  delayMicroseconds(BIT_PERIOD_US / 2);
  int val2 = analogRead(signalPin);

  bool bit;
  if      (val1 >  adcThreshold && val2 <= adcThreshold) bit = 1;  // 高→低 = 1
  else if (val1 <= adcThreshold && val2 >  adcThreshold) bit = 0;  // 低→高 = 0
  else    bit = 0;  // 无跳变，默认0

  // ---- 前导码检测 ----
  if (rxState == WAIT_PREAMBLE) {
    shiftReg = ((shiftReg << 1) | (bit ? 1 : 0)) & PREAMBLE_MASK;
    if (shiftReg == PREAMBLE_VALUE) {
      rxState    = READ_LEN;
      rxBitCount = 0;
      rxByte     = 0;
      Serial.println("[同步] 前导码匹配，开始接收帧");
    }
    return;
  }

  // ---- 定长短码帧：payload阶段逐字节由processByte处理 ----
  // ---- 字节级收集 ----
  rxByte = (rxByte << 1) | (bit ? 1 : 0);
  rxBitCount++;
  if (rxBitCount == 8) {
    processByte(rxByte);
    rxBitCount = 0;
    rxByte     = 0;
  }
}
