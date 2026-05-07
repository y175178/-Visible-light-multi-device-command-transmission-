// ==================== VLC 接收端完整代码（调试版）====================
// 新增功能：
//   1. 自动阈值校准（启动时自动采样计算中值，无需手动输入）
//   2. 误码率统计：总收帧数、CRC失败帧数，定期上报发射端 /ber_ack
//   3. 接收任意ASCII帧（配合发射端调试模式）
// 编码方案：定长短码（信源编码）+ CRC-8（信道编码）+ 曼彻斯特（物理层）

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ==================== 硬件配置 ====================
const int signalPin = A0;
const int outPin    = D1;

// ==================== 通信参数 ====================
const int BIT_PERIOD_US = 1000;
int adcThreshold        = 13;    // 启动时自动校准覆盖此值

// ==================== 设备角色 ====================
// 1=LIGHT  2=SERVO  3=FAN
const int DEVICE_ROLE = 1;

// ==================== WiFi配置 ====================
const char* ssid     = "打扫干净屋子再请客";
const char* password = "1751787761";
const char* txHost   = "192.168.198.53";
const int   txPort   = 5000;

// ==================== 帧类型 ====================
#define FRAME_TYPE_SHORT 0xBB
#define FRAME_END        0xFF

// ==================== 设备状态 ====================
bool   deviceOn     = false;
String lastCommand  = "";
String receivedData = "等待接收数据...";
unsigned long lastAckMs = 0;

// ==================== 误码率统计 ====================
uint32_t statTotalRecv  = 0;   // 成功解码帧数（CRC通过）
uint32_t statCrcFail    = 0;   // CRC失败帧数
uint32_t statFrameError = 0;   // 帧结构错误数（帧尾错误等）

// ==================== WiFi上传 ====================
bool wifiUploadPending  = false;
bool wifiBerPending     = false;   // 需要上报误码统计
bool wifiUploading      = false;
unsigned long lastUploadTime = 0;
const unsigned long UPLOAD_COOLDOWN = 100;

// ==================== 调试开关 ====================
bool debugSignal     = false;
bool continuousDebug = false;

// ==================== Web服务器 ====================
ESP8266WebServer server(80);

// ============================================================
//  CRC-8（多项式 0x07，与发射端完全一致）
// ============================================================
uint8_t crc8(uint8_t* d, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}

// ============================================================
//  定长短码解码
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
  Serial.print("[收到指令] "); Serial.println(cmd);

  if (commandMatchRole(cmd)) {
    deviceOn = cmd.endsWith("_ON");
    digitalWrite(outPin, deviceOn ? HIGH : LOW);
    lastCommand  = cmd;
    receivedData = cmd;
    lastAckMs    = millis();
    Serial.print("[执行] "); Serial.println(cmd);
    wifiUploadPending = true;
  } else {
    // 非本角色指令，仅记录不执行
    Serial.println("[忽略] 非本设备指令");
  }
}

// ============================================================
//  曼彻斯特解码状态机
// ============================================================
enum RxState { WAIT_PREAMBLE, READ_LEN, READ_PAYLOAD, READ_END };
RxState rxState = WAIT_PREAMBLE;

uint32_t shiftReg = 0;
const uint32_t PREAMBLE_MASK  = 0x01FFFFFF;
const uint32_t PREAMBLE_VALUE = 0x01FFFFFF;

uint8_t rxByte     = 0;
int     rxBitCount = 0;

// 短码帧
bool    shortMode    = false;
uint8_t shortCode    = 0;
uint8_t shortCrc     = 0;
bool    shortWaitCrc = false;

// ASCII帧
bool    asciiMode   = false;
int     expectedLen = 0;
String  rxBuffer    = "";
bool    asciiWaitCrc = false;
uint8_t asciiCrc    = 0;

// ============================================================
//  复位状态机
// ============================================================
void resetRx() {
  rxState      = WAIT_PREAMBLE;
  shiftReg     = 0;
  rxByte       = 0;
  rxBitCount   = 0;
  shortMode    = false;
  shortCode    = 0;
  shortCrc     = 0;
  shortWaitCrc = false;
  asciiMode    = false;
  expectedLen  = 0;
  rxBuffer     = "";
  asciiWaitCrc = false;
  asciiCrc     = 0;
}

// ============================================================
//  字节级帧处理
// ============================================================
void processByte(uint8_t b) {

  // ---------- READ_LEN：识别帧类型 ----------
  if (rxState == READ_LEN) {
    if (b == FRAME_TYPE_SHORT) {
      shortMode    = true;
      asciiMode    = false;
      shortCode    = 0;
      shortCrc     = 0;
      shortWaitCrc = false;
      rxState      = READ_PAYLOAD;
      return;
    }
    // ASCII帧：b为长度字节
    if (b == 0 || b > 64) {
      Serial.println("[帧] 长度非法，丢弃");
      statFrameError++;
      resetRx();
      return;
    }
    asciiMode    = true;
    shortMode    = false;
    expectedLen  = (int)b;
    rxBuffer     = "";
    asciiWaitCrc = false;
    rxState      = READ_PAYLOAD;
    return;
  }

  // ---------- READ_PAYLOAD ----------
  if (rxState == READ_PAYLOAD) {
    if (shortMode) {
      if (shortCode == 0) {
        shortCode    = b;
        shortWaitCrc = true;
        return;
      }
      if (shortWaitCrc) {
        shortCrc     = b;
        shortWaitCrc = false;
        rxState      = READ_END;
        return;
      }
    }
    if (asciiMode) {
      if (!asciiWaitCrc) {
        rxBuffer += (char)b;
        if ((int)rxBuffer.length() >= expectedLen) {
          asciiWaitCrc = true;  // 下一字节是CRC
        }
        return;
      }
      if (asciiWaitCrc) {
        asciiCrc     = b;
        asciiWaitCrc = false;
        rxState      = READ_END;
        return;
      }
    }
    return;
  }

  // ---------- READ_END：帧尾校验 ----------
  if (rxState == READ_END) {
    if (b == FRAME_END) {
      if (shortMode) {
        // CRC校验
        uint8_t calcCrc = crc8(&shortCode, 1);
        if (calcCrc == shortCrc) {
          statTotalRecv++;
          const char* cmdStr = shortDecode(shortCode);
          if (strlen(cmdStr) > 0) {
            Serial.print("[短码 CRC OK] "); Serial.println(cmdStr);
            applyCommand(String(cmdStr));
          } else {
            Serial.print("[短码 CRC OK] 未知码字: "); Serial.println(shortCode);
          }
        } else {
          statCrcFail++;
          Serial.print("[CRC FAIL] 期望:0x"); Serial.print(calcCrc, HEX);
          Serial.print(" 收到:0x"); Serial.println(shortCrc, HEX);
        }
        // 每10帧上报一次误码统计
        if ((statTotalRecv + statCrcFail) % 10 == 0) wifiBerPending = true;
      }

      if (asciiMode) {
        // CRC校验（对长度+内容）
        uint8_t buf[65];
        buf[0] = (uint8_t)rxBuffer.length();
        for (int i = 0; i < (int)rxBuffer.length(); i++) buf[i+1] = (uint8_t)rxBuffer[i];
        uint8_t calcCrc = crc8(buf, rxBuffer.length() + 1);
        if (calcCrc == asciiCrc) {
          statTotalRecv++;
          Serial.print("[ASCII CRC OK] "); Serial.println(rxBuffer);
          applyCommand(rxBuffer);
        } else {
          statCrcFail++;
          Serial.print("[ASCII CRC FAIL] 期望:0x"); Serial.print(calcCrc, HEX);
          Serial.print(" 收到:0x"); Serial.println(asciiCrc, HEX);
          Serial.print("[内容] "); Serial.println(rxBuffer);
        }
        if ((statTotalRecv + statCrcFail) % 10 == 0) wifiBerPending = true;
      }
    } else {
      statFrameError++;
      Serial.println("[帧] 帧尾错误，丢弃");
    }
    resetRx();
  }
}

// ============================================================
//  自动阈值校准（采样中值法）
// ============================================================
int autoCalibrate() {
  Serial.println("\n========== 自动阈值校准 ==========");
  Serial.println("请保持LED发射端关闭（环境光状态），校准中...");
  delay(500);

  const int sampleCount = 200;
  long sum = 0;
  int  minVal = 1023, maxVal = 0;

  for (int i = 0; i < sampleCount; i++) {
    int v = analogRead(signalPin);
    sum += v;
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    delay(5);
  }
  int avgVal = (int)(sum / sampleCount);

  Serial.print("环境光底噪: 最小="); Serial.print(minVal);
  Serial.print("  平均="); Serial.print(avgVal);
  Serial.print("  最大="); Serial.println(maxVal);

  // 阈值 = 平均值 + 20%浮动（留出判决余量）
  int threshold = avgVal + (maxVal - avgVal) / 2;
  // 保护：阈值不超过800，防止信号太弱时误判
  if (threshold > 800) threshold = avgVal + 5;
  if (threshold < 5)   threshold = 5;

  Serial.print("✅ 自动校准阈值: "); Serial.println(threshold);
  Serial.println("===================================\n");
  return threshold;
}

// ============================================================
//  手动设置阈值（备用）
// ============================================================
void manualSetThreshold() {
  Serial.println("请输入ADC阈值（0~1023），回车确认：");
  unsigned long t = millis();
  while (!Serial.available()) {
    if (millis() - t > 10000) {
      Serial.println("超时，保持当前阈值");
      return;
    }
    delay(100);
  }
  String input = Serial.readStringUntil('\n');
  int val = input.toInt();
  if (val >= 0 && val <= 1023) {
    adcThreshold = val;
    Serial.print("阈值已设置为: "); Serial.println(adcThreshold);
  } else {
    Serial.println("输入无效，保持原阈值");
  }
}

// ============================================================
//  Web服务器：CORS
// ============================================================
void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ============================================================
//  Web服务器：/ack
// ============================================================
void handleAck() {
  sendCorsHeaders();
  String json = "{\"ok\":true,\"device\":\"" + roleName() + "\"";
  json += ",\"state\":"    + String(deviceOn ? 1 : 0);
  json += ",\"lastCmd\":\"" + lastCommand + "\"";
  json += ",\"lastAckMs\":" + String(lastAckMs) + "}";
  server.send(200, "application/json", json);
}

// ============================================================
//  Web服务器：/ber（查询本机误码率统计）
// ============================================================
void handleBer() {
  sendCorsHeaders();
  uint32_t total = statTotalRecv + statCrcFail;
  float crcRate  = (total > 0) ? (float)statCrcFail / total * 100.0f : 0.0f;
  String json = "{";
  json += "\"total_recv\":"   + String(statTotalRecv)  + ",";
  json += "\"crc_fail\":"     + String(statCrcFail)    + ",";
  json += "\"frame_error\":"  + String(statFrameError) + ",";
  json += "\"total_frame\":"  + String(total)          + ",";
  json += "\"crc_fail_rate\":" + String(crcRate, 2)   + ",";
  json += "\"threshold\":"    + String(adcThreshold)   + "}";
  server.send(200, "application/json", json);
}

// ============================================================
//  Web服务器：/ber_reset
// ============================================================
void handleBerReset() {
  sendCorsHeaders();
  statTotalRecv = statCrcFail = statFrameError = 0;
  Serial.println("[BER] 统计已重置");
  server.send(200, "application/json", "{\"ok\":true}");
}

// ============================================================
//  Web服务器：/recalibrate（远程触发阈值重校准）
// ============================================================
void handleRecalibrate() {
  sendCorsHeaders();
  Serial.println("[校准] 远程触发阈值重校准...");
  int newThreshold = autoCalibrate();
  String json = "{\"ok\":true,\"threshold\":" + String(newThreshold) + "}";
  server.send(200, "application/json", json);
}

// ============================================================
//  Web服务器：/ 状态页
// ============================================================
void handleRoot() {
  sendCorsHeaders();
  uint32_t total = statTotalRecv + statCrcFail;
  float crcRate  = (total > 0) ? (float)statCrcFail / total * 100.0f : 0.0f;
  String html = "<html><head><meta charset='utf-8'><meta http-equiv='refresh' content='2'></head>";
  html += "<body style='text-align:center;margin-top:40px;font-family:sans-serif'>";
  html += "<h2>VLC接收端（调试版）</h2>";
  html += "<h3>角色: " + roleName() + "</h3>";
  html += "<h3>状态: " + String(deviceOn ? "<span style='color:green'>ON</span>" : "<span style='color:red'>OFF</span>") + "</h3>";
  html += "<h3>最后指令: " + receivedData + "</h3>";
  html += "<hr><h3>误码率统计</h3>";
  html += "<p>总收帧: " + String(total) + "</p>";
  html += "<p>CRC通过: " + String(statTotalRecv) + "</p>";
  html += "<p>CRC失败: " + String(statCrcFail) + " (" + String(crcRate, 1) + "%)</p>";
  html += "<p>帧结构错误: " + String(statFrameError) + "</p>";
  html += "<p>ADC阈值: " + String(adcThreshold) + "</p>";
  html += "<p><a href='/ber_reset'>重置统计</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ============================================================
//  WiFi操作：上传状态 + 误码统计
// ============================================================
void doWifiUpload() {
  if (wifiUploading) return;
  if ((millis() - lastUploadTime) < UPLOAD_COOLDOWN) return;

  wifiUploading  = true;
  lastUploadTime = millis();

  Serial.println("[WiFi] 唤醒...");
  WiFi.forceSleepWake(); delay(50);
  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) delay(100);

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;

    // 上传设备状态
    if (wifiUploadPending) {
      if (client.connect(txHost, txPort)) {
        String path = "/upload?device=" + roleName() + "&state=" + (deviceOn ? "1" : "0");
        client.print("GET " + path + " HTTP/1.1\r\nHost: " + String(txHost) + ":" + txPort + "\r\nConnection: close\r\n\r\n");
        delay(200); client.stop();
        Serial.println("[WiFi] 状态已上报");
        wifiUploadPending = false;
      } else {
        Serial.println("[WiFi] 连接发射端失败");
        wifiUploadPending = false;  // 失败也清除标志，避免死循环
      }
    }

    // 上报误码统计
    if (wifiBerPending) {
      if (client.connect(txHost, txPort)) {
        String path = "/ber_ack?recv=" + String(statTotalRecv) + "&crc_fail=" + String(statCrcFail);
        client.print("GET " + path + " HTTP/1.1\r\nHost: " + String(txHost) + ":" + txPort + "\r\nConnection: close\r\n\r\n");
        delay(200); client.stop();
        Serial.println("[WiFi] 误码统计已上报");
        wifiBerPending = false;
      } else {
        Serial.println("[WiFi] 连接发射端失败");
        wifiBerPending = false;  // 失败也清除标志，避免死循环
      }
    }
  } else {
    Serial.println("[WiFi] 连接失败");
    // ★ 关键修复：连接失败时也要清除标志
    wifiUploadPending = false;
    wifiBerPending = false;
  }

  WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin();
  Serial.println("[WiFi] 已关闭");
  wifiUploading = false;
}

// ============================================================
//  打印帮助菜单
// ============================================================
void printHelp() {
  Serial.println("\n========== 串口命令 ==========");
  Serial.println("  d1    开启信号调试（300ms）");
  Serial.println("  d2    开启连续打印（1ms）");
  Serial.println("  d0    关闭调试");
  Serial.println("  ber   查看本机误码率统计");
  Serial.println("  reset 重置误码率统计");
  Serial.println("  thr   手动输入阈值");
  Serial.println("  cal   重新自动校准阈值");
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

  // 先关WiFi，保证ADC采样干净
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(100);

  // ★ 自动阈值校准
  adcThreshold = autoCalibrate();

  // 校准完成后启动WiFi和Web服务器
  Serial.println("[WiFi] 连接中...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] 连接成功！");
    Serial.println("========================");
    Serial.print("设备ID: ESP8266-接收端-");
    Serial.println(roleName());
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
    Serial.print("端口: 80");
    Serial.println();
    Serial.print("访问地址: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.print("配置的发射端IP: ");
    Serial.print(txHost);
    Serial.print(":");
    Serial.println(txPort);
    Serial.println("========================");
  } else {
    Serial.println("\n[WiFi] 连接失败");
  }

  // 启动Web服务器
  server.on("/",            handleRoot);
  server.on("/ack",         handleAck);
  server.on("/ber",         handleBer);
  server.on("/ber_reset",   handleBerReset);
  server.on("/recalibrate", handleRecalibrate);
  server.begin();
  Serial.println("[Web] 服务器已启动（端口80）");

  Serial.println("========================");
  Serial.print  (" VLC接收端（调试版）ROLE=");
  Serial.println(roleName());
  Serial.println(" 编码：定长短码 + CRC-8 + 曼彻斯特");
  Serial.print  (" ADC阈值（自动）: ");
  Serial.println(adcThreshold);
  Serial.println("========================");

  printHelp();
}

// ============================================================
//  loop
// ============================================================
void loop() {
  server.handleClient();

  // WiFi上传（状态或误码统计）
  if ((wifiUploadPending || wifiBerPending) && !wifiUploading) {
    doWifiUpload();
  }

  // 串口命令
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if      (s == "d1")    { debugSignal = true;  continuousDebug = false; Serial.println("调试开（300ms）"); }
    else if (s == "d2")    { debugSignal = true;  continuousDebug = true;  Serial.println("调试开（连续）"); }
    else if (s == "d0")    { debugSignal = false; continuousDebug = false; Serial.println("调试关"); }
    else if (s == "cal")   { adcThreshold = autoCalibrate(); }
    else if (s == "thr")   { manualSetThreshold(); }
    else if (s == "reset") {
      statTotalRecv = statCrcFail = statFrameError = 0;
      Serial.println("[BER] 统计已重置");
    }
    else if (s == "ber") {
      uint32_t total = statTotalRecv + statCrcFail;
      float crcRate  = (total > 0) ? (float)statCrcFail / total * 100.0f : 0.0f;
      Serial.println("\n===== 本机误码统计 =====");
      Serial.print("总收帧:      "); Serial.println(total);
      Serial.print("CRC通过:     "); Serial.println(statTotalRecv);
      Serial.print("CRC失败:     "); Serial.println(statCrcFail);
      Serial.print("CRC失败率:   "); Serial.print(crcRate, 2); Serial.println("%");
      Serial.print("帧结构错误:  "); Serial.println(statFrameError);
      Serial.print("ADC阈值:     "); Serial.println(adcThreshold);
      Serial.println("========================\n");
    }
    else if (s == "info") {
      Serial.print("状态: ");    Serial.println(deviceOn ? "ON" : "OFF");
      Serial.print("上次指令: "); Serial.println(lastCommand);
      Serial.print("ADC阈值: "); Serial.println(adcThreshold);
      Serial.print("A0当前值: "); Serial.println(analogRead(signalPin));
    }
    else if (s == "help") { printHelp(); }
    else { Serial.println("未知命令，输入 help"); }
  }

  // 调试信号打印
  if (debugSignal) {
    if (continuousDebug) {
      static unsigned long lp = 0;
      if (millis() - lp >= 1) { lp = millis(); Serial.println(analogRead(signalPin)); }
    } else {
      static unsigned long ld = 0;
      if (millis() - ld >= 300) {
        ld = millis();
        Serial.print("A0="); Serial.print(analogRead(signalPin));
        Serial.print(" thr="); Serial.print(adcThreshold);
        Serial.print(" | "); Serial.println(deviceOn ? "ON" : "OFF");
      }
    }
  }

  // ============================================================
  //  曼彻斯特解码主循环
  // ============================================================
  static unsigned long lastSample = 0;
  unsigned long now = micros();
  if (now - lastSample < (unsigned long)BIT_PERIOD_US) return;
  lastSample = now;

  int val1 = analogRead(signalPin);
  delayMicroseconds(BIT_PERIOD_US / 2);
  int val2 = analogRead(signalPin);

  bool bit;
  if      (val1 >  adcThreshold && val2 <= adcThreshold) bit = 1;
  else if (val1 <= adcThreshold && val2 >  adcThreshold) bit = 0;
  else    bit = 0;

  // 前导码检测
  if (rxState == WAIT_PREAMBLE) {
    shiftReg = ((shiftReg << 1) | (bit ? 1 : 0)) & PREAMBLE_MASK;
    if (shiftReg == PREAMBLE_VALUE) {
      rxState    = READ_LEN;
      rxBitCount = 0;
      rxByte     = 0;
    }
    return;
  }

  // 字节级收集
  rxByte = (rxByte << 1) | (bit ? 1 : 0);
  rxBitCount++;
  if (rxBitCount == 8) {
    processByte(rxByte);
    rxBitCount = 0;
    rxByte     = 0;
  }
}
