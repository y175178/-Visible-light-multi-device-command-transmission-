// ==================== VLC 发射端完整代码（调试版）====================
// 新增功能：
//   1. 串口可发送任意字符串指令（不限于6条固定指令）
//   2. 自动误码率统计（需配合接收端 /ber 接口）
//   3. 批量自动发送测试模式
// 编码方案：定长短码（信源编码）+ CRC-8（信道编码）+ 曼彻斯特（物理层）

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

// ==================== 硬件配置 ====================
const int ledPin = 16;

// ==================== 通信参数 ====================
const int BIT_PERIOD_US = 1000;  // 1kbps
const int FRAME_REPEAT  = 2;     // 每条指令重复发送次数
const int RAW_REPEAT    = 1;     // Raw模式重复次数（测误码率不重复）
const int FRAME_GAP_MS  = 12;    // 两帧间隔

// ==================== 帧类型 ====================
#define FRAME_TYPE_SHORT 0xBB    // 定长短码帧
#define FRAME_END        0xFF    // 帧尾

// ==================== WiFi配置 ====================
const char* ssid     = "打扫干净屋子再请客";
const char* password = "1751787761";  // 请替换为实际密码

// ==================== 定长短码码表（固定6条指令）====================
struct ShortEntry {
  const char* cmd;
  uint8_t     code;
};
const ShortEntry SHORT_TABLE[] = {
  { "LIGHT_ON",  1 },
  { "LIGHT_OFF", 2 },
  { "SERVO_ON",  3 },
  { "SERVO_OFF", 4 },
  { "FAN_ON",    5 },
  { "FAN_OFF",   6 },
};
const int SHORT_TABLE_SIZE = 6;

// ==================== 状态管理 ====================
String expectedLightState = "0";
String expectedServoState = "0";
String expectedFanState   = "0";
unsigned long lastCommandTime = 0;
bool   lightOn = false;
bool   servoOn = false;
bool   fanOn   = false;
String pendingCommand = "";

// ==================== 误码率统计 ====================
uint32_t statTotalSent    = 0;   // 总发送帧数
uint32_t statTotalRecv    = 0;   // 接收端确认收到帧数（由/ber_ack更新）
uint32_t statCrcFail      = 0;   // 接收端CRC失败帧数（由/ber_ack更新）

// ==================== Raw模式（误码率测试） ====================
bool rawMode = false;  // true: 只发原始数据位，无前导码/CRC/帧尾

// ==================== 自动测试模式 ====================
bool     autoTestMode     = false;
uint32_t autoTestTotal    = 0;   // 计划发送总次数
uint32_t autoTestSent     = 0;   // 已发送次数
String   autoTestCmd      = "";  // 测试用指令
uint32_t autoTestInterval = 500; // 每次发送间隔ms
unsigned long lastAutoSend = 0;

// ==================== 传感器数据 ====================
String data = "{'light':0,'temp':0,'hum':0}";

WebServer server(5000);

// ============================================================
//  CRC-8（多项式 0x07）
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
//  曼彻斯特编码发送
// ============================================================
void sendBit(bool bit) {
  if (bit) {
    digitalWrite(ledPin, HIGH); delayMicroseconds(BIT_PERIOD_US / 2);
    digitalWrite(ledPin, LOW);  delayMicroseconds(BIT_PERIOD_US / 2);
  } else {
    digitalWrite(ledPin, LOW);  delayMicroseconds(BIT_PERIOD_US / 2);
    digitalWrite(ledPin, HIGH); delayMicroseconds(BIT_PERIOD_US / 2);
  }
}

void sendByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) sendBit((b >> i) & 1);
}

// ============================================================
//  发送NRZ字节（通用，带起始标记）
//  格式：[OFF 3ms] [ON 2ms] [OFF 1ms] [8bit NRZ]
//  接收端检测 ON→OFF 下降沿后，下一个采样点即为 data[7]
// ============================================================
void sendNrzByte(uint8_t b) {
  digitalWrite(ledPin, LOW);
  delayMicroseconds(3000);
  digitalWrite(ledPin, HIGH);     // ON脉冲 2ms
  delayMicroseconds(2000);
  digitalWrite(ledPin, LOW);      // OFF间隙 1ms（产生下降沿）
  delayMicroseconds(BIT_PERIOD_US);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(ledPin, (b >> i) & 1 ? HIGH : LOW);
    delayMicroseconds(BIT_PERIOD_US);
  }
  digitalWrite(ledPin, LOW);
}

// ============================================================
//  发送定长短码帧（NRZ编码，无前导码）
//  帧格式：[0xBB] [code] [CRC] [0xFF]  每字节独立NRZ发送
// ============================================================
void sendShortFrame(uint8_t code) {
  uint8_t crc = crc8(&code, 1);
  for (int n = 0; n < FRAME_REPEAT; n++) {
    sendNrzByte(FRAME_TYPE_SHORT);  // 帧类型 0xBB
    delay(2);
    sendNrzByte(code);              // 指令码
    delay(2);
    sendNrzByte(crc);               // CRC
    delay(2);
    sendNrzByte(FRAME_END);         // 帧尾 0xFF
    delay(FRAME_GAP_MS);
  }
  statTotalSent++;
}

// 发送NRZ原始字节（用于误码率测试，单个字节）
void sendRawByte(uint8_t code) {
  for (int n = 0; n < RAW_REPEAT; n++) {
    sendNrzByte(code);
    delay(FRAME_GAP_MS);
  }
  statTotalSent++;
}

// ============================================================
//  发送ASCII帧（NRZ编码，无前导码）
//  帧格式：[len] [字符串bytes] [CRC] [0xFF]  每字节独立NRZ发送
// ============================================================
void sendAsciiFrame(const String& msg) {
  uint8_t len = (uint8_t)msg.length();
  uint8_t buf[65];
  buf[0] = len;
  for (int i = 0; i < len; i++) buf[i + 1] = (uint8_t)msg[i];
  uint8_t crc = crc8(buf, len + 1);

  for (int n = 0; n < FRAME_REPEAT; n++) {
    sendNrzByte(len);               // 长度
    delay(2);
    for (char c : msg) {
      sendNrzByte((uint8_t)c);      // 内容
      delay(2);
    }
    sendNrzByte(crc);               // CRC
    delay(2);
    sendNrzByte(FRAME_END);         // 帧尾
    delay(FRAME_GAP_MS);
  }
  statTotalSent++;
}

// ============================================================
//  执行并发送指令（自动选择帧类型）
// ============================================================
void applyCommand(const String& command) {
  Serial.print("[发送] "); Serial.println(command);

  // ---- Raw模式：只发原始数据位，无帧结构 ----
  if (rawMode) {
    int idx = -1;
    for (int i = 0; i < SHORT_TABLE_SIZE; i++) {
      if (command == SHORT_TABLE[i].cmd) { idx = i; break; }
    }
    if (idx >= 0) {
      Serial.print("[Raw] 短码="); Serial.println(SHORT_TABLE[idx].code);
      sendRawByte(SHORT_TABLE[idx].code);
    } else {
      // 任意字符串：逐字节发送（每个字节独立发）
      Serial.print("[Raw] ASCII逐字节: ");
      for (int i = 0; i < (int)command.length(); i++) {
        if (i > 0) delay(FRAME_GAP_MS * 2);
        sendRawByte((uint8_t)command[i]);
        Serial.print(command[i]);
      }
      Serial.println();
    }
    Serial.println("[Raw发送完成] ----------------");
    return;
  }

  // 更新预期状态
  if      (command == "LIGHT_ON")  expectedLightState = "1";
  else if (command == "LIGHT_OFF") expectedLightState = "0";
  else if (command == "SERVO_ON")  expectedServoState = "1";
  else if (command == "SERVO_OFF") expectedServoState = "0";
  else if (command == "FAN_ON")    expectedFanState   = "1";
  else if (command == "FAN_OFF")   expectedFanState   = "0";

  lastCommandTime = millis();

  // 查短码表
  int idx = -1;
  for (int i = 0; i < SHORT_TABLE_SIZE; i++) {
    if (command == SHORT_TABLE[i].cmd) { idx = i; break; }
  }

  if (idx >= 0) {
    // 固定指令：用定长短码帧
    Serial.print("[短码帧] code="); Serial.println(SHORT_TABLE[idx].code);
    sendShortFrame(SHORT_TABLE[idx].code);
  } else {
    // 任意指令：用ASCII帧（调试模式）
    Serial.println("[ASCII帧] 任意指令模式");
    sendAsciiFrame(command);
  }

  Serial.println("[发送完成] ----------------");
}

void queueCommand(const String& command) { pendingCommand = command; }

// ============================================================
//  HTTP：CORS头
// ============================================================
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ============================================================
//  HTTP：/data
// ============================================================
void sendDataHandler() {
  addCorsHeaders();
  server.send(200, "text/plain", data);
}

// ============================================================
//  HTTP：/status
// ============================================================
void statusHandler() {
  addCorsHeaders();
  unsigned long timeSince = (lastCommandTime > 0) ? (millis() - lastCommandTime) : 999999;
  bool waiting = (timeSince < 2000);

  String s = "{";
  s += "\"confirmed_state\":{\"light\":" + String(lightOn?1:0) + ",\"servo\":" + String(servoOn?1:0) + ",\"fan\":" + String(fanOn?1:0) + "},";
  s += "\"expected_state\":{\"light\":" + expectedLightState + ",\"servo\":" + expectedServoState + ",\"fan\":" + expectedFanState + "},";
  s += "\"sync_status\":\"" + String(waiting ? "waiting_feedback" : "ready") + "\",";
  s += "\"time_since_command\":" + String(timeSince) + "}";
  server.send(200, "application/json", s);
}

// ============================================================
//  HTTP：/control（App发送指令）
// ============================================================
void controlHandler() {
  addCorsHeaders();
  String command = server.arg("cmd");
  command.trim(); command.toUpperCase();

  if (command.length() == 0) {
    server.send(400, "application/json", "{\"ok\":false,\"message\":\"missing cmd\"}");
    return;
  }

  String result = "{\"ok\":true,\"cmd\":\"" + command + "\",\"status\":\"command_sent_waiting_feedback\"}";
  server.send(200, "application/json", result);
  queueCommand(command);
}

// ============================================================
//  HTTP：/upload（接收端回报设备状态）
// ============================================================
void uploadHandler() {
  addCorsHeaders();
  String device   = server.arg("device"); device.trim(); device.toUpperCase();
  String stateStr = server.arg("state");
  int    state    = stateStr.toInt();

  if      (device == "LIGHT") lightOn = (state == 1);
  else if (device == "SERVO") servoOn = (state == 1);
  else if (device == "FAN")   fanOn   = (state == 1);

  Serial.print("[上报] "); Serial.print(device);
  Serial.print(" -> "); Serial.println(state ? "ON" : "OFF");

  server.send(200, "application/json",
    "{\"ok\":true,\"device\":\"" + device + "\",\"state\":" + state + "}");
}

// ============================================================
//  HTTP：/ber_ack（接收端上报误码率统计）
//  接收端调用：/ber_ack?recv=N&crc_fail=M
// ============================================================
void berAckHandler() {
  addCorsHeaders();
  String recvStr    = server.arg("recv");
  String crcFailStr = server.arg("crc_fail");

  if (recvStr.length() > 0)    statTotalRecv = recvStr.toInt();
  if (crcFailStr.length() > 0) statCrcFail   = crcFailStr.toInt();

  Serial.print("[BER] 接收端上报 recv="); Serial.print(statTotalRecv);
  Serial.print(" crc_fail="); Serial.println(statCrcFail);

  server.send(200, "application/json", "{\"ok\":true}");
}

// ============================================================
//  HTTP：/ber_stat（查询误码率统计）
// ============================================================
void berStatHandler() {
  addCorsHeaders();
  uint32_t lost    = (statTotalSent > statTotalRecv) ? (statTotalSent - statTotalRecv) : 0;
  float    per     = (statTotalSent > 0) ? (float)lost / statTotalSent * 100.0f : 0.0f;
  float    crcRate = (statTotalRecv > 0) ? (float)statCrcFail / statTotalRecv * 100.0f : 0.0f;

  String s = "{";
  s += "\"total_sent\":"    + String(statTotalSent)  + ",";
  s += "\"total_recv\":"    + String(statTotalRecv)  + ",";
  s += "\"lost\":"          + String(lost)           + ",";
  s += "\"loss_rate\":"     + String(per, 2)         + ",";
  s += "\"crc_fail\":"      + String(statCrcFail)    + ",";
  s += "\"crc_fail_rate\":" + String(crcRate, 2)     + "}";
  server.send(200, "application/json", s);
}

// ============================================================
//  HTTP：/ber_reset（重置统计）
// ============================================================
void berResetHandler() {
  addCorsHeaders();
  statTotalSent = statTotalRecv = statCrcFail = 0;
  Serial.println("[BER] 统计已重置");
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"reset ok\"}");
}

// ============================================================
//  HTTP：/ 根路径（状态页面）
// ============================================================
void handleRoot() {
  uint32_t lost = (statTotalSent > statTotalRecv) ? (statTotalSent - statTotalRecv) : 0;
  float per = (statTotalSent > 0) ? (float)lost / statTotalSent * 100.0f : 0.0f;
  float crcRate = (statTotalRecv > 0) ? (float)statCrcFail / statTotalRecv * 100.0f : 0.0f;

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta http-equiv='refresh' content='3'>";
  html += "<style>body{font-family:Arial;margin:40px;background:#f0f0f0;}";
  html += ".card{background:white;padding:20px;margin:10px 0;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
  html += "h1{color:#333;}h2{color:#666;border-bottom:2px solid #4CAF50;padding-bottom:10px;}";
  html += ".status{font-size:18px;margin:10px 0;}.on{color:green;font-weight:bold;}.off{color:red;font-weight:bold;}";
  html += ".stat{display:flex;justify-content:space-between;padding:8px;border-bottom:1px solid #eee;}";
  html += ".stat:last-child{border:none;}</style></head><body>";

  html += "<h1>🚀 VLC 发射端控制台</h1>";

  html += "<div class='card'><h2>设备状态</h2>";
  html += "<div class='status'>💡 灯光: <span class='" + String(lightOn?"on":"off") + "'>" + (lightOn?"ON":"OFF") + "</span></div>";
  html += "<div class='status'>🔧 舵机: <span class='" + String(servoOn?"on":"off") + "'>" + (servoOn?"ON":"OFF") + "</span></div>";
  html += "<div class='status'>🌀 风扇: <span class='" + String(fanOn?"on":"off") + "'>" + (fanOn?"ON":"OFF") + "</span></div>";
  html += "</div>";

  html += "<div class='card'><h2>📊 误码率统计</h2>";
  html += "<div class='stat'><span>总发送帧数:</span><span><b>" + String(statTotalSent) + "</b></span></div>";
  html += "<div class='stat'><span>接收端确认:</span><span><b>" + String(statTotalRecv) + "</b></span></div>";
  html += "<div class='stat'><span>丢失帧数:</span><span><b>" + String(lost) + "</b></span></div>";
  html += "<div class='stat'><span>丢帧率:</span><span><b>" + String(per, 2) + "%</b></span></div>";
  html += "<div class='stat'><span>CRC失败:</span><span><b>" + String(statCrcFail) + "</b></span></div>";
  html += "<div class='stat'><span>CRC失败率:</span><span><b>" + String(crcRate, 2) + "%</b></span></div>";
  html += "</div>";

  html += "<div class='card'><h2>🔗 API 接口</h2>";
  html += "<p><a href='/status'>/status</a> - 设备状态JSON</p>";
  html += "<p><a href='/ber_stat'>/ber_stat</a> - 误码率统计JSON</p>";
  html += "<p><a href='/control?cmd=LIGHT_ON'>/control?cmd=LIGHT_ON</a> - 发送指令</p>";
  html += "<p><a href='/ber_reset'>/ber_reset</a> - 重置统计</p>";
  html += "</div>";

  html += "<p style='text-align:center;color:#999;margin-top:30px;'>页面每3秒自动刷新</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ============================================================
//  自动测试：打印当前进度
// ============================================================
void printAutoTestProgress() {
  Serial.print("[自动测试] ");
  Serial.print(autoTestSent); Serial.print("/"); Serial.print(autoTestTotal);
  Serial.print("  已发="); Serial.print(statTotalSent);
  Serial.print("  收到="); Serial.print(statTotalRecv);
  Serial.print("  CRC失败="); Serial.println(statCrcFail);
}

// ============================================================
//  WiFi连接
// ============================================================
void wifiConnect() {
  Serial.println("Connecting WiFi...");
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi 连接成功！IP: " + WiFi.localIP().toString());
}

// ============================================================
//  服务器启动
// ============================================================
void serverStart() {
  server.on("/",          handleRoot);
  server.on("/data",      sendDataHandler);
  server.on("/status",    statusHandler);
  server.on("/control",   HTTP_GET,  controlHandler);
  server.on("/control",   HTTP_POST, controlHandler);
  server.on("/upload",    HTTP_GET,  uploadHandler);
  server.on("/ber_ack",   HTTP_GET,  berAckHandler);
  server.on("/ber_stat",  HTTP_GET,  berStatHandler);
  server.on("/ber_reset", HTTP_GET,  berResetHandler);
  server.begin();
  Serial.println("[Web] 服务器已启动（端口5000）");
}

// ============================================================
//  打印帮助
// ============================================================
void printHelp() {
  Serial.println("\n========== 串口命令 ==========");
  Serial.println("  直接输入任意字符串   → 发送该字符串（ASCII帧）");
  Serial.println("  LIGHT_ON/OFF         → 发送固定指令（短码帧）");
  Serial.println("  SERVO_ON/OFF         → 发送固定指令（短码帧）");
  Serial.println("  FAN_ON/OFF           → 发送固定指令（短码帧）");
  Serial.println("  stat                 → 查看误码率统计");
  Serial.println("  reset                → 重置误码率统计");
  Serial.println("  auto N CMD [interval]→ 自动发送CMD共N次，间隔Xms（默认500ms）");
  Serial.println("    例：auto 100 LIGHT_ON 300");
  Serial.println("  stop                 → 停止自动测试");
  Serial.println("  raw                  → 切换Raw模式（误码率测试，无帧结构）");
  Serial.println("  help                 → 显示本菜单");
  Serial.println("==============================\n");
}

// ============================================================
//  setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  wifiConnect();
  serverStart();

  Serial.println("========================");
  Serial.println(" VLC 发射端（调试版）");
  Serial.println("========================");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  printHelp();
}

// ============================================================
//  loop
// ============================================================
void loop() {
  server.handleClient();

  // 执行队列中的指令
  if (pendingCommand.length() > 0) {
    String cmd = pendingCommand;
    pendingCommand = "";
    applyCommand(cmd);
  }

  // 自动测试模式
  if (autoTestMode) {
    if (autoTestSent >= autoTestTotal) {
      autoTestMode = false;
      Serial.println("\n[自动测试] 完成！");
      printAutoTestProgress();
      // 打印最终统计
      uint32_t lost = (statTotalSent > statTotalRecv) ? (statTotalSent - statTotalRecv) : 0;
      float per = (statTotalSent > 0) ? (float)lost / statTotalSent * 100.0f : 0.0f;
      Serial.print("[自动测试] 丢帧率: "); Serial.print(per, 2); Serial.println("%");
      Serial.print("[自动测试] CRC失败: "); Serial.println(statCrcFail);
    } else if (millis() - lastAutoSend >= autoTestInterval) {
      lastAutoSend = millis();
      applyCommand(autoTestCmd);
      autoTestSent++;
      if (autoTestSent % 10 == 0) printAutoTestProgress();
    }
  }

  // 串口命令处理
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    String inputUpper = input;
    inputUpper.toUpperCase();

    // ---- 内置命令 ----
    if (inputUpper == "STAT") {
      uint32_t lost = (statTotalSent > statTotalRecv) ? (statTotalSent - statTotalRecv) : 0;
      float per     = (statTotalSent > 0) ? (float)lost / statTotalSent * 100.0f : 0.0f;
      float crcRate = (statTotalRecv > 0) ? (float)statCrcFail / statTotalRecv * 100.0f : 0.0f;
      Serial.println("\n===== 误码率统计 =====");
      Serial.print("总发送帧:    "); Serial.println(statTotalSent);
      Serial.print("接收端收到:  "); Serial.println(statTotalRecv);
      Serial.print("丢失帧:      "); Serial.println(lost);
      Serial.print("丢帧率:      "); Serial.print(per, 2); Serial.println("%");
      Serial.print("CRC失败帧:   "); Serial.println(statCrcFail);
      Serial.print("CRC失败率:   "); Serial.print(crcRate, 2); Serial.println("%");
      Serial.println("======================\n");
      return;
    }

    if (inputUpper == "RESET") {
      statTotalSent = statTotalRecv = statCrcFail = 0;
      Serial.println("[统计] 已重置");
      return;
    }

    if (inputUpper == "STOP") {
      autoTestMode = false;
      Serial.println("[自动测试] 已停止");
      return;
    }

    if (inputUpper == "HELP") { printHelp(); return; }

    if (inputUpper == "RAW") {
      rawMode = !rawMode;
      Serial.print("[Raw模式] ");
      Serial.println(rawMode ? "开启 — 只发原始数据位（无前导码/CRC/帧尾）" : "关闭 — 恢复完整帧");
      return;
    }

    // ---- 自动测试：auto N CMD [interval] ----
    if (inputUpper.startsWith("AUTO ")) {
      // 解析参数
      String params = input.substring(5);  // 去掉"auto "
      params.trim();
      int sp1 = params.indexOf(' ');
      if (sp1 < 0) { Serial.println("格式错误，例：auto 100 LIGHT_ON 300"); return; }

      autoTestTotal    = params.substring(0, sp1).toInt();
      String rest      = params.substring(sp1 + 1);
      rest.trim();
      int sp2          = rest.indexOf(' ');
      if (sp2 >= 0) {
        autoTestCmd      = rest.substring(0, sp2);
        autoTestInterval = rest.substring(sp2 + 1).toInt();
      } else {
        autoTestCmd      = rest;
        autoTestInterval = 500;
      }
      autoTestCmd.toUpperCase();
      autoTestSent  = 0;
      autoTestMode  = true;
      lastAutoSend  = 0;
      statTotalSent = statTotalRecv = statCrcFail = 0;  // 自动重置统计

      Serial.print("[自动测试] 开始，指令=");
      Serial.print(autoTestCmd);
      Serial.print(" 总次数="); Serial.print(autoTestTotal);
      Serial.print(" 间隔="); Serial.print(autoTestInterval); Serial.println("ms");
      return;
    }

    // ---- 普通发送：任意字符串 ----
    // raw模式保留原始大小写，正常模式转大写匹配指令
    if (rawMode) {
      input.trim();
      applyCommand(input);
    } else {
      inputUpper.trim();
      applyCommand(inputUpper);
    }

    Serial.flush();
    delay(200);
  }
}
