// ==================== VLC 接收端（按角色）====================
// 用法：
// 1) 修改 DEVICE_ROLE: 1=灯 2=舵机 3=风扇
// 2) 每块接收端烧录一份，分别对应三个设备
// 3) App 通过 /ack 读取 lastCmd + state 更新开关状态

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const int signalPin = A0;         // 改为A0模拟输入
const int outPin = D1;            // NodeMCU D1 作为继电器/驱动输出，可按接线更改
const int BIT_PERIOD_US = 1000;   // 改为1kbps (1000us = 1kbps)
const int SAMPLE_COUNT = 50;      // 采样次数
const int SAMPLE_DELAY_US = 4;    // 采样间隔
const int SAMPLE_HIGH_COUNT = 25; // 需要50%采样 > 阈值
int adcThreshold = 13; // 默认阈值

// 1=LIGHT, 2=SERVO, 3=FAN
const int DEVICE_ROLE = 2;

const char *ssid = "打扫干净屋子再请客";
const char *password = "1751787761";

ESP8266WebServer server(80);

bool deviceOn = false;
String lastCommand = "";
String receivedData = "等待接收数据...";
unsigned long lastAckMs = 0;
bool debugSignal = false;
bool continuousDebug = false;

// WiFi上传相关
bool wifiUploadPending = false;
bool wifiUploading = false;
const char *txHost = "192.168.54.52";  // 发射端IP
const int txPort = 5000;               // 发射端端口
unsigned long lastUploadTime = 0;
const unsigned long UPLOAD_COOLDOWN = 100;  // 100ms冷却时间

void autoCheckSignalPin()
{
  Serial.println("\n========== 开始 A0 输入检查 ==========");
  Serial.println("第1步：检查 A0 模拟输入状态...");
  delay(2000);

  const int sampleCount = 100;
  int minVal = 1023;
  int maxVal = 0;
  int sum = 0;
  for (int i = 0; i < sampleCount; i++)
  {
    int val = analogRead(signalPin);
    minVal = min(minVal, val);
    maxVal = max(maxVal, val);
    sum += val;
    delay(10);
  }
  int avgVal = sum / sampleCount;

  Serial.print("A0 输入采样结果: 最小=");
  Serial.print(minVal);
  Serial.print(" 平均=");
  Serial.print(avgVal);
  Serial.print(" 最大=");
  Serial.println(maxVal);

  if (maxVal - minVal < 50)
  {
    Serial.println("⚠️ A0 值变化太小，请检查 LM358 输出、反馈电阻和接线");
  }
  else
  {
    Serial.println("✅ A0 存在足够变化，输入信号正常");
  }
  Serial.println("========== 校准完成 ==========\n");
}

enum RxState
{
  WAIT_PREAMBLE,
  READ_LEN,
  READ_PAYLOAD,
  READ_END
};

RxState rxState = WAIT_PREAMBLE;
uint32_t shiftReg = 0;
uint8_t rxByte = 0;
int rxBitCount = 0;
int expectedLen = 0;
String rxBuffer = "";

const uint32_t PREAMBLE_MASK = 0x01FFFFFF;  // 25bit
const uint32_t PREAMBLE_VALUE = 0x01FFFFFF; // 全1

String roleName()
{
  if (DEVICE_ROLE == 1)
    return "LIGHT";
  if (DEVICE_ROLE == 2)
    return "SERVO";
  return "FAN";
}

bool commandMatchRole(const String &cmd)
{
  String role = roleName();
  return cmd == role + "_ON" || cmd == role + "_OFF";
}

void applyCommand(const String &cmd)
{
  if (!commandMatchRole(cmd))
    return;

  deviceOn = cmd.endsWith("_ON");
  digitalWrite(outPin, deviceOn ? HIGH : LOW);
  lastCommand = cmd;
  receivedData = cmd;
  lastAckMs = millis();

  Serial.print("执行指令: ");
  Serial.println(cmd);
  
  // 设置WiFi上传标志
  wifiUploadPending = true;
}

void resetRx()
{
  rxState = WAIT_PREAMBLE;
  shiftReg = 0;
  rxByte = 0;
  rxBitCount = 0;
  expectedLen = 0;
  rxBuffer = "";
}

void processByte(uint8_t b)
{
  if (rxState == READ_LEN)
  {
    expectedLen = (int)b;
    if (expectedLen <= 0 || expectedLen > 64)
    {
      resetRx();
      return;
    }
    rxState = READ_PAYLOAD;
    return;
  }

  if (rxState == READ_PAYLOAD)
  {
    rxBuffer += (char)b;
    if ((int)rxBuffer.length() >= expectedLen)
      rxState = READ_END;
    return;
  }

  if (rxState == READ_END)
  {
    if (b == 0xFF)
      applyCommand(rxBuffer);
    resetRx();
  }
}

void sendCorsHeaders()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleAck()
{
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

void handleRoot()
{
  sendCorsHeaders();
  String html = "<html><head><meta charset='utf-8'></head><body style='text-align:center;margin-top:40px'>";
  html += "<h2>VLC接收端</h2>";
  html += "<h3>ROLE: " + roleName() + "</h3>";
  html += "<h3>STATE: " + String(deviceOn ? "ON" : "OFF") + "</h3>";
  html += "<h3>LAST: " + receivedData + "</h3>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void serverStart()
{
  server.on("/", handleRoot);
  server.on("/ack", handleAck);
  server.begin();
  Serial.println("接收端Web服务器已启动 (端口80)");
}

void uploadStateToTx()
{
  if (wifiUploading) return;  // 防止重复上传
  if ((millis() - lastUploadTime) < UPLOAD_COOLDOWN) return;  // 冷却时间检查
  
  wifiUploading = true;
  lastUploadTime = millis();
  
  Serial.println("[WiFi] 启动上传...");
  
  // 唤醒WiFi并上传数据
  WiFi.forceSleepWake();
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  // 快速连接，最多等待20秒
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 20000)
  {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("[WiFi] 已连接");

    // 上传状态到网页
    WiFiClient client;
    if (client.connect(txHost, txPort))
    {
      String path = "/upload?device=" + roleName() + "&state=" + (deviceOn ? "1" : "0");
      client.print("GET " + path + " HTTP/1.1\r\n");
      client.print("Host: ");
      client.print(txHost);
      client.print(":" + String(txPort) + "\r\n");
      client.print("Connection: close\r\n\r\n");
      delay(300); // 等待响应
      client.stop();
      Serial.println("[WiFi] 状态已上报");
    }
    else
    {
      Serial.println("[WiFi] 连接发射端失败");
    }
  }
  else
  {
    Serial.println("[WiFi] 连接SSID失败");
  }

  // 关闭WiFi
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  Serial.println("[WiFi] 已关闭");
  wifiUploading = false;
}

// 确保WiFi在接收数据前完全关闭
void ensureWiFiOff() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(50);
  Serial.println("[WiFi] 已关闭");
}

void setThreshold() {
  Serial.println("请输入新的阈值（0-1023）：");
  while (!Serial.available()) {
    delay(100);
  }
  String input = Serial.readStringUntil('\n');
  int newThreshold = input.toInt();
  if (newThreshold >= 0 && newThreshold <= 1023) {
    adcThreshold = newThreshold;
    Serial.print("新的阈值已设置为：");
    Serial.println(adcThreshold);
  } else {
    Serial.println("输入无效，保持原阈值。");
  }
}

void setup()
{
  Serial.begin(9600);
  delay(100);
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);

  ensureWiFiOff(); // 确保WiFi关闭

  // 启动Web服务器
  serverStart();

  delay(500);
  Serial.println("接收端启动 (WiFi待命模式，A0模拟输入模式)");
  Serial.print("ROLE = ");
  Serial.println(roleName());
  Serial.println("观察模式：A0 模拟输入，位速率1kbps");
  Serial.println("开始信号输入检查...");
  autoCheckSignalPin();

  setThreshold(); // 手动设置阈值
}

void loop()
{
  // 处理HTTP请求（非阻塞）
  server.handleClient();
  
  // 检查是否需要上传状态（在不影响信号接收的时候执行）
  if (wifiUploadPending && !wifiUploading)
  {
    wifiUploadPending = false;
    // 异步执行上传，不阻塞主loop
    uploadStateToTx();
  }

  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "d1")
    {
      debugSignal = true;
      continuousDebug = false;
      Serial.println("✓ A0 信号调试已开启 (每300ms打印一次)");
    }
    else if (s == "d2")
    {
      debugSignal = true;
      continuousDebug = true;
      Serial.println("✓ A0 连续打印已开启 (每1ms打印一次)");
    }
    else if (s == "d0")
    {
      debugSignal = false;
      continuousDebug = false;
      Serial.println("✓ A0 信号调试已关闭");
    }
    else if (s == "auto" || s == "check")
    {
      autoCheckSignalPin();
    }
    else if (s == "info")
    {
      Serial.println("\n========== 诊断信息 ==========");
      Serial.print("当前 A0 输入: ");
      Serial.println(analogRead(signalPin));
      Serial.print("设备状态: ");
      Serial.println(deviceOn ? "ON" : "OFF");
      Serial.print("上次命令: ");
      Serial.println(lastCommand);
      Serial.println("========== 快速测试 ==========");
      Serial.println("输入命令:");
      Serial.println("  d1          - 开启A0信号调试（每300ms打印一次）");
      Serial.println("  d2          - 开启A0连续打印（每1ms打印一次，观测LM358输出）");
      Serial.println("  d0          - 关闭A0信号监控");
      Serial.println("  auto 或 check - 检查A0模拟输入状态");
      Serial.println("  info        - 显示本菜单");
      Serial.println("=========================================");
    }
    else
    {
      Serial.println("\n未知命令。输入 'info' 获取帮助\n");
    }
  }

  if (debugSignal)
  {
    if (continuousDebug)
    {
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 1)
      {
        lastPrint = millis();
        Serial.println(analogRead(signalPin));
      }
    }
    else
    {
      static unsigned long lastDbg = 0;
      if (millis() - lastDbg >= 300)
      {
        lastDbg = millis();
        int val = analogRead(signalPin);
        Serial.print("A0=");
        Serial.print(val);
        Serial.print(" | state=");
        Serial.println(deviceOn ? "ON" : "OFF");
      }
    }
  }

  // 使用micros()实现高精度1000us周期
  static unsigned long lastSample = 0;
  unsigned long now = micros();
  if (now - lastSample < BIT_PERIOD_US)
    return;
  lastSample = now;

  // 曼切斯特解码：采样两个半周期
  int val1 = analogRead(signalPin); // 第一个半周期
  delayMicroseconds(BIT_PERIOD_US / 2);
  int val2 = analogRead(signalPin); // 第二个半周期

  bool bit;
  if (val1 > adcThreshold && val2 <= adcThreshold)
    bit = 1; // 高→低 = 1
  else if (val1 <= adcThreshold && val2 > adcThreshold)
    bit = 0; // 低→高 = 0
  else
    bit = 0; // 其他情况，默认为0

  if (rxState == WAIT_PREAMBLE)
  {
    shiftReg = ((shiftReg << 1) | (bit ? 1 : 0)) & PREAMBLE_MASK;
    if (shiftReg == PREAMBLE_VALUE)
    {
      rxState = READ_LEN;
      rxBitCount = 0;
      rxByte = 0;
      expectedLen = 0;
      rxBuffer = "";
    }
    return;
  }

  rxByte = (rxByte << 1) | (bit ? 1 : 0);
  rxBitCount++;
  if (rxBitCount == 8)
  {
    processByte(rxByte);
    rxBitCount = 0;
    rxByte = 0;
  }
}
