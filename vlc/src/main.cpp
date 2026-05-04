#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

const int ledPin = 16;
const int BIT_PERIOD_US = 1000; // 1000us = 1kbps
const int FRAME_REPEAT = 2;     // 每条指令重复发送次数
const int FRAME_GAP_MS = 12;    // 两帧之间间隔

// ===== 定长3bit短码码表 =====
struct ShortEntry {
  const char* cmd;
  uint8_t     code;  // 1~6
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
// ===== 新增：CRC-8 计算（多项式 0x07）=====
uint8_t crc8(uint8_t *data, uint8_t len) {
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

void wifiConnect(void);
void serverStart(void);
void sendDataHandler(void);
void controlHandler(void);
void statusHandler(void);
void uploadHandler(void);
void applyCommand(const String &command);
void queueCommand(const String &command);

String expectedLightState = "0";   // 预期状态
String expectedServoState = "0";
String expectedFanState = "0";
unsigned long lastCommandTime = 0;  // 最后命令发送时间

// 接收端确认的实际状态（由 /upload 更新）
bool lightOn = false;
bool servoOn = false;
bool fanOn = false;
String pendingCommand = "";
String data = "{'light':0,'temp':0,'hum':0}";  // 传感器数据

WebServer server(5000);

void sendBit(bool bit)
{
  if (bit)
  {
    // 曼切斯特编码：1 = 高→低
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(BIT_PERIOD_US / 2);
    digitalWrite(ledPin, LOW);
    delayMicroseconds(BIT_PERIOD_US / 2);
  }
  else
  {
    // 曼切斯特编码：0 = 低→高
    digitalWrite(ledPin, LOW);
    delayMicroseconds(BIT_PERIOD_US / 2);
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(BIT_PERIOD_US / 2);
  }
}

void sendByte(uint8_t b)
{
  for (int i = 7; i >= 0; i--)
    sendBit((b >> i) & 1);
}

// ===== 发送霍夫曼编码帧 =====
// ===== 发送短码帧（含CRC）=====
void sendShortMessage(const String &cmd) {
  int idx = -1;
  for (int i = 0; i < SHORT_TABLE_SIZE; i++) {
    if (cmd == SHORT_TABLE[i].cmd) { idx = i; break; }
  }
  if (idx < 0) { Serial.println("[Short] 未知指令"); return; }

  uint8_t code = SHORT_TABLE[idx].code;

  // CRC-8校验（只对code这1字节）
  uint8_t crc = crc8(&code, 1);

  Serial.print("[Short] 发送 ");
  Serial.print(cmd);
  Serial.print(" -> code:");
  Serial.print(code);
  Serial.print(" CRC:");
  Serial.println(crc, HEX);

  for (int n = 0; n < FRAME_REPEAT; n++) {
    // 前导码
    for (int i = 0; i < 25; i++) sendBit(1);

    // 帧类型：0xBB 表示定长短码帧
    sendByte(0xBB);

    // 指令码（整字节发送，简单！）
    sendByte(code);

    // CRC
    sendByte(crc);

    // 帧尾
    sendByte(0xFF);

    delay(FRAME_GAP_MS);
  }
  digitalWrite(ledPin, LOW);
}

void wifiConnect(void)
{
  const char *ssid = "打扫干净屋子再请客";
  const char *password = "1751787761";

  Serial.println("Connecting WiFi...");
  WiFi.setSleep(false); // 降低WiFi省电造成的时序抖动
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi 连接成功！");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void serverStart(void)
{
  server.on("/data", sendDataHandler);
  server.on("/status", statusHandler);
  server.on("/control", HTTP_POST, controlHandler);
  server.on("/control", HTTP_GET, controlHandler);
  server.on("/upload", HTTP_GET, uploadHandler);
  server.begin();
  Serial.println("Web 服务器已启动");
}

void addCorsHeaders()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendDataHandler()
{
  data = "{'light':1,'temp':30,'hum':60}";
  addCorsHeaders();
  server.send(200, "text/plain", data);
}

void statusHandler(void)
{
  addCorsHeaders();
  
  // 计算自上次命令以来的时间（毫秒）
  unsigned long timeSinceCommand = (lastCommandTime > 0) ? (millis() - lastCommandTime) : 999999;
  bool isWaitingFeedback = (timeSinceCommand < 2000); // 2秒内视为等待反馈
  
  String status = "{";
  status += "\"confirmed_state\":{";
  status += "\"light\":";
  status += (lightOn ? "1" : "0");
  status += ",\"servo\":";
  status += (servoOn ? "1" : "0");
  status += ",\"fan\":";
  status += (fanOn ? "1" : "0");
  status += "},";
  
  status += "\"expected_state\":{";
  status += "\"light\":";
  status += expectedLightState;
  status += ",\"servo\":";
  status += expectedServoState;
  status += ",\"fan\":";
  status += expectedFanState;
  status += "},";
  
  status += "\"sync_status\":\"";
  status += isWaitingFeedback ? "waiting_feedback" : "ready";
  status += "\",";
  status += "\"time_since_command\":";
  status += String(timeSinceCommand);
  status += "}";

  server.send(200, "application/json", status);
}

void applyCommand(const String &command)
{
  Serial.print("[发送] ");
  Serial.println(command);
  
  // 根据命令更新预期状态
  if (command == "LIGHT_ON") {
    expectedLightState = "1";
  } else if (command == "LIGHT_OFF") {
    expectedLightState = "0";
  } else if (command == "SERVO_ON") {
    expectedServoState = "1";
  } else if (command == "SERVO_OFF") {
    expectedServoState = "0";
  } else if (command == "FAN_ON") {
    expectedFanState = "1";
  } else if (command == "FAN_OFF") {
    expectedFanState = "0";
  }
  
  lastCommandTime = millis();
  sendShortMessage(command);
  Serial.println("[发送完成]");
  Serial.println("----------------");
}

void controlHandler(void)
{
  addCorsHeaders();
  String command = server.arg("cmd");
  command.trim();
  command.toUpperCase();

  if (command.length() == 0)
  {
    server.send(400, "application/json", "{\"ok\":false,\"message\":\"missing cmd\"}");
    return;
  }

  if (command != "LIGHT_ON" && command != "LIGHT_OFF" &&
      command != "SERVO_ON" && command != "SERVO_OFF" &&
      command != "FAN_ON" && command != "FAN_OFF")
  {
    server.send(400, "application/json", "{\"ok\":false,\"message\":\"invalid cmd\"}");
    return;
  }

  // 根据命令解析预期的设备状态
  String device = "";
  int expectedState = 0;
  if (command == "LIGHT_ON" || command == "LIGHT_OFF") {
    device = "LIGHT";
    expectedState = (command == "LIGHT_ON") ? 1 : 0;
  } else if (command == "SERVO_ON" || command == "SERVO_OFF") {
    device = "SERVO";
    expectedState = (command == "SERVO_ON") ? 1 : 0;
  } else if (command == "FAN_ON" || command == "FAN_OFF") {
    device = "FAN";
    expectedState = (command == "FAN_ON") ? 1 : 0;
  }

  // 返回响应：包含命令、预期状态、和当前确认状态
  String result = "{\"ok\":true,\"cmd\":\"";
  result += command;
  result += "\",\"device\":\"";
  result += device;
  result += "\",\"expected_state\":";
  result += expectedState;
  result += ",\"confirmed_light\":";
  result += (lightOn ? "1" : "0");
  result += ",\"confirmed_servo\":";
  result += (servoOn ? "1" : "0");
  result += ",\"confirmed_fan\":";
  result += (fanOn ? "1" : "0");
  result += ",\"status\":\"command_sent_waiting_feedback\"}";
  server.send(200, "application/json", result);

  // 先响应HTTP，避免App在发光发送期间超时
  queueCommand(command);
}

void uploadHandler(void)
{
  addCorsHeaders();
  String device = server.arg("device");
  String stateStr = server.arg("state");
  device.trim();
  device.toUpperCase();
  
  int state = stateStr.toInt();
  
  // 更新对应设备的状态
  if (device == "LIGHT")
    lightOn = (state == 1);
  else if (device == "SERVO")
    servoOn = (state == 1);
  else if (device == "FAN")
    fanOn = (state == 1);
  
  Serial.print("[接收端上报] ");
  Serial.print(device);
  Serial.print(" 状态: ");
  Serial.print(state);
  Serial.print(" -> 已更新为: ");
  Serial.println(state == 1 ? "ON" : "OFF");
  
  String result = "{\"ok\":true,\"device\":\"";
  result += device;
  result += "\",\"state\":";
  result += state;
  result += "}";
  server.send(200, "application/json", result);
}

void queueCommand(const String &command)
{
  pendingCommand = command;
}

void setup()
{
  // ✅ 第一步必须开串口！！！（你之前放错位置）
  Serial.begin(115200);
  delay(100);

  wifiConnect();
  serverStart();

  Serial.println("========================");
  Serial.println(" VLC 发射端已准备就绪");
  Serial.println(" 请在串口输入指令发送");
  Serial.println("========================");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void loop()
{
  server.handleClient();

  if (pendingCommand.length() > 0)
  {
    String commandToSend = pendingCommand;
    pendingCommand = "";
    applyCommand(commandToSend);
  }

  if (Serial.available())
  {
    // ✅ 读取并清空缓存
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command.length() > 0)
    {
      applyCommand(command);

      // 清空串口，防止重复发送
      Serial.flush();
      delay(500);
    }
  }
}