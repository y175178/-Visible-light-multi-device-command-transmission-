// ==================== VLC 接收端（按角色）====================
// 用法：
// 1) 修改 DEVICE_ROLE: 1=灯 2=舵机 3=风扇
// 2) 每块接收端烧录一份，分别对应三个设备
// 3) App 通过 /ack 读取 lastCmd + state 更新开关状态

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const int adcPin = A0;
const int outPin = D1; // 继电器或驱动输入引脚，可按接线改
const int BIT_PERIOD_MS = 10;       // 与发射端保持一致(100bps)
const int SAMPLE_COUNT = 5;         // 每bit采样次数
const int SAMPLE_DELAY_MS = 1;      // 单次采样间隔
const int SAMPLE_HIGH_COUNT = 3;    // 判定为1的最小高电平次数

// 1=LIGHT, 2=SERVO, 3=FAN
const int DEVICE_ROLE = 1;

const char *ssid = "打扫干净屋子再请客";
const char *password = "1751787761";

ESP8266WebServer server(80);

int threshold = 2;
bool deviceOn = false;
String lastCommand = "";
String receivedData = "等待接收数据...";
unsigned long lastAckMs = 0;
bool debugAdc = false;

void autoCalibrateThreshold()
{
  int minV = 1024;
  int maxV = 0;
  for (int i = 0; i < 400; i++)
  {
    int v = analogRead(adcPin);
    if (v < minV)
      minV = v;
    if (v > maxV)
      maxV = v;
    delay(2);
  }
  int range = maxV - minV;
  if (range < 15)
  {
    // 动态范围太小，说明光信号变化不足，给一个保守阈值并提示用户
    threshold = minV + 8;
    if (threshold < 5)
      threshold = 5;
    if (threshold > 900)
      threshold = 900;
    Serial.println("warning: 光信号变化太小，请缩短距离/对准发射灯/降低环境光");
  }
  else
  {
    // 偏向高电平一侧，降低噪声误判为1
    threshold = minV + (range * 65) / 100;
  }
  Serial.print("auto threshold=");
  Serial.print(threshold);
  Serial.print(" (min=");
  Serial.print(minV);
  Serial.print(", max=");
  Serial.print(maxV);
  Serial.print(", range=");
  Serial.print(range);
  Serial.println(")");
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

void handleAck()
{
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
  String html = "<html><head><meta charset='utf-8'><meta http-equiv='refresh' content='1'></head><body style='text-align:center;margin-top:40px'>";
  html += "<h2>VLC接收端</h2>";
  html += "<h3>ROLE: " + roleName() + "</h3>";
  html += "<h3>THRESHOLD: " + String(threshold) + "</h3>";
  html += "<h3>LAST: " + receivedData + "</h3>";
  html += "<h3>STATE: " + String(deviceOn ? "ON" : "OFF") + "</h3>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup()
{
  Serial.begin(9600);
  delay(100);
  pinMode(adcPin, INPUT);
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);

  WiFi.begin(ssid, password);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // WiFi常开，减小采样抖动
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  server.on("/", handleRoot);
  server.on("/ack", handleAck);
  server.begin();

  Serial.println("接收端启动");
  Serial.print("ROLE = ");
  Serial.println(roleName());
  Serial.print("IP = ");
  Serial.println(WiFi.localIP());
  Serial.println("开始自动阈值校准...");
  autoCalibrateThreshold();
}

void loop()
{
  server.handleClient();

  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.startsWith("t "))
    {
      threshold = s.substring(2).toInt();
      Serial.print("threshold=");
      Serial.println(threshold);
    }
    else if (s == "d1")
    {
      debugAdc = true;
      Serial.println("ADC调试已开启");
    }
    else if (s == "d0")
    {
      debugAdc = false;
      Serial.println("ADC调试已关闭");
    }
    else if (s == "auto")
    {
      autoCalibrateThreshold();
    }
  }

  if (debugAdc)
  {
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg >= 300)
    {
      lastDbg = millis();
      int adc = analogRead(adcPin);
      Serial.print("ADC=");
      Serial.print(adc);
      Serial.print(" threshold=");
      Serial.println(threshold);
    }
  }

  static unsigned long lastSample = 0;
  if (millis() - lastSample < BIT_PERIOD_MS)
    return;
  lastSample = millis();

  int onCount = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    if (analogRead(adcPin) > threshold)
      onCount++;
    delay(SAMPLE_DELAY_MS);
  }
  bool bit = (onCount >= SAMPLE_HIGH_COUNT);

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
