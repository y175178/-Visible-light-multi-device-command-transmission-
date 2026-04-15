#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

const int ledPin = 16;
const int BIT_PERIOD_MS = 10;    // 100bps
const int FRAME_REPEAT = 2;      // 每条指令重复发送次数
const int FRAME_GAP_MS = 12;     // 两帧之间间隔

void wifiConnect(void);
void serverStart(void);
void sendDataHandler(void);
void controlHandler(void);
void statusHandler(void);
void applyCommand(const String &command);

String data = "{'light':0,'temp':0,'hum':0}";
bool lightOn = false;
bool servoOn = false;
bool fanOn = false;

WebServer server(5000);

void sendBit(bool bit)
{
  digitalWrite(ledPin, bit ? HIGH : LOW);
  delay(BIT_PERIOD_MS);
}

void sendByte(uint8_t b)
{
  for (int i = 7; i >= 0; i--)
    sendBit((b >> i) & 1);
}

void sendMessage(const String &msg)
{
  for (int n = 0; n < FRAME_REPEAT; n++)
  {
    // 前导码
    for (int i = 0; i < 25; i++)
      sendBit(1);

    sendByte(msg.length());
    for (char c : msg)
      sendByte((uint8_t)c);
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
  server.begin();
  Serial.println("Web 服务器已启动");
}

void sendDataHandler()
{
  data = "{'light':1,'temp':30,'hum':60}";
  server.send(200, "text/plain", data);
}

void statusHandler(void)
{
  String status = "{\"light\":";
  status += (lightOn ? "1" : "0");
  status += ",\"servo\":";
  status += (servoOn ? "1" : "0");
  status += ",\"fan\":";
  status += (fanOn ? "1" : "0");
  status += "}";
  server.send(200, "application/json", status);
}

void applyCommand(const String &command)
{
  if (command == "LIGHT_ON")
    lightOn = true;
  else if (command == "LIGHT_OFF")
    lightOn = false;
  else if (command == "SERVO_ON")
    servoOn = true;
  else if (command == "SERVO_OFF")
    servoOn = false;
  else if (command == "FAN_ON")
    fanOn = true;
  else if (command == "FAN_OFF")
    fanOn = false;

  Serial.print("[发送] ");
  Serial.println(command);
  sendMessage(command);
  Serial.println("[发送完成]");
  Serial.println("----------------");
}

void controlHandler(void)
{
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

  applyCommand(command);

  String result = "{\"ok\":true,\"cmd\":\"";
  result += command;
  result += "\",\"light\":";
  result += (lightOn ? "1" : "0");
  result += ",\"servo\":";
  result += (servoOn ? "1" : "0");
  result += ",\"fan\":";
  result += (fanOn ? "1" : "0");
  result += "}";
  server.send(200, "application/json", result);
}

void setup()
{
  // ✅ 第一步必须开串口！！！（你之前放错位置）
  Serial.begin(9600);
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