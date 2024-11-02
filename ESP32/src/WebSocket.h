#include <Arduino.h>
#include "WebServer.h"

JsonDocument rdoc;
JsonDocument wDoc;
String DataStr = "";
String fbDataString = "";

static long startUpdateIntervalTime = millis();
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
//AP
void accessPointMode(){
  WiFi.mode(WIFI_AP);
  Serial.println("Configuring access point!");
  WiFi.softAP(ssid_AP,password_AP);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
}//accessPointMode
//Switch
void Switch(){
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    timeout++;
    Serial.print(".");
    if(timeout == 10){
      accessPointMode();
      timeout = 0;
      break;
    }
  }
}//Switch
//STA
void stationMode(){
  Serial.println();
  Serial.println();
  loadSetting();
  Serial.print("Connecting to ");
  Serial.println(settings.ssid);
  unsigned long start = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(settings.ssid,settings.pass);
  if(WiFi.status() != WL_CONNECTED)
  {
    Switch();
  }
  else {
    Serial.println(WiFi.localIP());
  }
    Serial.println(WiFi.localIP());
    server.begin();
}//stationMode
//Send data to web
void notifyClients(const String &value) {
  ws.textAll(value);
}
void sendKinematicsData()
{
    float theta1 = rdoc["Theta1"].as<float>();
    float theta2 = rdoc["Theta2"].as<float>();
    float theta3 = rdoc["Theta3"].as<float>();
    float theta4 = rdoc["Theta4"].as<float>();

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "theta1:%.2f,theta2:%.2f,theta3:%.2f,theta4:%.2f\n", theta1, theta2, theta3, theta4);
    Serial.println(buffer);
    mySerial.println(buffer);
}

void sendSetPointData()
{
    float theta1 = rdoc["Theta1"].as<float>();
    float theta2 = rdoc["Theta2"].as<float>();
    float theta3 = rdoc["Theta3"].as<float>();
    float theta4 = rdoc["Theta4"].as<float>();

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "NhaT1:%.2f,NhaT2:%.2f,NhaT3:%.2f,NhaT4:%.2f\n", theta1, theta2, theta3, theta4);
    Serial.println(buffer);
    mySerial.println(buffer);
}

void sendSelectPointData()
{
    JsonArray dataArray = rdoc["Data"].as<JsonArray>();
    char allPoints[2048] = ""; 

    for (JsonObject pointObj : dataArray)
    {
        uint8_t point = pointObj["Point"];
        float theta1 = pointObj["Theta1"];
        float theta2 = pointObj["Theta2"];
        float theta3 = pointObj["Theta3"];
        float theta4 = pointObj["Theta4"];

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Point:%d, HutT1:%.2f, HutT2:%.2f, HutT3:%.2f, HutT4:%.2f;", point, theta1, theta2, theta3, theta4);
        strncat(allPoints, buffer, sizeof(allPoints) - strlen(allPoints) - 1); 
    }
    
    Serial.println("Sending Points: " + String(allPoints));
    mySerial.println(allPoints); 
}
//Receive data from websocket
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    const size_t max_buffer_size = 1024; 

    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        if (len < max_buffer_size) {
            data[len] = '\0'; 
        }

        String DataStr = String((char*)data);  
        DeserializationError error = deserializeJson(rdoc, DataStr);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        if (rdoc.containsKey("Command")) {
            String command = rdoc["Command"].as<String>();

            if (command == "forwardkinematics" || command == "inversekinematics")
            {
                sendKinematicsData();
            }
            else if (command == "startprogram")
            {
                mySerial.println("start\n");
            }
            else if (command == "Reset")
            {
                mySerial.println("Reset\n");
                Serial.println("Reset\n");
            }
            else if (command == "SetPoint")
            {
                sendSetPointData();
            }
            else if (command == "SelectPoint")
            {
                sendSelectPointData();
            }
            else if(command == "SetHome"){
              mySerial.println("home\n");
              Serial.println("home\n");
            }
        }
    }
}



//onEvent
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      char buffer[64];
      sprintf(buffer, "disconnected\n");
      Serial.println(buffer);
      mySerial.println(buffer);
      memset(buffer, 0, sizeof(buffer));  // Clear buffer after sending

      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}
//initwebsocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
void WB_setup(){  
  initWebSocket();
}

void WB_loop() {
  ws.cleanupClients();
}
void TaskFunction(void *pvParameter){
  
  for(;;){
    WB_loop();
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
} 