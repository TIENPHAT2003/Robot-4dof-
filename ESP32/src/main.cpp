#define LOG(x) {Serial.print(x);}
#define LOGLN(x) {Serial.println(x);}

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SoftwareSerial.h"

SoftwareSerial mySerial(16,17);
void TaskCaptivePortal(void *pvParameter);
void TaskFunction(void *pvParameter);

#define MAX_CLIENTS 4
#define WIFI_CHANNEL 6

const IPAddress localIP(192, 168, 4, 1);		
const IPAddress gatewayIP(192, 168, 4, 1);	   
const IPAddress subnetMask(255, 255, 255, 0);
const String localIPURL = "http://192.168.4.1";

struct Settings
{
  String ssid;
  String pass;
  String staticip;
  String waddress;
  String wgetway;
  String wsubnet;
  String wifimode;
  String chanel;
}settings;

// AP Mode
const char* ssid_AP = "NODE_IOT";
const char* password_AP = "123456789";

short timeout = 0;
bool isConnected = false;
DNSServer dnsServer;
#include "WebSocket.h"
String header;
String HTML = "\
<!DOCTYPE HTML>\
<html>\
<head>\
  <title>WIFI SETTING</title>\
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
  <link rel=\"icon\" href=\"data:,\">\
  <style>\
  html {\
    font-family: Arial, Helvetica, sans-serif;\
    text-align: center;\
  }\
  h1 {\
    font-size: 1.8rem;\
    color: white;\
  }\
  h2{\
    font-size: 1.5rem;\
    font-weight: bold;\
    color: #143642;\
  }\
  .topnav {\
    overflow: hidden;\
    background-color: #143642;\
  }\
  body {\
    margin: 0;\
  }\
  .content {\
    padding: 30px;\
    max-width: 600px;\
    margin: 0 auto;\
  }\
  .card {\
    background-color: #F8F7F9;;\
    box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);\
    padding-top:10px;\
    padding-bottom:20px;\
  }\
  .button {\
    padding: 15px 50px;\
    font-size: 24px;\
    text-align: center;\
    outline: none;\
    color: #fff;\
    background-color: #0f8b8d;\
    border: none;\
    border-radius: 5px;\
    -webkit-touch-callout: none;\
    -webkit-user-select: none;\
    -khtml-user-select: none;\
    -moz-user-select: none;\
    -ms-user-select: none;\
    user-select: none;\
    -webkit-tap-highlight-color: rgba(0,0,0,0);\
   }\
   .button:active {\
     background-color: #0f8b8d;\
     box-shadow: 2 2px #CDCDCD;\
     transform: translateY(2px);\
   }\
   .state {\
     font-size: 1.5rem;\
     color:#8c8c8c;\
     font-weight: bold;\
   }\
  </style>\
  <title>WIFI SETTING</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<link rel=\"icon\" href=\"data:,\">\
</head>\
<body>\
  <div class=\"topnav\">\
    <h1>WIFI SETTING</h1>\
  </div>\
  <div class=\"content\">\
    <div class=\"card\">\
    <span><label for=\"fname\">SSID:</label>\
    <input type=\"text\" id=\"input_ssid\" placeholder=\"Wifi Name\"></span><br><br>\
    <span><label for=\"lname\">PASSWORD:</label>\
    <input type=\"text\" id=\"input_pass\" placeholder=\"Wifi Pass\"></span><br><br>\
    <span><button id=\"buttonsave\" class=\"button\">SAVE</button></span>\
    </div>\
  </div>\
  <div class=\"content\">\
    <div class=\"card\">\
      <h2>LED STATUS</h2>\
      <p class=\"state\">STATE: <span id=\"state\">STATE</span></p>\
      <p><button id=\"buttontoggle\" class=\"button\">TOGGLE</button></p>\
    </div>\
  </div>\
<script>\
  var gateway = `ws://${window.location.hostname}/ws`;\
  var websocket;\
  function initWebSocket() {\
    console.log('Trying to open a WebSocket connection...');\
    websocket = new WebSocket(gateway);\
    websocket.onopen    = onOpen;\
    websocket.onclose   = onClose;\
    websocket.onmessage = onMessage; \
  }\
  function onOpen(event) {\
    console.log('Connection opened');\
  }\
  function onClose(event) {\
    console.log('Connection closed');\
    setTimeout(initWebSocket, 2000);\
  }\
  function onMessage(event) {\
    var state;\
    if (event.data == \"1\"){\
      state = \"ON\";\
    }\
    else{\
      state = \"OFF\";\
    }\
    document.getElementById('state').innerHTML = state;\
  }\
  window.addEventListener('load', onLoad);\
  function onLoad(event) {\
    initWebSocket();\
    initButton();\
  }\
  function initButton() {\
    document.getElementById('buttontoggle').addEventListener('click', toggle);\
    document.getElementById('buttonsave').addEventListener('click', save);\
  }\
  function toggle(){\
    websocket.send('toggle');\
  }\
  function save(){\
    var ssid_input = document.getElementById('input_ssid').value;\
    var pass_input = document.getElementById('input_pass').value;\
    if(ssid_input == \"\" && pass_input == \"\"){\
      console.log(\"Fail SSID và PASS\");\
      alert(\"Fail SSID and PASS\");\
    }\
    else if(ssid_input == \"\"){\
      console.log(\"SSID Fail\");\
      alert(\"SSID Fail\");\
    }\
    else if(pass_input == \"\"){\
      console.log(\"Pass Fail\");\
      alert(\"Pass Fail\");\
    }\
    else{\
      var json_output = \"{'SSID':'\" + ssid_input + \"','PASS':'\" + pass_input +\"'}\";\
      console.log(json_output);\
      websocket.send(json_output);\
      alert(\"Completed setting!!!\");\
    }\
  }\
</script>\
</body>\
</html>\
";

const char* SSID_ = "ssid";
const char* PASS_ = "pass";

void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
	dnsServer.setTTL(3600);
	dnsServer.start(53, "*", localIP);
}

void WebInit(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", HTML);
  });
}//WebInit
void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP) {

	server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });
	server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });								

	server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });		  
	server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			  
	server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });  
	server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });	  
	server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					   
	server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			  

	server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });	

	server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", HTML);
    // AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html", String(), false, processor);
		response->addHeader("Cache-Control", "public,max-age=31536000");  
		request->send(response);
		Serial.println("Served Basic HTML Page");
	});

	// the catch all
	server.onNotFound([](AsyncWebServerRequest *request) {
		request->redirect(localIPURL);
		Serial.print("onnotfound ");
		Serial.print(request->host());
		Serial.print(" ");
		Serial.print(request->url());
		Serial.print(" sent redirect to " + localIPURL + "\n");
	});
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  SPIFFS_Init();
  stationMode();
  setUpDNSServer(dnsServer, WiFi.softAPIP());
  setUpWebserver(server, WiFi.softAPIP());
  WebInit();
  WB_setup();
  xTaskCreatePinnedToCore(TaskCaptivePortal, "CaptivePortal",30000,NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskFunction, "Function", 30000, NULL, 2,NULL, 1);
}
void loop(){
  if (mySerial.available()) {
    String data = mySerial.readStringUntil('\n'); // Đọc cho đến khi gặp ký tự xuống dòng
    Serial.println("Received Data: " + data);     // Hiển thị dữ liệu trên Serial monitor

    // Tách và phân tích dữ liệu nếu cần
    float t1, t2, t3, t4;
    if (sscanf(data.c_str(), "t1:%f,t2:%f,t3:%f,t4:%f", &t1, &t2, &t3, &t4) == 4) {
      Serial.printf("Angle t1: %.1f, t2: %.1f, t3: %.1f, t4: %.1f\n", t1, t2, t3, t4);
    } else {
      Serial.println("Data format error.");
    }
  }
}

void TaskCaptivePortal(void *pvParameter){
  for(;;){
    // dnsServer.processNextRequest();
    vTaskDelay(30/portTICK_PERIOD_MS);
  } 
}

