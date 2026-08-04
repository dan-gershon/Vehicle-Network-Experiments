#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

const char* ssid = "**********";
const char* password = "**********";

WebServer server(80);
BMI270 imu;

const int CAN_TX_PIN = 13;
const float CRASH_THRESHOLD = 2.0;

float currentX = 0.0;
bool crashDetected = false;
unsigned long crashEndTime = 0;

// ==========================================
// ממשק המשתמש (HTML + CSS + JavaScript)
// ==========================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>APoloPulse Dashboard</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; margin: 0; padding: 20px; background-color: #121212; color: #ffffff; direction: rtl;}
    .card { background: #1e1e1e; padding: 30px; border-radius: 15px; display: inline-block; box-shadow: 0 8px 16px rgba(0,0,0,0.8); margin-top: 50px; border: 1px solid #333;}
    h1 { color: #00ffcc; margin-bottom: 5px;}
    .data { font-size: 4rem; margin: 20px; font-weight: bold; color: #03dac6;}
    .status { font-size: 1.5rem; font-weight: bold; padding: 15px; border-radius: 8px; transition: 0.3s; }
    .normal { background-color: #2e7d32; color: white; border: 2px solid #1b5e20;}
    .crash { background-color: #d32f2f; color: white; border: 2px solid #b71c1c; animation: blinker 0.4s linear infinite; }
    @keyframes blinker { 50% { opacity: 0.3; } }
  </style>
</head>
<body>
  <div class="card">
    <h1>לוח בקרה APoloPulse</h1>
    <p>תאוצה נוכחית (ציר X):</p>
    <div class="data" id="x_val">0.00g</div>
    <div class="status normal" id="status_box">סטטוס: רשת CAN מנוחה</div>
  </div>
  <script>
    setInterval(function() {
      fetch('/data').then(response => response.json()).then(data => {
        document.getElementById("x_val").innerText = data.x.toFixed(2) + "g";
        let box = document.getElementById("status_box");
        if(data.crash) {
          box.innerText = "זיהוי מכה! משדר CAN (מצב דומיננטי)";
          box.className = "status crash";
        } else {
          box.innerText = "סטטוס: רשת CAN מנוחה";
          box.className = "status normal";
        }
      });
    }, 100); 
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200); 
  
  pinMode(CAN_TX_PIN, OUTPUT);
  digitalWrite(CAN_TX_PIN, HIGH);
  
  Wire.begin(21, 22);
  
  Serial.println("Searching for BMI270 sensor at address 0x69...");
  
  // התיקון כאן: אנחנו אומרים לספרייה בדיוק לאיזו כתובת לגשת
  if (imu.beginI2C(0x69) != BMI2_OK) {
    Serial.println("ERROR: BMI270 not connected! Check your Qwiic wiring.");
  } else {
    Serial.println("SUCCESS: BMI270 connected and ready!");
  }
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Go to your browser and type this IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/data", []() {
    String json = "{\"x\":" + String(currentX) + ", \"crash\":" + (crashDetected ? "true" : "false") + "}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();
  
  imu.getSensorData();
  currentX = imu.data.accelX;
  
  if (abs(currentX) > CRASH_THRESHOLD && !crashDetected) {
    crashDetected = true;
    crashEndTime = millis() + 500; 
    digitalWrite(CAN_TX_PIN, LOW); 
  }
  
  if (crashDetected && millis() > crashEndTime) {
    crashDetected = false;
    digitalWrite(CAN_TX_PIN, HIGH);
  }
}
