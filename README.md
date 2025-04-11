# -FINAL-Quadcopter-drone-source-codes
This is Final quadcopter drone source codes to control it using web interface and full stability.


#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi credentials
const char* ssid = "ESP32-DRONE";
const char* password = "quadcopter";

// BLDC motor PWM pins and channels
const int motorPins[4] = {23, 22, 21, 19};    // M1, M2, M3, M4
const int motorChannels[4] = {0, 1, 2, 3};
const int pwmFreq = 50;
const int pwmResolution = 16;

// Variables for pitch, roll, throttle, and motor state
int throttle = 0;
int userPitch = 0;
int userRoll = 0;
bool motorActive = false;

// MPU6050 setup
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float pitchAngle = 0, rollAngle = 0;

// Web server
AsyncWebServer server(80);

// Function to write PWM signals to the motors
void writeMicroseconds(uint8_t channel, int microseconds) {
  int duty = (microseconds * 65536) / 20000;
  ledcWrite(channel, duty);
}

// Apply mixer logic with stabilization
void applyMixer(int pitch, int roll) {
  if (!motorActive) {
    for (int i = 0; i < 4; i++) {
      writeMicroseconds(motorChannels[i], 1000); // Stop signal
    }
    return;
  }

  int m1 = throttle + pitch + roll;
  int m2 = throttle + pitch - roll;
  int m3 = throttle - pitch - roll;
  int m4 = throttle - pitch + roll;

  m1 = constrain(m1, 0, 255);
  m2 = constrain(m2, 0, 255);
  m3 = constrain(m3, 0, 255);
  m4 = constrain(m4, 0, 255);

  int us1 = map(m1, 0, 255, 1000, 2000);
  int us2 = map(m2, 0, 255, 1000, 2000);
  int us3 = map(m3, 0, 255, 1000, 2000);
  int us4 = map(m4, 0, 255, 1000, 2000);

  writeMicroseconds(motorChannels[0], us1);
  writeMicroseconds(motorChannels[1], us2);
  writeMicroseconds(motorChannels[2], us3);
  writeMicroseconds(motorChannels[3], us4);
}

// Setup MPU6050
void setupMPU() {
  Wire.begin(18, 5);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
}

// Read and calculate pitch/roll
void readMPU() {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  pitchAngle = atan2(ay, az) * 180.0 / PI;
  rollAngle = atan2(ax, az) * 180.0 / PI;
}

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  delay(1000);
  Serial.println("ESP32 AP running at:");
  Serial.println(WiFi.softAPIP());

  for (int i = 0; i < 4; i++) {
    ledcSetup(motorChannels[i], pwmFreq, pwmResolution);
    ledcAttachPin(motorPins[i], motorChannels[i]);
    writeMicroseconds(motorChannels[i], 1000);
  }

  delay(2000);

  setupMPU();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <title>Quadcopter Control</title>
        <style>
          body {
            font-family: Arial;
            background-color: #111;
            color: #fff;
            text-align: center;
          }
          h2 {
            color: #03dac6;
            margin-bottom: 20px;
          }
          .slider-container {
            display: flex;
            justify-content: center;
            gap: 60px;
            margin-bottom: 20px;
          }
          .vertical-slider {
            writing-mode: bt-lr;
            -webkit-appearance: slider-vertical;
            height: 200px;
            width: 40px;
          }
          .horizontal-slider {
            width: 300px;
            margin: 20px auto;
          }
          .master-button {
            background: green;
            border: none;
            color: black;
            font-size: 18px;
            padding: 15px 40px;
            border-radius: 10px;
            cursor: pointer;
            margin-top: 30px;
          }
        </style>
      </head>
      <body>
        <h2>Quadcopter Control</h2>

        <div class="slider-container">
          <div>
            <p>Pitch (F/B)</p>
            <input type="range" min="-100" max="100" value="0" class="vertical-slider" id="pitch">
            <p id="valPitch">0</p>
          </div>
          <div>
            <p>Roll (L/R)</p>
            <input type="range" min="-100" max="100" value="0" class="vertical-slider" id="roll">
            <p id="valRoll">0</p>
          </div>
        </div>

        <p>Throttle</p>
        <input type="range" min="0" max="255" value="0" class="horizontal-slider" id="throttle">
        <p id="valThrottle">0</p>

        <button class="master-button" id="motorBtn" onclick="toggleMotor()">Start / Stop Motors</button>

        <script>
          let motorState = false;

          const pitchSlider = document.getElementById("pitch");
          const rollSlider = document.getElementById("roll");
          const throttleSlider = document.getElementById("throttle");

          const valPitch = document.getElementById("valPitch");
          const valRoll = document.getElementById("valRoll");
          const valThrottle = document.getElementById("valThrottle");

          let pitchInterval = null;
          let rollInterval = null;

          document.addEventListener("keydown", (e) => {
            if (e.repeat) return;

            if (e.key === "f" || e.key === "b") {
              clearInterval(pitchInterval);
              pitchInterval = setInterval(() => {
                let val = parseInt(pitchSlider.value);
                val += (e.key === "f") ? 1 : -1;
                val = Math.max(-100, Math.min(100, val));
                pitchSlider.value = val;
                updateDisplay();
                sendData();
              }, 50);
            }

            if (e.key === "r" || e.key === "l") {
              clearInterval(rollInterval);
              rollInterval = setInterval(() => {
                let val = parseInt(rollSlider.value);
                val += (e.key === "r") ? 1 : -1;
                val = Math.max(-100, Math.min(100, val));
                rollSlider.value = val;
                updateDisplay();
                sendData();
              }, 50);
            }
          });

          document.addEventListener("keyup", (e) => {
            if (e.key === "f" || e.key === "b") {
              clearInterval(pitchInterval);
              pitchSlider.value = 0;
              updateDisplay();
              sendData();
            }
            if (e.key === "r" || e.key === "l") {
              clearInterval(rollInterval);
              rollSlider.value = 0;
              updateDisplay();
              sendData();
            }
          });

          function updateDisplay() {
            valPitch.innerText = pitchSlider.value;
            valRoll.innerText = rollSlider.value;
            valThrottle.innerText = throttleSlider.value;
          }

          function sendData() {
            const pitch = pitchSlider.value;
            const roll = rollSlider.value;
            const throttle = throttleSlider.value;
            fetch(`/control?pitch=${pitch}&roll=${roll}&throttle=${throttle}`);
          }

          pitchSlider.oninput = rollSlider.oninput = throttleSlider.oninput = () => {
            updateDisplay();
            sendData();
          };

          pitchSlider.onmouseup = rollSlider.onmouseup = () => {
            pitchSlider.value = 0;
            rollSlider.value = 0;
            updateDisplay();
            sendData();
          };

          function toggleMotor() {
            motorState = !motorState;
            fetch(`/motor?state=${motorState ? 'on' : 'off'}`);
            document.getElementById("motorBtn").style.background = motorState ? "red" : "green";
          }

          updateDisplay();
        </script>
      </body>
      </html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
    String state = request->getParam("state")->value();
    motorActive = (state == "on");
    Serial.println(motorActive ? "Motors Activated" : "Motors Deactivated");
    applyMixer(0, 0);
    request->send(200, "text/plain", "OK");
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!motorActive) {
      request->send(200, "text/plain", "Motors are OFF");
      return;
    }

    userPitch = request->getParam("pitch")->value().toInt();
    userRoll = request->getParam("roll")->value().toInt();
    throttle = request->getParam("throttle")->value().toInt();
    request->send(200, "text/plain", "Control Updated");
  });

  server.begin();
}

void loop() {
  readMPU();

  int stabilizedPitch = userPitch - (int)(pitchAngle * 0.5);
  int stabilizedRoll  = userRoll  - (int)(rollAngle * 0.5);

  applyMixer(stabilizedPitch, stabilizedRoll);

  delay(50);
}
