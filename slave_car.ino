#include <WiFi.h>
#include <WebServer.h>
#include <RF24.h>

// NRF24L01 Pins
#define CE_PIN   5
#define CSN_PIN  4
RF24 radio(CE_PIN, CSN_PIN);

// Ultrasonic Sensor Pins
#define TRIGGER_PIN 2
#define ECHO_PIN 15

// Motor Control Pins
#define IN1 13
#define IN2 12
#define EN1 14
#define IN3 27
#define IN4 26
#define EN2 25

// Encoder Pins
#define ENCODER_LEFT 32   // Left wheel encoder
#define ENCODER_RIGHT 33  // Right wheel encoder

// Wi-Fi credentials for Access Point
const char* ssid = "Platoon_Car_main";
const char* password = "12345678";

// WebServer setup
WebServer server(80);

// Radio addresses
byte addresses[][6] = {"1Node", "2Node", "3Node"};

// Movement variables
String currentDirection = "STOP";
unsigned long lastCommandTime = 0;
const unsigned long timeout = 5000; // 5 seconds timeout

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
const float WHEEL_CIRCUMFERENCE = 0.21; // meters
const int PULSES_PER_REVOLUTION = 20;
float distanceTravelled = 0;
float speed = 0;

// Role flag
enum Role { HEADER, SLAVE1, SLAVE2 };
Role currentRole = HEADER;

// Encoder ISRs
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(EN1, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(EN2, OUTPUT);
  stopCar();

  // Ultrasonic setup
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Encoder setup
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);

  // Wi-Fi AP setup
  WiFi.softAP(ssid, password);
  Serial.println("Wi-Fi Access Point Started");
  Serial.println("IP Address: " + WiFi.softAPIP().toString());

  // NRF24L01 setup
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization failed!");
    while (1) delay(1000);
  }
  radio.setPALevel(RF24_PA_HIGH);
  updateRadioPipes();
  radio.startListening();
  Serial.println("NRF24L01 initialized successfully");

  // WebServer Setup
  server.on("/", handleRoot);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/switch", HTTP_GET, handleRoleSwitch);
  server.begin();
  Serial.println("Web Server Started");
}

void loop() {
  server.handleClient();
  receiveCommand();
  handleMovement();
  updateEncoderReadings();

  if (millis() - lastCommandTime > timeout) {
    stopCar();
    currentDirection = "STOP";
  }

  // Print sensor data every second
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {
    Serial.println(currentRole == HEADER ? "--- Header Status ---" : currentRole == SLAVE1 ? "--- Slave 1 Status ---" : "--- Slave 2 Status ---");
    Serial.println("Distance Travelled: " + String(distanceTravelled, 2) + " m");
    Serial.println("Speed: " + String(speed, 2) + " m/s");
    Serial.println("Ultrasonic Distance: " + String(getDistance(), 2) + " cm");
    Serial.println("Current Direction: " + currentDirection);
    lastPrintTime = millis();
  }
}

long getDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration > 0) ? (duration * 0.034 / 2) : -1; // Return in cm, -1 if no echo
}

void updateEncoderReadings() {
  static float previousDistance = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  // Calculate distance from encoder counts
  float leftDistance = (leftEncoderCount / (float)PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
  float rightDistance = (rightEncoderCount / (float)PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
  distanceTravelled = (leftDistance + rightDistance) / 2;

  // Calculate speed
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  if (deltaTime > 0) {
    speed = (distanceTravelled - previousDistance) / deltaTime; // m/s
  }
  previousDistance = distanceTravelled;
  lastTime = currentTime;
}

void handleMovement() {
  if (currentRole == HEADER) {
    // Header sends direction periodically
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 500) {
      sendRadioMessage(currentDirection);
      lastSendTime = millis();
    }
  }

  if (currentDirection == "FORWARD") {
    moveForward();
  } else if (currentDirection == "BACKWARD") {
    moveBackward();
  } else if (currentDirection == "LEFT") {
    turnLeft();
  } else if (currentDirection == "RIGHT") {
    turnRight();
  } else {
    stopCar();
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN1, 255); analogWrite(EN2, 255);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN1, 255); analogWrite(EN2, 255);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN1, 0); analogWrite(EN2, 180);
}

void turnRight() {
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN1, 180); analogWrite(EN2, 0);
}

void stopCar() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(EN1, 0); analogWrite(EN2, 0);
}

void sendRadioMessage(String message) {
  radio.stopListening();
  Serial.println("Sending: " + message);
  if (radio.write(message.c_str(), message.length() + 1)) {
    Serial.println("Sent successfully");
  } else {
    Serial.println("Send failed");
  }
  radio.startListening();
}

void receiveCommand() {
  if (radio.available()) {
    char command[32] = {0};
    radio.read(&command, sizeof(command));
    currentDirection = String(command);
    lastCommandTime = millis();
    Serial.println("Received: " + currentDirection);
  }
}

void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <style>
      .header { font-size: 24px; font-weight: bold; text-align: center; margin-top: 10px; margin-bottom: 20px; }
      .arrows { font-size: 40px; color: red; }
      td.button { background-color: black; border-radius: 25%; box-shadow: 5px 5px #888888; }
      td.button:active { transform: translate(5px, 5px); box-shadow: none; }
      .noselect { user-select: none; }
    </style>
  </head>
  <body class="noselect" align="center" style="background-color: white">
    <div class="header">PLATOON SYSTEM</div>
    <table style="width:400px; margin:auto; table-layout:fixed" cellspacing="10">
      <tr><td></td><td class="button" ontouchstart="sendInput('MoveCar','FORWARD')" ontouchend="sendInput('MoveCar','STOP')"><span class="arrows">⇧</span></td><td></td></tr>
      <tr><td class="button" ontouchstart="sendInput('MoveCar','LEFT')" ontouchend="sendInput('MoveCar','STOP')"><span class="arrows">⇦</span></td><td class="button"></td><td class="button" ontouchstart="sendInput('MoveCar','RIGHT')" ontouchend="sendInput('MoveCar','STOP')"><span class="arrows">⇨</span></td></tr>
      <tr><td></td><td class="button" ontouchstart="sendInput('MoveCar','BACKWARD')" ontouchend="sendInput('MoveCar','STOP')"><span class="arrows">⇩</span></td><td></td></tr>
    </table>
    <br><br>
    <button onclick="window.location.href='/switch'">Switch Role</button>
    <script>
      function sendInput(key, value) {
        fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'MoveCar=' + value });
      }
    </script>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleControl() {
  if (server.hasArg("MoveCar")) {
    String direction = server.arg("MoveCar");
    currentDirection = direction;
    if (currentRole == HEADER) {
      sendRadioMessage(direction);
    }
    lastCommandTime = millis();
    Serial.println("Control received: " + direction);
  }
  server.send(200, "text/plain", "OK");
}

void handleRoleSwitch() {
  currentRole = (Role)((currentRole + 1) % 3);
  stopCar();
  updateRadioPipes();
  radio.startListening();
  Serial.println(currentRole == HEADER ? "Switched to Header" : currentRole == SLAVE1 ? "Switched to Slave 1" : "Switched to Slave 2");
  server.send(200, "text/plain", currentRole == HEADER ? "Now in Header Role" : currentRole == SLAVE1 ? "Now in Slave 1 Role" : "Now in Slave 2 Role");
}

void updateRadioPipes() {
  switch (currentRole) {
    case HEADER:
      radio.openWritingPipe(addresses[1]);
      radio.openReadingPipe(1, addresses[0]);
      radio.openReadingPipe(2, addresses[2]);
      break;
    case SLAVE1:
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1, addresses[1]);
      radio.openReadingPipe(2, addresses[2]);
      break;
    case SLAVE2:
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1, addresses[1]);
      radio.openReadingPipe(2, addresses[2]);
      break;
  }
  Serial.println("Radio pipes updated for role: " + String(currentRole));
}