// Controller  - upload this .ino
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------------- WIFI ----------------
const char* ssid = "RRR";
const char* password = "20050522";

// ---------------- MQTT ----------------
const char* MQTT_BROKER = "broker.emqx.io";
const int   MQTT_PORT   = 1883;
const char* COUNT_TOPIC = "traffic/lane_count";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ---------------- TRAFFIC LIGHT PINS ----------------
#define L1_GREEN 14
#define L1_YELLOW 12
#define L1_RED 13

#define L2_GREEN 25
#define L2_YELLOW 26
#define L2_RED 27

#define L3_GREEN 18
#define L3_YELLOW 19
#define L3_RED 15

// ---------------- CAMERA TRIGGER ----------------
#define SERVO_PIN 5
#define TRIGGER_PIN 4 // controller -> CAM GPIO14

// ---------------- LANE SELECT BITS ----------------
#define BIT0_PIN 32   // controller -> CAM GPIO12 (BIT0)
#define BIT1_PIN 33   // controller -> CAM GPIO13 (BIT1)  (use GPIO33, not 35)

// ---------------- VARIABLES ----------------
Servo camServo;

int currentLane = 1;
int nextLane = 2;

int laneCount[4];
bool countAvailable[4];

bool firstCycle = true;

int MIN_TIME = 10;
int MAX_TIME = 45;
const int DEFAULT_FIRST_GREEN = 10;

// ---------------- SEND LANE BITS ----------------
void sendLaneBits(int lane) {
  // mapping: lane1 -> 00, lane2 -> 01, lane3 -> 10 (BIT1 BIT0)
  int b0 = 0;
  int b1 = 0;
  if (lane == 2) b0 = 1;
  if (lane == 3) b1 = 1;

  digitalWrite(BIT0_PIN, b0);
  digitalWrite(BIT1_PIN, b1);

  // small settle time so CAM reads stable bits before trigger/rotation
  delay(40);
  Serial.printf("→ Lane Bits Sent: lane=%d  BIT1=%d  BIT0=%d\n", lane, b1, b0);
}

// ---------------- LED CONTROL ----------------
void setLaneSignals(int lane, bool g, bool y, bool r) {
  if (lane == 1) { digitalWrite(L1_GREEN,g); digitalWrite(L1_YELLOW,y); digitalWrite(L1_RED,r); }
  if (lane == 2) { digitalWrite(L2_GREEN,g); digitalWrite(L2_YELLOW,y); digitalWrite(L2_RED,r); }
  if (lane == 3) { digitalWrite(L3_GREEN,g); digitalWrite(L3_YELLOW,y); digitalWrite(L3_RED,r); }
}

void forceSingleGreen(int lane) {
  // avoid long all-red by writing quickly
  digitalWrite(L1_GREEN, LOW); digitalWrite(L1_YELLOW, LOW); digitalWrite(L1_RED, HIGH);
  digitalWrite(L2_GREEN, LOW); digitalWrite(L2_YELLOW, LOW); digitalWrite(L2_RED, HIGH);
  digitalWrite(L3_GREEN, LOW); digitalWrite(L3_YELLOW, LOW); digitalWrite(L3_RED, HIGH);

  // immediate set target green
  digitalWrite((lane==1?L1_GREEN: lane==2?L2_GREEN: L3_GREEN), HIGH);
  digitalWrite((lane==1?L1_RED  : lane==2?L2_RED  : L3_RED),   LOW);
  digitalWrite((lane==1?L1_YELLOW: lane==2?L2_YELLOW: L3_YELLOW), LOW);

  Serial.printf(" Lane %d -> GREEN\n", lane);
}

// ---------------- SERVO MOVEMENT ----------------
void rotateToLane(int lane) {
  int angle = (lane == 1) ? 0 : (lane == 2) ? 90 : 180;
  Serial.printf("Rotating to %d° (Lane %d)\n", angle, lane);
  camServo.write(angle);
  delay(650); // allow travel and mechanical settle
  delay(100); // small extra settle
}

// ---------------- CAMERA TRIGGER ----------------
void triggerCameraOnce() {
  digitalWrite(TRIGGER_PIN, HIGH);
  delay(120);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println(" Trigger SENT");
}

// ---------------- MQTT CALLBACK ----------------
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];

  int idx = msg.indexOf('|');
  if (idx < 0) {
    Serial.printf("[MQTT] malformed '%s'\n", msg.c_str());
    return;
  }

  int lane = msg.substring(0,idx).toInt();
  int count = msg.substring(idx+1).toInt();

  if (lane < 1 || lane > 3) {
    Serial.printf("[MQTT] invalid lane %d\n", lane);
    return;
  }

  laneCount[lane] = count;
  countAvailable[lane] = true;
  Serial.printf("[MQTT] Received -> Lane=%d Count=%d\n", lane, count);
}

void ensureMQTT() {
  if (mqtt.connected()) return;
  Serial.print("Connecting MQTT...");
  // create a unique client id
  String clientId = "TRAFFIC_CTRL_";
  clientId += String((uint32_t)esp_random(), HEX);

  while (!mqtt.connected()) {
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");
      mqtt.subscribe(COUNT_TOPIC);
      Serial.printf("Subscribed to %s\n", COUNT_TOPIC);
    } else {
      Serial.print(".");
      delay(500);
    }
  }
}

// ---------------- GREEN TIME ----------------
int computeGreenFromCount(int count) {
  if (count < 1) count = 1;
  if (count > 25) count = 25;
  int t = map(count, 1, 25, MIN_TIME, MAX_TIME);
  return constrain(t, MIN_TIME, MAX_TIME);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\nController booting...");

  // lights
  pinMode(L1_GREEN,OUTPUT); pinMode(L1_YELLOW,OUTPUT); pinMode(L1_RED,OUTPUT);
  pinMode(L2_GREEN,OUTPUT); pinMode(L2_YELLOW,OUTPUT); pinMode(L2_RED,OUTPUT);
  pinMode(L3_GREEN,OUTPUT); pinMode(L3_YELLOW,OUTPUT); pinMode(L3_RED,OUTPUT);

  // lane bits
  pinMode(BIT0_PIN, OUTPUT);
  pinMode(BIT1_PIN, OUTPUT);

  // trigger
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  camServo.attach(SERVO_PIN);

  // init counts
  for (int i=1;i<=3;i++) {
    laneCount[i] = 0;
    countAvailable[i] = false;
  }

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMQTT();

  // start state: lane1 visible & green
  sendLaneBits(1);
  rotateToLane(1);
  forceSingleGreen(1);

  Serial.println("Controller ready.");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  mqtt.loop();
  ensureMQTT();

  Serial.printf("\n====== CURRENT LANE = %d ======\n", currentLane);

  // Determine green time
  int greenTime;
  if (firstCycle && currentLane == 1) {
    greenTime = DEFAULT_FIRST_GREEN;
  } else {
    if (countAvailable[currentLane]) greenTime = computeGreenFromCount(laneCount[currentLane]);
    else greenTime = MIN_TIME; // fallback
  }
  Serial.printf("Assigned GreenTime for lane %d = %d sec\n", currentLane, greenTime);

  // Prepare: ensure lane bits + servo + LED show current lane
  sendLaneBits(currentLane);
  rotateToLane(currentLane);
  forceSingleGreen(currentLane);

  // Determine next lane
  nextLane = (currentLane == 3) ? 1 : currentLane + 1;
  countAvailable[nextLane] = false; // reset waiting flag

  bool preRotated = false;

  // GREEN countdown
  for (int i = greenTime; i > 0; --i) {
    Serial.printf("Remaining: %d\n", i);

    // At 5s — pre-rotate to next lane, send bits and trigger once
    if (i == 5 && !preRotated) {
      Serial.printf("Pre-rotation: prepare next lane %d\n", nextLane);
      sendLaneBits(nextLane);
      rotateToLane(nextLane);
      delay(120); // small settle
      triggerCameraOnce();   // ask CAM to capture next lane (CAM will POST lane id)
      preRotated = true;
      // keep servo at next lane — do NOT rotate back
    }

    // keep MQTT processing alive
    mqtt.loop();
    delay(1000);
  }

  // YELLOW phase for current (short)
  Serial.println("YELLOW PHASE");
  setLaneSignals(currentLane, LOW, HIGH, LOW);
  delay(2000);

  // Set current lane to RED
  setLaneSignals(currentLane, LOW, LOW, HIGH);

  // Wait briefly for next lane count to arrive (it may have been published already)
  unsigned long waitStart = millis();
  while (!countAvailable[nextLane] && millis() - waitStart < 3000) {
    mqtt.loop();
    delay(50);
  }

  if (!countAvailable[nextLane]) {
    // fallback
    laneCount[nextLane] = 1;
    countAvailable[nextLane] = true;
    Serial.printf("[WARN] no count for lane %d -> fallback to 1\n", nextLane);
  } else {
    Serial.printf("[INFO] count for next lane %d = %d\n", nextLane, laneCount[nextLane]);
  }

  // move to next lane (servo already at nextLane due to pre-rotation)
  currentLane = nextLane;

  // firstCycle handling
  if (firstCycle && currentLane != 1) firstCycle = false;

  // Prevent transient all-red: immediately set new lane green
  forceSingleGreen(currentLane);
  sendLaneBits(currentLane);

  delay(200); // small stabilization
}