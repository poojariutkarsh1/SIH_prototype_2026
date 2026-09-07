#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================

#define TRIG_PIN 9
#define ECHO_PIN 10
#define IR_PIN 7

// =====================================================
// FOG STATE
// 1 = CLEAR
// 2 = MODERATE
// 3 = DENSE
// =====================================================

int fogState = 1;

// =====================================================
// BASE SPEEDS
// Change these as required
// =====================================================

const int CLEAR_SPEED    = 80;
const int MODERATE_SPEED = 60;
const int DENSE_SPEED    = 40;

// =====================================================
// TTC SETTINGS
// =====================================================

const int WINDOW_SIZE = 5;

const float MIN_CLOSING_SPEED = 3.0;  // cm/s

const float SAFE_TTC   = 8.0;
const float DANGER_TTC = 3.0;

const float EMERGENCY_DISTANCE = 10.0;

// =====================================================
// SLOPE SETTINGS
// =====================================================

const float LEVEL_THRESHOLD = 3.0;

const float MAX_POWER_ADJUSTMENT = 30.0;

// =====================================================
// MPU6050
// =====================================================

Adafruit_MPU6050 mpu;

// =====================================================
// MEDIAN FILTER
// =====================================================

float readings[WINDOW_SIZE];

int readingIndex = 0;
int readingCount = 0;

// =====================================================
// TTC VARIABLES
// =====================================================

float previousDistance = 0;

unsigned long previousTime = 0;

// =====================================================
// DANGER LATCH
// Prevents repeated DANGER commands
// =====================================================

bool dangerTriggered = false;

// =====================================================
// READ ULTRASONIC
// =====================================================

float readDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return 400.0;
  }

  float distance = duration * 0.0343 / 2.0;

  return distance;
}

// =====================================================
// MEDIAN FILTER
// =====================================================

float getMedian() {

  float sorted[WINDOW_SIZE];

  for (int i = 0; i < WINDOW_SIZE; i++) {
    sorted[i] = readings[i];
  }

  for (int i = 0; i < WINDOW_SIZE - 1; i++) {

    for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {

      if (sorted[j] > sorted[j + 1]) {

        float temp = sorted[j];

        sorted[j] = sorted[j + 1];

        sorted[j + 1] = temp;
      }
    }
  }

  return sorted[WINDOW_SIZE / 2];
}

// =====================================================
// GET BASE SPEED FROM FOG STATE
// =====================================================

int getBaseSpeed() {

  if (fogState == 1) {
    return CLEAR_SPEED;
  }

  else if (fogState == 2) {
    return MODERATE_SPEED;
  }

  else {
    return DENSE_SPEED;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // UART TO ESP8266
  // Add your chosen serial pins here if using SoftwareSerial.

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IR_PIN, INPUT);

  // MPU6050

  if (!mpu.begin()) {

    Serial.println("MPU6050 NOT FOUND!");

    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println();
  Serial.println("========================================");
  Serial.println("       MINELANDER SAFETY SYSTEM");
  Serial.println("========================================");

  Serial.print("FOG STATE: ");

  if (fogState == 1)
    Serial.println("CLEAR");

  else if (fogState == 2)
    Serial.println("MODERATE");

  else
    Serial.println("DENSE");

  Serial.print("BASE SPEED: ");
  Serial.print(getBaseSpeed());
  Serial.println("%");

  Serial.println("========================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // ===================================================
  // 1. READ ULTRASONIC
  // ===================================================

  float rawDistance = readDistance();

  readings[readingIndex] = rawDistance;

  readingIndex++;

  if (readingIndex >= WINDOW_SIZE) {
    readingIndex = 0;
  }

  if (readingCount < WINDOW_SIZE) {
    readingCount++;
  }

  // ===================================================
  // 2. READ IR
  // ===================================================

  int irState = digitalRead(IR_PIN);

  // ===================================================
  // 3. READ MPU6050
  // ===================================================

  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // ===================================================
  // PITCH CALCULATION
  // ===================================================

  float pitch = atan2(
    ax,
    sqrt(ay * ay + az * az)
  ) * 180.0 / PI;

  // ===================================================
  // 4. SLOPE CLASSIFICATION
  // ===================================================

  String slope;

  float powerAdjustment = 0;

  if (pitch > LEVEL_THRESHOLD) {

    slope = "UPHILL";

    powerAdjustment = pitch * 2.0;

    if (powerAdjustment > MAX_POWER_ADJUSTMENT) {
      powerAdjustment = MAX_POWER_ADJUSTMENT;
    }
  }

  else if (pitch < -LEVEL_THRESHOLD) {

    slope = "DOWNHILL";

    powerAdjustment = pitch * 2.0;

    if (powerAdjustment < -MAX_POWER_ADJUSTMENT) {
      powerAdjustment = -MAX_POWER_ADJUSTMENT;
    }
  }

  else {

    slope = "LEVEL";

    powerAdjustment = 0;
  }

  // ===================================================
  // WAIT UNTIL MEDIAN FILTER IS FULL
  // ===================================================

  if (readingCount == WINDOW_SIZE) {

    float filteredDistance = getMedian();

    unsigned long currentTime = millis();

    // =================================================
    // FIRST TTC READING
    // =================================================

    if (previousTime == 0) {

      previousDistance = filteredDistance;
      previousTime = currentTime;

      Serial.println("Initializing TTC...");
    }

    else {

      float deltaTime =
        (currentTime - previousTime) / 1000.0;

      if (deltaTime <= 0) {
        deltaTime = 0.001;
      }

      // =================================================
      // CLOSING SPEED
      // =================================================

      float closingSpeed =
        (previousDistance - filteredDistance) / deltaTime;

      // =================================================
      // SAFETY VARIABLES
      // =================================================

      String safetyStatus;
      String action;

      float ttc = -1;

      // =================================================
      // PRIORITY 1: EMERGENCY DISTANCE
      // =================================================

      if (filteredDistance <= EMERGENCY_DISTANCE) {

        safetyStatus = "DANGER";
        action = "STOP - OBSTACLE TOO CLOSE";
      }

      // =================================================
      // PRIORITY 2: IR
      // =================================================

      else if (irState == LOW) {

        safetyStatus = "DANGER";
        action = "STOP - IR OBSTACLE";
      }

      // =================================================
      // PRIORITY 3: TTC
      // =================================================

      else if (closingSpeed > MIN_CLOSING_SPEED) {

        ttc = filteredDistance / closingSpeed;

        if (ttc > SAFE_TTC) {

          safetyStatus = "SAFE";
          action = "CONTINUE";
        }

        else if (ttc > DANGER_TTC) {

          safetyStatus = "CAUTION";
          action = "SLOW DOWN";
        }

        else {

          safetyStatus = "DANGER";
          action = "STOP";
        }
      }

      // =================================================
      // NO MEANINGFUL CLOSING
      // =================================================

      else {

        safetyStatus = "SAFE";
        action = "CONTINUE";
      }

      // =================================================
      // BASE POWER FROM FOG
      // =================================================

      float basePower = getBaseSpeed();

      // =================================================
      // SUGGESTED POWER
      // =================================================

      float suggestedPower =
        basePower + powerAdjustment;

      if (suggestedPower > 100) {
        suggestedPower = 100;
      }

      if (suggestedPower < 0) {
        suggestedPower = 0;
      }

      // =================================================
      // CAUTION SPEED
      //
      // TTC remains independent of fog.
      // Only the driving speed is reduced.
      // =================================================

      float finalPower = suggestedPower;

      if (safetyStatus == "CAUTION") {

        finalPower = suggestedPower * 0.5;
      }

      // =================================================
      // DANGER
      // =================================================

      if (safetyStatus == "DANGER") {

        finalPower = 0;

        // Send DANGER only once

        if (!dangerTriggered) {

          Serial.println();
          Serial.println("!!! DANGER DETECTED !!!");

          // Send command to ESP
          Serial.println("UART -> DANGER");

          // Replace this with your actual UART
          // to ESP8266.
          //
          // Example:
          // espSerial.println("DANGER");

          dangerTriggered = true;
        }
      }

      // =================================================
      // SAFE
      // =================================================

      else if (safetyStatus == "SAFE") {

        if (!dangerTriggered) {

          // Replace with ESP UART
          //
          // espSerial.print("SAFE:");
          // espSerial.println((int)finalPower);

          Serial.println("UART -> SAFE");
        }
      }

      // =================================================
      // CAUTION
      // =================================================

      else if (safetyStatus == "CAUTION") {

        if (!dangerTriggered) {

          // Replace with ESP UART
          //
          // espSerial.print("CAUTION:");
          // espSerial.println((int)finalPower);

          Serial.println("UART -> CAUTION");
        }
      }

      // =================================================
      // SERIAL MONITOR
      // =================================================

      Serial.println();
      Serial.println("========== MINELANDER ==========");

      Serial.print("FOG STATE: ");

      if (fogState == 1)
        Serial.println("CLEAR");

      else if (fogState == 2)
        Serial.println("MODERATE");

      else
        Serial.println("DENSE");

      Serial.print("DISTANCE: ");
      Serial.print(filteredDistance, 2);
      Serial.println(" cm");

      Serial.print("CLOSING SPEED: ");
      Serial.print(closingSpeed, 2);
      Serial.println(" cm/s");

      if (ttc > 0) {

        Serial.print("TTC: ");
        Serial.print(ttc, 2);
        Serial.println(" sec");

      } else {

        Serial.println("TTC: NO MEANINGFUL CLOSING");
      }

      Serial.println();

      if (irState == LOW)
        Serial.println("IR STATUS: OBSTACLE DETECTED");
      else
        Serial.println("IR STATUS: CLEAR");

      Serial.print("SAFETY STATUS: ");
      Serial.println(safetyStatus);

      Serial.print("ACTION: ");
      Serial.println(action);

      Serial.println();

      Serial.print("PITCH: ");

      if (pitch >= 0)
        Serial.print("+");

      Serial.print(pitch, 2);
      Serial.println(" degrees");

      Serial.print("SLOPE: ");
      Serial.println(slope);

      Serial.print("POWER ADJUSTMENT: ");

      if (powerAdjustment >= 0)
        Serial.print("+");

      Serial.print(powerAdjustment, 1);
      Serial.println("%");

      Serial.print("BASE SPEED: ");
      Serial.print(basePower);
      Serial.println("%");

      Serial.print("FINAL POWER: ");
      Serial.print(finalPower, 1);
      Serial.println("%");

      Serial.print("FINAL COMMAND: ");

      if (safetyStatus == "DANGER")
        Serial.println("STOP");

      else if (safetyStatus == "CAUTION")
        Serial.println("SLOW DOWN");

      else
        Serial.println("CONTINUE");

      Serial.println("================================");

      // =================================================
      // SAVE TTC VALUES
      // =================================================

      previousDistance = filteredDistance;
      previousTime = currentTime;
    }
  }

  // ===================================================
  // 200 ms LOOP
  // ===================================================

  delay(200);
}
