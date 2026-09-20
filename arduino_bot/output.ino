// =====================================================
// MINELANDER - ESP8266
// MOTOR + LED + ACTIVE BUZZER
// =====================================================
//
// UART:
// ESP8266 D5 (TX) -> Nano D4 (RX)
// ESP8266 D6 (RX) <- Nano D3 (TX)
//
// MOTOR DRIVER:
// ESP8266 D1 -> M1
// ESP8266 D7 -> M2
//
// Other two motor-driver inputs are grounded.
// =====================================================

#include <SoftwareSerial.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================

// Motor driver
#define MOTOR_M1 D1
#define MOTOR_M2 D7

// LED
#define LED_PIN D2

// Active buzzer
#define BUZZER_PIN D4

// UART
#define ESP_RX D6
#define ESP_TX D5

SoftwareSerial nanoSerial(ESP_RX, ESP_TX);

// =====================================================
// VARIABLES
// =====================================================

bool dangerMode = false;
int currentSpeed = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {

  // USB Serial Monitor
  Serial.begin(9600);

  // UART communication with Nano
  nanoSerial.begin(9600);

  // Motor pins
  pinMode(MOTOR_M1, OUTPUT);
  pinMode(MOTOR_M2, OUTPUT);

  // LED
  pinMode(LED_PIN, OUTPUT);

  // Active buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initial state
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("================================");
  Serial.println("MINELANDER ESP8266");
  Serial.println("ACTUATOR SYSTEM READY");
  Serial.println("WAITING FOR NANO...");
  Serial.println("================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // ===================================================
  // RECEIVE COMMAND FROM NANO
  // ===================================================

  if (nanoSerial.available()) {

    String command = nanoSerial.readStringUntil('\n');
    command.trim();

    // Print received command to USB Serial Monitor
    Serial.print("NANO -> ESP: ");
    Serial.println(command);

    // =================================================
    // SAFE
    // =================================================

    if (command.startsWith("SAFE")) {

      if (!dangerMode) {

        int speed = extractSpeed(command);

        driveForward(speed);
        safeBuzzer();

        digitalWrite(LED_PIN, LOW);
      }
    }

    // =================================================
    // CAUTION
    // =================================================

    else if (command.startsWith("CAUTION")) {

      if (!dangerMode) {

        int speed = extractSpeed(command);

        driveForward(speed);
        cautionBuzzer();

        digitalWrite(LED_PIN, LOW);
      }
    }

    // =================================================
    // DANGER
    // =================================================

    else if (command.startsWith("DANGER")) {

      if (!dangerMode) {

        dangerMode = true;

        executeDangerSequence();
      }
    }
  }
}

// =====================================================
// EXTRACT SPEED
//
// Examples:
// SAFE:80
// CAUTION:30
// =====================================================

int extractSpeed(String command) {

  int separator = command.indexOf(':');

  if (separator == -1) {
    return 0;
  }

  int speed = command.substring(separator + 1).toInt();

  speed = constrain(speed, 0, 100);

  return speed;
}

// =====================================================
// DRIVE FORWARD
// =====================================================

void driveForward(int speed) {

  currentSpeed = speed;

  // 30% PWM on both motor control inputs
  analogWrite(MOTOR_M1, 307);
  analogWrite(MOTOR_M2, 307);

  Serial.println("MOTOR ON - PWM: 30%");
}

// =====================================================
// STOP MOTORS
// =====================================================

void stopMotors() {

  analogWrite(MOTOR_M1, 0);
  analogWrite(MOTOR_M2, 0);

  currentSpeed = 0;
}
// =====================================================
// TURN RIGHT 90 DEGREES
// =====================================================

void turnRight90() {

  Serial.println("TURNING RIGHT 90");

  analogWrite(MOTOR_M1, 307);
  analogWrite(MOTOR_M2, 0);

  delay(700);

  stopMotors();
}

// =====================================================
// TURN LEFT 90 DEGREES
// =====================================================

void turnLeft90() {

  Serial.println("TURNING LEFT 90");
  analogWrite(MOTOR_M1, 0);
  analogWrite(MOTOR_M2, 307);

  delay(700);

  stopMotors();
}

// =====================================================
// FORWARD FOR 3 SECONDS
// =====================================================

void forwardThreeSeconds() {

  Serial.println("FORWARD 3 SECONDS");

  driveForward(50);

  delay(3000);

  stopMotors();
}

// =====================================================
// DANGER SEQUENCE
// =====================================================

void executeDangerSequence() {

  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("!!! DANGER !!!");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!");

  // =================================================
  // IMMEDIATE STOP
  // =================================================

  stopMotors();

  // =================================================
  // LED ON
  // =================================================

  digitalWrite(LED_PIN, HIGH);

  // =================================================
  // AGGRESSIVE BUZZER
  // =================================================

  aggressiveBuzzer();

  delay(500);

  // =================================================
  // RIGHT 90
  // =================================================

  turnRight90();

  // =================================================
  // FORWARD 3 SEC
  // =================================================

  forwardThreeSeconds();

  // =================================================
  // LEFT 90
  // =================================================

  turnLeft90();

  // =================================================
  // FORWARD 3 SEC
  // =================================================

  forwardThreeSeconds();

  // =================================================
  // LEFT 90
  // =================================================

  turnLeft90();

  // =================================================
  // FORWARD 3 SEC
  // =================================================

  forwardThreeSeconds();

  // =================================================
  // RIGHT 90
  // =================================================

  turnRight90();

  // =================================================
  // FINAL FORWARD 3 SEC
  // =================================================

  forwardThreeSeconds();

  // =================================================
  // FINAL STOP
  // =================================================

  stopMotors();

  digitalWrite(LED_PIN, HIGH);

  Serial.println();
  Serial.println("================================");
  Serial.println("DETOUR COMPLETE");
  Serial.println("BOT STOPPED");
  Serial.println("================================");

  // =================================================
  // REMAIN IN DANGER MODE
  // =================================================

  while (true) {

    stopMotors();

    digitalWrite(LED_PIN, HIGH);

    aggressiveBuzzer();

    delay(1000);
  }
}

// =====================================================
// SAFE BUZZER
// =====================================================

void safeBuzzer() {

  digitalWrite(BUZZER_PIN, HIGH);

  delay(150);

  digitalWrite(BUZZER_PIN, LOW);
}

// =====================================================
// CAUTION BUZZER
// =====================================================

void cautionBuzzer() {

  digitalWrite(BUZZER_PIN, HIGH);

  delay(250);

  digitalWrite(BUZZER_PIN, LOW);
}

// =====================================================
// AGGRESSIVE BUZZER
// =====================================================

void aggressiveBuzzer() {

  for (int i = 0; i < 4; i++) {

    digitalWrite(BUZZER_PIN, HIGH);

    delay(150);

    digitalWrite(BUZZER_PIN, LOW);

    delay(70);
  }
}
