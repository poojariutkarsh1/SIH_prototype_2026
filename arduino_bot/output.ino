// =====================================================
// MINELANDER - ESP8266
// MOTOR + LED + BUZZER
// =====================================================

// =====================================================
// PIN DEFINITIONS
// ADD YOUR OWN PINS
// =====================================================

// Motor driver
#define MOTOR_L_IN1  YOUR_PIN
#define MOTOR_L_IN2  YOUR_PIN

#define MOTOR_R_IN1  YOUR_PIN
#define MOTOR_R_IN2  YOUR_PIN

// If your driver has ENA/ENB:
#define MOTOR_L_EN   YOUR_PIN
#define MOTOR_R_EN   YOUR_PIN

// LED
#define LED_PIN      YOUR_PIN

// Buzzer
#define BUZZER_PIN   YOUR_PIN

// =====================================================
// VARIABLES
// =====================================================

bool dangerMode = false;

int currentSpeed = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // Motor pins

  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);

  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);

  // LED

  pinMode(LED_PIN, OUTPUT);

  // Buzzer

  pinMode(BUZZER_PIN, OUTPUT);

  // Initial state

  stopMotors();

  digitalWrite(LED_PIN, LOW);

  noTone(BUZZER_PIN);

  Serial.println("================================");
  Serial.println("MINELANDER ESP8266");
  Serial.println("ACTUATOR SYSTEM READY");
  Serial.println("================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    command.trim();

    // =================================================
    // SAFE
    // =================================================

    if (command.startsWith("SAFE")) {

      if (!dangerMode) {

        dangerMode = false;

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

        dangerMode = false;

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
// Example:
// SAFE:80
// CAUTION:40
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

  // Set direction

  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);

  // PWM

  int pwm =
    map(speed, 0, 100, 0, 1023);

  analogWrite(MOTOR_L_EN, pwm);
  analogWrite(MOTOR_R_EN, pwm);
}

// =====================================================
// STOP MOTORS
// =====================================================

void stopMotors() {

  analogWrite(MOTOR_L_EN, 0);
  analogWrite(MOTOR_R_EN, 0);

  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);

  currentSpeed = 0;
}

// =====================================================
// TURN RIGHT
// =====================================================

void turnRight90() {

  // Left wheel forward
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);

  // Right wheel backward
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);

  analogWrite(MOTOR_L_EN, 700);
  analogWrite(MOTOR_R_EN, 700);

  // CALIBRATE THIS TIME
  delay(700);

  stopMotors();
}

// =====================================================
// TURN LEFT
// =====================================================

void turnLeft90() {

  // Left wheel backward
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);

  // Right wheel forward
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);

  analogWrite(MOTOR_L_EN, 700);
  analogWrite(MOTOR_R_EN, 700);

  // CALIBRATE THIS TIME
  delay(700);

  stopMotors();
}

// =====================================================
// FORWARD FOR 3 SECONDS
// =====================================================

void forwardThreeSeconds() {

  driveForward(50);

  delay(3000);

  stopMotors();
}

// =====================================================
// DANGER SEQUENCE
// =====================================================

void executeDangerSequence() {

  Serial.println("!!! DANGER !!!");

  // ================================================
  // IMMEDIATE STOP
  // ================================================

  stopMotors();

  // ================================================
  // LED ON
  // ================================================

  digitalWrite(LED_PIN, HIGH);

  // ================================================
  // AGGRESSIVE BUZZER
  // ================================================

  aggressiveBuzzer();

  delay(500);

  // ================================================
  // RIGHT 90
  // ================================================

  turnRight90();

  // ================================================
  // FORWARD 3 SEC
  // ================================================

  forwardThreeSeconds();

  // ================================================
  // LEFT 90
  // ================================================

  turnLeft90();

  // ================================================
  // FORWARD 3 SEC
  // ================================================

  forwardThreeSeconds();

  // ================================================
  // LEFT 90
  // ================================================

  turnLeft90();

  // ================================================
  // FORWARD 3 SEC
  // ================================================

  forwardThreeSeconds();

  // ================================================
  // RIGHT 90
  // ================================================

  turnRight90();

  // ================================================
  // FINAL FORWARD 3 SEC
  // ================================================

  forwardThreeSeconds();

  // ================================================
  // FINAL STOP
  // ================================================

  stopMotors();

  // LED remains ON
  digitalWrite(LED_PIN, HIGH);

  noTone(BUZZER_PIN);

  Serial.println("DETOUR COMPLETE");
  Serial.println("BOT STOPPED");

  // Remain in danger mode
  while (true) {

    stopMotors();

    digitalWrite(LED_PIN, HIGH);

    aggressiveBuzzer();

    delay(1000);
  }
}

// =====================================================
// SAFE BUZZER
// LIGHT BEEP
// =====================================================

void safeBuzzer() {

  tone(BUZZER_PIN, 1000);

  delay(50);

  noTone(BUZZER_PIN);
}

// =====================================================
// CAUTION BUZZER
// MEDIUM BEEP
// =====================================================

void cautionBuzzer() {

  tone(BUZZER_PIN, 1500);

  delay(200);

  noTone(BUZZER_PIN);
}

// =====================================================
// AGGRESSIVE BUZZER
// =====================================================

void aggressiveBuzzer() {

  for (int i = 0; i < 3; i++) {

    tone(BUZZER_PIN, 2500);

    delay(150);

    noTone(BUZZER_PIN);

    delay(80);
  }
}
