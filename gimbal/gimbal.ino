#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

MPU6050 mpu6050(Wire);
Servo servo;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    servo.attach(9);

}

void loop()
{
  mpu6050.update();

  float Kp = 1.2;
  float Kd = 0.08;
  float Ki = 0;

  float desired = 0;
  float integral = 0;

  float angle = mpu6050.getAngleX();
  float gyro = mpu6050.getGyroX();

  float error = desired - angle;
  integral += error;
  
  float correction = (Kp * error) + (Ki * integral) - (Kd * gyro);

  float servoAngle = 90 + correction;
  servoAngle = constrain(servoAngle, 0, 180);
  servo.write(servoAngle);


  delay(20);
    
}
