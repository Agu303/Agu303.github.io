#include <Wire.h>

// ───────── Motor Pins ─────────
//Right motor
#define PWM_A 9  // 
#define A1    8  //
#define A2    10 //

#define PWM_B 11 // Left motor
#define B1    12
#define B2    13

// ───────── RC Input Channels ─────────
const int steeringPin = 2;
const int throttlePin = 3;
const int pumpPin = 5;
const int actuatorPin = 4;
const int motorPin = 6;

// ───────── Signal Range + Thresholds ─────────
#define PULSE_MIN  980
#define PULSE_MAX  1980
#define SPEED_MIN -255
#define SPEED_MAX  255
#define DEADZONE   20

unsigned long lastUpdate = 0;
const int updateInterval = 50;

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Master mode

  pinMode(PWM_A, OUTPUT); pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  pinMode(PWM_B, OUTPUT); pinMode(B1, OUTPUT); pinMode(B2, OUTPUT);

  pinMode(steeringPin, INPUT); pinMode(throttlePin, INPUT);
  pinMode(pumpPin, INPUT);     pinMode(actuatorPin, INPUT);
  pinMode(motorPin, INPUT);
}

void loop() {
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    int throttle, steering, pumpSignal;
    readInputs(throttle, steering, pumpSignal);

    int rightMotorSpeed, leftMotorSpeed;
    calculateMotorSpeeds(throttle, steering, rightMotorSpeed, leftMotorSpeed);
    applyMotorSpeeds(rightMotorSpeed, leftMotorSpeed);

    int actuatorSignal = pulseIn(actuatorPin, HIGH);
    int motorSignal = pulseIn(motorPin, HIGH);
    sendToSlave(actuatorSignal, motorSignal, pumpSignal);

    debugOutput(throttle, steering, rightMotorSpeed, leftMotorSpeed, pumpSignal, actuatorSignal, motorSignal);
  }
}

void readInputs(int &throttle, int &steering, int &pumpSignal) {
  throttle = pulseIn(throttlePin, HIGH);
  steering = pulseIn(steeringPin, HIGH);
  pumpSignal = pulseIn(pumpPin, HIGH);

  throttle = map(throttle, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);
  steering = map(steering, PULSE_MIN, PULSE_MAX, SPEED_MIN, SPEED_MAX);

  if (abs(throttle) < DEADZONE) throttle = 0;
  if (abs(steering) < DEADZONE) steering = 0;
}

void calculateMotorSpeeds(int throttle, int steering, int &rightMotorSpeed, int &leftMotorSpeed) {
  if (throttle == 0) {
    rightMotorSpeed = -steering;
    leftMotorSpeed = steering;
  } else {
    rightMotorSpeed = constrain(throttle - steering / 2, SPEED_MIN, SPEED_MAX);
    leftMotorSpeed  = constrain(throttle + steering / 2, SPEED_MIN, SPEED_MAX);

    if (steering == 0) {
      rightMotorSpeed = leftMotorSpeed = throttle;
    }
  }
}

void applyMotorSpeeds(int rightMotorSpeed, int leftMotorSpeed) {
  setMotor(PWM_A, A1, A2, rightMotorSpeed);
  setMotor(PWM_B, B1, B2, -leftMotorSpeed);  // Inverted for tank drive
}

void setMotor(int pwm, int in1, int in2, int speed) {
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  analogWrite(pwm, abs(speed));
}

void sendToSlave(int actuatorSignal, int motorSignal, int pumpSignal) {
  Wire.beginTransmission(8); // Nano's I2C address

  Wire.write(highByte(actuatorSignal));
  Wire.write(lowByte(actuatorSignal));
  Wire.write(highByte(motorSignal));
  Wire.write(lowByte(motorSignal));
  Wire.write(highByte(pumpSignal));
  Wire.write(lowByte(pumpSignal));

  Wire.endTransmission();
}

void debugOutput(int throttle, int steering, int rightSpeed, int leftSpeed,
                 int pump, int actuator, int motor) {
  Serial.print("Throttle: "); Serial.print(throttle);
  Serial.print(" | Steering: "); Serial.print(steering);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.print(rightSpeed);
  Serial.print(" | Pump: "); Serial.print(pump);
  Serial.print(" | Act: "); Serial.print(actuator);
  Serial.print(" | Motor: "); Serial.println(motor);
}
