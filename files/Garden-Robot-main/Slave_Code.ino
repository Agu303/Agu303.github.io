#include <Wire.h> 

// ───── Pin Map ─────
#define ACTUATOR_PWM 3
#define ACTUATOR_IN1 5
#define ACTUATOR_IN2 6

#define MOTOR_ENA 9   // PWM-capable pin

#define PUMP_A_ENA 10
#define PUMP_B_ENA 11

// ───── Thresholds ─────
int prevActSignal = -1;
const int deadzone = 10;

// ───── Debug ─────
#define DEBUG_ISR 1

void setup() {
  Serial.begin(9600);
  Wire.begin(8);  // I2C slave address
  Wire.onReceive(receiveEvent);

  pinMode(ACTUATOR_PWM, OUTPUT);
  pinMode(ACTUATOR_IN1, OUTPUT);
  pinMode(ACTUATOR_IN2, OUTPUT);

  pinMode(MOTOR_ENA , OUTPUT);
  pinMode(PUMP_A_ENA, OUTPUT);
  pinMode(PUMP_B_ENA, OUTPUT);

  // Default state: all off
  analogWrite(ACTUATOR_PWM, 0);
  digitalWrite(ACTUATOR_IN1, LOW);
  digitalWrite(ACTUATOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(PUMP_A_ENA, 0);
  analogWrite(PUMP_B_ENA, 0);
}

void loop() {
  // All logic in I²C ISR
}

void receiveEvent(int howMany) {
  if (howMany < 6) return;

  int actSignal   = (Wire.read() << 8) | Wire.read();
  int motorSignal = (Wire.read() << 8) | Wire.read();
  int pumpSignal  = (Wire.read() << 8) | Wire.read();

#if DEBUG_ISR
  Serial.print("act="); Serial.print(actSignal);
  Serial.print(" mot="); Serial.print(motorSignal);
  Serial.print(" pump="); Serial.println(pumpSignal);
#endif

  // ───── 12V Motor ─────
  if (motorSignal > 1500) {
    analogWrite(MOTOR_ENA, 255);
  } else {
    analogWrite(MOTOR_ENA, 0);
  }

  // ───── Pumps ─────
  bool pumpON = pumpSignal > 1500;
  analogWrite(PUMP_A_ENA, pumpON ? 255 : 0);
  analogWrite(PUMP_B_ENA, pumpON ? 255 : 0);

  // ───── Actuator Control ─────
  if (actSignal < 1010) {
    digitalWrite(ACTUATOR_IN1, LOW);
    digitalWrite(ACTUATOR_IN2, HIGH);
    analogWrite(ACTUATOR_PWM, 255);
  } else if (actSignal > 1980) {
    digitalWrite(ACTUATOR_IN1, HIGH);
    digitalWrite(ACTUATOR_IN2, LOW);
    analogWrite(ACTUATOR_PWM, 255);
  } else {
    if (prevActSignal == -1) {
      prevActSignal = actSignal;
      analogWrite(ACTUATOR_PWM, 0);
    } else {
      int d = actSignal - prevActSignal;
      if (d > deadzone) {
        digitalWrite(ACTUATOR_IN1, HIGH);
        digitalWrite(ACTUATOR_IN2, LOW);
        analogWrite(ACTUATOR_PWM, 255);
      } else if (d < -deadzone) {
        digitalWrite(ACTUATOR_IN1, LOW);
        digitalWrite(ACTUATOR_IN2, HIGH);
        analogWrite(ACTUATOR_PWM, 255);
      } else {
        analogWrite(ACTUATOR_PWM, 0);
      }
      prevActSignal = actSignal;
    }
  }

#if DEBUG_ISR
  Serial.print(" ACT="); Serial.print(digitalRead(ACTUATOR_PWM));
  Serial.print(" MOT="); Serial.print(analogRead(MOTOR_ENA) > 0 ? "1" : "0");
  Serial.print(" PA=");  Serial.print(analogRead(PUMP_A_ENA) > 0 ? "1" : "0");
  Serial.print(" PB=");  Serial.println(analogRead(PUMP_B_ENA) > 0 ? "1" : "0");
#endif
}
