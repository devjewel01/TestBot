#include <Arduino.h>

// PWM configurations
#define PWM_FREQ 5000    // 5 KHz
#define PWM_RES  8       // 8-bit resolution (0-255)

// Motor Driver Pin Definitions
// Left Motor Pins
#define LEFT_MOTOR_SPEED  14  // PWMB
#define LEFT_MOTOR_FWD    27  // BIN2
#define LEFT_MOTOR_BWD    26  // BIN1

// Right Motor Pins
#define RIGHT_MOTOR_SPEED 32  // PWMA
#define RIGHT_MOTOR_FWD   33  // AIN1
#define RIGHT_MOTOR_BWD   25  // AIN2

#define MOTOR_ENABLE     12  // STBY

// Encoder Pins
#define LEFT_ENC_F   4   // Left encoder C1
#define LEFT_ENC_B   2   // Left encoder C2
#define RIGHT_ENC_F  18  // Right encoder C1
#define RIGHT_ENC_B  19  // Right encoder C2

// Global variables for encoder counting
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Function declarations
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_F) == digitalRead(LEFT_ENC_B)) {
    leftEncoderCount--;  
  } else {
    leftEncoderCount++;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_F) == digitalRead(RIGHT_ENC_B)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void setup() {
  Serial.begin(115200);
 
  // Initialize Motor Driver Pins
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_BWD, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_BWD, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
 
  // Configure PWM for motors
  if (!ledcAttach(LEFT_MOTOR_SPEED, PWM_FREQ, PWM_RES)) {
    Serial.println("Failed to attach Left Motor PWM!");
  }
  if (!ledcAttach(RIGHT_MOTOR_SPEED, PWM_FREQ, PWM_RES)) {
    Serial.println("Failed to attach Right Motor PWM!");
  }
 
  // Initialize Encoder Pins
  pinMode(LEFT_ENC_F, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_F, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
 
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_F), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_F), rightEncoderISR, CHANGE);
 
  // Enable motor driver
  digitalWrite(MOTOR_ENABLE, HIGH);
 
  Serial.println("\nMotor Test Program");
  Serial.println("Commands:");
  Serial.println("1: Left Motor Forward");
  Serial.println("2: Left Motor Backward");
  Serial.println("3: Right Motor Forward");
  Serial.println("4: Right Motor Backward");
  Serial.println("s: Stop Motors");
  Serial.println("r: Reset Encoder Counts");
}

void loop() {
  static unsigned long lastPrint = 0;
 
  // Print encoder values every 200ms
  if (millis() - lastPrint >= 200) {
    Serial.print("Left Encoder: ");
    Serial.print(leftEncoderCount);
    Serial.print(" | Right Encoder: ");
    Serial.println(rightEncoderCount);
    lastPrint = millis();
  }
 
  if (Serial.available()) {
    char cmd = Serial.read();
    int speed = 128;  // 50% speed for testing
   
    switch (cmd) {
      case '1':  // Left Forward
        Serial.println("\nLeft Motor Forward");
        digitalWrite(LEFT_MOTOR_FWD, HIGH);
        digitalWrite(LEFT_MOTOR_BWD, LOW);
        analogWrite(LEFT_MOTOR_SPEED, speed);
        break;
       
      case '2':  // Left Backward
        Serial.println("\nLeft Motor Backward");
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BWD, HIGH);
        analogWrite(LEFT_MOTOR_SPEED, speed);
        break;
       
      case '3':  // Right Forward
        Serial.println("\nRight Motor Forward");
        digitalWrite(RIGHT_MOTOR_FWD, HIGH);
        digitalWrite(RIGHT_MOTOR_BWD, LOW);
        analogWrite(RIGHT_MOTOR_SPEED, speed);
        break;
       
      case '4':  // Right Backward
        Serial.println("\nRight Motor Backward");
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BWD, HIGH);
        analogWrite(RIGHT_MOTOR_SPEED, speed);
        break;
       
      case 's':  // Stop
        Serial.println("\nStopping Motors");
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BWD, LOW);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BWD, LOW);
        analogWrite(LEFT_MOTOR_SPEED, 0);
        analogWrite(RIGHT_MOTOR_SPEED, 0);
        break;
       
      case 'r':  // Reset encoders
        Serial.println("\nResetting Encoder Counts");
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        break;
    }
  }
}
