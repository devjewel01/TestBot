#include <Arduino.h>

// Motor control definitions
#define PWM_FREQ 20000   // 20KHz for smoother operation
#define PWM_RES  8       // 8-bit resolution (0-255)

// Motor Driver Pin Definitions
#define LEFT_MOTOR_SPEED  14  // PWMB
#define LEFT_MOTOR_FWD    27  // BIN2
#define LEFT_MOTOR_BWD    26  // BIN1
#define RIGHT_MOTOR_SPEED 32  // PWMA
#define RIGHT_MOTOR_FWD   33  // AIN1
#define RIGHT_MOTOR_BWD   25  // AIN2
#define MOTOR_ENABLE      12  // STBY

// Encoder Pins
#define LEFT_ENC_A   4   // Left encoder C1
#define LEFT_ENC_B   2   // Left encoder C2
#define RIGHT_ENC_A  18  // Right encoder C1
#define RIGHT_ENC_B  19  // Right encoder C2

// Horn Pin
#define HORN_PIN     15  // Adjusted to actual pin

// Timing configurations
#define WATCHDOG_TIMEOUT 500    // Motor command timeout (ms)
#define ENCODER_PUBLISH_RATE 20 // Encoder update interval (ms)

// Speed Settings
#define MAX_SPEED 255

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long lastCmdTime = 0;
unsigned long lastEncoderUpdate = 0;

// Message handling
const int BUFFER_SIZE = 64;
char msgBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Function declarations
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
void initMotors();
void initEncoders();
void processCommand(char* cmd);
void moveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void sendEncoderData();
void sendStatus(const char* msg);
void sendError(const char* msg);

// Encoder interrupt handlers
void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
        leftEncoderCount--;
    } else {
        leftEncoderCount++;
    }
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
        rightEncoderCount--;
    } else {
        rightEncoderCount++;
    }
}

void initMotors() {
    // Configure motor pins
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BWD, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BWD, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);
    
    // Configure PWM channels
    ledcSetup(0, PWM_FREQ, PWM_RES);  // Channel 0 for left motor
    ledcSetup(1, PWM_FREQ, PWM_RES);  // Channel 1 for right motor
    ledcAttachPin(LEFT_MOTOR_SPEED, 0);
    ledcAttachPin(RIGHT_MOTOR_SPEED, 1);
    
    // Enable motor driver
    digitalWrite(MOTOR_ENABLE, HIGH);
    
    // Ensure motors are stopped
    stopMotors();
}

void initEncoders() {
    // Configure encoder pins with pullup
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
}

void moveMotors(int leftSpeed, int rightSpeed) {
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    // Update last command time
    lastCmdTime = millis();
    
    // Left motor
    digitalWrite(LEFT_MOTOR_FWD, leftSpeed >= 0);
    digitalWrite(LEFT_MOTOR_BWD, leftSpeed < 0);
    ledcWrite(0, abs(leftSpeed));
    
    // Right motor
    digitalWrite(RIGHT_MOTOR_FWD, rightSpeed >= 0);
    digitalWrite(RIGHT_MOTOR_BWD, rightSpeed < 0);
    ledcWrite(1, abs(rightSpeed));
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR_FWD, LOW);
    digitalWrite(LEFT_MOTOR_BWD, LOW);
    digitalWrite(RIGHT_MOTOR_FWD, LOW);
    digitalWrite(RIGHT_MOTOR_BWD, LOW);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
}

void processCommand(char* cmd) {
    char* type = strtok(cmd, ",");
    if (!type) return;
    
    if (strcmp(type, "M") == 0) {
        // Motor control command: M,left_speed,right_speed
        char* leftStr = strtok(NULL, ",");
        char* rightStr = strtok(NULL, ",");
        if (leftStr && rightStr) {
            int leftSpeed = atoi(leftStr);
            int rightSpeed = atoi(rightStr);
            moveMotors(leftSpeed, rightSpeed);
            
            // Acknowledge command
            sendStatus("Motors updated");
        } else {
            sendError("Invalid motor command format");
        }
    }
    else if (strcmp(type, "H") == 0) {
        // Horn control command: H,state
        char* stateStr = strtok(NULL, ",");
        if (stateStr) {
            digitalWrite(HORN_PIN, atoi(stateStr) ? HIGH : LOW);
            sendStatus("Horn state updated");
        }
    }
    else if (strcmp(type, "R") == 0) {
        // Reset encoders command: R
        noInterrupts();
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        interrupts();
        sendStatus("Encoders reset");
    }
    else {
        sendError("Unknown command");
    }
}

void sendEncoderData() {
    if (millis() - lastEncoderUpdate >= ENCODER_PUBLISH_RATE) {
        noInterrupts();
        long leftCount = leftEncoderCount;
        long rightCount = rightEncoderCount;
        interrupts();
        
        Serial.print("E,");
        Serial.print(leftCount);
        Serial.print(",");
        Serial.println(rightCount);
        
        lastEncoderUpdate = millis();
    }
}

void sendStatus(const char* msg) {
    Serial.print("S,");
    Serial.println(msg);
}

void sendError(const char* msg) {
    Serial.print("X,");
    Serial.println(msg);
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Initialize subsystems
    initMotors();
    initEncoders();
    pinMode(HORN_PIN, OUTPUT);
    
    sendStatus("ESP32 initialized");
}

void loop() {
    // Check for motor timeout
    if (millis() - lastCmdTime >= WATCHDOG_TIMEOUT) {
        stopMotors();
    }
    
    // Read serial commands
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n') {
            // End of message
            msgBuffer[bufferIndex] = '\0';
            processCommand(msgBuffer);
            bufferIndex = 0;
        }
        else if (bufferIndex < BUFFER_SIZE - 1) {
            msgBuffer[bufferIndex++] = c;
        }
    }
    
    // Send encoder data
    sendEncoderData();
}