/*
 * PROJECT: 2-DOF SCARA Robot Arm - Firmware
 * AUTHOR: Jorge Gael Santiago Simón
 * DESCRIPTION:
 * Receives target coordinates (X, Y) via Serial (USB/Bluetooth) 
 * and calculates servo angles using Inverse Kinematics (Law of Cosines).
 * Designed for SG90 micro servos and lightweight links.
 * * PROTOCOL:
 * Expects string format: "X{val}Y{val}" 
 * Where val is an integer between 0-100 representing relative position.
 * Example: "X50Y50" (Center)
 */

#include <Servo.h>
#include <math.h>

// --- ROBOT PHYSICAL CONFIGURATION ---
// IMPORTANT: Measure your arm lengths (from pivot to pivot) in cm
const float L1 = 7.0; // Shoulder to Elbow length
const float L2 = 7.65; // Elbow to End-Effector length

// --- PIN DEFINITIONS ---
const int PIN_SERVO_SHOULDER = 9;
const int PIN_SERVO_ELBOW    = 10;

// --- SAFETY LIMITS (Workspace) ---
const float MIN_DISTANCE = 5.0;  // Minimum distance from base (cm) to avoid self-collision
const float MAX_REACH    = L1 + L2 - 1.0; // Maximum reach (cm) with safety margin

// --- GLOBALS ---
Servo shoulder;
Servo elbow;

// Current state (in real-world coordinates cm)
float currentX = 0.0;
float currentY = 15.0; // Start position (Forward)

// Incoming control values (0-100 range)
int inputX = 50; 
int inputY = 50;

// Movement speed factor (Higher = Faster)
float speedFactor = 0.5;

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  
  // Attach Servos
  shoulder.attach(PIN_SERVO_SHOULDER);
  elbow.attach(PIN_SERVO_ELBOW);
  
  // Move to Home Position
  updateArmPosition(currentX, currentY);
  
  // Ready signal (Led Blink)
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); delay(200); digitalWrite(13, LOW);
}

void loop() {
  // --- SERIAL PACKET PARSING ---
  // Non-blocking serial read
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Parse X Value
    if (cmd == 'X') {
      inputX = Serial.parseInt();
    }
    // Parse Y Value and Update
    else if (cmd == 'Y') {
      inputY = Serial.parseInt();
      processInput(); // Execute movement after receiving full pair
    }
  }
}

/**
 * processInput()
 * Converts the abstract input (0-100) into physical coordinates (cm)
 * and applies simple interpolation for smooth movement.
 */
void processInput() {
  // 1. Mapping Input (Joystick Logic)
  // X: 0-100 -> Direction control
  // Y: 0-100 -> Depth control
  
  // Thresholds (Deadzone) to prevent jitter
  if (inputX < 40) currentX -= speedFactor;
  else if (inputX > 60) currentX += speedFactor;
  
  if (inputY < 40) currentY -= speedFactor;
  else if (inputY > 60) currentY += speedFactor;

  // 2. Physical Constraints (The Canvas)
  // Constrain X axis (Left/Right limit)
  currentX = constrain(currentX, -15.0, 15.0);
  
  // Constrain Y axis (Near/Far limit)
  currentY = constrain(currentY, MIN_DISTANCE, 22.0);

  // 3. Execute Kinematics
  updateArmPosition(currentX, currentY);
}

/**
 * updateArmPosition(x, y)
 * Performs Inverse Kinematics calculations to drive servos.
 * Uses Law of Cosines to solve the triangle formed by L1, L2 and distance to target.
 */
void updateArmPosition(float x, float y) {
  // Calculate distance from base to target
  float dist = sqrt(x*x + y*y);

  // Safety Check: Is the point reachable?
  if (dist > MAX_REACH || dist < MIN_DISTANCE) {
    return; // Ignore unreachable points
  }

  // --- INVERSE KINEMATICS ---
  
  // 1. Calculate Elbow Angle (Theta 2) using Law of Cosines
  // c^2 = a^2 + b^2 - 2ab*cos(C)
  float cosTheta2 = (dist*dist - L1*L1 - L2*L2) / (2 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0); // Floating point safety
  float angle2Rad = acos(cosTheta2);
  
  // 2. Calculate Shoulder Angle (Theta 1)
  // Uses atan2 for full quadrant support
  float angle1Rad = atan2(y, x) - atan2((L2 * sin(angle2Rad)), (L1 + L2 * cos(angle2Rad)));

  // 3. Convert Radians to Degrees
  float theta1 = angle1Rad * (180.0 / PI);
  float theta2 = angle2Rad * (180.0 / PI);

  // 4. Drive Servos (with physical limits)
  shoulder.write(constrain(theta1, 0, 180));
  elbow.write(constrain(theta2, 0, 180));
}