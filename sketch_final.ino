// Before uploading the code, install the necessary libraries:
// NewPing Library: https://bitbucket.org/teckel12/arduino-new-ping/downloads/
// Servo Library: Included in Arduino IDE

#include <NewPing.h>
#include <Servo.h>

// Define pins for L298N Motor Controller
// Motor A (Left Motor)
const int enA = 6;    // Enable pin for motor speed control (PWM)
const int in1 = 8;    // IN1 for motor direction
const int in2 = 2;    // IN2 for motor direction

// Motor B (Right Motor)
const int enB = 5;    // Enable pin for motor speed control (PWM)
const int in3 = 4;    // IN3 for motor direction
const int in4 = 3;    // IN4 for motor direction

// Define pin for Servo motor
const int servoPin = 9;  // Servo connected to pin 9 (uses Timer1 with Servo library)

// Define pins and constants for ultrasonic sensor
#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200

// Constants
#define MAX_SPEED 80 // Sets speed of DC motors (0-255)

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

Servo myServo;

boolean goesForward = false;
int distance = 100;
int speedSet = 0;

void setup() {
  // Initialize motor control pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize motors to stopped
  moveStop();

  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(115);
  delay(2000);

  // Initial distance readings
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  int distanceR = 0;
  int distanceL = 0;
  delay(40);

  if (distance <= 40) {
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL) {
      turnRight();
      moveStop();
    } else {
      turnLeft();
      moveStop();
    }
  } else {
    moveForward();
  }
  distance = readPing();
}

int lookRight() {
  myServo.write(30);
  delay(500);
  int distance = readPing();
  delay(100);
  myServo.write(90);
  return distance;
}

int lookLeft() {
  myServo.write(150);
  delay(500);
  int distance = readPing();
  delay(100);
  myServo.write(90);
  return distance;
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  goesForward = false;
}

void moveForward() {
  if (!goesForward) {
    goesForward = true;
    // Set direction to forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    // Gradually increase speed
    for (speedSet = 0; speedSet <= MAX_SPEED; speedSet += 2) {
      analogWrite(enA, speedSet);
      analogWrite(enB, speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  // Set direction to backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Gradually increase speed
  for (speedSet = 0; speedSet <= MAX_SPEED; speedSet += 2) {
    analogWrite(enA, speedSet);
    analogWrite(enB, speedSet);
    delay(5);
  }
}

void turnRight() {
  // Left motor forward, Right motor backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set speed
  analogWrite(enA, MAX_SPEED);
  analogWrite(enB, MAX_SPEED);

  delay(500); // Adjust delay as needed for turning
  moveStop();
}

void turnLeft() {
  // Left motor backward, Right motor forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set speed
  analogWrite(enA, MAX_SPEED);
  analogWrite(enB, MAX_SPEED);

  delay(500); // Adjust delay as needed for turning
  moveStop();
}