#include <Servo.h>

// Define motor driver pins
const int IN1 = 10;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;  
const int ENA = 9; // PWM pin for motor A speed control
const int ENB = 10; // PWM pin for motor B speed control

// Define ultrasonic sensor pins
const int trigPin = 8; 
const int echoPin = 7; 

// Define servo motor
Servo myServo; 
const int servoPin = 6;

// Variables to store distance and servo angle
long duration;
int distance;
int angle;
const int thresholdDistance = 20; // Distance to detect obstacle (in cm)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set up the ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Attach the servo to the pin
  myServo.attach(servoPin);
  
  // Move servo to initial position (center)
  myServo.write(90);

  // Debugging: confirm setup
  Serial.println("Setup complete. Starting loop...");
}

void loop() {
  // Get the distance from the ultrasonic sensor
  distance = getDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check if there is an obstacle within the threshold distance
  if (distance < thresholdDistance) {
    // Obstacle detected, stop motors
    stopMotors();
    Serial.println("Obstacle detected! Stopping motors.");

    // Scan with servo to find clear path
    int clearDirection = scanForClearPath();
    Serial.print("Clear direction angle: ");
    Serial.println(clearDirection);

    // Adjust movement direction based on the clear path
    if (clearDirection == 90) {
      // Move backward if both sides are blocked
      moveBackward();
      Serial.println("Both sides blocked, moving backward.");
      delay(1000); // Move backward for 1 second
    } else if (clearDirection < 90) {
      // Turn left
      moveLeft();
      Serial.println("Turning left.");
      delay(1000); // Turn left for 1 second
    } else if (clearDirection > 90) {
      // Turn right
      moveRight();
      Serial.println("Turning right.");
      delay(1000); // Turn right for 1 second
    }

    moveForward();
  } else {
    // No obstacle, move forward
    moveForward();
    Serial.println("Moving forward.");
  }

  // Return servo to initial position (center)
  myServo.write(90);

  delay(200); // Wait a bit before the next reading
}

// Function to get the distance from the ultrasonic sensor
int getDistance() {
  // Send a 10us pulse to trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (in cm)
  int distance = duration * 0.034 / 2;
  
  return distance;
}

// Function to scan for a clear path using the servo
int scanForClearPath() {


  int leftDistance, rightDistance;

  // Look to the left (0 degrees)
  myServo.write(0);
  delay(500); // Allow time for servo to move
  leftDistance = getDistance();
  Serial.print("Left distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");

  // Look to the right (180 degrees)
  myServo.write(180);
  delay(1000); // Allow time for servo to move
  rightDistance = getDistance();
  Serial.print("Right distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Return to center (90 degrees)
  myServo.write(90);
  delay(500); // Allow time for servo to move

  // Determine the clearer path
  if (leftDistance < thresholdDistance && rightDistance < thresholdDistance) {
    return 90; // Both sides are blocked, signal to move backward
  } else if (leftDistance > rightDistance) {
    return 0; // Clearer to the left
  } else {
    return 180; // Clearer to the right
  }
}

// Motor control functions
void moveForward() {
  // Set motors to move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, 150); // Full speed for motor A
  analogWrite(ENB, 150); // Full speed for motor B
}

void moveBackward() {
  // Set motors to move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set speed
  analogWrite(ENA, 150); // Full speed for motor A
  analogWrite(ENB, 150); // Full speed for motor B
}

void moveLeft() {
  // Set motors to turn left
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, 150); // Full speed for motor A
  analogWrite(ENB, 150); // Full speed for motor B
}

void moveRight() {
  // Set motors to turn right
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set speed
  analogWrite(ENA, 150); // Full speed for motor A
  analogWrite(ENB, 150); // Full speed for motor B
}

void stopMotors() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Set speed to zero
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
