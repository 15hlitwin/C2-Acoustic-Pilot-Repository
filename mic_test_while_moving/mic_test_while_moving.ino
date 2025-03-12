#include <Servo.h>                      // Include servo library

Servo servoLeft;                        // Declare left and right servos
Servo servoRight;

const int micPin = A0;                  // Microphone connected to analog pin A0
const int micPin2 = A1;                 // Second microphone connected to analog pin A1
const int baseline = 337;               // Baseline value for amplitude calculation

int amplitude;
int amplitude2;
int difference;

enum RobotState { FORWARD, TURN_LEFT, TURN_RIGHT, BACKWARD, STOPPED };
RobotState currentState = FORWARD;

unsigned long movementStartTime = 0;    // Stores start time of each movement
const int FORWARD_TIME = 2000;          // Milliseconds to move forward
const int TURN_TIME = 600;              // Milliseconds to turn left or right
const int BACKWARD_TIME = 2000;         // Milliseconds to move backward

void setup() {
  Serial.begin(9600);                   // Start serial communication at 9600 baud rate
  servoLeft.attach(13);                 // Attach left servo to pin 13
  servoRight.attach(12);                // Attach right servo to pin 12

                 // Start by moving forward
  movementStartTime = millis();         // Record the start time of this movement
}

void loop() {
  readMicrophone();                     // Continuously read microphone values
  controlMovement();                    // Control the robot's movement based on state

}

void readMicrophone() {
  int micValue = analogRead(micPin);    // Read microphone 1 value (0-1023)
  int micValue2 = analogRead(micPin2);  // Read microphone 2 value (0-1023)
  if (micValue<16) {
    micValue=(-(micValue+16));
  }
    if (micValue2<16) {
    micValue2=(-(micValue2+16));
  }
    if (micValue>=16) {
    micValue=(micValue+16);
  }
    if (micValue2>=16) {
    micValue2=(micValue2+16);
  }
  
  amplitude = abs(micValue - baseline);
  amplitude2 = abs(micValue2 - baseline);
  difference = abs(amplitude - amplitude2);
  Serial.print(difference);
    if ((difference) > 5) {
      currentState = FORWARD;
    }

  // Send data to Serial Plotter without labels
  Serial.print(amplitude);              // First microphone amplitude
  Serial.print("\t");                   // Tab separates the values for plotting
  Serial.println(amplitude2);           // Second microphone amplitude
  
  delay(50);                            // Small delay to make output readable
  currentState = STOPPED;
}

void controlMovement() {
  unsigned long currentTime = millis(); // Get the current time

  switch (currentState) {
    case FORWARD:
      forward();
      if (currentTime - movementStartTime >= FORWARD_TIME) {
        currentState = TURN_LEFT;       // Switch to the next movement
        movementStartTime = currentTime;
      }
      break;

    case TURN_LEFT:
      turnLeft();
      if (currentTime - movementStartTime >= TURN_TIME) {
        currentState = TURN_RIGHT;      // Switch to the next movement
        movementStartTime = currentTime;
      }
      break;

    case TURN_RIGHT:
      turnRight();
      if (currentTime - movementStartTime >= TURN_TIME) {
        currentState = BACKWARD;        // Switch to the next movement
        movementStartTime = currentTime;
      }
      break;

    case BACKWARD:
      backward();
      if (currentTime - movementStartTime >= BACKWARD_TIME) {
        currentState = STOPPED;         // Switch to stop
        disableServos();
      }
      break;

    case STOPPED:
      disableServos();                  // Stop servos indefinitely
      break;
  }
}

void forward() {
  servoLeft.writeMicroseconds(1700);    // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300);   // Right wheel clockwise
  delay(FORWARD_TIME);
}

void turnLeft() {
  servoLeft.writeMicroseconds(1300);    // Left wheel clockwise
  servoRight.writeMicroseconds(1300);   // Right wheel clockwise
  delay(TURN_TIME);
}

void turnRight() {
  servoLeft.writeMicroseconds(1700);    // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700);   // Right wheel counterclockwise
  delay(TURN_TIME);
}

void backward() {
  servoLeft.writeMicroseconds(1300);    // Left wheel clockwise
  servoRight.writeMicroseconds(1700);   // Right wheel counterclockwise
  delay(BACKWARD_TIME);
}

void disableServos() {
  servoLeft.detach();                   // Stop sending servo signals
  servoRight.detach();
}

