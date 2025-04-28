#include <VL53L4CD.h>
#include <Adafruit_TSL2591.h>
#include <Servo.h>
#include <Wire.h>

// --- Constants & Config ---
#define LIGHT_DETECTION 1
#define AUDIO_DETECTION 2
int challengeMode = LIGHT_DETECTION;

#define MOTOR_LEFT_PIN 13
#define MOTOR_RIGHT_PIN 12
#define MIC_LEFT_PIN A1
#define MIC_RIGHT_PIN A2
#define STATUS_LED_PIN 2
#define MODE_SELECT_PIN 3

const uint16_t OBSTACLE_DISTANCE_THRESHOLD = 66;
const uint16_t LIGHT_THRESHOLD = 15;
const int SOUND_THRESHOLD = 30;  // Minimum sound to trigger move
const int MIC_DIFF_THRESHOLD = 5; // Minimum L/R diff to steer
const int MOVEMENT_DURATION = 300; // Move for at least 300ms once started
const int STOP_CONFIRMATION_COUNT = 10;
const int sampleWindow = 20;



// --- Hardware Interfaces ---
Servo servoLeft;
Servo servoRight;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
VL53L4CD vl53;

// --- State ---
bool challengeComplete = false;
int consecutiveNoObstacleCount = 0;

// --- Setup ---
void setup() {
    Serial.begin(9600);
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);

    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(MODE_SELECT_PIN, INPUT);

    Wire.begin();
    Wire.setClock(400000);
    vl53.setAddress(0x30);

    if (!vl53.init()) {
        Serial.println("VL53L4CD not detected!");
        while (1);
    }

    vl53.setRangeTiming(50, 10);
    vl53.startContinuous();

    updateMode();

    if (challengeMode == LIGHT_DETECTION) {
        setupLightSensors();
    } else {
        Serial.println("Audio detection setup complete.");
    }
}


void loop() {
    updateMode();
    if (challengeMode == LIGHT_DETECTION) {
        lightDetectionLoop();
    } else {
        audioDetectionLoop();
    }
}

// --- Mode Management ---
void updateMode() {
    int modeState = digitalRead(MODE_SELECT_PIN);
    int newMode = (modeState == HIGH) ? LIGHT_DETECTION : AUDIO_DETECTION;
    if (newMode != challengeMode) {
        challengeMode = newMode;
        Serial.print("Mode switched to: ");
        Serial.println(challengeMode == LIGHT_DETECTION ? "LIGHT" : "AUDIO");
        blinkLED(4, 100);

        // Initialize appropriate sensors when switching
        if (challengeMode == LIGHT_DETECTION) {
            setupLightSensors();
        }
    }
}


// --- Utility ---
void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(delayMs);
    }
}

// ---------------------
// Light Detection Mode
// ---------------------
void setupLightSensors() {
    if (!tsl.begin()) {
        Serial.println("TSL2591 not detected!");
        while (1);
    }

    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

    Serial.println("Light detection setup complete.");
}

bool isSpinning = true;  // Start by spinning

const int SPIN_TO_MOVE_THRESHOLD = 350;
const int MOVE_TO_SPIN_THRESHOLD = 250;

void lightDetectionLoop() {
    if (checkObstacle()) return;

    uint16_t lightLevel = getLightIntensity();
    Serial.print("Light level: "); Serial.println(lightLevel);

    if (isSpinning) {
        if (lightLevel > SPIN_TO_MOVE_THRESHOLD) {
            isSpinning = false;
        }
    } else {
        if (lightLevel < MOVE_TO_SPIN_THRESHOLD) {
            isSpinning = true;
        }
    }

    if (isSpinning) {
        int spinSpeed = map(lightLevel, 0, 50, 150, 50);
        spinSpeed = constrain(spinSpeed, 50, 150);
        spinInPlaceScaled(spinSpeed);
        Serial.print(" | SpinSpeed: "); Serial.println(spinSpeed);
    } else {
        int moveSpeed = map(lightLevel, 20, 900, 50, 8);
        moveSpeed = constrain(moveSpeed, 8, 50);
        moveForwardScaled(moveSpeed);
        Serial.print(" | MoveSpeed: "); Serial.println(moveSpeed);
    }
}






uint16_t getObstacleDistance() {
    uint16_t distance;
    do {
        vl53.read();
        distance = vl53.ranging_data.range_mm;
    } while (distance == 0);
    return distance;
}

uint16_t getLightIntensity() {
    uint32_t lum = tsl.getFullLuminosity();
    return lum & 0xFFFF;  // Only visible light
}

// ---------------------
// Audio Detection Mode 
// ---------------------

// State Machine for audio behavior
enum AudioState {
    LISTENING,
    ACTING
};

AudioState audioState = LISTENING;
unsigned long stateStartTime = 0;
int lastDiff = 0; // To remember L/R difference
int lastTotalSound = 0;

void audioDetectionLoop() {
    switch (audioState) {
        case LISTENING:
            listenForAudio();
            break;
        case ACTING:
            actOnAudio();
            break;
    }
}

void listenForAudio() {
    static int leftAccum = 0, rightAccum = 0, samples = 0;

    if (stateStartTime == 0) {
        stateStartTime = millis();
        leftAccum = 0;
        rightAccum = 0;
        samples = 0;
    }

    if (millis() - stateStartTime < 1000) { // Listen for 1 second
        int leftAmp = sampleMicrophoneAC(MIC_LEFT_PIN);
        int rightAmp = sampleMicrophoneAC(MIC_RIGHT_PIN);

        leftAccum += leftAmp;
        rightAccum += rightAmp;
        samples++;

        // Debugging the raw microphone values
        Serial.print("Left: "); Serial.print(leftAmp);
        Serial.print(" | Right: "); Serial.println(rightAmp);
    } else {
        int avgLeft = leftAccum / max(samples, 1);
        int avgRight = rightAccum / max(samples, 1);
        lastDiff = avgLeft - avgRight;
        lastTotalSound = max(avgLeft, avgRight);

        // Debugging the averages
        Serial.print("Avg Left: "); Serial.print(avgLeft);
        Serial.print(" | Avg Right: "); Serial.println(avgRight);

        audioState = ACTING;
        stateStartTime = millis();
    }
}


void actOnAudio() {
    if (stateStartTime == 0) {
        stateStartTime = millis();
    }

    if (lastTotalSound > SOUND_THRESHOLD) {
        // Map the amplitude to a movement duration
        // Base duration is 150ms for low amplitude (20), and 20ms for high amplitude (60)
        int moveDuration = map(lastTotalSound, 20, 60, 300, 20);
        moveDuration = constrain(moveDuration, 20, 300); // Ensure the duration is between 20ms and 150ms

        // If the difference between the microphones is significant, turn left or right
        if (abs(lastDiff) > MIC_DIFF_THRESHOLD) {
            if (lastDiff > 0) {
                // Turn Left
                setMotors(1400, 1500, 150);  // turn left a bit
                Serial.print("Turning Left");
            } else {
                // Turn Right
                setMotors(1500, 1600, 150);  // turn right a bit
                Serial.print("Turning Right");
            }
        }
        delay(200);
        setMotors(1600, 1400, moveDuration); // Move forward for the scaled duration
        Serial.print("Moving Forward | Duration: "); Serial.println(moveDuration);
    } else {
        stopMotors(); // No loud sound detected, stop motors
        Serial.println("No significant sound detected, stopping.");
    }
    if (checkObstacle()) return;
    audioState = LISTENING; // Return to listening state
    stateStartTime = 0; // Reset for next listen
}


// --- Microphone Sampling ---
int sampleMicrophoneAC(int pin) {
  unsigned long startMillis = millis();
  int signalMax = 0;
  int signalMin = 1023;

  while (millis() - startMillis < 20) { // 20ms sample window
    int sample = analogRead(pin);
    if (sample < 1023) {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }
  return signalMax - signalMin; // Peak-to-peak amplitude
}

// ---------------------
// Movement Control
// ---------------------

bool checkObstacle() {
    uint16_t distance = getObstacleDistance();

    if (challengeComplete) {
        if (distance >= OBSTACLE_DISTANCE_THRESHOLD) {
            if (++consecutiveNoObstacleCount >= STOP_CONFIRMATION_COUNT) {
                challengeComplete = false;
                consecutiveNoObstacleCount = 0;
                Serial.println("Obstacle cleared. Resuming.");
                blinkLED(3, 200);
            }
        } else {
            consecutiveNoObstacleCount = 0;
        }
        stopMotors();
        return true;
    }

    if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
        challengeComplete = true;
        Serial.println("Obstacle detected. Stopping.");
        blinkLED(2, 300);
        stopMotors();
        return true;
    }

    return false;
}


void spinInPlaceScaled(int duration) {
    setMotors(1500, 1475, duration); // Clockwise spin with slight bias
}

void moveForwardScaled(int duration) {
    // Compensated forward movement (your original working setup)
    setMotors(1590, 1430, duration);
    delay(50);
}


void stopMotors() {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
    servoLeft.detach();
    servoRight.detach();
    delay(50);
}

void setMotors(int left, int right, int duration) {
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    servoLeft.writeMicroseconds(left);
    servoRight.writeMicroseconds(right);
    delay(duration);
    stopMotors();
}
