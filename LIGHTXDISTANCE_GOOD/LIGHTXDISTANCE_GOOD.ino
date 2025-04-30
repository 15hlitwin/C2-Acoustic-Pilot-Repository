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
#define MIC_LEFT_PIN A0
#define MIC_RIGHT_PIN A1
#define STATUS_LED_PIN 2
#define MODE_SELECT_PIN 3

uint16_t obstacleThresholdAudio = 100;
uint16_t obstacleThresholdLight = 67; 

const uint16_t LIGHT_THRESHOLD = 15;
const int SOUND_THRESHOLD = 10;  // Minimum sound to trigger move
const int MIC_DIFF_THRESHOLD = 3; // Minimum L/R diff to steer
const int STOP_CONFIRMATION_COUNT = 10;



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
    if (checkObstacle()) return;
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

void lightDetectionLoop() {
    uint16_t lightLevel = getLightIntensity();
    Serial.print("Light level: "); Serial.println(lightLevel);

    const uint16_t LIGHT_DETECTION_THRESHOLD = 350;  // Adjust as needed

    if (lightLevel < LIGHT_DETECTION_THRESHOLD) {
        // Spin slowly in place while looking for light
        Serial.println("Spinning to find light...");
        setMotors(1500, 1480, 150);  // Gentle spin
    } else {
        // Move forward at fixed speed if light is detected
        Serial.println("Bright light detected. Moving forward.");
        setMotors(1540, 1480, 100);  // Move forward
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

// --- Microphone Sampling ---
int sampleMicAmplitude(int pin, int baseline = 512, int samples = 20) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    int raw = analogRead(pin);
    sum += abs(raw - baseline);
    delay(1);  // Optional: ensures distinct samples
  }
  return sum / samples;
}

void listenForAudio() {
    static int leftSum = 0, rightSum = 0;
    static int samples = 0;

    if (stateStartTime == 0) {
        stateStartTime = millis();
        leftSum = 0;
        rightSum = 0;
        samples = 0;
    }

    if (millis() - stateStartTime < 400) {
        int leftAmp = sampleMicAmplitude(MIC_LEFT_PIN);
        int rightAmp = sampleMicAmplitude(MIC_RIGHT_PIN);


        leftSum += leftAmp;
        rightSum += rightAmp;
        samples++;

        //Serial monitor debugging
        //Serial.print("Left: "); Serial.print(leftAmp);
        //Serial.print(" | Right: "); Serial.println(rightAmp);

        //Graphing for debugging
        //Serial.print(leftAmp);
        //Serial.print(",");
        //Serial.println(rightAmp);
    } else {
        int avgLeft = leftSum / max(samples, 1);
        int avgRight = rightSum / max(samples, 1);
        lastDiff = avgLeft - avgRight;
        lastTotalSound = (avgLeft + avgRight) / 2;

        Serial.print("Avg Left: "); Serial.print(avgLeft);
        Serial.print(" | Avg Right: "); Serial.print(avgRight);
        Serial.print(" | Total Avg: "); Serial.print(lastTotalSound);
        Serial.print(" | Last Diff "); Serial.println(lastDiff);

        audioState = ACTING;
        stateStartTime = 0;
    }
}


void actOnAudio() {
    if (lastTotalSound > SOUND_THRESHOLD) {
        if (abs(lastDiff) > MIC_DIFF_THRESHOLD) {
            if (lastDiff > 0) {
                Serial.println("Sound stronger on LEFT. Turning Left.");
                setMotors(1400, 1500, 200);  // Left turn
            } else {
                Serial.println("Sound stronger on RIGHT. Turning Right.");
                setMotors(1500, 1600, 200);  // Right turn
            }
        } else {
            Serial.println("Sound centered. No turn.");
        }

        int moveDuration = map(lastTotalSound, 70, 400, 150, 600);
        moveDuration = constrain(moveDuration, 150, 600);
        Serial.print("Moving forward. Duration: "); Serial.println(moveDuration);
        setMotors(1590, 1430, moveDuration);
    } else {
        Serial.println("Quiet... Stopping.");
        stopMotors();
    }

    audioState = LISTENING;
}


// ---------------------
// Movement Control
// ---------------------

bool checkObstacle() {
    uint16_t distance = getObstacleDistance();
    uint16_t currentThreshold = (challengeMode == AUDIO_DETECTION) ? obstacleThresholdAudio : obstacleThresholdLight;

    if (challengeComplete) {
        if (distance >= currentThreshold) {
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

    if (distance < currentThreshold) {
        challengeComplete = true;
        Serial.println("Obstacle detected. Stopping.");
        blinkLED(2, 300);
        stopMotors();
        return true;
    }

    return false;
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
