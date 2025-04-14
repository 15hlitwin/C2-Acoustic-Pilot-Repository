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
const uint16_t LIGHT_THRESHOLD = 40;
const int MIC_THRESHOLD = 20;
const int SOUND_THRESHOLD = 120;
const int STOP_CONFIRMATION_COUNT = 10;
const int sampleWindow = 20;

int recentMaxSound = SOUND_THRESHOLD + 1; // Start with something valid


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
    Serial.begin(115200);
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

void lightDetectionLoop() {
    if (checkObstacle()) return;

    uint16_t lightLevel = getLightIntensity();
    if (lightLevel < LIGHT_THRESHOLD) {
        Serial.println("Low light. Spinning.");
        spinInPlace();
    } else {
        Serial.println("Light detected. Moving.");
        moveForward();
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
void audioDetectionLoop() {
    if (checkObstacle()) return;

    int MaxMic, PtPM, diff;
    readMicrophones(MaxMic, PtPM, diff);

    if (PtPM > SOUND_THRESHOLD) {
        moveTowardSound(MaxMic, diff);
    } else {
        Serial.println("No sound detected.");
        stopMotors();
    }
}


void readMicrophones(int& maxMic, int& peakToPeakMax, int& diff) {
    unsigned long startMillis = millis();
    unsigned int signalMax[2] = {0, 0}, signalMin[2] = {1024, 1024};

    while (millis() - startMillis < sampleWindow) {
        for (int i = 0; i < 2; i++) {
            int sample = analogRead(MIC_LEFT_PIN + i);
            if (sample < 1024) {
                if (sample > signalMax[i]) signalMax[i] = sample;
                if (sample < signalMin[i]) signalMin[i] = sample;
            }
        }
    }

    unsigned int p2p[2] = {signalMax[0] - signalMin[0], signalMax[1] - signalMin[1]};
    maxMic = (p2p[0] > p2p[1]) ? 0 : 1;
    peakToPeakMax = max(p2p[0], p2p[1]);
    diff = abs((int)p2p[0] - (int)p2p[1]);

    Serial.print("Mic A1: "); Serial.print(p2p[0]);
    Serial.print(" | Mic A2: "); Serial.print(p2p[1]);
    Serial.print(" | Diff: "); Serial.print(diff);
    Serial.print(" | Max Mic: "); Serial.println(maxMic == 0 ? "A1" : "A2");

    if (peakToPeakMax > recentMaxSound) {
    recentMaxSound = peakToPeakMax;
    }

    // Optional: decay recent max over time to prevent it sticking forever
    recentMaxSound = max(recentMaxSound - 1, SOUND_THRESHOLD + 1);
}

int calculateDurationFromSound(int peakToPeak) {
    const int MIN_DURATION = 30;
    const int MAX_DURATION = 120;

    // Use recentMaxSound for dynamic scaling
    int normalized = map(peakToPeak, SOUND_THRESHOLD, recentMaxSound, MAX_DURATION, MIN_DURATION);
    return constrain(normalized, MIN_DURATION, MAX_DURATION);
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


void moveForward() {
    setMotors(1590, 1420, 30);
}

void spinInPlace() {
    setMotors(1500, 1475, 150);
}

void moveTowardSound(int maxMic, int diff) {
    int duration = calculateDurationFromSound(max(diff, SOUND_THRESHOLD)); // Using diff or P2P here is your choice

    if (diff < MIC_THRESHOLD) {
        setMotors(1580, 1420, duration);
    } else if (maxMic == 1) {
        setMotors(1500, 1420, duration);
    } else {
        setMotors(1580, 1500, duration);
    }

    Serial.print("Moving with duration: ");
    Serial.println(duration);
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
