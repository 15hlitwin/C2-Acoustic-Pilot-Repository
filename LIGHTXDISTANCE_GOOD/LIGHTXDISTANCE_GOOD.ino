#include <VL53L4CD.h>
#include <Adafruit_TSL2591.h>
#include <Servo.h>
#include <Wire.h>

// Challenge mode selector
#define LIGHT_DETECTION 1
#define AUDIO_DETECTION 2
int challengeMode = LIGHT_DETECTION;  // Set to LIGHT_DETECTION or AUDIO_DETECTION

// --- Common Setup ---
Servo servoLeft;
Servo servoRight;

// Pins
#define MOTOR_LEFT_PIN 13
#define MOTOR_RIGHT_PIN 12
#define MIC_LEFT_PIN A1
#define MIC_RIGHT_PIN A2
#define STATUS_LED_PIN 2  // Status LED for messages
#define MODE_SELECT_PIN 3 // Mode select switch (with LED in series)

// --- Light Detection Setup ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
VL53L4CD vl53;

const uint16_t OBSTACLE_DISTANCE_THRESHOLD = 66;  // 10cm stop distance
const uint16_t LIGHT_THRESHOLD = 40;

void configureTSL2591() {
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}

// --- Audio Detection Setup ---
const int sampleWindow = 50;  // Sample window in ms
const int MIC_THRESHOLD = 5;
const int SOUND_THRESHOLD = 130;

// --- Setup ---
void setup() {
    Serial.begin(115200);
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);

    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(MODE_SELECT_PIN, INPUT);

    updateMode();  // Set initial mode based on D3

    if (challengeMode == LIGHT_DETECTION) {
        Wire.begin();
        Wire.setClock(400000);
        vl53.setAddress(0x30);

        if (!vl53.init()) {
            Serial.println("VL53L4CD not detected! Check wiring.");
            while (1);
        }
        vl53.setRangeTiming(50, 10);
        vl53.startContinuous();

        if (!tsl.begin()) {
            Serial.println("TSL2591 not detected! Check wiring.");
            while (1);
        }
        configureTSL2591();
        Serial.println("Light detection setup complete.");
    } else {
        Serial.println("Audio detection setup complete.");
    }
}

// --- Mode Switching Logic ---
void updateMode() {
    int modeState = digitalRead(MODE_SELECT_PIN);
    int newMode = (modeState == HIGH) ? LIGHT_DETECTION : AUDIO_DETECTION;

    if (newMode != challengeMode) {
        challengeMode = newMode;
        Serial.print("Mode switched to: ");
        Serial.println(challengeMode == LIGHT_DETECTION ? "LIGHT_DETECTION" : "AUDIO_DETECTION");

        blinkLED(4, 100);  // Indicate mode switch
    }
}

// --- LED Message Display ---
void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(delayMs);
    }
}

// --- Main Loop ---
void loop() {
    updateMode();  // Check if mode switch occurred

    if (challengeMode == LIGHT_DETECTION) {
        lightDetectionLoop();
    } else if (challengeMode == AUDIO_DETECTION) {
        audioDetectionLoop();
    }
}

// --- Light Detection Logic ---
bool challengeComplete = false;  // Flag to stop further movements
int consecutiveNoObstacleCount = 0;  // Count for consistent obstacle absence
const int STOP_CONFIRMATION_COUNT = 10;  // Consecutive readings needed to confirm movement

void lightDetectionLoop() {
    uint16_t distance = getObstacleDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    if (challengeComplete) {
        if (distance >= OBSTACLE_DISTANCE_THRESHOLD) {
            consecutiveNoObstacleCount++;
            if (consecutiveNoObstacleCount >= STOP_CONFIRMATION_COUNT) {
                Serial.println("Obstacle cleared consistently. Resuming movement.");
                challengeComplete = false;
                consecutiveNoObstacleCount = 0;
                blinkLED(3, 200);  // 3 short blinks for resuming
            }
        } else {
            consecutiveNoObstacleCount = 0;
        }
        stopMotors();
        return;
    }

    if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
        Serial.println("Obstacle detected. Stopping immediately.");
        stopMotors();
        challengeComplete = true;
        blinkLED(2, 300);  // 2 fast blinks for obstacle detected
        return;
    }

    uint16_t lightLevel = getLightIntensity();
    Serial.print("Light Intensity: ");
    Serial.println(lightLevel);

    if (lightLevel < LIGHT_THRESHOLD) {
        Serial.println("No significant light. Spinning.");
        spinInPlace();
    } else {
        Serial.println("Light detected. Moving forward.");
        moveForward();
    }
    delay(20);
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
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    return full;
}

void moveForward() {
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    
    servoLeft.writeMicroseconds(1590);
    servoRight.writeMicroseconds(1420);
    delay(30);
    
    stopMotors();
}

void spinInPlace() {
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1480);
    delay(150);
    
    stopMotors();
}

void stopMotors() {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
    servoLeft.detach();
    servoRight.detach();
    delay(50);
}

// --- Audio Detection Logic ---
void audioDetectionLoop() {
    int MaxMic, PtPM, diff;
    Mic_Read(MaxMic, PtPM, diff);

    if (PtPM > SOUND_THRESHOLD) {
        moveTowardSound(MaxMic, diff);
    } else {
        Serial.println("No significant sound detected, stopping");
        stopMotors();
    }

    delay(50);
}

void Mic_Read(int& Mm, int& Ppm, int& diff) {
    unsigned long startMillis = millis();
    unsigned int peakToPeakMax = 0;
    int maxMicrophone = -1;

    unsigned int signalMax[2] = {0, 0};
    unsigned int signalMin[2] = {1024, 1024};

    while (millis() - startMillis < sampleWindow) {
        for (int i = 0; i < 2; i++) {
            int sample = analogRead(MIC_LEFT_PIN + i);
            if (sample < 1024) {
                if (sample > signalMax[i]) signalMax[i] = sample;
                if (sample < signalMin[i]) signalMin[i] = sample;
            }
        }
    }

    unsigned int peakToPeak[2] = {signalMax[0] - signalMin[0], signalMax[1] - signalMin[1]};
    int difference = abs((int)peakToPeak[0] - (int)peakToPeak[1]);

    Serial.print("Mic_A1: ");
    Serial.print(peakToPeak[0]);
    Serial.print("\tMic_A2: ");
    Serial.print(peakToPeak[1]);
    Serial.print("\tDifference: ");
    Serial.print(difference);
    Serial.print("\tLoudest Mic: ");
    Serial.print(maxMicrophone == 0 ? "A1" : "A2");
    Serial.print("\tPeak-to-Peak Max: ");
    Serial.println(peakToPeakMax);

    Mm = (peakToPeak[0] > peakToPeak[1]) ? 0 : 1;
    Ppm = max(peakToPeak[0], peakToPeak[1]);
    diff = difference;
}

void moveTowardSound(int MaxMic, int diff) {
    if (diff < MIC_THRESHOLD) {
        servoLeft.writeMicroseconds(1580);
        servoRight.writeMicroseconds(1420);
    } else if (MaxMic == 0) {
        servoLeft.writeMicroseconds(1500);
        servoRight.writeMicroseconds(1420);
    } else if (MaxMic == 1) {
        servoLeft.writeMicroseconds(1580);
        servoRight.writeMicroseconds(1500);
    }
}
