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

// --- Light Detection Setup ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
VL53L4CD vl53;

const uint16_t OBSTACLE_DISTANCE_THRESHOLD = 80;  // 10cm stop distance
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

// --- Combined Loop ---
void loop() {
    if (challengeMode == LIGHT_DETECTION) {
        lightDetectionLoop();
    } else if (challengeMode == AUDIO_DETECTION) {
        audioDetectionLoop();
    }
}

// --- Light Detection Logic ---
void lightDetectionLoop() {
    uint16_t distance = getObstacleDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
        stopMotors();
        Serial.println("Obstacle detected. Stopping.");
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
    return full; //tsl.calculateLux(full, ir);
}

void moveForward() {
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    
    // Move for a short time (adjust as needed)
    servoLeft.writeMicroseconds(1540);
    servoRight.writeMicroseconds(1450);
    delay(150);  // Move for 150ms, then stop
    
    stopMotors();  // Immediately stop after movement
}

void spinInPlace() {
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    
    // Short spin movement
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1480);
    delay(150);  // Adjust for desired spin time
    
    stopMotors();
}

void stopMotors() {
    servoLeft.writeMicroseconds(1500);  // Neutral signal to stop movement
    servoRight.writeMicroseconds(1500);
    delay(50);  // Allow servos to fully stop before next command
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

    delay(50);  // Same sampling window as audio detection system
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

    if (peakToPeak[0] > peakToPeak[1]) {
        maxMicrophone = 0;
        peakToPeakMax = peakToPeak[0];
    } else {
        maxMicrophone = 1;
        peakToPeakMax = peakToPeak[1];
    }

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

    Mm = maxMicrophone;
    Ppm = peakToPeakMax;
    diff = difference;
}

void moveTowardSound(int MaxMic, int diff) {
    if (diff < MIC_THRESHOLD) {
        Serial.println("Moving forward");
        servoLeft.writeMicroseconds(1580);
        servoRight.writeMicroseconds(1420);
    } else if (MaxMic == 0) {
        Serial.println("Turning left");
        servoLeft.writeMicroseconds(1500);
        servoRight.writeMicroseconds(1420);
    } else if (MaxMic == 1) {
        Serial.println("Turning right");
        servoLeft.writeMicroseconds(1580);
        servoRight.writeMicroseconds(1500);
    }
}

