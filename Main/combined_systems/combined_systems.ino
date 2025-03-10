#include <VL53L4CD.h>
#include <Adafruit_TSL2591.h>
#include <Servo.h>
#include <Wire.h>

// Sensors and motors setup
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
VL53L4CD vl53;
Servo servoLeft;
Servo servoRight;

// Pin Definitions
#define MOTOR_LEFT_PIN 13
#define MOTOR_RIGHT_PIN 12
#define MIC_LEFT_PIN A0
#define MIC_RIGHT_PIN A1

// Thresholds
const uint16_t LIGHT_THRESHOLD = 1;
const uint16_t OBSTACLE_DISTANCE_THRESHOLD = 100;  // 10cm stop distance
const int SOUND_THRESHOLD = 130;  // Minimum sound level to trigger movement
const int MIC_DIFF_THRESHOLD = 5; // Minimum difference to turn instead of going forward
const int sampleWindow = 50;  // Sample window width in ms (50 ms = 20Hz)

// Mode selection: 0 for Light Detection, 1 for Audio Detection
int MODE = 1;

void configureTSL2591();
void Mic_Read(int& Mm, int& Ppm, int& diff);
void moveTowardSound(int MaxMic, int diff);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    servoLeft.attach(MOTOR_LEFT_PIN);
    servoRight.attach(MOTOR_RIGHT_PIN);
    
    if (!vl53.init()) {
        Serial.println("VL53L4CD not detected! Check wiring.");
        while (1);
    }
    vl53.setAddress(0x30);  // Change default address to avoid conflict
    vl53.setRangeTiming(50, 10);
    vl53.startContinuous();

    if (!tsl.begin()) {
        Serial.println("TSL2591 not detected! Check wiring.");
        while (1);
    }
    configureTSL2591();
}

void loop() {
    if (MODE == 0) {
        lightDetectionLoop();
    } else {
        audioDetectionLoop();
    }
    delay(100);
}

void lightDetectionLoop() {
    uint16_t distance = getObstacleDistance();
    if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
        stopMotors();
        Serial.println("Obstacle detected. Stopping.");
        return;
    }
    uint16_t lightLevel = getLightIntensity();
    Serial.print("Light Intensity: ");
    Serial.println(lightLevel);
    
    if (lightLevel < LIGHT_THRESHOLD) {
        Serial.println("No significant light detected. Stopping.");
        stopMotors();
    } else {
        Serial.println("Moving toward light.");
        moveForward();
    }
}

void audioDetectionLoop() {
    int MaxMic, PtPM, diff;
    Mic_Read(MaxMic, PtPM, diff);
    
    if (PtPM > SOUND_THRESHOLD) {
        moveTowardSound(MaxMic, diff);
    } else {
        Serial.println("No significant sound detected, stopping");
        stopMotors();
    }
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
    } else if (peakToPeak[1] > peakToPeak[0]) {
        maxMicrophone = 1;
        peakToPeakMax = peakToPeak[1];
    }
    Mm = maxMicrophone;
    Ppm = peakToPeakMax;
    diff = difference;
}

void moveTowardSound(int MaxMic, int diff) {
    if (diff < MIC_DIFF_THRESHOLD) {
        Serial.println("Moving forward");
        moveForward();
    } else if (MaxMic == 0) {
        Serial.println("Turning left");
        turnLeft();
    } else if (MaxMic == 1) {
        Serial.println("Turning right");
        turnRight();
    }
}

void moveForward() {
    servoLeft.writeMicroseconds(1580);
    servoRight.writeMicroseconds(1440);
}

void turnLeft() {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1440);
}

void turnRight() {
    servoLeft.writeMicroseconds(1520);
    servoRight.writeMicroseconds(1500);
}

void stopMotors() {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
}

uint16_t getObstacleDistance() {
    vl53.read();
    return vl53.ranging_data.range_mm;
}

uint16_t getLightIntensity() {
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    return tsl.calculateLux(full, ir);
}

void configureTSL2591() {
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}
