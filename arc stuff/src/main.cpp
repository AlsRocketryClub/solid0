#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_ISM330DHCX.h"
#include "LittleFS.h"
#include <Math.h>

enum RocketStatus {CALIBRATION, LAUNCH, FALL, DEPLOY_PARACHUTE, SHOOT_DEPLOY, LANDING, LANDED};

Adafruit_ISM330DHCX ism;
const char* fileName = "/fire.csv";
const int buttonPin = ;
const int triggerPin = 25;
const int parachuteDeployPin = 24;
float initialTiltX = 0;
float initialTiltY = 0;
float initialVelocity = 0;
float initialHeight = 0;
unsigned long lastRecordTime = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(triggerPin, OUTPUT);
  pinMode(parachuteDeployPin, OUTPUT);
  digitalWrite(parachuteDeployPin, LOW);

  if (!ism.begin_I2C()) {
    Serial.println("Sensor initialization failed!");
    while(1);
  }

  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialization failed!");
    while(1);
  }
}

void loop() {
  RocketStatus status = getStatus();
  if (status == LAUNCH) {
    if (initialHeight == 0) {
      initialHeight = calculateHeight(); // Record initial height at launch
    }

    if (millis() - lastRecordTime >= 250) {
      recordData();
      lastRecordTime = millis();
    }

    if (isFreefall()) {
      status = FALL;
      Serial.println("Rocket is falling!");
    }

    // Calculate height and deploy parachute when close to 200 meters
    float currentHeight = calculateHeight();
    if (currentHeight >= 200 && currentHeight <= 210) {
      status = DEPLOY_PARACHUTE;
      digitalWrite(parachuteDeployPin, HIGH); // Deploy parachute
      Serial.println("Deploying parachute!");
    }
  } else if (status == CALIBRATION) {
    calibrate();
    if (isButtonPressed()) {
      digitalWrite(triggerPin, HIGH);
      Serial.println("Button pressed! Triggering ON state.");
    }
  } else if (status == DEPLOY_PARACHUTE) {
    unsigned long currentTime = millis();
    if (currentTime - lastRecordTime >= 1000) { // Wait for 1 second after deploying parachute
      status = LANDING;
      Serial.println("Rocket is landing!");
    }
  } else if (status == LANDING) {
    // Check if the rocket is stationary (angular velocity is below a threshold)
    if (isStationary()) {
      status = LANDED;
      Serial.println("Rocket has landed!");
    }
  } else delay(500);
}

float calculateHeight() {
  sensors_event_t accelEvent;
  ism.getEvent(&accelEvent, NULL);
  float acceleration = sqrt(accelEvent.acceleration.x * accelEvent.acceleration.x +
                            accelEvent.acceleration.y * accelEvent.acceleration.y +
                            accelEvent.acceleration.z * accelEvent.acceleration.z);
  float velocity = initialVelocity + (acceleration * 9.81 * 0.5); 
  float height = initialHeight + (velocity * 0.5);
  return height;
}

bool isStationary() {
  sensors_event_t gyroEvent;
  ism.getEvent(NULL, &gyroEvent);

  float gyroMagnitudeSquared = gyroEvent.gyro.x * gyroEvent.gyro.x +
                              gyroEvent.gyro.y * gyroEvent.gyro.y +
                              gyroEvent.gyro.z * gyroEvent.gyro.z;

  float stationaryThreshold = 0.1; 

  return gyroMagnitudeSquared < stationaryThreshold;
}

bool isFreefall() {
  sensors_event_t accelEvent;
  ism.getEvent(&accelEvent, NULL);
  
  // Calculate the sum of squared accelerations
  float accelMagnitudeSquared = accelEvent.acceleration.x * accelEvent.acceleration.x +
                                accelEvent.acceleration.y * accelEvent.acceleration.y +
                                accelEvent.acceleration.z * accelEvent.acceleration.z;
  
  // Define a threshold for freefall detection (adjust as needed)
  float freefallThreshold = 1.0;
  
  return accelMagnitudeSquared < freefallThreshold;
}


void calibrate() {
  sensors_event_t accelEvent;
  ism.getEvent(&accelEvent, NULL);
  initialTiltX = atan2(accelEvent.acceleration.y, accelEvent.acceleration.z) * 180.0 / PI;
  initialTiltY = atan2(-accelEvent.acceleration.x, sqrt(accelEvent.acceleration.y * accelEvent.acceleration.y +
                                                        accelEvent.acceleration.z * accelEvent.acceleration.z)) * 180.0 / PI;
  delay(1000);
}

bool isButtonPressed() {
  return digitalRead(buttonPin) == LOW;
}

void recordData() {
  float currentHeight = calculateHeight();

  File dataFile = LittleFS.open(fileName, "a");
  if (dataFile) {
    sensors_event_t accelEvent, gyroEvent;
    ism.getEvent(&accelEvent, &gyroEvent);
    String data = String(currentHeight) + "," +
                  String(accelEvent.acceleration.x) + "," +
                  String(accelEvent.acceleration.y) + "," +
                  String(accelEvent.acceleration.z) + "," +
                  String(gyroEvent.gyro.x) + "," +
                  String(gyroEvent.gyro.y) + "," +
                  String(gyroEvent.gyro.z) + "\n";
    dataFile.print(data);
    dataFile.close();
  }
}

RocketStatus getStatus() {
  if (isButtonPressed()) return LAUNCH;
  else return CALIBRATION;
}
