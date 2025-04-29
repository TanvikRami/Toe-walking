#include <Wire.h>
#include "SparkFun_BMA400_Arduino_Library.h"


// ==== Accelerometer Setup ====
BMA400 accelerometer;
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14
int interruptPin = 21; // Connect to BMA400 INT2 pin
volatile bool interruptOccurred = false;
uint32_t lastStepCount = 0;


// ==== Force Sensor and LED ====
const int analogPin = A0;
const int digitalPin = 2;
const int ledPin = 6;
const float calibrationFactor = 157.15; // lb/V
const float forceThreshold = 10.0;


bool ledState = false;
bool prevState = false;
int ledCount = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection


  pinMode(digitalPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);


  // Start I2C
  Wire.begin();


  // Initialize accelerometer
  while (accelerometer.beginI2C(i2cAddress) != BMA400_OK) {
    Serial.println("Error: BMA400 not connected. Check wiring.");
    delay(1000);
  }


  Serial.println("BMA400 connected!");
 
 
  accelerometer.setRange(BMA400_RANGE_2G);
 
  // // ==== Sensitivity Configuration ====
  // bma400_step_counter_config_t stepConfig;
  // if (accelerometer.getStepCounterConfig(&stepConfig) == BMA400_OK) {
  //   stepConfig.activity_threshold = 5; // Lower = more sensitive
  //   stepConfig.min_step_buffer = 2;    // Fewer steps required before valid detection
  //   stepConfig.step_detector_mode = BMA400_STEP_DETECTOR_MODE_SENSITIVE;
  //   accelerometer.setStepCounterConfig(&stepConfig);
  //   Serial.println("Step sensitivity configuration applied.");
  // } else {
  //   Serial.println("Failed to read step config!");
  // }


  // Disable INT1, setup step interrupt on INT2
  accelerometer.enableInterrupt(BMA400_STEP_COUNTER_INT_EN, false);
  bma400_step_int_conf config = {
    .int_chan = BMA400_INT_CHANNEL_2
  };
  accelerometer.setStepCounterInterrupt(&config);
  accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_2, BMA400_INT_PUSH_PULL_ACTIVE_1);
  accelerometer.enableInterrupt(BMA400_STEP_COUNTER_INT_EN, true);


  attachInterrupt(digitalPinToInterrupt(interruptPin), bma400InterruptHandler, RISING);
}


void loop() {
  handleAccelerometer();  // Check for new step interrupt and update
  handleForceSensor();    // Check force sensor, toggle LED
  delay(200);             // Reduce frequency to make output readable
}


// ==== Accelerometer Interrupt Handler ====
void handleAccelerometer() {
  uint32_t stepCount = 0;
  uint8_t activityType = 0;


  // Poll step count every loop (for consistency)
  if (accelerometer.getStepCount(&stepCount, &activityType) == BMA400_OK) {
    if (stepCount != lastStepCount) {
      lastStepCount = stepCount;


      Serial.print("Step Count: ");
      Serial.print(stepCount);
      Serial.print(" | Activity: ");
      switch (activityType) {
        case BMA400_RUN_ACT: Serial.print("Running"); break;
        case BMA400_WALK_ACT: Serial.print("Walking"); break;
        case BMA400_STILL_ACT: Serial.print("Still"); break;
        default: Serial.print("Unknown"); break;
      }


      Serial.print(" | LED Count: ");
      Serial.println(ledCount);
    }
  }


  // Clear interrupt flag if triggered
  if (interruptOccurred) {
    interruptOccurred = false;


    uint16_t interruptStatus = 0;
    accelerometer.getInterruptStatus(&interruptStatus);
    // Optional: print raw interrupt status for debugging
    // Serial.print("Interrupt Status: 0x");
    // Serial.println(interruptStatus, HEX);
  }
}


// ==== Force Sensor Handler ====
void handleForceSensor() {
  int analogSensor1 = analogRead(analogPin);
  float voltage = analogSensor1 * (5.0 / 1023.0);
  float forceLbf = voltage * calibrationFactor;
  float forceN = forceLbf * 4.44822;


  ledState = (forceLbf >= forceThreshold);
  digitalWrite(ledPin, ledState ? HIGH : LOW);


  if (ledState && !prevState) {
    ledCount++;
    Serial.print("LED ON! Force: ");
    Serial.print(forceLbf, 2);
    Serial.print(" lbf | Total LED Events: ");
    Serial.println(ledCount);
  }


  prevState = ledState;
}


// ==== Interrupt Flag Setter ====
void bma400InterruptHandler() {
  interruptOccurred = true;
}
