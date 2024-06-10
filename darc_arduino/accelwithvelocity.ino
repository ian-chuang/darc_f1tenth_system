#include <TimerOne.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>

// Constants
const int high_threshold = 800;
const int low_threshold = 300;
const float wheel_radius = 0.05425; // radius in meters
const float circumference = 2 * PI * wheel_radius; // Circumference of the wheel in meters

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Sensor pins
const int analogPins[] = {A0, A1, A2, A3};

// Variables for each sensor
volatile int switchCounts[4] = {0, 0, 0, 0};
volatile bool currentStates[4] = {LOW, LOW, LOW, LOW};
volatile bool previousStates[4] = {LOW, LOW, LOW, LOW};

float velocities[4] = {0.0, 0.0, 0.0, 0.0};

unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);

  // Change the ADC prescaler to 16 (ADC clock of 1 MHz)
  ADCSRA = (ADCSRA & 0xF8) | 0x04;
  Serial.println("");

  // Initialise the sensor
  if (!bno.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Initialize each analog pin as input
  for (int i = 0; i < 4; ++i) {
    pinMode(analogPins[i], INPUT);
  }
  // Initialize Timer1 to call the ISR every 1 millisecond
  Timer1.initialize(1000); // 1000 microseconds = 1 millisecond
  Timer1.attachInterrupt(readEncoders);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 100) {  // Check if 100 ms have passed

    lastMillis = currentMillis;

    for (int i = 0; i < 4; ++i) {
      int count =  switchCounts[i];
      switchCounts[i] = 0;

      float distance = (count / 6.0) * circumference;
      float velocity = distance / 0.1; // 0.1 seconds = 100 milliseconds

      velocities[i] = velocity;
    }

    // Get acceleration data
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float x = accel.x();
    float y = accel.y();
    float z = accel.z();

    // Print the values to the Serial Plotter in a comma-separated format
    Serial.print(x, 5);
    Serial.print(",");
    Serial.print(y, 5);
    Serial.print(",");
    Serial.print(z, 5);
    Serial.print(",");
    Serial.print(velocities[0], 5);
    Serial.print(",");
    Serial.print(velocities[1], 5);
    Serial.print(",");
    Serial.print(velocities[2], 5);
    Serial.print(",");
    Serial.println(velocities[3], 5);
    
    
  }
  delay(1); // Small delay to stabilize the loop
}

void readEncoders() {
  for (int i = 0; i < 4; ++i) {
    int analogValue = analogRead(analogPins[i]);
    // Determine the current state based on the threshold
    if (analogValue > high_threshold) {
      currentStates[i] = HIGH;
    } else if (analogValue < low_threshold) {
      currentStates[i] = LOW;
    }
    // Detect a state change
    if (currentStates[i] != previousStates[i]) {
      switchCounts[i]++;
      previousStates[i] = currentStates[i];
    }
  }
}
