  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <Arduino.h>

  // Constants for sensor thresholds
  const int lowThreshold = 300;
  const int highThreshold = 850;
  const float wheelRadius = 54.25; // Radius in mm
  const float pi = 3.14159265358979323846;
  const float wheelCircumference = 2 * pi * wheelRadius; // Calculate wheel circumference

  Adafruit_BNO055 bno = Adafruit_BNO055(55);

  // Enum to represent sensor state
  enum SensorState {
    SENSOR_LOW,            // Sensor value is low
    SENSOR_HIGH,           // Sensor value is high
    TRANSITION_TO_HIGH,    // Transitioning from low to high
    TRANSITION_TO_LOW      // Transitioning from high to low
  };

  // Structure to hold sensor data
  struct SensorData {
    int pin;                // Analog pin number
    int value;              // Current sensor value
    SensorState state;      // Current state of the sensor
    int transitions;        // Count of transitions
    float velocity;         // Velocity of the wheel
  };

  // Initialize sensor data for each sensor
  SensorData sensors[] = {
    {A0, 0, SENSOR_LOW, 0, 0.0},
    {A1, 0, SENSOR_LOW, 0, 0.0},
    {A2, 0, SENSOR_LOW, 0, 0.0},
    {A3, 0, SENSOR_LOW, 0, 0.0},
  };

  void setup(void) {
    Serial.begin(9600);
    Serial.println("Acceleration Sensor Test");
    Serial.println("");

    // Initialise the sensor
    if (!bno.begin()) {
      // There was a problem detecting the BNO055 ... check your connections
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);
  }

  void loop(void) {
    // Iterate over each sensor and update its state and transition count
    for (SensorData &sensor : sensors) {
      int previousValue = sensor.value;
      sensor.value = analogRead(sensor.pin); // Read current sensor value

      // Update sensor state and count transitions based on the current value
      switch (sensor.state) {
        case SENSOR_LOW:
          if (sensor.value > highThreshold) {
            sensor.state = TRANSITION_TO_HIGH;
          }
          break;
        case SENSOR_HIGH:
          if (sensor.value < lowThreshold) {
            sensor.state = TRANSITION_TO_LOW;
          }
          break;
        case TRANSITION_TO_HIGH:
          if (sensor.value >= highThreshold) {
            sensor.state = SENSOR_HIGH;
            sensor.transitions++; // Count this transition
          }
          break;
        case TRANSITION_TO_LOW:
          if (sensor.value <= lowThreshold) {
            sensor.state = SENSOR_LOW;
            sensor.transitions++; // Count this transition
          }
          break;
      }
    }
    // Calculate and send velocity for each sensor every second
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) { // Update every second
      for (int i = 0; i < sizeof(sensors) / sizeof(SensorData); i++) {
        if (sensors[i].transitions > 0) {
          float rotations = sensors[i].transitions / 6.0; // 6 transitions equal one full rotation
          float distance = rotations * wheelCircumference; // Distance covered in mm
          sensors[i].velocity = distance / 1000.0; // Velocity in m/s (since time frame is 1 second)
          sensors[i].transitions = 0; // Reset transitions count for the next period
        } else {
          sensors[i].velocity = 0; // Set velocity to 0 if no transitions
        }
        
      }
      lastPrintTime = currentTime;
    
    }
  /* Get acceleration data */
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float x = accel.x();
  float y = accel.y();
  float z = accel.z();
  float a = sensors[0].velocity;
  float b = sensors[1].velocity;
  float c = sensors[2].velocity;
  float d = sensors[3].velocity;

  byte* bytesX = (byte*)&x;
  byte* bytesY = (byte*)&y;
  byte* bytesZ = (byte*)&z;
  byte* bytesA = (byte*)&a;
  byte* bytesB = (byte*)&b;
  byte* bytesC = (byte*)&c;
  byte* bytesD = (byte*)&d;

  // Serial.write(0xAA); // Start byte

  // // Sending x-axis acceleration
  // Serial.write(bytesX, sizeof(float));
  // // Sending y-axis acceleration
  // Serial.write(bytesY, sizeof(float));
  // // Sending z-axis acceleration
  // Serial.write(bytesZ, sizeof(float));
  // // sending sensors
  // Serial.write(bytesA, sizeof(float));
  // Serial.write(bytesB, sizeof(float));
  // Serial.write(bytesC, sizeof(float));
  // Serial.write(bytesD, sizeof(float));
  // Serial.println();

  Serial.print(x, 5);
  Serial.print(",");
  Serial.print(y, 5);
  Serial.print(",");
  Serial.print(z, 5);
  Serial.print(",");
  Serial.print(a, 5);
  Serial.print(",");
  Serial.print(b, 5);
  Serial.print(",");
  Serial.print(c, 5);
  Serial.print(",");
  Serial.print(d, 5);
  Serial.print(",");
  Serial.println();
  
}