#include <Arduino.h>
#include <Wire.h>

// PCF8575 I2C address
#define PCF8575_ADDRESS 0x20

// PCF8575 has 16 I/O pins (P0-P15)
// Assuming IR sensors are connected to P0-P7 (first 8 pins)
#define IR_SENSOR_COUNT 8

// Weight values for each sensor position (for PID logic)
// These weights create a center-weighted sum where center sensors have higher weight
const int sensorWeights[IR_SENSOR_COUNT] = {-35, -25, -15, -5, 5, 15, 25, 35};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("IR Sensor Array with PCF8575 I2C Expander - PID Weighted Sum");
  Serial.println("============================================================");
}

void loop() {
  // Read data from PCF8575
  Wire.requestFrom(PCF8575_ADDRESS, 2); // PCF8575 returns 2 bytes (16 bits)
  
  if (Wire.available() >= 2) {
    // Read the two bytes from PCF8575
    uint8_t lowByte = Wire.read();   // P0-P7
    uint8_t highByte = Wire.read();  // P8-P15
    
    // Combine bytes to get 16-bit value
    uint16_t sensorData = (highByte << 8) | lowByte;
    
    // Extract only the first 8 bits (P0-P7) for IR sensors
    uint8_t irData = lowByte;
    
    // Calculate weighted sum for PID logic
    int weightedSum = 0;
    int activeSensors = 0;
    
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      if ((irData >> i) & 1) {
        // Black line detected (sensor = 1)
        weightedSum += sensorWeights[i];
        activeSensors++;
      }
      // White surface (sensor = 0) contributes 0 to the sum
    }
    
    // Calculate average position (for PID error calculation)
    int averagePosition = 0;
    if (activeSensors > 0) {
      averagePosition = weightedSum / activeSensors;
    }
    
    // Display results
    Serial.print("IR Sensor Data (Binary): ");
    for (int i = 7; i >= 0; i--) {
      Serial.print((irData >> i) & 1);
    }
    
    Serial.print(" | Weighted Sum: ");
    Serial.print(weightedSum);
    
    Serial.print(" | Active Sensors: ");
    Serial.print(activeSensors);
    
    Serial.print(" | Average Position: ");
    Serial.print(averagePosition);
    
    // Display individual sensor states with weights
    Serial.print(" | Sensors: ");
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      if ((irData >> i) & 1) {
        Serial.print("1"); // Black line detected
      } else {
        Serial.print("0"); // White surface
      }
      if (i < IR_SENSOR_COUNT - 1) Serial.print(" ");
    }
    
    Serial.print(" | Weights: ");
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      Serial.print(sensorWeights[i]);
      if (i < IR_SENSOR_COUNT - 1) Serial.print(" ");
    }
    
    Serial.println();
  } else {
    Serial.println("Error: No data received from PCF8575");
  }
  
  delay(500); // Update every 100ms
}