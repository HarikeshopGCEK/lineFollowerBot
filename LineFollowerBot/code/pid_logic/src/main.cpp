
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ESP32 Pin Definitions for Multiplexer (CD4067)
#define S0 12 // Brown wire - Channel select bit 0 (LSB)
#define S1 13 // Orange wire - Channel select bit 1
#define S2 14 // Purple wire - Channel select bit 2
#define S3 15 // Yellow wire - Channel select bit 3 (MSB)

// Multiplexer Analog Output Pin (ESP32 ADC1_CH6)
#define MUX_OUT 34 // GPIO34 - ADC1_CH6 (Input only)

// Number of channels on the multiplexer
#define NUM_CHANNELS 16
#define IR_SENSOR_COUNT 16

// Small delay for multiplexer to settle after channel selection (in microseconds)
#define MUX_SETTLE_DELAY_US 100

// Number of calibration samples to take per channel
#define NUM_CALIBRATION_SAMPLES 700

// Arrays to store calibration data
int minValues[NUM_CHANNELS], maxValues[NUM_CHANNELS], medianValues[NUM_CHANNELS];

// Sensor weights for 16 sensors (centered around 0)
const int sensorWeights[IR_SENSOR_COUNT] = {-70, -60, -50, -40, -30, -20, -10, 0, 0, 10, 20, 30, 40, 50, 60, 70};

// ESP32 Motor Driver Pins (TB6612FNG)
const int ENA = 27;  // Left motor PWM (Channel 0)
const int IN1 = 26;  // Left motor direction A
const int IN2 = 25;  // Left motor direction B
const int ENB = 33;  // Right motor PWM (Channel 1)
const int IN3 = 32;  // Right motor direction A
const int IN4 = 4;   // Right motor direction B
const int STBY = 21; // TB6612FNG Standby pin

// PID variables
volatile float Kp = 2.0;
volatile float Ki = 0.0;
volatile float Kd = 1.0;
float error = 0, previous_error = 0, integral = 0, derivative = 0;
int baseSpeed = 120; // Adjust as needed

AsyncWebServer server(80);

/**
 * @brief Selects the active channel on the CD4067 multiplexer.
 * @param channel The channel number to select (0-15).
 */
void selectMuxChannel(byte channel)
{
  // S0 is LSB, S3 is MSB
  digitalWrite(S0, (channel & 0x01) ? HIGH : LOW); // bit 0
  digitalWrite(S1, (channel & 0x02) ? HIGH : LOW); // bit 1
  digitalWrite(S2, (channel & 0x04) ? HIGH : LOW); // bit 2
  digitalWrite(S3, (channel & 0x08) ? HIGH : LOW); // bit 3
  delayMicroseconds(MUX_SETTLE_DELAY_US);          // Allow multiplexer to settle
}

/**
 * @brief Performs sensor calibration to determine min, max, and median values for each channel.
 */
void calibrateSensors()
{
  Serial.println("Starting sensor calibration...");

  // Initialize min/max values for all channels (12-bit ADC: 0-4095)
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    minValues[i] = 4095; // Initialize with maximum possible value (12-bit)
    maxValues[i] = 0;    // Initialize with minimum possible value
  }

  Serial.print("Taking ");
  Serial.print(NUM_CALIBRATION_SAMPLES);
  Serial.println(" samples across all channels...");
  delay(1000);

  // Take NUM_CALIBRATION_SAMPLES iterations, reading all channels in each iteration
  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++)
  {
    // Read all channels for this sample iteration
    for (int j = 0; j < NUM_CHANNELS; j++)
    {
      selectMuxChannel(j);
      int reading = analogRead(MUX_OUT);

      // Update min/max for channel j
      if (reading < minValues[j])
        minValues[j] = reading;
      if (reading > maxValues[j])
        maxValues[j] = reading;
      // Optional: Add a small delay between reading channels in the same sample iteration
      delay(1);
    }
    // Optional: Add a delay between sample iterations
    delay(10);
  }

  // Calculate median (simple average of min/max) and print results after all samples are taken
  Serial.println("Calibration complete. Results:");
  Serial.println("-------------------");
  Serial.println("Channel\tMin Value\tMax Value\tMedian Value");
  Serial.println("-------------------");
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    medianValues[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(i);
    Serial.print("\t");
    Serial.print(minValues[i]);
    Serial.print("\t");
    Serial.print(maxValues[i]);
    Serial.print("\t");
    Serial.println(medianValues[i]);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 - 16 Channel IR Sensor Line Follower with PID Control");

  // Set up multiplexer select pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set up motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Enable motor driver

  // ESP32 ADC Configuration for better performance
  analogReadResolution(12);       // Set ADC resolution to 12-bit (0-4095)
  analogSetAttenuation(ADC_11db); // Set ADC attenuation for 0-3.3V range

  // Perform sensor calibration
  // IMPORTANT: During calibration, move the robot over both line and background
  // to get accurate min/max values for each sensor
  calibrateSensors();
  Serial.println("-------------------");
  Serial.println("Setup complete. Starting line following...");
  delay(1000);

  // Set up ESP32 as Access Point
  WiFi.softAP("PID-Bot", "447643899");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Simple web page for PID tuning
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        String html = "<!DOCTYPE html><html><head><title>PID Tuning Dashboard</title>"
            "<style>"
            "body { background: linear-gradient(135deg, #43cea2 0%, #185a9d 100%); color: #fff; font-family: 'Segoe UI', Arial, sans-serif; text-align: center; padding: 40px; }"
            "h2 { color: #ffd700; margin-bottom: 30px; }"
            ".card { background: rgba(0,0,0,0.3); border-radius: 20px; padding: 30px; display: inline-block; box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37); }"
            ".slider-label { font-size: 1.2em; margin-top: 20px; color: #ffeb3b; }"
            ".slider { width: 80%; margin: 10px 0 30px 0; }"
            ".value { font-weight: bold; color: #00e676; }"
            "button { background: #ffd700; color: #185a9d; border: none; border-radius: 10px; padding: 10px 30px; font-size: 1.1em; cursor: pointer; margin-top: 20px; }"
            "button:hover { background: #ffeb3b; }"
            "</style></head><body>"
            "<div class='card'>"
            "<h2>PID Tuning Dashboard</h2>"
            "<form action='/set' method='get' id='pidForm'>"
            "<div class='slider-label'>Kp: <span class='value' id='kpVal'>" + String(Kp) + "</span></div>"
            "<input type='range' min='0' max='10' step='0.01' name='kp' class='slider' value='" + String(Kp) + "' oninput='kpVal.innerText=this.value'>"
            "<div class='slider-label'>Ki: <span class='value' id='kiVal'>" + String(Ki) + "</span></div>"
            "<input type='range' min='0' max='2' step='0.01' name='ki' class='slider' value='" + String(Ki) + "' oninput='kiVal.innerText=this.value'>"
            "<div class='slider-label'>Kd: <span class='value' id='kdVal'>" + String(Kd) + "</span></div>"
            "<input type='range' min='0' max='5' step='0.01' name='kd' class='slider' value='" + String(Kd) + "' oninput='kdVal.innerText=this.value'>"
            "<br><button type='submit'>Update PID</button>"
            "</form>"
            "<p style='margin-top:30px;'>Current Kp: <span class='value'>" + String(Kp) + "</span>, Ki: <span class='value'>" + String(Ki) + "</span>, Kd: <span class='value'>" + String(Kd) + "</span></p>"
            "</div>"
            "</body></html>";
        request->send(200, "text/html", html); });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
        if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
        if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
        request->redirect("/"); });

  server.begin();
}

int getLineError()
{
  int weightedSum = 0;
  int activeSensors = 0;
  int sensorStates[IR_SENSOR_COUNT];

  // Read all sensors through the multiplexer
  for (int i = 0; i < IR_SENSOR_COUNT; i++)
  {
    selectMuxChannel(i);
    int sensorValue = analogRead(MUX_OUT);

    // Map the sensor reading from its calibrated range to 0-4095 (12-bit)
    int mappedValue = map(sensorValue, minValues[i], maxValues[i], 0, 4095);

    // Constrain the mapped value to ensure it stays within 0-4095
    mappedValue = constrain(mappedValue, 0, 4095);

    // Determine if sensor detects line (assuming line is darker than background)
    // For your sensors: 4095 = white/background, lower values = black line
    // Threshold can be adjusted based on your setup (try 3000-3500 for better sensitivity)
    int threshold = 3000; // Adjusted for better line detection sensitivity
    sensorStates[i] = (mappedValue < threshold) ? 1 : 0;

    if (sensorStates[i])
    {
      weightedSum += sensorWeights[i];
      activeSensors++;
    }
  }

  // Print sensor states for debugging (optional - can be removed for better performance)
  Serial.print("Raw Values: ");
  for (int i = 0; i < IR_SENSOR_COUNT; i++)
  {
    selectMuxChannel(i);
    int rawValue = analogRead(MUX_OUT);
    Serial.print(rawValue);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Sensor States: ");
  for (int i = 0; i < IR_SENSOR_COUNT; i++)
  {
    Serial.print(sensorStates[i]);
    Serial.print(" ");
  }
  Serial.println();

  if (activeSensors > 0)
  {
    return weightedSum / activeSensors;
  }

  // If no line detected, return previous error (or 0)
  return previous_error;
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  digitalWrite(STBY, HIGH); // Ensure motor driver is enabled
  // Left motor
  if (leftSpeed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENA, constrain(leftSpeed, 0, 255));

  // Right motor
  if (rightSpeed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

void loop()
{
  error = getLineError();
  integral += error;
  derivative = error - previous_error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotorSpeed(leftSpeed, rightSpeed);

  previous_error = error;

  delay(10);
}