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

// PWM channels for ESP32 LEDC
#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1
#define PWM_FREQ 20000 // 20kHz for quiet operation
#define PWM_RES_BITS 8 // 8-bit resolution (0-255)

// PID variables
volatile float Kp = 2.0;
volatile float Ki = 0.0;
volatile float Kd = 1.0;
float error = 0, previous_error = 0, integral = 0, derivative = 0;
int baseSpeed = 200; // Adjust as needed

// Turning behavior parameters
float sharpTurnThreshold = 0.7; // Percentage of baseSpeed (0.7 = 70%)
float tankTurnMultiplier = 0.8; // How aggressive the tank turning is (0.8 = 80%)
bool enableTankSteering = true; // Enable/disable tank steering mode

AsyncWebServer server(80);

// Global variables for sensor data (for web visualization)
int sensorStates[IR_SENSOR_COUNT];
int sensorRawValues[IR_SENSOR_COUNT];
int currentError = 0;
int activeSensors = 0;

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

  // Set up LEDC PWM for motors
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(ENA, LEFT_PWM_CHANNEL);
  ledcAttachPin(ENB, RIGHT_PWM_CHANNEL);

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

  // Enhanced web page with PID tuning and line detection visualization
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        String html = "<!DOCTYPE html><html><head><title>Line Follower Bot Dashboard</title>"
            "<style>"
            "body { background: linear-gradient(135deg, #43cea2 0%, #185a9d 100%); color: #fff; font-family: 'Segoe UI', Arial, sans-serif; text-align: center; padding: 20px; }"
            "h2 { color: #ffd700; margin-bottom: 20px; }"
            ".container { display: flex; justify-content: space-around; flex-wrap: wrap; gap: 20px; }"
            ".card { background: rgba(0,0,0,0.3); border-radius: 20px; padding: 20px; box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37); }"
            ".pid-card { width: 400px; }"
            ".sensor-card { width: 600px; }"
            ".slider-label { font-size: 1.2em; margin-top: 15px; color: #ffeb3b; }"
            ".slider { width: 80%; margin: 5px 0; }"
            ".text-input { width: 80px; padding: 8px; margin: 5px 0 15px 0; border: 2px solid #ffd700; border-radius: 5px; background: rgba(255,255,255,0.9); color: #185a9d; text-align: center; font-size: 1em; }"
            ".input-container { display: flex; align-items: center; justify-content: center; gap: 15px; margin: 5px 0; }"
            ".value { font-weight: bold; color: #00e676; }"
            "button { background: #ffd700; color: #185a9d; border: none; border-radius: 10px; padding: 10px 20px; font-size: 1em; cursor: pointer; margin: 5px; }"
            "button:hover { background: #ffeb3b; }"
            ".reset-btn { background: #f44336; color: white; }"
            ".reset-btn:hover { background: #d32f2f; }"
            ".sensor-array { display: flex; justify-content: center; gap: 8px; margin: 20px 0; flex-wrap: wrap; }"
            ".sensor-dot { width: 25px; height: 25px; border-radius: 50%; border: 2px solid #fff; display: flex; align-items: center; justify-content: center; font-size: 10px; font-weight: bold; }"
            ".sensor-active { background: #00e676; color: #000; }"
            ".sensor-inactive { background: #424242; color: #fff; }"
            ".line-indicator { margin: 10px 0; font-size: 1.1em; }"
            ".error-display { background: rgba(0,0,0,0.5); padding: 10px; border-radius: 10px; margin: 10px 0; }"
            ".status-info { display: flex; justify-content: space-between; margin: 10px 0; }"
            "@media (max-width: 1200px) { .container { flex-direction: column; align-items: center; } }"
            "</style></head><body>"
            "<h2>🤖 Line Follower Bot Dashboard</h2>"
            "<div class='container'>"
            "<div class='card pid-card'>"
            "<h3>PID Tuning Controls</h3>"
            "<form action='/set' method='get' id='pidForm'>"
            "<div class='slider-label'>Kp: <span class='value' id='kpVal'>" + String(Kp) + "</span></div>"
            "<div class='input-container'>"
            "<input type='range' min='0' max='10' step='0.01' name='kp' class='slider' value='" + String(Kp) + "' id='kpSlider' oninput='updateKp(this.value)'>"
            "<input type='number' min='0' max='10' step='0.01' class='text-input' id='kpInput' value='" + String(Kp) + "' oninput='updateKpSlider(this.value)'>"
            "</div>"
            "<div class='slider-label'>Ki: <span class='value' id='kiVal'>" + String(Ki) + "</span></div>"
            "<div class='input-container'>"
            "<input type='range' min='0' max='2' step='0.01' name='ki' class='slider' value='" + String(Ki) + "' id='kiSlider' oninput='updateKi(this.value)'>"
            "<input type='number' min='0' max='2' step='0.01' class='text-input' id='kiInput' value='" + String(Ki) + "' oninput='updateKiSlider(this.value)'>"
            "</div>"
            "<div class='slider-label'>Kd: <span class='value' id='kdVal'>" + String(Kd) + "</span></div>"
            "<div class='input-container'>"
            "<input type='range' min='0' max='5' step='0.01' name='kd' class='slider' value='" + String(Kd) + "' id='kdSlider' oninput='updateKd(this.value)'>"
            "<input type='number' min='0' max='5' step='0.01' class='text-input' id='kdInput' value='" + String(Kd) + "' oninput='updateKdSlider(this.value)'>"
            "</div>"
            "<br><button type='submit'>Update PID</button>"
            "<br><button type='button' class='reset-btn' onclick='resetValues()'>Reset to Defaults</button>"
            "</form>"
            "</div>"
            "<div class='card sensor-card'>"
            "<h3>📡 Real-time Line Detection</h3>"
            "<div class='sensor-array' id='sensorArray'></div>"
            "<div class='line-indicator'>"
            "<strong>Line Position Error: <span class='value' id='lineError'>0</span></strong>"
            "</div>"
            "<div class='error-display'>"
            "<div class='status-info'>"
            "<span>Active Sensors: <span class='value' id='activeCount'>0</span>/16</span>"
            "<span>Current PID: Kp=<span class='value' id='currentKp'>" + String(Kp) + "</span> Ki=<span class='value' id='currentKi'>" + String(Ki) + "</span> Kd=<span class='value' id='currentKd'>" + String(Kd) + "</span></span>"
            "</div>"
            "</div>"
            "</div>"
            "</div>"
            "<script>"
            "var updateInterval;"
            "function initSensorArray() {"
            "  var container = document.getElementById('sensorArray');"
            "  for(var i = 0; i < 16; i++) {"
            "    var dot = document.createElement('div');"
            "    dot.className = 'sensor-dot sensor-inactive';"
            "    dot.id = 'sensor' + i;"
            "    dot.textContent = i;"
            "    container.appendChild(dot);"
            "  }"
            "}"
            "function updateSensorDisplay(data) {"
            "  for(var i = 0; i < 16; i++) {"
            "    var sensor = document.getElementById('sensor' + i);"
            "    if(data.sensorStates[i] == 1) {"
            "      sensor.className = 'sensor-dot sensor-active';"
            "    } else {"
            "      sensor.className = 'sensor-dot sensor-inactive';"
            "    }"
            "  }"
            "  document.getElementById('lineError').textContent = data.error;"
            "  document.getElementById('activeCount').textContent = data.activeSensors;"
            "  document.getElementById('currentKp').textContent = data.kp.toFixed(2);"
            "  document.getElementById('currentKi').textContent = data.ki.toFixed(2);"
            "  document.getElementById('currentKd').textContent = data.kd.toFixed(2);"
            "}"
            "function fetchSensorData() {"
            "  fetch('/sensorData')"
            "    .then(response => response.json())"
            "    .then(data => updateSensorDisplay(data))"
            "    .catch(error => console.log('Error:', error));"
            "}"
            "function updateKp(value) {"
            "  document.getElementById('kpVal').innerText = parseFloat(value).toFixed(2);"
            "  document.getElementById('kpInput').value = parseFloat(value).toFixed(2);"
            "}"
            "function updateKi(value) {"
            "  document.getElementById('kiVal').innerText = parseFloat(value).toFixed(2);"
            "  document.getElementById('kiInput').value = parseFloat(value).toFixed(2);"
            "}"
            "function updateKd(value) {"
            "  document.getElementById('kdVal').innerText = parseFloat(value).toFixed(2);"
            "  document.getElementById('kdInput').value = parseFloat(value).toFixed(2);"
            "}"
            "function updateKpSlider(value) {"
            "  var numValue = parseFloat(value);"
            "  if (numValue >= 0 && numValue <= 10) {"
            "    document.getElementById('kpSlider').value = numValue;"
            "    document.getElementById('kpVal').innerText = numValue.toFixed(2);"
            "  }"
            "}"
            "function updateKiSlider(value) {"
            "  var numValue = parseFloat(value);"
            "  if (numValue >= 0 && numValue <= 2) {"
            "    document.getElementById('kiSlider').value = numValue;"
            "    document.getElementById('kiVal').innerText = numValue.toFixed(2);"
            "  }"
            "}"
            "function updateKdSlider(value) {"
            "  var numValue = parseFloat(value);"
            "  if (numValue >= 0 && numValue <= 5) {"
            "    document.getElementById('kdSlider').value = numValue;"
            "    document.getElementById('kdVal').innerText = numValue.toFixed(2);"
            "  }"
            "}"
            "function resetValues() {"
            "  var defaultKp = 2.0;"
            "  var defaultKi = 0.0;"
            "  var defaultKd = 1.0;"
            "  document.getElementById('kpSlider').value = defaultKp;"
            "  document.getElementById('kpInput').value = defaultKp;"
            "  document.getElementById('kpVal').innerText = defaultKp;"
            "  document.getElementById('kiSlider').value = defaultKi;"
            "  document.getElementById('kiInput').value = defaultKi;"
            "  document.getElementById('kiVal').innerText = defaultKi;"
            "  document.getElementById('kdSlider').value = defaultKd;"
            "  document.getElementById('kdInput').value = defaultKd;"
            "  document.getElementById('kdVal').innerText = defaultKd;"
            "}"
            "window.onload = function() {"
            "  initSensorArray();"
            "  fetchSensorData();"
            "  updateInterval = setInterval(fetchSensorData, 200);"
            "};"
            "</script>"
            "</body></html>";
        request->send(200, "text/html", html); });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
        if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
        if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
        request->redirect("/"); });

  // API endpoint for sensor data (for real-time visualization)
  server.on("/sensorData", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        String json = "{";
        json += "\"sensorStates\":[";
        for(int i = 0; i < IR_SENSOR_COUNT; i++) {
          json += String(sensorStates[i]);
          if(i < IR_SENSOR_COUNT - 1) json += ",";
        }
        json += "],\"sensorValues\":[";
        for(int i = 0; i < IR_SENSOR_COUNT; i++) {
          json += String(sensorRawValues[i]);
          if(i < IR_SENSOR_COUNT - 1) json += ",";
        }
        json += "],\"error\":" + String(currentError) + ",";
        json += "\"activeSensors\":" + String(activeSensors) + ",";
        json += "\"kp\":" + String(Kp) + ",";
        json += "\"ki\":" + String(Ki) + ",";
        json += "\"kd\":" + String(Kd);
        json += "}";
        request->send(200, "application/json", json); });

  server.begin();
}

int getLineError()
{
  int weightedSum = 0;
  activeSensors = 0; // Use global variable

  // Read all sensors through the multiplexer
  for (int i = 0; i < IR_SENSOR_COUNT; i++)
  {
    selectMuxChannel(i);
    int sensorValue = analogRead(MUX_OUT);

    // Store raw sensor value for visualization
    sensorRawValues[i] = sensorValue;

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
    currentError = weightedSum / activeSensors;
    return currentError;
  }

  // If no line detected, return previous error (or 0)
  currentError = previous_error;
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
  ledcWrite(LEFT_PWM_CHANNEL, constrain(leftSpeed, 0, 255));

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
  ledcWrite(RIGHT_PWM_CHANNEL, constrain(rightSpeed, 0, 255));
}

void loop()
{
  error = getLineError();
  // Anti-windup: reset integral if error crosses zero
  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0))
  {
    integral = 0;
  }
  integral += error;
  integral = constrain(integral, -1000, 1000); // Prevent windup
  derivative = error - previous_error;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  correction = constrain(correction, -200, 200); // Limit PID output

  int leftSpeed, rightSpeed;

  if (enableTankSteering && abs(correction) > baseSpeed * sharpTurnThreshold)
  {
    // Simplified tank steering
    if (correction > 0)
    {
      leftSpeed = baseSpeed;
      rightSpeed = -baseSpeed * tankTurnMultiplier;
    }
    else
    {
      leftSpeed = -baseSpeed * tankTurnMultiplier;
      rightSpeed = baseSpeed;
    }
  }
  else
  {
    // Differential steering
    leftSpeed = baseSpeed + correction;
    rightSpeed = baseSpeed - correction;
  }

  // Final safety limits
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotorSpeed(leftSpeed, rightSpeed);

  previous_error = error;

  delay(10);
}