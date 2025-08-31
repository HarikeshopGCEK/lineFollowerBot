
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// PCF8575 I2C address
#define PCF8575_ADDRESS 0x20
#define IR_SENSOR_COUNT 8
const int sensorWeights[IR_SENSOR_COUNT] = {-35, -25, -15, -5, 5, 15, 25, 35};

// Motor pins (adjust as per your hardware)
const int ENA = 5;    // Left motor PWM
const int IN1 = 18;   // Left motor direction
const int IN2 = 19;   // Left motor direction
const int ENB = 17;   // Right motor PWM
const int IN3 = 16;   // Right motor direction
const int IN4 = 4;    // Right motor direction
const int STBY = 2;   // TB6612FNG Standby pin (change pin number as needed)


// PID variables
volatile float Kp = 2.0;
volatile float Ki = 0.0;
volatile float Kd = 1.0;
float error = 0, previous_error = 0, integral = 0, derivative = 0;
int baseSpeed = 120; // Adjust as needed

AsyncWebServer server(80);


void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH); // Enable motor driver

    // Set up ESP32 as Access Point
    WiFi.softAP("PID-Bot", "447643899");
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Simple web page for PID tuning
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
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
        request->send(200, "text/html", html);
    });

    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
        if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
        if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
        request->redirect("/");
    });

    server.begin();
}

int getLineError() {
    Wire.requestFrom(PCF8575_ADDRESS, 2);
    if (Wire.available() >= 2) {
        uint8_t lowByte = Wire.read();   // P0-P7
        uint8_t highByte = Wire.read();  // P8-P15
        uint8_t irData = lowByte;

        // Print sensor states for debugging
        Serial.print("IR Sensors: ");
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            Serial.print((irData >> i) & 1);
            Serial.print(" ");
        }
        Serial.println();

        int weightedSum = 0;
        int activeSensors = 0;
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            if ((irData >> i) & 1) {
                weightedSum += sensorWeights[i];
                activeSensors++;
            }
        }
        if (activeSensors > 0) {
            return weightedSum / activeSensors;
        }
    }
    // If no line detected, return previous error (or 0)
    return previous_error;
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    digitalWrite(STBY, HIGH); // Ensure motor driver is enabled
    // Left motor
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    analogWrite(ENA, constrain(leftSpeed, 0, 255));

    // Right motor
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightSpeed = -rightSpeed;
    }
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

void loop() {
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