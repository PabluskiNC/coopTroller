// Digital & analog pins for various components

const int lightSense    = A0; // Light Sensor

// Trigger pin for WiFi re-config using AP mode (WiFiManager)
#define TRIGGER_PIN D0

const int doorTop       = D1; // Reed Switch
const int doorBottom    = D2; // Reed Switch

const int i2cSDA        = D3; // I2C SDA
const int i2cSCL        = D4; // I2C SCL

const int doorClose     = D5; // MotorA Close
const int doorOpen      = D7; // MotorA Open

