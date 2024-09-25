#include <Wire.h>
#include <Servo.h>

// Create servo objects for X and Y axes
Servo servoX;
Servo servoY;

// Define pin numbers for the servos
const int servoXPin = 9;
const int servoYPin = 10;

// MPU6050 I2C address constant
const int MPU_addr = 0x68;

// Variables to hold raw sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Manual control variables for the X-axis servo
bool isManualControlX = false; // Flag to toggle between manual and automatic modes
int manualXAngle = 90;          // Default angle for manual control
String inputString = "";         // To store serial input

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  Wire.begin();       // Initialize I2C communication

  // Set up the MPU6050 sensor
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // Access the power management register
  Wire.write(0);    // Wake up the sensor
  Wire.endTransmission(true);
  
  // Attach servos to their respective pins
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  
  // Initialize servos to center position
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  // Check for available serial input
  if (Serial.available() > 0) {
    inputString = Serial.readStringUntil('\n'); // Read input until newline
    inputString.trim(); // Clean up any extra spaces or newlines

    // Print the received input for debugging
    Serial.print("Received command: '");
    Serial.print(inputString);
    Serial.println("'");

    // Handle the command to switch to automatic mode
    if (inputString.equalsIgnoreCase("auto")) {
      isManualControlX = false; // Enable automatic control
      Serial.println("Switched to Automatic Control.");
    } 
    // Handle manual angle input
    else {
      int inputAngle = inputString.toInt(); // Convert string input to an integer
      if (inputAngle >= 0 && inputAngle <= 180) {
        manualXAngle = inputAngle; // Set the manual angle
        isManualControlX = true;    // Enable manual control
        Serial.print("Switched to Manual Control. Angle: ");
        Serial.println(manualXAngle);
      } else {
        Serial.println("Invalid input. Please enter an angle between 0 and 180 or 'auto'.");
      }
    }
  }

  // Read accelerometer data from MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // Request 6 bytes of data

  ax = Wire.read() << 8 | Wire.read(); // Read acceleration in X
  ay = Wire.read() << 8 | Wire.read(); // Read acceleration in Y
  az = Wire.read() << 8 | Wire.read(); // Read acceleration in Z

  // Read gyroscope data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // Starting register for gyroscope data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true); // Request 4 bytes of data

  gx = Wire.read() << 8 | Wire.read(); // Read gyroscope X
  gy = Wire.read() << 8 | Wire.read(); // Read gyroscope Y

  // Calculate angles from the raw data
  float angleX = atan2(ay, az) * 180 / PI; // Calculate tilt on the Y-axis
  float angleY = atan2(ax, az) * 180 / PI; // Calculate tilt on the X-axis

  // Map calculated angles to servo positions
  int servoXPos = map(angleY, -90, 90, 0, 180);
  int servoYPos = map(angleX, -90, 90, 0, 180);

  // Control the X-axis servo
  if (isManualControlX) {
    servoX.write(manualXAngle); // Use the manual angle if manual control is active
  } else {
    servoX.write(constrain(servoXPos, 0, 180)); // Constrain to servo range for automatic control
  }

  // Always control the Y-axis servo automatically
  servoY.write(constrain(servoYPos, 0, 180));

  // Debugging output for angles and accelerometer data
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print(" | Angle Y: ");
  Serial.print(angleY);
  Serial.print(" | Accel Z: ");
  Serial.println(az);
  
  delay(100); // Small delay for stability in readings
}
