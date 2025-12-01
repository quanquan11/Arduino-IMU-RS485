// SLAVE NODE - Sends data when polled by master
// Upload to Arduino #2, #3, #4, etc.
// Set NODE_ID = 1, 2, 3, ... in nodeConfig.h

#include "RS485.h"
#include "nodeConfig.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

// -----------------------------
// Pin Configuration
// -----------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// -----------------------------
// RS485 Initialization
// -----------------------------
RS485 rs485(&Serial1, DE_PIN, NODE_ID);

// For receiving poll requests from master
uint8_t senderID;
uint8_t receiveBuffer[32];
uint8_t receiveLen;

// -----------------------------
// Pendulum Configuration
// -----------------------------
// Wire length in cm (adjust for each node)
const float WIRE_LENGTH_CM = 100.0;

// Calibration: reference gravity vector when hanging at rest
float refGravityX = 0.0;
float refGravityY = 0.0;
float refGravityZ = 0.0;
bool isCalibrated = false;

// EEPROM storage addresses and magic number
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC 0x42494D55  // "BIMU" - BNO055 IMU magic number

struct CalibrationData {
  uint32_t magic;
  float gravityX;
  float gravityY;
  float gravityZ;
};

// Load calibration from EEPROM
bool loadCalibrationFromEEPROM() {
  CalibrationData data;
  EEPROM.get(EEPROM_MAGIC_ADDR, data);

  // Check if valid calibration exists
  if (data.magic == EEPROM_MAGIC) {
    refGravityX = data.gravityX;
    refGravityY = data.gravityY;
    refGravityZ = data.gravityZ;
    isCalibrated = true;

    float magnitude = sqrt(refGravityX*refGravityX + refGravityY*refGravityY + refGravityZ*refGravityZ);

    Serial.println("*** CALIBRATION LOADED FROM EEPROM ***");
    Serial.print("Reference gravity: (");
    Serial.print(refGravityX, 2);
    Serial.print(", ");
    Serial.print(refGravityY, 2);
    Serial.print(", ");
    Serial.print(refGravityZ, 2);
    Serial.print(") | Magnitude: ");
    Serial.print(magnitude, 2);
    Serial.println(" m/s²");

    return true;
  }

  return false;
}

// Save calibration to EEPROM
void saveCalibrationToEEPROM() {
  CalibrationData data;
  data.magic = EEPROM_MAGIC;
  data.gravityX = refGravityX;
  data.gravityY = refGravityY;
  data.gravityZ = refGravityZ;

  EEPROM.put(EEPROM_MAGIC_ADDR, data);
  Serial.println("Calibration saved to EEPROM");
}

// Calibrate zero position
void calibrateZeroPosition(imu::Vector<3> gravity) {
  refGravityX = gravity.x();
  refGravityY = gravity.y();
  refGravityZ = gravity.z();

  float magnitude = sqrt(refGravityX*refGravityX + refGravityY*refGravityY + refGravityZ*refGravityZ);

  Serial.println("*** CALIBRATION COMPLETE ***");
  Serial.print("Reference gravity: (");
  Serial.print(refGravityX, 2);
  Serial.print(", ");
  Serial.print(refGravityY, 2);
  Serial.print(", ");
  Serial.print(refGravityZ, 2);
  Serial.print(") | Magnitude: ");
  Serial.print(magnitude, 2);
  Serial.println(" m/s²");

  isCalibrated = true;
  saveCalibrationToEEPROM();
}

// Calculate horizontal displacement using gravity vector
float calculateDisplacement(imu::Vector<3> gravity) {
  if (!isCalibrated) {
    return 0.0;  // Return 0 if not calibrated
  }

  float gx = gravity.x();
  float gy = gravity.y();
  float gz = gravity.z();

  // Calculate angle between current gravity and reference gravity using dot product
  float dotProduct = gx*refGravityX + gy*refGravityY + gz*refGravityZ;
  float magCurrent = sqrt(gx*gx + gy*gy + gz*gz);
  float magRef = sqrt(refGravityX*refGravityX + refGravityY*refGravityY + refGravityZ*refGravityZ);

  float cosTheta = dotProduct / (magCurrent * magRef);
  // Clamp to valid range for acos
  cosTheta = constrain(cosTheta, -1.0, 1.0);

  float thetaRad = acos(cosTheta);

  // Calculate horizontal displacement
  float displacement = WIRE_LENGTH_CM * sin(thetaRad);

  return displacement;
}

void setup()
{
    Serial.begin(9600);       // For debug output
    Serial1.begin(9600);     // Initialize RS485 serial communication
    Wire.begin();             // Initialize I2C

    if (!bno.begin()) {
        Serial.println("BNO055 not detected. Please check your connection.");
        while (1);
    }

    delay(1000);              // Sensor startup delay
    Serial.println("BNO055 detected and initialized!");
    Serial.println("RS485 initialized!");
    Serial.print("SLAVE Node - ID: ");
    Serial.println(NODE_ID);

    // Try to load calibration from EEPROM
    Serial.println();
    if (loadCalibrationFromEEPROM()) {
        Serial.println("Ready to measure displacement!");
        Serial.println("Commands: C=calibrate");
    } else {
        Serial.println("No saved calibration found.");
        Serial.println("Send 'C' to calibrate when hanging at rest.");
    }

    Serial.println("Waiting for poll from master...");
    Serial.println("---------------------------");
}

// Function to read sensor and print data
void readAndPrintSensorData() {
    // Read IMU sensor data
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // Calculate displacement
    float displacement = calculateDisplacement(gravity);

    // Print data to Serial
    Serial.println("========== SENSOR DATA ==========");
    Serial.print("Node ID: ");
    Serial.println(NODE_ID);

    Serial.print("Gravity [m/s^2]: (");
    Serial.print(gravity.x(), 3);
    Serial.print(", ");
    Serial.print(gravity.y(), 3);
    Serial.print(", ");
    Serial.print(gravity.z(), 3);
    Serial.println(")");

    float gMagnitude = sqrt(gravity.x()*gravity.x() + gravity.y()*gravity.y() + gravity.z()*gravity.z());
    Serial.print("Gravity Magnitude: ");
    Serial.print(gMagnitude, 3);
    Serial.println(" m/s^2");

    Serial.print("Displacement: ");
    Serial.print(displacement, 2);
    Serial.print(" cm");
    if (!isCalibrated) {
        Serial.print(" (NOT CALIBRATED)");
    }
    Serial.println();

    if (isCalibrated) {
        Serial.print("Reference Gravity: (");
        Serial.print(refGravityX, 3);
        Serial.print(", ");
        Serial.print(refGravityY, 3);
        Serial.print(", ");
        Serial.print(refGravityZ, 3);
        Serial.println(")");
    }

    Serial.print("Quaternion [w, x, y, z]: (");
    Serial.print(quat.w(), 3);
    Serial.print(", ");
    Serial.print(quat.x(), 3);
    Serial.print(", ");
    Serial.print(quat.y(), 3);
    Serial.print(", ");
    Serial.print(quat.z(), 3);
    Serial.println(")");

    Serial.println("=================================");
    Serial.println();
}

void loop()
{
    // Check for calibration command via Serial
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'C' || cmd == 'c') {
            Serial.println("Calibrating zero position...");
            delay(500); // Wait for stability
            imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            calibrateZeroPosition(gravity);
        }
    }

#if TEST_MODE
    // TEST MODE: Continuously read and print sensor data to Serial
    readAndPrintSensorData();
    delay(1000);  // Read every 1 second

#else
    // NORMAL MODE: Wait for poll request from master
    // -----------------------------
    // Wait for poll request from master
    // -----------------------------
    if (rs485.receive(senderID, receiveBuffer, receiveLen))
    {
        receiveBuffer[receiveLen] = 0;  // Null terminate

        // Check if it's a calibrate command
        if (strcmp((char*)receiveBuffer, "CALIB") == 0 || strcmp((char*)receiveBuffer, "CALIBRATE") == 0)
        {
            Serial.print("Calibrate command received from master (ID: ");
            Serial.print(senderID);
            Serial.println(")");
            Serial.println("Calibrating zero position...");
            delay(500); // Wait for stability
            imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            calibrateZeroPosition(gravity);
        }
        // Check if it's a poll request command
        else if (strcmp((char*)receiveBuffer, "POLL") == 0)
        {
            Serial.print("Poll request received from master (ID: ");
            Serial.print(senderID);
            Serial.println(")");

            // -----------------------------
            // Read IMU sensor data
            // -----------------------------
            imu::Quaternion quat = bno.getQuat();
            imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

            // Calculate displacement
            float displacement = calculateDisplacement(gravity);

            // -----------------------------
            // Create RS485 data packet
            // -----------------------------
            // New packet structure (33 bytes):
            // Byte 0: NODE_ID (1 byte)
            // Bytes 1-16: Quaternion (4 floats = 16 bytes)
            // Bytes 17-28: Gravity (3 floats = 12 bytes)
            // Bytes 29-32: Displacement (1 float = 4 bytes)
            byte packet[33];
            packet[0] = NODE_ID; // Node identifier

            // Copy quaternion, gravity vector, and displacement into packet
            memcpy(packet + 1, &quat, sizeof(quat));
            memcpy(packet + 17, &gravity, sizeof(gravity));
            memcpy(packet + 29, &displacement, sizeof(displacement));

            // -----------------------------
            // Transmit via RS485
            // -----------------------------
            // send() returns total bytes transmitted including protocol overhead (7 bytes)
            // Protocol adds: receiverID, senderID, length, STX, checksum, ETX, EOT
            // Expected: sizeof(packet) + 7 protocol bytes = 33 + 7 = 40 bytes
            // Send to broadcast address (255) so Python receiver can capture it
            size_t bytesSent = rs485.send(255, packet, sizeof(packet));
            size_t expectedBytes = sizeof(packet) + 7;  // 7 = protocol overhead

            if (bytesSent == expectedBytes) {
                Serial.print("RS485 packet transmitted successfully! (");
                Serial.print(sizeof(packet));
                Serial.println(" bytes payload)");
            } else {
                Serial.print("RS485 transmission failed! Sent ");
                Serial.print(bytesSent);
                Serial.print(" bytes, expected ");
                Serial.println(expectedBytes);
            }

            // Print sensor data
            readAndPrintSensorData();
        }
    }
#endif
}