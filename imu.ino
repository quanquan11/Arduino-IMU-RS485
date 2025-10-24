#include "RS485.h"
#include "nodeConfig.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// -----------------------------
// Pin Configuration
// -----------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// -----------------------------
// RS485 Initialization
// -----------------------------
RS485 rs485(&Serial1, DE_PIN, NODE_ID);

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
}

void loop()
{
    // -----------------------------
    // Read IMU sensor data
    // -----------------------------
    // imu::Vector<3> accel_filtered = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // imu::Vector<3> gyro           = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // imu::Vector<3> magnetometer   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // -----------------------------
    // Create RS485 data packet
    // -----------------------------
    byte packet[29];
    packet[0] = NODE_ID; // Node identifier

    // Copy 3-byte (approx) vectors into packet
    // memcpy(packet + 1, &accel_filtered,  sizeof(accel_filtered));
    // memcpy(packet + 5, &gyro,            sizeof(gyro));
    // memcpy(packet + 9, &magnetometer,    sizeof(magnetometer));
    memcpy(packet + 1, &quat, sizeof(quat));
    memcpy(packet + 17, &accel, sizeof(accel));
    
    // -----------------------------
    // Transmit via RS485
    // -----------------------------
    // send() returns total bytes transmitted including protocol overhead (7 bytes)
    // Protocol adds: receiverID, senderID, length, STX, checksum, ETX, EOT
    // Expected: sizeof(packet) + 7 protocol bytes = 29 + 7 = 36 bytes
    size_t bytesSent = rs485.send(NODE_ID, packet, sizeof(packet));
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

    // -----------------------------
    // Optional debug output
    // -----------------------------
    Serial.print("Accel [m/s^2]: ");
    Serial.print(accel.x(), 3);
    Serial.print(", ");
    Serial.print(accel.y(), 3);
    Serial.print(", ");
    Serial.print(accel.z(), 3);
    Serial.println();

    Serial.print("Quat [w, x, y, z]: ");
    Serial.print(quat.w(), 3);
    Serial.print(", ");
    Serial.print(quat.x(), 3);
    Serial.print(", ");
    Serial.print(quat.y(), 3);
    Serial.print(", ");
    Serial.print(quat.z(), 3);
    Serial.println();
    Serial.println("---------------------------");

    // -----------------------------
    // Transmission rate control
    // -----------------------------
    delay(1000 / TRANSMIT_RATE); // e.g., 10 Hz if TRANSMIT_RATE = 10
}