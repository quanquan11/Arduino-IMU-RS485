// NODE 0 - MASTER (also sends its own data + polls slaves)
// Upload to Arduino #1
// Set NODE_ID = 0 in nodeConfig.h

#include "RS485.h"
#include "nodeConfig.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// -----------------------------
// SLAVE CONFIGURATION - Easy to add more!
// -----------------------------
#define NUM_SLAVES 1                    // Number of slave nodes
const uint8_t SLAVE_IDS[] = {1};        // Slave node IDs
                                         // To add more: {1, 2, 3, 4, ...}

// -----------------------------
// Pin Configuration
// -----------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// -----------------------------
// RS485 Initialization
// -----------------------------
RS485 rs485(&Serial1, DE_PIN, NODE_ID);

// For receiving responses from slaves
uint8_t senderID;
uint8_t receiveBuffer[50];
uint8_t receiveLen;

// Timing
uint8_t currentSlaveIndex = 0;

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
    Serial.print("MASTER Node - ID: ");
    Serial.println(NODE_ID);
    Serial.print("Managing ");
    Serial.print(NUM_SLAVES);
    Serial.println(" slave(s)");
    Serial.println("---------------------------");
}

void loop()
{
    // -----------------------------
    // Read IMU sensor data (Master's own data)
    // -----------------------------
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // -----------------------------
    // Create RS485 data packet
    // -----------------------------
    byte packet[29];
    packet[0] = NODE_ID; // Node identifier

    // Copy quaternion and acceleration into packet
    memcpy(packet + 1, &quat, sizeof(quat));
    memcpy(packet + 17, &accel, sizeof(accel));

    // -----------------------------
    // Transmit via RS485
    // -----------------------------
    // send() returns total bytes transmitted including protocol overhead (7 bytes)
    // Protocol adds: receiverID, senderID, length, STX, checksum, ETX, EOT
    // Expected: sizeof(packet) + 7 protocol bytes = 29 + 7 = 36 bytes
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

    delay(50);  // Small gap between master data and polling

    // -----------------------------
    // Poll current slave
    // -----------------------------
    uint8_t slaveID = SLAVE_IDS[currentSlaveIndex];

    Serial.print("Polling slave ");
    Serial.print(slaveID);
    Serial.print("... ");

    char pollCmd[] = "POLL";
    rs485.send(slaveID, (uint8_t*)pollCmd, strlen(pollCmd));

    // // Wait for slave to respond
    // delay(100);

    // Try to receive slave response
    if (rs485.receive(senderID, receiveBuffer, receiveLen))
    {
        Serial.print("Received response from slave ");
        Serial.print(senderID);
        Serial.print(" (");
        Serial.print(receiveLen);
        Serial.println(" bytes)");

        // Slave sends the same packet structure as master
        // The Python receiver will capture this data automatically
    }
    else
    {
        Serial.println("No response from slave");
    }

    // Move to next slave for next cycle (round-robin)
    currentSlaveIndex = (currentSlaveIndex + 1) % NUM_SLAVES;

    // -----------------------------
    // Transmission rate control
    // -----------------------------
    delay(1000 / TRANSMIT_RATE); // e.g., 10 Hz if TRANSMIT_RATE = 10
}
