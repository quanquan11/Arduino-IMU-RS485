# IMU Pendulum Displacement Measurement System

## Overview

This project uses a Arduino Pro Micro and BNO055 IMU sensor to measure the horizontal displacement of a pendulum by analyzing gravity vector changes. The system operates in an RS485 network as a slave node, responding to poll requests from a Raspberry Pi (Master Node written in Python).

## Concept

The core idea is to use the gravity vector from an IMU sensor to calculate how far a pendulum has swung horizontally from its rest position. When the pendulum is at rest, gravity points straight down. When it swings, the angle changes, and we can calculate the horizontal displacement using simple trigonometry.

## Hardware Requirements

### Components
- **Arduino Pro Micro board** (with Serial1 support for RS485)
- **BNO055 IMU sensor** (9-axis absolute orientation sensor)
- **RS485 To TTL UART Communication Serial Converter Module**
- **Pendulum wire** (known length e.g 100cm)

### Wiring Connections

#### BNO055 IMU Sensor (I2C)
- VCC → 5V
- GND → GND
- SDA → SDA (Arduino I2C data - Pin 2)
- SCL → SCL (Arduino I2C clock - Pin 3)

#### RS485 Module
- VCC → 5V
- GND → GND
- RO (Receiver Output) → Serial1 RX
- DI (Driver Input) → Serial1 TX
- DE/RE (Driver Enable) → Digital Pin (defined as DE_PIN in nodeConfig.h)
- A, B → RS485 bus lines (connect to master and other nodes)

## Prototype Assembly

1. **Mount the IMU**: Securely attach the BNO055 sensor to your pendulum bob (The Capsule)
2. **Measure wire length**: Accurately measure the distance from the pivot point to the IMU sensor (in centimeters)
3. **Update configuration**: Set `WIRE_LENGTH_CM` in the code to match your wire length
4. **Configure node**: Set unique `NODE_ID` in `nodeConfig.h` for each slave node
5. **Connect to RS485 network**: Wire all nodes to the same A/B bus lines

## Algorithm Explanation

### 1. Calibration Phase

When the pendulum is hanging at rest (equilibrium position):

```
1. Read gravity vector from IMU: g_ref = (gx_ref, gy_ref, gz_ref)
2. Store this as the reference "zero position"
3. Save to EEPROM for persistence across power cycles
```

The reference gravity vector represents the direction of gravity when the pendulum is perfectly vertical (not swinging).

### 2. Measurement Phase

When the pendulum swings:

```
1. Read current gravity vector: g_current = (gx, gy, gz)
2. Calculate angle θ between g_ref and g_current
3. Calculate horizontal displacement d from angle θ
```

### Mathematical Formulas

#### Step 1: Calculate the Angle Between Vectors

The angle between the reference gravity vector and the current gravity vector is calculated using the **dot product formula**:

```
cos(θ) = (g_ref · g_current) / (|g_ref| × |g_current|)
```

Where:
- **Dot product**: `g_ref · g_current = gx_ref×gx + gy_ref×gy + gz_ref×gz`
- **Magnitude of g_ref**: `|g_ref| = √(gx_ref² + gy_ref² + gz_ref²)`
- **Magnitude of g_current**: `|g_current| = √(gx² + gy² + gz²)`

Then solve for θ:
```
θ = arccos(cos(θ))
```

**Implementation** (from `imu.ino:128-137`):
```cpp
float dotProduct = gx*refGravityX + gy*refGravityY + gz*refGravityZ;
float magCurrent = sqrt(gx*gx + gy*gy + gz*gz);
float magRef = sqrt(refGravityX*refGravityX + refGravityY*refGravityY + refGravityZ*refGravityZ);

float cosTheta = dotProduct / (magCurrent * magRef);
cosTheta = constrain(cosTheta, -1.0, 1.0);  // Clamp to valid range
float thetaRad = acos(cosTheta);
```

#### Step 2: Calculate Horizontal Displacement

Once we have the tilt angle θ, we use **simple pendulum geometry**:

```
d = L × sin(θ)
```

Where:
- **d** = horizontal displacement (cm)
- **L** = wire length from pivot to sensor (cm)
- **θ** = tilt angle from vertical (radians)

**Implementation** (from `imu.ino:140`):
```cpp
float displacement = WIRE_LENGTH_CM * sin(thetaRad);
```

### Visual Representation

```
        Pivot Point
            |
            | L (wire length)
            |
     θ _____|
        \   |
         \  |
          \ |
           \|
          IMU (at rest)

When swinging:
        Pivot Point
            |
            |\
            | \
            |  \ L
            |   \
            |θ   \
            |     \
         ---|------* IMU
            |<--d-->|

d = horizontal displacement
θ = angle from vertical
L = wire length
```

## Code Structure

### Key Functions

- **`calibrateZeroPosition()`** - Records reference gravity vector
- **`calculateDisplacement()`** - Computes horizontal displacement from current gravity
- **`saveCalibrationToEEPROM()`** - Persists calibration data
- **`loadCalibrationFromEEPROM()`** - Restores calibration on startup

### Operation Modes

#### Test Mode (`TEST_MODE = 1`)
- Continuously reads and prints sensor data to Serial
- Updates every 1 second
- Useful for debugging and verification

#### Normal Mode (`TEST_MODE = 0`)
- Waits for RS485 poll requests from master
- Responds with sensor data packet
- Supports remote calibration via "CALIB" command

## Usage Instructions

### Initial Setup

1. **Upload the code** to your Arduino
2. **Open Serial Monitor** (9600 baud)
3. **Hang the pendulum** vertically at rest
4. **Send 'C'** via Serial Monitor to calibrate
5. **Verify calibration** - reference gravity should be ~9.8 m/s²

### RS485 Network Operation

The master node sends commands:
- **"POLL"** - Request sensor data
- **"CALIB"** - Trigger calibration

The slave responds with a 33-byte packet:
- Byte 0: Node ID
- Bytes 1-16: Quaternion (w, x, y, z)
- Bytes 17-28: Gravity vector (x, y, z)
- Bytes 29-32: Horizontal displacement

## Data Packet Structure

```
Total: 33 bytes payload + 7 bytes RS485 protocol overhead = 40 bytes

Payload breakdown:
┌────────┬──────────────────┬─────────────────┬──────────────┐
│ NodeID │   Quaternion     │  Gravity Vector │ Displacement │
│ 1 byte │   16 bytes       │   12 bytes      │   4 bytes    │
│        │ (4 × float)      │  (3 × float)    │  (1 × float) │
└────────┴──────────────────┴─────────────────┴──────────────┘
```

## Calibration Persistence

Calibration data is stored in EEPROM with a magic number (`0x42494D55`) to verify validity. The system automatically loads saved calibration on startup, eliminating the need to recalibrate after power cycles.

## Advantages of This Approach

1. **No integration errors** - Unlike accelerometers that require double integration, gravity-based measurement is direct
2. **Drift-free** - Gravity vector is an absolute reference
3. **Simple math** - Only requires vector dot product and trigonometry
4. **Low computational cost** - Suitable for Arduino
5. **Persistent calibration** - EEPROM storage means calibration survives restarts

## Limitations

1. **Assumes simple pendulum** - Wire must be non-stretchable and rigid
2. **Single plane measurement** - Displacement magnitude only (not direction)
3. **Requires calibration** - Must be calibrated at rest position
4. **Accuracy depends on**:
   - Wire length measurement accuracy
   - IMU sensor precision
   - Proper calibration at true vertical

## Troubleshooting

**Displacement shows 0.0 (NOT CALIBRATED)**
- Send 'C' to calibrate while hanging at rest

**Gravity magnitude not ~9.8 m/s²**
- Check IMU sensor connection
- Verify sensor is properly initialized

**No RS485 communication**
- Check A/B bus wiring
- Verify DE_PIN configuration
- Ensure all nodes share common ground

**Inaccurate displacement readings**
- Verify WIRE_LENGTH_CM matches physical setup
- Recalibrate at true vertical rest position
- Check for electromagnetic interference affecting IMU

## Dependencies

- Wire.h - I2C communication
- SPI.h - SPI support for BNO055
- Adafruit_Sensor.h - Unified sensor library
- Adafruit_BNO055.h - BNO055 IMU driver
- EEPROM.h - Non-volatile storage
- RS485.h - Custom RS485 protocol library
- nodeConfig.h - Node-specific configuration

## Authors

Lee Sheng Quan
