# Multi-Node IMU Pendulum System

## System Architecture

```
Raspberry Pi (Python Master)
         |
    USB-RS485
         |
   RS485 Bus
         |
    ┌────┼────┬────┬─────┬────┐
    |    |    |    |     |    |
  Node1 Node2 ... Node9 Node10
  (Arduino + BNO055 IMU)
```

## Core Idea

Use gravity vector measurement to calculate horizontal displacement of pendulums. When a pendulum tilts, the angle between current gravity and reference gravity gives the tilt angle θ, then displacement = L × sin(θ).

## Why Simultaneous Calibration is Critical

**Problem:** When multiple pendulums hang from the same structure, adding nodes changes the center of gravity, which affects ALL pendulum equilibrium positions.

**Possible Miss Calibration**
- Hang Node 1 → Calibrate → position A
- Add Node 2 → shifts to position B → Node 1 calibration INVALID

**Suggested Approach:**
1. Hang ALL 10 nodes at once
2. Wait for complete stabilization
3. Raspberry Pi broadcasts "CALIB" command to all nodes (address 255)
4. All nodes calibrate simultaneously

## Workflow

### Setup
1. Configure each Arduino with unique NODE_ID (1-10) in `nodeConfig.h`
2. Set `WIRE_LENGTH_CM` in each node's code
3. Upload `imu.ino` to all Arduinos
4. Wire RS485 bus with 120Ω termination at both ends
5. Connect USB-RS485 adapter to Raspberry Pi

### Calibration
1. Hang all 10 nodes vertically
2. Wait for stillness 
3. Python script send calibration command via RS485
4. All nodes simultaneously:
   - Read gravity vector
   - Store as reference
   - Save to EEPROM

## Algorithm - 2D Displacement Using Tilt Angles

**Calibration (at rest):**
```
g_ref = read_gravity_vector()
save_to_eeprom(g_ref)
```

**Measurement (during swing) - FULL 2D VECTOR:**
```
g_current = read_gravity_vector()

// Step 1: Calculate reference tilt angles (from calibration)
refTiltX = atan2(g_ref.x, g_ref.z)
refTiltY = atan2(g_ref.y, g_ref.z)

// Step 2: Calculate current tilt angles
curTiltX = atan2(g_current.x, g_current.z)
curTiltY = atan2(g_current.y, g_current.z)

// Step 3: Calculate change in tilt (automatically signed by atan2)
ΔtiltX = curTiltX - refTiltX
ΔtiltY = curTiltY - refTiltY

// Step 4: Calculate 2D displacement vector
displacement.x = WIRE_LENGTH_CM × sin(ΔtiltX)
displacement.y = WIRE_LENGTH_CM × sin(ΔtiltY)
displacement.magnitude = √(x² + y²)
```

**Key Advantages:**
- Returns **full 2D vector**: (x, y) tells you exact direction
- `atan2` automatically handles sign (±180°)
- No need to define measurement axis
- Works for any swing direction
- Can visualize as X-Y plot

**Example Output:**
```
Node 1: displacement = (+12.5, -8.3) cm, magnitude = 15.0 cm
  → Pendulum swung in +X and -Y direction (diagonal)

Node 2: displacement = (0.0, +15.7) cm, magnitude = 15.7 cm
  → Pendulum swung purely in +Y direction
```

