# ESP32-S3 CAM 2-Axis Gimbal Controller

Industrial-grade gimbal control library with trapezoidal motion profiles and GoPro-style smooth movements.

## Features

✅ **Trapezoidal Motion Profiles** - Smooth acceleration and deceleration  
✅ **Non-Blocking Sequential Control** - Thread-like task execution without RTOS  
✅ **Serial Command Interface** - Easy control via UART  
✅ **GoPro-Grade Smoothness** - Professional gimbal behavior  
✅ **Configurable Parameters** - Velocity, acceleration, limits  
✅ **Emergency Stop** - Safety features built-in  

## Hardware Requirements

- **Board**: ESP32-S3 CAM
- **Servos**: 2x standard hobby servos (e.g., SG90, MG996R)
- **Connections**:
  - Servo 1 (Pitch) → GPIO 1
  - Servo 2 (Yaw) → GPIO 2
- **Power**: External 5V power supply for servos (recommended)
- **Common Ground**: Connect ESP32 GND to servo power supply GND

## Installation

### Arduino IDE

1. Install **ESP32Servo** library:
   - Open Arduino IDE
   - Go to `Sketch` → `Include Library` → `Manage Libraries`
   - Search for "ESP32Servo"
   - Install by Kevin Harrington

2. Copy library files to your project:
   ```
   YourProject/
   ├── GimbalExample.ino
   ├── GimbalController.h
   └── GimbalController.cpp
   ```

3. Select board: `ESP32S3 Dev Module`

4. Upload and open Serial Monitor at **115200 baud**

## Quick Start

```cpp
#include "GimbalController.h"

// Initialize gimbal on GPIO 1 and 2, 100Hz update rate
GimbalController gimbal(1, 2, 100);

void setup() {
    Serial.begin(115200);
    gimbal.begin();
    gimbal.home(); // Move to center position
}

void loop() {
    gimbal.update();           // MUST be called every loop
    gimbal.processSerial();    // Handle serial commands
}
```

## Serial Commands

| Command | Description | Example |
|---------|-------------|---------|
| `P<angle>` | Move pitch to angle (0-180°) | `P45` |
| `Y<angle>` | Move yaw to angle (0-180°) | `Y90` |
| `M<p>,<y>` | Move both axes | `M45,90` |
| `H` | Home position (90°, 90°) | `H` |
| `S` | Emergency stop | `S` |
| `?` | Print current status | `?` |
| `V<speed>` | Set max velocity (deg/s) | `V60` |
| `A<accel>` | Set acceleration (deg/s²) | `A150` |
| `R<axis>,<min>,<max>` | Set angle range for axis | `RP,30,150` |
| `O<axis><offset>` | Set alpha/beta offset | `OP10` or `OY-5` |
| `C<axis>` | Calibrate axis | `CP` or `CY` |
| `CR` | Reset calibration | `CR` |

### Command Examples

```
> P45          // Move pitch to 45°
Pitch -> 45.0°

> M60,120      // Move to pitch=60°, yaw=120°
Move -> Pitch: 60.0°, Yaw: 120.0°

> RP,30,150    // Set pitch angle range to 30-150°
Pitch axis range: 30.0° to 150.0°

> OP10         // Set pitch offset (alpha) to +10°
Alpha angle (pitch offset) set to 10.0°

> OY-5         // Set yaw offset (beta) to -5°
Beta angle (yaw offset) set to -5.0°

> CP           // Calibrate pitch (sets current as reference)
Pitch calibrated: offset = 12.3°

> ?            // Check status
=== Gimbal Status ===
Pitch: 60.0° -> 60.0° (0.0 deg/s) [IDLE]
Yaw:   120.0° -> 120.0° (0.0 deg/s) [IDLE]

Calibration:
  Alpha (Pitch offset): 10.0°
  Beta (Yaw offset):    -5.0°

Angle Ranges:
  Pitch: 30.0° to 150.0°
  Yaw:   0.0° to 180.0°
====================
```

## Angle Range & Calibration

### Setting Angle Ranges

If your gimbal has mechanical limits or can't reach full 180° rotation:

```cpp
void setup() {
    gimbal.begin();
    
    // Set pitch range (e.g., mechanical stops at 30° and 150°)
    gimbal.setAxisAngleRange(PITCH, 30.0f, 150.0f);
    
    // Set yaw range (e.g., only 270° rotation)
    gimbal.setAxisAngleRange(YAW, 0.0f, 270.0f);
}
```

**Serial command:**
```
RP,30,150      // Set pitch range
RY,0,270       // Set yaw range
```

### Alpha & Beta Angles (Calibration Offsets)

Use these when servos are mounted at an angle or need alignment:

- **Alpha**: Pitch axis offset (compensates for forward/backward tilt)
- **Beta**: Yaw axis offset (compensates for left/right rotation)

```cpp
void setup() {
    gimbal.begin();
    
    // Servo mounted 10° forward - add offset to compensate
    gimbal.setAlphaAngle(10.0f);
    
    // Servo mounted 5° to the right - subtract to compensate  
    gimbal.setBetaAngle(-5.0f);
}
```

**Serial commands:**
```
OP10           // Set alpha (pitch offset) to +10°
OY-5           // Set beta (yaw offset) to -5°
```

### Interactive Calibration

Physically position the gimbal and set that as the reference:

```cpp
// 1. Manually move gimbal to level position
// 2. Call calibration:
gimbal.calibrateAxis(PITCH);  // Sets current position as new zero

// Or via serial:
// Send: CP (calibrate pitch)
// Send: CY (calibrate yaw)
// Send: CR (reset calibration)
```

### Real-World Example: Camera Gimbal

```cpp
void setup() {
    gimbal.begin();
    
    // Physical constraints of camera gimbal:
    // - Can't tilt below 20° (camera hits body)
    // - Can't tilt above 160° (cable interference)
    // - Pan limited to 45° - 315° (270° total)
    
    gimbal.setAxisAngleRange(PITCH, 20.0f, 160.0f);
    gimbal.setAxisAngleRange(YAW, 45.0f, 315.0f);
    
    // Servo mounting compensation:
    // - Pitch servo mounted 5° forward
    gimbal.setAlphaAngle(5.0f);
    gimbal.setBetaAngle(0.0f);
}
```

### Saving Calibration to Flash

```cpp
#include <Preferences.h>

void saveCalibration() {
    Preferences prefs;
    prefs.begin("gimbal", false);
    
    prefs.putFloat("alpha", gimbal.getAlphaAngle());
    prefs.putFloat("beta", gimbal.getBetaAngle());
    prefs.putFloat("p_min", gimbal.getAxis(PITCH)->getMinAngle());
    prefs.putFloat("p_max", gimbal.getAxis(PITCH)->getMaxAngle());
    
    prefs.end();
}

void loadCalibration() {
    Preferences prefs;
    prefs.begin("gimbal", true);
    
    if (prefs.isKey("alpha")) {
        gimbal.setAlphaAngle(prefs.getFloat("alpha"));
        gimbal.setBetaAngle(prefs.getFloat("beta"));
        gimbal.setAxisAngleRange(PITCH, 
            prefs.getFloat("p_min"), 
            prefs.getFloat("p_max"));
    }
    
    prefs.end();
}
```

## API Reference

### GimbalController Class

```cpp
// Constructor
GimbalController(uint8_t pitchPin, uint8_t yawPin, uint16_t updateRate = 100);

// Essential Methods
void begin();                              // Initialize system
void update();                             // Call in loop() - handles motion
void processSerial();                      // Process serial commands

// Movement Control
void moveAxis(GimbalAxis axis, float angle, bool smooth = true);
void moveToPosition(float pitchAngle, float yawAngle);
void home();                               // Move to center (90°, 90°)
void stopAll();                            // Emergency stop

// Status
bool isMoving() const;                     // Check if moving
void printStatus();                        // Print to serial

// Access individual axes
ServoAxis* getAxis(GimbalAxis axis);       // Get PITCH or YAW axis

// Angle Range & Calibration
void setAxisAngleRange(GimbalAxis axis, float minDeg, float maxDeg);
void setAlphaAngle(float alpha);           // Pitch offset
void setBetaAngle(float beta);             // Yaw offset
void calibrateAxis(GimbalAxis axis);       // Calibrate to current position
void resetCalibration();                   // Reset to defaults
float getAlphaAngle() const;
float getBetaAngle() const;
```

### ServoAxis Class

```cpp
// Movement
void moveTo(float angle, bool useMotionProfile = true);
void moveRelative(float deltaAngle);
bool update();                             // Returns true if moving
void stop();                               // Stop this axis

// Configuration
void setMaxVelocity(float vel);           // deg/s
void setAcceleration(float accel);        // deg/s²
void setDeceleration(float decel);        // deg/s²
void setAngleRange(float minDeg, float maxDeg);
void setAngleOffset(float offset);        // Calibration offset

// Status
float getPosition() const;
float getTarget() const;
float getVelocity() const;
bool isMoving() const;
float getMinAngle() const;
float getMaxAngle() const;
float getAngleOffset() const;
```

## Advanced Usage

### Custom Motion Parameters

```cpp
void setup() {
    gimbal.begin();
    
    // Slower, smoother motion for cinematic shots
    gimbal.getAxis(PITCH)->setMaxVelocity(30.0f);    // 30 deg/s
    gimbal.getAxis(PITCH)->setAcceleration(60.0f);   // 60 deg/s²
    
    // Faster yaw for quick pans
    gimbal.getAxis(YAW)->setMaxVelocity(150.0f);
    gimbal.getAxis(YAW)->setAcceleration(300.0f);
}
```

### Smooth Tracking Pattern

```cpp
void loop() {
    gimbal.update();
    
    static float angle = 0;
    angle += 0.5;
    
    // Smooth sine wave motion
    float pitch = 90 + 30 * sin(radians(angle));
    float yaw = 90 + 40 * cos(radians(angle * 0.7));
    
    if (!gimbal.isMoving()) {
        gimbal.moveToPosition(pitch, yaw);
    }
}
```

### Joystick/Analog Control

```cpp
void loop() {
    gimbal.update();
    
    int joyX = analogRead(34);  // Read analog pin
    int joyY = analogRead(35);
    
    // Map to servo range with deadzone
    if (abs(joyX - 2048) > 100 || abs(joyY - 2048) > 100) {
        float yaw = map(joyX, 0, 4095, 0, 180);
        float pitch = map(joyY, 0, 4095, 0, 180);
        gimbal.moveToPosition(pitch, yaw);
    }
}
```

### Camera Stabilization (with IMU)

```cpp
#include <MPU6050.h>  // Example IMU library

MPU6050 imu;

void loop() {
    gimbal.update();
    
    // Read IMU orientation
    float roll, pitch, yaw;
    imu.getOrientation(&roll, &pitch, &yaw);
    
    // Compensate camera tilt
    float targetPitch = 90 - pitch;  // Inverse to stabilize
    float targetYaw = gimbal.getAxis(YAW)->getPosition();
    
    gimbal.moveToPosition(targetPitch, targetYaw);
}
```

## How It Works

### Trapezoidal Motion Profile

The library implements a 3-phase motion profile:

1. **Acceleration Phase**: Smooth ramp-up from 0 to max velocity
2. **Constant Velocity Phase**: Travel at max speed (if distance allows)
3. **Deceleration Phase**: Smooth ramp-down to stop at target

```
Velocity
   ↑
   │      ╱────────╲
   │     ╱          ╲
   │    ╱            ╲
   │   ╱              ╲
   └──────────────────────→ Time
      Accel  Const  Decel
```

This creates the smooth, professional motion you see in GoPro gimbals.

### Sequential Control (Cooperative Multitasking)

Both servos update sequentially in each loop iteration, but with precise timing:
- Update rate: 100Hz (10ms intervals)
- Each axis calculates its position independently
- Non-blocking - your code continues running
- No RTOS overhead

## Performance Tuning

### For Smooth Cinematic Shots
```cpp
gimbal.getAxis(PITCH)->setMaxVelocity(40.0f);
gimbal.getAxis(PITCH)->setAcceleration(80.0f);
gimbal.getAxis(YAW)->setMaxVelocity(50.0f);
gimbal.getAxis(YAW)->setAcceleration(100.0f);
```

### For Fast Action Tracking
```cpp
gimbal.getAxis(PITCH)->setMaxVelocity(150.0f);
gimbal.getAxis(PITCH)->setAcceleration(300.0f);
gimbal.getAxis(YAW)->setMaxVelocity(200.0f);
gimbal.getAxis(YAW)->setAcceleration(400.0f);
```

### For Maximum Precision
```cpp
GimbalController gimbal(1, 2, 200);  // 200Hz update rate
gimbal.getAxis(PITCH)->setMaxVelocity(20.0f);
gimbal.getAxis(PITCH)->setAcceleration(40.0f);
```

## Troubleshooting

**Servos jittering:**
- Add external capacitor (100-470µF) across servo power supply
- Ensure common ground connection
- Lower update rate: `GimbalController gimbal(1, 2, 50);`

**Motion not smooth:**
- Increase acceleration: `setAcceleration(200.0f)`
- Check servo quality (cheap servos have backlash)

**Commands not working:**
- Check baud rate: 115200
- Add newline/carriage return after commands
- Verify GPIO pins are correct

**Servo not reaching target:**
- Check angle limits (default 0-180°)
- Verify servo pulse width range (500-2500µs)

**Gimbal hits mechanical stops:**
- Set appropriate angle ranges: `setAxisAngleRange(PITCH, 30, 150)`
- Test incrementally to find safe limits

**Servo mounted at wrong angle:**
- Use alpha/beta offsets: `setAlphaAngle(10)` or `setBetaAngle(-5)`
- Or use interactive calibration: send `CP` or `CY` after positioning

**Angles seem inverted or reversed:**
- Use negative offsets: `setAlphaAngle(-90)` to flip 180°
- Or swap min/max in angle range (not recommended)

**Calibration not saving:**
- Implement flash storage using Preferences library (see example)
- Call `saveCalibration()` after setting parameters

## License

MIT License - Use freely in commercial and personal projects

## Credits

Developed for ESP32-S3 CAM gimbal applications  
Trapezoidal motion algorithm based on industrial motion control principles
