/**
 * @file GimbalController.h
 * @brief Industrial-grade 2-axis gimbal controller with trapezoidal motion profiles
 * @version 1.0.0
 * @author ESP32-S3 Gimbal Project
 * 
 * Features:
 * - Trapezoidal velocity profiles for smooth motion
 * - Non-blocking sequential servo control
 * - Thread-like task scheduling
 * - Serial command interface
 * - GoPro-style smooth gimbal behavior
 */

#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Motion profile states
enum MotionState {
    IDLE,
    ACCELERATING,
    CONSTANT_VELOCITY,
    DECELERATING,
    HOLDING
};

// Axis identifiers
enum GimbalAxis {
    PITCH = 0,  // Tilt (up/down)
    YAW = 1     // Pan (left/right)
};

/**
 * @class ServoAxis
 * @brief Manages a single servo axis with trapezoidal motion profile
 */
class ServoAxis {
private:
    Servo servo;
    uint8_t pin;
    
    // Position limits (degrees)
    float minAngle;
    float maxAngle;
    
    // Current state
    float currentPosition;
    float targetPosition;
    float currentVelocity;
    
    // Motion profile parameters
    float maxVelocity;        // deg/sec
    float acceleration;       // deg/sec²
    float deceleration;       // deg/sec²
    
    // Motion state
    MotionState state;
    unsigned long lastUpdateTime;
    
    // Calculated motion profile values
    float accelDistance;
    float decelDistance;
    float constantVelDistance;
    float accelTime;
    float decelTime;
    float totalTime;
    
    // Calculate motion profile
    void calculateMotionProfile();
    
public:
    /**
     * @brief Constructor
     * @param servoPin GPIO pin for servo signal
     * @param minDeg Minimum angle limit
     * @param maxDeg Maximum angle limit
     * @param maxVel Maximum velocity (deg/s)
     * @param accel Acceleration rate (deg/s²)
     */
    ServoAxis(uint8_t servoPin, float minDeg = 0.0f, float maxDeg = 180.0f, 
              float maxVel = 90.0f, float accel = 180.0f);
    
    /**
     * @brief Initialize the servo
     */
    void begin();
    
    /**
     * @brief Set target position with smooth motion
     * @param angle Target angle in degrees
     * @param useMotionProfile If true, uses trapezoidal profile; if false, instant move
     */
    void moveTo(float angle, bool useMotionProfile = true);
    
    /**
     * @brief Move relative to current position
     * @param deltaAngle Relative angle change
     */
    void moveRelative(float deltaAngle);
    
    /**
     * @brief Update servo position (call in loop)
     * @return true if motion is in progress
     */
    bool update();
    
    /**
     * @brief Check if axis is moving
     */
    bool isMoving() const { return state != IDLE && state != HOLDING; }
    
    /**
     * @brief Get current position
     */
    float getPosition() const { return currentPosition; }
    
    /**
     * @brief Get target position
     */
    float getTarget() const { return targetPosition; }
    
    /**
     * @brief Set motion parameters
     */
    void setMaxVelocity(float vel) { maxVelocity = vel; }
    void setAcceleration(float accel) { acceleration = deceleration = accel; }
    void setDeceleration(float decel) { deceleration = decel; }
    
    /**
     * @brief Set angle range limits
     * @param minDeg Minimum angle limit
     * @param maxDeg Maximum angle limit
     */
    void setAngleRange(float minDeg, float maxDeg);
    
    /**
     * @brief Set angle offset for calibration
     * @param offset Offset in degrees to add to all positions
     */
    void setAngleOffset(float offset);
    
    /**
     * @brief Get angle limits
     */
    float getMinAngle() const { return minAngle; }
    float getMaxAngle() const { return maxAngle; }
    float getAngleOffset() const { return angleOffset; }
    
    /**
     * @brief Emergency stop
     */
    void stop();
    
    /**
     * @brief Get current velocity
     */
    float getVelocity() const { return currentVelocity; }

private:
    float angleOffset;  // Calibration offset
};

/**
 * @class GimbalController
 * @brief Main controller for 2-axis gimbal with sequential task management
 */
class GimbalController {
private:
    ServoAxis* axes[2];
    
    // Task scheduling
    unsigned long lastUpdateTime;
    uint16_t updateInterval;  // microseconds between updates
    
    // Serial command buffer
    String commandBuffer;
    
    // Smoothing filter for position commands
    bool smoothingEnabled;
    float smoothingFactor;
    
    // Parse serial commands
    void parseCommand(String cmd);
    
public:
    /**
     * @brief Constructor
     * @param pitchPin GPIO pin for pitch servo
     * @param yawPin GPIO pin for yaw servo
     * @param updateRate Update rate in Hz (default 100Hz)
     */
    GimbalController(uint8_t pitchPin, uint8_t yawPin, uint16_t updateRate = 100);
    
    /**
     * @brief Destructor
     */
    ~GimbalController();
    
    /**
     * @brief Initialize gimbal system
     */
    void begin();
    
    /**
     * @brief Main update function - call in loop()
     * Implements sequential servo control (non-blocking)
     */
    void update();
    
    /**
     * @brief Move single axis
     * @param axis PITCH or YAW
     * @param angle Target angle
     * @param smooth Use motion profile
     */
    void moveAxis(GimbalAxis axis, float angle, bool smooth = true);
    
    /**
     * @brief Move both axes simultaneously
     * @param pitchAngle Target pitch angle
     * @param yawAngle Target yaw angle
     */
    void moveToPosition(float pitchAngle, float yawAngle);
    
    /**
     * @brief Set home position (center both axes)
     */
    void home();
    
    /**
     * @brief Process serial commands
     */
    void processSerial();
    
    /**
     * @brief Check if gimbal is moving
     */
    bool isMoving() const;
    
    /**
     * @brief Get axis reference
     */
    ServoAxis* getAxis(GimbalAxis axis) { return axes[axis]; }
    
    /**
     * @brief Emergency stop all axes
     */
    void stopAll();
    
    /**
     * @brief Enable/disable position smoothing
     */
    void setSmoothing(bool enable, float factor = 0.3f);
    
    /**
     * @brief Set custom alpha angle (pitch offset/calibration)
     * @param alpha Pitch offset in degrees
     */
    void setAlphaAngle(float alpha);
    
    /**
     * @brief Set custom beta angle (yaw offset/calibration)
     * @param beta Yaw offset in degrees
     */
    void setBetaAngle(float beta);
    
    /**
     * @brief Set angle range for specific axis
     * @param axis PITCH or YAW
     * @param minDeg Minimum angle
     * @param maxDeg Maximum angle
     */
    void setAxisAngleRange(GimbalAxis axis, float minDeg, float maxDeg);
    
    /**
     * @brief Get alpha angle (pitch offset)
     */
    float getAlphaAngle() const { return alphaAngle; }
    
    /**
     * @brief Get beta angle (yaw offset)
     */
    float getBetaAngle() const { return betaAngle; }
    
    /**
     * @brief Calibrate - set current position as new zero point
     * @param axis PITCH or YAW
     */
    void calibrateAxis(GimbalAxis axis);
    
    /**
     * @brief Reset calibration to defaults
     */
    void resetCalibration();
    
    /**
     * @brief Print status to serial
     */
    void printStatus();

private:
    // Calibration angles
    float alphaAngle;  // Pitch offset
    float betaAngle;   // Yaw offset
};

#endif // GIMBAL_CONTROLLER_H
