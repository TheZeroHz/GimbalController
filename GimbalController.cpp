/**
 * @file GimbalController.cpp
 * @brief Implementation of industrial-grade gimbal controller
 */

#include "GimbalController.h"

// ============================================================================
// ServoAxis Implementation
// ============================================================================

ServoAxis::ServoAxis(uint8_t servoPin, float minDeg, float maxDeg, 
                     float maxVel, float accel)
    : pin(servoPin),
      minAngle(minDeg),
      maxAngle(maxDeg),
      currentPosition((minDeg + maxDeg) / 2.0f),
      targetPosition((minDeg + maxDeg) / 2.0f),
      currentVelocity(0.0f),
      maxVelocity(maxVel),
      acceleration(accel),
      deceleration(accel),
      state(IDLE),
      lastUpdateTime(0),
      angleOffset(0.0f) {
}

void ServoAxis::begin() {
    servo.attach(pin, 500, 2500);  // Standard servo pulse width
    int servoAngle = (int)(currentPosition + angleOffset);
    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
    lastUpdateTime = micros();
    state = HOLDING;
}

void ServoAxis::calculateMotionProfile() {
    float distance = abs(targetPosition - currentPosition);
    
    if (distance < 0.1f) {
        state = HOLDING;
        return;
    }
    
    // Calculate acceleration and deceleration distances
    accelTime = maxVelocity / acceleration;
    decelTime = maxVelocity / deceleration;
    accelDistance = 0.5f * acceleration * accelTime * accelTime;
    decelDistance = 0.5f * deceleration * decelTime * decelTime;
    
    // Check if we can reach max velocity
    if (accelDistance + decelDistance > distance) {
        // Triangular profile (never reach max velocity)
        float peakVel = sqrt(2.0f * distance * acceleration * deceleration / 
                            (acceleration + deceleration));
        accelTime = peakVel / acceleration;
        decelTime = peakVel / deceleration;
        accelDistance = 0.5f * acceleration * accelTime * accelTime;
        decelDistance = 0.5f * deceleration * decelTime * decelTime;
        constantVelDistance = 0.0f;
    } else {
        // Trapezoidal profile
        constantVelDistance = distance - accelDistance - decelDistance;
    }
    
    totalTime = accelTime + (constantVelDistance / maxVelocity) + decelTime;
    state = ACCELERATING;
}

void ServoAxis::moveTo(float angle, bool useMotionProfile) {
    // Clamp to limits
    targetPosition = constrain(angle, minAngle, maxAngle);
    
    if (!useMotionProfile) {
        currentPosition = targetPosition;
        currentVelocity = 0.0f;
        int servoAngle = (int)(currentPosition + angleOffset);
        servoAngle = constrain(servoAngle, 0, 180);
        servo.write(servoAngle);
        state = HOLDING;
        return;
    }
    
    currentVelocity = 0.0f;
    calculateMotionProfile();
}

void ServoAxis::moveRelative(float deltaAngle) {
    moveTo(currentPosition + deltaAngle, true);
}

bool ServoAxis::update() {
    if (state == IDLE || state == HOLDING) {
        return false;
    }
    
    unsigned long currentTime = micros();
    float dt = (currentTime - lastUpdateTime) / 1000000.0f; // Convert to seconds
    lastUpdateTime = currentTime;
    
    if (dt > 0.1f) dt = 0.001f; // Prevent huge jumps
    
    float distanceToTarget = targetPosition - currentPosition;
    float absDistance = abs(distanceToTarget);
    int direction = (distanceToTarget > 0) ? 1 : -1;
    
    // Trapezoidal motion profile state machine
    switch (state) {
        case ACCELERATING: {
            currentVelocity += acceleration * dt;
            
            // Check if we need to start decelerating
            float decelStartDistance = (currentVelocity * currentVelocity) / 
                                      (2.0f * deceleration);
            
            if (currentVelocity >= maxVelocity) {
                currentVelocity = maxVelocity;
                state = CONSTANT_VELOCITY;
            } else if (absDistance <= decelStartDistance) {
                state = DECELERATING;
            }
            break;
        }
        
        case CONSTANT_VELOCITY: {
            // Check if we need to start decelerating
            float decelStartDistance = (currentVelocity * currentVelocity) / 
                                      (2.0f * deceleration);
            
            if (absDistance <= decelStartDistance) {
                state = DECELERATING;
            }
            break;
        }
        
        case DECELERATING: {
            currentVelocity -= deceleration * dt;
            
            if (currentVelocity < 0.0f) {
                currentVelocity = 0.0f;
            }
            
            if (absDistance < 0.5f || currentVelocity < 0.1f) {
                currentPosition = targetPosition;
                currentVelocity = 0.0f;
                state = HOLDING;
                int servoAngle = (int)(currentPosition + angleOffset);
                servoAngle = constrain(servoAngle, 0, 180);
                servo.write(servoAngle);
                return false;
            }
            break;
        }
        
        default:
            return false;
    }
    
    // Update position
    float positionChange = currentVelocity * dt * direction;
    
    // Don't overshoot
    if (abs(positionChange) > absDistance) {
        currentPosition = targetPosition;
        currentVelocity = 0.0f;
        state = HOLDING;
    } else {
        currentPosition += positionChange;
    }
    
    // Write to servo (apply offset)
    int servoAngle = (int)(currentPosition + angleOffset);
    servoAngle = constrain(servoAngle, 0, 180);
    servo.write(servoAngle);
    
    return true;
}

void ServoAxis::stop() {
    targetPosition = currentPosition;
    currentVelocity = 0.0f;
    state = HOLDING;
}

void ServoAxis::setAngleRange(float minDeg, float maxDeg) {
    if (minDeg < maxDeg) {
        minAngle = minDeg;
        maxAngle = maxDeg;
        
        // Clamp current positions to new range
        currentPosition = constrain(currentPosition, minAngle, maxAngle);
        targetPosition = constrain(targetPosition, minAngle, maxAngle);
        
        Serial.printf("Angle range set: %.1f° to %.1f°\n", minAngle, maxAngle);
    }
}

void ServoAxis::setAngleOffset(float offset) {
    angleOffset = offset;
    Serial.printf("Angle offset set: %.1f°\n", angleOffset);
}

// ============================================================================
// GimbalController Implementation
// ============================================================================

GimbalController::GimbalController(uint8_t pitchPin, uint8_t yawPin, 
                                   uint16_t updateRate)
    : lastUpdateTime(0),
      smoothingEnabled(false),
      smoothingFactor(0.3f),
      alphaAngle(0.0f),
      betaAngle(0.0f) {
    
    updateInterval = 1000000 / updateRate; // Convert Hz to microseconds
    
    // Create pitch axis (GPIO 1) - typical range 0-180°
    axes[PITCH] = new ServoAxis(pitchPin, 0.0f, 180.0f, 90.0f, 180.0f);
    
    // Create yaw axis (GPIO 2) - typical range 0-180°
    axes[YAW] = new ServoAxis(yawPin, 0.0f, 180.0f, 120.0f, 200.0f);
}

GimbalController::~GimbalController() {
    delete axes[PITCH];
    delete axes[YAW];
}

void GimbalController::begin() {
    axes[PITCH]->begin();
    axes[YAW]->begin();
    lastUpdateTime = micros();
    
    Serial.println("=== Gimbal Controller Initialized ===");
    Serial.println("Commands:");
    Serial.println("  P<angle>       - Move pitch to angle (0-180)");
    Serial.println("  Y<angle>       - Move yaw to angle (0-180)");
    Serial.println("  M<p>,<y>       - Move both axes (pitch,yaw)");
    Serial.println("  H              - Home position (center)");
    Serial.println("  S              - Stop all motion");
    Serial.println("  ?              - Print status");
    Serial.println("  V<speed>       - Set max velocity (deg/s)");
    Serial.println("  A<accel>       - Set acceleration (deg/s²)");
    Serial.println("  R<axis>,<min>,<max> - Set angle range");
    Serial.println("                   Example: RP,30,150");
    Serial.println("  O<axis><value> - Set offset (alpha/beta)");
    Serial.println("                   Example: OP5 or OY-3");
    Serial.println("  C<axis>        - Calibrate axis (CP/CY)");
    Serial.println("  CR             - Reset calibration");
    Serial.println("=====================================\n");
}

void GimbalController::update() {
    unsigned long currentTime = micros();
    
    // Sequential update (cooperative multitasking)
    if (currentTime - lastUpdateTime >= updateInterval) {
        lastUpdateTime = currentTime;
        
        // Update both axes sequentially (non-blocking)
        axes[PITCH]->update();
        axes[YAW]->update();
    }
}

void GimbalController::moveAxis(GimbalAxis axis, float angle, bool smooth) {
    if (axis >= 0 && axis < 2) {
        axes[axis]->moveTo(angle, smooth);
    }
}

void GimbalController::moveToPosition(float pitchAngle, float yawAngle) {
    axes[PITCH]->moveTo(pitchAngle, true);
    axes[YAW]->moveTo(yawAngle, true);
}

void GimbalController::home() {
    moveToPosition(90.0f, 90.0f); // Center position
    Serial.println("Homing to center position...");
}

void GimbalController::processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (commandBuffer.length() > 0) {
                parseCommand(commandBuffer);
                commandBuffer = "";
            }
        } else {
            commandBuffer += c;
        }
    }
}

void GimbalController::parseCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd.length() == 0) return;
    
    char command = cmd.charAt(0);
    String value = cmd.substring(1);
    
    switch (command) {
        case 'P': {
            // Pitch control
            float angle = value.toFloat();
            moveAxis(PITCH, angle, true);
            Serial.printf("Pitch -> %.1f°\n", angle);
            break;
        }
        
        case 'Y': {
            // Yaw control
            float angle = value.toFloat();
            moveAxis(YAW, angle, true);
            Serial.printf("Yaw -> %.1f°\n", angle);
            break;
        }
        
        case 'M': {
            // Move both axes
            int commaIndex = value.indexOf(',');
            if (commaIndex > 0) {
                float pitch = value.substring(0, commaIndex).toFloat();
                float yaw = value.substring(commaIndex + 1).toFloat();
                moveToPosition(pitch, yaw);
                Serial.printf("Move -> Pitch: %.1f°, Yaw: %.1f°\n", pitch, yaw);
            }
            break;
        }
        
        case 'H': {
            // Home
            home();
            break;
        }
        
        case 'S': {
            // Stop
            stopAll();
            Serial.println("Emergency stop!");
            break;
        }
        
        case '?': {
            // Status
            printStatus();
            break;
        }
        
        case 'V': {
            // Set velocity
            float vel = value.toFloat();
            axes[PITCH]->setMaxVelocity(vel);
            axes[YAW]->setMaxVelocity(vel * 1.2f); // Yaw slightly faster
            Serial.printf("Max velocity set to %.1f deg/s\n", vel);
            break;
        }
        
        case 'A': {
            // Set acceleration
            float accel = value.toFloat();
            axes[PITCH]->setAcceleration(accel);
            axes[YAW]->setAcceleration(accel);
            Serial.printf("Acceleration set to %.1f deg/s²\n", accel);
            break;
        }
        
        case 'R': {
            // Set angle range: R<axis>,<min>,<max>
            // Example: RP,30,150 or RY,0,180
            if (value.length() > 0) {
                char axis = value.charAt(0);
                int comma1 = value.indexOf(',', 1);
                int comma2 = value.indexOf(',', comma1 + 1);
                
                if (comma1 > 0 && comma2 > 0) {
                    float minAngle = value.substring(comma1 + 1, comma2).toFloat();
                    float maxAngle = value.substring(comma2 + 1).toFloat();
                    
                    if (axis == 'P') {
                        setAxisAngleRange(PITCH, minAngle, maxAngle);
                    } else if (axis == 'Y') {
                        setAxisAngleRange(YAW, minAngle, maxAngle);
                    }
                }
            }
            break;
        }
        
        case 'O': {
            // Set offset (alpha/beta): OP<value> or OY<value>
            // Example: OP5 (pitch offset +5°) or OY-3 (yaw offset -3°)
            if (value.length() > 0) {
                char axis = value.charAt(0);
                float offset = value.substring(1).toFloat();
                
                if (axis == 'P') {
                    setAlphaAngle(offset);
                } else if (axis == 'Y') {
                    setBetaAngle(offset);
                }
            }
            break;
        }
        
        case 'C': {
            // Calibrate: CP or CY (set current position as reference)
            if (value.length() > 0) {
                char axis = value.charAt(0);
                if (axis == 'P') {
                    calibrateAxis(PITCH);
                } else if (axis == 'Y') {
                    calibrateAxis(YAW);
                } else if (axis == 'R') {
                    resetCalibration();
                }
            }
            break;
        }
        
        default:
            Serial.println("Unknown command. Send '?' for help.");
            break;
    }
}

bool GimbalController::isMoving() const {
    return axes[PITCH]->isMoving() || axes[YAW]->isMoving();
}

void GimbalController::stopAll() {
    axes[PITCH]->stop();
    axes[YAW]->stop();
}

void GimbalController::setSmoothing(bool enable, float factor) {
    smoothingEnabled = enable;
    smoothingFactor = constrain(factor, 0.0f, 1.0f);
}

void GimbalController::setAlphaAngle(float alpha) {
    alphaAngle = alpha;
    axes[PITCH]->setAngleOffset(alphaAngle);
    Serial.printf("Alpha angle (pitch offset) set to %.1f°\n", alphaAngle);
}

void GimbalController::setBetaAngle(float beta) {
    betaAngle = beta;
    axes[YAW]->setAngleOffset(betaAngle);
    Serial.printf("Beta angle (yaw offset) set to %.1f°\n", betaAngle);
}

void GimbalController::setAxisAngleRange(GimbalAxis axis, float minDeg, float maxDeg) {
    if (axis >= 0 && axis < 2) {
        axes[axis]->setAngleRange(minDeg, maxDeg);
        Serial.printf("%s axis range: %.1f° to %.1f°\n", 
                     (axis == PITCH) ? "Pitch" : "Yaw", minDeg, maxDeg);
    }
}

void GimbalController::calibrateAxis(GimbalAxis axis) {
    if (axis >= 0 && axis < 2) {
        float currentPos = axes[axis]->getPosition();
        float offset = 90.0f - currentPos;  // Calculate offset to make current position = 90°
        
        if (axis == PITCH) {
            alphaAngle = offset;
            axes[PITCH]->setAngleOffset(alphaAngle);
            Serial.printf("Pitch calibrated: offset = %.1f°\n", alphaAngle);
        } else {
            betaAngle = offset;
            axes[YAW]->setAngleOffset(betaAngle);
            Serial.printf("Yaw calibrated: offset = %.1f°\n", betaAngle);
        }
    }
}

void GimbalController::resetCalibration() {
    alphaAngle = 0.0f;
    betaAngle = 0.0f;
    axes[PITCH]->setAngleOffset(0.0f);
    axes[YAW]->setAngleOffset(0.0f);
    Serial.println("Calibration reset to defaults");
}

void GimbalController::printStatus() {
    Serial.println("\n=== Gimbal Status ===");
    Serial.printf("Pitch: %.1f° -> %.1f° (%.1f deg/s) %s\n",
                  axes[PITCH]->getPosition(),
                  axes[PITCH]->getTarget(),
                  axes[PITCH]->getVelocity(),
                  axes[PITCH]->isMoving() ? "[MOVING]" : "[IDLE]");
    
    Serial.printf("Yaw:   %.1f° -> %.1f° (%.1f deg/s) %s\n",
                  axes[YAW]->getPosition(),
                  axes[YAW]->getTarget(),
                  axes[YAW]->getVelocity(),
                  axes[YAW]->isMoving() ? "[MOVING]" : "[IDLE]");
    
    Serial.printf("\nCalibration:\n");
    Serial.printf("  Alpha (Pitch offset): %.1f°\n", alphaAngle);
    Serial.printf("  Beta (Yaw offset):    %.1f°\n", betaAngle);
    
    Serial.printf("\nAngle Ranges:\n");
    Serial.printf("  Pitch: %.1f° to %.1f°\n", 
                 axes[PITCH]->getMinAngle(), axes[PITCH]->getMaxAngle());
    Serial.printf("  Yaw:   %.1f° to %.1f°\n", 
                 axes[YAW]->getMinAngle(), axes[YAW]->getMaxAngle());
    Serial.println("====================\n");
}
