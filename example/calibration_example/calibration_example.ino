/**
 * @file CalibrationExample.ino
 * @brief Advanced example showing angle range and alpha/beta calibration
 * 
 * This example demonstrates:
 * - Setting custom angle ranges for each axis
 * - Using alpha (pitch offset) and beta (yaw offset) parameters
 * - Calibrating the gimbal to physical constraints
 * - Handling mechanical limits and servo alignment
 * 
 * Hardware Notes:
 * - If your servos don't reach full 0-180° range
 * - If your gimbal has physical mechanical stops
 * - If your servos are mounted at an offset angle
 * - Use these methods to configure proper operation
 */

#include "GimbalController.h"

#define SERVO_PITCH_PIN  1
#define SERVO_YAW_PIN    2

GimbalController gimbal(SERVO_PITCH_PIN, SERVO_YAW_PIN, 100);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║  Gimbal Calibration & Range Setup Demo   ║");
    Serial.println("╚════════════════════════════════════════════╝\n");
    
    // Initialize gimbal
    gimbal.begin();
    delay(500);
    
    // ========================================================================
    // EXAMPLE 1: Set Custom Angle Ranges
    // ========================================================================
    Serial.println(">>> Setting custom angle ranges...\n");
    
    // If your gimbal has mechanical stops at 30° and 150° for pitch:
    gimbal.setAxisAngleRange(PITCH, 30.0f, 150.0f);
    
    // If your yaw can only rotate 270° (e.g., 0° to 270°):
    gimbal.setAxisAngleRange(YAW, 0.0f, 270.0f);
    
    Serial.println("Angle ranges configured!\n");
    delay(1000);
    
    // ========================================================================
    // EXAMPLE 2: Set Alpha/Beta Offsets for Servo Alignment
    // ========================================================================
    Serial.println(">>> Setting alpha/beta offsets...\n");
    
    // Alpha: Pitch offset (if servo is mounted at 10° forward)
    // This adds 10° to all pitch commands to compensate
    gimbal.setAlphaAngle(10.0f);
    
    // Beta: Yaw offset (if servo is mounted 5° to the right)
    // This subtracts 5° from all yaw commands
    gimbal.setBetaAngle(-5.0f);
    
    Serial.println("Offsets applied!\n");
    delay(1000);
    
    // ========================================================================
    // EXAMPLE 3: Interactive Calibration
    // ========================================================================
    Serial.println(">>> Interactive Calibration Mode\n");
    Serial.println("This will help you find the correct alpha/beta values:\n");
    
    Serial.println("1. Manually move gimbal to LEVEL position");
    Serial.println("2. Send 'CP' to calibrate pitch");
    Serial.println("3. Manually move gimbal to CENTER position");
    Serial.println("4. Send 'CY' to calibrate yaw");
    Serial.println("5. Send '?' to verify calibration\n");
    
    // Move to home to start
    gimbal.home();
    
    Serial.println("Ready for calibration commands!\n");
    Serial.println("Alternatively, send 'DEMO' to run automated demo\n");
}

void loop() {
    // Update gimbal (REQUIRED!)
    gimbal.update();
    
    // Process serial commands
    processCustomCommands();
    gimbal.processSerial();
    
    delay(1);
}

/**
 * Handle custom commands for demo
 */
void processCustomCommands() {
    static String buffer = "";
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (buffer.length() > 0) {
                handleDemoCommand(buffer);
                buffer = "";
            }
        } else {
            buffer += c;
        }
    }
}

/**
 * Handle demo-specific commands
 */
void handleDemoCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "DEMO") {
        runCalibrationDemo();
    } else if (cmd == "TEST") {
        testAngleRanges();
    } else if (cmd == "RESET") {
        gimbal.resetCalibration();
        gimbal.setAxisAngleRange(PITCH, 0.0f, 180.0f);
        gimbal.setAxisAngleRange(YAW, 0.0f, 180.0f);
        Serial.println("All settings reset to defaults!");
    }
}

/**
 * Automated calibration demonstration
 */
void runCalibrationDemo() {
    Serial.println("\n=== Starting Calibration Demo ===\n");
    
    // Step 1: Show default behavior
    Serial.println("Step 1: Default Range (0-180°)");
    gimbal.moveToPosition(45, 90);
    delay(2000);
    gimbal.moveToPosition(135, 90);
    delay(2000);
    
    // Step 2: Apply limited range
    Serial.println("\nStep 2: Limited Range (45-135°)");
    gimbal.setAxisAngleRange(PITCH, 45.0f, 135.0f);
    gimbal.setAxisAngleRange(YAW, 45.0f, 135.0f);
    
    gimbal.moveToPosition(30, 30);  // Will clamp to 45°
    delay(2000);
    gimbal.moveToPosition(160, 160);  // Will clamp to 135°
    delay(2000);
    
    // Step 3: Add offsets
    Serial.println("\nStep 3: Adding Offsets (+10° pitch, -5° yaw)");
    gimbal.setAlphaAngle(10.0f);
    gimbal.setBetaAngle(-5.0f);
    gimbal.home();
    delay(2000);
    
    // Step 4: Test compensated motion
    Serial.println("\nStep 4: Testing Compensated Motion");
    gimbal.moveToPosition(60, 60);
    delay(2000);
    gimbal.moveToPosition(120, 120);
    delay(2000);
    gimbal.home();
    delay(2000);
    
    // Print final status
    gimbal.printStatus();
    
    Serial.println("\n=== Demo Complete! ===\n");
}

/**
 * Test angle range limits
 */
void testAngleRanges() {
    Serial.println("\n=== Testing Angle Ranges ===\n");
    
    // Get current ranges
    float pitchMin = gimbal.getAxis(PITCH)->getMinAngle();
    float pitchMax = gimbal.getAxis(PITCH)->getMaxAngle();
    float yawMin = gimbal.getAxis(YAW)->getMinAngle();
    float yawMax = gimbal.getAxis(YAW)->getMaxAngle();
    
    Serial.printf("Pitch range: %.1f° to %.1f°\n", pitchMin, pitchMax);
    Serial.printf("Yaw range:   %.1f° to %.1f°\n\n", yawMin, yawMax);
    
    Serial.println("Testing minimum positions...");
    gimbal.moveToPosition(pitchMin, yawMin);
    delay(3000);
    
    Serial.println("Testing maximum positions...");
    gimbal.moveToPosition(pitchMax, yawMax);
    delay(3000);
    
    Serial.println("Testing center position...");
    gimbal.moveToPosition((pitchMin + pitchMax) / 2, (yawMin + yawMax) / 2);
    delay(2000);
    
    Serial.println("\n=== Range Test Complete! ===\n");
}

/**
 * Real-world calibration example for a camera gimbal
 */
void cameraGimbalCalibration() {
    Serial.println("\n=== Camera Gimbal Calibration ===\n");
    
    // Typical camera gimbal constraints:
    // - Can't tilt below 20° (hits camera body)
    // - Can't tilt above 160° (cable interference)
    // - Pan limited to 45° - 315° (270° total range)
    
    Serial.println("Setting camera gimbal constraints...");
    gimbal.setAxisAngleRange(PITCH, 20.0f, 160.0f);
    gimbal.setAxisAngleRange(YAW, 45.0f, 315.0f);
    
    // Servo mounting compensation
    // - Pitch servo mounted 5° forward
    // - Yaw servo mounted perfectly aligned
    gimbal.setAlphaAngle(5.0f);
    gimbal.setBetaAngle(0.0f);
    
    Serial.println("Camera gimbal configured!\n");
    
    // Test typical camera angles
    Serial.println("Testing camera angles:");
    
    Serial.println("  - Eye level");
    gimbal.moveToPosition(90, 180);
    delay(2000);
    
    Serial.println("  - High angle");
    gimbal.moveToPosition(45, 180);
    delay(2000);
    
    Serial.println("  - Low angle");
    gimbal.moveToPosition(135, 180);
    delay(2000);
    
    Serial.println("  - Pan left");
    gimbal.moveToPosition(90, 90);
    delay(2000);
    
    Serial.println("  - Pan right");
    gimbal.moveToPosition(90, 270);
    delay(2000);
    
    gimbal.printStatus();
}

/**
 * Advanced: Save/Load calibration to preferences (EEPROM)
 */
#include <Preferences.h>

void saveCalibration() {
    Preferences prefs;
    prefs.begin("gimbal", false);
    
    prefs.putFloat("alpha", gimbal.getAlphaAngle());
    prefs.putFloat("beta", gimbal.getBetaAngle());
    prefs.putFloat("p_min", gimbal.getAxis(PITCH)->getMinAngle());
    prefs.putFloat("p_max", gimbal.getAxis(PITCH)->getMaxAngle());
    prefs.putFloat("y_min", gimbal.getAxis(YAW)->getMinAngle());
    prefs.putFloat("y_max", gimbal.getAxis(YAW)->getMaxAngle());
    
    prefs.end();
    Serial.println("Calibration saved to flash!");
}

void loadCalibration() {
    Preferences prefs;
    prefs.begin("gimbal", true);
    
    if (prefs.isKey("alpha")) {
        float alpha = prefs.getFloat("alpha", 0.0f);
        float beta = prefs.getFloat("beta", 0.0f);
        float p_min = prefs.getFloat("p_min", 0.0f);
        float p_max = prefs.getFloat("p_max", 180.0f);
        float y_min = prefs.getFloat("y_min", 0.0f);
        float y_max = prefs.getFloat("y_max", 180.0f);
        
        gimbal.setAlphaAngle(alpha);
        gimbal.setBetaAngle(beta);
        gimbal.setAxisAngleRange(PITCH, p_min, p_max);
        gimbal.setAxisAngleRange(YAW, y_min, y_max);
        
        Serial.println("Calibration loaded from flash!");
    } else {
        Serial.println("No saved calibration found.");
    }
    
    prefs.end();
}
