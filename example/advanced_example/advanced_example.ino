/**
 * @file AdvancedIntegration.ino
 * @brief Advanced gimbal control with WiFi, WebSocket, and HTTP interface
 * 
 * Features:
 * - WiFi AP mode for wireless control
 * - WebSocket real-time control
 * - HTTP REST API
 * - Web interface for browser control
 * - Preset positions
 * - Recording sequences
 */

#include "GimbalController.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ============================================================================
// Configuration
// ============================================================================

// WiFi Access Point settings
const char* AP_SSID = "ESP32_Gimbal";
const char* AP_PASS = "gimbal123";

// Servo pins
#define SERVO_PITCH_PIN  1
#define SERVO_YAW_PIN    2

// ============================================================================
// Global Objects
// ============================================================================

GimbalController gimbal(SERVO_PITCH_PIN, SERVO_YAW_PIN, 100);
WebServer server(80);
WebSocketsServer webSocket(81);

// ============================================================================
// Preset Positions
// ============================================================================

struct Position {
    float pitch;
    float yaw;
    const char* name;
};

Position presets[] = {
    {90, 90, "Center"},
    {45, 90, "Up"},
    {135, 90, "Down"},
    {90, 45, "Left"},
    {90, 135, "Right"},
    {60, 60, "Up-Left"},
    {60, 120, "Up-Right"},
    {120, 60, "Down-Left"},
    {120, 120, "Down-Right"}
};

const int numPresets = sizeof(presets) / sizeof(presets[0]);

// ============================================================================
// Recording Sequence
// ============================================================================

struct SequenceStep {
    float pitch;
    float yaw;
    unsigned long duration;  // milliseconds
};

SequenceStep sequence[20];
int sequenceLength = 0;
bool recordingMode = false;
bool playingSequence = false;
int currentStep = 0;
unsigned long stepStartTime = 0;

// ============================================================================
// HTML Web Interface
// ============================================================================

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32 Gimbal Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: Arial, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; margin-bottom: 30px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
        .card {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
        }
        .joystick {
            width: 300px;
            height: 300px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 50%;
            margin: 20px auto;
            position: relative;
            touch-action: none;
        }
        .stick {
            width: 80px;
            height: 80px;
            background: white;
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
        }
        .slider-container {
            margin: 15px 0;
        }
        input[type="range"] {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.2);
            outline: none;
        }
        button {
            background: rgba(255, 255, 255, 0.2);
            border: 2px solid white;
            color: white;
            padding: 12px 24px;
            border-radius: 8px;
            cursor: pointer;
            margin: 5px;
            font-size: 16px;
            transition: all 0.3s;
        }
        button:hover {
            background: rgba(255, 255, 255, 0.3);
            transform: scale(1.05);
        }
        .preset-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        .status {
            font-size: 14px;
            padding: 10px;
            background: rgba(0, 0, 0, 0.3);
            border-radius: 8px;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 ESP32 Gimbal Control</h1>
        
        <div class="card">
            <h2>Virtual Joystick</h2>
            <div class="joystick" id="joystick">
                <div class="stick" id="stick"></div>
            </div>
            <div class="status" id="position">Pitch: 90° | Yaw: 90°</div>
        </div>
        
        <div class="card">
            <h2>Manual Control</h2>
            <div class="slider-container">
                <label>Pitch (Tilt): <span id="pitchVal">90</span>°</label>
                <input type="range" id="pitchSlider" min="0" max="180" value="90">
            </div>
            <div class="slider-container">
                <label>Yaw (Pan): <span id="yawVal">90</span>°</label>
                <input type="range" id="yawSlider" min="0" max="180" value="90">
            </div>
            <button onclick="sendPosition()">Apply Position</button>
        </div>
        
        <div class="card">
            <h2>Preset Positions</h2>
            <div class="preset-grid" id="presets"></div>
        </div>
        
        <div class="card">
            <h2>Quick Actions</h2>
            <button onclick="sendCommand('home')">🏠 Home</button>
            <button onclick="sendCommand('stop')">🛑 Stop</button>
            <button onclick="sendCommand('status')">📊 Status</button>
        </div>
    </div>
    
    <script>
        let ws = new WebSocket('ws://' + location.hostname + ':81');
        let isDragging = false;
        
        ws.onopen = () => console.log('Connected to gimbal');
        ws.onmessage = (event) => {
            let data = JSON.parse(event.data);
            document.getElementById('position').innerHTML = 
                'Pitch: ' + data.pitch.toFixed(1) + '° | Yaw: ' + data.yaw.toFixed(1) + '°';
        };
        
        // Joystick control
        const joystick = document.getElementById('joystick');
        const stick = document.getElementById('stick');
        
        joystick.addEventListener('mousedown', startDrag);
        joystick.addEventListener('mousemove', drag);
        joystick.addEventListener('mouseup', endDrag);
        joystick.addEventListener('touchstart', startDrag);
        joystick.addEventListener('touchmove', drag);
        joystick.addEventListener('touchend', endDrag);
        
        function startDrag(e) {
            isDragging = true;
        }
        
        function drag(e) {
            if (!isDragging) return;
            e.preventDefault();
            
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            let x = (e.clientX || e.touches[0].clientX) - rect.left - centerX;
            let y = (e.clientY || e.touches[0].clientY) - rect.top - centerY;
            
            const distance = Math.sqrt(x*x + y*y);
            const maxDistance = rect.width / 2 - 40;
            
            if (distance > maxDistance) {
                x = x / distance * maxDistance;
                y = y / distance * maxDistance;
            }
            
            stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            
            // Convert to gimbal angles
            const pitch = 90 - (y / maxDistance) * 90;
            const yaw = 90 + (x / maxDistance) * 90;
            
            ws.send(JSON.stringify({pitch: pitch, yaw: yaw}));
        }
        
        function endDrag() {
            isDragging = false;
            stick.style.transform = 'translate(-50%, -50%)';
        }
        
        // Slider control
        document.getElementById('pitchSlider').oninput = function() {
            document.getElementById('pitchVal').textContent = this.value;
        };
        
        document.getElementById('yawSlider').oninput = function() {
            document.getElementById('yawVal').textContent = this.value;
        };
        
        function sendPosition() {
            const pitch = document.getElementById('pitchSlider').value;
            const yaw = document.getElementById('yawSlider').value;
            ws.send(JSON.stringify({pitch: parseFloat(pitch), yaw: parseFloat(yaw)}));
        }
        
        function sendCommand(cmd) {
            fetch('/' + cmd).then(r => r.text()).then(console.log);
        }
        
        // Load presets
        fetch('/presets').then(r => r.json()).then(data => {
            const grid = document.getElementById('presets');
            data.forEach((p, i) => {
                const btn = document.createElement('button');
                btn.textContent = p.name;
                btn.onclick = () => {
                    ws.send(JSON.stringify({pitch: p.pitch, yaw: p.yaw}));
                };
                grid.appendChild(btn);
            });
        });
    </script>
</body>
</html>
)rawliteral";

// ============================================================================
// WebSocket Handler
// ============================================================================

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, payload);
        
        float pitch = doc["pitch"];
        float yaw = doc["yaw"];
        
        gimbal.moveToPosition(pitch, yaw);
    }
}

// ============================================================================
// HTTP Handlers
// ============================================================================

void handleRoot() {
    server.send_P(200, "text/html", HTML_PAGE);
}

void handlePresets() {
    StaticJsonDocument<1024> doc;
    JsonArray array = doc.to<JsonArray>();
    
    for (int i = 0; i < numPresets; i++) {
        JsonObject obj = array.createNestedObject();
        obj["name"] = presets[i].name;
        obj["pitch"] = presets[i].pitch;
        obj["yaw"] = presets[i].yaw;
    }
    
    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

void handleHome() {
    gimbal.home();
    server.send(200, "text/plain", "Homing...");
}

void handleStop() {
    gimbal.stopAll();
    server.send(200, "text/plain", "Stopped");
}

void handleStatus() {
    StaticJsonDocument<200> doc;
    doc["pitch"] = gimbal.getAxis(PITCH)->getPosition();
    doc["yaw"] = gimbal.getAxis(YAW)->getPosition();
    doc["moving"] = gimbal.isMoving();
    
    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ESP32-S3 Gimbal - Advanced Integration ===\n");
    
    // Initialize gimbal
    gimbal.begin();
    gimbal.home();
    
    // Setup WiFi AP
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    
    // Setup HTTP server
    server.on("/", handleRoot);
    server.on("/presets", handlePresets);
    server.on("/home", handleHome);
    server.on("/stop", handleStop);
    server.on("/status", handleStatus);
    server.begin();
    
    // Setup WebSocket
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    Serial.println("\n=== System Ready ===");
    Serial.printf("Web Interface: http://%s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("WebSocket: ws://%s:81\n\n", WiFi.softAPIP().toString().c_str());
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    // Update gimbal (CRITICAL - must be called every loop)
    gimbal.update();
    
    // Handle connections
    server.handleClient();
    webSocket.loop();
    
    // Process serial commands
    gimbal.processSerial();
    
    // Send status via WebSocket periodically
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 100) {
        lastStatusUpdate = millis();
        
        StaticJsonDocument<200> doc;
        doc["pitch"] = gimbal.getAxis(PITCH)->getPosition();
        doc["yaw"] = gimbal.getAxis(YAW)->getPosition();
        
        String output;
        serializeJson(doc, output);
        webSocket.broadcastTXT(output);
    }
}
