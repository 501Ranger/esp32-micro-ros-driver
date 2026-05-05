#include "web_manager.h"
#include <WiFi.h>

namespace robot {

const char* INDEX_HTML PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32 Web Joystick</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      background-color: #111;
      color: #fff;
      font-family: sans-serif;
      overflow: hidden;
      touch-action: none; /* Prevent zooming/scrolling */
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
    }
    #status {
      position: absolute;
      top: 10px;
      left: 10px;
      font-size: 14px;
      padding: 5px 10px;
      border-radius: 5px;
      background: rgba(255, 255, 255, 0.1);
    }
    #joystick-zone {
      position: relative;
      width: 250px;
      height: 250px;
      background: rgba(255, 255, 255, 0.05);
      border: 2px solid rgba(255, 255, 255, 0.2);
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: inset 0 0 20px rgba(0,0,0,0.5);
    }
    #stick {
      position: absolute;
      width: 80px;
      height: 80px;
      background: radial-gradient(circle at 30% 30%, #555, #222);
      border-radius: 50%;
      box-shadow: 0 5px 15px rgba(0,0,0,0.5);
      pointer-events: none; /* Let events fall through to the zone */
      transition: transform 0.1s ease-out; /* Smooth return to center */
    }
    #stick.active {
      transition: none; /* Instant follow when dragging */
    }
  </style>
</head>
<body>
  <div id="status">Disconnected</div>
  <div id="joystick-zone">
    <div id="stick"></div>
  </div>

  <script>
    let ws;
    const statusEl = document.getElementById('status');
    const zone = document.getElementById('joystick-zone');
    const stick = document.getElementById('stick');
    
    let isDragging = false;
    let centerX, centerY;
    const maxRadius = 125 - 40; // Zone radius - Stick radius
    
    // Output values -1.0 to 1.0
    let outX = 0;
    let outY = 0;

    function connect() {
      // Connect to the ESP32's WebSocket
      ws = new WebSocket(`ws://${window.location.host}/ws`);
      
      ws.onopen = () => {
        statusEl.innerText = "Connected";
        statusEl.style.color = "#0f0";
      };
      
      ws.onclose = () => {
        statusEl.innerText = "Disconnected, retrying...";
        statusEl.style.color = "#f00";
        setTimeout(connect, 1000);
      };
    }

    function sendCommand() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        // Only send if active or recently returned to center to save bandwidth
        if (isDragging || (outX === 0 && outY === 0)) {
           ws.send(JSON.stringify({ x: outX, y: outY }));
        }
      }
    }

    // Send commands at 20Hz (50ms)
    setInterval(sendCommand, 50);

    function updateStickPosition(clientX, clientY) {
      let dx = clientX - centerX;
      let dy = clientY - centerY;
      const distance = Math.sqrt(dx * dx + dy * dy);
      
      if (distance > maxRadius) {
        const ratio = maxRadius / distance;
        dx *= ratio;
        dy *= ratio;
      }
      
      stick.style.transform = `translate(${dx}px, ${dy}px)`;
      
      // Normalize to -1.0 to 1.0 (Invert Y so Up is positive)
      outX = dx / maxRadius;
      outY = -(dy / maxRadius);
    }

    function resetStick() {
      isDragging = false;
      stick.classList.remove('active');
      stick.style.transform = 'translate(0px, 0px)';
      outX = 0;
      outY = 0;
      // Send one final zero command immediately
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ x: 0, y: 0 }));
      }
    }

    // Touch Events
    zone.addEventListener('touchstart', (e) => {
      e.preventDefault();
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });

    zone.addEventListener('touchmove', (e) => {
      e.preventDefault();
      if (!isDragging) return;
      updateStickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });

    zone.addEventListener('touchend', (e) => {
      e.preventDefault();
      resetStick();
    });
    
    zone.addEventListener('touchcancel', (e) => {
      e.preventDefault();
      resetStick();
    });

    // Mouse Events for testing on PC
    zone.addEventListener('mousedown', (e) => {
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.clientX, e.clientY);
    });

    document.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      updateStickPosition(e.clientX, e.clientY);
    });

    document.addEventListener('mouseup', () => {
      if (isDragging) resetStick();
    });

    // Start WebSocket
    connect();
  </script>
</body>
</html>
)=====";

WebManager::WebManager() : server_(80), ws_("/ws") {}

WebManager::~WebManager() {}

void WebManager::begin() {
  ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    this->onWebSocketEvent(server, client, type, arg, data, len);
  });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server_.begin();
#ifndef USE_SERIAL_TRANSPORT
  Serial.println("Web Server Started on port 80");
#endif
}

void WebManager::loop() {
  ws_.cleanupClients();
  
  // Timeout protection: if no commands received for a while, stop the robot
  if (is_active_ && millis() - last_command_ms_ > COMMAND_TIMEOUT_MS) {
    linear_out_ = 0.0f;
    angular_out_ = 0.0f;
    is_active_ = false;
    Serial.println("Web Joystick Timeout - Stopping");
  }
}

bool WebManager::getVelocity(float &linear_mps, float &angular_radps) const {
  linear_mps = linear_out_;
  angular_radps = angular_out_;
  return is_active_;
}

void WebManager::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                                  AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS Client Connected: %u\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS Client Disconnected: %u\n", client->id());
    linear_out_ = 0.0f;
    angular_out_ = 0.0f;
    is_active_ = false;
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0; // Null-terminate just in case
    
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, (char*)data);
    
    if (!error) {
      if (doc.containsKey("x") && doc.containsKey("y")) {
        float x = doc["x"];
        float y = doc["y"];
        
        // Map stick values to physical velocities
        // Max linear speed 1.0 m/s (forward/backward = y)
        // Max angular speed 1.5 rad/s (left/right = x, invert x for standard Z axis rotation)
        linear_out_ = y * 1.0f;
        angular_out_ = -x * 1.5f;
        
        // Update state
        is_active_ = (fabs(linear_out_) > 0.01f || fabs(angular_out_) > 0.01f);
        last_command_ms_ = millis();
      }
    } else {
#ifndef USE_SERIAL_TRANSPORT
      Serial.print("JSON Parse Error: ");
      Serial.println(error.c_str());
#endif
    }
  }
}

}  // namespace robot