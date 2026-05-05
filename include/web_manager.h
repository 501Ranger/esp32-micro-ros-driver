#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

namespace robot {

class WebManager {
 public:
  WebManager();
  ~WebManager();

  void begin();
  void loop();

  // Get the current velocity commands from the web joystick
  // Returns true if active commands are present (non-zero or recently zeroed), false if idle
  bool getVelocity(float &linear_mps, float &angular_radps) const;

 private:
  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                        AwsEventType type, void *arg, uint8_t *data, size_t len);
  void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

  AsyncWebServer server_;
  AsyncWebSocket ws_;

  float linear_out_ = 0.0f;
  float angular_out_ = 0.0f;
  unsigned long last_command_ms_ = 0;
  bool is_active_ = false;

  const uint32_t COMMAND_TIMEOUT_MS = 500;
};

}  // namespace robot
