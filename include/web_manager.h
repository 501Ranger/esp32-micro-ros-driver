#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

namespace robot {

struct BuzzerTone {
  uint32_t frequency;
  uint32_t duration_ms;
};

class WebManager {
 public:
  WebManager();
  ~WebManager();

  void begin();
  void loop();

  // Get the current velocity commands from the web joystick
  bool getVelocity(float &linear_mps, float &angular_radps) const;

 private:
  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                        AwsEventType type, void *arg, uint8_t *data, size_t len);
  void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
  void playConnectSound();
  void updateFeedback();

  AsyncWebServer server_;
  AsyncWebSocket ws_;

  float linear_out_ = 0.0f;
  float angular_out_ = 0.0f;
  unsigned long last_command_ms_ = 0;
  bool is_active_ = false;
  uint8_t connected_clients_ = 0;

  // Buzzer sequence
  const BuzzerTone connect_sequence_[3] = {
    {1000, 100},
    {1500, 100},
    {2000, 150}
  };
  int current_tone_idx_ = -1;
  unsigned long tone_start_ms_ = 0;

  const uint32_t COMMAND_TIMEOUT_MS = 500;
};

}  // namespace robot
