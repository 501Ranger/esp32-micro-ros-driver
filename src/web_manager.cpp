#include "web_manager.h"
#include "robot_config.h"
#include <WiFi.h>
#include "web_pages.h"

namespace robot {

using namespace config;

WebManager::WebManager() : server_(80), ws_("/ws") {}

WebManager::~WebManager() {}

void WebManager::begin(WifiConfigManager &wifi_config_manager) {
  wifi_config_manager_ = &wifi_config_manager;
  pinMode(GAMEPAD_LED_PIN, OUTPUT);
  digitalWrite(GAMEPAD_LED_PIN, HIGH);
  ledcSetup(BUZZER_PWM_CHANNEL, 1000, 10);
  ledcAttachPin(GAMEPAD_BUZZER_PIN, BUZZER_PWM_CHANNEL);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);

  ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    this->onWebSocketEvent(server, client, type, arg, data, len);
  });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server_.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", WIFI_HTML);
  });
  server_.on("/api/wifi/status", HTTP_GET, [this](AsyncWebServerRequest *request){
    this->sendWifiStatus(request);
  });
  server_.on("/api/wifi/clear", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiClear(request);
  });
  server_.on("/api/wifi/history", HTTP_GET, [this](AsyncWebServerRequest *request){
    this->handleWifiHistoryGet(request);
  });
  server_.on("/api/wifi/history/delete", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiHistoryDelete(request);
  });
  server_.on("/api/wifi", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiSave(request);
  });
  server_.on("/api/imu/calibrate", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleImuCalibrate(request);
  });

  server_.begin();
#ifndef USE_SERIAL_TRANSPORT
  Serial.println("Web Server Started on port 80");
#endif
}

void WebManager::loop() {
  ws_.cleanupClients();
  if (restart_at_ms_ != 0 && millis() >= restart_at_ms_) {
    ESP.restart();
  }

  // Dynamic MDNS & NetBIOS (NBNS) setup when IP is ready
  static bool names_started = false;
  bool has_ip = (WiFi.status() == WL_CONNECTED) || 
                ((WiFi.getMode() & WIFI_AP) && WiFi.softAPIP() != IPAddress(0, 0, 0, 0));
  
  if (has_ip) {
    if (!names_started) {
      // 1. Initialize/Restart MDNS
      if (MDNS.begin("esp32robot")) {
        MDNS.addService("http", "tcp", 80);
#ifndef USE_SERIAL_TRANSPORT
        Serial.print("mDNS responder started: http://esp32robot.local/ (IP: ");
        Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
        Serial.println(")");
#endif
      }
      
      // 2. Initialize NetBIOS (NBNS) - fallback for Windows
      NBNS.begin("esp32robot");
#ifndef USE_SERIAL_TRANSPORT
      Serial.println("NetBIOS responder started: http://esp32robot/");
#endif
      
      names_started = true;
    }
  } else {
    if (names_started) {
      MDNS.end();
      NBNS.end();
      names_started = false;
    }
  }
  
  // Timeout protection: if no commands received for a while, stop the robot
  if (is_active_ && millis() - last_command_ms_ > COMMAND_TIMEOUT_MS) {
    linear_out_ = 0.0f;
    angular_out_ = 0.0f;
    is_active_ = false;
#ifndef USE_SERIAL_TRANSPORT
    Serial.println("Web Joystick Timeout - Stopping");
#endif
  }
  
  // Broadcast system status to connected WebSocket clients every 1s
  if (connected_clients_ > 0 && millis() - last_status_broadcast_ms_ >= 1000) {
    last_status_broadcast_ms_ = millis();
    StaticJsonDocument<128> doc;
    doc["battery_v"] = status_.battery_voltage;
    doc["battery_p"] = status_.battery_percentage;
    
    const char* state_str = "WaitingAgent";
    switch (status_.agent_state) {
      case AgentState::WaitingAgent: state_str = "WaitingAgent"; break;
      case AgentState::AgentAvailable: state_str = "AgentAvailable"; break;
      case AgentState::AgentConnected: state_str = "AgentConnected"; break;
      case AgentState::AgentDisconnected: state_str = "AgentDisconnected"; break;
    }
    doc["agent_state"] = state_str;
    doc["wifi_rssi"] = status_.wifi_rssi;
    doc["vofa_debug"] = vofa_debug_enabled_;

    char output[128];
    serializeJson(doc, output, sizeof(output));
    ws_.textAll(output);
  }
  
  updateFeedback();
}

bool WebManager::getVelocity(float &linear_mps, float &angular_radps) const {
  linear_mps = linear_out_;
  angular_radps = angular_out_;
  return is_active_;
}

void WebManager::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                                  AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
#ifndef USE_SERIAL_TRANSPORT
    Serial.printf("WS Client Connected: %u\n", client->id());
#endif
    connected_clients_++;
    if (connected_clients_ == 1) {
      playConnectSound();
    }
  } else if (type == WS_EVT_DISCONNECT) {
#ifndef USE_SERIAL_TRANSPORT
    Serial.printf("WS Client Disconnected: %u\n", client->id());
#endif
    if (connected_clients_ > 0) connected_clients_--;
    if (connected_clients_ == 0) {
      linear_out_ = 0.0f;
      angular_out_ = 0.0f;
      is_active_ = false;
    }
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (!error) {
      if (doc.containsKey("x") && doc.containsKey("y")) {
        float x = doc["x"];
        float y = doc["y"];
        
        linear_out_ = y * 1.0f;
        angular_out_ = -x * 1.5f;
        
        // Update state
        is_active_ = (fabs(linear_out_) > 0.01f || fabs(angular_out_) > 0.01f);
        last_command_ms_ = millis();
      }
      if (doc.containsKey("vofa_debug")) {
        vofa_debug_enabled_ = doc["vofa_debug"];
      }
    } else {
#ifndef USE_SERIAL_TRANSPORT
      Serial.print("JSON Parse Error: ");
      Serial.println(error.c_str());
#endif
    }
  }
}

void WebManager::handleWifiSave(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }
  if (!request->hasParam("ssid", true) ||
      !request->hasParam("agent_ip", true) ||
      !request->hasParam("agent_port", true)) {
    request->send(400, "text/plain", "Missing ssid, agent_ip, or agent_port");
    return;
  }

  NetworkConfig config;
  config.ssid = request->getParam("ssid", true)->value();
  config.ssid.trim();
  config.password = request->hasParam("password", true) ?
      request->getParam("password", true)->value() : "";
  config.agent_ip = request->getParam("agent_ip", true)->value();
  config.agent_ip.trim();
  long requested_port = request->getParam("agent_port", true)->value().toInt();
  if (requested_port < 1) {
    requested_port = 1;
  } else if (requested_port > 65535) {
    requested_port = 65535;
  }
  config.agent_port = static_cast<uint16_t>(requested_port);
  config.has_wifi = config.ssid.length() > 0;
  config.vofa_debug = request->hasParam("vofa_debug", true);

  IPAddress parsed_agent_ip;
  bool is_valid_ip = parsed_agent_ip.fromString(config.agent_ip);
  bool is_valid_hostname = false;
  
  if (!is_valid_ip && config.agent_ip.length() > 0) {
    is_valid_hostname = true;
    for (size_t i = 0; i < config.agent_ip.length(); ++i) {
      char c = config.agent_ip[i];
      if (!isalnum(c) && c != '.' && c != '-') {
        is_valid_hostname = false;
        break;
      }
    }
  }

  if (!config.has_wifi || (!is_valid_ip && !is_valid_hostname)) {
    request->send(400, "text/plain", "Invalid WiFi SSID or Agent IP/Hostname");
    return;
  }

  if (!wifi_config_manager_->save(config)) {
    request->send(500, "text/plain", "Failed to save WiFi config");
    return;
  }

  wifi_config_manager_->addToHistory(config);

  restart_at_ms_ = millis() + 1200;
  request->send(200, "text/plain", "Saved. The robot will restart and connect with the new settings.");
}

void WebManager::handleWifiClear(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  if (!wifi_config_manager_->clear()) {
    request->send(500, "text/plain", "Failed to clear WiFi config");
    return;
  }

  restart_at_ms_ = millis() + 1200;
  request->send(200, "text/plain", "Cleared. The robot will restart and use the built-in fallback settings.");
}

void WebManager::handleWifiHistoryGet(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  std::vector<NetworkConfig> history = wifi_config_manager_->getHistory();

  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto &item : history) {
    JsonObject obj = arr.createNestedObject();
    obj["ssid"] = item.ssid;
    obj["password"] = item.password;
    obj["agent_ip"] = item.agent_ip;
    obj["agent_port"] = item.agent_port;
  }

  char output[1024];
  serializeJson(doc, output, sizeof(output));
  request->send(200, "application/json", output);
}

void WebManager::handleWifiHistoryDelete(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  if (!request->hasParam("index", true)) {
    request->send(400, "text/plain", "Missing index parameter");
    return;
  }

  int index = request->getParam("index", true)->value().toInt();
  if (index < 0) {
    request->send(400, "text/plain", "Invalid index");
    return;
  }

  if (!wifi_config_manager_->deleteFromHistory(static_cast<size_t>(index))) {
    request->send(500, "text/plain", "Failed to delete history item");
    return;
  }

  request->send(200, "text/plain", "Deleted successfully");
}

void WebManager::sendWifiStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<384> doc;
  const NetworkConfig config = wifi_config_manager_ != nullptr ? wifi_config_manager_->load() : NetworkConfig();

  doc["ssid"] = config.ssid;
  doc["agent_ip"] = config.agent_ip;
  doc["agent_port"] = config.agent_port;
  doc["from_flash"] = config.loaded_from_flash;
  doc["connected"] = WiFi.status() == WL_CONNECTED;
  doc["local_ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  doc["ap_ip"] = WiFi.softAPIP().toString();
  doc["vofa_debug"] = config.vofa_debug;

  char output[384];
  serializeJson(doc, output, sizeof(output));
  request->send(200, "application/json", output);
}

void WebManager::updateFeedback() {
  if (connected_clients_ == 0) {
    digitalWrite(GAMEPAD_LED_PIN, HIGH); // Off
  } else {
    digitalWrite(GAMEPAD_LED_PIN, LOW); // On
  }

  if (current_tone_idx_ != -1) {
    if (millis() - tone_start_ms_ > connect_sequence_[current_tone_idx_].duration_ms) {
      current_tone_idx_++;
      if (current_tone_idx_ >= 3) {
        current_tone_idx_ = -1;
        ledcWrite(BUZZER_PWM_CHANNEL, 0);
      } else {
        ledcWriteTone(BUZZER_PWM_CHANNEL, connect_sequence_[current_tone_idx_].frequency);
        ledcWrite(BUZZER_PWM_CHANNEL, 512); // 50% duty cycle
        tone_start_ms_ = millis();
      }
    }
  }
}

void WebManager::playConnectSound() {
  current_tone_idx_ = 0;
  tone_start_ms_ = millis();
  ledcWriteTone(BUZZER_PWM_CHANNEL, connect_sequence_[0].frequency);
  ledcWrite(BUZZER_PWM_CHANNEL, 512);
}

void WebManager::setBatteryStatus(float voltage, int percentage) {
  battery_voltage_ = voltage;
  battery_percentage_ = percentage;
  status_.battery_voltage = voltage;
  status_.battery_percentage = percentage;
}

void WebManager::handleImuCalibrate(AsyncWebServerRequest *request) {
  imu_calibrate_requested_ = true;
  request->send(200, "text/plain", "IMU校准指令已下发，请保持机器人静止！");
}

}  // namespace robot
