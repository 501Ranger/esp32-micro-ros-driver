#pragma once

#include <Arduino.h>
#include <Preferences.h>

namespace robot {

struct NetworkConfig {
  String ssid;
  String password;
  String agent_ip;
  uint16_t agent_port = 8888;
  bool has_wifi = false;
  bool loaded_from_flash = false;
};

class WifiConfigManager {
 public:
  void begin();
  NetworkConfig load() const;
  bool save(const NetworkConfig &config);
  bool clear();

 private:
  mutable Preferences preferences_;
};

}  // namespace robot
