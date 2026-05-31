#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <vector>

namespace robot {

struct NetworkConfig {
  String ssid;
  String password;
  String agent_ip;
  uint16_t agent_port = 8888;
  bool has_wifi = false;
  bool loaded_from_flash = false;
  bool vofa_debug = false;
};

class WifiConfigManager {
 public:
  void begin();
  NetworkConfig load() const;
  bool save(const NetworkConfig &config);
  bool clear();

  // History config management
  std::vector<NetworkConfig> getHistory() const;
  bool saveHistory(const std::vector<NetworkConfig> &history);
  bool addToHistory(const NetworkConfig &config);
  bool deleteFromHistory(size_t index);

 private:
  mutable Preferences preferences_;
};

}  // namespace robot
