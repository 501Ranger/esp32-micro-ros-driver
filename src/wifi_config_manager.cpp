#include "wifi_config_manager.h"

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef AGENT_IP
#define AGENT_IP "192.168.1.1"
#endif

#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif

namespace robot {

namespace {

constexpr char CONFIG_NAMESPACE[] = "netcfg";
constexpr char KEY_SSID[] = "ssid";
constexpr char KEY_PASSWORD[] = "pass";
constexpr char KEY_AGENT_IP[] = "agent_ip";
constexpr char KEY_AGENT_PORT[] = "agent_port";

}  // namespace

void WifiConfigManager::begin() {
  preferences_.begin(CONFIG_NAMESPACE, false);
}

NetworkConfig WifiConfigManager::load() const {
  NetworkConfig config;
  config.ssid = preferences_.getString(KEY_SSID, "");
  config.password = preferences_.getString(KEY_PASSWORD, "");
  config.agent_ip = preferences_.getString(KEY_AGENT_IP, "");
  config.agent_port = preferences_.getUShort(KEY_AGENT_PORT, AGENT_PORT);
  config.loaded_from_flash = config.ssid.length() > 0;

  if (config.ssid.length() == 0) {
    config.ssid = WIFI_SSID;
    config.password = WIFI_PASSWORD;
  }
  if (config.agent_ip.length() == 0) {
    config.agent_ip = AGENT_IP;
  }
  if (config.agent_port == 0) {
    config.agent_port = AGENT_PORT;
  }

  config.has_wifi = config.ssid.length() > 0;
  return config;
}

bool WifiConfigManager::save(const NetworkConfig &config) {
  if (config.ssid.length() == 0 || config.agent_ip.length() == 0 || config.agent_port == 0) {
    return false;
  }

  bool ok = preferences_.putString(KEY_SSID, config.ssid) > 0;
  preferences_.putString(KEY_PASSWORD, config.password);
  ok = (preferences_.putString(KEY_AGENT_IP, config.agent_ip) > 0) && ok;
  ok = (preferences_.putUShort(KEY_AGENT_PORT, config.agent_port) > 0) && ok;
  return ok;
}

bool WifiConfigManager::clear() {
  return preferences_.clear();
}

}  // namespace robot
