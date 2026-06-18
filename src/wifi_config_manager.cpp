#include "wifi_config_manager.h"
#include <ArduinoJson.h>

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
  config.vofa_debug = preferences_.getBool("vofa_dbg", false);
  config.enable_buzzer = preferences_.getBool("buzzer", true);
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
  ok = preferences_.putBool("vofa_dbg", config.vofa_debug) && ok;
  ok = preferences_.putBool("buzzer", config.enable_buzzer) && ok;
  return ok;
}

bool WifiConfigManager::clear() {
  bool ok = preferences_.clear();
  Preferences prefs;
  if (prefs.begin("nethist", false)) {
    ok = prefs.clear() && ok;
    prefs.end();
  }
  return ok;
}

std::vector<NetworkConfig> WifiConfigManager::getHistory() const {
  std::vector<NetworkConfig> history;
  Preferences prefs;
  if (prefs.begin("nethist", true)) {
    String json_str = prefs.getString("history", "[]");
    prefs.end();

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, json_str);
    if (!error && doc.is<JsonArray>()) {
      JsonArray arr = doc.as<JsonArray>();
      for (JsonVariant val : arr) {
        NetworkConfig item;
        item.ssid = val["ssid"] | "";
        item.password = val["pass"] | "";
        item.agent_ip = val["ip"] | "";
        item.agent_port = val["port"] | 8888;
        item.enable_buzzer = val["buzzer"] | true;
        item.has_wifi = item.ssid.length() > 0;
        item.loaded_from_flash = true;
        history.push_back(item);
      }
    }
  }
  return history;
}

bool WifiConfigManager::saveHistory(const std::vector<NetworkConfig> &history) {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto &item : history) {
    JsonObject obj = arr.createNestedObject();
    obj["ssid"] = item.ssid;
    obj["pass"] = item.password;
    obj["ip"] = item.agent_ip;
    obj["port"] = item.agent_port;
    obj["buzzer"] = item.enable_buzzer;
  }

  String json_str;
  serializeJson(doc, json_str);

  Preferences prefs;
  if (prefs.begin("nethist", false)) {
    size_t written = prefs.putString("history", json_str);
    prefs.end();
    return written > 0;
  }
  return false;
}

bool WifiConfigManager::addToHistory(const NetworkConfig &config) {
  if (config.ssid.length() == 0) return false;

  std::vector<NetworkConfig> history = getHistory();

  // Remove existing entries with the same SSID to avoid duplicates
  for (auto it = history.begin(); it != history.end(); ) {
    if (it->ssid == config.ssid) {
      it = history.erase(it);
    } else {
      ++it;
    }
  }

  // Insert at the front
  history.insert(history.begin(), config);

  // Keep max 5 items
  if (history.size() > 5) {
    history.resize(5);
  }

  return saveHistory(history);
}

bool WifiConfigManager::deleteFromHistory(size_t index) {
  std::vector<NetworkConfig> history = getHistory();
  if (index >= history.size()) {
    return false;
  }
  history.erase(history.begin() + index);
  return saveHistory(history);
}

}  // namespace robot
