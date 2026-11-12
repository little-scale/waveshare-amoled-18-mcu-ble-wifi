#include "ConfigStore.h"

namespace {
constexpr char kNamespace[] = "classroom";
}

bool ConfigStore::begin() {
  return preferences_.begin(kNamespace, false);
}

void ConfigStore::load(DeviceSettings &settings) {
  settings.deviceName = preferences_.getString("name", settings.deviceName);
  settings.wifiSsid = preferences_.getString("ssid", settings.wifiSsid);
  settings.wifiPassword = preferences_.getString("pass", settings.wifiPassword);
  settings.oscTarget = preferences_.getString("osc_host", settings.oscTarget);
  settings.oscSendPort = preferences_.getUShort("osc_tx", settings.oscSendPort);
  settings.oscReceivePort = preferences_.getUShort("osc_rx", settings.oscReceivePort);

  settings.dimAfterMs = preferences_.getULong("dim_ms", settings.dimAfterMs);

  settings.imuRateHz = preferences_.getUChar("motion_hz",100);
  if (settings.imuRateHz!=50 && settings.imuRateHz!=100) settings.imuRateHz=100;
  settings.continuousStream=preferences_.getBool("motion_cont",false);
  const uint8_t defaultTransport=static_cast<uint8_t>(TransportMode::Ble);
  const uint8_t transport=preferences_.getUChar("motion_link",defaultTransport);
  settings.transport=static_cast<TransportMode>(transport<=2?transport:defaultTransport);
  settings.bleRateHz=preferences_.getUChar("motion_ble_hz",50)==100?100:50;
  for(unsigned i=0;i<3;++i) {
    const char *keys[]={"motion_bx","motion_by","motion_bz"};
    settings.gyroBias[i]=preferences_.getFloat(keys[i],0);
  }
  settings.oscSendPort = constrain(settings.oscSendPort, 1, 65535);
  settings.oscReceivePort = constrain(settings.oscReceivePort, 1, 65535);
  settings.deviceName.trim();
  if (settings.deviceName.isEmpty()) settings.deviceName = "device-0000";
}

bool ConfigStore::save(const DeviceSettings &settings) {
  bool ok = true;
  ok &= preferences_.putString("name", settings.deviceName) > 0;
  preferences_.putString("ssid", settings.wifiSsid);
  preferences_.putString("pass", settings.wifiPassword);
  ok &= preferences_.putString("osc_host", settings.oscTarget) > 0;
  ok &= preferences_.putUShort("osc_tx", settings.oscSendPort) == sizeof(uint16_t);
  ok &= preferences_.putUShort("osc_rx", settings.oscReceivePort) == sizeof(uint16_t);
  ok &= preferences_.putUChar("motion_hz", settings.imuRateHz)==sizeof(uint8_t);
  ok &= preferences_.putBool("motion_cont",settings.continuousStream)==sizeof(bool);
  ok &= preferences_.putUChar("motion_link",static_cast<uint8_t>(settings.transport))==1;
  ok &= preferences_.putUChar("motion_ble_hz",settings.bleRateHz)==1;
  for(unsigned i=0;i<3;++i) {
    const char *keys[]={"motion_bx","motion_by","motion_bz"};
    ok &= preferences_.putFloat(keys[i],settings.gyroBias[i])==sizeof(float);
  }
  ok &= preferences_.putULong("dim_ms", settings.dimAfterMs) == sizeof(uint32_t);
  return ok;
}

uint32_t ConfigStore::provisioningRevision() {
  // This project has its own revision marker even though both firmwares share
  // the same settings namespace on a board.
  return preferences_.getULong("micro_prov", 0);
}

bool ConfigStore::saveProvisioningRevision(uint32_t revision) {
  return preferences_.putULong("micro_prov", revision) == sizeof(uint32_t);
}

void ConfigStore::clear() {
  preferences_.clear();
}
