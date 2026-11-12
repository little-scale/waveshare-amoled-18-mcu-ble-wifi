#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

#include "AppTypes.h"

class OscTransport {
 public:
  void begin(const DeviceSettings &settings, RemoteMessageHandler handler);
  void reconfigure(const DeviceSettings &settings);
  void loop();

  bool sendImu(const ImuFrame &frame);

 private:
  bool sendPacket(const String &address, const char *types, const float *floats,
                  size_t floatCount, const int32_t *integers, size_t integerCount);
  bool decodePacket(const uint8_t *data, size_t length, RemoteMessage &message) const;
  String addressFor(const char *control, uint8_t index) const;

  WiFiUDP udp_;
  DeviceSettings settings_;
  RemoteMessageHandler handler_ = nullptr;
  bool listening_ = false;
};
