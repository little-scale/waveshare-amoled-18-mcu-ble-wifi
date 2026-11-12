#pragma once
#include <Arduino.h>
#include "MotionMath.h"

inline uint16_t hardwareDeviceSuffix() {
  const uint64_t mac=ESP.getEfuseMac();
  return (static_cast<uint16_t>((mac>>32)&255)<<8) | ((mac>>40)&255);
}
inline String hardwareDeviceName() {
  char name[16]; snprintf(name,sizeof(name),"device-%04x",hardwareDeviceSuffix()); return name;
}
enum class InputSource : uint8_t { Osc, Ble };
enum class TransportMode : uint8_t { Wifi, Ble, Both };
struct BleStatus {
  bool enabled=false, connected=false, subscribed=false;
  uint16_t mtu=23;
  bool ready() const { return enabled && connected && subscribed && mtu>=39; }
};
struct ControlState {
  uint8_t background[3]={0,0,0};
  uint8_t foreground[3]={255,255,255};
  bool displayEnabled=true;
};
struct ImuFrame {
  float accel[3]={}, gyro[3]={}, orientation[3]={};
  MotionQuaternion rotation;
  uint32_t sampleUs=0;
  uint32_t referenceId=0;
  bool valid=false;
};
enum class MessageType : uint8_t { Invalid, Background, Foreground, Display };
struct RemoteMessage {
  MessageType type=MessageType::Invalid;
  int32_t intValue=0;
  uint8_t rgb[3]={};
};
using RemoteMessageHandler=void (*)(const RemoteMessage &, InputSource);
struct DeviceSettings {
  String deviceName="device-0000", wifiSsid, wifiPassword;
  String oscTarget="192.168.1.2";
  uint16_t oscSendPort=9000, oscReceivePort=9001;
  uint8_t imuRateHz=100;
  bool continuousStream=false;
  TransportMode transport=TransportMode::Ble;
  uint8_t bleRateHz=50;
  uint32_t dimAfterMs=60000;
  float gyroBias[3]={};
};
