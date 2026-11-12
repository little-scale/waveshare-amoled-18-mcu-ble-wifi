#include "OscTransport.h"

#include <WiFi.h>

namespace {
constexpr size_t kMaxOscPacket = 256;
constexpr char kSpectrumOscTypes[] = ",ffffffffffffffffffffffffffffffff";

size_t paddedStringLength(size_t stringLength) {
  return (stringLength + 1 + 3) & ~static_cast<size_t>(3);
}

bool writePaddedString(uint8_t *buffer, size_t capacity, size_t &offset, const char *text) {
  const size_t textLength = strlen(text);
  const size_t required = paddedStringLength(textLength);
  if (offset + required > capacity) return false;
  memcpy(buffer + offset, text, textLength);
  memset(buffer + offset + textLength, 0, required - textLength);
  offset += required;
  return true;
}

void writeU32Be(uint8_t *target, uint32_t value) {
  target[0] = static_cast<uint8_t>(value >> 24);
  target[1] = static_cast<uint8_t>(value >> 16);
  target[2] = static_cast<uint8_t>(value >> 8);
  target[3] = static_cast<uint8_t>(value);
}

uint32_t readU32Be(const uint8_t *source) {
  return (static_cast<uint32_t>(source[0]) << 24) |
         (static_cast<uint32_t>(source[1]) << 16) |
         (static_cast<uint32_t>(source[2]) << 8) |
         static_cast<uint32_t>(source[3]);
}

bool readPaddedString(const uint8_t *data, size_t length, size_t &offset, String &result) {
  if (offset >= length) return false;
  size_t end = offset;
  while (end < length && data[end] != 0) ++end;
  if (end == length) return false;
  result = String(reinterpret_cast<const char *>(data + offset), end - offset);
  offset += paddedStringLength(end - offset);
  return offset <= length;
}

float readFloatBe(const uint8_t *source) {
  const uint32_t bits = readU32Be(source);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeFloatBe(uint8_t *target, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  writeU32Be(target, bits);
}

bool finiteUnit(float value) {
  return isfinite(value) && value >= 0.0f && value <= 1.0f;
}
}  // namespace

void OscTransport::begin(const DeviceSettings &settings, RemoteMessageHandler handler) {
  handler_ = handler;
  reconfigure(settings);
}

void OscTransport::reconfigure(const DeviceSettings &settings) {
  settings_ = settings;
  if (listening_) udp_.stop();
  listening_ = false;
}

String OscTransport::addressFor(const char *control, uint8_t index) const {
  String address = "/";
  address += settings_.deviceName;
  address += "/";
  address += control;
  address += String(index);
  return address;
}

bool OscTransport::sendPacket(const String &address, const char *types, const float *floats,
                              size_t floatCount, const int32_t *integers, size_t integerCount) {
  if (WiFi.status() != WL_CONNECTED || settings_.oscTarget.isEmpty()) return false;

  uint8_t packet[kMaxOscPacket] = {};
  size_t offset = 0;
  if (!writePaddedString(packet, sizeof(packet), offset, address.c_str()) ||
      !writePaddedString(packet, sizeof(packet), offset, types)) {
    return false;
  }

  size_t floatIndex = 0;
  size_t integerIndex = 0;
  for (size_t i = 1; types[i] != '\0'; ++i) {
    if (offset + 4 > sizeof(packet)) return false;
    if (types[i] == 'f' && floatIndex < floatCount) {
      writeFloatBe(packet + offset, floats[floatIndex++]);
    } else if (types[i] == 'i' && integerIndex < integerCount) {
      writeU32Be(packet + offset, static_cast<uint32_t>(integers[integerIndex++]));
    } else {
      return false;
    }
    offset += 4;
  }

  if (!udp_.beginPacket(settings_.oscTarget.c_str(), settings_.oscSendPort)) return false;
  udp_.write(packet, offset);
  return udp_.endPacket() == 1;
}

bool OscTransport::sendImu(const ImuFrame &frame) {
  float values[9];
  memcpy(values,frame.accel,12); memcpy(values+3,frame.gyro,12);
  memcpy(values+6,frame.orientation,12);
  return sendPacket(addressFor("imu",0),",fffffffff",values,9,nullptr,0);
}

bool OscTransport::decodePacket(const uint8_t *data,size_t length,RemoteMessage &message) const {
  size_t offset=0; String address,types;
  if(!readPaddedString(data,length,offset,address) ||
      !readPaddedString(data,length,offset,types)) return false;
  const String root="/"+settings_.deviceName+"/";
  if(!address.startsWith(root)) return false;
  const String endpoint=address.substring(root.length());
  if((endpoint=="foreground" || endpoint=="background") && types==",iii" && offset+12==length) {
    message.type=endpoint=="foreground"?MessageType::Foreground:MessageType::Background;
    for(unsigned i=0;i<3;++i) {
      const int32_t value=static_cast<int32_t>(readU32Be(data+offset+i*4));
      if(value<0 || value>255) return false;
      message.rgb[i]=value;
    }
    return true;
  }
  if(endpoint=="display" && types==",i" && offset+4==length) {
    message.type=MessageType::Display;
    message.intValue=static_cast<int32_t>(readU32Be(data+offset));
    return message.intValue==0 || message.intValue==1;
  }
  return false;
}
void OscTransport::loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (listening_) {
      udp_.stop();
      listening_ = false;
    }
    return;
  }
  if (!listening_) listening_ = udp_.begin(settings_.oscReceivePort) == 1;
  if (!listening_) return;

  int packetSize;
  for (uint8_t count=0;count<8 && (packetSize=udp_.parsePacket())>0;++count) {
    uint8_t packet[kMaxOscPacket];
    if(packetSize>sizeof(packet)) { udp_.flush(); continue; }
    const size_t received = udp_.read(packet, packetSize);
    RemoteMessage message;
    if (decodePacket(packet, received, message) && handler_) handler_(message, InputSource::Osc);
  }
}
