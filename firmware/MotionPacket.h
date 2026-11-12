#pragma once
#include <cstring>
#include <limits>
#include "AppTypes.h"

// Wire format: nine IEEE-754 binary32 values, little endian, no header.
inline void encodeMotionPacket(const ImuFrame &frame,uint8_t (&bytes)[36]) {
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559,"Requires binary32");
  for(unsigned i=0;i<9;++i) {
    const float value=i<3?frame.accel[i]:i<6?frame.gyro[i-3]:frame.orientation[i-6];
    uint32_t bits; memcpy(&bits,&value,4);
    for(unsigned b=0;b<4;++b) bytes[i*4+b]=static_cast<uint8_t>(bits>>(8*b));
  }
}
inline bool decodeBleControl(const uint8_t *bytes,size_t size,RemoteMessage &message) {
  message=RemoteMessage{};
  if(size==4 && (bytes[0]==1 || bytes[0]==2)) {
    message.type=bytes[0]==1?MessageType::Background:MessageType::Foreground;
    memcpy(message.rgb,bytes+1,3); return true;
  }
  if(size==2 && bytes[0]==3 && bytes[1]<=1) {
    message.type=MessageType::Display; message.intValue=bytes[1]; return true;
  }
  return false;
}
