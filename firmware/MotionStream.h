#pragma once
#include "AppTypes.h"

class MotionStream {
 public:
  void reset() { haveLast_=false; }
  bool due(const ImuFrame &frame,uint32_t now,uint8_t hz,bool continuous) const {
    if(!frame.valid) return false;
    if(!haveLast_) return true;
    if(now-lastSentUs_<1000000UL/hz) return false;
    if(continuous) return true;
    for(unsigned i=0;i<3;++i) {
      if(fabsf(frame.accel[i]-last_.accel[i])>=0.01f ||
          fabsf(frame.gyro[i]-last_.gyro[i])>=0.5f ||
          motionAngleDifference(frame.orientation[i],last_.orientation[i])>=0.25f) return true;
    }
    return false;
  }
  void sent(const ImuFrame &frame,uint32_t now) {
    last_=frame; lastSentUs_=now; haveLast_=true;
  }
 private:
  ImuFrame last_;
  bool haveLast_=false;
  uint32_t lastSentUs_=0;
};
