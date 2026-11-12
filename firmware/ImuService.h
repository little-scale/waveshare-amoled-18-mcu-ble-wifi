#pragma once
#include <SensorQMI8658.hpp>
#include "AppTypes.h"
class ImuService {
 public:
  bool begin(const DeviceSettings &settings);
  bool update(ImuFrame &frame);
  void recenter() { fusion_.recenter(); }
  void startCalibration();
  bool calibrating() const { return calibrationActive_; }
  bool takeCalibration(float bias[3], bool &success);
 private:
  SensorQMI8658 sensor_;
  MotionFusion fusion_;
  bool available_=false;
  uint32_t lastSampleUs_=0;
  float bias_[3]={};
  bool calibrationActive_=false, calibrationDone_=false, calibrationSuccess_=false;
  uint32_t calibrationStarted_=0;
  unsigned calibrationCount_=0;
  float calibrationSum_[3]={}, calibrationAccel_[3]={};
};
