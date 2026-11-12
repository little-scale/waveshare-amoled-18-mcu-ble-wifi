#include "ImuService.h"
#include "BusLock.h"
#include "pin_config.h"
#include <Wire.h>

bool ImuService::begin(const DeviceSettings &settings) {
  memcpy(bias_,settings.gyroBias,sizeof(bias_));
  SensorBusLock lock;
  available_=sensor_.begin(Wire,QMI8658_L_SLAVE_ADDRESS,IIC_SDA,IIC_SCL);
  if (!available_) return false;
  sensor_.configAccelerometer(SensorQMI8658::ACC_RANGE_8G,
      SensorQMI8658::ACC_ODR_250Hz,SensorQMI8658::LPF_MODE_0);
  sensor_.configGyroscope(SensorQMI8658::GYR_RANGE_1024DPS,
      SensorQMI8658::GYR_ODR_224_2Hz,SensorQMI8658::LPF_MODE_3);
  sensor_.enableAccelerometer(); sensor_.enableGyroscope();
  return true;
}
void ImuService::startCalibration() {
  calibrationActive_=true; calibrationDone_=false; calibrationCount_=0;
  memset(calibrationSum_,0,sizeof(calibrationSum_));
  calibrationStarted_=millis();
}
bool ImuService::takeCalibration(float bias[3], bool &success) {
  if (!calibrationDone_) return false;
  calibrationDone_=false; success=calibrationSuccess_;
  memcpy(bias,bias_,sizeof(bias_)); return true;
}
bool ImuService::update(ImuFrame &frame) {
  if (!available_) return false;
  const uint32_t now=micros();
  if (lastSampleUs_ && now-lastSampleUs_<5000) return false;
  IMUdata a,g;
  {
    SensorBusLock lock;
    if (!sensor_.getDataReady() ||
        !sensor_.getAccelerometer(a.x,a.y,a.z) ||
        !sensor_.getGyroscope(g.x,g.y,g.z)) return false;
  }
  // Proper 90-degree rotation, applied identically to both sensors.
  // Swapping X/Y alone reflects the axes and breaks quaternion handedness.
  frame.accel[0]=a.y; frame.accel[1]=-a.x; frame.accel[2]=a.z;
  const float rawGyro[3]={g.y,-g.x,g.z};
  for (unsigned i=0;i<3;++i) frame.gyro[i]=rawGyro[i]-bias_[i];
  if (calibrationActive_) {
    float norm=0, speed=0, delta=0;
    for (unsigned i=0;i<3;++i) {
      norm+=frame.accel[i]*frame.accel[i]; speed+=rawGyro[i]*rawGyro[i];
      const float d=frame.accel[i]-calibrationAccel_[i]; delta+=d*d;
      calibrationAccel_[i]=frame.accel[i];
    }
    if (norm<0.90f || norm>1.10f || speed>9.0f || delta>0.0009f) {
      calibrationCount_=0; memset(calibrationSum_,0,sizeof(calibrationSum_));
    } else {
      for (unsigned i=0;i<3;++i) calibrationSum_[i]+=rawGyro[i];
      ++calibrationCount_;
    }
    if (calibrationCount_>=300 || millis()-calibrationStarted_>=6000) {
      calibrationSuccess_=calibrationCount_>=300;
      if (calibrationSuccess_) {
        for (unsigned i=0;i<3;++i) bias_[i]=calibrationSum_[i]/calibrationCount_;
      }
      calibrationActive_=false; calibrationDone_=true;
    }
  }
  float dt=lastSampleUs_ ? (now-lastSampleUs_)*0.000001f : 0.005f;
  lastSampleUs_=now;
  if (dt>0.05f) dt=0.005f;
  fusion_.update(frame.accel,frame.gyro,dt);
  frame.rotation=fusion_.relative(); frame.rotation.euler(frame.orientation);
  frame.sampleUs=now; frame.valid=true;
  return true;
}
