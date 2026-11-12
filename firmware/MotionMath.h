#pragma once
#include <cmath>

// Right-handed quaternions: sensor/body vectors rotated into the reference frame.
struct MotionQuaternion {
  float w = 1, x = 0, y = 0, z = 0;
  MotionQuaternion conjugate() const { return {w, -x, -y, -z}; }
  MotionQuaternion operator*(const MotionQuaternion &b) const {
    return {w*b.w-x*b.x-y*b.y-z*b.z, w*b.x+x*b.w+y*b.z-z*b.y,
            w*b.y-x*b.z+y*b.w+z*b.x, w*b.z+x*b.y-y*b.x+z*b.w};
  }
  void normalize() {
    const float n = std::sqrt(w*w+x*x+y*y+z*z);
    if (n > 0.000001f) { w/=n; x/=n; y/=n; z/=n; }
    else { w=1; x=y=z=0; }
  }
  void rotate(float a, float b, float c, float &ox, float &oy, float &oz) const {
    const auto v = (*this) * MotionQuaternion{0,a,b,c} * conjugate();
    ox=v.x; oy=v.y; oz=v.z;
  }
  void euler(float out[3]) const {
    constexpr float degrees = 57.2957795131f;
    float sine = 2*(w*y-z*x);
    sine = sine > 1 ? 1 : (sine < -1 ? -1 : sine);
    out[0] = std::asin(sine)*degrees; // pitch
    out[1] = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y))*degrees; // roll
    out[2] = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))*degrees; // yaw
  }
  static MotionQuaternion fromTilt(float roll, float pitch) {
    const float cr=std::cos(roll/2), sr=std::sin(roll/2);
    const float cp=std::cos(pitch/2), sp=std::sin(pitch/2);
    return {cr*cp, sr*cp, cr*sp, -sr*sp};
  }
};

class MotionFusion {
 public:
  void reset() { q_={}; reference_={}; initialized_=false; }
  void recenter() { reference_=q_.conjugate(); }
  MotionQuaternion relative() const { return reference_*q_; }
  void update(const float accel[3], const float gyro[3], float dt) {
    constexpr float radians=0.0174532925199f;
    const float magnitude=std::sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    if (!initialized_) {
      if (magnitude < 0.1f) return;
      q_=MotionQuaternion::fromTilt(std::atan2(accel[1],accel[2]),
          std::atan2(-accel[0], std::sqrt(accel[1]*accel[1]+accel[2]*accel[2])));
      initialized_=true;
    }
    float gx=gyro[0]*radians, gy=gyro[1]*radians, gz=gyro[2]*radians;
    // Proportional gravity correction. Reduce trust during dynamic acceleration;
    // this is six-axis fusion, so there is no absolute yaw correction.
    if (magnitude > 0.8f && magnitude < 1.2f) {
      float vx,vy,vz;
      q_.conjugate().rotate(0,0,1,vx,vy,vz);
      const float ax=accel[0]/magnitude, ay=accel[1]/magnitude, az=accel[2]/magnitude;
      const float gain=1.5f*(1.0f-std::fabs(magnitude-1.0f)/0.2f);
      gx+=gain*(ay*vz-az*vy);
      gy+=gain*(az*vx-ax*vz);
      gz+=gain*(ax*vy-ay*vx);
    }
    const auto derivative=q_*MotionQuaternion{0,gx,gy,gz};
    q_.w+=0.5f*dt*derivative.w; q_.x+=0.5f*dt*derivative.x;
    q_.y+=0.5f*dt*derivative.y; q_.z+=0.5f*dt*derivative.z;
    q_.normalize();
  }
 private:
  MotionQuaternion q_, reference_;
  bool initialized_=false;
};

inline float motionAngleDifference(float a, float b) {
  return std::fabs(std::remainder(a-b,360.0f));
}
