#include <cassert>
#include <cmath>
#include <cstdio>
#include "MotionMath.h"
#include "MotionStream.h"
#include "MotionPacket.h"
static bool near(float a,float b,float e=0.1f) { return std::fabs(a-b)<e; }
int main() {
  const float up[3]={0,0,1},still[3]={0,0,0},yaw[3]={0,0,90};
  MotionFusion f; f.update(up,still,0.005f);
  for(int i=0;i<200;++i) f.update(up,yaw,0.005f);
  float angles[3]; f.relative().euler(angles);
  assert(near(angles[0],0) && near(angles[1],0) && near(angles[2],90));
  f.recenter(); f.relative().euler(angles);
  for(float a:angles) assert(near(a,0));
  for(int i=0;i<100;++i) f.update(up,yaw,0.01f);
  f.relative().euler(angles); assert(near(angles[2],90));
  const auto pose=MotionQuaternion::fromTilt(0.7f,-0.5f);
  (pose.conjugate()*pose).euler(angles);
  for(float a:angles) assert(near(a,0));
  const auto moved=pose*MotionQuaternion::fromTilt(0,0.3f);
  (pose.conjugate()*moved).euler(angles);
  assert(near(angles[0],0.3f*57.29578f) && near(angles[1],0));
  MotionFusion g; g.update(up,still,0.005f);
  const float tilt[3]={-0.5f,0,0.8660254f};
  for(int i=0;i<2000;++i) g.update(tilt,still,0.005f);
  g.relative().euler(angles); assert(near(angles[0],30,0.2f));
  const float impact[3]={4,0,1};
  for(int i=0;i<100;++i) g.update(impact,still,0.005f);
  g.relative().euler(angles); assert(near(angles[0],30,0.2f));
  assert(near(motionAngleDifference(179.9f,-179.9f),0.2f,0.001f));
  MotionStream gate; ImuFrame frame; frame.valid=true;
  assert(gate.due(frame,100000,100,false)); gate.sent(frame,100000);
  assert(!gate.due(frame,120000,100,false));
  frame.accel[0]=0.006f; assert(!gate.due(frame,120000,100,false));
  frame.accel[0]=0.012f; assert(gate.due(frame,120000,100,false));
  assert(!gate.due(frame,109999,100,false));
  assert(!gate.due(frame,119999,50,false));
  gate.sent(frame,120000);
  assert(gate.due(frame,130000,100,true));
  gate.sent(frame,0xfffffff0u);
  assert(gate.due(frame,0x00002700u,100,true));
  frame.valid=false; assert(!gate.due(frame,200000,100,true));
  const float values[9]={0.0f,1.0f,-1.0f,0.125f,-123.75f,1024.0f,-90.0f,179.5f,-0.0f};
  memcpy(frame.accel,values,12); memcpy(frame.gyro,values+3,12); memcpy(frame.orientation,values+6,12);
  uint8_t bytes[36]; encodeMotionPacket(frame,bytes);
  assert(bytes[4]==0 && bytes[5]==0 && bytes[6]==128 && bytes[7]==63);
  for(unsigned i=0;i<9;++i) {
    uint32_t bits=0; for(unsigned b=0;b<4;++b) bits|=uint32_t(bytes[i*4+b])<<(8*b);
    float decoded; memcpy(&decoded,&bits,4);
    assert(memcmp(&decoded,values+i,4)==0);
  }
  RemoteMessage message;
  const uint8_t background[]={1,127,128,255}, hide[]={3,0},invalid[]={3,2};
  assert(decodeBleControl(background,4,message) && message.type==MessageType::Background && message.rgb[2]==255);
  assert(decodeBleControl(hide,2,message) && message.type==MessageType::Display && message.intValue==0);
  assert(!decodeBleControl(invalid,2,message));
  assert(!decodeBleControl(background,3,message));
  assert(!decodeBleControl(nullptr,0,message));
  BleStatus status{true,true,true,23}; assert(!status.ready()); status.mtu=39; assert(status.ready());
  MotionStream independent; frame.valid=true;
  gate.sent(frame,100000); assert(!gate.due(frame,130000,100,false));
  assert(independent.due(frame,130000,50,false));
  puts("PASS: fusion, tilted recenter, gravity correction, acceleration rejection, angle wrap, stream thresholds/rates.");
  puts("PASS: 36-byte float order, little-endian bits, signed zero, BLE commands, MTU gate, independent streams.");
}
