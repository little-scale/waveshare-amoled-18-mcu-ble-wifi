#include "BleTransport.h"
#include "MotionPacket.h"

namespace {
constexpr char serviceUuid[]="89689de0-1604-42c1-85ba-e286075c4488";
constexpr char imuUuid[]="2e5abe1b-4f44-429e-8b47-1526afbe343d";
constexpr char controlUuid[]="c2bbf0cb-9029-4925-abbc-3dd86492fc06";
constexpr char infoUuid[]="fe9c3dcb-451c-4512-b845-a611ffa3280f";
}
bool BleTransport::begin(const DeviceSettings &settings,RemoteMessageHandler handler) {
  if(settings.transport==TransportMode::Wifi) return true;
  commands_=xQueueCreate(16,sizeof(RemoteMessage));
  if(!commands_) return false;
  handler_=handler;
  BLEDevice::init(settings.deviceName.c_str()); BLEDevice::setMTU(64);
  BLEServer *server=BLEDevice::createServer(); server->setCallbacks(this);
  BLEService *service=server->createService(serviceUuid);
  imu_=service->createCharacteristic(imuUuid,BLECharacteristic::PROPERTY_NOTIFY);
  imu_->setCallbacks(this);
#if defined(CONFIG_BLUEDROID_ENABLED)
  subscription_=new BLE2902(); subscription_->setCallbacks(this);
  imu_->addDescriptor(subscription_);
#endif
  control_=service->createCharacteristic(controlUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  control_->setCallbacks(this);
  auto *info=service->createCharacteristic(infoUuid,BLECharacteristic::PROPERTY_READ);
  info->setValue("{\"protocol\":2,\"bytes\":36,\"format\":\"float32-le\",\"header\":false,"
      "\"order\":[\"ax\",\"ay\",\"az\",\"gx\",\"gy\",\"gz\",\"pitch\",\"roll\",\"yaw\"],\"min_mtu\":39}");
  service->start();
  auto *advertising=BLEDevice::getAdvertising();
  advertising->addServiceUUID(serviceUuid); advertising->setScanResponse(true);
  enabled_=true; advertising->start(); return true;
}
BleStatus BleTransport::status() const {
  return {enabled_,connected_.load(),subscribed_.load(),mtu_.load()};
}
void BleTransport::onConnect(BLEServer *) {
  mtu_=23; subscribed_=false;
  if(subscription_) subscription_->setNotifications(false);
  connected_=true; ++epoch_;
}
void BleTransport::onDisconnect(BLEServer *) {
  connected_=false; subscribed_=false; mtu_=23; ++epoch_;
  BLEDevice::startAdvertising();
}
#if defined(CONFIG_NIMBLE_ENABLED)
void BleTransport::onMtuChanged(BLEServer *,ble_gap_conn_desc *,uint16_t mtu) {
  mtu_=mtu; ++epoch_;
}
void BleTransport::onSubscribe(BLECharacteristic *characteristic,ble_gap_conn_desc *,uint16_t value) {
  if(characteristic==imu_) { subscribed_=(value&1)!=0; ++epoch_; }
}
#else
void BleTransport::onMtuChanged(BLEServer *,esp_ble_gatts_cb_param_t *param) {
  mtu_=param->mtu.mtu; ++epoch_;
}
#endif
void BleTransport::onWrite(BLEDescriptor *) {
  if(subscription_) { subscribed_=subscription_->getNotifications(); ++epoch_; }
}
void BleTransport::onWrite(BLECharacteristic *characteristic) {
  if(characteristic!=control_) return;
  auto value=characteristic->getValue();
  RemoteMessage message;
  if(decodeBleControl(reinterpret_cast<const uint8_t*>(value.c_str()),value.length(),message))
    xQueueSend(commands_,&message,0);
}
void BleTransport::onStatus(BLECharacteristic *characteristic,
    BLECharacteristicCallbacks::Status status,uint32_t) {
  if(characteristic==imu_) sent_=status==BLECharacteristicCallbacks::SUCCESS_NOTIFY;
}
bool BleTransport::sendImu(const ImuFrame &frame) {
  if(!status().ready()) return false;
  uint8_t packet[36]; encodeMotionPacket(frame,packet);
  imu_->setValue(packet,sizeof(packet)); sent_=false; imu_->notify();
  return sent_.load(); // Stack accepted the notification; not an application acknowledgement.
}
void BleTransport::loop() {
  if(!commands_) return;
  RemoteMessage message;
  for(unsigned i=0;i<16 && xQueueReceive(commands_,&message,0)==pdTRUE;++i)
    if(handler_) handler_(message,InputSource::Ble);
}
