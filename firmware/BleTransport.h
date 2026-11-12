#pragma once
#include <atomic>
#include <BLEDevice.h>
#include <BLE2902.h>
#include "AppTypes.h"

class BleTransport : private BLEServerCallbacks, private BLECharacteristicCallbacks,
                     private BLEDescriptorCallbacks {
 public:
  bool begin(const DeviceSettings &settings,RemoteMessageHandler handler);
  void loop();
  bool sendImu(const ImuFrame &frame);
  BleStatus status() const;
  uint32_t epoch() const { return epoch_.load(); }
 private:
  void onConnect(BLEServer *) override;
  void onDisconnect(BLEServer *) override;
#if defined(CONFIG_NIMBLE_ENABLED)
  void onMtuChanged(BLEServer *,ble_gap_conn_desc *,uint16_t mtu) override;
  void onSubscribe(BLECharacteristic *,ble_gap_conn_desc *,uint16_t value) override;
#else
  void onMtuChanged(BLEServer *,esp_ble_gatts_cb_param_t *param) override;
#endif
  void onWrite(BLEDescriptor *descriptor) override;
  void onWrite(BLECharacteristic *characteristic) override;
  void onStatus(BLECharacteristic *,BLECharacteristicCallbacks::Status status,uint32_t) override;
  BLECharacteristic *imu_=nullptr,*control_=nullptr;
  BLE2902 *subscription_=nullptr;
  QueueHandle_t commands_=nullptr;
  RemoteMessageHandler handler_=nullptr;
  bool enabled_=false;
  std::atomic<bool> connected_{false},subscribed_{false},sent_{false};
  std::atomic<uint16_t> mtu_{23};
  std::atomic<uint32_t> epoch_{0};
};
