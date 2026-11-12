#pragma once
#include <Adafruit_XCA9554.h>
#include <Arduino_DriveBus_Library.h>
#include <Arduino_GFX_Library.h>
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>
#include "AppTypes.h"

enum class UiEventType : uint8_t { WifiCredentials, OscSettings, DeviceName, MotionSettings, TransportSettings, Recenter, Calibrate };
struct UiEvent {
  UiEventType type=UiEventType::Recenter;
  String text[2];
  uint16_t numbers[2]={};
};
class UserInterface {
 public:
  bool begin(const DeviceSettings &settings);
  void loop(const ControlState &state, const ImuFrame &frame, bool wifiConnected,const BleStatus &ble);
  bool popEvent(UiEvent &event);
  void openWifiSetup(bool openNetworkPicker=false);
  void wake();
  void forceRedraw();
  void calibrationResult(bool success);
 private:
  bool initializeTouch(uint8_t attempts);
  void serviceTouchRecovery();
  bool readTouchPoint(bool &pressed, int16_t &x, int16_t &y);
  void readWifiSetupTouch();
  void handleWifiSetupTap(int16_t x,int16_t y);
  void startWifiScan();
  void beginWifiScanAttempt();
  void serviceWifiSetup();
  void closeWifiSetup();
  void drawWifiSetup();
  void drawSettingsMenu(Arduino_GFX *target);
  void drawWifiNetworks(Arduino_GFX *target);
  void drawWifiKeyboard(Arduino_GFX *target);
  void drawOscSettings(Arduino_GFX *target);
  void drawDeviceSettings(Arduino_GFX *target);
  void drawMotionSettings(Arduino_GFX *target);
  void openTransport();
  void drawTransportSettings(Arduino_GFX *target);
  void drawWifiMessage(Arduino_GFX *target,const char *message);
  bool oscSettingsValid() const;
  bool deviceNameValid() const;
  const char *keyboardRow(uint8_t row) const;
  const char *deviceKeyboardRow(uint8_t row) const;
  void enqueue(const UiEvent &event);
  void updatePalette(const ControlState &state);
  void updatePowerStatus();
  void drawTop(bool wifiConnected, bool imuValid);
  void drawMotion(const ControlState &state,const ImuFrame &frame);
  uint16_t rgb565(uint8_t r,uint8_t g,uint8_t b) const;
  uint16_t mixColor(const uint8_t background[3],uint8_t foreground,uint8_t amount) const;
  Arduino_DataBus *displayBus_=nullptr;
  Arduino_CO5300 *display_=nullptr;
  Arduino_Canvas *topCanvas_=nullptr,*xyCanvas_=nullptr,*setupCanvas_=nullptr;
  std::shared_ptr<Arduino_IIC_DriveBus> i2cBus_;
  std::unique_ptr<Arduino_IIC> touch_;
  Adafruit_XCA9554 expander_;
  XPowersPMU power_;
  bool touchReady_=false,powerReady_=false;
  uint32_t lastTouchRecoveryMs_=0,lastTouchPollMs_=0,lastActivityMs_=0,lastBatteryReadMs_=0;
  uint32_t dimAfterMs_=60000,lastDrawUs_=0;
  bool dimmed_=false,touching_=false,ignoreTouchUntilRelease_=false;
  uint32_t ignoreTouchStartedMs_=0,touchStartedMs_=0;
  int16_t touchStartX_=0,touchStartY_=0;
  bool tapEligible_=false;
  int batteryPercent_=-1;
  bool charging_=false,hasDrawn_=false,drawnWifiConnected_=false,drawnImuValid_=false;
  int drawnBatteryPercent_=-2;
  bool drawnCharging_=false;
  ControlState drawnState_;
  MotionQuaternion drawnRotation_;
  uint8_t motionRate_=100;
  bool motionContinuous_=false;
  uint8_t savedMotionRate_=100;
  bool savedMotionContinuous_=false;
  String calibrationText_;
  TransportMode transport_=TransportMode::Wifi,savedTransport_=TransportMode::Wifi;
  uint8_t bleRate_=50,savedBleRate_=50;
  BleStatus bleStatus_;
  bool bleChanged_=false;
  enum class WifiSetupPage : uint8_t {
    Menu,
    Scanning,
    Networks,
    Password,
    Connecting,
    Connected,
    Osc,
    Device,
    Motion,
    Transport,
    Saved,
    Restarting,
  };
  static constexpr uint8_t kMaxWifiNetworks = 16;
  bool wifiSetupActive_ = false;
  bool wifiSetupTouching_ = false;
  bool wifiScanPending_ = false;
  bool wifiShift_ = false;
  uint8_t wifiKeyboardMode_ = 0;
  bool deviceKeyboardNumbers_ = false;
  bool wifiPasswordVisible_ = false;
  bool wifiPasswordError_ = false;
  WifiSetupPage wifiSetupPage_ = WifiSetupPage::Scanning;
  String wifiNetworks_[kMaxWifiNetworks];
  int32_t wifiRssi_[kMaxWifiNetworks] = {};
  bool wifiSecure_[kMaxWifiNetworks] = {};
  uint8_t wifiNetworkCount_ = 0;
  uint8_t wifiNetworkPage_ = 0;
  String wifiSelectedSsid_;
  String wifiPassword_;
  String savedWifiSsid_;
  String savedWifiPassword_;
  String oscTargetText_;
  String oscSendPortText_;
  String oscReceivePortText_;
  String deviceNameText_;
  String savedDeviceName_;
  String defaultDeviceName_;
  uint8_t oscActiveField_ = 0;
  bool oscInputError_ = false;
  bool deviceNameInputError_ = false;
  uint32_t wifiSetupLastActivityMs_ = 0;
  uint32_t wifiScanStartedMs_ = 0;
  uint32_t wifiScanRetryAtMs_ = 0;
  uint8_t wifiScanAttempt_ = 0;
  uint32_t wifiConnectStartedMs_ = 0;
  uint32_t wifiConnectedShownMs_ = 0;

  uint16_t backgroundColor_=0,foregroundColor_=0xffff,mutedColor_=0x7bef,panelColor_=0x1082;
  static constexpr uint8_t kEventQueueSize=16;
  UiEvent events_[kEventQueueSize];
  uint8_t eventRead_=0,eventWrite_=0;
};
