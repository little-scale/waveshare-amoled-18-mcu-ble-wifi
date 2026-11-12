#include "UserInterface.h"

#include <Wire.h>
#include <WiFi.h>
#include "BusLock.h"

#include "pin_config.h"

namespace {
constexpr int16_t kScreenWidth = 368;
constexpr int16_t kScreenHeight = 448;
constexpr int16_t kTopHeight = 43;
constexpr int16_t kTopCanvasHeight = kTopHeight + 1;

constexpr int16_t kControlX = 20;
constexpr int16_t kControlY = 57;
constexpr int16_t kControlW = 328;
constexpr int16_t kControlH = 380;
// CO5300 region transfers are expanded to even starts and odd inclusive ends.
// These invisible padding rows preserve the visible layout while making the
// whole control canvas conform to that boundary requirement.
constexpr int16_t kControlCanvasY = kControlY - 1;
constexpr int16_t kControlCanvasH = kControlH + 2;
constexpr int16_t kControlCanvasInsetY = kControlY - kControlCanvasY;

constexpr int16_t kControlGap = 12;
constexpr int16_t kButtonSize = 143;

constexpr int16_t kBallArenaSize = 310;

constexpr int16_t kMazeCellSize = 32;
constexpr int16_t kMazeSize = 9 * kMazeCellSize;

constexpr int16_t kLineWidth = 5;
constexpr float kPi = 3.14159265358979323846f;

constexpr uint8_t kAwakeBrightness = 210;
constexpr uint8_t kDimBrightness = 28;

constexpr int16_t kKeyboardOctaveGap = 12;

constexpr uint32_t kWifiSetupTimeoutMs = 150000;
constexpr uint32_t kWifiScanTimeoutMs = 12000;
constexpr uint32_t kWifiConnectTimeoutMs = 15000;
constexpr uint8_t kWifiRowsPerPage = 5;
constexpr int16_t kWifiListButtonY = 366;
constexpr int16_t kWifiListButtonH = 68;
constexpr int16_t kWifiListTouchY = 350;

void IRAM_ATTR touchInterrupt() {
  // The touch driver owns the interrupt flag; polling finger count keeps release handling reliable.
}

bool inside(int16_t x, int16_t y, int16_t left, int16_t top, int16_t width, int16_t height) {
  return x >= left && x < left + width && y >= top && y < top + height;
}

void drawThickRoundRect(Arduino_GFX *target, int16_t x, int16_t y, int16_t width,
                        int16_t height, int16_t radius, uint16_t color) {
  for (int16_t inset = 0; inset < kLineWidth; ++inset) {
    const int16_t insetRadius = radius - inset > 1 ? radius - inset : 1;
    target->drawRoundRect(x + inset, y + inset, width - 2 * inset,
                          height - 2 * inset, insetRadius, color);
  }
}

void drawCenteredText(Arduino_GFX *target, const String &text, int16_t x, int16_t y,
                      int16_t width, uint8_t size, uint16_t color) {
  target->setTextSize(size);
  target->setTextColor(color);
  target->setTextWrap(false);
  const int16_t textWidth = text.length() * 6 * size;
  const int16_t inset = width > textWidth ? (width - textWidth) / 2 : 0;
  target->setCursor(x + inset, y);
  target->print(text);
}

void drawSetupButton(Arduino_GFX *target, int16_t x, int16_t y, int16_t width,
                     int16_t height, const String &label, uint16_t fill,
                     uint16_t border, uint16_t text) {
  target->fillRoundRect(x, y, width, height, 6, fill);
  drawThickRoundRect(target, x, y, width, height, 6, border);
  drawCenteredText(target, label, x, y + (height - 16) / 2, width, 2, text);
}
}  // namespace

bool UserInterface::begin(const DeviceSettings &settings) {
  dimAfterMs_ = settings.dimAfterMs;
  oscTargetText_ = settings.oscTarget;
  oscSendPortText_ = String(settings.oscSendPort);
  oscReceivePortText_ = String(settings.oscReceivePort);
  deviceNameText_ = settings.deviceName;
  savedDeviceName_ = settings.deviceName;
  savedWifiSsid_ = settings.wifiSsid;
  savedWifiPassword_ = settings.wifiPassword;
  motionRate_=settings.imuRateHz;
  transport_=savedTransport_=settings.transport;
  bleRate_=savedBleRate_=settings.bleRateHz;
  motionContinuous_=settings.continuousStream;
  savedMotionRate_=motionRate_;
  savedMotionContinuous_=motionContinuous_;
  defaultDeviceName_ = hardwareDeviceName();
  Wire.begin(IIC_SDA, IIC_SCL);

  if (expander_.begin(0x20, &Wire)) {
    for (uint8_t pin : {0, 1, 2, 6}) {
      expander_.pinMode(pin, OUTPUT);
      expander_.digitalWrite(pin, LOW);
    }
    delay(20);
    for (uint8_t pin : {0, 1, 2, 6}) expander_.digitalWrite(pin, HIGH);
  }

  displayBus_ = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1,
                                      LCD_SDIO2, LCD_SDIO3);
  display_ = new Arduino_CO5300(displayBus_, GFX_NOT_DEFINED, 0, LCD_WIDTH, LCD_HEIGHT,
                                16, 0, 0, 0);
  if (!display_->begin()) return false;
  display_->setBrightness(kAwakeBrightness);

  // Render changing regions into PSRAM, then transfer each as one image.  Keep
  // the update bounds even-aligned for the CO5300 controller.
  topCanvas_ = new Arduino_Canvas(kScreenWidth, kTopCanvasHeight, display_, 0, 0);
  xyCanvas_ = new Arduino_Canvas(kControlW, kControlCanvasH, display_,
                                 kControlX, kControlCanvasY);
  setupCanvas_ = new Arduino_Canvas(kScreenWidth, kScreenHeight, display_, 0, 0);
  const bool canvasesReady = topCanvas_->begin(GFX_SKIP_OUTPUT_BEGIN) &&
                             xyCanvas_->begin(GFX_SKIP_OUTPUT_BEGIN) &&
                             setupCanvas_->begin(GFX_SKIP_OUTPUT_BEGIN);
  if (!canvasesReady) {
    delete topCanvas_;
    delete xyCanvas_;
    delete setupCanvas_;
    topCanvas_ = nullptr;
    xyCanvas_ = nullptr;
    setupCanvas_ = nullptr;
  }
  Serial.printf("display canvases=%s free_psram=%u\n", canvasesReady ? "ok" : "failed",
                ESP.getFreePsram());

  i2cBus_ = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
  touch_.reset(new Arduino_CST816x(i2cBus_, CST816T_DEVICE_ADDRESS,
                                   DRIVEBUS_DEFAULT_VALUE, TP_INT, touchInterrupt));
  // The CST816 occasionally wakes a little later than the display after a USB
  // flash/reset. A single failed probe used to disable touch for the whole boot.
  // Give it several chances here, then keep a low-rate recovery path in loop().
  initializeTouch(4);

  powerReady_ = power_.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (powerReady_) {
    power_.enableBattDetection();
    power_.enableBattVoltageMeasure();
  }
  lastActivityMs_ = millis();
  forceRedraw();
  return touchReady_;
}

bool UserInterface::initializeTouch(uint8_t attempts) {
  SensorBusLock busLock;
  if (!touch_) return false;
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    if (touch_->begin()) {
      touchReady_ = true;
      touch_->IIC_Write_Device_State(
          touch_->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
          touch_->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);
      Serial.printf("touch initialized attempt=%u\n", attempt + 1);
      return true;
    }
    delay(75);
  }
  touchReady_ = false;
  return false;
}

void UserInterface::serviceTouchRecovery() {
  if (touchReady_ || millis() - lastTouchRecoveryMs_ < 2000) return;
  lastTouchRecoveryMs_ = millis();
  if (initializeTouch(1)) Serial.println("touch recovered");
}

void UserInterface::forceRedraw() { hasDrawn_=false; }

uint16_t UserInterface::rgb565(uint8_t r, uint8_t g, uint8_t b) const {
  return static_cast<uint16_t>(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

uint16_t UserInterface::mixColor(const uint8_t background[3], uint8_t foreground,
                                 uint8_t amount) const {
  const uint8_t inverse = 255 - amount;
  return rgb565((background[0] * inverse + foreground * amount) / 255,
                (background[1] * inverse + foreground * amount) / 255,
                (background[2] * inverse + foreground * amount) / 255);
}

void UserInterface::updatePalette(const ControlState &state) {
  backgroundColor_=rgb565(state.background[0],state.background[1],state.background[2]);
  foregroundColor_=rgb565(state.foreground[0],state.foreground[1],state.foreground[2]);
  auto blend=[&](uint8_t amount) { return rgb565(
      (state.background[0]*(255-amount)+state.foreground[0]*amount)/255,
      (state.background[1]*(255-amount)+state.foreground[1]*amount)/255,
      (state.background[2]*(255-amount)+state.foreground[2]*amount)/255); };
  mutedColor_=blend(105); panelColor_=blend(22);
}

void UserInterface::updatePowerStatus() {
  SensorBusLock busLock;
  if (!powerReady_ || millis() - lastBatteryReadMs_ < 5000) return;
  lastBatteryReadMs_ = millis();
  batteryPercent_ = power_.getBatteryPercent();
  charging_ = power_.isCharging();
}

void UserInterface::openWifiSetup(bool openNetworkPicker) {
  if(openNetworkPicker && savedTransport_==TransportMode::Ble) { openTransport(); return; }
  if (!display_ || wifiSetupActive_) return;
  wifiSetupActive_ = true;
  wifiSetupTouching_ = false;
  wifiSetupLastActivityMs_ = millis();
  wifiPasswordError_ = false;
  wake();
  if (openNetworkPicker) {
    startWifiScan();
  } else {
    wifiSetupPage_ = WifiSetupPage::Menu;
    drawWifiSetup();
  }
}

void UserInterface::closeWifiSetup() {
  if (!wifiSetupActive_) return;
  WiFi.scanDelete();
  wifiScanPending_ = false;
  wifiSetupActive_ = false;
  wifiSetupTouching_ = false;
  // Settings buttons act on touch-down. Do not allow that same physical touch
  // to activate an app control on the next frame.
  touching_ = false;
  ignoreTouchUntilRelease_ = true;
  ignoreTouchStartedMs_ = millis();
  WiFi.setAutoReconnect(true);
  if (savedTransport_!=TransportMode::Ble && WiFi.status() != WL_CONNECTED && !savedWifiSsid_.isEmpty()) {
    WiFi.begin(savedWifiSsid_.c_str(), savedWifiPassword_.c_str());
  }
  forceRedraw();
}

void UserInterface::startWifiScan() {
  wifiNetworkCount_ = 0;
  wifiNetworkPage_ = 0;
  wifiSetupPage_ = WifiSetupPage::Scanning;
  wifiScanAttempt_ = 0;
  beginWifiScanAttempt();
  drawWifiSetup();
}

void UserInterface::beginWifiScanAttempt() {
  WiFi.scanDelete();
  // Auto-reconnect can make the ESP32 radio reject a new asynchronous scan.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(120);
  wifiScanStartedMs_ = millis();
  const int16_t scanStart = WiFi.scanNetworks(true, true);
  wifiScanPending_ = scanStart == WIFI_SCAN_RUNNING || scanStart >= 0;
  wifiScanRetryAtMs_ = millis() + 300;
  Serial.printf("wifi scan attempt=%u start=%d pending=%s\n",
                wifiScanAttempt_ + 1, scanStart,
                wifiScanPending_ ? "yes" : "no");
}

void UserInterface::serviceWifiSetup() {
  if (!wifiSetupActive_) return;
  if (millis() - wifiSetupLastActivityMs_ >= kWifiSetupTimeoutMs) {
    closeWifiSetup();
    return;
  }

  if (wifiSetupPage_ == WifiSetupPage::Scanning && !wifiScanPending_) {
    if (static_cast<int32_t>(millis() - wifiScanRetryAtMs_) >= 0 &&
        wifiScanAttempt_ < 2) {
      ++wifiScanAttempt_;
      beginWifiScanAttempt();
    } else if (wifiScanAttempt_ >= 2) {
      wifiSetupPage_ = WifiSetupPage::Networks;
      drawWifiSetup();
    }
  } else if (wifiSetupPage_ == WifiSetupPage::Scanning && wifiScanPending_) {
    const int16_t result = WiFi.scanComplete();
    const bool timedOut = millis() - wifiScanStartedMs_ >= kWifiScanTimeoutMs;
    if ((result == 0 || (result != WIFI_SCAN_RUNNING && result < 0) || timedOut) &&
        wifiScanAttempt_ < 2) {
      Serial.printf("wifi scan attempt=%u result=%d timeout=%s; retrying\n",
                    wifiScanAttempt_ + 1, result, timedOut ? "yes" : "no");
      ++wifiScanAttempt_;
      beginWifiScanAttempt();
    } else if (result >= 0) {
      Serial.printf("wifi scan attempt=%u complete networks=%d\n",
                    wifiScanAttempt_ + 1, result);
      wifiNetworkCount_ = 0;
      for (int16_t scanIndex = 0; scanIndex < result; ++scanIndex) {
        const String ssid = WiFi.SSID(scanIndex);
        if (ssid.isEmpty()) continue;
        const int32_t rssi = WiFi.RSSI(scanIndex);
        int16_t existing = -1;
        for (uint8_t i = 0; i < wifiNetworkCount_; ++i) {
          if (wifiNetworks_[i] == ssid) {
            existing = i;
            break;
          }
        }
        if (existing >= 0) {
          if (rssi > wifiRssi_[existing]) wifiRssi_[existing] = rssi;
          continue;
        }

        uint8_t position = wifiNetworkCount_;
        if (position < kMaxWifiNetworks) {
          ++wifiNetworkCount_;
        } else {
          if (rssi <= wifiRssi_[kMaxWifiNetworks - 1]) continue;
          position = kMaxWifiNetworks - 1;
        }
        while (position > 0 && rssi > wifiRssi_[position - 1]) {
          if (position < kMaxWifiNetworks) {
            wifiNetworks_[position] = wifiNetworks_[position - 1];
            wifiRssi_[position] = wifiRssi_[position - 1];
            wifiSecure_[position] = wifiSecure_[position - 1];
          }
          --position;
        }
        if (position < kMaxWifiNetworks) {
          wifiNetworks_[position] = ssid;
          wifiRssi_[position] = rssi;
          wifiSecure_[position] = WiFi.encryptionType(scanIndex) != WIFI_AUTH_OPEN;
        }
      }
      WiFi.scanDelete();
      wifiScanPending_ = false;
      wifiSetupPage_ = WifiSetupPage::Networks;
      drawWifiSetup();
    } else if (result != WIFI_SCAN_RUNNING || timedOut) {
      Serial.printf("wifi scan attempt=%u failed result=%d timeout=%s\n",
                    wifiScanAttempt_ + 1, result, timedOut ? "yes" : "no");
      WiFi.scanDelete();
      wifiScanPending_ = false;
      wifiSetupPage_ = WifiSetupPage::Networks;
      drawWifiSetup();
    }
  }

  if (wifiSetupPage_ == WifiSetupPage::Connecting) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiSetupPage_ = WifiSetupPage::Connected;
      wifiConnectedShownMs_ = millis();
      drawWifiSetup();
    } else if (millis() - wifiConnectStartedMs_ >= kWifiConnectTimeoutMs) {
      WiFi.disconnect(false, false);
      wifiPasswordError_ = true;
      wifiSetupPage_ = WifiSetupPage::Password;
      drawWifiSetup();
    }
  } else if (wifiSetupPage_ == WifiSetupPage::Connected &&
             millis() - wifiConnectedShownMs_ >= 900) {
    closeWifiSetup();
  } else if (wifiSetupPage_ == WifiSetupPage::Saved &&
             millis() - wifiConnectedShownMs_ >= 900) {
    closeWifiSetup();
  }
}

void UserInterface::readWifiSetupTouch() {
  if (!touchReady_ || millis() - lastTouchPollMs_ < 8) return;
  lastTouchPollMs_ = millis();
  bool pressed = false;
  int16_t x = 0;
  int16_t y = 0;
  if (!readTouchPoint(pressed, x, y)) return;
  if (pressed && !wifiSetupTouching_) {
    wifiSetupTouching_ = true;
    wifiSetupLastActivityMs_ = millis();
    wake();
    handleWifiSetupTap(x, y);
  } else if (!pressed) {
    wifiSetupTouching_ = false;
  }
}

const char *UserInterface::keyboardRow(uint8_t row) const {
  static const char *lower[] = {"abcdefg", "hijklmn", "opqrstu", "vwxyz"};
  static const char *upper[] = {"ABCDEFG", "HIJKLMN", "OPQRSTU", "VWXYZ"};
  static const char *common[] = {"1234567", "890-_.@", "!#$%&*+", "=/?():;"};
  static const char *more[] = {"\"',<>", "[]{}", "\\^`|~", ""};
  if (wifiKeyboardMode_ == 1) return common[row];
  if (wifiKeyboardMode_ == 2) return more[row];
  return wifiShift_ ? upper[row] : lower[row];
}

const char *UserInterface::deviceKeyboardRow(uint8_t row) const {
  static const char *letters[] = {"abcdefg", "hijklmn", "opqrstu", "vwxyz"};
  static const char *numbers[] = {"1234567", "890-", "", ""};
  return deviceKeyboardNumbers_ ? numbers[row] : letters[row];
}

void UserInterface::handleWifiSetupTap(int16_t x, int16_t y) {
  if (wifiSetupPage_ == WifiSetupPage::Scanning) {
    WiFi.scanDelete();
    wifiScanPending_ = false;
    wifiSetupPage_ = WifiSetupPage::Menu;
    drawWifiSetup();
    return;
  }
  if (wifiSetupPage_ == WifiSetupPage::Connecting ||
      wifiSetupPage_ == WifiSetupPage::Connected ||
      wifiSetupPage_ == WifiSetupPage::Saved ||
      wifiSetupPage_ == WifiSetupPage::Restarting) return;

  if (wifiSetupPage_ == WifiSetupPage::Menu) {
    if (inside(x, y, 24, 40, 320, 50)) {
      if(savedTransport_==TransportMode::Ble) openTransport(); else startWifiScan();
    } else if (inside(x, y, 24, 100, 320, 50)) {
      oscActiveField_ = 0;
      oscInputError_ = false;
      wifiSetupPage_ = WifiSetupPage::Osc;
      drawWifiSetup();
    } else if (inside(x, y, 24, 160, 320, 50)) {
      deviceNameText_ = savedDeviceName_;
      deviceNameInputError_ = false;
      deviceKeyboardNumbers_ = false;
      wifiSetupPage_ = WifiSetupPage::Device;
      drawWifiSetup();
    } else if (inside(x, y, 24, 220, 320, 50)) {
      calibrationText_="";
      motionRate_=savedMotionRate_;
      motionContinuous_=savedMotionContinuous_;
      wifiSetupPage_ = WifiSetupPage::Motion;
      drawWifiSetup();
    } else if (inside(x,y,24,280,320,50)) {
      openTransport();
    } else if (inside(x, y, 24, 350, 320, 64)) {
      closeWifiSetup();
    }
    return;
  }

  if (wifiSetupPage_ == WifiSetupPage::Networks) {
    if (y >= 56 && y < kWifiListTouchY) {
      const uint8_t row = (y - 56) / 60;
      const uint8_t index = wifiNetworkPage_ * kWifiRowsPerPage + row;
      if (index < wifiNetworkCount_) {
        wifiSelectedSsid_ = wifiNetworks_[index];
        wifiPassword_ = "";
        wifiShift_ = false;
        wifiKeyboardMode_ = 0;
        wifiPasswordVisible_ = false;
        wifiPasswordError_ = false;
        wifiSetupPage_ = WifiSetupPage::Password;
        drawWifiSetup();
      }
      return;
    }
    if (y >= kWifiListTouchY) {
      if (x < 124) {
        wifiSetupPage_ = WifiSetupPage::Menu;
        drawWifiSetup();
      }
      else if (x < 246) startWifiScan();
      else {
        const uint8_t calculatedPages =
            (wifiNetworkCount_ + kWifiRowsPerPage - 1) / kWifiRowsPerPage;
        const uint8_t pages = calculatedPages > 0 ? calculatedPages : 1;
        wifiNetworkPage_ = (wifiNetworkPage_ + 1) % pages;
        drawWifiSetup();
      }
    }
    return;
  }

  if (wifiSetupPage_ == WifiSetupPage::Osc) {
    const int16_t fieldY[3] = {30, 78, 126};
    for (uint8_t field = 0; field < 3; ++field) {
      if (inside(x, y, 10, fieldY[field], 348, 42)) {
        oscActiveField_ = field;
        if (x >= 310) {
          if (field == 0) oscTargetText_ = "";
          else if (field == 1) oscSendPortText_ = "";
          else oscReceivePortText_ = "";
        }
        oscInputError_ = false;
        drawWifiSetup();
        return;
      }
    }

    if (y >= 178 && y < 366) {
      const uint8_t row = (y - 178) / 47;
      const int16_t keyX[3] = {10, 130, 250};
      for (uint8_t col = 0; col < 3; ++col) {
        if (!inside(x, y, keyX[col] - 5, 178 + row * 47, 118, 47)) continue;
        static const char keys[] = "123456789.0<";
        const char key = keys[row * 3 + col];
        String *active = oscActiveField_ == 0 ? &oscTargetText_
                          : (oscActiveField_ == 1 ? &oscSendPortText_
                                                  : &oscReceivePortText_);
        if (key == '<') {
          if (!active->isEmpty()) active->remove(active->length() - 1);
        } else if (key != '.' || oscActiveField_ == 0) {
          const size_t limit = oscActiveField_ == 0 ? 15 : 5;
          if (active->length() < limit) *active += key;
        }
        oscInputError_ = false;
        drawWifiSetup();
        return;
      }
    }

    if (y >= 378) {
      if (x < kScreenWidth / 2) {
        wifiSetupPage_ = WifiSetupPage::Menu;
        drawWifiSetup();
      } else if (oscSettingsValid()) {
        UiEvent event;
        event.type = UiEventType::OscSettings;
        event.text[0] = oscTargetText_;
        event.numbers[0] = static_cast<uint16_t>(oscSendPortText_.toInt());
        event.numbers[1] = static_cast<uint16_t>(oscReceivePortText_.toInt());
        enqueue(event);
        wifiSetupPage_ = WifiSetupPage::Saved;
        wifiConnectedShownMs_ = millis();
        drawWifiSetup();
      } else {
        oscInputError_ = true;
        drawWifiSetup();
      }
    }
    return;
  }

  if (wifiSetupPage_ == WifiSetupPage::Device) {
    if (y >= 110 && y < 306) {
      const uint8_t row = (y - 110) / 50;
      if ((y - 110) % 50 < 48) {
        const char *keys = deviceKeyboardRow(row);
        const uint8_t count = strlen(keys);
        if (count == 0) return;
        const int16_t gap = 3;
        const int16_t keyWidth = (kScreenWidth - 16 - (count - 1) * gap) / count;
        for (uint8_t col = 0; col < count; ++col) {
          const int16_t keyX = 8 + col * (keyWidth + gap);
          if (!inside(x, y, keyX, 110 + row * 50, keyWidth + gap, 48)) continue;
          if (deviceNameText_.length() < 24) deviceNameText_ += keys[col];
          deviceNameInputError_ = false;
          drawWifiSetup();
          return;
        }
      }
    }
    if (y >= 314 && y < 360) {
      if (x < 90) {
        deviceKeyboardNumbers_ = !deviceKeyboardNumbers_;
      } else if (x < 180) {
        deviceNameText_ = "";
      } else if (x < 284) {
        deviceNameText_ = defaultDeviceName_;
      } else if (!deviceNameText_.isEmpty()) {
        deviceNameText_.remove(deviceNameText_.length() - 1);
      }
      deviceNameInputError_ = false;
      drawWifiSetup();
      return;
    }
    if (y >= 376) {
      if (x < kScreenWidth / 2) {
        wifiSetupPage_ = WifiSetupPage::Menu;
        drawWifiSetup();
      } else if (deviceNameValid()) {
        UiEvent event;
        event.type = UiEventType::DeviceName;
        event.text[0] = deviceNameText_;
        enqueue(event);
        wifiSetupPage_ = WifiSetupPage::Restarting;
        drawWifiSetup();
      } else {
        deviceNameInputError_ = true;
        drawWifiSetup();
      }
    }
    return;
  }

  if (wifiSetupPage_ == WifiSetupPage::Transport) {
    if(inside(x,y,24,55,320,60)) {
      transport_=static_cast<TransportMode>((static_cast<uint8_t>(transport_)+1)%3);
    } else if(inside(x,y,24,130,320,60)) {
      bleRate_=bleRate_==50?100:50;
    } else if(inside(x,y,10,378,168,56)) {
      wifiSetupPage_=WifiSetupPage::Menu;
    } else if(inside(x,y,190,378,168,56)) {
      UiEvent event; event.type=UiEventType::TransportSettings;
      event.numbers[0]=static_cast<uint8_t>(transport_); event.numbers[1]=bleRate_;
      enqueue(event);
      wifiSetupPage_=transport_!=savedTransport_?WifiSetupPage::Restarting:WifiSetupPage::Menu;
      savedTransport_=transport_; savedBleRate_=bleRate_;
    }
    drawWifiSetup(); return;
  }
  if (wifiSetupPage_ == WifiSetupPage::Motion) {
    if (inside(x,y,24,55,320,60)) {
      motionRate_=motionRate_==100?50:100;
    } else if (inside(x,y,24,130,320,60)) {
      motionContinuous_=!motionContinuous_;
    } else if (inside(x,y,24,205,320,60)) {
      UiEvent event; event.type=UiEventType::Calibrate; enqueue(event);
      calibrationText_="KEEP STILL...";
    } else if (inside(x,y,24,280,320,60)) {
      UiEvent event; event.type=UiEventType::Recenter; enqueue(event);
      calibrationText_="CENTRED";
    } else if (y>=378) {
      if (x>=kScreenWidth/2) {
        UiEvent event; event.type=UiEventType::MotionSettings;
        event.numbers[0]=motionRate_; event.numbers[1]=motionContinuous_;
        savedMotionRate_=motionRate_;
        savedMotionContinuous_=motionContinuous_;
        enqueue(event);
      }
      wifiSetupPage_=WifiSetupPage::Menu;
    }
    drawWifiSetup(); return;
  }

  if (wifiSetupPage_ != WifiSetupPage::Password) return;
  if (inside(x, y, 278, 48, 80, 48)) {
    wifiPasswordVisible_ = !wifiPasswordVisible_;
    drawWifiSetup();
    return;
  }
  if (y >= 110 && y < 306) {
    const uint8_t row = (y - 110) / 50;
    if ((y - 110) % 50 >= 48) return;
    const char *keys = keyboardRow(row);
    const uint8_t count = strlen(keys);
    if (count == 0) return;
    const int16_t gap = 3;
    const int16_t keyWidth = (kScreenWidth - 16 - (count - 1) * gap) / count;
    for (uint8_t col = 0; col < count; ++col) {
      const int16_t keyX = 8 + col * (keyWidth + gap);
      if (inside(x, y, keyX, 110 + row * 50, keyWidth + gap, 48)) {
        if (wifiPassword_.length() < 63) wifiPassword_ += keys[col];
        wifiPasswordError_ = false;
        drawWifiSetup();
        return;
      }
    }
  }
  if (y >= 314 && y < 360) {
    if (x < 90) {
      if (wifiKeyboardMode_ == 0) wifiShift_ = !wifiShift_;
    } else if (x < 180) {
      wifiKeyboardMode_ = (wifiKeyboardMode_ + 1) % 3;
    } else if (x < 284) {
      if (wifiPassword_.length() < 63) wifiPassword_ += ' ';
    } else if (!wifiPassword_.isEmpty()) {
      wifiPassword_.remove(wifiPassword_.length() - 1);
    }
    wifiPasswordError_ = false;
    drawWifiSetup();
    return;
  }
  if (y >= 376) {
    if (x < kScreenWidth / 2) {
      wifiSetupPage_ = WifiSetupPage::Networks;
      drawWifiSetup();
    } else {
      UiEvent event;
      event.type = UiEventType::WifiCredentials;
      event.text[0] = wifiSelectedSsid_;
      event.text[1] = wifiPassword_;
      savedWifiSsid_ = wifiSelectedSsid_;
      savedWifiPassword_ = wifiPassword_;
      enqueue(event);
      wifiSetupPage_ = WifiSetupPage::Connecting;
      wifiConnectStartedMs_ = millis();
      drawWifiSetup();
    }
  }
}

void UserInterface::drawWifiMessage(Arduino_GFX *target, const char *message) {
  drawCenteredText(target, message, 0, 198, kScreenWidth, 2, foregroundColor_);
}

bool UserInterface::oscSettingsValid() const {
  IPAddress address;
  if (!address.fromString(oscTargetText_)) return false;
  const long sendPort = oscSendPortText_.toInt();
  const long receivePort = oscReceivePortText_.toInt();
  return sendPort >= 1 && sendPort <= 65535 &&
         receivePort >= 1 && receivePort <= 65535;
}

bool UserInterface::deviceNameValid() const {
  if (deviceNameText_.isEmpty() || deviceNameText_.length() > 24 ||
      deviceNameText_.startsWith("-") || deviceNameText_.endsWith("-") ||
      deviceNameText_.indexOf("--") >= 0) return false;
  for (size_t i = 0; i < deviceNameText_.length(); ++i) {
    const char value = deviceNameText_[i];
    if (!((value >= 'a' && value <= 'z') || (value >= '0' && value <= '9') ||
          value == '-')) return false;
  }
  return true;
}

void UserInterface::drawSettingsMenu(Arduino_GFX *target) {
  drawCenteredText(target, "SETTINGS", 0, 12, kScreenWidth, 2, foregroundColor_);
  drawSetupButton(target, 24, 40, 320, 50, "NETWORK", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 24, 100, 320, 50, "OSC", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 24, 160, 320, 50, "DEVICE", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 24, 220, 320, 50, "MOTION", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target,24,280,320,50,"TRANSPORT",panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target, 24, 350, 320, 64, "EXIT", panelColor_, mutedColor_,
                  foregroundColor_);
}

void UserInterface::drawWifiNetworks(Arduino_GFX *target) {
  drawCenteredText(target, "WI-FI", 0, 18, kScreenWidth, 2, foregroundColor_);
  const uint8_t calculatedPages =
      (wifiNetworkCount_ + kWifiRowsPerPage - 1) / kWifiRowsPerPage;
  const uint8_t pages = calculatedPages > 0 ? calculatedPages : 1;
  if (wifiNetworkCount_ == 0) {
    drawCenteredText(target, "NO NETWORKS", 0, 195, kScreenWidth, 2, mutedColor_);
  }
  for (uint8_t row = 0; row < kWifiRowsPerPage; ++row) {
    const uint8_t index = wifiNetworkPage_ * kWifiRowsPerPage + row;
    if (index >= wifiNetworkCount_) break;
    const int16_t y = 56 + row * 60;
    target->fillRoundRect(10, y, 348, 52, 6, panelColor_);
    drawThickRoundRect(target, 10, y, 348, 52, 6, mutedColor_);
    String label = wifiNetworks_[index];
    if (label.length() > 20) label = label.substring(0, 19) + ">";
    target->setTextSize(2);
    target->setTextColor(foregroundColor_);
    target->setCursor(20, y + 18);
    target->print(label);
    const uint8_t bars = constrain(map(wifiRssi_[index], -90, -40, 1, 4), 1L, 4L);
    for (uint8_t bar = 0; bar < 4; ++bar) {
      const int16_t height = 5 + bar * 5;
      if (bar < bars) target->fillRect(300 + bar * 6, y + 38 - height, 4, height,
                                       foregroundColor_);
      else target->drawRect(300 + bar * 6, y + 38 - height, 4, height, mutedColor_);
    }
    if (wifiSecure_[index]) {
      target->drawRect(332, y + 24, 14, 12, foregroundColor_);
      target->drawRoundRect(335, y + 15, 8, 12, 4, foregroundColor_);
    }
  }
  drawSetupButton(target, 10, kWifiListButtonY, 108, kWifiListButtonH,
                  "BACK", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 130, kWifiListButtonY, 108, kWifiListButtonH,
                  "SCAN", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 250, kWifiListButtonY, 108, kWifiListButtonH,
                  pages > 1 ? "MORE" : "-", panelColor_,
                  foregroundColor_, pages > 1 ? foregroundColor_ : mutedColor_);
}

void UserInterface::drawOscSettings(Arduino_GFX *target) {
  const char *labels[3] = {"TARGET", "SEND", "RECV"};
  const String *values[3] = {&oscTargetText_, &oscSendPortText_, &oscReceivePortText_};
  const int16_t fieldY[3] = {30, 78, 126};
  drawCenteredText(target, "OSC", 0, 8, kScreenWidth, 2, foregroundColor_);
  for (uint8_t field = 0; field < 3; ++field) {
    target->fillRoundRect(10, fieldY[field], 348, 42, 6, panelColor_);
    const uint16_t border = oscInputError_ ? rgb565(255, 82, 82)
                                           : (field == oscActiveField_ ? foregroundColor_
                                                                      : mutedColor_);
    drawThickRoundRect(target, 10, fieldY[field], 348, 42, 6, border);
    drawCenteredText(target, labels[field], 14, fieldY[field] + 13,
                     76, 2, mutedColor_);
    target->setTextSize(2);
    target->setTextColor(foregroundColor_);
    target->setCursor(96, fieldY[field] + 13);
    target->print(*values[field]);
    drawCenteredText(target, "X", 314, fieldY[field] + 13, 34, 2, mutedColor_);
  }

  static const char keys[] = "123456789.0<";
  const int16_t keyX[3] = {10, 130, 250};
  for (uint8_t row = 0; row < 4; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      const int16_t x = keyX[col];
      const int16_t y = 178 + row * 47;
      String label(keys[row * 3 + col]);
      if (label == "<") label = "DEL";
      const bool unavailableDot = label == "." && oscActiveField_ != 0;
      drawSetupButton(target, x, y, 108, 45, unavailableDot ? "-" : label,
                      panelColor_, mutedColor_,
                      unavailableDot ? mutedColor_ : foregroundColor_);
    }
  }

  drawSetupButton(target, 10, 378, 168, 56, "BACK", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 190, 378, 168, 56, "SAVE", foregroundColor_, foregroundColor_,
                  backgroundColor_);
}

void UserInterface::drawDeviceSettings(Arduino_GFX *target) {
  drawCenteredText(target, "DEVICE", 0, 12, kScreenWidth, 2, foregroundColor_);
  target->fillRoundRect(10, 48, 348, 48, 6, panelColor_);
  drawThickRoundRect(target, 10, 48, 348, 48, 6,
                     deviceNameInputError_ ? rgb565(255, 82, 82) : foregroundColor_);
  drawCenteredText(target, deviceNameText_, 16, 64, 336, 2, foregroundColor_);

  for (uint8_t row = 0; row < 4; ++row) {
    const char *keys = deviceKeyboardRow(row);
    const uint8_t count = strlen(keys);
    if (count == 0) continue;
    const int16_t gap = 3;
    const int16_t keyWidth = (kScreenWidth - 16 - (count - 1) * gap) / count;
    for (uint8_t col = 0; col < count; ++col) {
      const int16_t x = 8 + col * (keyWidth + gap);
      const int16_t y = 110 + row * 50;
      target->fillRoundRect(x, y, keyWidth, 46, 4, panelColor_);
      target->drawRoundRect(x, y, keyWidth, 46, 4, mutedColor_);
      drawCenteredText(target, String(keys[col]), x, y + 15, keyWidth, 2,
                       foregroundColor_);
    }
  }

  drawSetupButton(target, 8, 314, 80, 46,
                  deviceKeyboardNumbers_ ? "ABC" : "123", panelColor_,
                  mutedColor_, foregroundColor_);
  drawSetupButton(target, 94, 314, 80, 46, "CLEAR", panelColor_, mutedColor_,
                  foregroundColor_);
  drawSetupButton(target, 180, 314, 98, 46, "DEFAULT", panelColor_, mutedColor_,
                  foregroundColor_);
  drawSetupButton(target, 284, 314, 76, 46, "DEL", panelColor_, mutedColor_,
                  foregroundColor_);
  drawSetupButton(target, 10, 378, 168, 56, "BACK", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 190, 378, 168, 56, "SAVE", foregroundColor_, foregroundColor_,
                  backgroundColor_);
}

void UserInterface::drawWifiKeyboard(Arduino_GFX *target) {
  String title = wifiSelectedSsid_;
  if (title.length() > 24) title = title.substring(0, 23) + ">";
  drawCenteredText(target, title, 0, 12, kScreenWidth, 2, foregroundColor_);
  target->fillRoundRect(10, 48, 348, 48, 6, panelColor_);
  drawThickRoundRect(target, 10, 48, 348, 48, 6,
                     wifiPasswordError_ ? rgb565(255, 82, 82) : mutedColor_);
  String passwordDisplay;
  const uint8_t visibleCharacters = 20;
  const size_t start = wifiPassword_.length() > visibleCharacters
                           ? wifiPassword_.length() - visibleCharacters
                           : 0;
  if (wifiPasswordVisible_) {
    passwordDisplay = wifiPassword_.substring(start);
  } else {
    for (size_t i = start; i < wifiPassword_.length(); ++i) passwordDisplay += '*';
  }
  if (start > 0) passwordDisplay = ">" + passwordDisplay;
  drawCenteredText(target, passwordDisplay, 18, 64, 250, 2, foregroundColor_);
  drawSetupButton(target, 278, 55, 72, 34, wifiPasswordVisible_ ? "HIDE" : "SHOW",
                  panelColor_, foregroundColor_, foregroundColor_);

  for (uint8_t row = 0; row < 4; ++row) {
    const char *keys = keyboardRow(row);
    const uint8_t count = strlen(keys);
    if (count == 0) continue;
    const int16_t gap = 3;
    const int16_t keyWidth = (kScreenWidth - 16 - (count - 1) * gap) / count;
    for (uint8_t col = 0; col < count; ++col) {
      const int16_t x = 8 + col * (keyWidth + gap);
      const int16_t y = 110 + row * 50;
      target->fillRoundRect(x, y, keyWidth, 46, 4, panelColor_);
      target->drawRoundRect(x, y, keyWidth, 46, 4, mutedColor_);
      String label(keys[col]);
      drawCenteredText(target, label, x, y + 15, keyWidth, 2, foregroundColor_);
    }
  }

  drawSetupButton(target, 8, 314, 80, 46,
                  wifiKeyboardMode_ == 0 ? (wifiShift_ ? "abc" : "ABC") : "-",
                  panelColor_, mutedColor_, foregroundColor_);
  const char *nextMode = wifiKeyboardMode_ == 0 ? "123"
                         : (wifiKeyboardMode_ == 1 ? "MORE" : "ABC");
  drawSetupButton(target, 94, 314, 80, 46, nextMode, panelColor_,
                  mutedColor_, foregroundColor_);
  drawSetupButton(target, 180, 314, 98, 46, "SPACE", panelColor_, mutedColor_,
                  foregroundColor_);
  drawSetupButton(target, 284, 314, 76, 46, "DEL", panelColor_, mutedColor_,
                  foregroundColor_);
  drawSetupButton(target, 10, 378, 168, 56, "BACK", panelColor_, foregroundColor_,
                  foregroundColor_);
  drawSetupButton(target, 190, 378, 168, 56, "CONNECT", foregroundColor_,
                  foregroundColor_, backgroundColor_);
}

void UserInterface::drawWifiSetup() {
  if (!display_ || !wifiSetupActive_) return;
  Arduino_GFX *target = setupCanvas_ ? static_cast<Arduino_GFX *>(setupCanvas_)
                                     : static_cast<Arduino_GFX *>(display_);
  target->fillScreen(backgroundColor_);
  switch (wifiSetupPage_) {
    case WifiSetupPage::Menu:
      drawSettingsMenu(target);
      break;
    case WifiSetupPage::Scanning:
      drawWifiMessage(target, "SCANNING...");
      break;
    case WifiSetupPage::Networks:
      drawWifiNetworks(target);
      break;
    case WifiSetupPage::Password:
      drawWifiKeyboard(target);
      break;
    case WifiSetupPage::Connecting:
      drawWifiMessage(target, "CONNECTING...");
      break;
    case WifiSetupPage::Connected:
      drawWifiMessage(target, "CONNECTED");
      break;
    case WifiSetupPage::Osc:
      drawOscSettings(target);
      break;
    case WifiSetupPage::Device:
      drawDeviceSettings(target);
      break;
    case WifiSetupPage::Motion:
      drawMotionSettings(target);
      break;
    case WifiSetupPage::Transport:
      drawTransportSettings(target);
      break;
    case WifiSetupPage::Saved:
      drawWifiMessage(target, "SAVED");
      break;
    case WifiSetupPage::Restarting:
      drawWifiMessage(target, "RESTARTING...");
      break;
  }
  if (setupCanvas_) setupCanvas_->flush();
}

bool UserInterface::readTouchPoint(bool &pressed, int16_t &x, int16_t &y) {
  SensorBusLock busLock;
  if (!i2cBus_) return false;
  uint8_t sample[5] = {};
  i2cBus_->BeginTransmission(CST816T_DEVICE_ADDRESS);
  if (!i2cBus_->Write(CST816x_RD_DEVICE_FINGERNUM) || !i2cBus_->EndTransmission() ||
      !i2cBus_->RequestFrom(CST816T_DEVICE_ADDRESS, sizeof(sample))) {
    return false;
  }
  for (uint8_t &value : sample) value = i2cBus_->Read();
  pressed = (sample[0] & 0x0f) > 0;
  if (!pressed) return true;
  x = constrain(static_cast<int16_t>(((sample[1] & 0x0f) << 8) | sample[2]),
                static_cast<int16_t>(0), static_cast<int16_t>(kScreenWidth - 1));
  y = constrain(static_cast<int16_t>(((sample[3] & 0x0f) << 8) | sample[4]),
                static_cast<int16_t>(0), static_cast<int16_t>(kScreenHeight - 1));
  return true;
}

void UserInterface::enqueue(const UiEvent &event) {
  const uint8_t next = (eventWrite_ + 1) % kEventQueueSize;
  if (next == eventRead_) return;
  events_[eventWrite_] = event;
  eventWrite_ = next;
}

bool UserInterface::popEvent(UiEvent &event) {
  if (eventRead_ == eventWrite_) return false;
  event = events_[eventRead_];
  eventRead_ = (eventRead_ + 1) % kEventQueueSize;
  return true;
}

void UserInterface::wake() {
  lastActivityMs_ = millis();
  if (dimmed_ && display_) {
    display_->setBrightness(kAwakeBrightness);
    dimmed_ = false;
  }
}

void UserInterface::drawMotionSettings(Arduino_GFX *target) {
  drawCenteredText(target,"MOTION",0,12,kScreenWidth,2,foregroundColor_);
  drawSetupButton(target,24,55,320,60,String(motionRate_)+" Hz",
                  panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,24,130,320,60,motionContinuous_?"CONTINUOUS":"CHANGE ONLY",
                  panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,24,205,320,60,"CALIBRATE GYRO",
                  panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,24,280,320,60,"RE-CENTRE",
                  panelColor_,foregroundColor_,foregroundColor_);
  drawCenteredText(target,calibrationText_,0,350,kScreenWidth,2,foregroundColor_);
  drawSetupButton(target,10,378,168,56,"BACK",panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,190,378,168,56,"SAVE",foregroundColor_,foregroundColor_,backgroundColor_);
}
void UserInterface::calibrationResult(bool success) {
  calibrationText_=success?"CALIBRATED":"MOVED: TRY AGAIN";
  if(wifiSetupActive_ && wifiSetupPage_==WifiSetupPage::Motion) drawWifiSetup();
}

void UserInterface::openTransport() {
  if(!wifiSetupActive_) openWifiSetup(false);
  transport_=savedTransport_; bleRate_=savedBleRate_;
  wifiSetupPage_=WifiSetupPage::Transport; drawWifiSetup();
}
void UserInterface::drawTransportSettings(Arduino_GFX *target) {
  const char *names[]={"WI-FI / OSC","BLE","WI-FI + BLE"};
  drawCenteredText(target,"TRANSPORT",0,12,kScreenWidth,2,foregroundColor_);
  drawSetupButton(target,24,55,320,60,names[static_cast<uint8_t>(transport_)],panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,24,130,320,60,"BLE: "+String(bleRate_)+" Hz",panelColor_,foregroundColor_,foregroundColor_);
  const char *status=!bleStatus_.enabled?"BLE OFF":!bleStatus_.connected?"ADVERTISING":
      bleStatus_.mtu<39?"NEEDS MTU >= 39":!bleStatus_.subscribed?"SUBSCRIBE TO IMU":"READY TO STREAM";
  drawCenteredText(target,status,0,220,kScreenWidth,2,foregroundColor_);
  drawCenteredText(target,"MTU: "+String(bleStatus_.mtu),0,250,kScreenWidth,2,mutedColor_);
  drawCenteredText(target,"Mode change restarts",0,305,kScreenWidth,1,mutedColor_);
  drawSetupButton(target,10,378,168,56,"BACK",panelColor_,foregroundColor_,foregroundColor_);
  drawSetupButton(target,190,378,168,56,"SAVE",foregroundColor_,foregroundColor_,backgroundColor_);
}
void UserInterface::loop(const ControlState &state,const ImuFrame &frame,bool wifiConnected,const BleStatus &ble) {
  if (!display_) return;
  bleChanged_=ble.enabled!=bleStatus_.enabled || ble.connected!=bleStatus_.connected ||
      ble.subscribed!=bleStatus_.subscribed || ble.mtu!=bleStatus_.mtu;
  bleStatus_=ble;
  serviceTouchRecovery();
  const bool paletteChanged=memcmp(state.background,drawnState_.background,3) ||
      memcmp(state.foreground,drawnState_.foreground,3);
  if(paletteChanged || !hasDrawn_) updatePalette(state);
  if (wifiSetupActive_) {
    if(paletteChanged || (bleChanged_ && wifiSetupPage_==WifiSetupPage::Transport)) drawWifiSetup();
    drawnState_=state;
    readWifiSetupTouch(); serviceWifiSetup(); return;
  }
  if(touchReady_ && millis()-lastTouchPollMs_>=8) {
    lastTouchPollMs_=millis();
    bool pressed=false; int16_t x=0,y=0;
    if(readTouchPoint(pressed,x,y)) {
      if(ignoreTouchUntilRelease_) {
        if(!pressed) ignoreTouchUntilRelease_=false;
      } else if(pressed) {
        if(!touching_) {
          touching_=true; touchStartX_=x; touchStartY_=y;
          touchStartedMs_=millis(); tapEligible_=true; wake();
        } else if(abs(x-touchStartX_)>18 || abs(y-touchStartY_)>18) tapEligible_=false;
      } else if(touching_) {
        touching_=false;
        if(tapEligible_ && millis()-touchStartedMs_<600) {
          if(touchStartY_<kTopHeight) {
            if(touchStartX_<72) openWifiSetup();
            else if(touchStartX_<130) openWifiSetup(true);
            else if(touchStartX_<182) openTransport();
          } else if(inside(touchStartX_,touchStartY_,kControlX,kControlY,kControlW,kControlH)) {
            UiEvent event; event.type=UiEventType::Recenter; enqueue(event);
          }
        }
      }
    }
  }
  if(wifiSetupActive_) return;
  updatePowerStatus();
  if(frame.valid && (fabsf(frame.gyro[0])>5 || fabsf(frame.gyro[1])>5 || fabsf(frame.gyro[2])>5)) wake();
  if(!dimmed_ && millis()-lastActivityMs_>=dimAfterMs_) {
    display_->setBrightness(kDimBrightness); dimmed_=true;
  }
  const bool full=!hasDrawn_ || paletteChanged;
  const bool validityChanged=drawnImuValid_!=frame.valid;
  if(full) display_->fillScreen(backgroundColor_);
  if(full || bleChanged_ || drawnWifiConnected_!=wifiConnected || drawnImuValid_!=frame.valid ||
      drawnBatteryPercent_!=batteryPercent_ || drawnCharging_!=charging_) {
    drawTop(wifiConnected,frame.valid);
  }
  const auto &q=frame.rotation;
  const float dot=fabsf(q.w*drawnRotation_.w+q.x*drawnRotation_.x+
                         q.y*drawnRotation_.y+q.z*drawnRotation_.z);
  const bool moved=dot<0.999999f;
  if(full || validityChanged || state.displayEnabled!=drawnState_.displayEnabled ||
      (state.displayEnabled && moved && micros()-lastDrawUs_>=33333)) {
    lastDrawUs_=micros();
    drawMotion(state,frame);
    drawnRotation_=q;
  }
  drawnState_=state; hasDrawn_=true;
}

void UserInterface::drawTop(bool wifiConnected,bool imuValid) {
  Arduino_GFX *t=topCanvas_?static_cast<Arduino_GFX*>(topCanvas_):display_;
  t->fillRect(0,0,kScreenWidth,kTopCanvasHeight,backgroundColor_);
  const int cx=42,cy=22;
  t->fillCircle(cx,cy,12,foregroundColor_);
  for(int axis=0;axis<4;++axis) {
    const float a=axis*1.5707963f;
    const int x=cx+roundf(cosf(a)*13),y=cy+roundf(sinf(a)*13);
    t->fillRect(x-3,y-3,7,7,foregroundColor_);
  }
  t->fillCircle(cx,cy,5,backgroundColor_);
  const uint16_t wifiInk=wifiConnected?foregroundColor_:mutedColor_;
  // Three arcs above a dot; monochrome connection status.
  for(int radius : {8,15,22}) {
    for(int deg=225;deg<=315;++deg) {
      const float a=deg*0.01745329252f;
      t->fillCircle(100+roundf(radius*cosf(a)),34+roundf(radius*sinf(a)),1,wifiInk);
    }
  }
  t->fillCircle(100,34,2,wifiInk);
  const uint16_t bleInk=bleStatus_.ready()?foregroundColor_:mutedColor_;
  for(int offset=0;offset<2;++offset) {
    t->drawLine(150+offset,8,150+offset,36,bleInk);
    t->drawLine(150+offset,8,161+offset,16,bleInk);
    t->drawLine(161+offset,16,140+offset,30,bleInk);
    t->drawLine(140+offset,14,161+offset,28,bleInk);
    t->drawLine(161+offset,28,150+offset,36,bleInk);
  }
  if(bleStatus_.connected) t->fillCircle(170,22,2,bleInk);
  if(!imuValid) drawCenteredText(t,"IMU?",188,17,100,2,foregroundColor_);
  drawThickRoundRect(t,313,12,30,17,3,foregroundColor_);
  t->fillRect(343,17,3,7,foregroundColor_);
  if(batteryPercent_>=0) t->fillRect(317,16,constrain(batteryPercent_,0,100)*22/100,9,foregroundColor_);
  if(topCanvas_) topCanvas_->flush();
  drawnWifiConnected_=wifiConnected; drawnImuValid_=imuValid;
  drawnBatteryPercent_=batteryPercent_; drawnCharging_=charging_;
}

void UserInterface::drawMotion(const ControlState &state,const ImuFrame &frame) {
  Arduino_GFX *t=xyCanvas_?static_cast<Arduino_GFX*>(xyCanvas_):display_;
  const int ox=xyCanvas_?0:kControlX, oy=xyCanvas_?kControlCanvasInsetY:kControlY;
  t->fillRect(ox,oy,kControlW,kControlH,backgroundColor_);
  if(state.displayEnabled && frame.valid) {
    const int cx=ox+kControlW/2,cy=oy+kControlH/2;
    constexpr int radius=139;
    for(int r=radius-2;r<=radius+2;++r) t->drawCircle(cx,cy,r,foregroundColor_);
    // Orthographic wire sphere: show only the facing half of each great circle.
    for(int plane=0;plane<3;++plane) {
      bool previousVisible=false; int px=0,py=0;
      for(int step=0;step<=180;++step) {
        float a=step*0.034906585f, c=cosf(a),s=sinf(a),x,y,z;
        if(plane==0) frame.rotation.rotate(c,s,0,x,y,z);
        else if(plane==1) frame.rotation.rotate(c,0,s,x,y,z);
        else frame.rotation.rotate(0,c,s,x,y,z);
        int sx=cx+roundf(radius*x),sy=cy-roundf(radius*y);
        bool visible=z>=0;
        if(visible && previousVisible) {
          for(int offset=-1;offset<=1;++offset) {
            t->drawLine(px+offset,py,sx+offset,sy,foregroundColor_);
            t->drawLine(px,py+offset,sx,sy+offset,foregroundColor_);
          }
        }
        previousVisible=visible; px=sx;py=sy;
      }
    }
    float x,y,z;
    frame.rotation.rotate(0.45f,0.35f,0.8215838f,x,y,z);
    const int mx=cx+roundf(radius*x),my=cy-roundf(radius*y);
    if(z>=0) t->fillCircle(mx,my,10,foregroundColor_);
    else for(int r=7;r<=9;++r) t->drawCircle(mx,my,r,foregroundColor_);
  }
  if(xyCanvas_) xyCanvas_->flush();
}
