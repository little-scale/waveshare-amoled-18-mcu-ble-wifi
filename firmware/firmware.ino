#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "AppTypes.h"
#include "ConfigStore.h"
#include "ImuService.h"
#include "OscTransport.h"
#include "Provisioning.h"
#include "UserInterface.h"
#include "MotionStream.h"
#include "BleTransport.h"
#include "BusLock.h"

SemaphoreHandle_t sensorBusMutex=nullptr;
namespace {
DeviceSettings settings;
ControlState controls;
ConfigStore configStore;
ImuService imu;
UserInterface ui;
OscTransport receiver;
BleTransport ble;
ImuFrame latest;
portMUX_TYPE frameMutex=portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t commands, calibrationResults, networkSettings;
uint32_t restartAtMs=0;
bool mdnsStarted=false;
uint32_t sampleCount=0,packetCount=0,blePacketCount=0;
struct CalibrationResult { float bias[3]; bool success; };
struct NetworkSettings {
  char name[32],target[64];
  uint16_t sendPort,receivePort;
  uint8_t hz,bleHz;
  TransportMode transport;
  bool continuous;
};
void publishNetworkSettings() {
  NetworkSettings next={};
  settings.deviceName.toCharArray(next.name,sizeof(next.name));
  settings.oscTarget.toCharArray(next.target,sizeof(next.target));
  next.sendPort=settings.oscSendPort; next.receivePort=settings.oscReceivePort;
  next.hz=settings.imuRateHz; next.continuous=settings.continuousStream;
  next.bleHz=settings.bleRateHz; next.transport=settings.transport;
  xQueueOverwrite(networkSettings,&next);
}
ImuFrame readLatest() {
  portENTER_CRITICAL(&frameMutex);
  ImuFrame frame=latest;
  portEXIT_CRITICAL(&frameMutex);
  if(frame.valid && micros()-frame.sampleUs>100000) frame.valid=false;
  return frame;
}
void sensorTask(void *) {
  ImuFrame frame;
  uint32_t referenceId=0;
  for(;;) {
    uint8_t command;
    while(xQueueReceive(commands,&command,0)==pdTRUE) {
      if(command==1) { imu.recenter(); ++referenceId; }
      else if(command==2) imu.startCalibration();
    }
    if(imu.update(frame)) {
      frame.referenceId=referenceId;
      portENTER_CRITICAL(&frameMutex);
      latest=frame; ++sampleCount;
      portEXIT_CRITICAL(&frameMutex);
    }
    CalibrationResult result;
    if(imu.takeCalibration(result.bias,result.success)) xQueueOverwrite(calibrationResults,&result);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void streamTask(void *) {
  OscTransport sender;
  MotionStream gate,bleGate;
  DeviceSettings active;
  bool configured=false,wasConnected=false;
  uint32_t referenceId=0,lastSample=0,lastAttempt=0;
  uint32_t bleEpoch=0,bleLastSample=0,bleLastAttempt=0;
  for(;;) {
    NetworkSettings config;
    if(xQueueReceive(networkSettings,&config,0)==pdTRUE) {
      active.deviceName=config.name; active.oscTarget=config.target;
      active.oscSendPort=config.sendPort; active.oscReceivePort=config.receivePort;
      active.imuRateHz=config.hz; active.continuousStream=config.continuous;
      active.bleRateHz=config.bleHz; active.transport=config.transport;
      if(active.transport!=TransportMode::Ble) sender.begin(active,nullptr);
      gate.reset(); bleGate.reset(); configured=true;
    }
    const bool connected=WiFi.status()==WL_CONNECTED;
    if(connected && !wasConnected) gate.reset();
    wasConnected=connected;
    const ImuFrame frame=readLatest();
    if(frame.referenceId!=referenceId) { referenceId=frame.referenceId; gate.reset(); bleGate.reset(); }
    const uint32_t epoch=ble.epoch();
    if(epoch!=bleEpoch) { bleEpoch=epoch; bleGate.reset(); }
    const uint32_t now=micros();
    // Latest-frame mailbox: never queue historical motion behind the display/network.
    if(configured && active.transport!=TransportMode::Ble && connected && frame.valid && now-frame.sampleUs<100000 &&
        frame.sampleUs!=lastSample && now-lastAttempt>=1000000UL/active.imuRateHz &&
        gate.due(frame,now,active.imuRateHz,active.continuousStream)) {
      lastAttempt=now;
      if(sender.sendImu(frame)) {
        gate.sent(frame,now); lastSample=frame.sampleUs;
        portENTER_CRITICAL(&frameMutex); ++packetCount; portEXIT_CRITICAL(&frameMutex);
      }
    }
    if(configured && ble.status().ready() && frame.valid && frame.sampleUs!=bleLastSample &&
        now-bleLastAttempt>=1000000UL/active.bleRateHz &&
        bleGate.due(frame,now,active.bleRateHz,active.continuousStream)) {
      bleLastAttempt=now;
      if(ble.sendImu(frame)) {
        bleGate.sent(frame,now); bleLastSample=frame.sampleUs;
        portENTER_CRITICAL(&frameMutex); ++blePacketCount; portEXIT_CRITICAL(&frameMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void applyRemote(const RemoteMessage &message,InputSource) {
  switch(message.type) {
    case MessageType::Background: memcpy(controls.background,message.rgb,3); break;
    case MessageType::Foreground: memcpy(controls.foreground,message.rgb,3); break;
    case MessageType::Display: controls.displayEnabled=message.intValue!=0; break;
    default: return;
  }
  ui.wake();
}
void handleEvents() {
  UiEvent event;
  while(ui.popEvent(event)) {
    switch(event.type) {
      case UiEventType::Recenter: {
        uint8_t command=1; xQueueSend(commands,&command,0); break;
      }
      case UiEventType::Calibrate: {
        uint8_t command=2; xQueueSend(commands,&command,0); break;
      }
      case UiEventType::WifiCredentials:
        settings.wifiSsid=event.text[0]; settings.wifiPassword=event.text[1];
        configStore.save(settings);
        WiFi.disconnect(false,false);
        WiFi.begin(settings.wifiSsid.c_str(),settings.wifiPassword.c_str());
        break;
      case UiEventType::OscSettings:
        settings.oscTarget=event.text[0]; settings.oscSendPort=event.numbers[0];
        settings.oscReceivePort=event.numbers[1];
        configStore.save(settings); receiver.reconfigure(settings); publishNetworkSettings();
        if(mdnsStarted) { MDNS.end(); mdnsStarted=false; }
        break;
      case UiEventType::DeviceName:
        settings.deviceName=event.text[0]; configStore.save(settings);
        restartAtMs=millis()+700; break;
      case UiEventType::MotionSettings:
        settings.imuRateHz=event.numbers[0]; settings.continuousStream=event.numbers[1]!=0;
        configStore.save(settings); publishNetworkSettings(); break;
      case UiEventType::TransportSettings: {
        const auto mode=static_cast<TransportMode>(event.numbers[0]);
        const bool restart=mode!=settings.transport;
        settings.transport=mode; settings.bleRateHz=event.numbers[1];
        configStore.save(settings);
        if(restart) restartAtMs=millis()+700;
        else publishNetworkSettings();
        break;
      }
    }
  }
}
}
void setup() {
  Serial.begin(115200); delay(100);
  sensorBusMutex=xSemaphoreCreateMutex();
  commands=xQueueCreate(8,sizeof(uint8_t));
  calibrationResults=xQueueCreate(1,sizeof(CalibrationResult));
  networkSettings=xQueueCreate(1,sizeof(NetworkSettings));
  if(!sensorBusMutex || !commands || !calibrationResults || !networkSettings) {
    Serial.println("Motion startup failed: allocation"); return;
  }
  const bool configReady=configStore.begin();
  if(configReady) configStore.load(settings);
  if(Provisioning::available() && Provisioning::revision()>configStore.provisioningRevision()) {
    Provisioning::apply(settings);
    if(configStore.save(settings)) configStore.saveProvisioningRevision(Provisioning::revision());
  }
  if(settings.deviceName=="device-0000" || settings.deviceName=="classroom-01") {
    settings.deviceName=hardwareDeviceName(); configStore.save(settings);
  }
  const bool touchReady=ui.begin(settings);
  const bool imuReady=imu.begin(settings);
  if(settings.transport!=TransportMode::Ble) {
    WiFi.mode(WIFI_STA); WiFi.setHostname(settings.deviceName.c_str());
    WiFi.setAutoReconnect(true); WiFi.persistent(false);
    WiFi.setSleep(settings.transport==TransportMode::Both);
    if(!settings.wifiSsid.isEmpty()) WiFi.begin(settings.wifiSsid.c_str(),settings.wifiPassword.c_str());
    receiver.begin(settings,applyRemote);
  } else WiFi.mode(WIFI_OFF);
  const bool bleStarted=ble.begin(settings,applyRemote);
  publishNetworkSettings();
  const bool sensorStarted=xTaskCreatePinnedToCore(sensorTask,"motion-sensor",4096,nullptr,3,nullptr,0)==pdPASS;
  const bool streamStarted=xTaskCreatePinnedToCore(streamTask,"motion-osc",6144,nullptr,2,nullptr,0)==pdPASS;
  Serial.printf("motion boot touch=%d imu=%d sensor_task=%d osc_task=%d device=%s rate=%u\n",
      touchReady,imuReady,sensorStarted,streamStarted,settings.deviceName.c_str(),settings.imuRateHz);
  Serial.printf("transport=%u ble_started=%d ble_cap=%u\n",static_cast<unsigned>(settings.transport),bleStarted,settings.bleRateHz);
  if(settings.transport!=TransportMode::Ble && settings.wifiSsid.isEmpty()) ui.openWifiSetup(true);
}
void loop() {
  if(!sensorBusMutex || !commands || !calibrationResults || !networkSettings) { delay(100); return; }
  if(settings.transport!=TransportMode::Ble) receiver.loop();
  ble.loop();
  const bool connected=WiFi.status()==WL_CONNECTED;
  if(connected && !mdnsStarted) {
    mdnsStarted=MDNS.begin(settings.deviceName.c_str());
    if(mdnsStarted) MDNS.addService("osc","udp",settings.oscReceivePort);
  } else if(!connected && mdnsStarted) { MDNS.end(); mdnsStarted=false; }
  CalibrationResult result;
  if(xQueueReceive(calibrationResults,&result,0)==pdTRUE) {
    if(result.success) { memcpy(settings.gyroBias,result.bias,sizeof(result.bias)); configStore.save(settings); }
    ui.calibrationResult(result.success);
  }
  const ImuFrame frame=readLatest();
  ui.loop(controls,frame,connected,ble.status());
  handleEvents();
  static uint32_t lastReport=0,previousSamples=0,previousPackets=0,previousBlePackets=0;
  if(millis()-lastReport>=5000) {
    const uint32_t now=millis(),elapsed=now-lastReport;
    uint32_t samples,packets,blePackets;
    portENTER_CRITICAL(&frameMutex);
    samples=sampleCount; packets=packetCount; blePackets=blePacketCount;
    portEXIT_CRITICAL(&frameMutex);
    Serial.printf("motion sample_hz=%.1f osc_hz=%.1f wifi=%d heap=%u\n",
        (samples-previousSamples)*1000.0f/elapsed,
        (packets-previousPackets)*1000.0f/elapsed,connected,ESP.getFreeHeap());
    lastReport=now; previousSamples=samples; previousPackets=packets;
    const auto status=ble.status();
    Serial.printf("ble_hz=%.1f connected=%d subscribed=%d mtu=%u\n",
        (blePackets-previousBlePackets)*1000.0f/elapsed,status.connected,status.subscribed,status.mtu);
    previousBlePackets=blePackets;
  }
  if(restartAtMs && static_cast<int32_t>(millis()-restartAtMs)>=0) ESP.restart();
  delay(1);
}
