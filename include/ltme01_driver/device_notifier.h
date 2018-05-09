#ifndef DEVICE_NOTIFIER_H
#define DEVICE_NOTIFIER_H

#include <thread>

enum DeviceEvent
{
  EVENT_DEVICE_NODE_CREATED,
  EVENT_DEVICE_NODE_DELETED
};
typedef std::function<void(DeviceEvent)> DeviceEventCallback;

class DeviceNotifier
{
public:
  DeviceNotifier();
  ~DeviceNotifier();

  void registerDeviceEventCallback(DeviceEventCallback callback);
  void start(const std::string& deviceNode);
  void stop();

private:
  std::string deviceNodeDir_;
  std::string deviceNodeName_;

  DeviceEventCallback deviceEventCallback_;

  std::thread thread_;
  bool notifierStarted_;
  bool runEventLoop_;
};

#endif
