#ifndef DEVICE_NOTIFIER_H
#define DEVICE_NOTIFIER_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/DeviceInfo.h"

#include <functional>
#include <thread>
#include <vector>
#include <memory>

namespace ltme01_sdk
{

enum DeviceEvent
{
  DEVICE_EVENT_ATTACH,
  DEVICE_EVENT_DETACH
};
typedef std::function<void(DeviceEvent, DeviceInfo)> DeviceEventCallback;

class UsbDeviceScanner;
class LanDeviceScanner;

class LTME01_SDK_API DeviceNotifier
{
public:
  DeviceNotifier();
  ~DeviceNotifier();

  void registerCallback(DeviceEventCallback callback);
  void start();
  void stop();

private:
  std::thread thread_;
  bool activeFlag_;

  std::unique_ptr<UsbDeviceScanner> usbDeviceScanner_;
  std::unique_ptr<LanDeviceScanner> lanDeviceScanner_;

  std::vector<DeviceInfo> devices_;

  DeviceEventCallback callback_;
};

}

#endif
