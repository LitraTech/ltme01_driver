#ifndef USB_DEVICE_MANAGER_H
#define USB_DEVICE_MANAGER_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/usb/UsbLocation.h"

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace ltme01_sdk
{

enum UsbHotplugEvent
{
  EVENT_DEVICE_ATTACHED,
  EVENT_DEVICE_DETACHED
};
typedef std::function<void(UsbHotplugEvent, UsbLocation)> HotplugEventCallback;

class UsbHotplugNotifier;

class LTME01_SDK_API UsbDeviceManager
{
public:
  UsbDeviceManager();
  ~UsbDeviceManager();

  bool startDeviceMonitor(bool enumerate);
  void stopDeviceMonitor();
  std::vector<UsbLocation> listConnectedDevices();
  void registerHotplugEventCallback(HotplugEventCallback callback);

private:
  void rescanDevices(bool clearCache, bool invokeCallbacks);

private:
  std::unique_ptr<UsbHotplugNotifier> hotplugNotifier_;
  bool notifierStarted_;

  std::mutex mutex_;
  std::vector<UsbLocation> cachedDevices_;

  HotplugEventCallback hotplugEventCallback_;
};

}

#endif
