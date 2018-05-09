#ifndef USB_HOTPLUG_NOTIFIER_H
#define USB_HOTPLUG_NOTIFIER_H

#include <functional>

namespace ltme01_sdk
{

class UsbHotplugNotifier
{
public:
  static UsbHotplugNotifier* createInstance();

public:
  virtual bool startEventMonitor(std::function<void(bool)> recipient, bool enumerate) = 0;
  virtual void stopEventMonitor() = 0;
};

}

#endif
