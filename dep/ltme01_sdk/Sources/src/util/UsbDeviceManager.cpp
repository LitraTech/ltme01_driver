#include "ltme01_sdk/util/UsbDeviceManager.h"

#include "ltme01_sdk/util/UsbHotplugNotifier.h"

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <iterator>

ltme01_sdk::UsbDeviceManager::UsbDeviceManager()
  : hotplugNotifier_(UsbHotplugNotifier::createInstance())
  , notifierStarted_(false)
{
  libusb_init(NULL);
}

ltme01_sdk::UsbDeviceManager::~UsbDeviceManager()
{
  if (notifierStarted_)
    hotplugNotifier_->stopEventMonitor();
  libusb_exit(NULL);
}

bool ltme01_sdk::UsbDeviceManager::startDeviceMonitor(bool enumerate)
{
  notifierStarted_ = true;
  hotplugNotifier_->startEventMonitor(
    std::bind(&UsbDeviceManager::rescanDevices, this, std::placeholders::_1, true), enumerate);
  return true;
}

void ltme01_sdk::UsbDeviceManager::stopDeviceMonitor()
{
  hotplugNotifier_->stopEventMonitor();
  notifierStarted_ = false;
}

std::vector<ltme01_sdk::UsbLocation> ltme01_sdk::UsbDeviceManager::listConnectedDevices()
{
  if (!notifierStarted_)
    rescanDevices(true, false);

  mutex_.lock();
  std::vector<UsbLocation> devices(cachedDevices_);
  mutex_.unlock();
  return devices;
}

void ltme01_sdk::UsbDeviceManager::registerHotplugEventCallback(HotplugEventCallback callback)
{
  this->hotplugEventCallback_ = callback;
}

void ltme01_sdk::UsbDeviceManager::rescanDevices(bool clearCache, bool invokeCallbacks)
{
  if (clearCache)
    cachedDevices_.clear();

  std::vector<UsbLocation> currentDevices;

  libusb_device** usbDeviceList = NULL;
  int count = libusb_get_device_list(NULL, &usbDeviceList);
  if (count > 0) {
    for (int i = 0; i < count; i++) {
      libusb_device* usbDevice = usbDeviceList[i];
      libusb_device_descriptor deviceDescriptor;
      int result = libusb_get_device_descriptor(usbDevice, &deviceDescriptor);
      if (result != LIBUSB_SUCCESS)
        continue;
      if (deviceDescriptor.idVendor == UsbLocation::TARGET_VID
        && deviceDescriptor.idProduct == UsbLocation::TARGET_PID) {
        uint8_t busNumber = libusb_get_bus_number(usbDevice);
        uint8_t deviceAddress = libusb_get_device_address(usbDevice);
        currentDevices.push_back(UsbLocation(busNumber, deviceAddress));
      }
    }

    libusb_free_device_list(usbDeviceList, 1);
  }

  std::vector<UsbLocation> remainingDevices, detachedDevices, attachedDevices;
  std::copy_if(cachedDevices_.begin(), cachedDevices_.end(), std::back_inserter(remainingDevices),
    [&currentDevices](const UsbLocation& cached) {
      return (std::find(currentDevices.begin(), currentDevices.end(), cached) != currentDevices.end());
    });
  std::copy_if(cachedDevices_.begin(), cachedDevices_.end(), std::back_inserter(detachedDevices),
    [&currentDevices](const UsbLocation& cached) {
      return (std::find(currentDevices.begin(), currentDevices.end(), cached) == currentDevices.end());
    });
  std::copy_if(currentDevices.begin(), currentDevices.end(), std::back_inserter(attachedDevices),
    [this](const UsbLocation& current) {
      return (std::find(cachedDevices_.begin(), cachedDevices_.end(), current) == cachedDevices_.end());
    });

  mutex_.lock();
  cachedDevices_.clear();
  cachedDevices_.insert(cachedDevices_.end(), remainingDevices.begin(), remainingDevices.end());
  cachedDevices_.insert(cachedDevices_.end(), attachedDevices.begin(), attachedDevices.end());
  mutex_.unlock();

  if (invokeCallbacks && hotplugEventCallback_) {
    for (const UsbLocation& location : detachedDevices)
      hotplugEventCallback_(EVENT_DEVICE_DETACHED, location);
    for (const UsbLocation& location : attachedDevices)
      hotplugEventCallback_(EVENT_DEVICE_ATTACHED, location);
  }
}
