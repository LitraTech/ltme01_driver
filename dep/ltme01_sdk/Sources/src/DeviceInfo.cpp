#include "ltme01_sdk/DeviceInfo.h"

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/lan/LanLocation.h"

ltme01_sdk::DeviceInfo::DeviceInfo(const ltme01_sdk::Location& location)
  : location_(location.clone())
{
}

ltme01_sdk::DeviceInfo::DeviceInfo(const ltme01_sdk::DeviceInfo& other)
  : location_(other.location_->clone())
{
}

ltme01_sdk::DeviceInfo& ltme01_sdk::DeviceInfo::operator=(const ltme01_sdk::DeviceInfo& other)
{
  if (&other != this)
    location_ = other.location_->clone();
  return *this;
}

bool ltme01_sdk::DeviceInfo::operator==(const ltme01_sdk::DeviceInfo& other) const
{
  return *location_ == *other.location_;
}

ltme01_sdk::DeviceType ltme01_sdk::DeviceInfo::type() const
{
  const Location& location = *location_;

  if (typeid(location) == typeid(UsbLocation))
    return DEVICE_TYPE_USB;
  else
    return DEVICE_TYPE_LAN;
}

const ltme01_sdk::Location& ltme01_sdk::DeviceInfo::location() const
{
  return *location_;
}
