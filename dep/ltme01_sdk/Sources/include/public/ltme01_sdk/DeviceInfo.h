#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/Location.h"

#include <memory>

namespace ltme01_sdk
{

enum DeviceType
{
  DEVICE_TYPE_USB = 0x01 << 0,
  DEVICE_TYPE_LAN = 0x01 << 1
};

class LTME01_SDK_API DeviceInfo
{
public:
  DeviceInfo(const Location& location);
  DeviceInfo(const DeviceInfo& other);

  DeviceInfo& operator=(const DeviceInfo& other);
  bool operator==(const DeviceInfo& other) const;

  DeviceType type() const;
  const Location& location() const;

private:
  std::unique_ptr<Location> location_;
};

}

#endif
