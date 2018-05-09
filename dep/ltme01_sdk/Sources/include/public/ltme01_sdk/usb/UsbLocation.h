#ifndef USB_LOCATION_H
#define USB_LOCATION_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/Location.h"

#include <cstdint>

namespace ltme01_sdk
{

class LTME01_SDK_API UsbLocation : public Location
{
public:
  static const uint16_t TARGET_VID = 0x16D0;
  static const uint16_t TARGET_PID = 0x0DB7;

public:
  UsbLocation(uint8_t busNumber, uint8_t deviceAddress);
  UsbLocation(const UsbLocation& other);

  UsbLocation& operator=(const UsbLocation& other);
  bool operator==(const UsbLocation& other) const;

  uint8_t busNumber() const;
  uint8_t deviceAddress() const;

private:
  uint8_t busNumber_;
  uint8_t deviceAddress_;
};

}

#endif
