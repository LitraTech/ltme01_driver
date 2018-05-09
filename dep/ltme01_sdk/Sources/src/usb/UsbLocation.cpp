#include "ltme01_sdk/usb/UsbLocation.h"

ltme01_sdk::UsbLocation::UsbLocation(uint8_t busNumber, uint8_t deviceAddress)
  : busNumber_(busNumber)
  , deviceAddress_(deviceAddress)
{
}

ltme01_sdk::UsbLocation::UsbLocation(const ltme01_sdk::UsbLocation& other)
{
  this->busNumber_ = other.busNumber_;
  this->deviceAddress_ = other.deviceAddress_;
}

ltme01_sdk::UsbLocation& ltme01_sdk::UsbLocation::operator=(const ltme01_sdk::UsbLocation& other)
{
  this->busNumber_ = other.busNumber_;
  this->deviceAddress_ = other.deviceAddress_;

  return *this;
}

bool ltme01_sdk::UsbLocation::operator==(const ltme01_sdk::UsbLocation& other) const
{
  return ((this->busNumber_ == other.busNumber_) && (this->deviceAddress_ == other.deviceAddress_));
}

uint8_t ltme01_sdk::UsbLocation::busNumber() const
{
  return busNumber_;
}

uint8_t ltme01_sdk::UsbLocation::deviceAddress() const
{
  return deviceAddress_;
}
