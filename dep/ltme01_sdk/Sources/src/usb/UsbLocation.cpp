#include "ltme01_sdk/usb/UsbLocation.h"

#include <sstream>
#include <iomanip>

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

std::unique_ptr<ltme01_sdk::Location> ltme01_sdk::UsbLocation::clone() const
{
  return std::unique_ptr<Location>(new UsbLocation(*this));
}

bool ltme01_sdk::UsbLocation::equals(const ltme01_sdk::Location& other) const
{
  if (typeid(other) != typeid(UsbLocation))
    return false;
  return *this == dynamic_cast<const UsbLocation&>(other);
}

std::string ltme01_sdk::UsbLocation::label() const
{
  std::ostringstream stream;
  stream << std::setfill('0');
  stream << "USB " << std::setw(3) << (int)busNumber_ << ":" << std::setw(3) << (int)deviceAddress_;
  return stream.str();
}

uint8_t ltme01_sdk::UsbLocation::busNumber() const
{
  return busNumber_;
}

uint8_t ltme01_sdk::UsbLocation::deviceAddress() const
{
  return deviceAddress_;
}
