#include "ltme01_sdk/lan/LanLocation.h"

#include <asio.hpp>

#if defined(__linux__) && defined(htons)
#undef htons
#endif

const in_addr_t ltme01_sdk::LanLocation::DEFAULT_DEVICE_ADDRESS
  = { inet_addr("192.168.10.160") };
const in_port_t ltme01_sdk::LanLocation::DEFAULT_DATA_CHANNEL_PORT = htons(8100);
const in_port_t ltme01_sdk::LanLocation::CTRL_CHANNEL_PORT = htons(8200);

ltme01_sdk::LanLocation::LanLocation(in_addr_t deviceAddress, in_port_t dataChannelPort)
  : deviceAddress_(deviceAddress)
  , dataChannelPort_(dataChannelPort)
{
}

ltme01_sdk::LanLocation::LanLocation(const ltme01_sdk::LanLocation& other)
{
  this->deviceAddress_ = other.deviceAddress_;
  this->dataChannelPort_ = other.dataChannelPort_;
}

ltme01_sdk::LanLocation& ltme01_sdk::LanLocation::operator=(const ltme01_sdk::LanLocation& other)
{
  if (this != &other) {
    this->deviceAddress_ = other.deviceAddress_;
    this->dataChannelPort_ = other.dataChannelPort_;
  }

  return *this;
}

bool ltme01_sdk::LanLocation::operator==(const ltme01_sdk::LanLocation& other) const
{
  return ((this->deviceAddress_ == other.deviceAddress_) &&
          (this->dataChannelPort_ == other.dataChannelPort_));
}

std::unique_ptr<ltme01_sdk::Location> ltme01_sdk::LanLocation::clone() const
{
  return std::unique_ptr<Location>(new LanLocation(*this));
}

bool ltme01_sdk::LanLocation::equals(const ltme01_sdk::Location& other) const
{
  if (typeid(other) != typeid(LanLocation))
    return false;
  return *this == dynamic_cast<const LanLocation&>(other);
}

std::string ltme01_sdk::LanLocation::label() const
{
  return "LAN " + asio::ip::address_v4(ntohl(deviceAddress_)).to_string() + ":" +
      std::to_string(ntohs(dataChannelPort_));
}

in_addr_t ltme01_sdk::LanLocation::deviceAddress() const
{
  return deviceAddress_;
}

in_port_t ltme01_sdk::LanLocation::dataChannelPort() const
{
  return dataChannelPort_;
}

in_port_t ltme01_sdk::LanLocation::ctrlChannelPort() const
{
  return CTRL_CHANNEL_PORT;
}
