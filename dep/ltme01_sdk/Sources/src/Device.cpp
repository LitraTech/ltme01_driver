#include "ltme01_sdk/Device.h"

#include "ltme01_sdk/Transport.h"
#include "ltme01_sdk/ControlPacket.h"

#include <ctime>
#include <cstring>

ltme01_sdk::Device::Device(const Location& location)
  : transport_(Transport::createInstance(location))
{
  srand((unsigned int)time(NULL));
  reference_ = (uint16_t)rand();
}

ltme01_sdk::Device::~Device()
{
  close();
}

const ltme01_sdk::Location& ltme01_sdk::Device::location() const
{
  return transport_->location();
}

int ltme01_sdk::Device::open()
{
  return transport_->open();
}

void ltme01_sdk::Device::close()
{
  transport_->close();
}

int ltme01_sdk::Device::readDataPacket(DataPacket& dataPacket)
{
  return readDataPacket(dataPacket, 0);
}

int ltme01_sdk::Device::readDataPacket(DataPacket& dataPacket, unsigned int timeout)
{
  return transport_->doDataTransaction(dataPacket, timeout);
}
