#include "ltme01_sdk/DataPacket.h"

#include <cstring>

ltme01_sdk::DataPacket::DataPacket()
{
  header_ = (DataPacketHeader*)data_;
}

ltme01_sdk::DataPacket::DataPacket(uint8_t* buffer, int length)
  : DataPacket()
{
  fillIn(buffer, length);
}

void ltme01_sdk::DataPacket::fillIn(uint8_t* buffer, int length)
{
  length = (length <= MAX_DATA_PACKET_SIZE) ? length : MAX_DATA_PACKET_SIZE;
  memcpy(data_, buffer, length);
}

bool ltme01_sdk::DataPacket::isValid() const
{
  if (header_->signature != DATA_PACKET_SIGNATURE)
    return false;
  if (header_->index > DATA_PACKET_INDEX_MAX)
    return false;
  if (header_->count > DATA_PACKET_COUNT_MAX)
    return false;

  uint16_t checksum = 0;
  checksum += header_->signature + ((header_->count << 8) | header_->index) + header_->reserved1
    + (header_->reserved2 & 0xFFFF) + (header_->reserved2 >> 16);
  for (int i = 0; i < header_->count; i++)
    checksum += ((uint16_t*)(data_ + sizeof(DataPacketHeader)))[i];

  if (checksum == header_->checksum)
    return true;
  else
    return false;
}

uint8_t ltme01_sdk::DataPacket::index() const
{
  return header_->index;
}

uint8_t ltme01_sdk::DataPacket::count() const
{
  return header_->count;
}

uint16_t ltme01_sdk::DataPacket::range(int i) const
{
  return ((uint16_t*)(data_ + sizeof(DataPacketHeader)))[i];
}

uint8_t* ltme01_sdk::DataPacket::data()
{
  return data_;
}
