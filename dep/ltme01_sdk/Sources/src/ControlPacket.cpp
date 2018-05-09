#include "ltme01_sdk/ControlPacket.h"

#include <cstring>

ltme01_sdk::RequestPacket::RequestPacket()
  : header_((RequestPacketHeader*)data_)
{
  header_->signature = REQUEST_PACKET_SIGNATURE;
  header_->reference = 0;
  header_->request = 0;
  header_->value = 0;
  header_->length = sizeof(RequestPacketHeader);
}

uint8_t* ltme01_sdk::RequestPacket::data()
{
  return data_;
}

uint16_t ltme01_sdk::RequestPacket::reference() const
{
  return header_->reference;
}

uint16_t ltme01_sdk::RequestPacket::request() const
{
  return header_->request;
}

uint16_t ltme01_sdk::RequestPacket::value() const
{
  return header_->value;
}

uint16_t ltme01_sdk::RequestPacket::length() const
{
  return header_->length;
}

uint8_t* ltme01_sdk::RequestPacket::payload()
{
  return data_ + sizeof(RequestPacketHeader);
}

void ltme01_sdk::RequestPacket::updateChecksum()
{
  header_->checksum = header_->signature + header_->reference + header_->request + header_->value + header_->length;
  int count = header_->length - sizeof(RequestPacketHeader);
  for (int i = 0; i < (count + count % 2) / 2; i++)
    header_->checksum += *((uint16_t*)(data_ + sizeof(RequestPacketHeader)) + i);
}

ltme01_sdk::GenericRequestPacket::GenericRequestPacket(uint16_t request)
{
  header_->request = request;
}

void ltme01_sdk::GenericRequestPacket::setReference(uint16_t reference)
{
  header_->reference = reference;
}

void ltme01_sdk::GenericRequestPacket::setValue(uint16_t value)
{
  header_->value = value;
}

void ltme01_sdk::GenericRequestPacket::setLength(uint16_t length)
{
  header_->length = length;
}

ltme01_sdk::ResponsePacket::ResponsePacket()
  : header_((ResponsePacketHeader*)data_)
{
}

ltme01_sdk::ResponsePacket::ResponsePacket(uint8_t* buffer, int length)
{
  int count = (length <= MAX_RESPONSE_PACKET_SIZE) ? length : MAX_RESPONSE_PACKET_SIZE;
  memcpy(data_, buffer, count);
}

uint8_t* ltme01_sdk::ResponsePacket::data()
{
  return data_;
}

uint16_t ltme01_sdk::ResponsePacket::reference() const
{
  return header_->reference;
}

uint16_t ltme01_sdk::ResponsePacket::result() const
{
  return header_->result;
}

uint16_t ltme01_sdk::ResponsePacket::value() const
{
  return header_->value;
}

uint16_t ltme01_sdk::ResponsePacket::length() const
{
  return header_->length;
}

uint8_t* ltme01_sdk::ResponsePacket::payload()
{
  return data_ + sizeof(ResponsePacketHeader);
}

bool ltme01_sdk::ResponsePacket::isValid() const
{
  if (header_->signature != RESPONSE_PACKET_SIGNATURE)
    return false;

  uint16_t checksum = 0;
  checksum += header_->signature + header_->reference + header_->result + header_->value + header_->length;
  int count = header_->length - sizeof(ResponsePacketHeader);
  for (int i = 0; i < (count + count % 2) / 2; i++)
    checksum += *((uint16_t*)(data_ + sizeof(ResponsePacketHeader)) + i);

  if (checksum == header_->checksum)
    return true;
  else
    return false;
}

ltme01_sdk::GenericResponsePacket::GenericResponsePacket()
{
}

ltme01_sdk::GenericResponsePacket::GenericResponsePacket(uint8_t* buffer, int length)
  : ResponsePacket(buffer, length)
{
}
