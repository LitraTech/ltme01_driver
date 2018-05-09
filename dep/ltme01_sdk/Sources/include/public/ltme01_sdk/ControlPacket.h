#ifndef CONTROL_PACKET_H
#define CONTROL_PACKET_H

#include "ltme01_sdk/Common.h"

#include <cstdint>

namespace ltme01_sdk
{

#pragma pack(push, 1)
typedef struct
{
  uint16_t signature;
  uint16_t reference;
  uint16_t request;
  uint16_t value;
  uint16_t length;
  uint16_t checksum;
} RequestPacketHeader;
#pragma pack(pop)

class LTME01_SDK_API RequestPacket
{
public:
  static const int MAX_REQUEST_PACKET_SIZE = 128;

  static const uint16_t REQUEST_PACKET_SIGNATURE = 0xFFFA;

public:
  RequestPacket();
  virtual ~RequestPacket() = 0;

  uint8_t* data();

  uint16_t reference() const;
  uint16_t request() const;
  uint16_t value() const;
  uint16_t length() const;

  uint8_t* payload();

  void updateChecksum();

protected:
  RequestPacketHeader* header_;
  uint8_t data_[MAX_REQUEST_PACKET_SIZE];
};

inline RequestPacket::~RequestPacket() = default;

class LTME01_SDK_API GenericRequestPacket : public RequestPacket
{
public:
  GenericRequestPacket(uint16_t request);

  void setReference(uint16_t reference);
  void setValue(uint16_t value);
  void setLength(uint16_t length);
};

#pragma pack(push, 1)
typedef struct
{
  uint16_t signature;
  uint16_t reference;
  uint16_t result;
  uint16_t value;
  uint16_t length;
  uint16_t checksum;
} ResponsePacketHeader;
#pragma pack(pop)

class LTME01_SDK_API ResponsePacket
{
public:
  static const int MAX_RESPONSE_PACKET_SIZE = 128;

  static const uint16_t RESPONSE_PACKET_SIGNATURE = 0xFFFB;

  static const uint16_t RESULT_SUCCESS = 0x0000;
  static const uint16_t RESULT_CORRUPTED_PACKET = 0x0001;
  static const uint16_t RESULT_UNSUPPORTED_REQUEST = 0x0002;
  static const uint16_t RESULT_INVALID_VALUE = 0x0003;
  static const uint16_t RESULT_OPERATION_FAILED = 0x0004;

public:
  ResponsePacket();
  ResponsePacket(uint8_t* buffer, int length);
  virtual ~ResponsePacket() = 0;

  uint8_t* data();

  uint16_t reference() const;
  uint16_t result() const;
  uint16_t value() const;
  uint16_t length() const;

  uint8_t* payload();

  bool isValid() const;

protected:
  ResponsePacketHeader* header_;
  uint8_t data_[MAX_RESPONSE_PACKET_SIZE];
};

inline ResponsePacket::~ResponsePacket() = default;

class LTME01_SDK_API GenericResponsePacket : public ResponsePacket
{
public:
  GenericResponsePacket();
  GenericResponsePacket(uint8_t* buffer, int length);
};

}

#endif
