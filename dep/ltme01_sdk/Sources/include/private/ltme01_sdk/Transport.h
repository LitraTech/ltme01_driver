#ifndef TRANSPORT_H
#define TRANSPORT_H

#include "ltme01_sdk/Location.h"
#include "ltme01_sdk/DataPacket.h"
#include "ltme01_sdk/ControlPacket.h"

namespace ltme01_sdk
{

class Transport
{
public:
  static Transport* createInstance(const Location& location);

public:
  virtual ~Transport() = default;

  virtual const Location& location() const = 0;

  virtual int open() = 0;
  virtual void close() = 0;

  virtual int doDataTransaction(DataPacket& dataPacket, unsigned int timeout) = 0;
  virtual int doCtrlTransaction(RequestPacket& requestPacket, ResponsePacket& responsePacket, unsigned int timeout) = 0;
};

}

#endif
