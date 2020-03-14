#ifndef DEVICE_H
#define DEVICE_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/DeviceInfo.h"
#include "ltme01_sdk/Location.h"
#include "ltme01_sdk/DataPacket.h"

#include <memory>
#include <atomic>

namespace ltme01_sdk
{

enum
{
  RESULT_SUCCESS = 0,
  RESULT_DEVICE_DISCONNECTED,
  RESULT_ACCESS_DENIED,
  RESULT_TIMEOUT,
  RESULT_ABORTED,
  RESULT_UNKNOWN_ERROR
};

class Transport;

class LTME01_SDK_API Device
{
public:
  Device(const DeviceInfo& deviceInfo);
  Device(const Location& location);
  virtual ~Device();

  const Location& location() const;

  int open();
  void close();

  int readDataPacket(DataPacket& dataPacket);
  int readDataPacket(DataPacket& dataPacket, unsigned int timeout);

  bool checkConnectivity();
  bool getSerial(std::string& serial);
  bool getTimestamp(uint32_t& timestamp);
  bool resetTimestamp();
  bool enterLowPowerMode();
  bool exitLowPowerMode();

protected:
  std::unique_ptr<Transport> transport_;
  std::atomic<uint16_t> reference_;

private:
  static const int SERIAL_LENGTH = 20;
};

}

#endif
