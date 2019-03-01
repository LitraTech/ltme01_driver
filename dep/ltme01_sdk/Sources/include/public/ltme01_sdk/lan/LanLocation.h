#ifndef LAN_LOCATION_H
#define LAN_LOCATION_H

#include "ltme01_sdk/Common.h"

#include "ltme01_sdk/Location.h"

#ifdef __linux__
#include <netinet/in.h>
#elif _WIN32
#include <WinSock2.h>
typedef ULONG in_addr_t;
typedef USHORT in_port_t;
#endif

#include <cstdint>

namespace ltme01_sdk
{

class LTME01_SDK_API LanLocation : public Location
{
public:
  static const in_addr_t DEFAULT_DEVICE_ADDRESS;
  static const in_port_t DEFAULT_DATA_CHANNEL_PORT;
  static const in_port_t CTRL_CHANNEL_PORT;

public:
  LanLocation(in_addr_t deviceAddress = DEFAULT_DEVICE_ADDRESS,
              in_port_t dataChannelPort = DEFAULT_DATA_CHANNEL_PORT);
  LanLocation(const LanLocation& other);

  LanLocation& operator=(const LanLocation& other);
  bool operator==(const LanLocation& other) const;

  virtual std::unique_ptr<Location> clone() const;
  virtual bool equals(const Location& other) const;

  virtual std::string label() const;

  in_addr_t deviceAddress() const;
  in_port_t dataChannelPort() const;
  in_port_t ctrlChannelPort() const;

private:
  in_addr_t deviceAddress_;
  in_port_t dataChannelPort_;
};

}

#endif
