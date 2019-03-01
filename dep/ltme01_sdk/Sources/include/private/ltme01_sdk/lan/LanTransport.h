#ifndef LAN_TRANSPORT_H
#define LAN_TRANSPORT_H

#include "ltme01_sdk/Transport.h"

#include "ltme01_sdk/lan/LanLocation.h"

#include <asio.hpp>

namespace ltme01_sdk
{

class LanTransport : public Transport
{
public:
  LanTransport(const LanLocation& location);

  const Location& location() const;

  int open();
  void close();

  int doDataTransaction(DataPacket& dataPacket, unsigned int timeout);
  int doCtrlTransaction(RequestPacket& requestPacket, ResponsePacket& responsePacket, unsigned int timeout);

private:
  static const int DEFAULT_DATA_TRANSACTION_TIMEOUT = 3000;
  static const int DEFAULT_CTRL_TRANSACTION_TIMEOUT = 3000;
  static const int DEFAULT_CTRL_TRANSACTION_RETRY_TIMEOUT = 300;

private:
  LanLocation location_;

  asio::io_service dataTransactionIoService_;
  asio::io_service ctrlTransactionIoService_;
  asio::ip::udp::socket dataChannelSocket_;
  asio::ip::udp::socket ctrlChannelSocket_;
  asio::basic_waitable_timer<std::chrono::system_clock> dataTransactionTimer_;
  asio::basic_waitable_timer<std::chrono::system_clock> ctrlTransactionTimer_;
};

}

#endif
