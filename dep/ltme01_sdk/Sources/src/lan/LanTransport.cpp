#include "ltme01_sdk/lan/LanTransport.h"

#include "ltme01_sdk/Device.h"

ltme01_sdk::LanTransport::LanTransport(const ltme01_sdk::LanLocation& location)
  : location_(location)
  , dataChannelSocket_(dataTransactionIoService_)
  , ctrlChannelSocket_(ctrlTransactionIoService_)
  , dataTransactionTimer_(dataTransactionIoService_)
  , ctrlTransactionTimer_(ctrlTransactionIoService_)
{
}

const ltme01_sdk::Location& ltme01_sdk::LanTransport::location() const
{
  return location_;
}

int ltme01_sdk::LanTransport::open()
{
  dataChannelSocket_.open(asio::ip::udp::v4());
  ctrlChannelSocket_.open(asio::ip::udp::v4());

  asio::ip::udp::endpoint localEndpoint(asio::ip::udp::v4(), ntohs(location_.dataChannelPort()));
  asio::error_code error;
  dataChannelSocket_.bind(localEndpoint, error);

  if (!error)
    return RESULT_SUCCESS;
  else {
    close();
    return RESULT_ACCESS_DENIED;
  }
}

void ltme01_sdk::LanTransport::close()
{
  dataChannelSocket_.close();
  ctrlChannelSocket_.close();
}

int ltme01_sdk::LanTransport::doDataTransaction(ltme01_sdk::DataPacket& dataPacket, unsigned int timeout)
{
  timeout = (timeout == 0) ? DEFAULT_DATA_TRANSACTION_TIMEOUT : timeout;

  asio::error_code timerResult, transactionResult;
  timerResult = transactionResult = asio::error::would_block;

  dataTransactionTimer_.expires_after(std::chrono::milliseconds(timeout));
  dataTransactionTimer_.async_wait([&](const std::error_code& error) {
    timerResult = error;
  });

  auto buffer = asio::buffer(dataPacket.data(), DataPacket::MAX_DATA_PACKET_SIZE);
  asio::ip::udp::endpoint peerEndpoint;
  std::function<void(const asio::error_code&, std::size_t)> handler;
  handler = [&](const asio::error_code& error, std::size_t bytesTransferred) {
    if (!error && peerEndpoint.address().to_v4().to_ulong() != ntohl(location_.deviceAddress()))
      dataChannelSocket_.async_receive_from(buffer, peerEndpoint, handler);
    else
      transactionResult = error;
  };
  dataChannelSocket_.async_receive_from(buffer, peerEndpoint, handler);

  dataTransactionIoService_.restart();
  while (dataTransactionIoService_.run_one()) {
    if (transactionResult != asio::error::would_block)
      dataTransactionTimer_.cancel();
    if (timerResult != asio::error::would_block)
      dataChannelSocket_.cancel();
  }

  if (!timerResult)
    return RESULT_TIMEOUT;
  else if (!transactionResult)
    return RESULT_SUCCESS;
  else
    return RESULT_UNKNOWN_ERROR;
}

int ltme01_sdk::LanTransport::doCtrlTransaction(ltme01_sdk::RequestPacket& requestPacket, ltme01_sdk::ResponsePacket& responsePacket, unsigned int timeout)
{
  timeout = (timeout == 0) ? DEFAULT_CTRL_TRANSACTION_TIMEOUT : timeout;

  asio::error_code timerResult, transactionResult;
  timerResult = transactionResult = asio::error::would_block;

  auto writeBuffer = asio::buffer(requestPacket.data(), requestPacket.length());
  auto readBuffer = asio::buffer(responsePacket.data(), ResponsePacket::MAX_RESPONSE_PACKET_SIZE);
  asio::ip::udp::endpoint deviceEndpoint(
    asio::ip::address_v4(ntohl(location_.deviceAddress())), ntohs(LanLocation::CTRL_CHANNEL_PORT));

  unsigned int retryTimeout = DEFAULT_CTRL_TRANSACTION_RETRY_TIMEOUT;
  ctrlTransactionTimer_.expires_after(std::chrono::milliseconds(retryTimeout));
  ctrlTransactionTimer_.async_wait([&](const std::error_code& error) {
    if (!error && timeout > retryTimeout) {
      ctrlChannelSocket_.async_send_to(
        writeBuffer, deviceEndpoint, [&](const asio::error_code& error, std::size_t bytesTransferred) {
          if (error)
            transactionResult = error;
        });
      ctrlTransactionTimer_.expires_after(std::chrono::milliseconds(timeout - retryTimeout));
      ctrlTransactionTimer_.async_wait([&](const std::error_code& error) { timerResult = error; });
    }
    else
      timerResult = error;
  });

  ctrlChannelSocket_.async_send_to(
    writeBuffer, deviceEndpoint, [&](const asio::error_code& error, std::size_t bytesTransferred) {
      if (!error)
        ctrlChannelSocket_.async_receive_from(readBuffer, deviceEndpoint,
          [&](const asio::error_code& error, std::size_t bytesTransferred) { transactionResult = error; });
      else
        transactionResult = error;
    });

  ctrlTransactionIoService_.restart();
  while (ctrlTransactionIoService_.run_one()) {
    if (transactionResult != asio::error::would_block)
      ctrlTransactionTimer_.cancel();
    if (timerResult != asio::error::would_block)
      ctrlChannelSocket_.cancel();
  }

  if (!timerResult)
    return RESULT_TIMEOUT;
  else if (!transactionResult)
    return RESULT_SUCCESS;
  else
    return RESULT_UNKNOWN_ERROR;
}
