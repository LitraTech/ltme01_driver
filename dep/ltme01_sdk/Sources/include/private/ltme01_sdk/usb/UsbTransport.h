#ifndef USB_TRANSPORT_H
#define USB_TRANSPORT_H

#include "ltme01_sdk/Transport.h"

#include "ltme01_sdk/usb/UsbLocation.h"

#include <libusb-1.0/libusb.h>

#include <atomic>
#include <thread>

namespace ltme01_sdk
{

class UsbTransport : public Transport
{
public:
  UsbTransport(const UsbLocation& location);
  ~UsbTransport();

  const Location& location() const;

  int open();
  void close();

  int doDataTransaction(DataPacket& dataPacket, unsigned int timeout);
  int doCtrlTransaction(RequestPacket& requestPacket, ResponsePacket& responsePacket, unsigned int timeout);

private:
  static const int DEFAULT_DATA_TRANSACTION_TIMEOUT = 200;
  static const int DEFAULT_CTRL_TRANSACTION_TIMEOUT = 200;

  static const uint8_t DATA_IN_EP = 0x81;
  static const uint8_t CTRL_OUT_EP = 0x02;
  static const uint8_t CTRL_IN_EP = 0x82;

private:
  void transferEventHandler();

  static void LIBUSB_CALL dataEndpointRxCallback(struct libusb_transfer* transfer);
  static void LIBUSB_CALL ctrlEndpointTxCallback(struct libusb_transfer* transfer);
  static void LIBUSB_CALL ctrlEndpointRxCallback(struct libusb_transfer* transfer);

private:
  UsbLocation location_;

  libusb_context* usbContext_;
  libusb_device_handle* usbDeviceHandle_;

  std::atomic<bool> handleTransferEvents_;
  std::thread thread_;

  std::atomic<int> outstandingTransfers_;
};

}

#endif
