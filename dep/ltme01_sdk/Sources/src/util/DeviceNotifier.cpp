#include "ltme01_sdk/util/DeviceNotifier.h"

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/lan/LanLocation.h"

#include "ltme01_sdk/ControlPacket.h"

#include <libusb-1.0/libusb.h>

#include <asio.hpp>

#include <algorithm>

namespace ltme01_sdk
{

class DeviceScanner
{
public:
  virtual ~DeviceScanner() = default;

  virtual std::vector<DeviceInfo> scanDevices() = 0;
};

class UsbDeviceScanner : public DeviceScanner
{
public:
  UsbDeviceScanner();
  ~UsbDeviceScanner();

  virtual std::vector<DeviceInfo> scanDevices();

private:
  libusb_context* usbContext_;
};

UsbDeviceScanner::UsbDeviceScanner()
{
  libusb_init(&usbContext_);
}

UsbDeviceScanner::~UsbDeviceScanner()
{
  libusb_exit(usbContext_);
}

std::vector<DeviceInfo> UsbDeviceScanner::scanDevices()
{
  std::vector<DeviceInfo> devices;

  libusb_device** usbDeviceList = NULL;
  int count = libusb_get_device_list(usbContext_, &usbDeviceList);
  if (count > 0) {
    for (int i = 0; i < count; i++) {
      libusb_device* usbDevice = usbDeviceList[i];
      libusb_device_descriptor deviceDescriptor;
      int result = libusb_get_device_descriptor(usbDevice, &deviceDescriptor);
      if (result != LIBUSB_SUCCESS)
        continue;
      if (deviceDescriptor.idVendor == UsbLocation::TARGET_VID
        && deviceDescriptor.idProduct == UsbLocation::TARGET_PID) {
        uint8_t busNumber = libusb_get_bus_number(usbDevice);
        uint8_t deviceAddress = libusb_get_device_address(usbDevice);
        devices.push_back(DeviceInfo(UsbLocation(busNumber, deviceAddress)));
      }
    }

    libusb_free_device_list(usbDeviceList, 1);
  }

  return devices;
}

class LanDeviceScanner : public DeviceScanner
{
public:
  LanDeviceScanner();
  ~LanDeviceScanner();

  virtual std::vector<DeviceInfo> scanDevices();

private:
  void responsePacketHandler(const asio::error_code& error, std::size_t bytesTransferred);

private:
  std::vector<DeviceInfo> devices_;

  ltme01_sdk::GenericRequestPacket requestPacket_;
  ltme01_sdk::GenericResponsePacket responsePacket_;

  asio::io_service ioService_;
  asio::ip::udp::socket socket_;
  asio::basic_waitable_timer<std::chrono::system_clock> timer_;
  asio::const_buffer writeBuffer_;
  asio::mutable_buffer readBuffer_;
  asio::ip::udp::endpoint peerEndpoint_;
};

LanDeviceScanner::LanDeviceScanner()
  : requestPacket_(GenericRequestPacket::REQUEST_DEVICE_DISCOVERY)
  , socket_(ioService_, asio::ip::udp::v4())
  , timer_(ioService_)
  , writeBuffer_(requestPacket_.data(), requestPacket_.length())
  , readBuffer_(responsePacket_.data(), ResponsePacket::MAX_RESPONSE_PACKET_SIZE)
{
  requestPacket_.setValue(1);
  requestPacket_.updateChecksum();
  socket_.set_option(asio::socket_base::broadcast(true));
}

LanDeviceScanner::~LanDeviceScanner()
{
  socket_.close();
}

std::vector<DeviceInfo> LanDeviceScanner::scanDevices()
{
  devices_.clear();

  asio::error_code timerResult = asio::error::would_block;
  timer_.expires_after(std::chrono::milliseconds(1000));
  timer_.async_wait([&](const std::error_code& error) {
    timerResult = error;
  });

  asio::ip::udp::endpoint broadcastEndpoint(asio::ip::address_v4::broadcast(), ntohs(LanLocation::CTRL_CHANNEL_PORT));
  socket_.async_send_to(writeBuffer_, broadcastEndpoint, [&](const asio::error_code& error, std::size_t bytesTransferred) {
    if (!error)
      socket_.async_receive_from(readBuffer_, peerEndpoint_,
        std::bind(&LanDeviceScanner::responsePacketHandler, this, std::placeholders::_1, std::placeholders::_2));
  });

  ioService_.reset();
  while (ioService_.run_one()) {
    if (timerResult != asio::error::would_block)
      socket_.cancel();
  }

  return devices_;
}

void LanDeviceScanner::responsePacketHandler(const asio::error_code& error, size_t bytesTransferred)
{
  if (!error) {
    if (responsePacket_.isValid()) {
      in_addr_t address = htonl(peerEndpoint_.address().to_v4().to_ulong());
      in_port_t port = htons(responsePacket_.value());
      devices_.push_back(DeviceInfo(LanLocation(address, port)));
    }

    auto buffer = asio::buffer(responsePacket_.data(), ResponsePacket::MAX_RESPONSE_PACKET_SIZE);
    socket_.async_receive_from(buffer, peerEndpoint_,
      std::bind(&LanDeviceScanner::responsePacketHandler, this, std::placeholders::_1, std::placeholders::_2));
  }
}

}

ltme01_sdk::DeviceNotifier::DeviceNotifier()
  : activeFlag_(false)
  , usbDeviceScanner_(new UsbDeviceScanner)
  , lanDeviceScanner_(new LanDeviceScanner)
{
}

ltme01_sdk::DeviceNotifier::~DeviceNotifier()
{
  if (activeFlag_)
    stop();
}

void ltme01_sdk::DeviceNotifier::registerCallback(ltme01_sdk::DeviceEventCallback callback)
{
  callback_ = callback;
}

void ltme01_sdk::DeviceNotifier::start()
{
  activeFlag_ = true;
  thread_ = std::thread([&]() {
    while (activeFlag_) {
      std::vector<DeviceInfo> currentDevices;
      std::vector<DeviceInfo> usbDevices = usbDeviceScanner_->scanDevices();
      currentDevices.insert(currentDevices.end(), usbDevices.begin(), usbDevices.end());
      std::vector<DeviceInfo> lanDevices = lanDeviceScanner_->scanDevices();
      currentDevices.insert(currentDevices.end(), lanDevices.begin(), lanDevices.end());

      std::vector<DeviceInfo> remainingDevices, detachedDevices, attachedDevices;
      std::copy_if(devices_.begin(), devices_.end(), std::back_inserter(remainingDevices),
        [&currentDevices](const DeviceInfo& device) {
          return (std::find(currentDevices.begin(), currentDevices.end(), device) != currentDevices.end());
        });
      std::copy_if(devices_.begin(), devices_.end(), std::back_inserter(detachedDevices),
        [&currentDevices](const DeviceInfo& device) {
          return (std::find(currentDevices.begin(), currentDevices.end(), device) == currentDevices.end());
        });
      std::copy_if(currentDevices.begin(), currentDevices.end(), std::back_inserter(attachedDevices),
        [this](const DeviceInfo& current) {
          return (std::find(devices_.begin(), devices_.end(), current) == devices_.end());
        });

      devices_ = currentDevices;

      if (callback_) {
        for (const DeviceInfo& device : detachedDevices)
          callback_(DEVICE_EVENT_DETACH, device);
        for (const DeviceInfo& device : attachedDevices)
          callback_(DEVICE_EVENT_ATTACH, device);
      }
    }
  });
}

void ltme01_sdk::DeviceNotifier::stop()
{
  if (activeFlag_) {
    activeFlag_ = false;
    thread_.join();
  }
}
