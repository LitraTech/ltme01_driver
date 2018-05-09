#include "ltme01_sdk/usb/UsbTransport.h"

#include "ltme01_sdk/Device.h"

#include <condition_variable>

#if defined(_MSC_VER) && (_MSC_VER < 1900)
typedef std::chrono::system_clock SteadyClock;
#else
typedef std::chrono::steady_clock SteadyClock;
#endif
typedef std::chrono::time_point<SteadyClock> TimePoint;

typedef struct
{
  ltme01_sdk::UsbTransport* transport;
  struct
  {
    TimePoint submission;
    TimePoint expiration;
  } time;
  std::mutex mutex;
  std::condition_variable cv;
  bool finished;
  int result;
  void* extra;
} WaitItem;

ltme01_sdk::UsbTransport::UsbTransport(const ltme01_sdk::UsbLocation& location)
  : location_(location)
  , usbDeviceHandle_(NULL)
  , handleTransferEvents_(false)
  , outstandingTransfers_(0)
{
  libusb_init(&usbContext_);
}

ltme01_sdk::UsbTransport::~UsbTransport()
{
  libusb_exit(usbContext_);
}

const ltme01_sdk::Location& ltme01_sdk::UsbTransport::location() const
{
  return location_;
}

int ltme01_sdk::UsbTransport::open()
{
  int result = RESULT_SUCCESS;

  libusb_device** deviceList = NULL;
  int count = libusb_get_device_list(usbContext_, &deviceList);
  if (count < 0)
    return RESULT_UNKNOWN_ERROR;
  else if (count == 0) {
    libusb_free_device_list(deviceList, 1);
    return RESULT_DEVICE_DISCONNECTED;
  }

  libusb_device* targetUsbDevice = NULL;
  for (int i = 0; i < count; i++) {
    libusb_device* usbDevice = deviceList[i];

    uint8_t busNumber = libusb_get_bus_number(usbDevice);
    uint8_t deviceAddress = libusb_get_device_address(usbDevice);

    if (busNumber == this->location_.busNumber() && deviceAddress == this->location_.deviceAddress()) {
      libusb_device_descriptor usbDeviceDescriptor;
      result = libusb_get_device_descriptor(usbDevice, &usbDeviceDescriptor);
      if (result != LIBUSB_SUCCESS)
        continue;
      if (usbDeviceDescriptor.idVendor == UsbLocation::TARGET_VID
        && usbDeviceDescriptor.idProduct == UsbLocation::TARGET_PID) {
        targetUsbDevice = usbDevice;
        break;
      }
    }
  }

  if (targetUsbDevice != NULL) {
    result = libusb_open(targetUsbDevice, &usbDeviceHandle_);
    libusb_free_device_list(deviceList, 1);
    if (result != LIBUSB_SUCCESS) {
      if (result == LIBUSB_ERROR_ACCESS)
        return RESULT_ACCESS_DENIED;
      else if (result == LIBUSB_ERROR_NO_DEVICE)
        return RESULT_DEVICE_DISCONNECTED;
      else
        return RESULT_UNKNOWN_ERROR;
    }

    if (libusb_kernel_driver_active(usbDeviceHandle_, 0) == 1) {
      result = libusb_detach_kernel_driver(usbDeviceHandle_, 0);
      if (result != LIBUSB_SUCCESS) {
        libusb_close(usbDeviceHandle_);
        usbDeviceHandle_ = NULL;

        if (result == LIBUSB_ERROR_NO_DEVICE)
          return RESULT_DEVICE_DISCONNECTED;
        else
          return RESULT_UNKNOWN_ERROR;
      }
    }

    result = libusb_claim_interface(usbDeviceHandle_, 0);
    if (result == LIBUSB_ERROR_NOT_FOUND) {
      // Normally the system will set configuration #1 for a USB device on enumeration.
      // In the rare case that this operation is not performed or failed, subsequent call to
      // libusb_claim_interface() will return LIBUSB_ERROR_NOT_FOUND (since no configuration
      // is selected in the first place), so we must call libusb_set_configuration() here to
      // make sure the device is in a proper state.
      libusb_set_configuration(usbDeviceHandle_, 1);
      result = libusb_claim_interface(usbDeviceHandle_, 0);
    }
    if (result != LIBUSB_SUCCESS) {
      libusb_close(usbDeviceHandle_);
      usbDeviceHandle_ = NULL;

      if (result == LIBUSB_ERROR_BUSY)
        return RESULT_ACCESS_DENIED;
      else if (result == LIBUSB_ERROR_NO_DEVICE)
        return RESULT_DEVICE_DISCONNECTED;
      else
        return RESULT_UNKNOWN_ERROR;
    }
    else {
      handleTransferEvents_ = true;
      thread_ = std::thread(std::bind(&UsbTransport::transferEventHandler, this));
      return RESULT_SUCCESS;
    }
  }
  else {
    libusb_free_device_list(deviceList, 1);
    return RESULT_DEVICE_DISCONNECTED;
  }
}

void ltme01_sdk::UsbTransport::close()
{
  if (usbDeviceHandle_ != NULL) {
    if (handleTransferEvents_) {
      handleTransferEvents_ = false;
      libusb_interrupt_event_handler(usbContext_);
      thread_.join();
    }

    libusb_release_interface(usbDeviceHandle_, 0);
    libusb_close(usbDeviceHandle_);
    usbDeviceHandle_ = NULL;
  }
}

int ltme01_sdk::UsbTransport::doDataTransaction(ltme01_sdk::DataPacket& dataPacket, unsigned int timeout)
{
  timeout = (timeout == 0) ? DEFAULT_DATA_TRANSACTION_TIMEOUT : timeout;

  std::shared_ptr<WaitItem> waitItem = std::make_shared<WaitItem>();
  waitItem->transport = this;
  waitItem->time.submission = SteadyClock::now();
  waitItem->time.expiration = waitItem->time.submission + std::chrono::milliseconds(timeout);
  waitItem->finished = false;
  waitItem->extra = NULL;

  libusb_transfer* transfer = libusb_alloc_transfer(0);
  libusb_fill_bulk_transfer(transfer, usbDeviceHandle_, DATA_IN_EP, dataPacket.data(), DataPacket::MAX_DATA_PACKET_SIZE,
    &UsbTransport::dataEndpointRxCallback, (void*)&waitItem, timeout);

  int result = libusb_submit_transfer(transfer);
  if (result == LIBUSB_SUCCESS) {
    outstandingTransfers_++;
    std::unique_lock<std::mutex> lock(waitItem->mutex);
    waitItem->cv.wait(lock, [&waitItem]() { return waitItem->finished; });
  }
  else {
    waitItem->finished = true;
    waitItem->result = RESULT_UNKNOWN_ERROR;
  }

  libusb_free_transfer(transfer);

  return waitItem->result;
}

int ltme01_sdk::UsbTransport::doCtrlTransaction(ltme01_sdk::RequestPacket& requestPacket, ltme01_sdk::ResponsePacket& responsePacket, unsigned int timeout)
{
  timeout = (timeout == 0) ? DEFAULT_CTRL_TRANSACTION_TIMEOUT : timeout;

  std::pair<RequestPacket*, ResponsePacket*> packets(&requestPacket, &responsePacket);

  std::shared_ptr<WaitItem> waitItem = std::make_shared<WaitItem>();
  waitItem->transport = this;
  waitItem->time.submission = SteadyClock::now();
  waitItem->time.expiration = waitItem->time.submission + std::chrono::milliseconds(timeout);
  waitItem->finished = false;
  waitItem->extra = &packets;

  libusb_transfer* transfer = libusb_alloc_transfer(0);
  libusb_fill_bulk_transfer(transfer, usbDeviceHandle_, CTRL_OUT_EP, requestPacket.data(),
    requestPacket.length(), &UsbTransport::ctrlEndpointTxCallback, (void*)&waitItem, timeout);

  int result = libusb_submit_transfer(transfer);
  if (result == LIBUSB_SUCCESS) {
    outstandingTransfers_++;
    std::unique_lock<std::mutex> lock(waitItem->mutex);
    waitItem->cv.wait(lock, [&waitItem]() { return waitItem->finished; });
  }
  else {
    waitItem->finished = true;
    waitItem->result = RESULT_UNKNOWN_ERROR;
  }

  libusb_free_transfer(transfer);

  return waitItem->result;
}

void ltme01_sdk::UsbTransport::transferEventHandler()
{
  while (handleTransferEvents_ || outstandingTransfers_ > 0)
    libusb_handle_events(usbContext_);
}

void ltme01_sdk::UsbTransport::dataEndpointRxCallback(libusb_transfer* transfer)
{
  std::shared_ptr<WaitItem> waitItem = *(std::shared_ptr<WaitItem>*)transfer->user_data;

  waitItem->transport->outstandingTransfers_--;
  switch (transfer->status) {
  case LIBUSB_TRANSFER_COMPLETED:
    waitItem->result = RESULT_SUCCESS;
    break;
  case LIBUSB_TRANSFER_TIMED_OUT:
    waitItem->result = RESULT_TIMEOUT;
    break;
  case LIBUSB_TRANSFER_NO_DEVICE:
  case LIBUSB_TRANSFER_STALL:
    waitItem->result = RESULT_DEVICE_DISCONNECTED;
    break;
  default:
    waitItem->result = RESULT_UNKNOWN_ERROR;
    break;
  }

  std::unique_lock<std::mutex> lock(waitItem->mutex);
  waitItem->finished = true;
  waitItem->cv.notify_one();
  lock.unlock();
}

void ltme01_sdk::UsbTransport::ctrlEndpointTxCallback(libusb_transfer* transfer)
{
  std::shared_ptr<WaitItem> waitItem = *(std::shared_ptr<WaitItem>*)transfer->user_data;

  switch (transfer->status) {
  case LIBUSB_TRANSFER_COMPLETED: {
    int left
      = (int)std::chrono::duration_cast<std::chrono::milliseconds>(waitItem->time.expiration - SteadyClock::now()).count();
    if (left > 0) {
      ResponsePacket* responsePacket = ((std::pair<RequestPacket*, ResponsePacket*>*)waitItem->extra)->second;
      libusb_fill_bulk_transfer(transfer, waitItem->transport->usbDeviceHandle_, CTRL_IN_EP, responsePacket->data(),
        ResponsePacket::MAX_RESPONSE_PACKET_SIZE, &UsbTransport::ctrlEndpointRxCallback, transfer->user_data, left);
      int result = libusb_submit_transfer(transfer);
      if (result == LIBUSB_SUCCESS)
        return;
      else {
        waitItem->result = RESULT_UNKNOWN_ERROR;
        break;
      }
    }
  }
  case LIBUSB_TRANSFER_TIMED_OUT:
    waitItem->result = RESULT_TIMEOUT;
    break;
  case LIBUSB_TRANSFER_NO_DEVICE:
  case LIBUSB_TRANSFER_STALL:
    waitItem->result = RESULT_DEVICE_DISCONNECTED;
    break;
  default:
    waitItem->result = RESULT_UNKNOWN_ERROR;
    break;
  }

  waitItem->transport->outstandingTransfers_--;

  std::unique_lock<std::mutex> lock(waitItem->mutex);
  waitItem->finished = true;
  waitItem->cv.notify_one();
  lock.unlock();
}

void ltme01_sdk::UsbTransport::ctrlEndpointRxCallback(libusb_transfer* transfer)
{
  std::shared_ptr<WaitItem> waitItem = *(std::shared_ptr<WaitItem>*)transfer->user_data;

  switch (transfer->status) {
  case LIBUSB_TRANSFER_COMPLETED:
    waitItem->result = RESULT_SUCCESS;
    break;
  case LIBUSB_TRANSFER_TIMED_OUT:
    waitItem->result = RESULT_TIMEOUT;
    break;
  case LIBUSB_TRANSFER_NO_DEVICE:
  case LIBUSB_TRANSFER_STALL:
    waitItem->result = RESULT_DEVICE_DISCONNECTED;
    break;
  default:
    waitItem->result = RESULT_UNKNOWN_ERROR;
    break;
  }

  waitItem->transport->outstandingTransfers_--;

  std::unique_lock<std::mutex> lock(waitItem->mutex);
  waitItem->finished = true;
  waitItem->cv.notify_one();
  lock.unlock();
}
