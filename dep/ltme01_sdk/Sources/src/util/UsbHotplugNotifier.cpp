#include "ltme01_sdk/util/UsbHotplugNotifier.h"

#include <thread>

#ifdef __linux__

#include <libusb-1.0/libusb.h>

#include "ltme01_sdk/usb/UsbLocation.h"

namespace ltme01_sdk
{

class HotplugNotifierLibusb : public UsbHotplugNotifier
{
public:
  HotplugNotifierLibusb();

  virtual bool startEventMonitor(std::function<void(bool)> recipient, bool enumerate);
  virtual void stopEventMonitor();

private:
  static int LIBUSB_CALL HotplugCallback(
      libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data);

private:
  std::function<void(bool)> recipient_;
  std::thread thread_;
  libusb_hotplug_callback_handle hotplugCallbackHandle_;
  bool runEventLoop_;
};

}

ltme01_sdk::HotplugNotifierLibusb::HotplugNotifierLibusb()
  : runEventLoop_(false)
{
}

bool ltme01_sdk::HotplugNotifierLibusb::startEventMonitor(std::function<void(bool)> recipient, bool enumerate)
{
  this->recipient_ = recipient;

  thread_ = std::thread([recipient, enumerate, this]() {
    libusb_hotplug_flag hotplugFlags = enumerate ? LIBUSB_HOTPLUG_ENUMERATE : LIBUSB_HOTPLUG_NO_FLAGS;
    libusb_hotplug_register_callback(NULL,
      (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
      hotplugFlags, UsbLocation::TARGET_VID, UsbLocation::TARGET_PID, LIBUSB_HOTPLUG_MATCH_ANY, HotplugCallback, (void*)this,
      &hotplugCallbackHandle_);

    runEventLoop_ = true;
    while (runEventLoop_) {
      struct timeval tv = { 0, 100000 };
      libusb_handle_events_timeout(NULL, &tv);
    }

    libusb_hotplug_deregister_callback(NULL, hotplugCallbackHandle_);
  });

  return true;
}

void ltme01_sdk::HotplugNotifierLibusb::stopEventMonitor()
{
  runEventLoop_ = false;
  thread_.join();
}

int ltme01_sdk::HotplugNotifierLibusb::HotplugCallback(
  libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data)
{
  HotplugNotifierLibusb* hotplugNotifier = (HotplugNotifierLibusb*)user_data;
  hotplugNotifier->recipient_(false);
  return 0;
}

#elif _WIN32

#include <windows.h>
#include <dbt.h>

namespace ltme01_sdk
{

class HotplugNotifierWindows : public UsbHotplugNotifier
{
public:
  static const GUID TARGET_DEVICE_GUID;
  static const TCHAR* WINDOW_CLASS_NAME;

public:
  virtual bool startEventMonitor(std::function<void(bool)> recipient, bool enumerate);
  virtual void stopEventMonitor();

private:
  static INT_PTR WINAPI WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

private:
  std::function<void(bool)> recipient_;
  std::thread thread_;
  HWND windowHandle_;
};

}

const GUID ltme01_sdk::HotplugNotifierWindows::TARGET_DEVICE_GUID
  = { 0xD73BB3B7, 0x78CE, 0x4EE7, 0x90, 0xD7, 0x32, 0xC2, 0x3C, 0x03, 0x28, 0x8E };
const TCHAR* ltme01_sdk::HotplugNotifierWindows::WINDOW_CLASS_NAME = TEXT("LTME-01 Hotplug Notifier Window Class");

bool ltme01_sdk::HotplugNotifierWindows::startEventMonitor(std::function<void(bool)> recipient, bool enumerate)
{
  this->recipient_ = recipient;

  thread_ = std::thread([this, enumerate]() {
    WNDCLASS wndClass;
    ZeroMemory(&wndClass, sizeof(wndClass));
    wndClass.lpfnWndProc = reinterpret_cast<WNDPROC>(WindowProc);
    wndClass.lpszClassName = WINDOW_CLASS_NAME;
    RegisterClass(&wndClass);
    windowHandle_ = CreateWindow(WINDOW_CLASS_NAME, NULL, 0, 0, 0, 0, 0, NULL, NULL, NULL, this);

    if (enumerate)
      SendMessage(windowHandle_, WM_USER, 0, 0);

    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0) > 0) {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
  });

  return true;
}

void ltme01_sdk::HotplugNotifierWindows::stopEventMonitor()
{
  PostMessage(windowHandle_, WM_CLOSE, 0, 0);
  thread_.join();
}

INT_PTR ltme01_sdk::HotplugNotifierWindows::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  LRESULT result = 1;
  static HDEVNOTIFY hDeviceNotify = INVALID_HANDLE_VALUE;

  switch (uMsg) {
  case WM_NCCREATE: {
    LPCREATESTRUCT lpCreateStruct = (LPCREATESTRUCT)lParam;
    HotplugNotifierWindows* hotplugNotifier = (HotplugNotifierWindows*)lpCreateStruct->lpCreateParams;
    SetWindowLongPtr(hwnd, GWLP_USERDATA, LONG_PTR(hotplugNotifier));
    break;
  }

  case WM_CREATE: {
    DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;

    ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
    NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
    NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    NotificationFilter.dbcc_classguid = TARGET_DEVICE_GUID;

    hDeviceNotify = RegisterDeviceNotification(hwnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);

    break;
  }

  case WM_USER: {
    HotplugNotifierWindows* hotplugNotifier = (HotplugNotifierWindows*)GetWindowLongPtr(hwnd, GWLP_USERDATA);
    if (hotplugNotifier && hotplugNotifier->recipient_)
      hotplugNotifier->recipient_(true);
  }

  case WM_DEVICECHANGE: {
    switch (wParam) {
    case DBT_DEVICEARRIVAL:
    case DBT_DEVICEREMOVECOMPLETE: {
      HotplugNotifierWindows* hotplugNotifier = (HotplugNotifierWindows*)GetWindowLongPtr(hwnd, GWLP_USERDATA);
      if (hotplugNotifier && hotplugNotifier->recipient_)
        hotplugNotifier->recipient_(false);
      break;
    }

    default:
      break;
    }

    break;
  }

  case WM_CLOSE:
    UnregisterDeviceNotification(hDeviceNotify);
    DestroyWindow(hwnd);
    break;

  case WM_DESTROY:
    PostQuitMessage(0);
    break;

  default:
    result = DefWindowProc(hwnd, uMsg, wParam, lParam);
    break;
  }

  return result;
}

#endif

ltme01_sdk::UsbHotplugNotifier* ltme01_sdk::UsbHotplugNotifier::createInstance()
{
#ifdef __linux__
  return new HotplugNotifierLibusb();
#elif _WIN32
  return new HotplugNotifierWindows();
#endif
}
