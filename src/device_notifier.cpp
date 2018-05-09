#include "ltme01_driver/device_notifier.h"

#include <boost/filesystem.hpp>

#include <sys/inotify.h>

#include <ros/ros.h>

DeviceNotifier::DeviceNotifier()
  : notifierStarted_(false)
  , runEventLoop_(false)
{
}

DeviceNotifier::~DeviceNotifier()
{
  stop();
}

void DeviceNotifier::registerDeviceEventCallback(DeviceEventCallback callback)
{
  deviceEventCallback_ = callback;
}

void DeviceNotifier::start(const std::string& deviceNode)
{
  boost::filesystem::path path(deviceNode);
  deviceNodeDir_ = path.parent_path().string();
  deviceNodeName_ = path.filename().string();

  notifierStarted_ = true;
  thread_ = std::thread([this]() {
    int fd = inotify_init1(IN_NONBLOCK);
    int wd = inotify_add_watch(fd, deviceNodeDir_.c_str(), IN_CREATE | IN_DELETE);

    if (fd == -1 || wd == -1) {
      ROS_ERROR("Can't initialize device notification");
      exit(-1);
    }

    runEventLoop_ = true;
    while (runEventLoop_ && ros::ok()) {
      char buffer[sizeof(struct inotify_event) + NAME_MAX + 1];
      int length = read(fd, buffer, sizeof(buffer));

      if (length == -1) {
        if (errno == EAGAIN) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
        else {
          ROS_ERROR("Unknown inotify error (%d)", errno);
          exit(-1);
        }
      }
      else if (length > 0) {
        if (!deviceEventCallback_)
          continue;

        struct inotify_event* nextEvent = (struct inotify_event*)buffer;
        while (nextEvent < (void*)(buffer + sizeof(buffer))) {
          if (std::string(nextEvent->name) == deviceNodeName_) {
            if (nextEvent->mask & IN_CREATE)
              deviceEventCallback_(EVENT_DEVICE_NODE_CREATED);
            else if (nextEvent->mask & IN_DELETE)
              deviceEventCallback_(EVENT_DEVICE_NODE_DELETED);
          }

          nextEvent += sizeof(struct inotify_event) + nextEvent->len;
        }
      }
    }
  });
}

void DeviceNotifier::stop()
{
  if (notifierStarted_) {
    runEventLoop_ = false;
    thread_.join();

    notifierStarted_ = false;
  }
}
