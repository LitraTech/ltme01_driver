#include "ltme01_driver/lidar_driver.h"

#include <libudev.h>
#include <sys/stat.h>

#include "ltme01_sdk/Device.h"
#include "ltme01_sdk/usb/UsbLocation.h"

const std::string LidarDriver::DEFAULT_DEVICE = "/dev/ltme01";
const std::string LidarDriver::DEFAULT_FRAME_ID = "ltme01";
const double LidarDriver::ANGLE_MIN_LIMIT = -2.356;
const double LidarDriver::ANGLE_MAX_LIMIT = 2.356;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.2;
const double LidarDriver::RANGE_MAX_LIMIT = 30;

LidarDriver::LidarDriver()
  : nhPrivate_("~")
{
  nhPrivate_.param<std::string>("device", device_, DEFAULT_DEVICE);
  nhPrivate_.param<std::string>("frame_id", frameId_, DEFAULT_FRAME_ID);
  nhPrivate_.param<double>("angle_min", angleMin_, ANGLE_MIN_LIMIT);
  nhPrivate_.param<double>("angle_max", angleMax_, ANGLE_MAX_LIMIT);
  nhPrivate_.param<double>("angle_excluded_min", angleExcludedMin_, DEFAULT_ANGLE_EXCLUDED_MIN);
  nhPrivate_.param<double>("angle_excluded_max", angleExcludedMax_, DEFAULT_ANGLE_EXCLUDED_MAX);
  nhPrivate_.param<double>("range_min", rangeMin_, RANGE_MIN_LIMIT);
  nhPrivate_.param<double>("range_max", rangeMax_, RANGE_MAX_LIMIT);

  if (!(angleMin_ < angleMax_)) {
    ROS_ERROR("angle_min (%f) can't be larger than or equal to angle_max (%f)", angleMin_, angleMax_);
    exit(-1);
  }
  if (angleMin_ < ANGLE_MIN_LIMIT) {
    ROS_ERROR("angle_min is set to %f while its minimum allowed value is %f", angleMin_, ANGLE_MIN_LIMIT);
    exit(-1);
  }
  if (angleMax_ > ANGLE_MAX_LIMIT) {
    ROS_ERROR("angle_max is set to %f while its maximum allowed value is %f", angleMax_, ANGLE_MAX_LIMIT);
    exit(-1);
  }
  if (!(rangeMin_ < rangeMax_)) {
    ROS_ERROR("range_min (%f) can't be larger than or equal to range_max (%f)", rangeMin_, rangeMax_);
    exit(-1);
  }
  if (rangeMin_ < RANGE_MIN_LIMIT) {
    ROS_ERROR("range_min is set to %f while its minimum allowed value is %f", rangeMin_, RANGE_MIN_LIMIT);
    exit(-1);
  }
  if (rangeMax_ > RANGE_MAX_LIMIT) {
    ROS_ERROR("range_max is set to %f while its maximum allowed value is %f", rangeMax_, RANGE_MAX_LIMIT);
    exit(-1);
  }

  laserScanPublisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 16);
}

void LidarDriver::run()
{
  deviceNotifier_.registerDeviceEventCallback([this](DeviceEvent event) {
    if (event == EVENT_DEVICE_NODE_CREATED) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_one();
      lock.unlock();
    }
  });
  deviceNotifier_.start(device_);

  while (nh_.ok()) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool status = cv_.wait_for(lock, std::chrono::milliseconds(500), [this]() {
      return (access(device_.c_str(), F_OK) == 0);
    });
    if (!status)
      continue;

    struct stat sb;
    if (stat(device_.c_str(), &sb) == -1) {
      ROS_WARN_THROTTLE(5, "Can't access device %s", device_.c_str());
      continue;
    }

    int busNum = 0, deviceNum = 0;

    struct udev* udev = udev_new();
    if (udev != NULL) {
      struct udev_device* device = udev_device_new_from_devnum(udev, 'c', sb.st_rdev);
      if (device != NULL) {
        busNum = std::atoi(udev_device_get_sysattr_value(device, "busnum"));
        deviceNum = std::atoi(udev_device_get_sysattr_value(device, "devnum"));
        udev_device_unref(device);
      }
      udev_unref(udev);
    }

    if (busNum == 0 || deviceNum == 0) {
      ROS_WARN_THROTTLE(5, "Can't query attributes for device %s", device_.c_str());
      continue;
    }

    ROS_INFO("Found LTME-01 device at bus %03d, address %03d", busNum, deviceNum);

    ltme01_sdk::Device device(ltme01_sdk::UsbLocation(busNum, deviceNum));

    int result = device.open();
    if (result != ltme01_sdk::RESULT_SUCCESS) {
      if (result == ltme01_sdk::RESULT_ACCESS_DENIED)
        ROS_ERROR("Unable to open device (access denied)");
      else if (result == ltme01_sdk::RESULT_DEVICE_DISCONNECTED)
        ROS_ERROR("Unable to open device (device disconnected)");
      else
        ROS_ERROR("Unable to open device (unknown error)");
    }
    else {
      ROS_INFO("Device opened");

      auto readRangeData = [&device](ltme01_sdk::DataPacket& dataPacket) {
        int count = 0;
        do {
          int result = device.readDataPacket(dataPacket, 6000);
          if (result != ltme01_sdk::RESULT_SUCCESS)
            throw std::exception();
        } while (count++ < 5 && !dataPacket.isValid());
        if (count == 5)
          throw std::exception();
      };

      ltme01_sdk::DataPacket dataPacket;
      try {
        readRangeData(dataPacket);

        int beamCount = dataPacket.count() * 32;
        int beamIndexMin = std::ceil(angleMin_ * beamCount / (2 * M_PI));
        int beamIndexMax = std::floor(angleMax_ * beamCount / (2 * M_PI));
        int beamIndexExcludedMin = std::ceil(angleExcludedMin_ * beamCount / (2 * M_PI));
        int beamIndexExcludedMax = std::floor(angleExcludedMax_ * beamCount / (2 * M_PI));

        laserScan_.header.frame_id = frameId_;
        laserScan_.angle_increment = 2 * M_PI / beamCount;
        laserScan_.angle_min = angleMin_;
        laserScan_.angle_max = angleMax_;
        laserScan_.time_increment = 0.1 / beamCount;
        laserScan_.scan_time = 0.1;
        laserScan_.range_min = rangeMin_;
        laserScan_.range_max = rangeMax_;
        laserScan_.ranges.resize(beamIndexMax - beamIndexMin + 1);

        auto addRangeData = [&](const ltme01_sdk::DataPacket& dataPacket) {
          for (int i = 0; i < dataPacket.count(); i++) {
            int beamIndex = dataPacket.count() * (dataPacket.index() - 12) + i;
            if (beamIndex < beamIndexMin || beamIndex > beamIndexMax)
              continue;
            if (beamIndex >= beamIndexExcludedMin && beamIndex <= beamIndexExcludedMax)
              continue;

            uint16_t range = dataPacket.range(i);
            if (range == ltme01_sdk::DataPacket::RANGE_TIMEOUT || range == ltme01_sdk::DataPacket::RANGE_NO_DATA)
              laserScan_.ranges[beamIndex - beamIndexMin] = 0;
            else
              laserScan_.ranges[beamIndex - beamIndexMin] = range / 100.0;
          }
        };

        while (nh_.ok()) {
          std::fill(laserScan_.ranges.begin(), laserScan_.ranges.end(), 0.0);

          do {
            readRangeData(dataPacket);
          } while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN);

          while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX) {
            addRangeData(dataPacket);
            readRangeData(dataPacket);
          }
          addRangeData(dataPacket);

          laserScan_.header.stamp = ros::Time::now();
          laserScanPublisher_.publish(laserScan_);
        }
      }
      catch (std::exception& e) {
        ROS_WARN("Error reading data from device");
      }

      device.close();

      ROS_INFO("Device closed");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ltme01_driver_node");
  ROS_INFO("ltme01_driver_node started");

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
