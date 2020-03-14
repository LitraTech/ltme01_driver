#include "ltme01_driver/lidar_driver.h"

#include <libudev.h>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/lan/LanLocation.h"

const std::string LidarDriver::DEFAULT_FRAME_ID = "ltme01";
const double LidarDriver::ANGLE_MIN_LIMIT = -2.356;
const double LidarDriver::ANGLE_MAX_LIMIT = 2.356;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.2;
const double LidarDriver::RANGE_MAX_LIMIT = 30;

LidarDriver::LidarDriver()
  : nhPrivate_("~")
  , spinner_(1)
  , lowPowerEnabled_(false)
{
  if (!nhPrivate_.getParam("device", device_)) {
    ROS_ERROR("Missing required parameter \"device\"");
    exit(-1);
  }
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
  setLowPowerService_ = nhPrivate_.advertiseService("set_low_power", &LidarDriver::setLowPowerService, this);

  spinner_.start();
}

void LidarDriver::run()
{
  while (nh_.ok()) {
    std::unique_ptr<ltme01_sdk::DeviceInfo> deviceInfo = waitForDevice(device_);
    if (!deviceInfo)
      break;

    if (deviceInfo->type() == ltme01_sdk::DEVICE_TYPE_USB) {
      const ltme01_sdk::UsbLocation& location = (const ltme01_sdk::UsbLocation&)deviceInfo->location();
      ROS_INFO("Found LTME-01B at USB bus %03d, address %03d", location.busNumber(), location.deviceAddress());
    }
    else if (deviceInfo->type() == ltme01_sdk::DEVICE_TYPE_LAN) {
      const ltme01_sdk::LanLocation& location = (const ltme01_sdk::LanLocation&)deviceInfo->location();
      ROS_INFO("Found LTME-01C at local network address %s",
               boost::asio::ip::address_v4(ntohl(location.deviceAddress())).to_string().c_str());
    }

    ltme01_sdk::Device device(*deviceInfo);

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

      try {
        while (nh_.ok()) {
          if (!lowPowerEnabled_.load())
            runDataLoop(device);
          else
            runIdleLoop(device);
        }
      }
      catch (std::exception& e) {
        if (!lowPowerEnabled_.load())
          ROS_WARN("Error reading data from device");
        else
          ROS_WARN("Lost connection to device");
      }

      device.close();

      ROS_INFO("Device closed");
    }

    ros::Duration(0.5).sleep();
  }
}

std::unique_ptr<ltme01_sdk::DeviceInfo> LidarDriver::waitForDevice(const std::string& devicePathOrAddress)
{
  std::unique_ptr<ltme01_sdk::DeviceInfo> deviceInfo;

  boost::filesystem::path path(devicePathOrAddress);
  if (path.has_root_path()) {
    struct udev* udev = udev_new();
    if (!udev) {
      ROS_ERROR("Can't initialize udev");
      exit(-1);
    }
    struct udev_enumerate* udevEnumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysattr(udevEnumerate, "idVendor", "16d0");
    udev_enumerate_add_match_sysattr(udevEnumerate, "idProduct", "0db7");

    while (nh_.ok()) {
      if (boost::filesystem::exists(path)) {
        boost::filesystem::path canonicalPath = boost::filesystem::canonical(path);

        udev_enumerate_scan_devices(udevEnumerate);
        struct udev_list_entry* deviceList = udev_enumerate_get_list_entry(udevEnumerate);
        struct udev_list_entry* deviceListEntry = NULL;
        udev_list_entry_foreach(deviceListEntry, deviceList) {
          const char* syspath = udev_list_entry_get_name(deviceListEntry);
          struct udev_device* device = udev_device_new_from_syspath(udev, syspath);
          if (canonicalPath.string() == udev_device_get_devnode(device)) {
            int busNumber = std::atoi(udev_device_get_sysattr_value(device, "busnum"));
            int deviceAddress = std::atoi(udev_device_get_sysattr_value(device, "devnum"));
            deviceInfo = std::unique_ptr<ltme01_sdk::DeviceInfo>(
                  new ltme01_sdk::DeviceInfo(ltme01_sdk::UsbLocation(busNumber, deviceAddress)));
          }
          udev_device_unref(device);
        }
      }

      if (deviceInfo)
        break;
      else {
        ROS_INFO_THROTTLE(5, "Waiting for device... [%s]", devicePathOrAddress.c_str());
        ros::Duration(1).sleep();
      }
    }

    udev_enumerate_unref(udevEnumerate);
    udev_unref(udev);
  }
  else {
    std::string addressStr = devicePathOrAddress;
    std::string portStr = "8100";

    size_t position = devicePathOrAddress.find(':');
    if (position != std::string::npos) {
      addressStr = devicePathOrAddress.substr(0, position);
      portStr = devicePathOrAddress.substr(position + 1);
    }

    in_addr_t address = INADDR_NONE;
    in_port_t port = 0;
    try {
      address = htonl(boost::asio::ip::address_v4::from_string(addressStr).to_ulong());
      port = htons(std::stoi(portStr));
    }
    catch (...) {
      ROS_ERROR("Invalid device address: %s", devicePathOrAddress.c_str());
      exit(-1);
    }

    std::unique_ptr<ltme01_sdk::Device> device(
          new ltme01_sdk::Device(ltme01_sdk::LanLocation(address, port)));

    int result = device->open();
    if (result != ltme01_sdk::RESULT_SUCCESS) {
      if (result == ltme01_sdk::RESULT_ACCESS_DENIED)
        ROS_ERROR("Can't bind to port %d. Please make sure you have sufficient permissions, "
                        "and only one driver node instance is running.", ntohs(port));
      else
        ROS_ERROR("Unable to initiate device discovery (unknown error)");
      exit(-1);
    }

    while (nh_.ok()) {
      if (device->checkConnectivity()) {
        deviceInfo = std::unique_ptr<ltme01_sdk::DeviceInfo>(
              new ltme01_sdk::DeviceInfo(ltme01_sdk::LanLocation(address, port)));
        break;
      }
      else {
        ROS_INFO_THROTTLE(5, "Waiting for device... [%s]", (boost::asio::ip::address_v4(ntohl(address)).to_string() +
                                                            ":" + std::to_string(ntohs(port))).c_str());
        ros::Duration(0.5).sleep();
      }
    }
  }

  return deviceInfo;
}

void LidarDriver::runDataLoop(ltme01_sdk::Device& device)
{
  auto readRangeData = [&device](ltme01_sdk::DataPacket& dataPacket) {
    int count = 0;
    do {
      int result = device.readDataPacket(dataPacket, 6000);
      if (result != ltme01_sdk::RESULT_SUCCESS)
        throw std::exception();
    } while (count++ < 5 && !dataPacket.isValid());
    if (count >= 5)
      throw std::exception();
  };

  ltme01_sdk::DataPacket dataPacket;

  try {
    readRangeData(dataPacket);
  }
  catch (...) {
    device.exitLowPowerMode();
    readRangeData(dataPacket);
  }
  int beamCount = dataPacket.count() * 32;
  int beamIndexMin = std::ceil(angleMin_ * beamCount / (2 * M_PI));
  int beamIndexMax = std::floor(angleMax_ * beamCount / (2 * M_PI));
  int beamIndexExcludedMin = std::ceil(angleExcludedMin_ * beamCount / (2 * M_PI));
  int beamIndexExcludedMax = std::floor(angleExcludedMax_ * beamCount / (2 * M_PI));

  laserScan_.header.frame_id = frameId_;
  laserScan_.angle_min = angleMin_;
  laserScan_.angle_max = angleMax_;
  laserScan_.angle_increment = 2 * M_PI / beamCount;
  if (typeid(device.location()) == typeid(ltme01_sdk::UsbLocation)) {
    laserScan_.time_increment = 1.0 / 10 / beamCount;
    laserScan_.scan_time = 1.0 / 10;
  }
  else {
    laserScan_.time_increment = 1.0 / 15 / beamCount;
    laserScan_.scan_time = 1.0 / 15;
  }
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
      if (range == ltme01_sdk::DataPacket::RANGE_TIMEOUT)
        laserScan_.ranges[beamIndex - beamIndexMin] = std::numeric_limits<float>::infinity();
      else if (range == ltme01_sdk::DataPacket::RANGE_NO_DATA)
        laserScan_.ranges[beamIndex - beamIndexMin] = 0;
      else {
        float value = range / 100.0;
        if (value < laserScan_.range_min)
          value = 0;
        else if (value > laserScan_.range_max)
          value = std::numeric_limits<float>::infinity();
        laserScan_.ranges[beamIndex - beamIndexMin] = value;
      }
    }
  };

  while (nh_.ok() && !lowPowerEnabled_.load()) {
    std::fill(laserScan_.ranges.begin(), laserScan_.ranges.end(), 0.0);

    do {
      readRangeData(dataPacket);
    } while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN);

    laserScan_.header.stamp = ros::Time::now();

    while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX) {
      addRangeData(dataPacket);
      readRangeData(dataPacket);
    }
    addRangeData(dataPacket);

    laserScanPublisher_.publish(laserScan_);
  }
}

void LidarDriver::runIdleLoop(ltme01_sdk::Device& device)
{
  device.enterLowPowerMode();
  ROS_INFO("Device now in low-power mode");

  auto checkDevicePresence = [&device]() {
    int count = 0;
    while (!device.checkConnectivity() && ++count < 5);
    if (count >= 5)
      throw std::exception();
  };
  while (nh_.ok() && lowPowerEnabled_.load())
    checkDevicePresence();

  device.exitLowPowerMode();
  ROS_INFO("Device brought out of low-power mode");
}

bool LidarDriver::setLowPowerService(std_srvs::SetBool::Request& request,
                                     std_srvs::SetBool::Response& response)
{
  lowPowerEnabled_ = request.data;
  response.success = true;
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ltme01_driver_node");
  ROS_INFO("ltme01_driver_node started");

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
