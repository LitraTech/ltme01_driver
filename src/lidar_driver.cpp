#include "ltme01_driver/lidar_driver.h"

#include <libudev.h>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>

#include "ltme01_sdk/usb/UsbLocation.h"
#include "ltme01_sdk/lan/LanLocation.h"
#include "ltme01_sdk/Device.h"

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
  nhPrivate_.param<bool>("calibrate_time", calibrateTime_, false);

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

      ltme01_sdk::DataPacket dataPacket;

      ros::Duration packetLatency;
      if (deviceInfo->type() == ltme01_sdk::DEVICE_TYPE_LAN && calibrateTime_) {
        ROS_INFO("Starting time calibration...This may take a few seconds");

        if (!device.resetTimestamp())
          ROS_ERROR("Can't prepare device for calibration. "
                    "Please make sure that your device supports this feature");
        else {
          std::vector<ros::Duration> deviceToHostOffsets(CALIBRATION_STAGE_1_ITERATIONS);
          for (int i = 0; i < CALIBRATION_STAGE_1_ITERATIONS; i++) {
            ros::Time timeOfRequest = ros::Time::now();
            uint32_t deviceTimestamp = 0;
            if (!device.getTimestamp(deviceTimestamp)) {
              ROS_ERROR("Failed to query for device timestamp");
              continue;
            }
            ros::Time timeOfResponse = ros::Time::now();

            deviceToHostOffsets[i] = ros::Time((timeOfResponse.toSec() + timeOfRequest.toSec()) / 2) -
                ros::Time((double)deviceTimestamp / 1e6);
          }

          std::nth_element(deviceToHostOffsets.begin(),
                           deviceToHostOffsets.begin() + deviceToHostOffsets.size() / 2,
                           deviceToHostOffsets.end());
          ros::Duration deviceToHostOffset = deviceToHostOffsets[deviceToHostOffsets.size() / 2];

          ROS_INFO("Clearing cached packets from underlying protocol stack...");
          while (true) {
            int result = device.readDataPacket(dataPacket, 1);
            if (result != ltme01_sdk::RESULT_SUCCESS) {
              if (result == ltme01_sdk::RESULT_TIMEOUT)
                ROS_INFO("Cache cleared");
              else
                ROS_INFO("Error reading packet. Is device functioning properly?");
              break;
            }
          }

          std::vector<ros::Duration> packetLatencies(CALIBRATION_STAGE_2_ITERATIONS);
          for (int i = 0; i < CALIBRATION_STAGE_2_ITERATIONS; i++) {
            int result = device.readDataPacket(dataPacket, 100);
            if (result != ltme01_sdk::RESULT_SUCCESS) {
              ROS_ERROR("Can't read data packet for calibration");
              continue;
            }
            ros::Time timeOfReception = ros::Time::now();
            ros::Time timeOfPacketStart = ros::Time().fromSec((double)dataPacket.timestamp() / 1e6 +
                                                              deviceToHostOffset.toSec());

            packetLatencies[i] = timeOfReception - timeOfPacketStart;
          }

          std::nth_element(packetLatencies.begin(),
                           packetLatencies.begin() + packetLatencies.size() / 2,
                           packetLatencies.end());
          packetLatency = packetLatencies[packetLatencies.size() / 2];

          ROS_INFO("Calibration finished. Latency is %.3f milliseconds", packetLatency.toSec() * 1e3);
        }
      }

      ROS_INFO("Publishing LaserScan messages...");

      auto readDataPacket = [&device](ltme01_sdk::DataPacket& dataPacket) {
        int count = 0;
        do {
          int result = device.readDataPacket(dataPacket, 6000);
          if (result != ltme01_sdk::RESULT_SUCCESS)
            throw std::exception();
        } while (count++ < 5 && !dataPacket.isValid());
        if (count == 5)
          throw std::exception();
      };

      try {
        readDataPacket(dataPacket);

        int beamCount = dataPacket.count() * 32;
        int beamIndexMin = std::ceil(angleMin_ * beamCount / (2 * M_PI));
        int beamIndexMax = std::floor(angleMax_ * beamCount / (2 * M_PI));
        int beamIndexExcludedMin = std::ceil(angleExcludedMin_ * beamCount / (2 * M_PI));
        int beamIndexExcludedMax = std::floor(angleExcludedMax_ * beamCount / (2 * M_PI));

        laserScan_.header.frame_id = frameId_;
        laserScan_.angle_min = angleMin_;
        laserScan_.angle_max = angleMax_;
        laserScan_.angle_increment = 2 * M_PI / beamCount;
        if (deviceInfo->type() == ltme01_sdk::DEVICE_TYPE_USB) {
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

        uint32_t dataPacketTimestamps[ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX -
            ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN + 1] = { 0 };

        auto processDataPacket = [&](const ltme01_sdk::DataPacket& dataPacket) {
          dataPacketTimestamps[dataPacket.index()] = dataPacket.timestamp();

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

        while (nh_.ok()) {
          std::fill(laserScan_.ranges.begin(), laserScan_.ranges.end(), 0.0);

          do {
            readDataPacket(dataPacket);
          } while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN);

          laserScan_.header.stamp = ros::Time::now() - packetLatency;

          while (dataPacket.index() != ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX) {
            processDataPacket(dataPacket);
            readDataPacket(dataPacket);
          }
          processDataPacket(dataPacket);

          int estimatedScanTimeUs = 0;
          for (int i = 1; i < sizeof(dataPacketTimestamps) / sizeof(uint32_t); i++) {
            if (dataPacketTimestamps[i] >= dataPacketTimestamps[i - 1])
              estimatedScanTimeUs += dataPacketTimestamps[i] - dataPacketTimestamps[i - 1];
            else
              estimatedScanTimeUs += dataPacketTimestamps[i] + (0xFFFFFFFF - dataPacketTimestamps[i - 1] + 1);
          }
          if (estimatedScanTimeUs != 0) {
            double estimatedScanTime = ((double)estimatedScanTimeUs / 1e6) /
                (ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX - ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN) *
                (ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MAX - ltme01_sdk::DataPacket::DATA_PACKET_INDEX_MIN + 1) *
                4 / 3;
            if (fabs(estimatedScanTime - laserScan_.scan_time) / laserScan_.scan_time < 0.1) {
              laserScan_.time_increment = estimatedScanTime / beamCount;
              laserScan_.scan_time = estimatedScanTime;
            }
          }

          laserScanPublisher_.publish(laserScan_);
        }
      }
      catch (std::exception& e) {
        ROS_WARN("Error reading data from device");
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

    boost::asio::io_service ioService;
    boost::asio::ip::udp::socket socket(ioService, boost::asio::ip::udp::v4());
    boost::asio::deadline_timer timer(ioService);

    boost::asio::ip::udp::endpoint localEndpoint(boost::asio::ip::udp::v4(), ntohs(port));
    boost::system::error_code error;
    socket.bind(localEndpoint, error);
    if (error) {
      ROS_ERROR("Can't bind to port %d. Please make sure you have sufficient permissions, "
                "and only one driver node instance is running.", ntohs(port));
      exit(-1);
    }

    ltme01_sdk::DataPacket dataPacket;
    while (nh_.ok()) {
      boost::system::error_code timerResult, transactionResult;
      timerResult = transactionResult = boost::asio::error::would_block;

      timer.expires_from_now(boost::posix_time::milliseconds(500));
      timer.async_wait([&](const boost::system::error_code& error) {
        timerResult = error;
      });

      auto buffer = boost::asio::buffer(dataPacket.data(), ltme01_sdk::DataPacket::MAX_DATA_PACKET_SIZE);
      boost::asio::ip::udp::endpoint peerEndpoint;
      std::function<void(const boost::system::error_code&, std::size_t)> handler;
      handler = [&](const boost::system::error_code& error, std::size_t) {
        if (!error) {
          if (peerEndpoint.address().to_v4().to_ulong() != ntohl(address))
            socket.async_receive_from(buffer, peerEndpoint, handler);
          else
            deviceInfo = std::unique_ptr<ltme01_sdk::DeviceInfo>(
                  new ltme01_sdk::DeviceInfo(ltme01_sdk::LanLocation(address, port)));
        }
        else
          transactionResult = error;
      };
      socket.async_receive_from(buffer, peerEndpoint, handler);

      ioService.reset();
      while (ioService.run_one()) {
        if (transactionResult != boost::asio::error::would_block)
          timer.cancel();
        if (timerResult != boost::asio::error::would_block)
          socket.cancel();
      }

      if (deviceInfo)
        break;
      else {
        ROS_INFO_THROTTLE(5, "Waiting for device... [%s]", (boost::asio::ip::address_v4(ntohl(address)).to_string() +
                                                            ":" + std::to_string(ntohs(port))).c_str());
        ros::Duration(0.5).sleep();
      }
    }
  }

  return deviceInfo;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ltme01_driver_node");
  ROS_INFO("ltme01_driver_node started");

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
