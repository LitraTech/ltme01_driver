#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

#include "ltme01_sdk/DeviceInfo.h"
#include "ltme01_sdk/Device.h"

#include <atomic>

class LidarDriver
{
public:
  const static std::string DEFAULT_FRAME_ID;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT;

public:
  LidarDriver();
  void run();

private:
  std::unique_ptr<ltme01_sdk::DeviceInfo> waitForDevice(const std::string& devicePathOrAddress);
  void runDataLoop(ltme01_sdk::Device& device);
  void runIdleLoop(ltme01_sdk::Device& device);
  bool setLowPowerService(std_srvs::SetBool::Request &request,
                          std_srvs::SetBool::Response &response);

private:
  ros::NodeHandle nh_, nhPrivate_;
  ros::Publisher laserScanPublisher_;
  sensor_msgs::LaserScan laserScan_;
  ros::ServiceServer setLowPowerService_;
  ros::AsyncSpinner spinner_;

  std::string device_;
  std::string frameId_;
  double angleMin_;
  double angleMax_;
  double angleExcludedMin_;
  double angleExcludedMax_;
  double rangeMin_;
  double rangeMax_;

  std::atomic<bool> lowPowerEnabled_;
};

#endif
