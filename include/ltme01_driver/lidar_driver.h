#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "ltme01_driver/device_notifier.h"

#include <condition_variable>

class LidarDriver
{
public:
  const static std::string DEFAULT_DEVICE;
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
  ros::NodeHandle nh_, nhPrivate_;
  ros::Publisher laserScanPublisher_;
  sensor_msgs::LaserScan laserScan_;

  std::string device_;
  std::string frameId_;
  double angleMin_;
  double angleMax_;
  double angleExcludedMin_;
  double angleExcludedMax_;
  double rangeMin_;
  double rangeMax_;

  DeviceNotifier deviceNotifier_;

  std::mutex mutex_;
  std::condition_variable cv_;
};

#endif
