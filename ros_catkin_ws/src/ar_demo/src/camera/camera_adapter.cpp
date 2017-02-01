//! Unified camera node - camera driver adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_adapter.hpp"

// ROS headers.
#include <ros/ros.h>


namespace ar_demo {

  CameraAdapter::CameraAdapter(std::shared_ptr<const Configuration> configuration):
		configuration_{configuration}
	{
    ;
  }

  CameraAdapter::~CameraAdapter() {
  }

  bool CameraAdapter::isAutoExposureSupported() {
    return false;
  }

  bool CameraAdapter::setAutoExposure(double target_brightness, double time_s_min, double time_s_max,
          double gain_db_min, double gain_db_max) {
    ROS_ERROR("Setting automatic exposure is not supported by camera driver.");
    return false;
  }

  bool CameraAdapter::isAutoExposureRoiSupported() {
    return false;
  }

  bool CameraAdapter::setAutoExposureRoi(int column, int row, int width, int height) {
    ROS_ERROR("Setting automatic exposure region of is interest not supported by camera driver.");
    return false;
  }

  bool CameraAdapter::isAutoWhiteBalanceSupported() {
    return false;
  }

  bool CameraAdapter::setAutoWhiteBalance() {
    ROS_ERROR("Setting automatic white balance is not supported by camera driver.");
    return false;
  }

  bool CameraAdapter::isAutoWhiteBalanceRoiSupported() {
    return false;
  }

  bool CameraAdapter::setAutoWhiteBalanceRoi(int column, int row, int width, int height) {
    ROS_ERROR("Setting automatic white balance region of interest is not supported by camera driver.");
    return false;
  }
}
