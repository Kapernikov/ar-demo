//! Unified camera node - simulated camera adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __AR_DEMO__CAMERA_ADAPTER_SIMULATED
#define __AR_DEMO__CAMERA_ADAPTER_SIMULATED


#include "camera_adapter.hpp"

// Standard C++ headers.
#include <string>
// OpenCV headers.
#include <opencv2/opencv.hpp>
// Basler Pylon headers.
#include <pylon/PylonVersionNumber.h>
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/usb/_BaslerUsbCameraParams.h>
// ROS headers.
#include <ros/ros.h>


namespace ar_demo {

  class SimulatedCameraAdapter: public CameraAdapter {
    private:
      bool camera_is_initialised_;
      std::string camera_name_;
      std::string camera_identification_;
			ImageMode image_mode_;
      bool camera_is_streaming_;
			std::string replay_video_;
			cv::VideoCapture video_capture_;
			ros::Duration frame_interval_;
			ros::Time frame_previous_;

    public:
      SimulatedCameraAdapter(std::shared_ptr<const Configuration> configuration, const std::string & replay_video);
      virtual ~SimulatedCameraAdapter();
      
      virtual bool initialise(const std::string & camera_name, const std::string & model,
          const std::string & serial_number, const std::string & user_defined_name, ImageMode image_mode);
      virtual bool isInitialised();
      virtual std::string getCameraIdentification();

      virtual bool setManualExposure(double time_s, double gain_db);
      virtual bool isAutoExposureSupported();
      virtual bool isAutoExposureRoiSupported();

      virtual bool setManualWhiteBalance(double red, double green, double blue);
      virtual bool isAutoWhiteBalanceSupported();
      virtual bool isAutoWhiteBalanceRoiSupported();

      virtual bool setFrameRate(double frame_rate_hz);

      virtual bool startStreaming();
      virtual bool stopStreaming();
      virtual bool isStreaming();
      virtual bool getImage(cv::Mat & image, ImageInfo & image_info);

      virtual bool isCameraAttached();
      virtual bool isCameraRemoved();
  };

}

#endif // __AR_DEMO__CAMERA_ADAPTER_SIMULATED
