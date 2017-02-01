//! Unified camera node - Basler Pylon GigE adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __AR_DEMO__CAMERA_ADAPTER_BASLER_PYLON_GIGE
#define __AR_DEMO__CAMERA_ADAPTER_BASLER_PYLON_GIGE


#include "camera_adapter.hpp"

// Standard C++ headers.
#include <string>
// OpenCV headers.
#include <opencv2/core/core.hpp>
// Basler Pylon headers.
#include <pylon/PylonVersionNumber.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/_BaslerGigECameraParams.h>
// ROS headers.
#include <ros/ros.h>


namespace ar_demo {

  class BaslerPylonGigECameraAdapter: public CameraAdapter {
    private:
      bool camera_is_initialised_;
      std::string camera_name_;
      std::string camera_identification_;
      ros::Duration image_transmission_time_;
      bool camera_is_streaming_;
      double aperture_;
      double ev_correction_;

      Pylon::PylonAutoInitTerm pait_;
      Pylon::CBaslerGigEInstantCamera * camera_;
      Pylon::CImageFormatConverter format_converter_;

    public:
      BaslerPylonGigECameraAdapter(std::shared_ptr<const Configuration> configuration);
      virtual ~BaslerPylonGigECameraAdapter();
      
      virtual bool initialise(const std::string & camera_name, const std::string & model,
          const std::string & serial_number, const std::string & user_defined_name, ImageMode image_mode);
      virtual bool isInitialised();
      virtual std::string getCameraIdentification();

      virtual bool setManualExposure(double time_s, double gain_db);
      virtual bool isAutoExposureSupported();
      virtual bool setAutoExposure(double target_brightness, double time_s_min, double time_s_max,
          double gain_db_min, double gain_db_max);
      virtual bool isAutoExposureRoiSupported();
      virtual bool setAutoExposureRoi(int column, int row, int width, int height);

      virtual bool setManualWhiteBalance(double red, double green, double blue);
      virtual bool isAutoWhiteBalanceSupported();
      virtual bool setAutoWhiteBalance();
      virtual bool isAutoWhiteBalanceRoiSupported();
      virtual bool setAutoWhiteBalanceRoi(int column, int row, int width, int height);

      virtual bool setFrameRate(double frame_rate_hz);

      virtual bool startStreaming();
      virtual bool stopStreaming();
      virtual bool isStreaming();
      virtual bool getImage(cv::Mat & image, ImageInfo & image_info);

      virtual bool isCameraAttached();
      virtual bool isCameraRemoved();
  };

}

#endif // __AR_DEMO__CAMERA_ADAPTER_BASLER_PYLON_GIGE
