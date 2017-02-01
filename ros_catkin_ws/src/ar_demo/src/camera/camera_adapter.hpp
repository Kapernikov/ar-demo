//! Unified camera node - camera driver adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __AR_DEMO__CAMERA_ADAPTER
#define __AR_DEMO__CAMERA_ADAPTER


// Standard C++ headers.
#include <memory>
#include <string>
// OpenCV headers.
#include <opencv2/core/core.hpp>
// ROS headers.
#include <ar_demo/ImageInfo.h>
#include "../configuration/configuration.hpp"


namespace ar_demo {

  class CameraAdapter {
		protected:
			std::shared_ptr<const Configuration> configuration_;

    public:
      enum class ImageMode { GRAY8, BGR888 };

      CameraAdapter(std::shared_ptr<const Configuration> configuration);
      virtual ~CameraAdapter();
      
      virtual bool initialise(const std::string & camera_name, const std::string & model,
          const std::string & serial_number, const std::string & user_defined_name, ImageMode image_mode) = 0;
      virtual bool isInitialised() = 0;
      virtual std::string getCameraIdentification() = 0;

      virtual bool setManualExposure(double time_s, double gain_db) = 0;
      virtual bool isAutoExposureSupported();
      virtual bool setAutoExposure(double target_brightness, double time_s_min, double time_s_max,
          double gain_db_min, double gain_db_max);
      virtual bool isAutoExposureRoiSupported();
      virtual bool setAutoExposureRoi(int column, int row, int width, int height);

      virtual bool setManualWhiteBalance(double red, double green, double blue) = 0;
      virtual bool isAutoWhiteBalanceSupported();
      virtual bool setAutoWhiteBalance();
      virtual bool isAutoWhiteBalanceRoiSupported();
      virtual bool setAutoWhiteBalanceRoi(int column, int row, int width, int height);

      virtual bool setFrameRate(double frame_rate_hz) = 0;

      virtual bool startStreaming() = 0;
      virtual bool stopStreaming() = 0;
      virtual bool isStreaming() = 0;
      virtual bool getImage(cv::Mat & image, ImageInfo & image_info) = 0;

      virtual bool isCameraAttached() = 0;
      virtual bool isCameraRemoved() = 0;
  };

}

#endif // __AR_DEMO__CAMERA_ADAPTER
