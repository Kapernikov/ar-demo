//! Unified camera node - simulated camera adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_adapter_simulated.hpp"


namespace ar_demo {

  SimulatedCameraAdapter::SimulatedCameraAdapter(std::shared_ptr<const Configuration> configuration,
			const std::string & replay_video):
		CameraAdapter(configuration),
    camera_is_initialised_{false},
    camera_name_{""},
    camera_identification_{""},
		image_mode_{ImageMode::BGR888},
    camera_is_streaming_{false},
		replay_video_{replay_video},
		frame_interval_{1. / 24.},
		frame_previous_{0.} {
    ROS_INFO("Using simulated camera adapter.");
  }

  SimulatedCameraAdapter::~SimulatedCameraAdapter() {
		;
  }

  bool SimulatedCameraAdapter::initialise(const std::string & camera_name, const std::string & model,
      const std::string & serial_number, const std::string & user_defined_name, ImageMode image_mode) {
    camera_is_initialised_ = false;
    camera_name_ = "";
		camera_identification_ = model + "_" + serial_number;
		image_mode_ = image_mode;
		ROS_INFO("Simulating camera with model name '%s', serial number '%s' and user defined name '%s'.",
				model.c_str(), serial_number.c_str(), user_defined_name.c_str());
    try {
			// Open video file.
			video_capture_.open(replay_video_);
			if(!video_capture_.isOpened()) {
				ROS_ERROR("Unable to open video file '%s'.", replay_video_.c_str());
				throw std::runtime_error("Unable to open video file");
			}
      camera_name_ = camera_name;
      camera_is_initialised_ = true;
      return true;
    }
    catch(std::exception & exception) {
      ROS_ERROR("An error occured during the initialisation of the camera.");
      return false;
    }
  }

  bool SimulatedCameraAdapter::isInitialised() {
    return camera_is_initialised_;
  }

  std::string SimulatedCameraAdapter::getCameraIdentification() {
    if(camera_is_initialised_) {
      return camera_identification_;
    }
    else {
      return "";
    }
  }

  bool SimulatedCameraAdapter::setManualExposure(double time_s, double gain_db) {
		return false;
	}

  bool SimulatedCameraAdapter::isAutoExposureSupported() {
    return false;
  }

  bool SimulatedCameraAdapter::isAutoExposureRoiSupported() {
    return false;
  }

  bool SimulatedCameraAdapter::setManualWhiteBalance(double red, double green, double blue) {
		return false;
	}

  bool SimulatedCameraAdapter::isAutoWhiteBalanceSupported() {
    return false;
  }

  bool SimulatedCameraAdapter::isAutoWhiteBalanceRoiSupported() {
    return false;
  }

  bool SimulatedCameraAdapter::setFrameRate(double frame_rate_hz) {
    if(camera_is_initialised_) {
			frame_interval_ = ros::Duration(1. / frame_rate_hz);
			ROS_INFO("Framerate of camera '%s' set to  %.2f fps.", camera_name_.c_str(), frame_rate_hz);
			return true;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::startStreaming() {
    if(camera_is_initialised_) {
			frame_previous_ = ros::Time::now() - frame_interval_; // We want the next frame right now.
      camera_is_streaming_ = true;
      ROS_INFO("Camera '%s' started grabbing and streaming.", camera_name_.c_str());
      return true;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::stopStreaming() {
    if(camera_is_initialised_) {
			camera_is_streaming_ = false;
      ROS_INFO("Camera '%s' stopped grabbing and streaming.", camera_name_.c_str());
			return true;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::isStreaming() {
    if(camera_is_initialised_) {
			return camera_is_streaming_;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::getImage(cv::Mat & image, ImageInfo & image_info) {
    if(camera_is_initialised_) {
			cv::Mat image_captured;
			ros::Time frame_current = frame_previous_;
			// Get image, skip images we didn't receive.
			do {
				if(!video_capture_.read(image_captured)) {
					camera_is_streaming_ = false;
					camera_is_initialised_ = false; // Handle end of movie as a detached camera.
					return false;
				}
				frame_current += frame_interval_;
			} while(frame_current + frame_interval_ < ros::Time::now());
			// Convert to grayscale if requested. Assume video is in color, sincs OpenCV 2.4 doesn't support grayscale videos on Linux.
			if(image_mode_ == ImageMode::BGR888) {
				image = image_captured;
			}
			else if(image_mode_ == ImageMode::GRAY8) {
				cv::Mat image_grayscale(image_captured.size(), CV_8UC1);
				cv::cvtColor(image_captured, image_grayscale, CV_BGR2GRAY);
				image = image_grayscale;
			}
			// Set image info (we don't know anything about the image).
			image_info.header.stamp = frame_current;
			image_info.exposure_us = std::numeric_limits<double>::quiet_NaN();
			image_info.aperture = std::numeric_limits<double>::quiet_NaN();
			image_info.gain_db = std::numeric_limits<double>::quiet_NaN();
			image_info.wb_red = std::numeric_limits<double>::quiet_NaN();
			image_info.wb_green = std::numeric_limits<double>::quiet_NaN();
			image_info.wb_blue = std::numeric_limits<double>::quiet_NaN();
			image_info.estimated_illuminance_lux = std::numeric_limits<double>::quiet_NaN();
			// If we called the function before the frame should be available, wait until it should be available.
			if(frame_current > ros::Time::now()) {
				ros::Duration(ros::Time::now() - frame_current).sleep();
			}
			frame_previous_ = frame_current;
			return true;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::isCameraAttached() {
    if(camera_is_initialised_) {
      return true;
    }
    else {
      return false;
    }
  }

  bool SimulatedCameraAdapter::isCameraRemoved() {
    if(camera_is_initialised_) {
      return false;
    }
    else {
      return true;
    }
  }

}
