//! Unified camera node - camera.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera.hpp"
#include "camera_adapter.hpp"
#include "camera_adapter_basler_pylon_usb.hpp"
#include "camera_adapter_basler_pylon_gige.hpp"
// Boost headers.
#include <boost/date_time/posix_time/posix_time.hpp>
// OpenCV headers.
#include <opencv2/highgui/highgui.hpp>
// ROS headers.
#include <cv_bridge/cv_bridge.h>


namespace ar_demo {

  Camera::Camera(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle,
			const std::string & camera_name):
		configuration_{configuration},
		node_handle_{node_handle},
    camera_name_{camera_name},
    image_transport_{node_handle_},
		camera_driver_{nullptr},
    camera_is_initialised_{false},
    camera_is_streaming_{false}
  {
		// Select camera driver.
		auto camera_driver_name = configuration_->getCameraDriver(camera_name);
		if(camera_driver_name.second == "BaslerPylonGige" || camera_driver_name.second == "BaslerPylonGigE") {
			camera_driver_ = new ar_demo::BaslerPylonGigECameraAdapter(configuration);
		}
		else if(camera_driver_name.second == "BaslerPylonUsb") {
			camera_driver_ = new ar_demo::BaslerPylonUsbCameraAdapter(configuration);
		}
		else {
			if(!camera_driver_name.first) {
				ROS_FATAL("No driver specified for camera '%s'.", camera_name.c_str());
				throw std::runtime_error("No camera driver specified");
			}
			else {
				ROS_FATAL("Camera driver '%s' specified for camera '%s' is not supported.", camera_driver_name.second.c_str(),
						camera_name.c_str());
				throw std::runtime_error("Unsupported camera driver specified");
			}
		}
    // Get hardware information from parameter server.
		auto model = configuration_->getCameraModel(camera_name);
		auto serial_number = configuration_->getCameraSerialNumber(camera_name);
		auto user_defined_name = configuration_->getCameraUserDefinedName(camera_name);
		auto image_mode_string = configuration_->getCameraImageMode(camera_name);
		if(image_mode_string.first == false) {
			ROS_WARN("No mode specified, configuring camera '%s' as a color camera.", camera_name_.c_str());
			image_mode_string.second = "color";
		}
    CameraAdapter::ImageMode image_mode;
    if(image_mode_string.second == "color" || image_mode_string.second == "colour") {
      image_mode = CameraAdapter::ImageMode::BGR888;
    }
    else if(image_mode_string.second == "gray" || image_mode_string.second == "grey"
				|| image_mode_string.second == "mono") {
      image_mode = CameraAdapter::ImageMode::GRAY8;
    }
    else {
      ROS_FATAL("Unknown image mode '%s' specified.", image_mode_string.second.c_str());
			throw std::runtime_error("Unsupported image mode specified");
      return;
    }
    // Initialise camera driver.
    if(camera_driver_->initialise(camera_name_, model.second, serial_number.second, user_defined_name.second,
					image_mode) == false
				|| camera_driver_->isInitialised() == false) {
      ROS_FATAL("An error occured during the initialisation of camera '%s' by the camera driver",
          camera_name_.c_str());
			throw std::runtime_error("Unable to initialise camera");
    }

    // Initialise camera.
		framerate_ = configuration_->getCameraFrameRate(camera_name_).second;
    camera_driver_->setFrameRate(framerate_);
    getSettingsAndConfigureCamera();

    // Initialise CameraInfoManager.
    std::string camera_info_url{configuration_->getCameraInfoUrl(camera_name_).second};
		std::string camera_identification{camera_name_};
    std::transform(camera_identification.begin(), camera_identification.end(), camera_identification.begin(),
        [](char c) {return isalnum(c)? tolower(c): '_';});
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
					node_handle_, camera_identification, camera_info_url));

    // Advertise camera stream topics (using ROS image_transport to support compressed camera streams).
    image_publisher_ = image_transport_.advertiseCamera("stream", ROS_IMAGE_QUEUE_SIZE);
    image_info_publisher_ = node_handle_.advertise<ImageInfo>("image_info", 1);
    // Subscribe to the camera control topic.
    control_subscriber_ = node_handle_.subscribe("control", 1, &Camera::ControlCallback, this);
    // Advertise camera info service.
    camera_info_service_ = node_handle_.advertiseService("get_camera_info", &Camera::GetCameraInfoCallback, this);
    // Advertise still image service.
    image_service_ = node_handle_.advertiseService("get_image", &Camera::GetImageCallback, this);
    image_compressed_service_ = node_handle_.advertiseService("get_image_compressed",
				&Camera::GetImageCompressedCallback, this);

    // Done.
    camera_is_initialised_ = true;
  }

  Camera::~Camera() {
    delete camera_driver_;
  }

  bool Camera::isInitialised() {
    return camera_is_initialised_;
  }

  bool Camera::getSettingsAndConfigureCamera() {
		auto ae = configuration_->getCameraAutoExposureEnabled(camera_name_);
		auto awb = configuration_->getCameraAutoWhiteBalanceEnabled(camera_name_);
    if(ae.second && !camera_driver_->isAutoExposureSupported()) {
      ROS_WARN("Automatic exposure requested but not supported by camera driver for camera '%s'",
          camera_name_.c_str());
      ae.second = false;
    }
    if(awb.second && !camera_driver_->isAutoWhiteBalanceSupported()) {
      ROS_WARN("Automatic white balance requested but not supported by camera driver for camera '%s'",
          camera_name_.c_str());
      awb.second = false;
    }
    if(ae.second) {
      ROS_INFO("Automatic exposure enabled for camera '%s'", camera_name_.c_str());
      if(camera_driver_->isAutoExposureRoiSupported()) {
				auto ae_roi = configuration_->getCameraAutoExposureRoi(camera_name_);
        camera_driver_->setAutoExposureRoi(ae_roi.second[0], ae_roi.second[1], ae_roi.second[2], ae_roi.second[3]);
      }
      ae_target_brightness_ = configuration_->getCameraAutoTargetBrightness(camera_name_).second;
      ae_time_min_ = configuration_->getCameraAutoMinimumExposure(camera_name_).second;
      ae_time_max_ = configuration_->getCameraAutoMaximumExposure(camera_name_).second;
			ae_gain_min_ = configuration_->getCameraAutoMinimumGain(camera_name_).second;
			ae_gain_max_ = configuration_->getCameraAutoMaximumGain(camera_name_).second;
      camera_driver_->setAutoExposure(ae_target_brightness_, ae_time_min_, ae_time_max_, ae_gain_min_, ae_gain_max_);
    }
    else {
      ROS_INFO("Manual exposure enabled for camera '%s'", camera_name_.c_str());
      auto exposure = configuration_->getCameraManualExposure(camera_name_);
			auto gain = configuration_->getCameraManualGain(camera_name_);
      camera_driver_->setManualExposure(exposure.second, gain.second);
    }
    if(awb.second) {
      ROS_INFO("Automatic white balance enabled for camera '%s'", camera_name_.c_str());
      if(camera_driver_->isAutoWhiteBalanceRoiSupported()) {
				auto awb_roi = configuration_->getCameraAutoWhiteBalanceRoi(camera_name_);
        camera_driver_->setAutoWhiteBalanceRoi(awb_roi.second[0], awb_roi.second[1], awb_roi.second[2],
						awb_roi.second[3]);
      }
      camera_driver_->setAutoWhiteBalance();
    }
    else {
      ROS_INFO("Manual white balance enabled for camera '%s'", camera_name_.c_str());
			auto balance = configuration_->getCameraManualWhiteBalance(camera_name_);
      camera_driver_->setManualWhiteBalance(balance.second[0], balance.second[1], balance.second[2]);
    }
  }

  void Camera::spin() {
    ros::Rate loop_rate(framerate_);
    bool camera_removed = false;
    while(node_handle_.ok() && !(camera_removed = camera_driver_->isCameraRemoved())) {
      if(camera_is_streaming_) {
        // Get image.
        cv::Mat image;
        ImageInfo image_info;
        if(camera_driver_->getImage(image, image_info)) {
          // Publish image and image metadata.
          sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(image_info.header, "bgr8", image).toImageMsg();
          sensor_msgs::CameraInfoPtr msg_camera_info(new sensor_msgs::CameraInfo(
                camera_info_manager_->getCameraInfo()));
          msg_camera_info->header = image_info.header;
          image_publisher_.publish(msg_image, msg_camera_info);
          image_info_publisher_.publish(image_info);
          // Cache this image and the corresponding metadata and camera info.
          boost::lock_guard < boost::shared_mutex > lock { cache_image_mutex_ };
          cached_image_ = image;
          cached_image_timestamp_ = image_info.header.stamp;
          cached_image_camera_info_ = camera_info_manager_->getCameraInfo();
          cached_image_info_ = image_info;
        }
        else {
          ROS_WARN("Failed getting image from camera '%s', ignoring frame.", camera_name_.c_str());
        }
      }
      ros::spinOnce();
      loop_rate.sleep(); // Decrease CPU usage when not streaming.
    }
    if(camera_removed) {
      ROS_ERROR("Camera '%s' is removed. Terminating camera node.", camera_name_.c_str());
    }
  }

  void Camera::ControlCallback(const ControlCamera::ConstPtr & message) {
    if(message->stream == true) {
      if(message->reset_to_default_settings) {
        getSettingsAndConfigureCamera();
      }
      else {
        ae_target_brightness_ *= message->adjust_target_brightness;
        camera_driver_->setAutoExposure(ae_target_brightness_, ae_time_min_, ae_time_max_, ae_gain_min_, ae_gain_max_);
      }
      if(message->reference == "") {
        ROS_WARN("Camera started without a reference. Other clients can turn off the camera.");
      }
      else {
        camera_active_references_.insert(message->reference);
      }
      if(!camera_is_streaming_) {
        camera_driver_->startStreaming();
        camera_is_streaming_ = true;
      }
      ROS_INFO("Camera '%s' started streaming.", camera_name_.c_str());
    }
    else /* (msg->stream == false) */ {
      if (message->reference == "") {
        ROS_WARN("Camera stopped without a reference. Did you forget it?");
      }
      else {
        if(camera_active_references_.erase(message->reference) == 0) {
          ROS_ERROR("Camera stopped with an unknown reference.");
        }
      }
      if (camera_is_streaming_ && camera_active_references_.empty()) {
        camera_driver_->stopStreaming();
        camera_is_streaming_ = false;
        ROS_INFO("Camera '%s' stopped streaming.", camera_name_.c_str());
      }
    }
  }

  bool Camera::GetCameraInfoCallback(GetCameraInfo::Request& request, GetCameraInfo::Response& response) {
    response.camera_info = camera_info_manager_->getCameraInfo();
		// If the camera calibration was not found, make the call fail.
		if(response.camera_info.width * response.camera_info.height == 0) {
			return false;
		}
    return true;
  }

  bool Camera::GetImageCallback(GetImage::Request & request, GetImage::Response & response) {
    boost::shared_lock < boost::shared_mutex > lock { cache_image_mutex_ };
    std_msgs::Header header;
    header.stamp = cached_image_timestamp_;
    cv_bridge::CvImage(header, "bgr8", cached_image_).toImageMsg(response.image);
    response.camera_info = cached_image_camera_info_;
    response.image_info = cached_image_info_;
    ROS_INFO("Uncompressed image service returned image with original timestamp %s",
        boost::posix_time::to_simple_string(cached_image_timestamp_.toBoost()).c_str());
    return true;
  }

  bool Camera::GetImageCompressedCallback(GetImageCompressed::Request & request,
      GetImageCompressed::Response & response) {
    boost::shared_lock < boost::shared_mutex > lock { cache_image_mutex_ };
    response.image.header.stamp = cached_image_timestamp_;
    response.image.format = "jpeg";
    std::vector<uchar> buffer;
    std::vector<int> parameters { cv::IMWRITE_JPEG_QUALITY, 80 }; // TODO: arbitrary constant.
    cv::imencode(".jpeg", cached_image_, buffer, parameters);
    response.image.data = std::move(buffer); // We don't need the buffer anymore.
    response.camera_info = cached_image_camera_info_;
    response.image_info = cached_image_info_;
    ROS_INFO("Compressed image service returned image with original timestamp %s",
        boost::posix_time::to_simple_string(cached_image_timestamp_.toBoost()).c_str());
    return true;
  }

}
