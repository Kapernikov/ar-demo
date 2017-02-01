//! Unified camera node - camera.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __AR_DEMO__CAMERA
#define __AR_DEMO__CAMERA


#define ROS_IMAGE_QUEUE_SIZE 1


// Standard C++ header files.
#include <memory>
#include <string>
#include <unordered_set>
// Boost headers.
#include <boost/thread/shared_mutex.hpp> // NOTE: Use C++14 version when available.
// OpenCV headers.
#include <opencv2/core/core.hpp>
// ROS headers.
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ar_demo/ControlCamera.h>
#include <ar_demo/GetCameraInfo.h>
#include <ar_demo/GetImage.h>
#include <ar_demo/GetImageCompressed.h>
#include <ar_demo/ImageInfo.h>
#include "../configuration/configuration.hpp"


namespace ar_demo {

	class CameraAdapter;

  class Camera {
    private:

			std::shared_ptr<const Configuration> configuration_;
      CameraAdapter * camera_driver_;

      std::string camera_name_;
      bool camera_is_initialised_;
      bool camera_is_streaming_;
      std::unordered_set<std::string> camera_active_references_;
      double framerate_;
      double ae_target_brightness_;
      double ae_time_min_;
      double ae_time_max_;
      double ae_gain_min_;
      double ae_gain_max_;

      ros::NodeHandle node_handle_;
      image_transport::ImageTransport image_transport_;
      ros::Subscriber control_subscriber_;
      image_transport::CameraPublisher image_publisher_;
      ros::Publisher image_info_publisher_;
      std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
      ros::ServiceServer camera_info_service_;
      ros::ServiceServer image_service_;
      ros::ServiceServer image_compressed_service_;
      
      boost::shared_mutex cache_image_mutex_;
      ros::Time cached_image_timestamp_;
      cv::Mat cached_image_;
      sensor_msgs::CameraInfo cached_image_camera_info_;
      ImageInfo cached_image_info_;

    public:

      Camera(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle,
          const std::string & camera_name);
      ~Camera();
      bool isInitialised();
      bool getSettingsAndConfigureCamera();

      void spin();

      void ControlCallback(const ControlCamera::ConstPtr& msg);

      bool GetCameraInfoCallback(GetCameraInfo::Request& request, GetCameraInfo::Response& response);
      bool GetImageCallback(GetImage::Request & request, GetImage::Response & response);
      bool GetImageCompressedCallback(GetImageCompressed::Request & request, GetImageCompressed::Response & response);
  };

}

#endif // __AR_DEMO__CAMERA
