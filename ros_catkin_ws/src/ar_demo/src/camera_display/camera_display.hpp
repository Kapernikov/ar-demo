//! Camera display node - camera display.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


// Standard C++ headers.
#include <memory>
#include <string>
// OpenCV headers.
#include <opencv2/highgui/highgui.hpp>
// ROS headers.
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "../configuration/configuration.hpp"


namespace ar_demo {

	class CameraDisplay {
		private:
			std::shared_ptr<const Configuration> configuration_;
			ros::NodeHandle node_handle_;
			std::string camera_name_;
			double image_reduction_factor_;
			bool show_ae_roi_;
			bool show_awb_roi_;
			image_transport::ImageTransport image_transport_;
			image_transport::Subscriber image_subscriber_;
			std::string opencv_window_;

		public:
			CameraDisplay(std::shared_ptr<const Configuration> configuration, const ros::NodeHandle & node_handle,
					const std::string & camera_name, double image_reduction_factor, bool show_ae_roi, bool show_awb_roi);
			~CameraDisplay();
			void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	};

}
