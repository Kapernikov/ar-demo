//! Monitor relative position of an ArUco board relative to another ArUco board. 
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __AR_DEMO__ARUCO_RELATIVE
#define __AR_DEMO__ARUCO_RELATIVE


#include "../configuration/configuration.hpp"
#include "aruco_facade.hpp"
// Standard C++ headers.
#include <string>
// Boost headers.
#include <boost/numeric/ublas/matrix.hpp>
// ROS headers.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
// 0MQ headers.
#include <zmq.hpp>


namespace ar_demo {

	class ArUcoRelative {

		private:

			bool is_initialised_;

			std::shared_ptr<const Configuration> configuration_;
			ros::NodeHandle node_handle_;
			image_transport::ImageTransport image_transport_;
			image_transport::Subscriber subscription_camera_stream_;

			std::string camera_name_;
			sensor_msgs::CameraInfo camera_info_;
			std::string reference_board_name_;
			std::string moving_board_name_;
			ArucoFacade aruco_facade_;

			zmq::context_t zmq_context_;
			zmq::socket_t zmq_publisher_;
		public:

			ArUcoRelative(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle,
					const std::string & camera_name, const std::string & reference_board_name,
					const std::string & moving_board_name);
			~ArUcoRelative();
			bool isInitialised();

		private:

			void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg);
			template<typename T> bool InvertMatrix(const boost::numeric::ublas::matrix<T> & input,
					boost::numeric::ublas::matrix<T> & output);

	};

}


#endif // __AR_DEMO__ARUCO_RELATIVE
