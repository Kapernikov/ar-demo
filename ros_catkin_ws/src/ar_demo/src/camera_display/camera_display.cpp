//! Camera display node - camera display.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_display.hpp"
// Standard C++ headers.
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
// OpenCV headers.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS headers.
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


namespace ar_demo {

	CameraDisplay::CameraDisplay(std::shared_ptr<const Configuration> configuration, const ros::NodeHandle & node_handle,
			const std::string & camera_name, double image_reduction_factor, bool show_ae_roi, bool show_awb_roi):
		configuration_{configuration},
		node_handle_{node_handle},
		camera_name_{camera_name},
		image_reduction_factor_{image_reduction_factor},
		show_ae_roi_{show_awb_roi},
		show_awb_roi_{show_awb_roi},
		opencv_window_{"'" + camera_name + "' monitor"},
		image_transport_{node_handle_},
		image_subscriber_{image_transport_.subscribe("/camera/" + camera_name + "/stream", 1, &CameraDisplay::imageCallback,
				this)} {
		cv::namedWindow(opencv_window_);
	}


	CameraDisplay::~CameraDisplay() {
		cv::destroyWindow(opencv_window_);
	}


	void CameraDisplay::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
		// Get OpenCV image from cv_bridge.
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg); // Use image encoding of source.
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat original = cv_ptr->image;

		// Show AE and AWB ROI
		if(show_ae_roi_) {
			auto ae_roi = configuration_->getCameraAutoExposureRoi(camera_name_);
			if(ae_roi.first) {
				cv::rectangle(original, cv::Point(ae_roi.second[0], ae_roi.second[1]),
						cv::Point(ae_roi.second[0] + ae_roi.second[2], ae_roi.second[1] + ae_roi.second[3]),
						cv::Scalar(255, 127, 0), std::ceil(image_reduction_factor_), 1, 0);
			}
		}
		if(show_awb_roi_) {
			auto awb_roi = configuration_->getCameraAutoWhiteBalanceRoi(camera_name_);
			if(awb_roi.first) {
				cv::rectangle(original, cv::Point(awb_roi.second[0], awb_roi.second[1]),
						cv::Point(awb_roi.second[0] + awb_roi.second[2], awb_roi.second[1] + awb_roi.second[3]),
						cv::Scalar(255, 0, 127), std::ceil(image_reduction_factor_), 1, 0);
			}
		}

		// Resize image.
		cv::Mat resized;
		if(image_reduction_factor_ != 1.) {
			cv::resize(original, resized, cv::Size(), 1. / image_reduction_factor_, 1. / image_reduction_factor_,
					cv::INTER_AREA);
		}
		else {
			resized = original;
		}
		// Display image.
		cv::imshow(opencv_window_, resized);
		cv::waitKey(20);
	}

}
