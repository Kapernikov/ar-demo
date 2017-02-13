//!ArucoFacade.
/*!
 *  \file
 *  Source: http://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
 *
 *  \author Pieter Langsweirdt <pieter@kapernikov.com>
 */

#ifndef __AR_DEMO__ARUCO_FACADE
#define __AR_DEMO__ARUCO_FACADE


#include "../configuration/configuration.hpp"
// C++ headers.
#include <map>
#include <memory>
#include <string>
#include <vector>
// OpenCV headers
#include <opencv2/core/core.hpp>
// ROS headers.
#include <ros/ros.h>
#include <ar_demo/ArucoResults.h>
// ArUco headers.
#include <aruco/aruco.h>
#include <aruco/highlyreliablemarkers.h>


namespace ar_demo {

	class ArucoFacade {

		private:
			enum UrlType {
				URL_INVALID = 0,
				URL_FILE,
				URL_PACKAGE
			};
			std::shared_ptr<const Configuration> configuration_;
			ros::NodeHandle node_handle_;
			std::string camera_name_;
			aruco::CameraParameters camera_parameters_;
			ArucoResults aruco_results_;
			std::map<string, aruco::Dictionary> dictionary_map_;
			std::map<string, aruco::BoardDetector> board_detector_map_;

			aruco::MarkerDetector MDetector;
			std::vector<aruco::Marker> Markers;

			UrlType ParseUrl(const std::string & resolvedUrl);
			std::string ResolveUrl(const std::string & cs_in, const std::string & cs_out);
			std::string ResolveFilename(const std::string & resolvedUrl);

		public:
			ArucoFacade(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle);
			virtual ~ArucoFacade();
			bool initialise(const std::string & camera_name, const std::vector<std::string> & requested_markers);
			void reset();
			void detect(cv::Mat image);
			void detect(cv::Mat InImage, ros::Time frameTimestamp);
			ArucoResults retrieveResult();
			bool hasValidResult();

	};
}

#endif
