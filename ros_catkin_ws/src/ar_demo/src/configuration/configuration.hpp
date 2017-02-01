//! Configuration.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 *  \author Pieter Langsweirdt <pieter@kapernikov.com>
 */


#if !defined __AR_DEMO__CONFIGURATION
#define __AR_DEMO__CONFIGURATION


// C++ headers.
#include <map>
#include <set>
#include <string>
// ROS headers.
#include <ros/ros.h>


namespace ar_demo {

	class Configuration {
		protected:
			ros::NodeHandle node_handle_;

			template<typename T> inline std::pair<bool, T> getParameter(
					const std::string & parameter_name, const T & parameter_value_default) const {
				bool parameter_defined{node_handle_.hasParam(parameter_name)};
				T parameter_value{parameter_value_default};
				if(parameter_defined) {
					node_handle_.getParam(parameter_name, parameter_value);
				}
				return std::make_pair(parameter_defined, parameter_value);
			};

		public:
			Configuration(const ros::NodeHandle & node_handle);
			virtual ~Configuration();

			std::pair<bool, std::string> getCameraInfoUrl(const std::string & camera_name) const;
			std::set<std::string> getConfiguredCameras() const;
			std::pair<bool, std::string> getCameraDriver(const std::string & camera_name) const;
			std::pair<bool, std::string> getCameraModel(const std::string & camera_name) const;
			std::pair<bool, std::string> getCameraSerialNumber(const std::string & camera_name) const;
			std::pair<bool, std::string> getCameraUserDefinedName(const std::string & camera_name) const;
			std::pair<bool, std::string> getCameraImageMode(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAperture(const std::string & camera_name) const;
			std::pair<bool, double> getCameraExposureValueCorrection(const std::string & camera_name) const;
			std::pair<bool, double> getCameraImageTransmissionTime(const std::string & camera_name) const;
			std::pair<bool, double> getCameraFrameRate(const std::string & camera_name) const;
			std::pair<bool, bool> getCameraAutoExposureEnabled(const std::string & camera_name) const;
			std::pair<bool, std::array<int, 4>> getCameraAutoExposureRoi(const std::string & camera_name) const;
			bool setCameraAutoExposureRoi(const std::string & camera_name, const std::array<int, 4> & roi);
			std::pair<bool, double> getCameraAutoTargetBrightness(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAutoMinimumExposure(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAutoMaximumExposure(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAutoMinimumGain(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAutoMaximumGain(const std::string & camera_name) const;
			std::pair<bool, bool> getCameraAutoWhiteBalanceEnabled(const std::string & camera_name) const;
			std::pair<bool, std::array<int, 4>> getCameraAutoWhiteBalanceRoi(const std::string & camera_name) const;
			bool setCameraAutoWhiteBalanceRoi(const std::string & camera_name, const std::array<int, 4> & roi);
			std::pair<bool, double> getCameraManualExposure(const std::string & camera_name) const;
			std::pair<bool, double> getCameraManualGain(const std::string & camera_name) const;
			std::pair<bool, std::array<double, 3>> getCameraManualWhiteBalance(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAcceptableMinimumIlluminance(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAcceptableMaximumIlluminance(const std::string & camera_name) const;
			std::pair<bool, double> getCameraAcceptableMaximumGain(const std::string & camera_name) const;

			std::pair<bool, std::string> getArucoBoardDefinitionUrl() const;
			std::pair<bool, std::string> getArucoBoardName(const std::string & board_alias) const;
			std::pair<bool, double> getArucoMarkerSize(const std::string & board_alias) const;
	};

}


#endif
