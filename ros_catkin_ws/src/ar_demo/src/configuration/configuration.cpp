//! Configuration.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 *  \author Pieter Langsweirdt <pieter@kapernikov.com>
 */


#include "configuration.hpp"


namespace ar_demo {

	Configuration::Configuration(const ros::NodeHandle & node_handle):
		node_handle_{node_handle} {
			;
	}


	Configuration::~Configuration() {
		;
	}


	std::pair<bool, std::string> Configuration::getCameraInfoUrl(const std::string & camera_name) const {
		std::string camera_info_url{"file://${ROS_HOME}/calibrations/camera_${NAME}.yaml"};
		if(camera_name != "" && node_handle_.getParam("/camera/" + camera_name + "/camera_info_url", camera_info_url)) {
			return std::make_pair(true, camera_info_url);
		}
		else if(node_handle_.getParam("/camera/camera_info_url", camera_info_url)) {
			return std::make_pair(true, camera_info_url);
		}
		else if(node_handle_.getParam("/camera_info_url", camera_info_url)) {
			return std::make_pair(true, camera_info_url);
		}
		else {
			return std::make_pair(false, camera_info_url);
		}
	}


	std::set<std::string> Configuration::getConfiguredCameras() const {
		std::set<std::string> configured_cameras;
		XmlRpc::XmlRpcValue cameras;
		node_handle_.getParam("/camera", cameras);
		if(cameras.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(auto & camera: cameras) {
				if(camera.second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
					configured_cameras.insert(camera.first);
				}
			}
		}
		return configured_cameras;
	}


	std::pair<bool, std::string> Configuration::getCameraDriver(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<std::string>("/camera/" + camera_name + "/hardware/driver", "unknown");
		}
		else {
			return std::make_pair(false, "unknown");
		}
	}


	std::pair<bool, std::string> Configuration::getCameraModel(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<std::string>("/camera/" + camera_name + "/hardware/model", "");
		}
		else {
			return std::make_pair(false, "");
		}
	}


	std::pair<bool, std::string> Configuration::getCameraSerialNumber(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<std::string>("/camera/" + camera_name + "/hardware/serialnumber", "");
		}
		else {
			return std::make_pair(false, "");
		}
	}


	std::pair<bool, std::string> Configuration::getCameraUserDefinedName(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<std::string>("/camera/" + camera_name + "/hardware/userdefinedname", "");
		}
		else {
			return std::make_pair(false, "");
		}
	}


	std::pair<bool, std::string> Configuration::getCameraImageMode(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<std::string>("/camera/" + camera_name + "/hardware/mode", "color");
		}
		else {
			return std::make_pair(false, "color");
		}
	}


	std::pair<bool, double> Configuration::getCameraAperture(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/hardware/aperture", 2.8);
	}


	std::pair<bool, double> Configuration::getCameraExposureValueCorrection(
			const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/hardware/ev_correction", 0.);
	}


	std::pair<bool, double> Configuration::getCameraImageTransmissionTime(const std::string & camera_name)
		const {
		auto transmission_time = getParameter<double>(
				"/camera/" + camera_name + "/hardware/image_transmission_us", 0.); // Fetch in µs.
		transmission_time.second /= 1.e+6; // Convert to s.
		return transmission_time;
	}


	std::pair<bool, double> Configuration::getCameraFrameRate(const std::string & camera_name) const {
		if(camera_name != "") {
			return getParameter<double>("/camera/" + camera_name + "/settings/fps", 24.);
		}
		else {
			return std::make_pair(false, 24.);
		}
	}


	std::pair<bool, bool> Configuration::getCameraAutoExposureEnabled(const std::string & camera_name) const {
		bool enabled = false;
		if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/enable_ae", enabled)) {
			return std::make_pair(true, enabled);
		}
		else if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/enable", enabled)) {
			return std::make_pair(true, enabled);
		}
		else {
			return std::make_pair(false, enabled);
		}
	}


	std::pair<bool, std::array<int, 4>> Configuration::getCameraAutoExposureRoi(const std::string & camera_name) const {
		std::array<int, 4> roi{-1, -1, -1, -1};
		if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/ae_roi/column", roi[0]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/ae_roi/row", roi[1]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/ae_roi/width", roi[2]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/ae_roi/height", roi[3])) {
			return std::make_pair(true, roi);
		}
		else if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/column", roi[0]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/row", roi[1]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/width", roi[2]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/height", roi[3])) {
			return std::make_pair(true, roi);
		}
		else {
			return std::make_pair(false, std::array<int, 4>{-1, -1, -1, -1});
		}
	}


	bool Configuration::setCameraAutoExposureRoi(const std::string & camera_name, const std::array<int, 4> & roi) {
		// Set width and height to 1 first to prevent an invalid ROI in case of race conditions.
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/width", 1);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/height", 1);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/column", roi[0]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/row", roi[1]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/width", roi[2]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/ae_roi/height", roi[3]);
		return true;
	}

	std::pair<bool, double> Configuration::getCameraAutoTargetBrightness(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/settings/auto/target_brightness", 0.50);
	}


	std::pair<bool, double> Configuration::getCameraAutoMinimumExposure(const std::string & camera_name) const {
		auto exposure_min = getParameter<double>("/camera/" + camera_name + "/settings/auto/exposure_us_min",
				1. / 1000. * 1.e+6); // Fetch in µs.
		exposure_min.second /= 1.e+6; // Convert to s.
		return exposure_min;
	}


	std::pair<bool, double> Configuration::getCameraAutoMaximumExposure(const std::string & camera_name) const {
		auto exposure_max = getParameter<double>("/camera/" + camera_name + "/settings/auto/exposure_us_max",
				1. / 25. * 1.e+6); // Fetch in µs.
		exposure_max.second /= 1.e+6; // Convert to s.
		return exposure_max;
	}


	std::pair<bool, double> Configuration::getCameraAutoMinimumGain(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/settings/auto/gain_db_min", 0.);
	}


	std::pair<bool, double> Configuration::getCameraAutoMaximumGain(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/settings/auto/gain_db_max", 30.);
	}


	std::pair<bool, bool> Configuration::getCameraAutoWhiteBalanceEnabled(const std::string & camera_name) const {
		bool enabled = false;
		if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/enable_awb", enabled)) {
			return std::make_pair(true, enabled);
		}
		else if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/enable", enabled)) {
			return std::make_pair(true, enabled);
		}
		else {
			return std::make_pair(false, enabled);
		}
	}


	std::pair<bool, std::array<int, 4>> Configuration::getCameraAutoWhiteBalanceRoi(const std::string & camera_name)
		const {
		std::array<int, 4> roi{-1, -1, -1, -1};
		if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/awb_roi/column", roi[0]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/awb_roi/row", roi[1]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/awb_roi/width", roi[2]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/awb_roi/height", roi[3])) {
			return std::make_pair(true, roi);
		}
		else if(node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/column", roi[0]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/row", roi[1]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/width", roi[2]) &
				node_handle_.getParam("/camera/" + camera_name + "/settings/auto/roi/height", roi[3])) {
			return std::make_pair(true, roi);
		}
		else {
			return std::make_pair(false, std::array<int, 4>{-1, -1, -1, -1});
		}
	}


	bool Configuration::setCameraAutoWhiteBalanceRoi(const std::string & camera_name, const std::array<int, 4> & roi) {
		// Set width and height to 1 first to prevent an invalid ROI in case of race conditions.
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/width", 1);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/height", 1);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/column", roi[0]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/row", roi[1]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/width", roi[2]);
		node_handle_.setParam("/camera/" + camera_name + "/settings/auto/awb_roi/height", roi[3]);
		return true;
	}


	std::pair<bool, double> Configuration::getCameraManualExposure(const std::string & camera_name) const {
		auto exposure = getParameter<double>("/camera/" + camera_name + "/settings/manual/exposure_us",
				1. / 25. * 1.e+6); // Fetch in µs.
		exposure.second /= 1.e+6; // Convert to s.
		return exposure;
	}


	std::pair<bool, double> Configuration::getCameraManualGain(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/settings/manual/gain_db", 0.);
	}


	std::pair<bool, std::array<double, 3>> Configuration::getCameraManualWhiteBalance(const std::string & camera_name)
	const {
		std::array<double, 3> wb{1.0, 1.0, 1.0};
		if(node_handle_.getParam("/camera/" + camera_name + "/settings/manual/balance_red", wb[0]) &
			node_handle_.getParam("/camera/" + camera_name + "/settings/manual/balance_green", wb[1]) &
			node_handle_.getParam("/camera/" + camera_name + "/settings/manual/balance_blue", wb[2])) {
			return std::make_pair(true, wb);
		}
		else {
			return std::make_pair(false, wb);
		}
	}


	std::pair<bool, double> Configuration::getCameraAcceptableMinimumIlluminance(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/limits/illuminance_lux_min",
				std::numeric_limits<double>::quiet_NaN());
	}


	std::pair<bool, double> Configuration::getCameraAcceptableMaximumIlluminance(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/limits/illuminance_lux_max",
				std::numeric_limits<double>::quiet_NaN());
	}


	std::pair<bool, double> Configuration::getCameraAcceptableMaximumGain(const std::string & camera_name) const {
		return getParameter<double>("/camera/" + camera_name + "/limits/gain_db_max",
				std::numeric_limits<double>::quiet_NaN());
	}


	std::pair<bool, std::string> Configuration::getArucoBoardDefinitionUrl() const {
		return getParameter<std::string>("/aruco/board_info_url", "file://${ROSHOME}/aruco/${TYPE}_${BOARD}.yaml");
	}


	std::pair<bool, std::string> Configuration::getArucoBoardName(const std::string & board_alias) const {
		return getParameter<std::string>("/aruco/" + board_alias + "/board_name", "");
	}


	std::pair<bool, double> Configuration::getArucoMarkerSize(const std::string & board_alias) const {
		return getParameter<double>("/aruco/" + board_alias + "/marker_size_m", 0.);
	}

}
