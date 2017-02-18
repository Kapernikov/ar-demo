//!ArucoFacade.
/*!
 *  \file
 *  Aruco
 *  http://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
 *  http://maztories.blogspot.be/2013/07/installing-aruco-augmented-reality.html
 *  Online Aruco generator
 *  http://terpconnect.umd.edu/~jwelsh12/enes100/markergen.html
 *
 *  \author Pieter Langsweirdt <pieter@kapernikov.com>
 */

// TODO probability = #markers found / #markers total.
// ==> nakijken wat threshold voor single marker is.
//#define DEBUG_ARUCO
#include "aruco_facade.hpp"
// Boost headers.
#include <boost/algorithm/string/predicate.hpp>
// OpenCV headers.
#ifdef DEBUG_ARUCO
#include <opencv2/opencv.hpp>
#endif
// ROS headers.
#include <ros/package.h>
#include <ar_demo/GetCameraInfo.h>

using namespace cv;
using namespace aruco;
using namespace std;

namespace ar_demo {

	ArucoFacade::ArucoFacade(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle):
		node_handle_{node_handle},
		configuration_{configuration},
		camera_name_(""),
		camera_parameters_(),
		aruco_results_(),
		dictionary_map_(),
		board_detector_map_() {
		;
	}

	ArucoFacade::~ArucoFacade() {
		;
	}

	bool ArucoFacade::initialise(const std::string & camera_name, const std::vector<std::string> & requested_markers) {
		bool success = true;
		reset();
		//
		// Get and set camera information.
		//
		camera_name_ = camera_name;
		ros::service::waitForService("/camera/" + camera_name_ + "/get_camera_info");
		ar_demo::GetCameraInfo camera_info_srv;
		auto camera_info = node_handle_.serviceClient<ar_demo::GetCameraInfo>(
				"/camera/" + camera_name_ + "/get_camera_info");
		if (!camera_info.call(camera_info_srv)) {
			ROS_ERROR("Unable to get camera information for camera '%s'", camera_name_.c_str());
			throw std::runtime_error("Unable to get camera information");
		}
		cv::Mat cameraMatrix(3, 3, CV_64FC1);
		for (int row = 0; row < 3; ++row) {
			for (int col = 0; col < 3; ++col) {
				cameraMatrix.at<double>(row, col) = camera_info_srv.response.camera_info.K[row * 3 + col];
			}
		}
		assert(camera_info_srv.response.camera_info.distortion_model == "plumb_bob");
		cv::Mat distortionCoefficients(1, 5, CV_64FC1); // Row vector. Column vector gives strange errors.
		for (int col = 0; col < 5; ++col) {
			distortionCoefficients.at<double>(0, col) = camera_info_srv.response.camera_info.D[col];
		}
		camera_parameters_.setParams(cameraMatrix, distortionCoefficients,
				cv::Size(camera_info_srv.response.camera_info.width, camera_info_srv.response.camera_info.height));
		aruco_results_.camera_name = camera_name_;
		//
		// Get marker board definitions.
		//
		for(const auto & marker: requested_markers) {
			std::string board_name{configuration_->getArucoBoardName(marker).second};
			double marker_size{configuration_->getArucoMarkerSize(marker).second};
			if(board_name != "") {
				try {
					// Marker definitions.
					ROS_DEBUG_STREAM("Initialising marker '" << board_name << "'.");
					std::string dictionaryfile = ResolveFilename(ResolveUrl("dict", board_name));
					// Dictionary.
					aruco::Dictionary dictionary;
					if (dictionary.fromFile(dictionaryfile) == false) {
						ROS_ERROR_STREAM("Error reading dictionary '" << dictionaryfile << "'.");
						throw;
					}
					if (dictionary.size() == 0) {
						ROS_ERROR_STREAM("Error parsing dictionary '" << dictionaryfile << "'.");
						throw;
					};
					// Configuration.
					std::string configurationfile = ResolveFilename(ResolveUrl("board", board_name));
					BoardConfiguration board_config;
					board_config.readFromFile(configurationfile);
					// Detector.
					BoardDetector board_detector;
					if (marker_size <= 0.) {
						throw std::runtime_error("BoardDetection requires non zero MarkerSize");
					}
					board_detector.setParams(board_config, camera_parameters_, marker_size);
					board_detector.setYPerperdicular(false);
					board_detector.getMarkerDetector().setThresholdParams(21, 7); // for blue-green markers, the window size has to be larger
					board_detector.getMarkerDetector().setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
					board_detector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::LINES);
					board_detector.getMarkerDetector().setWarpSize((dictionary[0].n() + 2) * 8);
					board_detector.getMarkerDetector().setMinMaxSize(0.005, 0.5);
					// Sets the threshold for reprjection test. Pixels that after estimating the camera location projects 'repj_err_thres' pixels farther from its original location are discarded as outliers. By default it is set to -1, meaning that not reprojection test is performed						
					// TODO literal constant:
					board_detector.set_repj_err_thres(5.0); // -1 = no reprojection test
					// Store in map.
					dictionary_map_[board_name] = dictionary;
					board_detector_map_[board_name] = board_detector;
				}
				catch(...) {
					ROS_ERROR("Error initialising requested AR board '%s'. Trying to continue without it.", marker.c_str());
					success = false;
				}
			}
			else {
				ROS_ERROR("Definition of requested AR board '%s' not found. Is the parameter file loaded? "
						"Trying to continue without it.", marker.c_str());
				success = false;
			}
		}
		// Return success or failure.
		return success;
	}

	void ArucoFacade::reset() {
		camera_name_ = "";
		camera_parameters_ = aruco::CameraParameters();
		dictionary_map_.clear();
		board_detector_map_.clear();
		aruco_results_ = ArucoResults();
	}

	/**
	 * On calling retrieve results, the data container is flushed.
	 */
	ArucoResults ArucoFacade::retrieveResult() {
		/*
		 * Prepare return result
		 */
		ArucoResults result(aruco_results_);
		/*
		 * Flush container
		 */
		aruco_results_ = ArucoResults();
		/*
		 * Return results
		 */
		return result;
	}

	bool ArucoFacade::hasValidResult() {
		return aruco_results_.valid;
	}

	void ArucoFacade::detect(Mat InImage, ros::Time frameTimestamp) {

		/*
		 * Init Frame container
		 */
		DetectedFrame detectedFrame;
		ROS_DEBUG("Aruco received a new frame.");
		try {
#ifdef DEBUG_ARUCO
			cv::namedWindow("in", 1);
			cv::Mat InImageGui;
			InImage.copyTo(InImageGui);
#endif
			for (std::map<std::string, aruco::BoardDetector>::iterator bdi = board_detector_map_.begin();
					bdi != board_detector_map_.end(); ++bdi) {
				string boardLabel = bdi->first; // string (key)
				ROS_DEBUG_STREAM("Searching for board '" << boardLabel << "'.");

				// Load dict
				Dictionary D = dictionary_map_.at(boardLabel);
				HighlyReliableMarkers::loadDictionary(D);

				BoardDetector & bd = bdi->second; // (value)

				// Detection of the board
				float probDetect = 0.;
				try {
					probDetect = bd.detect(InImage); // TODO
					ROS_DEBUG_STREAM("Board '" << boardLabel << "' detected with probability " << probDetect << ".");
				} catch (cv::Exception &e) {
					cout << "Exception: " << e.what() << endl;
					probDetect = 0.;
				}
#ifdef DEBUG_ARUCO
				// print marker borders
				for (unsigned int i = 0; i < bd.getDetectedBoard().size(); i++) {
					bd.getDetectedBoard()[i].draw(InImageGui, Scalar(0, 0, 255), 1);
				}
#endif
				// print board
				if (camera_parameters_.isValid()) {
					if (probDetect > 0.49) { // TODO Literal threshold  ==> make parameter
						/*
						 * Add Board to data object
						 */
						Board board(bd.getDetectedBoard());

						DetectedBoard detectedBoard;
						detectedBoard.name = boardLabel;
						detectedBoard.position.x = board.Tvec.at<float>(0, 0);
						detectedBoard.position.y = board.Tvec.at<float>(1, 0);
						detectedBoard.position.z = board.Tvec.at<float>(2, 0);
						detectedBoard.orientation.x = board.Rvec.at<float>(0, 0);
						detectedBoard.orientation.y = board.Rvec.at<float>(1, 0);
						detectedBoard.orientation.z = board.Rvec.at<float>(2, 0);
						detectedFrame.timestamp = frameTimestamp;
						detectedFrame.boards.push_back(detectedBoard);
						ROS_DEBUG_STREAM("Aruco is adding detectedBoard in boards collection.");

#ifdef DEBUG_ARUCO
						CvDrawingUtils::draw3dAxis(InImageGui, bd.getDetectedBoard(), camera_parameters_);
#endif
					}
					else {
						ROS_INFO_STREAM("Probability in Aruco marker detection is too low.");
						// TODO This code was used to debug perform.cpp. May not be active while cambot_calib-bing
//						ROS_ERROR("ArUcu Facade is keeping empty boards for not found markers !!!");
//						DetectedBoard detectedBoard;
//						detectedBoard.name = boardLabel;
//						detectedFrame.boards.push_back(detectedBoard);
					}
				}else{
					ROS_WARN_STREAM("Aruco: camera parameters not valid!.");
				}
			}

			// Only register those frames where we could find 2 boards
			if (detectedFrame.boards.size() == 2) {
				ROS_DEBUG_STREAM("Aruco is adding valid detectedFrame in collection.");
				aruco_results_.frames.push_back(detectedFrame);
				// TODO: Validation does not count detected markers.
//				if (detectedFrame.boards.size() == board_detector_map_.size())
				aruco_results_.valid = true;
			}
			else {
				ROS_INFO_STREAM("No boards found in detected frame.");
			}


#ifdef DEBUG_ARUCO
			int reduction = 2;
			cv::resize(InImageGui, InImageGui, cv::Size(InImage.cols / reduction, InImage.rows / reduction), 0, 0,
					cv::INTER_AREA);
			cv::imshow("in", InImageGui);

			if (false) {
				cv::Mat thresGui;
				cv::resize(MDetector.getThresholdedImage(), thresGui,
						cv::Size(InImage.cols / reduction, InImage.rows / reduction), 0, 0, cv::INTER_AREA);
				//show also the internal image resulting from the threshold operation
				cv::imshow("thes", thresGui);
			}

			cv::waitKey(20);  //wait for key to be pressed
#endif
		} catch (std::exception &ex)

		{
			ROS_ERROR_STREAM("ArUco facade exception: " <<  ex.what());
		}

		return;
	}

	/**
	 * Exists for backward compatibility towards cambot_calib
	 */
	void ArucoFacade::detect(Mat InImage) {
		detect(InImage, ros::Time::now());
	}

	//! Get board info url type.
	/*!
	 *  Based on the equivalent ROS CameraInfoManager function.
	 */
	ArucoFacade::UrlType ArucoFacade::ParseUrl(const std::string & resolvedUrl) {
		if (boost::iequals(resolvedUrl.substr(0, 8), "file:///")) {
			return URL_FILE;
		}
		if (boost::iequals(resolvedUrl.substr(0, 10), "package://")) {
			// look for a '/' following the package name, make sure it is
			// there, the name is not empty, and something follows it
			size_t rest = resolvedUrl.find('/', 10);
			if (rest < resolvedUrl.length() - 1 && rest > 10)
				return URL_PACKAGE;
		}
		return URL_INVALID;
	}

	//! Resolve board info url.
	/*!
	 *  Based on the equivalent ROS CameraInfoManager function.
	 */
	std::string ArucoFacade::ResolveUrl(const std::string & type, const std::string & board) {
		const std::string board_info_url{configuration_->getArucoBoardDefinitionUrl().second};
		std::string resolved;
		size_t rest = 0;
		while (true) {
			// find the next '$' in the URL string
			size_t dollar = board_info_url.find('$', rest);
			if (dollar >= board_info_url.length()) {
				// no more variables left in the URL
				resolved += board_info_url.substr(rest);
				break;
			}
			// copy characters up to the next '$'
			resolved += board_info_url.substr(rest, dollar - rest);
			if (board_info_url.substr(dollar + 1, 1) != "{") {
				// no '{' follows, so keep the '$'
				resolved += "$";
			} else if (board_info_url.substr(dollar + 1, 6) == "{TYPE}") {
				// substitute first coordinate system
				resolved += type;
				dollar += 6;
			} else if (board_info_url.substr(dollar + 1, 7) == "{BOARD}") {
				// substitute second coordinate system
				resolved += board;
				dollar += 7;
			} else if (board_info_url.substr(dollar + 1, 10) == "{ROS_HOME}") {
				// substitute $ROS_HOME
				std::string ros_home;
				char *ros_home_env;
				if ((ros_home_env = getenv("ROS_HOME"))) {
					// use environment variable
					ros_home = ros_home_env;
				} else if ((ros_home_env = getenv("HOME"))) {
					// use "$HOME/.ros"
					ros_home = ros_home_env;
					ros_home += "/.ros";
				}
				resolved += ros_home;
				dollar += 10;
			} else {
				// not a valid substitution variable
				ROS_ERROR_STREAM(
						"Camera <-> robot calibration: invalid URL substitution " "(not resolved): " << board_info_url);
				resolved += "$";            // keep the bogus '$'
			}
			// look for next '$'
			rest = dollar + 1;
		}
		return resolved;
	}

	std::string ArucoFacade::ResolveFilename(const std::string & resolvedUrl) {
		switch (ParseUrl(resolvedUrl)) {
			case URL_FILE: {
				return resolvedUrl.substr(7);
				break;
			}
			case URL_PACKAGE: {
				// Scan URL from after "package://" until next '/' and extract
				// package name.  The parseURL() already checked that it's present.
				size_t prefix_len = std::string("package://").length();
				size_t rest = resolvedUrl.find('/', prefix_len);
				std::string package(resolvedUrl.substr(prefix_len, rest - prefix_len));
				// Look up the ROS package path name.
				std::string pkgPath(ros::package::getPath(package));
				if (pkgPath.empty()) {
					// package not found.
					ROS_WARN_STREAM("unknown package: " << package << " (ignored)");
					return std::string();
				} else {
					// Construct file name from package location and remainder of URL.
					return pkgPath + resolvedUrl.substr(rest);
				}
				break;
			}
			default: {
				return std::string();
				break;
			}
		}
	}

}
