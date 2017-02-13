//! Monitor relative position of an ArUco board relative to another ArUco board. 
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "aruco_relative.hpp"
// Boost headers.
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/lu.hpp>
// OpenCV headers.
#include <opencv2/opencv.hpp>
// ROS headers.
#include <cv_bridge/cv_bridge.h>
#include <ar_demo/GetCameraInfo.h>


namespace ublas = boost::numeric::ublas;


namespace ar_demo {

	ArUcoRelative::ArUcoRelative(std::shared_ptr<const Configuration> configuration, ros::NodeHandle node_handle,
			const std::string & camera_name, const std::string & reference_board_name, const std::string & moving_board_name):
		is_initialised_{false},
		configuration_{configuration},
		node_handle_{node_handle},
		image_transport_{node_handle_},
		subscription_camera_stream_{image_transport_.subscribe("/camera/" + camera_name + "/stream", 1,
				&ArUcoRelative::imageCallback, this)},
		aruco_facade_{configuration, node_handle},
		camera_name_{camera_name},
		reference_board_name_{reference_board_name},
		moving_board_name_{moving_board_name} {
		// Get camera parameters from camera.
		ros::service::waitForService("/camera/" + camera_name_ + "/get_camera_info");
		GetCameraInfo camera_info_srv;
		auto camera_info = node_handle_.serviceClient<GetCameraInfo>("/camera/" + camera_name_ + "/get_camera_info");
		if(camera_info.call(camera_info_srv)) {
			camera_info_ = camera_info_srv.response.camera_info;
		}
		else {
			ROS_FATAL("Unable to get camera information for camera '%s'", camera_name_.c_str());
			throw std::runtime_error("Unable to get camera information");
		}
		// Initialise ArUco facade.
		std::vector<std::string> boards{reference_board_name, moving_board_name};
		aruco_facade_.initialise(camera_name_, boards);
		// Initialisation done.
		is_initialised_ = true;
	}

	ArUcoRelative::~ArUcoRelative() {
		;
	}

	bool ArUcoRelative::isInitialised() {
		return is_initialised_;
	}

	void ArUcoRelative::imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
		// Get OpenCV image from cv_bridge.
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat image = cv_ptr->image;
		// Detect ArUco boards.
		aruco_facade_.detect(image, imageMsg->header.stamp);
		if(aruco_facade_.hasValidResult()) {
			ArucoResults aruco_results{aruco_facade_.retrieveResult()};
			std::vector<DetectedFrame> frames = aruco_results.frames;
			// Lambda function to convert the board's t and r vectors to a transformation matrix.
			auto convert_board_to_transformation_matrix = [] (DetectedBoard & board) {
				ublas::matrix<double> transformation_matrix {ublas::identity_matrix<double> {4}};
				auto board_rodrigues = cv::Mat_<double>(3, 1);
				board_rodrigues.at<double>(0, 0) = board.orientation.x;
				board_rodrigues.at<double>(1, 0) = board.orientation.y;
				board_rodrigues.at<double>(2, 0) = board.orientation.z;
				auto board_rotation = cv::Mat_<double>(3, 3);
				cv::Rodrigues(board_rodrigues, board_rotation);
				for(auto row = 0; row < 3; ++ row) {
					for(auto col = 0; col < 3; ++ col) {
						transformation_matrix(row, col) = board_rotation.at<double>(row, col);
					}
				}
				transformation_matrix(0, 3) = board.position.x;
				transformation_matrix(1, 3) = board.position.y;
				transformation_matrix(2, 3) = board.position.z;
				return transformation_matrix;
			};
			// Calculate position of boards in the reference CS (based on the reference board).
			for(auto frame = frames.begin(); frame != frames.end(); ++frame) {
				bool transform_reference_camera_valid = false;
				bool transform_moving_camera_valid = false;
				ublas::matrix<double> transform_reference_camera;
				ublas::matrix<double> transform_camera_reference { 4, 4 };
				ublas::matrix<double> transform_moving_camera;
				for(auto board = frame->boards.begin(); board != frame->boards.end(); ++board) {
					ROS_DEBUG_STREAM("Board name '" << *board << "'.");
					if(board->name == reference_board_name_) {
						transform_reference_camera = convert_board_to_transformation_matrix(*board);
						InvertMatrix(transform_reference_camera, transform_camera_reference);
						transform_reference_camera_valid = true;
					}
					else if(board->name == moving_board_name_) {
						transform_moving_camera = convert_board_to_transformation_matrix(*board);
						transform_moving_camera_valid = true;
					}
					else {
						throw std::logic_error("Unexpected initialised ArUco board found.");
					}
				}
				if(transform_reference_camera_valid && transform_moving_camera_valid) {
					ublas::vector<double> reference { 4 };
					reference <<= 0.0, 0.0, 0.0, 1.0;
					ublas::vector<double> reference_camera = ublas::prod(transform_moving_camera, reference);
					ublas::vector<double> reference_reference = ublas::prod(transform_camera_reference, reference_camera);
					geometry_msgs::Point moving_location;
					moving_location.x = reference_reference(0) / reference_reference(3);
					moving_location.y = reference_reference(1) / reference_reference(3);
					moving_location.z = reference_reference(2) / reference_reference(3);
					ROS_INFO("Position of %s marker in %s CS: (%.3f, %.3f, %.3f)",
							moving_board_name_.c_str(), reference_board_name_.c_str(),
							reference_reference(0) / reference_reference(3),
							reference_reference(1) / reference_reference(3),
							reference_reference(2) / reference_reference(3));
					// TODO: send message!
				}
				else {
					ROS_WARN("ArUcoRelative did not find all boards found; cannot calculate relative positions.");
				}
			}
		}
		else {
			ROS_DEBUG("ArUcoRelative received an image but didn't recognise any known boards on it.");
		}
	}

	template<typename T> bool ArUcoRelative::InvertMatrix(const ublas::matrix<T> & input, ublas::matrix<T> & output) {
		// From <https://savingyoutime.wordpress.com/2009/09/21/c-matrix-inversion-boostublas/>
		ublas::matrix<T> A(input);
		ublas::permutation_matrix<std::size_t> pm(A.size1());
		int res = lu_factorize(A, pm);
		if (res != 0)
			return false;
		output.assign(ublas::identity_matrix<T>(A.size1()));
		lu_substitute(A, pm, output);
		return true;
	}

}
