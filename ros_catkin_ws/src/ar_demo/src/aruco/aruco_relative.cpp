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
		moving_board_name_{moving_board_name},
		zmq_context_{1},
		zmq_publisher_{zmq_context_, ZMQ_PUB} {
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
		ROS_DEBUG_STREAM("ArUcoRelative is using boards: " << reference_board_name_ << " & " << moving_board_name_);

		std::vector<std::string> boards{reference_board_name, moving_board_name};
		aruco_facade_.initialise(camera_name_, boards);
		// Initialise 0MQ publisher.
		zmq_publisher_.bind("tcp://*:5556");
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
			ROS_DEBUG("ArUcoRelative spotted valid Aruco result.");
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
					ROS_DEBUG_STREAM("Board '" << *board << "'.");
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
					// This is the homogeneous 'moving board' -> (camera) -> 'reference board' transformation matrix:
					// | R11 R12 R13  T1 |
					// | R21 R22 R23  T2 |
					// | R31 R32 R33  T3 |
					// |   0   0   0   1 |
					// where R11...R13, R21...R23, R31...R33 is the rotation matrix and T1...T3 is the translation matrix.
					//
					// Multiply with a vector in homogeneous coordinates (X, Y, Z, 1) to perform rotation and translation in a
					// single multiplication.
					ublas::matrix<double> transformation_matrix = ublas::prod(transform_camera_reference, transform_moving_camera);

					// Calculate translation and rotation parameters.
					double tx = transformation_matrix(0, 3) / transformation_matrix(3, 3);
					double ty = transformation_matrix(1, 3) / transformation_matrix(3, 3);
					double tz = transformation_matrix(2, 3) / transformation_matrix(3, 3);
					double sy = sqrt(transformation_matrix(0,0) * transformation_matrix(0,0) +  transformation_matrix(1,0) * transformation_matrix(1,0));
					bool singular = (sy < 1e-6? true: false);
					double rx, ry, rz;
					if(!singular) {
						rx = atan2(transformation_matrix(2,1) , transformation_matrix(2,2));
						ry = atan2(-transformation_matrix(2,0), sy);
						rz = atan2(transformation_matrix(1,0), transformation_matrix(0,0));
					}
					else {
						rx = atan2(-transformation_matrix(1,2), transformation_matrix(1,1));
						ry = atan2(-transformation_matrix(2,0), sy);
						rz = 0;
					}
					// Send message using 0MQ.
					std::ostringstream msgJson;
					// Convert to Three.js scale
					// Conversie is verhuisd naar main.js					
					//tx = (-tx + 0.35) / 1000 * 2;
					//ty = (ty + 0.35) / 1000 * 2;
					//tz = (tz + 0.022) ;
					//rx = -rx ;
					//ry = rz ;
					//rz = ry ;
					// Pack Json
					msgJson << "{\"x\": " << tx << ", \"y\": " << ty << ", \"z\": " << tz << ", "
						<< "\"rx\": " << rx << ", \"ry\": " << ry << ", \"rz\": " << rz << "}" << std::endl;
					zmq::message_t message(msgJson.str().size());
					memcpy(message.data (), (msgJson.str().c_str()), (msgJson.str().size()));
					zmq_publisher_.send(message);
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
