//! Camera display node - camera display node.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_display.hpp"
// Standard C++ headers.
#include <iostream>
#include <string>
// Boost headers.
#include <boost/program_options.hpp>
// ROS headers.
#include <ros/ros.h>
#include "../configuration/configuration.hpp"


namespace program_options = boost::program_options;


//! Main function, initialising the camera stream display.
/*!
 *  \param argc Argument count.
 *  \param argv Argument values.
 */
int main(int argc, char** argv) {
	// Initialise ROS node.
	ros::init(argc, argv, "camera_display_node");
	ros::NodeHandle node_handle;
	std::shared_ptr<ar_demo::Configuration> configuration{new ar_demo::Configuration(node_handle)};

	// Get command line parameters.
	program_options::options_description description("Recognised options");
	description.add_options()
		("help", "display this help and exit")
		("camera", program_options::value<std::string>(), "input camera name (required)")
    ("downsample", program_options::value<double>(), "image reduction factor (optional)")
		("ae-roi", "show auto exposure ROI")
		("awb-roi", "show auto white balance ROI");
	program_options::variables_map variables;
	program_options::store(program_options::parse_command_line(argc, argv, description), variables);
	program_options::notify(variables);

	// Display help message.
	if (variables.count("help")) {
		std::cout << description << std::endl;
		return EXIT_FAILURE;
	}

	// Get parameters from command line.
	std::string camera_name{""};
	if(variables.count("camera")) {
		camera_name = variables["camera"].as<std::string>();
	}
	else {
		ROS_FATAL("No input camera specified. Use the --camera option to specify one.");
		return EXIT_FAILURE;
	}
  double image_reduction_factor{1.0};
  if(variables.count("downsample")) {
    image_reduction_factor = variables["downsample"].as<double>();
    ROS_INFO("Downsampling input images by a factor %.2f.", image_reduction_factor);
  }
  if(image_reduction_factor < 1.0) {
    ROS_FATAL("Incorrect image reduction factor specified.");
    return EXIT_FAILURE;
  }
	bool show_ae_roi = (variables.count("ae-roi") > 0? true: false);
	bool show_awb_roi = (variables.count("awb-roi") > 0? true: false);

	// Initialise camera display.
	ar_demo::CameraDisplay camera_display(configuration, node_handle, camera_name, image_reduction_factor,
			show_ae_roi, show_awb_roi);

	// And GO!
	ros::spin();

	// Take everything down.
	return EXIT_SUCCESS;
}
