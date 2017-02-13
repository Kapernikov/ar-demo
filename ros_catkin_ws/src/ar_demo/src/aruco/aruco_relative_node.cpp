//! Monitor relative position of an ArUco board relative to another ArUco board. 
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "aruco_relative.hpp"
// Boost headers.
#include <boost/program_options.hpp>


namespace program_options = boost::program_options;


int main(int argc, char ** argv) {

  // Initialise ROS node.
  ros::init(argc, argv, "aruco_relative_node");
  ros::NodeHandle node_handle;
	std::shared_ptr<ar_demo::Configuration> configuration{new ar_demo::Configuration(node_handle)};

	// Get command line parameters.
	program_options::options_description description("Recognised options");
	description.add_options()
		("help", "display this help and exit")
		("camera", program_options::value<std::string>(), "input camera name (required)")
		("list-cameras", "list configured cameras")
		("board-reference", program_options::value<std::string>(), "reference board name (required)")
		("board-moving", program_options::value<std::string>(), "moving board name (required)");
	program_options::variables_map variables;
	program_options::store(program_options::parse_command_line(argc, argv, description), variables);
	program_options::notify(variables);

	// Display help message.
	if(variables.count("help")) {
		std::cout << description << std::endl;
		return EXIT_FAILURE;
	}
	if(variables.count("list-cameras")) {
		std::cout << "Configured cameras:" << std::endl;
		for(const auto & camera: configuration->getConfiguredCameras()) {
			std::cout << camera << std::endl;
		}
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
	std::string reference_board_name{""};
	if(variables.count("board-reference")) {
		reference_board_name = variables["board-reference"].as<std::string>();
	}
	else {
		ROS_FATAL("No reference board specified. Use the --board-reference option to specify one.");
		return EXIT_FAILURE;
	}
	std::string moving_board_name{""};
	if(variables.count("board-moving")) {
		moving_board_name = variables["board-moving"].as<std::string>();
	}
	else {
		ROS_FATAL("No moving board specified. Use the --board-moving option to specify one.");
		return EXIT_FAILURE;
	}

	// Initialise ArUco relative locator.
	ar_demo::ArUcoRelative aruco_relative(configuration, node_handle, camera_name, reference_board_name, moving_board_name);
	if(!aruco_relative.isInitialised()) {
		ROS_FATAL("An error occured while initialising the ArUco relative locator.");
		return EXIT_FAILURE;
	}

	// Spin node.
	ros::spin();

  // Take everything down.
  return EXIT_SUCCESS;
}
