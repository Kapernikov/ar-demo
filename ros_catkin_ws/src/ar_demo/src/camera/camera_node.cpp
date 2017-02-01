//! Unified camera node - camera node.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera.hpp"
// Standard C++ headers.
#include <memory>
#include <string>
// Boost headers.
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
// ROS headers.
#include <ros/ros.h>
#include "../configuration/configuration.hpp"


namespace program_options = boost::program_options;


int main(int argc, char ** argv) {

  // Initialise ROS node.
  ros::init(argc, argv, "unified_camera_node");
  ros::NodeHandle node_handle;
	std::shared_ptr<ar_demo::Configuration> configuration{new ar_demo::Configuration(node_handle)};

	// Get command line parameters.
	program_options::options_description description("Recognised options");
	description.add_options()
		("help", "display this help and exit")
		("camera", program_options::value<std::string>(), "input camera name (required)")
		("list-cameras", "list configured cameras");
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

  // Initialise camera.
  ar_demo::Camera camera(configuration, ros::NodeHandle("/camera/" + camera_name), camera_name);
  if(!camera.isInitialised()) {
    ROS_FATAL("An error occured while initialising camera '%s'.", camera_name.c_str());
    return EXIT_FAILURE;
  }

  // Spin camera node.
  const bool multithreaded{false};
  if(multithreaded) {
    boost::thread thread_camera{&ar_demo::Camera::spin, &camera};
    ros::spin();
  }
  else {
    camera.spin();
  }

  // Take everything down.
  return EXIT_SUCCESS;
}
