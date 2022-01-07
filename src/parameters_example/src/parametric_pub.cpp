/**
 * Node parameters example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * December 3, 2021
 */

#include <iostream>

#include "../include/parameters_example/parametric_node.hpp"

//! No special setup is required here, you only have to start this with:
//! ros2 run parameters_examples parametric_pub --ros-args -p number:=...
//! or load the parameter file with:
//! ros2 run parameters_example parametric_pub --ros-args --params-file PATH_TO_FILE
//! Test with:
//! ros2 param list [...]
//! ros2 param describe [...]
//! ros2 param get <node_name> <parameter_name>
//! ros2 param set <node_name> <parameter_name> <value>
//! ros2 param dump <node_name>
//! ros2 param load <node_name> <parameter_file>
//! Note how there's autocompletion for everything!
//! For more APIs (some will be shown later) see documentation of class rclcpp::Node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); //! Parameter specification arguments will be parsed here
  auto node_ptr = std::make_shared<ParametricPub>();
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

//! Note that all the functionalities that CLI tools offer can also be achieved
//! at runtime by other nodes, which can interact with each other by querying and
//! setting parameters
//! Just run "ros2 service list -t" after this application is up to notice that
//! every node offers specific services to interact with parameters
//! You can use those services via CLI as well as from another node's code
//! Alternatively, there's the native "AsyncParametersClient" object that
//! offers APIs to do more or less that

