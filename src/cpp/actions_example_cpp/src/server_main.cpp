/**
 * Fibonacci computer action server main source file.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <actions_example_cpp/fib_server.hpp>

int main(int argc, char ** argv)
{
  std::cout << "Starting Fibonacci Computer action server..." << std::endl;
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<FibonacciComputer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
