/**
 * Intra-process communication facilities test.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 22, 2022
 */

#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include <intrapc_test/intrapc_chatters.hpp>

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto talker = std::make_shared<IPCTalker>();
  auto listener = std::make_shared<IPCListener>();
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
