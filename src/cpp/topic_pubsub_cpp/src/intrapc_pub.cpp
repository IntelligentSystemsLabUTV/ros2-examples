/**
 * Intra-process communication compatibility test.
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
  auto talker = std::make_shared<IPCTalker>("ipc_single_talker");
  rclcpp::spin(talker);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
