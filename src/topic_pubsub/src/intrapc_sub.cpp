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
  auto listener = std::make_shared<IPCListener>("ipc_single_listener");
  rclcpp::spin(listener);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
