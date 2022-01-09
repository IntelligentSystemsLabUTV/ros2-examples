/**
 * Dummy publisher/server.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 9, 2022
 */

#ifndef PUBSRV_HPP
#define PUBSRV_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <example_interfaces/srv/trigger.hpp>

using namespace std_msgs::msg;
using namespace example_interfaces::srv;

/**
 * Dummy publisher/server node.
 */
class DummyPubSrv : public rclcpp::Node
{
public:
  DummyPubSrv();

private:
  rclcpp::Publisher<Bool>::SharedPtr pub_;
  rclcpp::Service<Trigger>::SharedPtr server_;
  void server_clbk(
    Trigger::Request::SharedPtr req,
    Trigger::Response::SharedPtr resp);
};

#endif