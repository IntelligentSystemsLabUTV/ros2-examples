/**
 * Simple listener node, to be used with demo_nodes_-/talker.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 19, 2022
 */

#ifndef LISTENER_NODE_HPP
#define LISTENER_NODE_HPP

#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std_msgs::msg;

/**
 * String message listener node.
 */
class Listener : public rclcpp::Node
{
public:
  Listener(rclcpp::NodeOptions & opts)
  : Node("listener", opts)
  {
    sub_ = this->create_subscription<String>(
      "/chatter",
      rclcpp::QoS(10),
      [this](const String::SharedPtr msg) -> void {
        RCLCPP_INFO_STREAM(this->get_logger(), msg->data);
      }
    );

    //! Note that some basic rclcpp operations now require an explicit pointer
    //! to the current context (like below or sleep_for)
    if (!rclcpp::ok(this->get_node_options().context())) {
      throw std::runtime_error("Listener::Listener: invalid context");
    }

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

private:
  rclcpp::Subscription<String>::SharedPtr sub_;
};

#endif
